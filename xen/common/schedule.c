/****************************************************************************
 * (C) 2002-2003 - Rolf Neugebauer - Intel Research Cambridge
 * (C) 2002-2003 University of Cambridge
 * (C) 2004      - Mark Williamson - Intel Research Cambridge
 ****************************************************************************
 *
 *        File: common/schedule.c
 *      Author: Rolf Neugebauer & Keir Fraser
 *              Updated for generic API by Mark Williamson
 *
 * Description: Generic CPU scheduling code
 *              implements support functionality for the Xen scheduler API.
 *
 */

#ifndef COMPAT
#include <xen/init.h>
#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/domain.h>
#include <xen/delay.h>
#include <xen/event.h>
#include <xen/time.h>
#include <xen/timer.h>
#include <xen/perfc.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <xen/trace.h>
#include <xen/mm.h>
#include <xen/err.h>
#include <xen/guest_access.h>
#include <xen/hypercall.h>
#include <xen/multicall.h>
#include <xen/cpu.h>
#include <xen/preempt.h>
#include <xen/event.h>
#include <public/sched.h>
#include <xsm/xsm.h>
#include <xen/err.h>

/* opt_sched: scheduler - default to configured value */
static char __initdata opt_sched[10] = CONFIG_SCHED_DEFAULT;
string_param("sched", opt_sched);

/* if sched_smt_power_savings is set,
 * scheduler will give preferrence to partially idle package compared to
 * the full idle package, when picking pCPU to schedule vCPU.
 */
bool_t sched_smt_power_savings = 0;
boolean_param("sched_smt_power_savings", sched_smt_power_savings);

/* Default scheduling rate limit: 1ms
 * The behavior when sched_ratelimit_us is greater than sched_credit_tslice_ms is undefined
 * */
int sched_ratelimit_us = SCHED_DEFAULT_RATELIMIT_US;
integer_param("sched_ratelimit_us", sched_ratelimit_us);

/* Number of vcpus per struct sched_item. */
static unsigned int sched_granularity = 1;
const cpumask_t *sched_res_mask = &cpumask_all;

/* Various timer handlers. */
static void s_timer_fn(void *unused);
static void vcpu_periodic_timer_fn(void *data);
static void vcpu_singleshot_timer_fn(void *data);
static void poll_timer_fn(void *data);

/* This is global for now so that private implementations can reach it */
DEFINE_PER_CPU(struct scheduler *, scheduler);
DEFINE_PER_CPU(struct sched_resource *, sched_res);

/* Scratch space for cpumasks. */
DEFINE_PER_CPU(cpumask_t, cpumask_scratch);

extern const struct scheduler *__start_schedulers_array[], *__end_schedulers_array[];
#define NUM_SCHEDULERS (__end_schedulers_array - __start_schedulers_array)
#define schedulers __start_schedulers_array

static struct scheduler __read_mostly ops;

static inline struct scheduler *dom_scheduler(const struct domain *d)
{
    if ( likely(d->cpupool != NULL) )
        return d->cpupool->sched;

    /*
     * If d->cpupool is NULL, this is the idle domain. This is special
     * because the idle domain does not really belong to any cpupool, and,
     * hence, does not really have a scheduler.
     *
     * This is (should be!) only called like this for allocating the idle
     * vCPUs for the first time, during boot, in which case what we want
     * is the default scheduler that has been, choosen at boot.
     */
    ASSERT(is_idle_domain(d));
    return &ops;
}

static inline struct scheduler *vcpu_scheduler(const struct vcpu *v)
{
    struct domain *d = v->domain;

    if ( likely(d->cpupool != NULL) )
        return d->cpupool->sched;

    /*
     * If d->cpupool is NULL, this is a vCPU of the idle domain. And this
     * case is special because the idle domain does not really belong to
     * a cpupool and, hence, doesn't really have a scheduler). In fact, its
     * vCPUs (may) run on pCPUs which are in different pools, with different
     * schedulers.
     *
     * What we want, in this case, is the scheduler of the pCPU where this
     * particular idle vCPU is running. And, since v->processor never changes
     * for idle vCPUs, it is safe to use it, with no locks, to figure that out.
     */
    ASSERT(is_idle_domain(d));
    return per_cpu(scheduler, v->processor);
}
#define VCPU2ONLINE(_v) cpupool_domain_cpumask((_v)->domain)

static inline void trace_runstate_change(struct vcpu *v, int new_state)
{
    struct { uint32_t vcpu:16, domain:16; } d;
    uint32_t event;

    if ( likely(!tb_init_done) )
        return;

    d.vcpu = v->vcpu_id;
    d.domain = v->domain->domain_id;

    event = TRC_SCHED_RUNSTATE_CHANGE;
    event |= ( v->runstate.state & 0x3 ) << 8;
    event |= ( new_state & 0x3 ) << 4;

    __trace_var(event, 1/*tsc*/, sizeof(d), &d);
}

static inline void trace_continue_running(struct vcpu *v)
{
    struct { uint32_t vcpu:16, domain:16; } d;

    if ( likely(!tb_init_done) )
        return;

    d.vcpu = v->vcpu_id;
    d.domain = v->domain->domain_id;

    __trace_var(TRC_SCHED_CONTINUE_RUNNING, 1/*tsc*/, sizeof(d), &d);
}

static inline void vcpu_urgent_count_update(struct vcpu *v)
{
    if ( is_idle_vcpu(v) )
        return;

    if ( unlikely(v->is_urgent) )
    {
        if ( !(v->pause_flags & VPF_blocked) ||
             !test_bit(v->vcpu_id, v->domain->poll_mask) )
        {
            v->is_urgent = 0;
            atomic_dec(&per_cpu(sched_res, v->processor)->urgent_count);
        }
    }
    else
    {
        if ( unlikely(v->pause_flags & VPF_blocked) &&
             unlikely(test_bit(v->vcpu_id, v->domain->poll_mask)) )
        {
            v->is_urgent = 1;
            atomic_inc(&per_cpu(sched_res, v->processor)->urgent_count);
        }
    }
}

static inline void vcpu_runstate_change(
    struct vcpu *v, int new_state, s_time_t new_entry_time)
{
    s_time_t delta;
    struct sched_item *item = v->sched_item;

    ASSERT(spin_is_locked(per_cpu(sched_res, v->processor)->schedule_lock));
    if ( v->runstate.state == new_state )
        return;

    vcpu_urgent_count_update(v);

    trace_runstate_change(v, new_state);

    item->runstate_cnt[v->runstate.state]--;
    item->runstate_cnt[new_state]++;

    delta = new_entry_time - v->runstate.state_entry_time;
    if ( delta > 0 )
    {
        v->runstate.time[v->runstate.state] += delta;
        v->runstate.state_entry_time = new_entry_time;
    }

    v->runstate.state = new_state;
}

static inline void sched_item_runstate_change(struct sched_item *item,
    bool running, s_time_t new_entry_time)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        if ( running )
            vcpu_runstate_change(v, v->new_state, new_entry_time);
        else
            vcpu_runstate_change(v,
                ((v->pause_flags & VPF_blocked) ? RUNSTATE_blocked :
                 (vcpu_runnable(v) ? RUNSTATE_runnable : RUNSTATE_offline)),
                new_entry_time);
}

void vcpu_runstate_get(struct vcpu *v, struct vcpu_runstate_info *runstate)
{
    spinlock_t *lock = likely(v == current)
                       ? NULL : item_schedule_lock_irq(v->sched_item);
    s_time_t delta;

    memcpy(runstate, &v->runstate, sizeof(*runstate));
    delta = NOW() - runstate->state_entry_time;
    if ( delta > 0 )
        runstate->time[runstate->state] += delta;

    if ( unlikely(lock != NULL) )
        item_schedule_unlock_irq(lock, v->sched_item);
}

uint64_t get_cpu_idle_time(unsigned int cpu)
{
    struct vcpu_runstate_info state = { 0 };
    struct vcpu *v = idle_vcpu[cpu];

    if ( cpu_online(cpu) && v )
        vcpu_runstate_get(v, &state);

    return state.time[RUNSTATE_running];
}

/*
 * If locks are different, take the one with the lower address first.
 * This avoids dead- or live-locks when this code is running on both
 * cpus at the same time.
 */
static void sched_spin_lock_double(spinlock_t *lock1, spinlock_t *lock2,
                                   unsigned long *flags)
{
    if ( lock1 == lock2 )
    {
        spin_lock_irqsave(lock1, *flags);
    }
    else if ( lock1 < lock2 )
    {
        spin_lock_irqsave(lock1, *flags);
        spin_lock(lock2);
    }
    else
    {
        spin_lock_irqsave(lock2, *flags);
        spin_lock(lock1);
    }
}

static void sched_spin_unlock_double(spinlock_t *lock1, spinlock_t *lock2,
                                     unsigned long flags)
{
    if ( lock1 != lock2 )
        spin_unlock(lock2);
    spin_unlock_irqrestore(lock1, flags);
}

static void sched_free_item(struct sched_item *item, struct vcpu *v)
{
    struct sched_item *prev_item;
    struct domain *d = item->domain;
    struct vcpu *vitem;
    unsigned int cnt = 0;

    /* Don't count to be released vcpu, might be not in vcpu list yet. */
    for_each_sched_item_vcpu ( item, vitem )
        if ( vitem != v )
            cnt++;

    v->sched_item = NULL;

    if ( cnt )
        return;

    if ( item->vcpu == v )
        item->vcpu = v->next_in_list;

    if ( d->sched_item_list == item )
        d->sched_item_list = item->next_in_list;
    else
    {
        for_each_sched_item(d, prev_item)
        {
            if ( prev_item->next_in_list == item )
            {
                prev_item->next_in_list = item->next_in_list;
                break;
            }
        }
    }

    free_cpumask_var(item->cpu_hard_affinity);
    free_cpumask_var(item->cpu_hard_affinity_tmp);
    free_cpumask_var(item->cpu_hard_affinity_saved);
    free_cpumask_var(item->cpu_soft_affinity);

    xfree(item);
}

static void sched_item_add_vcpu(struct sched_item *item, struct vcpu *v)
{
    v->sched_item = item;
    if ( !item->vcpu || item->vcpu->vcpu_id > v->vcpu_id )
    {
        item->vcpu = v;
        item->item_id = v->vcpu_id;
    }
}

static struct sched_item *sched_alloc_item(struct vcpu *v)
{
    struct sched_item *item, **prev_item;
    struct domain *d = v->domain;

    for_each_sched_item ( d, item )
        if ( item->vcpu->vcpu_id / sched_granularity ==
             v->vcpu_id / sched_granularity )
            break;

    if ( item )
    {
        sched_item_add_vcpu(item, v);
        return item;
    }

    if ( (item = xzalloc(struct sched_item)) == NULL )
        return NULL;

    sched_item_add_vcpu(item, v);
    item->domain = d;

    for ( prev_item = &d->sched_item_list; *prev_item;
          prev_item = &(*prev_item)->next_in_list )
        if ( (*prev_item)->next_in_list &&
             (*prev_item)->next_in_list->item_id > item->item_id )
            break;

    item->next_in_list = *prev_item;
    *prev_item = item;

    if ( !zalloc_cpumask_var(&item->cpu_hard_affinity) ||
         !zalloc_cpumask_var(&item->cpu_hard_affinity_tmp) ||
         !zalloc_cpumask_var(&item->cpu_hard_affinity_saved) ||
         !zalloc_cpumask_var(&item->cpu_soft_affinity) )
        goto fail;

    return item;

 fail:
    sched_free_item(item, v);
    return NULL;
}

static unsigned int sched_select_initial_cpu(struct vcpu *v)
{
    struct domain *d = v->domain;
    nodeid_t node;
    cpumask_t cpus;

    cpumask_clear(&cpus);
    for_each_node_mask ( node, d->node_affinity )
        cpumask_or(&cpus, &cpus, &node_to_cpumask(node));
    cpumask_and(&cpus, &cpus, d->cpupool->cpu_valid);
    if ( cpumask_empty(&cpus) )
        cpumask_copy(&cpus, d->cpupool->cpu_valid);

    if ( v->vcpu_id == 0 )
        return cpumask_first(&cpus);

    /* We can rely on previous vcpu being available. */
    ASSERT(!is_idle_domain(d));

    return cpumask_cycle(d->vcpu[v->vcpu_id - 1]->processor, &cpus);
}

int sched_init_vcpu(struct vcpu *v)
{
    struct domain *d = v->domain;
    struct sched_item *item;
    unsigned int processor;

    if ( (item = sched_alloc_item(v)) == NULL )
        return 1;

    item->runstate_cnt[v->runstate.state]++;

    if ( is_idle_domain(d) )
        processor = v->vcpu_id;
    else
        processor = sched_select_initial_cpu(v);

    /* Initialise the per-vcpu timers. */
    init_timer(&v->periodic_timer, vcpu_periodic_timer_fn,
               v, v->processor);
    init_timer(&v->singleshot_timer, vcpu_singleshot_timer_fn,
               v, v->processor);
    init_timer(&v->poll_timer, poll_timer_fn,
               v, v->processor);

    /* If this is not the first vcpu of the item we are done. */
    if ( item->priv != NULL )
    {
        /* We can rely on previous vcpu to exist. */
        v->processor = cpumask_next(d->vcpu[v->vcpu_id - 1]->processor,
                                    item->res->cpus);
        return 0;
    }

    /* The first vcpu of an item can be set via sched_set_res(). */
    sched_set_res(item, per_cpu(sched_res, processor));

    item->priv = sched_alloc_vdata(dom_scheduler(d), item, d->sched_priv);
    if ( item->priv == NULL )
    {
        sched_free_item(item, v);
        return 1;
    }

    /*
     * Initialize affinity settings. The idler, and potentially
     * domain-0 VCPUs, are pinned onto their respective physical CPUs.
     */
    if ( is_idle_domain(d) || (is_hardware_domain(d) && opt_dom0_vcpus_pin) )
        sched_set_affinity(v, cpumask_of(processor), &cpumask_all);
    else
        sched_set_affinity(v, &cpumask_all, &cpumask_all);

    /* Idle VCPUs are scheduled immediately, so don't put them in runqueue. */
    if ( is_idle_domain(d) )
    {
        per_cpu(sched_res, v->processor)->curr = item;
        v->is_running = 1;
        item->is_running = 1;
        item->state_entry_time = NOW();
    }
    else
    {
        sched_insert_item(dom_scheduler(d), item);
    }

    return 0;
}

static void vcpu_move_irqs(struct vcpu *v)
{
    arch_move_irqs(v);
    evtchn_move_pirqs(v);
}

static void sched_move_irqs(struct sched_item *item)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        vcpu_move_irqs(v);
}

int sched_move_domain(struct domain *d, struct cpupool *c)
{
    struct vcpu *v;
    struct sched_item *item;
    unsigned int new_p;
    void **item_priv;
    void *domdata;
    void *itemdata;
    struct scheduler *old_ops;
    void *old_domdata;

    for_each_sched_item ( d, item )
    {
        if ( item->affinity_broken )
            return -EBUSY;
    }

    domdata = sched_alloc_domdata(c->sched, d);
    if ( IS_ERR(domdata) )
        return PTR_ERR(domdata);

    item_priv = xzalloc_array(void *, d->max_vcpus);
    if ( item_priv == NULL )
    {
        sched_free_domdata(c->sched, domdata);
        return -ENOMEM;
    }

    for_each_sched_item ( d, item )
    {
        item_priv[item->item_id] = sched_alloc_vdata(c->sched, item, domdata);
        if ( item_priv[item->item_id] == NULL )
        {
            for_each_sched_item ( d, item )
                xfree(item_priv[item->item_id]);
            xfree(item_priv);
            sched_free_domdata(c->sched, domdata);
            return -ENOMEM;
        }
    }

    domain_pause(d);

    old_ops = dom_scheduler(d);
    old_domdata = d->sched_priv;

    for_each_sched_item ( d, item )
    {
        sched_remove_item(old_ops, item);
    }

    d->cpupool = c;
    d->sched_priv = domdata;

    new_p = cpumask_first(c->cpu_valid);
    for_each_sched_item ( d, item )
    {
        spinlock_t *lock;
        unsigned int item_p = new_p;

        itemdata = item->priv;

        for_each_sched_item_vcpu( item, v )
        {
            migrate_timer(&v->periodic_timer, new_p);
            migrate_timer(&v->singleshot_timer, new_p);
            migrate_timer(&v->poll_timer, new_p);
            new_p = cpumask_cycle(new_p, c->cpu_valid);
        }

        lock = item_schedule_lock_irq(item);

        sched_set_affinity(item->vcpu, &cpumask_all, &cpumask_all);

        sched_set_res(item, per_cpu(sched_res, item_p));
        /*
         * With v->processor modified we must not
         * - make any further changes assuming we hold the scheduler lock,
         * - use item_schedule_unlock_irq().
         */
        spin_unlock_irq(lock);

        item->priv = item_priv[item->item_id];
        if ( !d->is_dying )
            sched_move_irqs(v->sched_item);

        sched_insert_item(c->sched, item);

        sched_free_vdata(old_ops, itemdata);
    }

    domain_update_node_affinity(d);

    domain_unpause(d);

    sched_free_domdata(old_ops, old_domdata);

    xfree(item_priv);

    return 0;
}

void sched_destroy_vcpu(struct vcpu *v)
{
    struct sched_item *item = v->sched_item;

    kill_timer(&v->periodic_timer);
    kill_timer(&v->singleshot_timer);
    kill_timer(&v->poll_timer);
    if ( test_and_clear_bool(v->is_urgent) )
        atomic_dec(&per_cpu(sched_res, v->processor)->urgent_count);
    /*
     * Vcpus are being destroyed top-down. So being the first vcpu of an item
     * is the same as being the only one.
     */
    if ( item->vcpu == v )
    {
        sched_remove_item(vcpu_scheduler(v), item);
        sched_free_vdata(vcpu_scheduler(v), item->priv);
        sched_free_item(item, v);
    }
}

int sched_init_domain(struct domain *d, int poolid)
{
    void *sdom;
    int ret;

    ASSERT(d->cpupool == NULL);
    ASSERT(d->domain_id < DOMID_FIRST_RESERVED);

    if ( (ret = cpupool_add_domain(d, poolid)) )
        return ret;

    SCHED_STAT_CRANK(dom_init);
    TRACE_1D(TRC_SCHED_DOM_ADD, d->domain_id);

    sdom = sched_alloc_domdata(dom_scheduler(d), d);
    if ( IS_ERR(sdom) )
        return PTR_ERR(sdom);

    d->sched_priv = sdom;

    return 0;
}

void sched_destroy_domain(struct domain *d)
{
    ASSERT(d->domain_id < DOMID_FIRST_RESERVED);

    if ( d->cpupool )
    {
        SCHED_STAT_CRANK(dom_destroy);
        TRACE_1D(TRC_SCHED_DOM_REM, d->domain_id);

        sched_free_domdata(dom_scheduler(d), d->sched_priv);
        d->sched_priv = NULL;

        cpupool_rm_domain(d);
    }
}

void vcpu_sleep_nosync_locked(struct vcpu *v)
{
    ASSERT(spin_is_locked(per_cpu(sched_res, v->processor)->schedule_lock));

    if ( likely(!vcpu_runnable(v)) )
    {
        if ( v->runstate.state == RUNSTATE_runnable )
            vcpu_runstate_change(v, RUNSTATE_offline, NOW());

        sched_sleep(vcpu_scheduler(v), v->sched_item);
    }
}

void vcpu_sleep_nosync(struct vcpu *v)
{
    unsigned long flags;
    spinlock_t *lock;

    TRACE_2D(TRC_SCHED_SLEEP, v->domain->domain_id, v->vcpu_id);

    lock = item_schedule_lock_irqsave(v->sched_item, &flags);

    vcpu_sleep_nosync_locked(v);

    item_schedule_unlock_irqrestore(lock, flags, v->sched_item);
}

void vcpu_sleep_sync(struct vcpu *v)
{
    vcpu_sleep_nosync(v);

    while ( !vcpu_runnable(v) && v->is_running )
        cpu_relax();

    sync_vcpu_execstate(v);
}

void vcpu_wake(struct vcpu *v)
{
    unsigned long flags;
    spinlock_t *lock;

    TRACE_2D(TRC_SCHED_WAKE, v->domain->domain_id, v->vcpu_id);

    lock = item_schedule_lock_irqsave(v->sched_item, &flags);

    if ( likely(vcpu_runnable(v)) )
    {
        if ( v->runstate.state >= RUNSTATE_blocked )
            vcpu_runstate_change(v, RUNSTATE_runnable, NOW());
        sched_wake(vcpu_scheduler(v), v->sched_item);
    }
    else if ( !(v->pause_flags & VPF_blocked) )
    {
        if ( v->runstate.state == RUNSTATE_blocked )
            vcpu_runstate_change(v, RUNSTATE_offline, NOW());
    }

    item_schedule_unlock_irqrestore(lock, flags, v->sched_item);
}

void vcpu_unblock(struct vcpu *v)
{
    if ( !test_and_clear_bit(_VPF_blocked, &v->pause_flags) )
        return;

    /* Polling period ends when a VCPU is unblocked. */
    if ( unlikely(v->poll_evtchn != 0) )
    {
        v->poll_evtchn = 0;
        /*
         * We *must* re-clear _VPF_blocked to avoid racing other wakeups of
         * this VCPU (and it then going back to sleep on poll_mask).
         * Test-and-clear is idiomatic and ensures clear_bit not reordered.
         */
        if ( test_and_clear_bit(v->vcpu_id, v->domain->poll_mask) )
            clear_bit(_VPF_blocked, &v->pause_flags);
    }

    vcpu_wake(v);
}

/*
 * Do the actual movement of an item from old to new CPU. Locks for *both*
 * CPUs needs to have been taken already when calling this!
 */
static void sched_item_move_locked(struct sched_item *item,
                                   unsigned int new_cpu)
{
    unsigned int old_cpu = item->res->processor;
    struct vcpu *v;

    /*
     * Transfer urgency status to new CPU before switching CPUs, as
     * once the switch occurs, v->is_urgent is no longer protected by
     * the per-CPU scheduler lock we are holding.
     */
    for_each_sched_item_vcpu ( item, v )
    {
        if ( unlikely(v->is_urgent) && (old_cpu != new_cpu) )
        {
            atomic_inc(&per_cpu(sched_res, new_cpu)->urgent_count);
            atomic_dec(&per_cpu(sched_res, old_cpu)->urgent_count);
        }
    }

    /*
     * Actual CPU switch to new CPU.  This is safe because the lock
     * pointer can't change while the current lock is held.
     */
    sched_migrate(vcpu_scheduler(item->vcpu), item, new_cpu);
}

/*
 * Initiating migration
 *
 * In order to migrate, we need the item in question to have stopped
 * running and had sched_sleep() called (to take it off any
 * runqueues, for instance); and if it is currently running, it needs
 * to be scheduled out.  Finally, we need to hold the scheduling locks
 * for both the processor we're migrating from, and the processor
 * we're migrating to.
 *
 * In order to avoid deadlock while satisfying the final requirement,
 * we must release any scheduling lock we hold, then try to grab both
 * locks we want, then double-check to make sure that what we started
 * to do hasn't been changed in the mean time.
 *
 * These steps are encapsulated in the following two functions; they
 * should be called like this:
 *
 *     lock = item_schedule_lock_irq(item);
 *     sched_item_migrate_start(item);
 *     item_schedule_unlock_irq(lock, item)
 *     sched_item_migrate_finish(item);
 *
 * sched_item_migrate_finish() will do the work now if it can, or simply
 * return if it can't (because item is still running); in that case
 * sched_item_migrate_finish() will be called by context_saved().
 */
static void sched_item_migrate_start(struct sched_item *item)
{
    struct vcpu *v;

    for_each_sched_item_vcpu ( item, v )
    {
        set_bit(_VPF_migrating, &v->pause_flags);
        vcpu_sleep_nosync_locked(v);
    }
}

static void sched_item_migrate_finish(struct sched_item *item)
{
    unsigned long flags;
    unsigned int old_cpu, new_cpu;
    spinlock_t *old_lock, *new_lock;
    bool_t pick_called = 0;
    struct vcpu *v;

    /*
     * If the item is currently running, this will be handled by
     * context_saved(); and in any case, if the bit is cleared, then
     * someone else has already done the work so we don't need to.
     */
    for_each_sched_item_vcpu ( item, v )
    {
        if ( item->is_running || !test_bit(_VPF_migrating, &v->pause_flags) )
            return;
    }

    old_cpu = new_cpu = item->res->processor;
    for ( ; ; )
    {
        /*
         * We need another iteration if the pre-calculated lock addresses
         * are not correct any longer after evaluating old and new cpu holding
         * the locks.
         */
        old_lock = per_cpu(sched_res, old_cpu)->schedule_lock;
        new_lock = per_cpu(sched_res, new_cpu)->schedule_lock;

        sched_spin_lock_double(old_lock, new_lock, &flags);

        old_cpu = item->res->processor;
        if ( old_lock == per_cpu(sched_res, old_cpu)->schedule_lock )
        {
            /*
             * If we selected a CPU on the previosu iteration, check if it
             * remains suitable for running this vCPU.
             */
            if ( pick_called &&
                 (new_lock == per_cpu(sched_res, new_cpu)->schedule_lock) &&
                 cpumask_test_cpu(new_cpu, item->cpu_hard_affinity) &&
                 cpumask_test_cpu(new_cpu, item->domain->cpupool->cpu_valid) )
                break;

            /* Select a new CPU. */
            new_cpu = sched_pick_resource(vcpu_scheduler(item->vcpu),
                                          item)->processor;
            if ( (new_lock == per_cpu(sched_res, new_cpu)->schedule_lock) &&
                 cpumask_test_cpu(new_cpu, item->domain->cpupool->cpu_valid) )
                break;
            pick_called = 1;
        }
        else
        {
            /*
             * We do not hold the scheduler lock appropriate for this vCPU.
             * Thus we cannot select a new CPU on this iteration. Try again.
             */
            pick_called = 0;
        }

        sched_spin_unlock_double(old_lock, new_lock, flags);
    }

    /*
     * NB. Check of v->running happens /after/ setting migration flag
     * because they both happen in (different) spinlock regions, and those
     * regions are strictly serialised.
     */
    for_each_sched_item_vcpu ( item, v )
    {
        if ( item->is_running ||
             !test_and_clear_bit(_VPF_migrating, &v->pause_flags) )
        {
            sched_spin_unlock_double(old_lock, new_lock, flags);
            return;
        }
    }

    sched_item_move_locked(item, new_cpu);

    sched_spin_unlock_double(old_lock, new_lock, flags);

    if ( old_cpu != new_cpu )
        sched_move_irqs(item);

    /* Wake on new CPU. */
    for_each_sched_item_vcpu ( item, v )
        vcpu_wake(v);
}

/*
 * Set the periodic timer of a vcpu.
 */
void vcpu_set_periodic_timer(struct vcpu *v, s_time_t value)
{
    s_time_t now = NOW();

    if ( v != current )
        vcpu_pause(v);
    else
        stop_timer(&v->periodic_timer);

    v->periodic_period = value;
    v->periodic_last_event = now;

    if ( v != current )
        vcpu_unpause(v);
    else if ( value != 0 )
        set_timer(&v->periodic_timer, now + value);
}

void restore_vcpu_affinity(struct domain *d)
{
    unsigned int cpu = smp_processor_id();
    struct sched_item *item;

    ASSERT(system_state == SYS_STATE_resume);

    for_each_sched_item ( d, item )
    {
        spinlock_t *lock;
        unsigned int old_cpu = sched_item_cpu(item);
        struct sched_resource *res;

        ASSERT(!item_runnable(item));

        /*
         * Re-assign the initial processor as after resume we have no
         * guarantee the old processor has come back to life again.
         *
         * Therefore, here, before actually unpausing the domains, we should
         * set v->processor of each of their vCPUs to something that will
         * make sense for the scheduler of the cpupool in which they are in.
         */
        cpumask_and(cpumask_scratch_cpu(cpu), item->cpu_hard_affinity,
                    cpupool_domain_cpumask(d));
        if ( cpumask_empty(cpumask_scratch_cpu(cpu)) )
        {
            if ( item->affinity_broken )
            {
                sched_set_affinity(item->vcpu, item->cpu_hard_affinity_saved,
                                   NULL);
                item->affinity_broken = 0;
                cpumask_and(cpumask_scratch_cpu(cpu), item->cpu_hard_affinity,
                            cpupool_domain_cpumask(d));
            }

            if ( cpumask_empty(cpumask_scratch_cpu(cpu)) )
            {
                printk(XENLOG_DEBUG "Breaking affinity for %pv\n", item->vcpu);
                sched_set_affinity(item->vcpu, &cpumask_all, NULL);
                cpumask_and(cpumask_scratch_cpu(cpu), item->cpu_hard_affinity,
                            cpupool_domain_cpumask(d));
            }
        }

        res = per_cpu(sched_res, cpumask_any(cpumask_scratch_cpu(cpu)));
        sched_set_res(item, res);

        lock = item_schedule_lock_irq(item);
        res = sched_pick_resource(vcpu_scheduler(item->vcpu), item);
        sched_set_res(item, res);
        spin_unlock_irq(lock);

        if ( old_cpu != sched_item_cpu(item) )
            sched_move_irqs(item);
    }

    domain_update_node_affinity(d);
}

/*
 * This function is used by cpu_hotplug code via cpu notifier chain
 * and from cpupools to switch schedulers on a cpu.
 * Caller must get domlist_read_lock.
 */
int cpu_disable_scheduler(unsigned int cpu)
{
    struct domain *d;
    struct cpupool *c;
    cpumask_t online_affinity;
    int ret = 0;

    c = per_cpu(cpupool, cpu);
    if ( c == NULL )
        return ret;

    for_each_domain_in_cpupool ( d, c )
    {
        struct sched_item *item;

        for_each_sched_item ( d, item )
        {
            unsigned long flags;
            spinlock_t *lock = item_schedule_lock_irqsave(item, &flags);

            cpumask_and(&online_affinity, item->cpu_hard_affinity, c->cpu_valid);
            if ( cpumask_empty(&online_affinity) &&
                 cpumask_test_cpu(cpu, item->cpu_hard_affinity) )
            {
                if ( item->affinity_broken )
                {
                    /* The vcpu is temporarily pinned, can't move it. */
                    item_schedule_unlock_irqrestore(lock, flags, item);
                    ret = -EADDRINUSE;
                    break;
                }

                printk(XENLOG_DEBUG "Breaking affinity for %pv\n", item->vcpu);

                sched_set_affinity(item->vcpu, &cpumask_all, NULL);
            }

            if ( sched_item_cpu(item) != sched_get_resource_cpu(cpu) )
            {
                /* The item is not on this cpu, so we can move on. */
                item_schedule_unlock_irqrestore(lock, flags, item);
                continue;
            }

            /* If it is on this cpu, we must send it away.
             * We are doing some cpupool manipulations:
             *  * we want to call the scheduler, and let it re-evaluation
             *    the placement of the vcpu, taking into account the new
             *    cpupool configuration;
             *  * the scheduler will always find a suitable solution, or
             *    things would have failed before getting in here.
             */
            sched_item_migrate_start(item);
            item_schedule_unlock_irqrestore(lock, flags, item);
            sched_item_migrate_finish(item);

            /*
             * The only caveat, in this case, is that if a vcpu active in
             * the hypervisor isn't migratable. In this case, the caller
             * should try again after releasing and reaquiring all locks.
             */
            if ( sched_item_cpu(item) == sched_get_resource_cpu(cpu) )
                ret = -EAGAIN;
        }
    }

    return ret;
}

static int cpu_disable_scheduler_check(unsigned int cpu)
{
    struct domain *d;
    struct cpupool *c;
    struct sched_item *item;

    c = per_cpu(cpupool, cpu);
    if ( c == NULL )
        return 0;

    for_each_domain_in_cpupool ( d, c )
        for_each_sched_item ( d, item )
            if ( item->affinity_broken )
                return -EADDRINUSE;

    return 0;
}

/*
 * In general, this must be called with the scheduler lock held, because the
 * adjust_affinity hook may want to modify the vCPU state. However, when the
 * vCPU is being initialized (either for dom0 or domU) there is no risk of
 * races, and it's fine to not take the look (we're talking about
 * dom0_setup_vcpu() an sched_init_vcpu()).
 */
void sched_set_affinity(
    struct vcpu *v, const cpumask_t *hard, const cpumask_t *soft)
{
    struct sched_item *item = v->sched_item;

    sched_adjust_affinity(dom_scheduler(v->domain), item, hard, soft);

    if ( hard )
        cpumask_copy(item->cpu_hard_affinity, hard);
    if ( soft )
        cpumask_copy(item->cpu_soft_affinity, soft);

    item->soft_aff_effective = !cpumask_subset(item->cpu_hard_affinity,
                                               item->cpu_soft_affinity) &&
                               cpumask_intersects(item->cpu_soft_affinity,
                                                  item->cpu_hard_affinity);
}

static int vcpu_set_affinity(
    struct vcpu *v, const cpumask_t *affinity, const cpumask_t *which)
{
    struct sched_item *item = v->sched_item;
    spinlock_t *lock;
    int ret = 0;

    lock = item_schedule_lock_irq(item);

    if ( item->affinity_broken )
        ret = -EBUSY;
    else
    {
        /*
         * Tell the scheduler we changes something about affinity,
         * and ask to re-evaluate vcpu placement.
         */
        if ( which == item->cpu_hard_affinity )
        {
            sched_set_affinity(v, affinity, NULL);
        }
        else
        {
            ASSERT(which == item->cpu_soft_affinity);
            sched_set_affinity(v, NULL, affinity);
        }
        sched_item_migrate_start(item);
    }

    item_schedule_unlock_irq(lock, item);

    domain_update_node_affinity(v->domain);

    sched_item_migrate_finish(item);

    return ret;
}

int vcpu_set_hard_affinity(struct vcpu *v, const cpumask_t *affinity)
{
    cpumask_t online_affinity;
    cpumask_t *online;

    online = VCPU2ONLINE(v);
    cpumask_and(&online_affinity, affinity, online);
    if ( cpumask_empty(&online_affinity) )
        return -EINVAL;

    return vcpu_set_affinity(v, affinity, v->sched_item->cpu_hard_affinity);
}

int vcpu_set_soft_affinity(struct vcpu *v, const cpumask_t *affinity)
{
    return vcpu_set_affinity(v, affinity, v->sched_item->cpu_soft_affinity);
}

/* Block the currently-executing domain until a pertinent event occurs. */
void vcpu_block(void)
{
    struct vcpu *v = current;

    set_bit(_VPF_blocked, &v->pause_flags);

    arch_vcpu_block(v);

    /* Check for events /after/ blocking: avoids wakeup waiting race. */
    if ( local_events_need_delivery() )
    {
        clear_bit(_VPF_blocked, &v->pause_flags);
    }
    else
    {
        TRACE_2D(TRC_SCHED_BLOCK, v->domain->domain_id, v->vcpu_id);
        raise_softirq(SCHEDULE_SOFTIRQ);
    }
}

static void vcpu_block_enable_events(void)
{
    local_event_delivery_enable();
    vcpu_block();
}

static long do_poll(struct sched_poll *sched_poll)
{
    struct vcpu   *v = current;
    struct domain *d = v->domain;
    evtchn_port_t  port = 0;
    long           rc;
    unsigned int   i;

    /* Fairly arbitrary limit. */
    if ( sched_poll->nr_ports > 128 )
        return -EINVAL;

    if ( !guest_handle_okay(sched_poll->ports, sched_poll->nr_ports) )
        return -EFAULT;

    set_bit(_VPF_blocked, &v->pause_flags);
    v->poll_evtchn = -1;
    set_bit(v->vcpu_id, d->poll_mask);

    arch_vcpu_block(v);

#ifndef CONFIG_X86 /* set_bit() implies mb() on x86 */
    /* Check for events /after/ setting flags: avoids wakeup waiting race. */
    smp_mb();

    /*
     * Someone may have seen we are blocked but not that we are polling, or
     * vice versa. We are certainly being woken, so clean up and bail. Beyond
     * this point others can be guaranteed to clean up for us if they wake us.
     */
    rc = 0;
    if ( (v->poll_evtchn == 0) ||
         !test_bit(_VPF_blocked, &v->pause_flags) ||
         !test_bit(v->vcpu_id, d->poll_mask) )
        goto out;
#endif

    rc = 0;
    if ( local_events_need_delivery() )
        goto out;

    for ( i = 0; i < sched_poll->nr_ports; i++ )
    {
        rc = -EFAULT;
        if ( __copy_from_guest_offset(&port, sched_poll->ports, i, 1) )
            goto out;

        rc = -EINVAL;
        if ( port >= d->max_evtchns )
            goto out;

        rc = 0;
        if ( evtchn_port_is_pending(d, port) )
            goto out;
    }

    if ( sched_poll->nr_ports == 1 )
        v->poll_evtchn = port;

    if ( sched_poll->timeout != 0 )
        set_timer(&v->poll_timer, sched_poll->timeout);

    TRACE_2D(TRC_SCHED_BLOCK, d->domain_id, v->vcpu_id);
    raise_softirq(SCHEDULE_SOFTIRQ);

    return 0;

 out:
    v->poll_evtchn = 0;
    clear_bit(v->vcpu_id, d->poll_mask);
    clear_bit(_VPF_blocked, &v->pause_flags);
    return rc;
}

/* Voluntarily yield the processor for this allocation. */
long vcpu_yield(void)
{
    struct vcpu * v=current;
    spinlock_t *lock = item_schedule_lock_irq(v->sched_item);

    sched_yield(vcpu_scheduler(v), v->sched_item);
    item_schedule_unlock_irq(lock, v->sched_item);

    SCHED_STAT_CRANK(vcpu_yield);

    TRACE_2D(TRC_SCHED_YIELD, current->domain->domain_id, current->vcpu_id);
    raise_softirq(SCHEDULE_SOFTIRQ);
    return 0;
}

static void domain_watchdog_timeout(void *data)
{
    struct domain *d = data;

    if ( d->is_shutting_down || d->is_dying )
        return;

    printk("Watchdog timer fired for domain %u\n", d->domain_id);
    domain_shutdown(d, SHUTDOWN_watchdog);
}

static long domain_watchdog(struct domain *d, uint32_t id, uint32_t timeout)
{
    if ( id > NR_DOMAIN_WATCHDOG_TIMERS )
        return -EINVAL;

    spin_lock(&d->watchdog_lock);

    if ( id == 0 )
    {
        for ( id = 0; id < NR_DOMAIN_WATCHDOG_TIMERS; id++ )
        {
            if ( test_and_set_bit(id, &d->watchdog_inuse_map) )
                continue;
            set_timer(&d->watchdog_timer[id], NOW() + SECONDS(timeout));
            break;
        }
        spin_unlock(&d->watchdog_lock);
        return id == NR_DOMAIN_WATCHDOG_TIMERS ? -ENOSPC : id + 1;
    }

    id -= 1;
    if ( !test_bit(id, &d->watchdog_inuse_map) )
    {
        spin_unlock(&d->watchdog_lock);
        return -EINVAL;
    }

    if ( timeout == 0 )
    {
        stop_timer(&d->watchdog_timer[id]);
        clear_bit(id, &d->watchdog_inuse_map);
    }
    else
    {
        set_timer(&d->watchdog_timer[id], NOW() + SECONDS(timeout));
    }

    spin_unlock(&d->watchdog_lock);
    return 0;
}

void watchdog_domain_init(struct domain *d)
{
    unsigned int i;

    spin_lock_init(&d->watchdog_lock);

    d->watchdog_inuse_map = 0;

    for ( i = 0; i < NR_DOMAIN_WATCHDOG_TIMERS; i++ )
        init_timer(&d->watchdog_timer[i], domain_watchdog_timeout, d, 0);
}

void watchdog_domain_destroy(struct domain *d)
{
    unsigned int i;

    for ( i = 0; i < NR_DOMAIN_WATCHDOG_TIMERS; i++ )
        kill_timer(&d->watchdog_timer[i]);
}

int vcpu_pin_override(struct vcpu *v, int cpu)
{
    struct sched_item *item = v->sched_item;
    spinlock_t *lock;
    int ret = -EINVAL;

    lock = item_schedule_lock_irq(item);

    if ( cpu < 0 )
    {
        if ( item->affinity_broken )
        {
            sched_set_affinity(v, item->cpu_hard_affinity_saved, NULL);
            item->affinity_broken = 0;
            ret = 0;
        }
    }
    else if ( cpu < nr_cpu_ids )
    {
        if ( item->affinity_broken )
            ret = -EBUSY;
        else if ( cpumask_test_cpu(cpu, VCPU2ONLINE(v)) )
        {
            cpumask_copy(item->cpu_hard_affinity_saved,
                         item->cpu_hard_affinity);
            item->affinity_broken = 1;
            sched_set_affinity(v, cpumask_of(cpu), NULL);
            ret = 0;
        }
    }

    if ( ret == 0 )
        sched_item_migrate_start(item);

    item_schedule_unlock_irq(lock, item);

    domain_update_node_affinity(v->domain);

    sched_item_migrate_finish(item);

    return ret;
}

typedef long ret_t;

#endif /* !COMPAT */

ret_t do_sched_op(int cmd, XEN_GUEST_HANDLE_PARAM(void) arg)
{
    ret_t ret = 0;

    switch ( cmd )
    {
    case SCHEDOP_yield:
    {
        ret = vcpu_yield();
        break;
    }

    case SCHEDOP_block:
    {
        vcpu_block_enable_events();
        break;
    }

    case SCHEDOP_shutdown:
    {
        struct sched_shutdown sched_shutdown;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_shutdown, arg, 1) )
            break;

        TRACE_3D(TRC_SCHED_SHUTDOWN,
                 current->domain->domain_id, current->vcpu_id,
                 sched_shutdown.reason);
        ret = domain_shutdown(current->domain, (u8)sched_shutdown.reason);

        break;
    }

    case SCHEDOP_shutdown_code:
    {
        struct sched_shutdown sched_shutdown;
        struct domain *d = current->domain;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_shutdown, arg, 1) )
            break;

        TRACE_3D(TRC_SCHED_SHUTDOWN_CODE,
                 d->domain_id, current->vcpu_id, sched_shutdown.reason);

        spin_lock(&d->shutdown_lock);
        if ( d->shutdown_code == SHUTDOWN_CODE_INVALID )
            d->shutdown_code = (u8)sched_shutdown.reason;
        spin_unlock(&d->shutdown_lock);

        ret = 0;
        break;
    }

    case SCHEDOP_poll:
    {
        struct sched_poll sched_poll;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_poll, arg, 1) )
            break;

        ret = do_poll(&sched_poll);

        break;
    }

    case SCHEDOP_remote_shutdown:
    {
        struct domain *d;
        struct sched_remote_shutdown sched_remote_shutdown;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_remote_shutdown, arg, 1) )
            break;

        ret = -ESRCH;
        d = rcu_lock_domain_by_id(sched_remote_shutdown.domain_id);
        if ( d == NULL )
            break;

        ret = xsm_schedop_shutdown(XSM_DM_PRIV, current->domain, d);
        if ( likely(!ret) )
            domain_shutdown(d, sched_remote_shutdown.reason);

        rcu_unlock_domain(d);

        break;
    }

    case SCHEDOP_watchdog:
    {
        struct sched_watchdog sched_watchdog;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_watchdog, arg, 1) )
            break;

        ret = domain_watchdog(
            current->domain, sched_watchdog.id, sched_watchdog.timeout);
        break;
    }

    case SCHEDOP_pin_override:
    {
        struct sched_pin_override sched_pin_override;

        ret = -EPERM;
        if ( !is_hardware_domain(current->domain) )
            break;

        ret = -EFAULT;
        if ( copy_from_guest(&sched_pin_override, arg, 1) )
            break;

        ret = vcpu_pin_override(current, sched_pin_override.pcpu);

        break;
    }

    default:
        ret = -ENOSYS;
    }

    return ret;
}

#ifndef COMPAT

/* Per-vcpu oneshot-timer hypercall. */
long do_set_timer_op(s_time_t timeout)
{
    struct vcpu *v = current;
    s_time_t offset = timeout - NOW();

    if ( timeout == 0 )
    {
        stop_timer(&v->singleshot_timer);
    }
    else if ( unlikely(timeout < 0) || /* overflow into 64th bit? */
              unlikely((offset > 0) && ((uint32_t)(offset >> 50) != 0)) )
    {
        /*
         * Linux workaround: occasionally we will see timeouts a long way in
         * the future due to wrapping in Linux's jiffy time handling. We check
         * for timeouts wrapped negative, and for positive timeouts more than
         * about 13 days in the future (2^50ns). The correct fix is to trigger
         * an interrupt immediately (since Linux in fact has pending work to
         * do in this situation). However, older guests also set a long timeout
         * when they have *no* pending timers at all: setting an immediate
         * timeout in this case can burn a lot of CPU. We therefore go for a
         * reasonable middleground of triggering a timer event in 100ms.
         */
        gdprintk(XENLOG_INFO, "Warning: huge timeout set: %"PRIx64"\n",
                 timeout);
        set_timer(&v->singleshot_timer, NOW() + MILLISECS(100));
    }
    else
    {
        migrate_timer(&v->singleshot_timer, smp_processor_id());
        set_timer(&v->singleshot_timer, timeout);
    }

    return 0;
}

/* sched_id - fetch ID of current scheduler */
int sched_id(void)
{
    return ops.sched_id;
}

/* Adjust scheduling parameter for a given domain. */
long sched_adjust(struct domain *d, struct xen_domctl_scheduler_op *op)
{
    long ret;

    ret = xsm_domctl_scheduler_op(XSM_HOOK, d, op->cmd);
    if ( ret )
        return ret;

    if ( op->sched_id != dom_scheduler(d)->sched_id )
        return -EINVAL;

    switch ( op->cmd )
    {
    case XEN_DOMCTL_SCHEDOP_putinfo:
    case XEN_DOMCTL_SCHEDOP_getinfo:
    case XEN_DOMCTL_SCHEDOP_putvcpuinfo:
    case XEN_DOMCTL_SCHEDOP_getvcpuinfo:
        break;
    default:
        return -EINVAL;
    }

    /* NB: the pluggable scheduler code needs to take care
     * of locking by itself. */
    if ( (ret = sched_adjust_dom(dom_scheduler(d), d, op)) == 0 )
        TRACE_1D(TRC_SCHED_ADJDOM, d->domain_id);

    return ret;
}

long sched_adjust_global(struct xen_sysctl_scheduler_op *op)
{
    struct cpupool *pool;
    int rc;

    rc = xsm_sysctl_scheduler_op(XSM_HOOK, op->cmd);
    if ( rc )
        return rc;

    if ( (op->cmd != XEN_SYSCTL_SCHEDOP_putinfo) &&
         (op->cmd != XEN_SYSCTL_SCHEDOP_getinfo) )
        return -EINVAL;

    pool = cpupool_get_by_id(op->cpupool_id);
    if ( pool == NULL )
        return -ESRCH;

    rc = ((op->sched_id == pool->sched->sched_id)
          ? sched_adjust_cpupool(pool->sched, op) : -EINVAL);

    cpupool_put(pool);

    return rc;
}

static void vcpu_periodic_timer_work(struct vcpu *v)
{
    s_time_t now = NOW();
    s_time_t periodic_next_event;

    if ( v->periodic_period == 0 )
        return;

    periodic_next_event = v->periodic_last_event + v->periodic_period;

    if ( now >= periodic_next_event )
    {
        send_timer_event(v);
        v->periodic_last_event = now;
        periodic_next_event = now + v->periodic_period;
    }

    migrate_timer(&v->periodic_timer, smp_processor_id());
    set_timer(&v->periodic_timer, periodic_next_event);
}

static void sched_switch_items(struct sched_resource *sd,
                               struct sched_item *next, struct sched_item *prev,
                               s_time_t now)
{
    sd->curr = next;

    TRACE_3D(TRC_SCHED_SWITCH_INFPREV, prev->domain->domain_id, prev->item_id,
             now - prev->state_entry_time);
    TRACE_4D(TRC_SCHED_SWITCH_INFNEXT, next->domain->domain_id, next->item_id,
             (next->vcpu->runstate.state == RUNSTATE_runnable) ?
             (now - next->state_entry_time) : 0, prev->next_time);

    ASSERT(item_running(prev));

    TRACE_4D(TRC_SCHED_SWITCH, prev->domain->domain_id, prev->item_id,
             next->domain->domain_id, next->item_id);

    sched_item_runstate_change(prev, false, now);
    prev->last_run_time = now;

    ASSERT(!item_running(next));
    sched_item_runstate_change(next, true, now);

    /*
     * NB. Don't add any trace records from here until the actual context
     * switch, else lost_records resume will not work properly.
     */

    ASSERT(!next->is_running);
    next->vcpu->is_running = 1;
    next->is_running = 1;
}

static bool sched_tasklet_check(void)
{
    unsigned long *tasklet_work;
    bool tasklet_work_scheduled = false;
    const cpumask_t *mask = this_cpu(sched_res)->cpus;
    int cpu;

    for_each_cpu ( cpu, mask )
    {
        tasklet_work = &per_cpu(tasklet_work_to_do, cpu);

        switch ( *tasklet_work )
        {
        case TASKLET_enqueued:
            set_bit(_TASKLET_scheduled, tasklet_work);
            /* fallthrough */
        case TASKLET_enqueued|TASKLET_scheduled:
            tasklet_work_scheduled = true;
            break;
        case TASKLET_scheduled:
            clear_bit(_TASKLET_scheduled, tasklet_work);
        case 0:
            /*tasklet_work_scheduled = false;*/
            break;
        default:
            BUG();
        }
    }

    return tasklet_work_scheduled;
}

static struct sched_item *do_schedule(struct sched_item *prev, s_time_t now)
{
    struct scheduler *sched = this_cpu(scheduler);
    struct sched_resource *sd = this_cpu(sched_res);
    struct sched_item *next;

    /* get policy-specific decision on scheduling... */
    sched->do_schedule(sched, prev, now, sched_tasklet_check());

    next = prev->next_task;

    if ( prev->next_time >= 0 ) /* -ve means no limit */
        set_timer(&sd->s_timer, now + prev->next_time);

    if ( likely(prev != next) )
        sched_switch_items(sd, next, prev, now);

    return next;
}

static void context_saved(struct vcpu *prev)
{
    struct sched_item *item = prev->sched_item;

    item->is_running = 0;
    item->state_entry_time = NOW();

    /* Check for migration request /after/ clearing running flag. */
    smp_mb();

    sched_context_saved(vcpu_scheduler(prev), item);

    sched_item_migrate_finish(item);
}

/*
 * Rendezvous on end of context switch.
 * As no lock is protecting this rendezvous function we need to use atomic
 * access functions on the counter.
 * The counter will be 0 in case no rendezvous is needed. For the rendezvous
 * case it is initialised to the number of cpus to rendezvous plus 1. Each
 * member entering decrements the counter. The last one will decrement it to
 * 1 and perform the final needed action in that case (call of context_saved()
 * if vcpu was switched), and then set the counter to zero. The other members
 * will wait until the counter becomes zero until they proceed.
 */
void sched_context_switched(struct vcpu *vprev, struct vcpu *vnext)
{
    struct sched_item *next = vnext->sched_item;

    /* Clear running flag /after/ writing context to memory. */
    smp_wmb();

    vprev->is_running = 0;

    if ( atomic_read(&next->rendezvous_out_cnt) )
    {
        int cnt = atomic_dec_return(&next->rendezvous_out_cnt);

        /* Call context_saved() before releasing other waiters. */
        if ( cnt == 1 )
        {
            if ( vprev != vnext )
                context_saved(vprev);
            atomic_set(&next->rendezvous_out_cnt, 0);
        }
        else
            while ( atomic_read(&next->rendezvous_out_cnt) )
                cpu_relax();
    }
    else if ( vprev != vnext && sched_granularity == 1 )
        context_saved(vprev);
}

static void sched_context_switch(struct vcpu *vprev, struct vcpu *vnext,
                                 s_time_t now)
{
    if ( unlikely(vprev == vnext) )
    {
        TRACE_4D(TRC_SCHED_SWITCH_INFCONT,
                 vnext->domain->domain_id, vnext->sched_item->item_id,
                 now - vprev->runstate.state_entry_time,
                 vprev->sched_item->next_time);
        sched_context_switched(vprev, vnext);
        trace_continue_running(vnext);
        return continue_running(vprev);
    }

    SCHED_STAT_CRANK(sched_ctx);

    stop_timer(&vprev->periodic_timer);

    if ( vnext->sched_item->migrated )
        vcpu_move_irqs(vnext);

    vcpu_periodic_timer_work(vnext);

    context_switch(vprev, vnext);
}

/*
 * Rendezvous before taking a scheduling decision.
 * Called with schedule lock held, so all accesses to the rendezvous counter
 * can be normal ones (no atomic accesses needed).
 * The counter is initialized to the number of cpus to rendezvous initially.
 * Each cpu entering will decrement the counter. In case the counter becomes
 * zero do_schedule() is called and the rendezvous counter for leaving
 * context_switch() is set. All other members will wait until the counter is
 * becoming zero, dropping the schedule lock in between.
 */
static struct sched_item *sched_wait_rendezvous_in(struct sched_item *prev,
                                                   spinlock_t *lock, int cpu,
                                                   s_time_t now)
{
    struct sched_item *next;

    if ( !--prev->rendezvous_in_cnt )
    {
        next = do_schedule(prev, now);
        atomic_set(&next->rendezvous_out_cnt, sched_granularity + 1);
        return next;
    }

    while ( prev->rendezvous_in_cnt )
    {
        pcpu_schedule_unlock_irq(lock, cpu);
        cpu_relax();
        pcpu_schedule_lock_irq(cpu);
    }

    return prev->next_task;
}

static void sched_slave(void)
{
    struct vcpu          *vprev = current;
    struct sched_item    *prev = vprev->sched_item, *next;
    s_time_t              now;
    spinlock_t           *lock;
    int cpu = smp_processor_id();

    ASSERT_NOT_IN_ATOMIC();

    lock = pcpu_schedule_lock_irq(cpu);

    now = NOW();

    if ( !prev->rendezvous_in_cnt )
    {
        pcpu_schedule_unlock_irq(lock, cpu);
        return;
    }

    stop_timer(&this_cpu(sched_res)->s_timer);

    next = sched_wait_rendezvous_in(prev, lock, cpu, now);

    pcpu_schedule_unlock_irq(lock, cpu);

    sched_context_switch(vprev, next->vcpu, now);
}

/*
 * The main function
 * - deschedule the current domain (scheduler independent).
 * - pick a new domain (scheduler dependent).
 */
static void schedule(void)
{
    struct vcpu          *vnext, *vprev = current;
    struct sched_item    *prev = vprev->sched_item, *next = NULL;
    s_time_t              now;
    struct sched_resource *sd;
    spinlock_t           *lock;
    int cpu = smp_processor_id();

    ASSERT_NOT_IN_ATOMIC();

    SCHED_STAT_CRANK(sched_run);

    sd = this_cpu(sched_res);

    lock = pcpu_schedule_lock_irq(cpu);

    if ( prev->rendezvous_in_cnt )
    {
        /*
         * We have a race: sched_slave() should be called, so raise a softirq
         * in order to re-enter schedule() later and call sched_slave() now.
         */
        pcpu_schedule_unlock_irq(lock, cpu);

        raise_softirq(SCHEDULE_SOFTIRQ);
        return sched_slave();
    }

    now = NOW();

    stop_timer(&sd->s_timer);

    if ( sched_granularity > 1 )
    {
        cpumask_t mask;

        prev->rendezvous_in_cnt = sched_granularity;
        cpumask_andnot(&mask, sd->cpus, cpumask_of(cpu));
        cpumask_raise_softirq(&mask, SCHED_SLAVE_SOFTIRQ);
        next = sched_wait_rendezvous_in(prev, lock, cpu, now);
    }
    else
    {
        prev->rendezvous_in_cnt = 0;
        next = do_schedule(prev, now);
        atomic_set(&next->rendezvous_out_cnt, 0);
    }

    pcpu_schedule_unlock_irq(lock, cpu);

    vnext = next->vcpu;
    sched_context_switch(vprev, vnext, now);
}

/* The scheduler timer: force a run through the scheduler */
static void s_timer_fn(void *unused)
{
    raise_softirq(SCHEDULE_SOFTIRQ);
    SCHED_STAT_CRANK(sched_irq);
}

/* Per-VCPU periodic timer function: sends a virtual timer interrupt. */
static void vcpu_periodic_timer_fn(void *data)
{
    struct vcpu *v = data;
    vcpu_periodic_timer_work(v);
}

/* Per-VCPU single-shot timer function: sends a virtual timer interrupt. */
static void vcpu_singleshot_timer_fn(void *data)
{
    struct vcpu *v = data;
    send_timer_event(v);
}

/* SCHEDOP_poll timeout callback. */
static void poll_timer_fn(void *data)
{
    struct vcpu *v = data;

    if ( test_and_clear_bit(v->vcpu_id, v->domain->poll_mask) )
        vcpu_unblock(v);
}

static int cpu_schedule_up(unsigned int cpu)
{
    struct sched_resource *sd;
    void *sched_priv;

    sd = xzalloc(struct sched_resource);
    if ( sd == NULL )
        return -ENOMEM;
    sd->processor = cpu;
    sd->cpus = cpumask_of(cpu);
    per_cpu(sched_res, cpu) = sd;

    per_cpu(scheduler, cpu) = &ops;
    spin_lock_init(&sd->_lock);
    sd->schedule_lock = &sd->_lock;
    init_timer(&sd->s_timer, s_timer_fn, NULL, cpu);
    atomic_set(&sd->urgent_count, 0);

    /* Boot CPU is dealt with later in schedule_init(). */
    if ( cpu == 0 )
        return 0;

    if ( idle_vcpu[cpu] == NULL )
        vcpu_create(idle_vcpu[0]->domain, cpu);
    else
    {
        struct vcpu *idle = idle_vcpu[cpu];
        struct sched_item *item = idle->sched_item;

        /*
         * During (ACPI?) suspend the idle vCPU for this pCPU is not freed,
         * while its scheduler specific data (what is pointed by sched_priv)
         * is. Also, at this stage of the resume path, we attach the pCPU
         * to the default scheduler, no matter in what cpupool it was before
         * suspend. To avoid inconsistency, let's allocate default scheduler
         * data for the idle vCPU here. If the pCPU was in a different pool
         * with a different scheduler, it is schedule_cpu_switch(), invoked
         * later, that will set things up as appropriate.
         */
        ASSERT(item->priv == NULL);

        item->priv = sched_alloc_vdata(&ops, item, idle->domain->sched_priv);
        if ( item->priv == NULL )
            return -ENOMEM;
    }
    if ( idle_vcpu[cpu] == NULL )
        return -ENOMEM;

    sd->curr = idle_vcpu[cpu]->sched_item;

    /*
     * We don't want to risk calling xfree() on an sd->sched_priv
     * (e.g., inside free_pdata, from cpu_schedule_down() called
     * during CPU_UP_CANCELLED) that contains an IS_ERR value.
     */
    sched_priv = sched_alloc_pdata(&ops, cpu);
    if ( IS_ERR(sched_priv) )
        return PTR_ERR(sched_priv);

    sd->sched_priv = sched_priv;

    return 0;
}

static void cpu_schedule_down(unsigned int cpu)
{
    struct sched_resource *sd = per_cpu(sched_res, cpu);
    struct scheduler *sched = per_cpu(scheduler, cpu);

    sched_free_pdata(sched, sd->sched_priv, cpu);
    sched_free_vdata(sched, idle_vcpu[cpu]->sched_item->priv);

    idle_vcpu[cpu]->sched_item->priv = NULL;
    sd->sched_priv = NULL;

    kill_timer(&sd->s_timer);

    xfree(per_cpu(sched_res, cpu));
    per_cpu(sched_res, cpu) = NULL;
}

static int cpu_schedule_callback(
    struct notifier_block *nfb, unsigned long action, void *hcpu)
{
    unsigned int cpu = (unsigned long)hcpu;
    struct scheduler *sched = per_cpu(scheduler, cpu);
    struct sched_resource *sd = per_cpu(sched_res, cpu);
    int rc = 0;

    /*
     * From the scheduler perspective, bringing up a pCPU requires
     * allocating and initializing the per-pCPU scheduler specific data,
     * as well as "registering" this pCPU to the scheduler (which may
     * involve modifying some scheduler wide data structures).
     * This happens by calling the alloc_pdata and init_pdata hooks, in
     * this order. A scheduler that does not need to allocate any per-pCPU
     * data can avoid implementing alloc_pdata. init_pdata may, however, be
     * necessary/useful in this case too (e.g., it can contain the "register
     * the pCPU to the scheduler" part). alloc_pdata (if present) is called
     * during CPU_UP_PREPARE. init_pdata (if present) is called during
     * CPU_STARTING.
     *
     * On the other hand, at teardown, we need to reverse what has been done
     * during initialization, and then free the per-pCPU specific data. This
     * happens by calling the deinit_pdata and free_pdata hooks, in this
     * order. If no per-pCPU memory was allocated, there is no need to
     * provide an implementation of free_pdata. deinit_pdata may, however,
     * be necessary/useful in this case too (e.g., it can undo something done
     * on scheduler wide data structure during init_pdata). Both deinit_pdata
     * and free_pdata are called during CPU_DEAD.
     *
     * If someting goes wrong during bringup, we go to CPU_UP_CANCELLED
     * *before* having called init_pdata. In this case, as there is no
     * initialization needing undoing, only free_pdata should be called.
     * This means it is possible to call free_pdata just after alloc_pdata,
     * without a init_pdata/deinit_pdata "cycle" in between the two.
     *
     * So, in summary, the usage pattern should look either
     *  - alloc_pdata-->init_pdata-->deinit_pdata-->free_pdata, or
     *  - alloc_pdata-->free_pdata.
     */
    switch ( action )
    {
    case CPU_STARTING:
        if ( system_state != SYS_STATE_resume )
            sched_init_pdata(sched, sd->sched_priv, cpu);
        break;
    case CPU_UP_PREPARE:
        if ( system_state != SYS_STATE_resume )
            rc = cpu_schedule_up(cpu);
        break;
    case CPU_DOWN_PREPARE:
        rcu_read_lock(&domlist_read_lock);
        rc = cpu_disable_scheduler_check(cpu);
        rcu_read_unlock(&domlist_read_lock);
        break;
    case CPU_RESUME_FAILED:
    case CPU_DEAD:
        if ( system_state == SYS_STATE_suspend )
            break;
        rcu_read_lock(&domlist_read_lock);
        rc = cpu_disable_scheduler(cpu);
        BUG_ON(rc);
        rcu_read_unlock(&domlist_read_lock);
        sched_deinit_pdata(sched, sd->sched_priv, cpu);
        cpu_schedule_down(cpu);
        break;
    case CPU_UP_CANCELED:
        if ( system_state != SYS_STATE_resume )
            cpu_schedule_down(cpu);
        break;
    default:
        break;
    }

    return !rc ? NOTIFY_DONE : notifier_from_errno(rc);
}

static struct notifier_block cpu_schedule_nfb = {
    .notifier_call = cpu_schedule_callback
};

/* Initialise the data structures. */
void __init scheduler_init(void)
{
    struct domain *idle_domain;
    int i;

    open_softirq(SCHEDULE_SOFTIRQ, schedule);
    open_softirq(SCHED_SLAVE_SOFTIRQ, sched_slave);

    for ( i = 0; i < NUM_SCHEDULERS; i++)
    {
        if ( schedulers[i]->global_init && schedulers[i]->global_init() < 0 )
            schedulers[i] = NULL;
        else if ( !ops.name && !strcmp(schedulers[i]->opt_name, opt_sched) )
            ops = *schedulers[i];
    }

    if ( !ops.name )
    {
        printk("Could not find scheduler: %s\n", opt_sched);
        for ( i = 0; i < NUM_SCHEDULERS; i++ )
            if ( schedulers[i] &&
                 !strcmp(schedulers[i]->opt_name, CONFIG_SCHED_DEFAULT) )
            {
                ops = *schedulers[i];
                break;
            }
        BUG_ON(!ops.name);
        printk("Using '%s' (%s)\n", ops.name, ops.opt_name);
    }

    if ( cpu_schedule_up(0) )
        BUG();
    register_cpu_notifier(&cpu_schedule_nfb);

    printk("Using scheduler: %s (%s)\n", ops.name, ops.opt_name);
    if ( sched_init(&ops) )
        panic("scheduler returned error on init\n");

    if ( sched_ratelimit_us &&
         (sched_ratelimit_us > XEN_SYSCTL_SCHED_RATELIMIT_MAX
          || sched_ratelimit_us < XEN_SYSCTL_SCHED_RATELIMIT_MIN) )
    {
        printk("WARNING: sched_ratelimit_us outside of valid range [%d,%d].\n"
               " Resetting to default %u\n",
               XEN_SYSCTL_SCHED_RATELIMIT_MIN,
               XEN_SYSCTL_SCHED_RATELIMIT_MAX,
               SCHED_DEFAULT_RATELIMIT_US);
        sched_ratelimit_us = SCHED_DEFAULT_RATELIMIT_US;
    }

    idle_domain = domain_create(DOMID_IDLE, NULL, false);
    BUG_ON(IS_ERR(idle_domain));
    BUG_ON(nr_cpu_ids > ARRAY_SIZE(idle_vcpu));
    idle_domain->vcpu = idle_vcpu;
    idle_domain->max_vcpus = nr_cpu_ids;
    if ( vcpu_create(idle_domain, 0) == NULL )
        BUG();
    this_cpu(sched_res)->curr = idle_vcpu[0]->sched_item;
    this_cpu(sched_res)->sched_priv = sched_alloc_pdata(&ops, 0);
    BUG_ON(IS_ERR(this_cpu(sched_res)->sched_priv));
    sched_init_pdata(&ops, this_cpu(sched_res)->sched_priv, 0);
}

/*
 * Move a pCPU outside of the influence of the scheduler of its current
 * cpupool, or subject it to the scheduler of a new cpupool.
 *
 * For the pCPUs that are removed from their cpupool, their scheduler becomes
 * &ops (the default scheduler, selected at boot, which also services the
 * default cpupool). However, as these pCPUs are not really part of any pool,
 * there won't be any scheduling event on them, not even from the default
 * scheduler. Basically, they will just sit idle until they are explicitly
 * added back to a cpupool.
 */
int schedule_cpu_switch(unsigned int cpu, struct cpupool *c)
{
    struct vcpu *idle;
    void *ppriv, *ppriv_old, *vpriv, *vpriv_old;
    struct scheduler *old_ops = per_cpu(scheduler, cpu);
    struct scheduler *new_ops = (c == NULL) ? &ops : c->sched;
    struct cpupool *old_pool = per_cpu(cpupool, cpu);
    spinlock_t * old_lock;

    /*
     * pCPUs only move from a valid cpupool to free (i.e., out of any pool),
     * or from free to a valid cpupool. In the former case (which happens when
     * c is NULL), we want the CPU to have been marked as free already, as
     * well as to not be valid for the source pool any longer, when we get to
     * here. In the latter case (which happens when c is a valid cpupool), we
     * want the CPU to still be marked as free, as well as to not yet be valid
     * for the destination pool.
     */
    ASSERT(c != old_pool && (c != NULL || old_pool != NULL));
    ASSERT(cpumask_test_cpu(cpu, &cpupool_free_cpus));
    ASSERT((c == NULL && !cpumask_test_cpu(cpu, old_pool->cpu_valid)) ||
           (c != NULL && !cpumask_test_cpu(cpu, c->cpu_valid)));

    if ( old_ops == new_ops )
        goto out;

    /*
     * To setup the cpu for the new scheduler we need:
     *  - a valid instance of per-CPU scheduler specific data, as it is
     *    allocated by sched_alloc_pdata(). Note that we do not want to
     *    initialize it yet (i.e., we are not calling sched_init_pdata()).
     *    That will be done by the target scheduler, in sched_switch_sched(),
     *    in proper ordering and with locking.
     *  - a valid instance of per-vCPU scheduler specific data, for the idle
     *    vCPU of cpu. That is what the target scheduler will use for the
     *    sched_priv field of the per-vCPU info of the idle domain.
     */
    idle = idle_vcpu[cpu];
    ppriv = sched_alloc_pdata(new_ops, cpu);
    if ( IS_ERR(ppriv) )
        return PTR_ERR(ppriv);
    vpriv = sched_alloc_vdata(new_ops, idle->sched_item,
                              idle->domain->sched_priv);
    if ( vpriv == NULL )
    {
        sched_free_pdata(new_ops, ppriv, cpu);
        return -ENOMEM;
    }

    sched_do_tick_suspend(old_ops, cpu);

    /*
     * The actual switch, including (if necessary) the rerouting of the
     * scheduler lock to whatever new_ops prefers,  needs to happen in one
     * critical section, protected by old_ops' lock, or races are possible.
     * It is, in fact, the lock of another scheduler that we are taking (the
     * scheduler of the cpupool that cpu still belongs to). But that is ok
     * as, anyone trying to schedule on this cpu will spin until when we
     * release that lock (bottom of this function). When he'll get the lock
     * --thanks to the loop inside *_schedule_lock() functions-- he'll notice
     * that the lock itself changed, and retry acquiring the new one (which
     * will be the correct, remapped one, at that point).
     */
    old_lock = pcpu_schedule_lock_irq(cpu);

    vpriv_old = idle->sched_item->priv;
    ppriv_old = per_cpu(sched_res, cpu)->sched_priv;
    sched_switch_sched(new_ops, cpu, ppriv, vpriv);

    /* _Not_ pcpu_schedule_unlock(): schedule_lock may have changed! */
    spin_unlock_irq(old_lock);

    sched_do_tick_resume(new_ops, cpu);

    sched_deinit_pdata(old_ops, ppriv_old, cpu);

    sched_free_vdata(old_ops, vpriv_old);
    sched_free_pdata(old_ops, ppriv_old, cpu);

 out:
    per_cpu(cpupool, cpu) = c;
    /* When a cpu is added to a pool, trigger it to go pick up some work */
    if ( c != NULL )
        cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);

    return 0;
}

struct scheduler *scheduler_get_default(void)
{
    return &ops;
}

struct scheduler *scheduler_alloc(unsigned int sched_id, int *perr)
{
    int i;
    struct scheduler *sched;

    for ( i = 0; i < NUM_SCHEDULERS; i++ )
        if ( schedulers[i] && schedulers[i]->sched_id == sched_id )
            goto found;
    *perr = -ENOENT;
    return NULL;

 found:
    *perr = -ENOMEM;
    if ( (sched = xmalloc(struct scheduler)) == NULL )
        return NULL;
    memcpy(sched, schedulers[i], sizeof(*sched));
    if ( (*perr = sched_init(sched)) != 0 )
    {
        xfree(sched);
        sched = NULL;
    }

    return sched;
}

void scheduler_free(struct scheduler *sched)
{
    BUG_ON(sched == &ops);
    sched_deinit(sched);
    xfree(sched);
}

void schedule_dump(struct cpupool *c)
{
    unsigned int      i;
    struct scheduler *sched;
    cpumask_t        *cpus;

    /* Locking, if necessary, must be handled withing each scheduler */

    if ( c != NULL )
    {
        sched = c->sched;
        cpus = c->cpu_valid;
        printk("Scheduler: %s (%s)\n", sched->name, sched->opt_name);
        sched_dump_settings(sched);
    }
    else
    {
        sched = &ops;
        cpus = &cpupool_free_cpus;
    }

    if ( sched->dump_cpu_state != NULL )
    {
        printk("CPUs info:\n");
        for_each_cpu (i, cpus)
            sched_dump_cpu_state(sched, i);
    }
}

void sched_tick_suspend(void)
{
    struct scheduler *sched;
    unsigned int cpu = smp_processor_id();

    sched = per_cpu(scheduler, cpu);
    sched_do_tick_suspend(sched, cpu);
    rcu_idle_enter(cpu);
    rcu_idle_timer_start();
}

void sched_tick_resume(void)
{
    struct scheduler *sched;
    unsigned int cpu = smp_processor_id();

    rcu_idle_timer_stop();
    rcu_idle_exit(cpu);
    sched = per_cpu(scheduler, cpu);
    sched_do_tick_resume(sched, cpu);
}

void wait(void)
{
    schedule();
}

#ifdef CONFIG_COMPAT
#include "compat/schedule.c"
#endif

#endif /* !COMPAT */

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
