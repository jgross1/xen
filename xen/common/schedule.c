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

/* Number of vcpus per struct sched_unit. */
enum sched_gran opt_sched_granularity = SCHED_GRAN_cpu;
unsigned int sched_granularity = 1;
bool sched_disable_smt_switching;
const cpumask_t *sched_res_mask = &cpumask_all;

/* Common lock for free cpus. */
static DEFINE_SPINLOCK(sched_free_cpu_lock);

/* Various timer handlers. */
static void s_timer_fn(void *unused);
static void vcpu_periodic_timer_fn(void *data);
static void vcpu_singleshot_timer_fn(void *data);
static void poll_timer_fn(void *data);

/* This is global for now so that private implementations can reach it */
DEFINE_PER_CPU(struct sched_resource *, sched_res);
static DEFINE_PER_CPU(unsigned int, sched_res_idx);

/* Scratch space for cpumasks. */
DEFINE_PER_CPU(cpumask_t, cpumask_scratch);

extern const struct scheduler *__start_schedulers_array[], *__end_schedulers_array[];
#define NUM_SCHEDULERS (__end_schedulers_array - __start_schedulers_array)
#define schedulers __start_schedulers_array

static struct scheduler __read_mostly ops;

static inline struct vcpu *unit2vcpu_cpu(struct sched_unit *unit,
                                         unsigned int cpu)
{
    unsigned int idx = unit->unit_id + per_cpu(sched_res_idx, cpu);
    const struct domain *d = unit->domain;

    return (idx < d->max_vcpus && d->vcpu[idx]) ? d->vcpu[idx] : NULL;
}

static inline struct vcpu *sched_unit2vcpu_cpu(struct sched_unit *unit,
                                               unsigned int cpu)
{
    struct vcpu *v = unit2vcpu_cpu(unit, cpu);

    return (v && v->new_state == RUNSTATE_running) ? v : idle_vcpu[cpu];
}

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
    return get_sched_res(v->processor)->scheduler;
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
            atomic_dec(&get_sched_res(v->processor)->urgent_count);
        }
    }
    else
    {
        if ( unlikely(v->pause_flags & VPF_blocked) &&
             unlikely(test_bit(v->vcpu_id, v->domain->poll_mask)) )
        {
            v->is_urgent = 1;
            atomic_inc(&get_sched_res(v->processor)->urgent_count);
        }
    }
}

static inline void vcpu_runstate_change(
    struct vcpu *v, int new_state, s_time_t new_entry_time)
{
    s_time_t delta;
    struct sched_unit *unit = v->sched_unit;

    ASSERT(spin_is_locked(get_sched_res(v->processor)->schedule_lock));
    if ( v->runstate.state == new_state )
        return;

    vcpu_urgent_count_update(v);

    trace_runstate_change(v, new_state);

    if ( !is_idle_vcpu(v) )
    {
        unit->runstate_cnt[v->runstate.state]--;
        unit->runstate_cnt[new_state]++;
    }

    delta = new_entry_time - v->runstate.state_entry_time;
    if ( delta > 0 )
    {
        v->runstate.time[v->runstate.state] += delta;
        v->runstate.state_entry_time = new_entry_time;
    }

    v->runstate.state = new_state;
}

void sched_guest_idle(void (*idle) (void), unsigned int cpu)
{
    atomic_inc(&get_sched_res(cpu)->urgent_count);
    idle();
    atomic_dec(&get_sched_res(cpu)->urgent_count);
}

void vcpu_runstate_get(struct vcpu *v, struct vcpu_runstate_info *runstate)
{
    spinlock_t *lock = likely(v == current)
                       ? NULL : unit_schedule_lock_irq(v->sched_unit);
    s_time_t delta;

    memcpy(runstate, &v->runstate, sizeof(*runstate));
    delta = NOW() - runstate->state_entry_time;
    if ( delta > 0 )
        runstate->time[runstate->state] += delta;

    if ( unlikely(lock != NULL) )
        unit_schedule_unlock_irq(lock, v->sched_unit);
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

static void sched_free_unit_mem(struct sched_unit *unit)
{
    struct sched_unit *prev_unit;
    struct domain *d = unit->domain;

    if ( d->sched_unit_list == unit )
        d->sched_unit_list = unit->next_in_list;
    else
    {
        for_each_sched_unit ( d, prev_unit )
        {
            if ( prev_unit->next_in_list == unit )
            {
                prev_unit->next_in_list = unit->next_in_list;
                break;
            }
        }
    }

    free_cpumask_var(unit->cpu_hard_affinity);
    free_cpumask_var(unit->cpu_hard_affinity_tmp);
    free_cpumask_var(unit->cpu_hard_affinity_saved);
    free_cpumask_var(unit->cpu_soft_affinity);

    xfree(unit);
}

static void sched_free_unit(struct sched_unit *unit, struct vcpu *v)
{
    struct vcpu *vunit;
    unsigned int cnt = 0;

    /* Don't count to be released vcpu, might be not in vcpu list yet. */
    for_each_sched_unit_vcpu ( unit, vunit )
        if ( vunit != v )
            cnt++;

    v->sched_unit = NULL;
    unit->runstate_cnt[v->runstate.state]--;

    if ( unit->vcpu == v )
        unit->vcpu = v->next_in_list;

    if ( !cnt )
        sched_free_unit_mem(unit);
}

static void sched_unit_add_vcpu(struct sched_unit *unit, struct vcpu *v)
{
    v->sched_unit = unit;
    if ( !unit->vcpu || unit->vcpu->vcpu_id > v->vcpu_id )
    {
        unit->vcpu = v;
        unit->unit_id = v->vcpu_id;
    }
    unit->runstate_cnt[v->runstate.state]++;
}

static struct sched_unit *sched_alloc_unit(struct vcpu *v)
{
    struct sched_unit *unit, **prev_unit;
    struct domain *d = v->domain;
    unsigned int gran = d->cpupool ? d->cpupool->granularity : 1;

    for_each_sched_unit ( d, unit )
        if ( unit->vcpu->vcpu_id / gran == v->vcpu_id / gran )
            break;

    if ( unit )
    {
        sched_unit_add_vcpu(unit, v);
        return unit;
    }

    if ( (unit = xzalloc(struct sched_unit)) == NULL )
        return NULL;

    sched_unit_add_vcpu(unit, v);
    unit->domain = d;

    for ( prev_unit = &d->sched_unit_list; *prev_unit;
          prev_unit = &(*prev_unit)->next_in_list )
        if ( (*prev_unit)->next_in_list &&
             (*prev_unit)->next_in_list->unit_id > unit->unit_id )
            break;

    unit->next_in_list = *prev_unit;
    *prev_unit = unit;

    if ( !zalloc_cpumask_var(&unit->cpu_hard_affinity) ||
         !zalloc_cpumask_var(&unit->cpu_hard_affinity_tmp) ||
         !zalloc_cpumask_var(&unit->cpu_hard_affinity_saved) ||
         !zalloc_cpumask_var(&unit->cpu_soft_affinity) )
        goto fail;

    return unit;

 fail:
    sched_free_unit(unit, v);
    return NULL;
}

static unsigned int sched_select_initial_cpu(const struct vcpu *v)
{
    const struct domain *d = v->domain;
    nodeid_t node;
    spinlock_t *lock;
    unsigned long flags;
    unsigned int cpu_ret, cpu = smp_processor_id();
    cpumask_t *cpus = cpumask_scratch_cpu(cpu);

    lock = pcpu_schedule_lock_irqsave(cpu, &flags);
    cpumask_clear(cpus);
    for_each_node_mask ( node, d->node_affinity )
        cpumask_or(cpus, cpus, &node_to_cpumask(node));
    cpumask_and(cpus, cpus, d->cpupool->cpu_valid);
    if ( cpumask_empty(cpus) )
        cpumask_copy(cpus, d->cpupool->cpu_valid);

    if ( v->vcpu_id == 0 )
        cpu_ret = cpumask_first(cpus);
    else
    {
        /* We can rely on previous vcpu being available. */
        ASSERT(!is_idle_domain(d));

        cpu_ret = cpumask_cycle(d->vcpu[v->vcpu_id - 1]->processor, cpus);
    }

    pcpu_schedule_unlock_irqrestore(lock, flags, cpu);

    return cpu_ret;
}

int sched_init_vcpu(struct vcpu *v)
{
    struct domain *d = v->domain;
    struct sched_unit *unit;
    unsigned int processor;

    if ( (unit = sched_alloc_unit(v)) == NULL )
        return 1;

    if ( is_idle_domain(d) )
        processor = v->vcpu_id;
    else
        processor = sched_select_initial_cpu(v);

    /* Initialise the per-vcpu timers. */
    init_timer(&v->periodic_timer, vcpu_periodic_timer_fn, v, processor);
    init_timer(&v->singleshot_timer, vcpu_singleshot_timer_fn, v, processor);
    init_timer(&v->poll_timer, poll_timer_fn, v, processor);

    /* If this is not the first vcpu of the unit we are done. */
    if ( unit->priv != NULL )
    {
        v->processor = processor;
        return 0;
    }

    /* The first vcpu of an unit can be set via sched_set_res(). */
    sched_set_res(unit, get_sched_res(processor));

    unit->priv = sched_alloc_vdata(dom_scheduler(d), unit, d->sched_priv);
    if ( unit->priv == NULL )
    {
        sched_free_unit(unit, v);
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
        get_sched_res(v->processor)->curr = unit;
        get_sched_res(v->processor)->sched_unit_idle = unit;
        v->is_running = 1;
        unit->is_running = 1;
        unit->state_entry_time = NOW();
    }
    else
    {
        sched_insert_unit(dom_scheduler(d), unit);
    }

    return 0;
}

static void vcpu_move_irqs(struct vcpu *v)
{
    arch_move_irqs(v);
    evtchn_move_pirqs(v);
}

static void sched_move_irqs(struct sched_unit *unit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        vcpu_move_irqs(v);
}

int sched_move_domain(struct domain *d, struct cpupool *c)
{
    struct vcpu *v;
    struct sched_unit *unit;
    unsigned int new_p;
    void **unit_priv;
    void *domdata;
    void *unitdata;
    struct scheduler *old_ops;
    void *old_domdata;

    for_each_sched_unit ( d, unit )
    {
        if ( unit->affinity_broken )
            return -EBUSY;
    }

    domdata = sched_alloc_domdata(c->sched, d);
    if ( IS_ERR(domdata) )
        return PTR_ERR(domdata);

    unit_priv = xzalloc_array(void *, d->max_vcpus);
    if ( unit_priv == NULL )
    {
        sched_free_domdata(c->sched, domdata);
        return -ENOMEM;
    }

    for_each_sched_unit ( d, unit )
    {
        unit_priv[unit->unit_id] = sched_alloc_vdata(c->sched, unit, domdata);
        if ( unit_priv[unit->unit_id] == NULL )
        {
            for_each_sched_unit ( d, unit )
                xfree(unit_priv[unit->unit_id]);
            xfree(unit_priv);
            sched_free_domdata(c->sched, domdata);
            return -ENOMEM;
        }
    }

    domain_pause(d);

    old_ops = dom_scheduler(d);
    old_domdata = d->sched_priv;

    for_each_sched_unit ( d, unit )
    {
        sched_remove_unit(old_ops, unit);
    }

    d->cpupool = c;
    d->sched_priv = domdata;

    new_p = cpumask_first(c->cpu_valid);
    for_each_sched_unit ( d, unit )
    {
        spinlock_t *lock;
        unsigned int unit_p = new_p;

        unitdata = unit->priv;

        for_each_sched_unit_vcpu ( unit, v )
        {
            migrate_timer(&v->periodic_timer, new_p);
            migrate_timer(&v->singleshot_timer, new_p);
            migrate_timer(&v->poll_timer, new_p);
            new_p = cpumask_cycle(new_p, c->cpu_valid);
        }

        lock = unit_schedule_lock_irq(unit);

        sched_set_affinity(unit->vcpu, &cpumask_all, &cpumask_all);

        sched_set_res(unit, get_sched_res(unit_p));
        /*
         * With v->processor modified we must not
         * - make any further changes assuming we hold the scheduler lock,
         * - use unit_schedule_unlock_irq().
         */
        spin_unlock_irq(lock);

        unit->priv = unit_priv[unit->unit_id];
        if ( !d->is_dying )
            sched_move_irqs(unit);

        sched_insert_unit(c->sched, unit);

        sched_free_vdata(old_ops, unitdata);
    }

    domain_update_node_affinity(d);

    domain_unpause(d);

    sched_free_domdata(old_ops, old_domdata);

    xfree(unit_priv);

    return 0;
}

void sched_destroy_vcpu(struct vcpu *v)
{
    struct sched_unit *unit = v->sched_unit;

    kill_timer(&v->periodic_timer);
    kill_timer(&v->singleshot_timer);
    kill_timer(&v->poll_timer);
    if ( test_and_clear_bool(v->is_urgent) )
        atomic_dec(&get_sched_res(v->processor)->urgent_count);
    /*
     * Vcpus are being destroyed top-down. So being the first vcpu of an unit
     * is the same as being the only one.
     */
    if ( unit->vcpu == v )
    {
        sched_remove_unit(vcpu_scheduler(v), unit);
        sched_free_vdata(vcpu_scheduler(v), unit->priv);
        sched_free_unit(unit, v);
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

static void vcpu_sleep_nosync_locked(struct vcpu *v)
{
    struct sched_unit *unit = v->sched_unit;

    ASSERT(spin_is_locked(get_sched_res(v->processor)->schedule_lock));

    if ( likely(!vcpu_runnable(v)) )
    {
        if ( v->runstate.state == RUNSTATE_runnable )
            vcpu_runstate_change(v, RUNSTATE_offline, NOW());

        if ( likely(!unit_runnable(unit)) )
            sched_sleep(vcpu_scheduler(v), unit);
        else if ( unit_running(unit) > 1 && v->is_running &&
                  !v->force_context_switch )
        {
            v->force_context_switch = true;
            cpu_raise_softirq(v->processor, SCHED_SLAVE_SOFTIRQ);
        }
    }
}

void vcpu_sleep_nosync(struct vcpu *v)
{
    unsigned long flags;
    spinlock_t *lock;

    TRACE_2D(TRC_SCHED_SLEEP, v->domain->domain_id, v->vcpu_id);

    lock = unit_schedule_lock_irqsave(v->sched_unit, &flags);

    vcpu_sleep_nosync_locked(v);

    unit_schedule_unlock_irqrestore(lock, flags, v->sched_unit);
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
    struct sched_unit *unit = v->sched_unit;

    TRACE_2D(TRC_SCHED_WAKE, v->domain->domain_id, v->vcpu_id);

    lock = unit_schedule_lock_irqsave(unit, &flags);

    if ( likely(vcpu_runnable(v)) )
    {
        if ( v->runstate.state >= RUNSTATE_blocked )
            vcpu_runstate_change(v, RUNSTATE_runnable, NOW());
        sched_wake(vcpu_scheduler(v), unit);
        if ( unit->is_running && !v->is_running && !v->force_context_switch )
        {
            v->force_context_switch = true;
            cpu_raise_softirq(v->processor, SCHED_SLAVE_SOFTIRQ);
        }
    }
    else if ( !(v->pause_flags & VPF_blocked) )
    {
        if ( v->runstate.state == RUNSTATE_blocked )
            vcpu_runstate_change(v, RUNSTATE_offline, NOW());
    }

    unit_schedule_unlock_irqrestore(lock, flags, unit);
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
 * Do the actual movement of an unit from old to new CPU. Locks for *both*
 * CPUs needs to have been taken already when calling this!
 */
static void sched_unit_move_locked(struct sched_unit *unit,
                                   unsigned int new_cpu)
{
    unsigned int old_cpu = unit->res->processor;
    struct vcpu *v;

    /*
     * Transfer urgency status to new CPU before switching CPUs, as
     * once the switch occurs, v->is_urgent is no longer protected by
     * the per-CPU scheduler lock we are holding.
     */
    for_each_sched_unit_vcpu ( unit, v )
    {
        if ( unlikely(v->is_urgent) && (old_cpu != new_cpu) )
        {
            atomic_inc(&get_sched_res(new_cpu)->urgent_count);
            atomic_dec(&get_sched_res(old_cpu)->urgent_count);
        }
    }

    /*
     * Actual CPU switch to new CPU.  This is safe because the lock
     * pointer can't change while the current lock is held.
     */
    sched_migrate(vcpu_scheduler(unit->vcpu), unit, new_cpu);
}

/*
 * Initiating migration
 *
 * In order to migrate, we need the unit in question to have stopped
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
 *     lock = unit_schedule_lock_irq(unit);
 *     sched_unit_migrate_start(unit);
 *     unit_schedule_unlock_irq(lock, unit)
 *     sched_unit_migrate_finish(unit);
 *
 * sched_unit_migrate_finish() will do the work now if it can, or simply
 * return if it can't (because unit is still running); in that case
 * sched_unit_migrate_finish() will be called by context_saved().
 */
static void sched_unit_migrate_start(struct sched_unit *unit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
    {
        set_bit(_VPF_migrating, &v->pause_flags);
        vcpu_sleep_nosync_locked(v);
    }
}

static void sched_unit_migrate_finish(struct sched_unit *unit)
{
    unsigned long flags;
    unsigned int old_cpu, new_cpu;
    spinlock_t *old_lock, *new_lock;
    bool_t pick_called = 0;
    struct vcpu *v;

    /*
     * If the unit is currently running, this will be handled by
     * context_saved(); and in any case, if the bit is cleared, then
     * someone else has already done the work so we don't need to.
     */
    for_each_sched_unit_vcpu ( unit, v )
    {
        if ( unit->is_running || !test_bit(_VPF_migrating, &v->pause_flags) )
            return;
    }

    old_cpu = new_cpu = unit->res->processor;
    for ( ; ; )
    {
        /*
         * We need another iteration if the pre-calculated lock addresses
         * are not correct any longer after evaluating old and new cpu holding
         * the locks.
         */
        old_lock = get_sched_res(old_cpu)->schedule_lock;
        new_lock = get_sched_res(new_cpu)->schedule_lock;

        sched_spin_lock_double(old_lock, new_lock, &flags);

        old_cpu = unit->res->processor;
        if ( old_lock == get_sched_res(old_cpu)->schedule_lock )
        {
            /*
             * If we selected a CPU on the previosu iteration, check if it
             * remains suitable for running this vCPU.
             */
            if ( pick_called &&
                 (new_lock == get_sched_res(new_cpu)->schedule_lock) &&
                 cpumask_test_cpu(new_cpu, unit->cpu_hard_affinity) &&
                 cpumask_test_cpu(new_cpu, unit->domain->cpupool->cpu_valid) )
                break;

            /* Select a new CPU. */
            new_cpu = sched_pick_resource(vcpu_scheduler(unit->vcpu),
                                          unit)->processor;
            if ( (new_lock == get_sched_res(new_cpu)->schedule_lock) &&
                 cpumask_test_cpu(new_cpu, unit->domain->cpupool->cpu_valid) )
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
    for_each_sched_unit_vcpu ( unit, v )
    {
        if ( unit->is_running ||
             !test_and_clear_bit(_VPF_migrating, &v->pause_flags) )
        {
            sched_spin_unlock_double(old_lock, new_lock, flags);
            return;
        }
    }

    sched_unit_move_locked(unit, new_cpu);

    sched_spin_unlock_double(old_lock, new_lock, flags);

    if ( old_cpu != new_cpu )
        sched_move_irqs(unit);

    /* Wake on new CPU. */
    for_each_sched_unit_vcpu ( unit, v )
        vcpu_wake(v);
}

/*
 * Set the periodic timer of a vcpu.
 */
void vcpu_set_periodic_timer(struct vcpu *v, s_time_t value)
{
    s_time_t now;

    if ( v != current )
        vcpu_pause(v);
    else
        stop_timer(&v->periodic_timer);

    now = NOW();
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
    struct sched_unit *unit;

    ASSERT(system_state == SYS_STATE_resume);

    for_each_sched_unit ( d, unit )
    {
        spinlock_t *lock;
        unsigned int old_cpu = sched_unit_cpu(unit);
        struct sched_resource *res;

        ASSERT(!unit_runnable(unit));

        /*
         * Re-assign the initial processor as after resume we have no
         * guarantee the old processor has come back to life again.
         *
         * Therefore, here, before actually unpausing the domains, we should
         * set v->processor of each of their vCPUs to something that will
         * make sense for the scheduler of the cpupool in which they are in.
         */
        cpumask_and(cpumask_scratch_cpu(cpu), unit->cpu_hard_affinity,
                    cpupool_domain_cpumask(d));
        if ( cpumask_empty(cpumask_scratch_cpu(cpu)) )
        {
            if ( unit->affinity_broken )
            {
                sched_set_affinity(unit->vcpu, unit->cpu_hard_affinity_saved,
                                   NULL);
                unit->affinity_broken = 0;
                cpumask_and(cpumask_scratch_cpu(cpu), unit->cpu_hard_affinity,
                            cpupool_domain_cpumask(d));
            }

            if ( cpumask_empty(cpumask_scratch_cpu(cpu)) )
            {
                printk(XENLOG_DEBUG "Breaking affinity for %pv\n", unit->vcpu);
                sched_set_affinity(unit->vcpu, &cpumask_all, NULL);
                cpumask_and(cpumask_scratch_cpu(cpu), unit->cpu_hard_affinity,
                            cpupool_domain_cpumask(d));
            }
        }

        res = get_sched_res(cpumask_any(cpumask_scratch_cpu(cpu)));
        sched_set_res(unit, res);

        lock = unit_schedule_lock_irq(unit);
        res = sched_pick_resource(vcpu_scheduler(unit->vcpu), unit);
        sched_set_res(unit, res);
        spin_unlock_irq(lock);

        if ( old_cpu != sched_unit_cpu(unit) )
            sched_move_irqs(unit);
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

    c = get_sched_res(cpu)->cpupool;
    if ( c == NULL )
        return ret;

    for_each_domain_in_cpupool ( d, c )
    {
        struct sched_unit *unit;

        for_each_sched_unit ( d, unit )
        {
            unsigned long flags;
            spinlock_t *lock = unit_schedule_lock_irqsave(unit, &flags);

            cpumask_and(&online_affinity, unit->cpu_hard_affinity, c->cpu_valid);
            if ( cpumask_empty(&online_affinity) &&
                 cpumask_test_cpu(cpu, unit->cpu_hard_affinity) )
            {
                if ( unit->affinity_broken )
                {
                    /* The vcpu is temporarily pinned, can't move it. */
                    unit_schedule_unlock_irqrestore(lock, flags, unit);
                    ret = -EADDRINUSE;
                    break;
                }

                printk(XENLOG_DEBUG "Breaking affinity for %pv\n", unit->vcpu);

                sched_set_affinity(unit->vcpu, &cpumask_all, NULL);
            }

            if ( sched_unit_cpu(unit) != sched_get_resource_cpu(cpu) )
            {
                /* The unit is not on this cpu, so we can move on. */
                unit_schedule_unlock_irqrestore(lock, flags, unit);
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
            sched_unit_migrate_start(unit);
            unit_schedule_unlock_irqrestore(lock, flags, unit);
            sched_unit_migrate_finish(unit);

            /*
             * The only caveat, in this case, is that if a vcpu active in
             * the hypervisor isn't migratable. In this case, the caller
             * should try again after releasing and reaquiring all locks.
             */
            if ( sched_unit_cpu(unit) == sched_get_resource_cpu(cpu) )
                ret = -EAGAIN;
        }
    }

    return ret;
}

static int cpu_disable_scheduler_check(unsigned int cpu)
{
    struct domain *d;
    struct cpupool *c;
    struct sched_unit *unit;

    c = get_sched_res(cpu)->cpupool;
    if ( c == NULL )
        return 0;

    for_each_domain_in_cpupool ( d, c )
        for_each_sched_unit ( d, unit )
            if ( unit->affinity_broken )
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
    struct sched_unit *unit = v->sched_unit;

    sched_adjust_affinity(dom_scheduler(v->domain), unit, hard, soft);

    if ( hard )
        cpumask_copy(unit->cpu_hard_affinity, hard);
    if ( soft )
        cpumask_copy(unit->cpu_soft_affinity, soft);

    unit->soft_aff_effective = !cpumask_subset(unit->cpu_hard_affinity,
                                               unit->cpu_soft_affinity) &&
                               cpumask_intersects(unit->cpu_soft_affinity,
                                                  unit->cpu_hard_affinity);
}

static int vcpu_set_affinity(
    struct vcpu *v, const cpumask_t *affinity, const cpumask_t *which)
{
    struct sched_unit *unit = v->sched_unit;
    spinlock_t *lock;
    int ret = 0;

    lock = unit_schedule_lock_irq(unit);

    if ( unit->affinity_broken )
        ret = -EBUSY;
    else
    {
        /*
         * Tell the scheduler we changes something about affinity,
         * and ask to re-evaluate vcpu placement.
         */
        if ( which == unit->cpu_hard_affinity )
        {
            sched_set_affinity(v, affinity, NULL);
        }
        else
        {
            ASSERT(which == unit->cpu_soft_affinity);
            sched_set_affinity(v, NULL, affinity);
        }
        sched_unit_migrate_start(unit);
    }

    unit_schedule_unlock_irq(lock, unit);

    domain_update_node_affinity(v->domain);

    sched_unit_migrate_finish(unit);

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

    return vcpu_set_affinity(v, affinity, v->sched_unit->cpu_hard_affinity);
}

int vcpu_set_soft_affinity(struct vcpu *v, const cpumask_t *affinity)
{
    return vcpu_set_affinity(v, affinity, v->sched_unit->cpu_soft_affinity);
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
    spinlock_t *lock = unit_schedule_lock_irq(v->sched_unit);

    sched_yield(vcpu_scheduler(v), v->sched_unit);
    unit_schedule_unlock_irq(lock, v->sched_unit);

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
    struct sched_unit *unit = v->sched_unit;
    spinlock_t *lock;
    int ret = -EINVAL;

    lock = unit_schedule_lock_irq(unit);

    if ( cpu < 0 )
    {
        if ( unit->affinity_broken )
        {
            sched_set_affinity(v, unit->cpu_hard_affinity_saved, NULL);
            unit->affinity_broken = 0;
            ret = 0;
        }
    }
    else if ( cpu < nr_cpu_ids )
    {
        if ( unit->affinity_broken )
            ret = -EBUSY;
        else if ( cpumask_test_cpu(cpu, VCPU2ONLINE(v)) )
        {
            cpumask_copy(unit->cpu_hard_affinity_saved,
                         unit->cpu_hard_affinity);
            unit->affinity_broken = 1;
            sched_set_affinity(v, cpumask_of(cpu), NULL);
            ret = 0;
        }
    }

    if ( ret == 0 )
        sched_unit_migrate_start(unit);

    unit_schedule_unlock_irq(lock, unit);

    domain_update_node_affinity(v->domain);

    sched_unit_migrate_finish(unit);

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

static void sched_switch_units(struct sched_resource *sd,
                               struct sched_unit *next, struct sched_unit *prev,
                               s_time_t now)
{
    int cpu;

    ASSERT(unit_running(prev));

    if ( prev != next )
    {
        sd->curr = next;
        sd->prev = prev;

        TRACE_3D(TRC_SCHED_SWITCH_INFPREV, prev->domain->domain_id,
                 prev->unit_id, now - prev->state_entry_time);
        TRACE_4D(TRC_SCHED_SWITCH_INFNEXT, next->domain->domain_id,
                 next->unit_id,
                 (next->vcpu->runstate.state == RUNSTATE_runnable) ?
                 (now - next->state_entry_time) : 0, prev->next_time);
        TRACE_4D(TRC_SCHED_SWITCH, prev->domain->domain_id, prev->unit_id,
                 next->domain->domain_id, next->unit_id);

        prev->last_run_time = now;

        ASSERT(!unit_running(next));

        /*
         * NB. Don't add any trace records from here until the actual context
         * switch, else lost_records resume will not work properly.
         */

        ASSERT(!next->is_running);
        next->is_running = 1;

        if ( is_idle_unit(prev) )
        {
            prev->runstate_cnt[RUNSTATE_running] = 0;
            prev->runstate_cnt[RUNSTATE_runnable] = 1;
        }
        if ( is_idle_unit(next) )
        {
            next->runstate_cnt[RUNSTATE_running] = 1;
            next->runstate_cnt[RUNSTATE_runnable] = 0;
        }
    }

    for_each_cpu ( cpu, sd->cpus )
    {
        struct vcpu *vprev = get_cpu_current(cpu);
        struct vcpu *vnext = sched_unit2vcpu_cpu(next, cpu);

        if ( vprev != vnext || vprev->runstate.state != vnext->new_state )
        {
            vcpu_runstate_change(vprev,
                ((vprev->pause_flags & VPF_blocked) ? RUNSTATE_blocked :
                 (vcpu_runnable(vprev) ? RUNSTATE_runnable : RUNSTATE_offline)),
                now);
            vcpu_runstate_change(vnext, vnext->new_state, now);
        }

        vnext->is_running = 1;

        if ( is_idle_vcpu(vnext) )
            vnext->sched_unit = next;
    }
}

static bool sched_tasklet_check_cpu(unsigned int cpu)
{
    unsigned long *tasklet_work = &per_cpu(tasklet_work_to_do, cpu);

    switch ( *tasklet_work )
    {
    case TASKLET_enqueued:
        set_bit(_TASKLET_scheduled, tasklet_work);
        /* fallthrough */
    case TASKLET_enqueued|TASKLET_scheduled:
        return true;
        break;
    case TASKLET_scheduled:
        clear_bit(_TASKLET_scheduled, tasklet_work);
        /* fallthrough */
    case 0:
        /* return false; */
        break;
    default:
        BUG();
    }

    return false;
}

static bool sched_tasklet_check(unsigned int cpu)
{
    bool tasklet_work_scheduled = false;
    const cpumask_t *mask = get_sched_res(cpu)->cpus;
    int cpu_iter;

    for_each_cpu ( cpu_iter, mask )
        if ( sched_tasklet_check_cpu(cpu_iter) )
            tasklet_work_scheduled = true;

    return tasklet_work_scheduled;
}

static struct sched_unit *do_schedule(struct sched_unit *prev, s_time_t now,
                                      unsigned int cpu)
{
    struct sched_resource *sd = get_sched_res(cpu);
    struct scheduler *sched = sd->scheduler;
    struct sched_unit *next;

    /* get policy-specific decision on scheduling... */
    sched->do_schedule(sched, prev, now, sched_tasklet_check(cpu));

    next = prev->next_task;

    if ( prev->next_time >= 0 ) /* -ve means no limit */
        set_timer(&sd->s_timer, now + prev->next_time);

    sched_switch_units(sd, next, prev, now);

    return next;
}

static void context_saved(struct sched_unit *unit)
{
    unit->is_running = 0;
    unit->state_entry_time = NOW();
    get_sched_res(smp_processor_id())->prev = NULL;

    /* Check for migration request /after/ clearing running flag. */
    smp_mb();

    sched_context_saved(vcpu_scheduler(unit->vcpu), unit);

    /* Idle never migrates and idle vcpus might belong to other units. */
    if ( !is_idle_unit(unit) )
        sched_unit_migrate_finish(unit);
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
    struct sched_unit *next = vnext->sched_unit;
    struct sched_resource *sd = get_sched_res(smp_processor_id());

    /* Clear running flag /after/ writing context to memory. */
    smp_wmb();

    if ( vprev != vnext )
        vprev->is_running = 0;

    if ( atomic_read(&next->rendezvous_out_cnt) )
    {
        int cnt = atomic_dec_return(&next->rendezvous_out_cnt);

        /* Call context_saved() before releasing other waiters. */
        if ( cnt == 1 )
        {
            if ( sd->prev )
                context_saved(sd->prev);
            atomic_set(&next->rendezvous_out_cnt, 0);
        }
        else
            while ( atomic_read(&next->rendezvous_out_cnt) )
                cpu_relax();
    }
    else if ( sd->prev )
        context_saved(sd->prev);

    if ( is_idle_vcpu(vprev) && vprev != vnext )
        vprev->sched_unit = sd->sched_unit_idle;
}

static void sched_context_switch(struct vcpu *vprev, struct vcpu *vnext,
                                 bool reset_idle_unit, s_time_t now)
{
    if ( unlikely(vprev == vnext) )
    {
        TRACE_4D(TRC_SCHED_SWITCH_INFCONT,
                 vnext->domain->domain_id, vnext->sched_unit->unit_id,
                 now - vprev->runstate.state_entry_time,
                 vprev->sched_unit->next_time);
        sched_context_switched(vprev, vnext);

        if ( reset_idle_unit )
            vnext->sched_unit =
                get_sched_res(smp_processor_id())->sched_unit_idle;

        trace_continue_running(vnext);
        return continue_running(vprev);
    }

    SCHED_STAT_CRANK(sched_ctx);

    stop_timer(&vprev->periodic_timer);

    if ( vnext->sched_unit->migrated )
        vcpu_move_irqs(vnext);

    vcpu_periodic_timer_work(vnext);

    context_switch(vprev, vnext);
}

/*
 * Force a context switch of a single vcpu of an unit.
 * Might be called either if a vcpu of an already running unit is woken up
 * or if a vcpu of a running unit is put asleep with other vcpus of the same
 * unit still running.
 */
static struct vcpu *sched_force_context_switch(struct vcpu *vprev,
                                               struct vcpu *v,
                                               int cpu, s_time_t now)
{
    v->force_context_switch = false;

    if ( vcpu_runnable(v) == v->is_running )
        return NULL;

    if ( vcpu_runnable(v) )
    {
        if ( is_idle_vcpu(vprev) )
        {
            vcpu_runstate_change(vprev, RUNSTATE_runnable, now);
            vprev->sched_unit = get_sched_res(cpu)->sched_unit_idle;
        }
        vcpu_runstate_change(v, RUNSTATE_running, now);
    }
    else
    {
        /* Make sure not to switch last vcpu of an unit away. */
        if ( unit_running(v->sched_unit) == 1 )
            return NULL;

        vcpu_runstate_change(v, vcpu_runstate_blocked(v), now);
        v = sched_unit2vcpu_cpu(vprev->sched_unit, cpu);
        if ( v != vprev )
        {
            if ( is_idle_vcpu(vprev) )
            {
                vcpu_runstate_change(vprev, RUNSTATE_runnable, now);
                vprev->sched_unit = get_sched_res(cpu)->sched_unit_idle;
            }
            else
            {
                v->sched_unit = vprev->sched_unit;
                vcpu_runstate_change(v, RUNSTATE_running, now);
            }
        }
    }

    v->is_running = 1;

    /* Make sure not to loose another slave call. */
    raise_softirq(SCHED_SLAVE_SOFTIRQ);

    return v;
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
static struct sched_unit *sched_wait_rendezvous_in(struct sched_unit *prev,
                                                   spinlock_t *lock, int cpu,
                                                   s_time_t now)
{
    struct sched_unit *next;
    struct vcpu *v;
    unsigned int gran = get_sched_res(cpu)->granularity;

    if ( !--prev->rendezvous_in_cnt )
    {
        next = do_schedule(prev, now, cpu);
        atomic_set(&next->rendezvous_out_cnt, gran + 1);
        return next;
    }

    v = unit2vcpu_cpu(prev, cpu);
    while ( prev->rendezvous_in_cnt )
    {
        if ( v && v->force_context_switch )
        {
            struct vcpu *vprev = current;

            v = sched_force_context_switch(vprev, v, cpu, now);

            if ( v )
            {
                /* We'll come back another time, so adjust rendezvous_in_cnt. */
                prev->rendezvous_in_cnt++;

                pcpu_schedule_unlock_irq(lock, cpu);

                sched_context_switch(vprev, v, false, now);
            }

            v = unit2vcpu_cpu(prev, cpu);
        }

        pcpu_schedule_unlock_irq(lock, cpu);

        /* Coming from idle might need to do tasklet work. */
        if ( is_idle_unit(prev) && sched_tasklet_check_cpu(cpu) )
            do_tasklet();
        else
            cpu_relax();

        pcpu_schedule_lock_irq(cpu);
    }

    return prev->next_task;
}

static void sched_slave(void)
{
    struct vcpu          *v, *vprev = current;
    struct sched_unit    *prev = vprev->sched_unit, *next;
    s_time_t              now;
    spinlock_t           *lock;
    bool                  do_softirq = false;
    int cpu = smp_processor_id();

    ASSERT_NOT_IN_ATOMIC();

    lock = pcpu_schedule_lock_irq(cpu);

    now = NOW();

    v = unit2vcpu_cpu(prev, cpu);
    if ( v && v->force_context_switch )
    {
        v = sched_force_context_switch(vprev, v, cpu, now);

        if ( v )
        {
            pcpu_schedule_unlock_irq(lock, cpu);

            sched_context_switch(vprev, v, false, now);
        }

        do_softirq = true;
    }

    if ( !prev->rendezvous_in_cnt )
    {
        pcpu_schedule_unlock_irq(lock, cpu);

        /* Check for failed forced context switch. */
        if ( do_softirq )
            raise_softirq(SCHEDULE_SOFTIRQ);

        return;
    }

    stop_timer(&get_sched_res(cpu)->s_timer);

    next = sched_wait_rendezvous_in(prev, lock, cpu, now);

    pcpu_schedule_unlock_irq(lock, cpu);

    sched_context_switch(vprev, sched_unit2vcpu_cpu(next, cpu),
                         is_idle_unit(next) && !is_idle_unit(prev), now);
}

/*
 * The main function
 * - deschedule the current domain (scheduler independent).
 * - pick a new domain (scheduler dependent).
 */
static void schedule(void)
{
    struct vcpu          *vnext, *vprev = current;
    struct sched_unit    *prev = vprev->sched_unit, *next = NULL;
    s_time_t              now;
    struct sched_resource *sd;
    spinlock_t           *lock;
    int cpu = smp_processor_id();
    unsigned int          gran = get_sched_res(cpu)->granularity;

    ASSERT_NOT_IN_ATOMIC();

    SCHED_STAT_CRANK(sched_run);

    sd = get_sched_res(cpu);

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

    if ( gran > 1 )
    {
        cpumask_t mask;

        prev->rendezvous_in_cnt = gran;
        cpumask_andnot(&mask, sd->cpus, cpumask_of(cpu));
        cpumask_raise_softirq(&mask, SCHED_SLAVE_SOFTIRQ);
        next = sched_wait_rendezvous_in(prev, lock, cpu, now);
    }
    else
    {
        prev->rendezvous_in_cnt = 0;
        next = do_schedule(prev, now, cpu);
        atomic_set(&next->rendezvous_out_cnt, 0);
    }

    pcpu_schedule_unlock_irq(lock, cpu);

    vnext = sched_unit2vcpu_cpu(next, cpu);
    sched_context_switch(vprev, vnext,
                         !is_idle_unit(prev) && is_idle_unit(next), now);
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
    set_sched_res(cpu, sd);

    sd->scheduler = &ops;
    spin_lock_init(&sd->_lock);
    sd->schedule_lock = &sched_free_cpu_lock;
    init_timer(&sd->s_timer, s_timer_fn, NULL, cpu);
    atomic_set(&sd->urgent_count, 0);

    /* We start with cpu granularity. */
    sd->granularity = 1;

    /* Boot CPU is dealt with later in schedule_init(). */
    if ( cpu == 0 )
        return 0;

    if ( idle_vcpu[cpu] == NULL )
        vcpu_create(idle_vcpu[0]->domain, cpu);
    else
    {
        struct vcpu *idle = idle_vcpu[cpu];
        struct sched_unit *unit = idle->sched_unit;

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
        ASSERT(unit->priv == NULL);

        unit->priv = sched_alloc_vdata(&ops, unit, idle->domain->sched_priv);
        if ( unit->priv == NULL )
            return -ENOMEM;

        /* Update the resource pointer in the idle unit. */
        unit->res = sd;
    }
    if ( idle_vcpu[cpu] == NULL )
        return -ENOMEM;

    sd->curr = idle_vcpu[cpu]->sched_unit;
    sd->sched_unit_idle = idle_vcpu[cpu]->sched_unit;

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
    struct sched_resource *sd = get_sched_res(cpu);
    struct scheduler *sched = sd->scheduler;

    sched_free_pdata(sched, sd->sched_priv, cpu);
    sched_free_vdata(sched, idle_vcpu[cpu]->sched_unit->priv);

    idle_vcpu[cpu]->sched_unit->priv = NULL;
    sd->sched_priv = NULL;

    kill_timer(&sd->s_timer);

    set_sched_res(cpu, NULL);
    xfree(sd);
}

void scheduler_percpu_init(unsigned int cpu)
{
    struct sched_resource *sd = get_sched_res(cpu);
    struct scheduler *sched = sd->scheduler;

    if ( system_state != SYS_STATE_resume )
        sched_init_pdata(sched, sd->sched_priv, cpu);
}

void sched_rm_cpu(unsigned int cpu)
{
    int rc;
    struct sched_resource *sd = get_sched_res(cpu);
    struct scheduler *sched = sd->scheduler;

    rcu_read_lock(&domlist_read_lock);
    rc = cpu_disable_scheduler(cpu);
    BUG_ON(rc);
    rcu_read_unlock(&domlist_read_lock);
    sched_deinit_pdata(sched, sd->sched_priv, cpu);
    cpu_schedule_down(cpu);
}

static int cpu_schedule_callback(
    struct notifier_block *nfb, unsigned long action, void *hcpu)
{
    unsigned int cpu = (unsigned long)hcpu;
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
     * during CPU_UP_PREPARE. init_pdata (if present) is called before
     * CPU_STARTING in scheduler_percpu_init().
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
    case CPU_UP_PREPARE:
        if ( system_state != SYS_STATE_resume )
            rc = cpu_schedule_up(cpu);
        break;
    case CPU_DOWN_PREPARE:
        rcu_read_lock(&domlist_read_lock);
        rc = cpu_disable_scheduler_check(cpu);
        rcu_read_unlock(&domlist_read_lock);
        break;
    case CPU_DEAD:
        if ( system_state == SYS_STATE_suspend )
            break;
        sched_rm_cpu(cpu);
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
#define sched_test_func(f)                               \
        if ( !schedulers[i]->f )                         \
        {                                                \
            printk("scheduler %s misses .%s, dropped\n", \
                   schedulers[i]->opt_name, #f);         \
            schedulers[i] = NULL;                        \
        }

        sched_test_func(init);
        sched_test_func(deinit);
        sched_test_func(pick_resource);
        sched_test_func(alloc_vdata);
        sched_test_func(free_vdata);
        sched_test_func(switch_sched);
        sched_test_func(do_schedule);

#undef sched_test_func

        if ( schedulers[i]->global_init && schedulers[i]->global_init() < 0 )
        {
            printk("scheduler %s failed initialization, dropped\n",
                   schedulers[i]->opt_name);
            schedulers[i] = NULL;
        }

        if ( schedulers[i] && !ops.name &&
             !strcmp(schedulers[i]->opt_name, opt_sched) )
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
    get_sched_res(0)->curr = idle_vcpu[0]->sched_unit;
    get_sched_res(0)->sched_unit_idle = idle_vcpu[0]->sched_unit;
    get_sched_res(0)->sched_priv = sched_alloc_pdata(&ops, 0);
    BUG_ON(IS_ERR(get_sched_res(0)->sched_priv));
    scheduler_percpu_init(0);
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
    struct scheduler *old_ops = get_sched_res(cpu)->scheduler;
    struct scheduler *new_ops = (c == NULL) ? &ops : c->sched;
    struct sched_resource *sd = get_sched_res(cpu);
    struct cpupool *old_pool = sd->cpupool;
    spinlock_t *old_lock, *new_lock;
    unsigned long flags;

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
    vpriv = sched_alloc_vdata(new_ops, idle->sched_unit,
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
    old_lock = pcpu_schedule_lock_irqsave(cpu, &flags);

    vpriv_old = idle->sched_unit->priv;
    ppriv_old = sd->sched_priv;
    new_lock = sched_switch_sched(new_ops, cpu, ppriv, vpriv);

    sd->scheduler = new_ops;
    sd->sched_priv = ppriv;

    /*
     * (Re?)route the lock to the per pCPU lock as /last/ thing. In fact,
     * if it is free (and it can be) we want that anyone that manages
     * taking it, finds all the initializations we've done above in place.
     */
    smp_mb();
    sd->schedule_lock = c ? new_lock : &sched_free_cpu_lock;

    /* _Not_ pcpu_schedule_unlock(): schedule_lock may have changed! */
    spin_unlock_irqrestore(old_lock, flags);

    sched_do_tick_resume(new_ops, cpu);

    sched_deinit_pdata(old_ops, ppriv_old, cpu);

    sched_free_vdata(old_ops, vpriv_old);
    sched_free_pdata(old_ops, ppriv_old, cpu);

 out:
    get_sched_res(cpu)->granularity = c ? c->granularity : 1;
    get_sched_res(cpu)->cpupool = c;
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

    sched = get_sched_res(cpu)->scheduler;
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
    sched = get_sched_res(cpu)->scheduler;
    sched_do_tick_resume(sched, cpu);
}

void wait(void)
{
    schedule();
}

/*
 * vcpu is urgent if vcpu is polling event channel
 *
 * if urgent vcpu exists, CPU should not enter deep C state
 */
int sched_has_urgent_vcpu(void)
{
    return atomic_read(&get_sched_res(smp_processor_id())->urgent_count);
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
