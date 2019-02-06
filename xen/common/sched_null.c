/*
 * xen/common/sched_null.c
 *
 *  Copyright (c) 2017, Dario Faggioli, Citrix Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * The 'null' scheduler always choose to run, on each pCPU, either nothing
 * (i.e., the pCPU stays idle) or always the same Item.
 *
 * It is aimed at supporting static scenarios, where there always are
 * less Items than pCPUs (and the Items don't need to move among pCPUs
 * for any reason) with the least possible overhead.
 *
 * Typical usecase are embedded applications, but also HPC, especially
 * if the scheduler is used inside a cpupool.
 */

#include <xen/sched.h>
#include <xen/sched-if.h>
#include <xen/softirq.h>
#include <xen/keyhandler.h>
#include <xen/trace.h>

/*
 * null tracing events. Check include/public/trace.h for more details.
 */
#define TRC_SNULL_PICKED_CPU    TRC_SCHED_CLASS_EVT(SNULL, 1)
#define TRC_SNULL_ITEM_ASSIGN   TRC_SCHED_CLASS_EVT(SNULL, 2)
#define TRC_SNULL_ITEM_DEASSIGN TRC_SCHED_CLASS_EVT(SNULL, 3)
#define TRC_SNULL_MIGRATE       TRC_SCHED_CLASS_EVT(SNULL, 4)
#define TRC_SNULL_SCHEDULE      TRC_SCHED_CLASS_EVT(SNULL, 5)
#define TRC_SNULL_TASKLET       TRC_SCHED_CLASS_EVT(SNULL, 6)

/*
 * Locking:
 * - Scheduler-lock (a.k.a. runqueue lock):
 *  + is per-pCPU;
 *  + serializes assignment and deassignment of Items to a pCPU.
 * - Private data lock (a.k.a. private scheduler lock):
 *  + is scheduler-wide;
 *  + serializes accesses to the list of domains in this scheduler.
 * - Waitqueue lock:
 *  + is scheduler-wide;
 *  + serialize accesses to the list of Items waiting to be assigned
 *    to pCPUs.
 *
 * Ordering is: private lock, runqueue lock, waitqueue lock. Or, OTOH,
 * waitqueue lock nests inside runqueue lock which nests inside private
 * lock. More specifically:
 *  + if we need both runqueue and private locks, we must acquire the
 *    private lock for first;
 *  + if we need both runqueue and waitqueue locks, we must acquire
 *    the runqueue lock for first;
 *  + if we need both private and waitqueue locks, we must acquire
 *    the private lock for first;
 *  + if we already own a runqueue lock, we must never acquire
 *    the private lock;
 *  + if we already own the waitqueue lock, we must never acquire
 *    the runqueue lock or the private lock.
 */

/*
 * System-wide private data
 */
struct null_private {
    spinlock_t lock;        /* scheduler lock; nests inside cpupool_lock */
    struct list_head ndom;  /* Domains of this scheduler                 */
    struct list_head waitq; /* Items not assigned to any pCPU            */
    spinlock_t waitq_lock;  /* serializes waitq; nests inside runq locks */
    cpumask_t cpus_free;    /* CPUs without a Item associated to them    */
};

/*
 * Physical CPU
 */
struct null_pcpu {
    struct sched_item *item;
};
DEFINE_PER_CPU(struct null_pcpu, npc);

/*
 * Schedule Item
 */
struct null_item {
    struct list_head waitq_elem;
    struct sched_item *item;
};

/*
 * Domain
 */
struct null_dom {
    struct list_head ndom_elem;
    struct domain *dom;
};

/*
 * Accessor helpers functions
 */
static inline struct null_private *null_priv(const struct scheduler *ops)
{
    return ops->sched_data;
}

static inline struct null_item *null_item(const struct sched_item *item)
{
    return item->priv;
}

static inline bool item_check_affinity(struct sched_item *item,
                                       unsigned int cpu,
                                       unsigned int balance_step)
{
    affinity_balance_cpumask(item, balance_step, cpumask_scratch_cpu(cpu));
    cpumask_and(cpumask_scratch_cpu(cpu), cpumask_scratch_cpu(cpu),
                cpupool_domain_cpumask(item->domain));

    return cpumask_test_cpu(cpu, cpumask_scratch_cpu(cpu));
}

static int null_init(struct scheduler *ops)
{
    struct null_private *prv;

    printk("Initializing null scheduler\n"
           "WARNING: This is experimental software in development.\n"
           "Use at your own risk.\n");

    prv = xzalloc(struct null_private);
    if ( prv == NULL )
        return -ENOMEM;

    spin_lock_init(&prv->lock);
    spin_lock_init(&prv->waitq_lock);
    INIT_LIST_HEAD(&prv->ndom);
    INIT_LIST_HEAD(&prv->waitq);

    ops->sched_data = prv;

    return 0;
}

static void null_deinit(struct scheduler *ops)
{
    xfree(ops->sched_data);
    ops->sched_data = NULL;
}

static void init_pdata(struct null_private *prv, unsigned int cpu)
{
    /* Mark the pCPU as free, and with no item assigned */
    cpumask_set_cpu(cpu, &prv->cpus_free);
    per_cpu(npc, cpu).item = NULL;
}

static void null_init_pdata(const struct scheduler *ops, void *pdata, int cpu)
{
    struct null_private *prv = null_priv(ops);
    struct sched_resource *sd = per_cpu(sched_res, cpu);

    /* alloc_pdata is not implemented, so we want this to be NULL. */
    ASSERT(!pdata);

    /*
     * The scheduler lock points already to the default per-cpu spinlock,
     * so there is no remapping to be done.
     */
    ASSERT(sd->schedule_lock == &sd->_lock && !spin_is_locked(&sd->_lock));

    init_pdata(prv, cpu);
}

static void null_deinit_pdata(const struct scheduler *ops, void *pcpu, int cpu)
{
    struct null_private *prv = null_priv(ops);

    /* alloc_pdata not implemented, so this must have stayed NULL */
    ASSERT(!pcpu);

    cpumask_clear_cpu(cpu, &prv->cpus_free);
    per_cpu(npc, cpu).item = NULL;
}

static void *null_alloc_vdata(const struct scheduler *ops,
                              struct sched_item *item, void *dd)
{
    struct null_item *nvc;

    nvc = xzalloc(struct null_item);
    if ( nvc == NULL )
        return NULL;

    INIT_LIST_HEAD(&nvc->waitq_elem);
    nvc->item = item;

    SCHED_STAT_CRANK(item_alloc);

    return nvc;
}

static void null_free_vdata(const struct scheduler *ops, void *priv)
{
    struct null_item *nvc = priv;

    xfree(nvc);
}

static void * null_alloc_domdata(const struct scheduler *ops,
                                 struct domain *d)
{
    struct null_private *prv = null_priv(ops);
    struct null_dom *ndom;
    unsigned long flags;

    ndom = xzalloc(struct null_dom);
    if ( ndom == NULL )
        return ERR_PTR(-ENOMEM);

    ndom->dom = d;

    spin_lock_irqsave(&prv->lock, flags);
    list_add_tail(&ndom->ndom_elem, &null_priv(ops)->ndom);
    spin_unlock_irqrestore(&prv->lock, flags);

    return ndom;
}

static void null_free_domdata(const struct scheduler *ops, void *data)
{
    struct null_dom *ndom = data;
    struct null_private *prv = null_priv(ops);

    if ( ndom )
    {
        unsigned long flags;

        spin_lock_irqsave(&prv->lock, flags);
        list_del_init(&ndom->ndom_elem);
        spin_unlock_irqrestore(&prv->lock, flags);

        xfree(ndom);
    }
}

/*
 * item to pCPU assignment and placement. This _only_ happens:
 *  - on insert,
 *  - on migrate.
 *
 * Insert occurs when a item joins this scheduler for the first time
 * (e.g., when the domain it's part of is moved to the scheduler's
 * cpupool).
 *
 * Migration may be necessary if a pCPU (with a item assigned to it)
 * is removed from the scheduler's cpupool.
 *
 * So this is not part of any hot path.
 */
static struct sched_resource *
pick_res(struct null_private *prv, struct sched_item *item)
{
    unsigned int bs;
    unsigned int cpu = sched_item_cpu(item), new_cpu;
    cpumask_t *cpus = cpupool_domain_cpumask(item->domain);

    ASSERT(spin_is_locked(per_cpu(sched_res, cpu)->schedule_lock));

    for_each_affinity_balance_step( bs )
    {
        if ( bs == BALANCE_SOFT_AFFINITY && !has_soft_affinity(item) )
            continue;

        affinity_balance_cpumask(item, bs, cpumask_scratch_cpu(cpu));
        cpumask_and(cpumask_scratch_cpu(cpu), cpumask_scratch_cpu(cpu), cpus);

        /*
         * If our processor is free, or we are assigned to it, and it is also
         * still valid and part of our affinity, just go for it.
         * (Note that we may call item_check_affinity(), but we deliberately
         * don't, so we get to keep in the scratch cpumask what we have just
         * put in it.)
         */
        if ( likely((per_cpu(npc, cpu).item == NULL ||
                     per_cpu(npc, cpu).item == item)
                    && cpumask_test_cpu(cpu, cpumask_scratch_cpu(cpu))) )
        {
            new_cpu = cpu;
            goto out;
        }

        /* If not, just go for a free pCPU, within our affinity, if any */
        cpumask_and(cpumask_scratch_cpu(cpu), cpumask_scratch_cpu(cpu),
                    &prv->cpus_free);
        new_cpu = cpumask_first(cpumask_scratch_cpu(cpu));

        if ( likely(new_cpu != nr_cpu_ids) )
            goto out;
    }

    /*
     * If we didn't find any free pCPU, just pick any valid pcpu, even if
     * it has another Item assigned. This will happen during shutdown and
     * suspend/resume, but it may also happen during "normal operation", if
     * all the pCPUs are busy.
     *
     * In fact, there must always be something sane in v->processor, or
     * item_schedule_lock() and friends won't work. This is not a problem,
     * as we will actually assign the Item to the pCPU we return from here,
     * only if the pCPU is free.
     */
    cpumask_and(cpumask_scratch_cpu(cpu), cpus, item->cpu_hard_affinity);
    new_cpu = cpumask_any(cpumask_scratch_cpu(cpu));

 out:
    if ( unlikely(tb_init_done) )
    {
        struct {
            uint16_t item, dom;
            uint32_t new_cpu;
        } d;
        d.dom = item->domain->domain_id;
        d.item = item->item_id;
        d.new_cpu = new_cpu;
        __trace_var(TRC_SNULL_PICKED_CPU, 1, sizeof(d), &d);
    }

    return per_cpu(sched_res, new_cpu);
}

static void item_assign(struct null_private *prv, struct sched_item *item,
                        unsigned int cpu)
{
    per_cpu(npc, cpu).item = item;
    sched_set_res(item, per_cpu(sched_res, cpu));
    cpumask_clear_cpu(cpu, &prv->cpus_free);

    dprintk(XENLOG_G_INFO, "%d <-- %pdv%d\n", cpu, item->domain, item->item_id);

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint16_t item, dom;
            uint32_t cpu;
        } d;
        d.dom = item->domain->domain_id;
        d.item = item->item_id;
        d.cpu = cpu;
        __trace_var(TRC_SNULL_ITEM_ASSIGN, 1, sizeof(d), &d);
    }
}

static void item_deassign(struct null_private *prv, struct sched_item *item,
                          unsigned int cpu)
{
    per_cpu(npc, cpu).item = NULL;
    cpumask_set_cpu(cpu, &prv->cpus_free);

    dprintk(XENLOG_G_INFO, "%d <-- NULL (%pdv%d)\n", cpu, item->domain,
            item->item_id);

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint16_t item, dom;
            uint32_t cpu;
        } d;
        d.dom = item->domain->domain_id;
        d.item = item->item_id;
        d.cpu = cpu;
        __trace_var(TRC_SNULL_ITEM_DEASSIGN, 1, sizeof(d), &d);
    }
}

/* Change the scheduler of cpu to us (null). */
static void null_switch_sched(struct scheduler *new_ops, unsigned int cpu,
                              void *pdata, void *vdata)
{
    struct sched_resource *sd = per_cpu(sched_res, cpu);
    struct null_private *prv = null_priv(new_ops);
    struct null_item *nvc = vdata;

    ASSERT(nvc && is_idle_item(nvc->item));

    sched_idle_item(cpu)->priv = vdata;

    /*
     * We are holding the runqueue lock already (it's been taken in
     * schedule_cpu_switch()). It actually may or may not be the 'right'
     * one for this cpu, but that is ok for preventing races.
     */
    ASSERT(!local_irq_is_enabled());

    init_pdata(prv, cpu);

    per_cpu(scheduler, cpu) = new_ops;
    per_cpu(sched_res, cpu)->sched_priv = pdata;

    /*
     * (Re?)route the lock to the per pCPU lock as /last/ thing. In fact,
     * if it is free (and it can be) we want that anyone that manages
     * taking it, finds all the initializations we've done above in place.
     */
    smp_mb();
    sd->schedule_lock = &sd->_lock;
}

static void null_item_insert(const struct scheduler *ops,
                             struct sched_item *item)
{
    struct null_private *prv = null_priv(ops);
    struct null_item *nvc = null_item(item);
    unsigned int cpu;
    spinlock_t *lock;

    ASSERT(!is_idle_item(item));

    lock = item_schedule_lock_irq(item);
 retry:

    sched_set_res(item, pick_res(prv, item));
    cpu = sched_item_cpu(item);

    spin_unlock(lock);

    lock = item_schedule_lock(item);

    cpumask_and(cpumask_scratch_cpu(cpu), item->cpu_hard_affinity,
                cpupool_domain_cpumask(item->domain));

    /* If the pCPU is free, we assign item to it */
    if ( likely(per_cpu(npc, cpu).item == NULL) )
    {
        /*
         * Insert is followed by vcpu_wake(), so there's no need to poke
         * the pcpu with the SCHEDULE_SOFTIRQ, as wake will do that.
         */
        item_assign(prv, item, cpu);
    }
    else if ( cpumask_intersects(&prv->cpus_free, cpumask_scratch_cpu(cpu)) )
    {
        /*
         * If the pCPU is not free (e.g., because we raced with another
         * insert or a migrate), but there are other free pCPUs, we can
         * try to pick again.
         */
         goto retry;
    }
    else
    {
        /*
         * If the pCPU is not free, and there aren't any (valid) others,
         * we have no alternatives than to go into the waitqueue.
         */
        spin_lock(&prv->waitq_lock);
        list_add_tail(&nvc->waitq_elem, &prv->waitq);
        dprintk(XENLOG_G_WARNING, "WARNING: %pdv%d not assigned to any CPU!\n",
                item->domain, item->item_id);
        spin_unlock(&prv->waitq_lock);
    }
    spin_unlock_irq(lock);

    SCHED_STAT_CRANK(item_insert);
}

static void _item_remove(struct null_private *prv, struct sched_item *item)
{
    unsigned int bs;
    unsigned int cpu = sched_item_cpu(item);
    struct null_item *wvc;

    ASSERT(list_empty(&null_item(item)->waitq_elem));

    item_deassign(prv, item, cpu);

    spin_lock(&prv->waitq_lock);

    /*
     * If item is assigned to a pCPU, let's see if there is someone waiting,
     * suitable to be assigned to it (prioritizing items that have
     * soft-affinity with cpu).
     */
    for_each_affinity_balance_step( bs )
    {
        list_for_each_entry( wvc, &prv->waitq, waitq_elem )
        {
            if ( bs == BALANCE_SOFT_AFFINITY && !has_soft_affinity(wvc->item) )
                continue;

            if ( item_check_affinity(wvc->item, cpu, bs) )
            {
                list_del_init(&wvc->waitq_elem);
                item_assign(prv, wvc->item, cpu);
                cpu_raise_softirq(cpu, SCHEDULE_SOFTIRQ);
                spin_unlock(&prv->waitq_lock);
                return;
            }
        }
    }
    spin_unlock(&prv->waitq_lock);
}

static void null_item_remove(const struct scheduler *ops,
                             struct sched_item *item)
{
    struct null_private *prv = null_priv(ops);
    struct null_item *nvc = null_item(item);
    spinlock_t *lock;

    ASSERT(!is_idle_item(item));

    lock = item_schedule_lock_irq(item);

    /* If item is in waitqueue, just get it out of there and bail */
    if ( unlikely(!list_empty(&nvc->waitq_elem)) )
    {
        spin_lock(&prv->waitq_lock);
        list_del_init(&nvc->waitq_elem);
        spin_unlock(&prv->waitq_lock);

        goto out;
    }

    ASSERT(per_cpu(npc, sched_item_cpu(item)).item == item);
    ASSERT(!cpumask_test_cpu(sched_item_cpu(item), &prv->cpus_free));

    _item_remove(prv, item);

 out:
    item_schedule_unlock_irq(lock, item);

    SCHED_STAT_CRANK(item_remove);
}

static void null_item_wake(const struct scheduler *ops,
                           struct sched_item *item)
{
    ASSERT(!is_idle_item(item));

    if ( unlikely(curr_on_cpu(sched_item_cpu(item)) == item) )
    {
        SCHED_STAT_CRANK(item_wake_running);
        return;
    }

    if ( unlikely(!list_empty(&null_item(item)->waitq_elem)) )
    {
        /* Not exactly "on runq", but close enough for reusing the counter */
        SCHED_STAT_CRANK(item_wake_onrunq);
        return;
    }

    if ( likely(item_runnable(item)) )
        SCHED_STAT_CRANK(item_wake_runnable);
    else
        SCHED_STAT_CRANK(item_wake_not_runnable);

    /* Note that we get here only for items assigned to a pCPU */
    cpu_raise_softirq(sched_item_cpu(item), SCHEDULE_SOFTIRQ);
}

static void null_item_sleep(const struct scheduler *ops,
                            struct sched_item *item)
{
    ASSERT(!is_idle_item(item));

    /* If item isn't assigned to a pCPU, or isn't running, no need to bother */
    if ( curr_on_cpu(sched_item_cpu(item)) == item )
        cpu_raise_softirq(sched_item_cpu(item), SCHEDULE_SOFTIRQ);

    SCHED_STAT_CRANK(item_sleep);
}

static struct sched_resource *
null_res_pick(const struct scheduler *ops, struct sched_item *item)
{
    ASSERT(!is_idle_item(item));
    return pick_res(null_priv(ops), item);
}

static void null_item_migrate(const struct scheduler *ops,
                              struct sched_item *item, unsigned int new_cpu)
{
    struct null_private *prv = null_priv(ops);
    struct null_item *nvc = null_item(item);

    ASSERT(!is_idle_item(item));

    if ( sched_item_cpu(item) == new_cpu )
        return;

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint16_t item, dom;
            uint16_t cpu, new_cpu;
        } d;
        d.dom = item->domain->domain_id;
        d.item = item->item_id;
        d.cpu = sched_item_cpu(item);
        d.new_cpu = new_cpu;
        __trace_var(TRC_SNULL_MIGRATE, 1, sizeof(d), &d);
    }

    /*
     * item is either assigned to a pCPU, or in the waitqueue.
     *
     * In the former case, the pCPU to which it was assigned would
     * become free, and we, therefore, should check whether there is
     * anyone in the waitqueue that can be assigned to it.
     *
     * In the latter, there is just nothing to do.
     */
    if ( likely(list_empty(&nvc->waitq_elem)) )
    {
        _item_remove(prv, item);
        SCHED_STAT_CRANK(migrate_running);
    }
    else
        SCHED_STAT_CRANK(migrate_on_runq);

    SCHED_STAT_CRANK(migrated);

    /*
     * Let's now consider new_cpu, which is where item is being sent. It can be
     * either free, or have a item already assigned to it.
     *
     * In the former case we should assign item to it, and try to get it to run,
     * if possible, according to affinity.
     *
     * In latter, all we can do is to park item in the waitqueue.
     */
    if ( per_cpu(npc, new_cpu).item == NULL &&
         item_check_affinity(item, new_cpu, BALANCE_HARD_AFFINITY) )
    {
        /* item might have been in the waitqueue, so remove it */
        spin_lock(&prv->waitq_lock);
        list_del_init(&nvc->waitq_elem);
        spin_unlock(&prv->waitq_lock);

        item_assign(prv, item, new_cpu);
    }
    else
    {
        /* Put item in the waitqueue, if it wasn't there already */
        spin_lock(&prv->waitq_lock);
        if ( list_empty(&nvc->waitq_elem) )
        {
            list_add_tail(&nvc->waitq_elem, &prv->waitq);
            dprintk(XENLOG_G_WARNING,
                    "WARNING: %pdv%d not assigned to any CPU!\n", item->domain,
                    item->item_id);
        }
        spin_unlock(&prv->waitq_lock);
    }

    /*
     * Whatever all the above, we always at least override v->processor.
     * This is especially important for shutdown or suspend/resume paths,
     * when it is important to let our caller (cpu_disable_scheduler())
     * know that the migration did happen, to the best of our possibilities,
     * at least. In case of suspend, any temporary inconsistency caused
     * by this, will be fixed-up during resume.
     */
    sched_set_res(item, per_cpu(sched_res, new_cpu));
}

#ifndef NDEBUG
static inline void null_item_check(struct sched_item *item)
{
    struct null_item * const nvc = null_item(item);
    struct null_dom * const ndom = item->domain->sched_priv;

    BUG_ON(nvc->item != item);

    if ( ndom )
        BUG_ON(is_idle_item(item));
    else
        BUG_ON(!is_idle_item(item));

    SCHED_STAT_CRANK(item_check);
}
#define NULL_ITEM_CHECK(item)  (null_item_check(item))
#else
#define NULL_ITEM_CHECK(item)
#endif


/*
 * The most simple scheduling function of all times! We either return:
 *  - the item assigned to the pCPU, if there's one and it can run;
 *  - the idle item, otherwise.
 */
static void null_schedule(const struct scheduler *ops, struct sched_item *prev,
                          s_time_t now, bool tasklet_work_scheduled)
{
    unsigned int bs;
    const unsigned int cpu = smp_processor_id();
    const unsigned int sched_cpu = sched_get_resource_cpu(cpu);
    struct null_private *prv = null_priv(ops);
    struct null_item *wvc;

    SCHED_STAT_CRANK(schedule);
    NULL_ITEM_CHECK(current->sched_item);

    if ( unlikely(tb_init_done) )
    {
        struct {
            uint16_t tasklet, cpu;
            int16_t item, dom;
        } d;
        d.cpu = cpu;
        d.tasklet = tasklet_work_scheduled;
        if ( per_cpu(npc, sched_cpu).item == NULL )
        {
            d.item = d.dom = -1;
        }
        else
        {
            d.item = per_cpu(npc, sched_cpu).item->item_id;
            d.dom = per_cpu(npc, sched_cpu).item->domain->domain_id;
        }
        __trace_var(TRC_SNULL_SCHEDULE, 1, sizeof(d), &d);
    }

    if ( tasklet_work_scheduled )
    {
        trace_var(TRC_SNULL_TASKLET, 1, 0, NULL);
        prev->next_task = sched_idle_item(sched_cpu);
    }
    else
        prev->next_task = per_cpu(npc, sched_cpu).item;
    prev->next_time = -1;

    /*
     * We may be new in the cpupool, or just coming back online. In which
     * case, there may be items in the waitqueue that we can assign to us
     * and run.
     */
    if ( unlikely(prev->next_task == NULL) )
    {
        spin_lock(&prv->waitq_lock);

        if ( list_empty(&prv->waitq) )
            goto unlock;

        /*
         * We scan the waitqueue twice, for prioritizing items that have
         * soft-affinity with cpu. This may look like something expensive to
         * do here in null_schedule(), but it's actually fine, because we do
         * it only in cases where a pcpu has no item associated (e.g., as
         * said above, the cpu has just joined a cpupool).
         */
        for_each_affinity_balance_step( bs )
        {
            list_for_each_entry( wvc, &prv->waitq, waitq_elem )
            {
                if ( bs == BALANCE_SOFT_AFFINITY &&
                     !has_soft_affinity(wvc->item) )
                    continue;

                if ( item_check_affinity(wvc->item, sched_cpu, bs) )
                {
                    item_assign(prv, wvc->item, sched_cpu);
                    list_del_init(&wvc->waitq_elem);
                    prev->next_task = wvc->item;
                    goto unlock;
                }
            }
        }
 unlock:
        spin_unlock(&prv->waitq_lock);
    }

    if ( unlikely(prev->next_task == NULL || !item_runnable(prev->next_task)) )
        prev->next_task = sched_idle_item(sched_cpu);

    NULL_ITEM_CHECK(prev->next_task);

    prev->next_task->migrated = false;
}

static inline void dump_item(struct null_private *prv, struct null_item *nvc)
{
    printk("[%i.%i] pcpu=%d", nvc->item->domain->domain_id,
            nvc->item->item_id, list_empty(&nvc->waitq_elem) ?
                                sched_item_cpu(nvc->item) : -1);
}

static void null_dump_pcpu(const struct scheduler *ops, int cpu)
{
    struct null_private *prv = null_priv(ops);
    struct null_item *nvc;
    spinlock_t *lock;
    unsigned long flags;

    lock = pcpu_schedule_lock_irqsave(cpu, &flags);

    printk("CPU[%02d] sibling=%*pb, core=%*pb",
           cpu,
           nr_cpu_ids, cpumask_bits(per_cpu(cpu_sibling_mask, cpu)),
           nr_cpu_ids, cpumask_bits(per_cpu(cpu_core_mask, cpu)));
    if ( per_cpu(npc, cpu).item != NULL )
        printk(", item=%pdv%d", per_cpu(npc, cpu).item->domain,
               per_cpu(npc, cpu).item->item_id);
    printk("\n");

    /* current item (nothing to say if that's the idle item) */
    nvc = null_item(curr_on_cpu(cpu));
    if ( nvc && !is_idle_item(nvc->item) )
    {
        printk("\trun: ");
        dump_item(prv, nvc);
        printk("\n");
    }

    pcpu_schedule_unlock_irqrestore(lock, flags, cpu);
}

static void null_dump(const struct scheduler *ops)
{
    struct null_private *prv = null_priv(ops);
    struct list_head *iter;
    unsigned long flags;
    unsigned int loop;

    spin_lock_irqsave(&prv->lock, flags);

    printk("\tcpus_free = %*pbl\n", nr_cpu_ids, cpumask_bits(&prv->cpus_free));

    printk("Domain info:\n");
    loop = 0;
    list_for_each( iter, &prv->ndom )
    {
        struct null_dom *ndom;
        struct sched_item *item;

        ndom = list_entry(iter, struct null_dom, ndom_elem);

        printk("\tDomain: %d\n", ndom->dom->domain_id);
        for_each_sched_item( ndom->dom, item )
        {
            struct null_item * const nvc = null_item(item);
            spinlock_t *lock;

            lock = item_schedule_lock(item);

            printk("\t%3d: ", ++loop);
            dump_item(prv, nvc);
            printk("\n");

            item_schedule_unlock(lock, item);
        }
    }

    printk("Waitqueue: ");
    loop = 0;
    spin_lock(&prv->waitq_lock);
    list_for_each( iter, &prv->waitq )
    {
        struct null_item *nvc = list_entry(iter, struct null_item, waitq_elem);

        if ( loop++ != 0 )
            printk(", ");
        if ( loop % 24 == 0 )
            printk("\n\t");
        printk("%pdv%d", nvc->item->domain, nvc->item->item_id);
    }
    printk("\n");
    spin_unlock(&prv->waitq_lock);

    spin_unlock_irqrestore(&prv->lock, flags);
}

const struct scheduler sched_null_def = {
    .name           = "null Scheduler",
    .opt_name       = "null",
    .sched_id       = XEN_SCHEDULER_NULL,
    .sched_data     = NULL,

    .init           = null_init,
    .deinit         = null_deinit,
    .init_pdata     = null_init_pdata,
    .switch_sched   = null_switch_sched,
    .deinit_pdata   = null_deinit_pdata,

    .alloc_vdata    = null_alloc_vdata,
    .free_vdata     = null_free_vdata,
    .alloc_domdata  = null_alloc_domdata,
    .free_domdata   = null_free_domdata,

    .insert_item    = null_item_insert,
    .remove_item    = null_item_remove,

    .wake           = null_item_wake,
    .sleep          = null_item_sleep,
    .pick_resource  = null_res_pick,
    .migrate        = null_item_migrate,
    .do_schedule    = null_schedule,

    .dump_cpu_state = null_dump_pcpu,
    .dump_settings  = null_dump,
};

REGISTER_SCHEDULER(sched_null_def);
