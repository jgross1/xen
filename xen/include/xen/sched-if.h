/******************************************************************************
 * Additional declarations for the generic scheduler interface.  This should
 * only be included by files that implement conforming schedulers.
 *
 * Portions by Mark Williamson are (C) 2004 Intel Research Cambridge
 */

#ifndef __XEN_SCHED_IF_H__
#define __XEN_SCHED_IF_H__

#include <xen/percpu.h>
#include <xen/err.h>

/* A global pointer to the initial cpupool (POOL0). */
extern struct cpupool *cpupool0;

/* cpus currently in no cpupool */
extern cpumask_t cpupool_free_cpus;

/* Scheduler generic parameters
 * */
#define SCHED_DEFAULT_RATELIMIT_US 1000
extern int sched_ratelimit_us;

/* Scheduling resource mask. */
extern const cpumask_t *sched_res_mask;

/*
 * In order to allow a scheduler to remap the lock->cpu mapping,
 * we have a per-cpu pointer, along with a pre-allocated set of
 * locks.  The generic schedule init code will point each schedule lock
 * pointer to the schedule lock; if the scheduler wants to remap them,
 * it can simply modify the schedule locks.
 * 
 * For cache betterness, keep the actual lock in the same cache area
 * as the rest of the struct.  Just have the scheduler point to the
 * one it wants (This may be the one right in front of it).*/
struct sched_resource {
    spinlock_t         *schedule_lock,
                       _lock;
    struct sched_item  *curr;           /* current task                    */
    void               *sched_priv;
    struct timer        s_timer;        /* scheduling timer                */
    atomic_t            urgent_count;   /* how many urgent vcpus           */
    unsigned            processor;
    const cpumask_t    *cpus;           /* cpus covered by this struct     */
};

#define curr_on_cpu(c)    (per_cpu(sched_res, c)->curr)

DECLARE_PER_CPU(struct scheduler *, scheduler);
DECLARE_PER_CPU(struct cpupool *, cpupool);
DECLARE_PER_CPU(struct sched_resource *, sched_res);

struct sched_item {
    struct domain         *domain;
    struct vcpu           *vcpu;
    void                  *priv;      /* scheduler private data */
    struct sched_item     *next_in_list;
    struct sched_resource *res;
    int                    item_id;

    /* Last time when item has been scheduled out. */
    uint64_t               last_run_time;
    /* Last time item got (de-)scheduled. */
    uint64_t               state_entry_time;

    /* Vcpu state summary. */
    unsigned int           run_cnt;   /* vcpus running or runnable */
    unsigned int           idle_cnt;  /* vcpus blocked or offline */

    /* Currently running on a CPU? */
    bool                   is_running;
    /* Item needs affinity restored */
    bool                   affinity_broken;
    /* Does soft affinity actually play a role (given hard affinity)? */
    bool                   soft_aff_effective;
    /* Item has been migrated to other cpu(s). */
    bool                   migrated;
    /* Bitmask of CPUs on which this VCPU may run. */
    cpumask_var_t          cpu_hard_affinity;
    /* Used to change affinity temporarily. */
    cpumask_var_t          cpu_hard_affinity_tmp;
    /* Used to restore affinity across S3. */
    cpumask_var_t          cpu_hard_affinity_saved;
    /* Bitmask of CPUs on which this VCPU prefers to run. */
    cpumask_var_t          cpu_soft_affinity;

    /* Next item to run. */
    struct sched_item      *next_task;
    s_time_t                next_time;

    /* Number of vcpus not yet joined for context switch. */
    unsigned int            rendezvous_in_cnt;

    /* Number of vcpus not yet finished with context switch. */
    atomic_t                rendezvous_out_cnt;
};

#define for_each_sched_item(d, e)                                         \
    for ( (e) = (d)->sched_item_list; (e) != NULL; (e) = (e)->next_in_list )

#define for_each_sched_item_vcpu(i, v)                                    \
    for ( (v) = (i)->vcpu; (v) != NULL && (v)->sched_item == (i);         \
          (v) = (v)->next_in_list )

static inline bool is_idle_item(const struct sched_item *item)
{
    return is_idle_vcpu(item->vcpu);
}

static inline bool item_runnable(const struct sched_item *item)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        if ( vcpu_runnable(v) )
            return true;

    return false;
}

static inline void sched_set_res(struct sched_item *item,
                                 struct sched_resource *res)
{
    int cpu = cpumask_first(res->cpus);
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
    {
        ASSERT(cpu < nr_cpu_ids);
        v->processor = cpu;
        cpu = cpumask_next(cpu, res->cpus);
    }

    item->res = res;
}

static inline unsigned int sched_item_cpu(struct sched_item *item)
{
    return item->res->processor;
}

static inline void sched_set_pause_flags(struct sched_item *item,
                                         unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        __set_bit(bit, &v->pause_flags);
}

static inline void sched_clear_pause_flags(struct sched_item *item,
                                           unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        __clear_bit(bit, &v->pause_flags);
}

static inline void sched_set_pause_flags_atomic(struct sched_item *item,
                                                unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        set_bit(bit, &v->pause_flags);
}

static inline void sched_clear_pause_flags_atomic(struct sched_item *item,
                                                  unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        clear_bit(bit, &v->pause_flags);
}

static inline struct sched_item *sched_idle_item(unsigned int cpu)
{
    return idle_vcpu[cpu]->sched_item;
}

static inline bool vcpu_running(struct vcpu *v)
{
    return v->sched_item->is_running;
}

static inline unsigned int sched_get_resource_cpu(unsigned int cpu)
{
    return per_cpu(sched_res, cpu)->processor;
}

void sched_vcpu_idle(struct vcpu *v);
void guest_idle_loop(void);

/*
 * Scratch space, for avoiding having too many cpumask_t on the stack.
 * Within each scheduler, when using the scratch mask of one pCPU:
 * - the pCPU must belong to the scheduler,
 * - the caller must own the per-pCPU scheduler lock (a.k.a. runqueue
 *   lock).
 */
DECLARE_PER_CPU(cpumask_t, cpumask_scratch);
#define cpumask_scratch        (&this_cpu(cpumask_scratch))
#define cpumask_scratch_cpu(c) (&per_cpu(cpumask_scratch, c))

#define sched_lock(kind, param, cpu, irq, arg...) \
static inline spinlock_t *kind##_schedule_lock##irq(param EXTRA_TYPE(arg)) \
{ \
    for ( ; ; ) \
    { \
        spinlock_t *lock = per_cpu(sched_res, cpu)->schedule_lock; \
        /* \
         * v->processor may change when grabbing the lock; but \
         * per_cpu(v->processor) may also change, if changing cpu pool \
         * also changes the scheduler lock.  Retry until they match. \
         * \
         * It may also be the case that v->processor may change but the \
         * lock may be the same; this will succeed in that case. \
         */ \
        spin_lock##irq(lock, ## arg); \
        if ( likely(lock == per_cpu(sched_res, cpu)->schedule_lock) ) \
            return lock; \
        spin_unlock##irq(lock, ## arg); \
    } \
}

#define sched_unlock(kind, param, cpu, irq, arg...) \
static inline void kind##_schedule_unlock##irq(spinlock_t *lock \
                                               EXTRA_TYPE(arg), param) \
{ \
    ASSERT(lock == per_cpu(sched_res, cpu)->schedule_lock); \
    spin_unlock##irq(lock, ## arg); \
}

#define EXTRA_TYPE(arg)
sched_lock(pcpu, unsigned int cpu,     cpu, )
sched_lock(item, const struct sched_item *i, i->res->processor, )
sched_lock(pcpu, unsigned int cpu,     cpu,          _irq)
sched_lock(item, const struct sched_item *i, i->res->processor, _irq)
sched_unlock(pcpu, unsigned int cpu,     cpu, )
sched_unlock(item, const struct sched_item *i, i->res->processor, )
sched_unlock(pcpu, unsigned int cpu,     cpu,          _irq)
sched_unlock(item, const struct sched_item *i, i->res->processor, _irq)
#undef EXTRA_TYPE

#define EXTRA_TYPE(arg) , unsigned long arg
#define spin_unlock_irqsave spin_unlock_irqrestore
sched_lock(pcpu, unsigned int cpu,     cpu,          _irqsave, *flags)
sched_lock(item, const struct sched_item *i, i->res->processor, _irqsave, *flags)
#undef spin_unlock_irqsave
sched_unlock(pcpu, unsigned int cpu,     cpu,          _irqrestore, flags)
sched_unlock(item, const struct sched_item *i, i->res->processor, _irqrestore, flags)
#undef EXTRA_TYPE

#undef sched_unlock
#undef sched_lock

static inline spinlock_t *pcpu_schedule_trylock(unsigned int cpu)
{
    spinlock_t *lock = per_cpu(sched_res, cpu)->schedule_lock;

    if ( !spin_trylock(lock) )
        return NULL;
    if ( lock == per_cpu(sched_res, cpu)->schedule_lock )
        return lock;
    spin_unlock(lock);
    return NULL;
}

struct scheduler {
    char *name;             /* full name for this scheduler      */
    char *opt_name;         /* option name for this scheduler    */
    unsigned int sched_id;  /* ID for this scheduler             */
    void *sched_data;       /* global data pointer               */

    int          (*global_init)    (void);

    int          (*init)           (struct scheduler *);
    void         (*deinit)         (struct scheduler *);

    void         (*free_vdata)     (const struct scheduler *, void *);
    void *       (*alloc_vdata)    (const struct scheduler *,
                                    struct sched_item *, void *);
    void         (*free_pdata)     (const struct scheduler *, void *, int);
    void *       (*alloc_pdata)    (const struct scheduler *, int);
    void         (*init_pdata)     (const struct scheduler *, void *, int);
    void         (*deinit_pdata)   (const struct scheduler *, void *, int);

    /* Returns ERR_PTR(-err) for error, NULL for 'nothing needed'. */
    void *       (*alloc_domdata)  (const struct scheduler *, struct domain *);
    /* Idempotent. */
    void         (*free_domdata)   (const struct scheduler *, void *);

    void         (*switch_sched)   (struct scheduler *, unsigned int,
                                    void *, void *);

    /* Activate / deactivate items in a cpu pool */
    void         (*insert_item)    (const struct scheduler *,
                                    struct sched_item *);
    void         (*remove_item)    (const struct scheduler *,
                                    struct sched_item *);

    void         (*sleep)          (const struct scheduler *,
                                    struct sched_item *);
    void         (*wake)           (const struct scheduler *,
                                    struct sched_item *);
    void         (*yield)          (const struct scheduler *,
                                    struct sched_item *);
    void         (*context_saved)  (const struct scheduler *,
                                    struct sched_item *);

    void         (*do_schedule)    (const struct scheduler *,
                                    struct sched_item *, s_time_t,
                                    bool tasklet_work_scheduled);

    struct sched_resource * (*pick_resource) (const struct scheduler *,
                                              struct sched_item *);
    void         (*migrate)        (const struct scheduler *,
                                    struct sched_item *, unsigned int);
    int          (*adjust)         (const struct scheduler *, struct domain *,
                                    struct xen_domctl_scheduler_op *);
    void         (*adjust_affinity)(const struct scheduler *,
                                    struct sched_item *,
                                    const struct cpumask *,
                                    const struct cpumask *);
    int          (*adjust_global)  (const struct scheduler *,
                                    struct xen_sysctl_scheduler_op *);
    void         (*dump_settings)  (const struct scheduler *);
    void         (*dump_cpu_state) (const struct scheduler *, int);

    void         (*tick_suspend)    (const struct scheduler *, unsigned int);
    void         (*tick_resume)     (const struct scheduler *, unsigned int);
};

static inline void *sched_alloc_domdata(const struct scheduler *s,
                                        struct domain *d)
{
    if ( s->alloc_domdata )
        return s->alloc_domdata(s, d);
    else
        return NULL;
}

static inline void sched_free_domdata(const struct scheduler *s,
                                      void *data)
{
    if ( s->free_domdata )
        s->free_domdata(s, data);
    else
        /*
         * Check that if there isn't a free_domdata hook, we haven't got any
         * data we're expected to deal with.
         */
        ASSERT(!data);
}

static inline void sched_item_pause_nosync(struct sched_item *item)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        vcpu_pause_nosync(v);
}

static inline void sched_item_unpause(struct sched_item *item)
{
    struct vcpu *v;

    for_each_sched_item_vcpu( item, v )
        vcpu_unpause(v);
}

#define REGISTER_SCHEDULER(x) static const struct scheduler *x##_entry \
  __used_section(".data.schedulers") = &x;

struct cpupool
{
    int              cpupool_id;
    cpumask_var_t    cpu_valid;      /* all cpus assigned to pool */
    cpumask_var_t    res_valid;      /* all scheduling resources of pool */
    cpumask_var_t    cpu_suspended;  /* cpus in S3 that should be in this pool */
    struct cpupool   *next;
    unsigned int     n_dom;
    struct scheduler *sched;
    atomic_t         refcnt;
};

#define cpupool_online_cpumask(_pool) \
    (((_pool) == NULL) ? &cpu_online_map : (_pool)->cpu_valid)

static inline cpumask_t* cpupool_domain_cpumask(struct domain *d)
{
    /*
     * d->cpupool is NULL only for the idle domain, and no one should
     * be interested in calling this for the idle domain.
     */
    ASSERT(d->cpupool != NULL);
    return d->cpupool->res_valid;
}

/*
 * Hard and soft affinity load balancing.
 *
 * Idea is each vcpu has some pcpus that it prefers, some that it does not
 * prefer but is OK with, and some that it cannot run on at all. The first
 * set of pcpus are the ones that are both in the soft affinity *and* in the
 * hard affinity; the second set of pcpus are the ones that are in the hard
 * affinity but *not* in the soft affinity; the third set of pcpus are the
 * ones that are not in the hard affinity.
 *
 * We implement a two step balancing logic. Basically, every time there is
 * the need to decide where to run a vcpu, we first check the soft affinity
 * (well, actually, the && between soft and hard affinity), to see if we can
 * send it where it prefers to (and can) run on. However, if the first step
 * does not find any suitable and free pcpu, we fall back checking the hard
 * affinity.
 */
#define BALANCE_SOFT_AFFINITY    0
#define BALANCE_HARD_AFFINITY    1

#define for_each_affinity_balance_step(step) \
    for ( (step) = 0; (step) <= BALANCE_HARD_AFFINITY; (step)++ )

/*
 * Hard affinity balancing is always necessary and must never be skipped.
 * But soft affinity need only be considered when it has a functionally
 * different effect than other constraints (such as hard affinity, cpus
 * online, or cpupools).
 *
 * Soft affinity only needs to be considered if:
 * * The cpus in the cpupool are not a subset of soft affinity
 * * The hard affinity is not a subset of soft affinity
 * * There is an overlap between the soft and hard affinity masks
 */
static inline int has_soft_affinity(const struct sched_item *item)
{
    return item->soft_aff_effective &&
           !cpumask_subset(cpupool_domain_cpumask(item->vcpu->domain),
                           item->cpu_soft_affinity);
}

/*
 * This function copies in mask the cpumask that should be used for a
 * particular affinity balancing step. For the soft affinity one, the pcpus
 * that are not part of vc's hard affinity are filtered out from the result,
 * to avoid running a vcpu where it would like, but is not allowed to!
 */
static inline void
affinity_balance_cpumask(const struct sched_item *item, int step,
                         cpumask_t *mask)
{
    if ( step == BALANCE_SOFT_AFFINITY )
    {
        cpumask_and(mask, item->cpu_soft_affinity, item->cpu_hard_affinity);

        if ( unlikely(cpumask_empty(mask)) )
            cpumask_copy(mask, item->cpu_hard_affinity);
    }
    else /* step == BALANCE_HARD_AFFINITY */
        cpumask_copy(mask, item->cpu_hard_affinity);
}

#endif /* __XEN_SCHED_IF_H__ */
