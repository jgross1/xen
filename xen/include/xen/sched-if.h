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

/* Number of vcpus per struct sched_unit. */
enum sched_gran {
    SCHED_GRAN_cpu,
    SCHED_GRAN_core,
    SCHED_GRAN_socket
};
extern enum sched_gran opt_sched_granularity;
extern unsigned int sched_granularity;

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
    struct scheduler   *scheduler;
    struct cpupool     *cpupool;
    spinlock_t         *schedule_lock,
                       _lock;
    struct sched_unit  *curr;           /* current task                    */
    struct sched_unit  *sched_unit_idle;
    struct sched_unit  *prev;           /* previous task                   */
    void               *sched_priv;
    struct timer        s_timer;        /* scheduling timer                */
    atomic_t            urgent_count;   /* how many urgent vcpus           */
    unsigned int        processor;
    unsigned int        granularity;
    const cpumask_t    *cpus;           /* cpus covered by this struct     */
};

#define curr_on_cpu(c)    (get_sched_res(c)->curr)

DECLARE_PER_CPU(struct sched_resource *, sched_res);

static inline struct sched_resource *get_sched_res(unsigned int cpu)
{
    return per_cpu(sched_res, cpu);
}

static inline void set_sched_res(unsigned int cpu, struct sched_resource *res)
{
    per_cpu(sched_res, cpu) = res;
}

static inline bool is_idle_unit(const struct sched_unit *unit)
{
    return is_idle_vcpu(unit->vcpu);
}

static inline unsigned int unit_running(const struct sched_unit *unit)
{
    return unit->runstate_cnt[RUNSTATE_running];
}

static inline bool unit_runnable(const struct sched_unit *unit)
{
    struct vcpu *v;

    if ( is_idle_unit(unit) )
        return true;

    for_each_sched_unit_vcpu ( unit, v )
        if ( vcpu_runnable(v) )
            return true;

    return false;
}

static inline int vcpu_runstate_blocked(struct vcpu *v)
{
    return (v->pause_flags & VPF_blocked) ? RUNSTATE_blocked : RUNSTATE_offline;
}

static inline bool unit_runnable_state(const struct sched_unit *unit)
{
    struct vcpu *v;
    bool runnable, ret = false;

    if ( is_idle_unit(unit) )
        return true;

    for_each_sched_unit_vcpu ( unit, v )
    {
        runnable = vcpu_runnable(v);

        v->new_state = runnable ? RUNSTATE_running : vcpu_runstate_blocked(v);

        if ( runnable )
            ret = true;
    }

    return ret;
}

static inline void sched_set_res(struct sched_unit *unit,
                                 struct sched_resource *res)
{
    int cpu = cpumask_first(res->cpus);
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
    {
        ASSERT(cpu < nr_cpu_ids);
        v->processor = cpu;
        cpu = cpumask_next(cpu, res->cpus);
    }

    unit->res = res;
}

static inline unsigned int sched_unit_cpu(struct sched_unit *unit)
{
    return unit->res->processor;
}

static inline void sched_set_pause_flags(struct sched_unit *unit,
                                         unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        __set_bit(bit, &v->pause_flags);
}

static inline void sched_clear_pause_flags(struct sched_unit *unit,
                                           unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        __clear_bit(bit, &v->pause_flags);
}

static inline void sched_set_pause_flags_atomic(struct sched_unit *unit,
                                                unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        set_bit(bit, &v->pause_flags);
}

static inline void sched_clear_pause_flags_atomic(struct sched_unit *unit,
                                                  unsigned int bit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        clear_bit(bit, &v->pause_flags);
}

static inline struct sched_unit *sched_idle_unit(unsigned int cpu)
{
    return get_sched_res(cpu)->sched_unit_idle;
}

static inline unsigned int sched_get_resource_cpu(unsigned int cpu)
{
    return get_sched_res(cpu)->processor;
}

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
        spinlock_t *lock = get_sched_res(cpu)->schedule_lock; \
        /* \
         * v->processor may change when grabbing the lock; but \
         * per_cpu(v->processor) may also change, if changing cpu pool \
         * also changes the scheduler lock.  Retry until they match. \
         * \
         * It may also be the case that v->processor may change but the \
         * lock may be the same; this will succeed in that case. \
         */ \
        spin_lock##irq(lock, ## arg); \
        if ( likely(lock == get_sched_res(cpu)->schedule_lock) ) \
            return lock; \
        spin_unlock##irq(lock, ## arg); \
    } \
}

#define sched_unlock(kind, param, cpu, irq, arg...) \
static inline void kind##_schedule_unlock##irq(spinlock_t *lock \
                                               EXTRA_TYPE(arg), param) \
{ \
    ASSERT(lock == get_sched_res(cpu)->schedule_lock); \
    spin_unlock##irq(lock, ## arg); \
}

#define EXTRA_TYPE(arg)
sched_lock(pcpu, unsigned int cpu,     cpu, )
sched_lock(unit, const struct sched_unit *i, i->res->processor, )
sched_lock(pcpu, unsigned int cpu,     cpu,          _irq)
sched_lock(unit, const struct sched_unit *i, i->res->processor, _irq)
sched_unlock(pcpu, unsigned int cpu,     cpu, )
sched_unlock(unit, const struct sched_unit *i, i->res->processor, )
sched_unlock(pcpu, unsigned int cpu,     cpu,          _irq)
sched_unlock(unit, const struct sched_unit *i, i->res->processor, _irq)
#undef EXTRA_TYPE

#define EXTRA_TYPE(arg) , unsigned long arg
#define spin_unlock_irqsave spin_unlock_irqrestore
sched_lock(pcpu, unsigned int cpu,     cpu,          _irqsave, *flags)
sched_lock(unit, const struct sched_unit *i, i->res->processor, _irqsave, *flags)
#undef spin_unlock_irqsave
sched_unlock(pcpu, unsigned int cpu,     cpu,          _irqrestore, flags)
sched_unlock(unit, const struct sched_unit *i, i->res->processor, _irqrestore, flags)
#undef EXTRA_TYPE

#undef sched_unlock
#undef sched_lock

static inline spinlock_t *pcpu_schedule_trylock(unsigned int cpu)
{
    spinlock_t *lock = get_sched_res(cpu)->schedule_lock;

    if ( !spin_trylock(lock) )
        return NULL;
    if ( lock == get_sched_res(cpu)->schedule_lock )
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
                                    struct sched_unit *, void *);
    void         (*free_pdata)     (const struct scheduler *, void *, int);
    void *       (*alloc_pdata)    (const struct scheduler *, int);
    void         (*init_pdata)     (const struct scheduler *, void *, int);
    void         (*deinit_pdata)   (const struct scheduler *, void *, int);

    /* Returns ERR_PTR(-err) for error, NULL for 'nothing needed'. */
    void *       (*alloc_domdata)  (const struct scheduler *, struct domain *);
    /* Idempotent. */
    void         (*free_domdata)   (const struct scheduler *, void *);

    spinlock_t * (*switch_sched)   (struct scheduler *, unsigned int,
                                    void *, void *);

    /* Activate / deactivate units in a cpu pool */
    void         (*insert_unit)    (const struct scheduler *,
                                    struct sched_unit *);
    void         (*remove_unit)    (const struct scheduler *,
                                    struct sched_unit *);

    void         (*sleep)          (const struct scheduler *,
                                    struct sched_unit *);
    void         (*wake)           (const struct scheduler *,
                                    struct sched_unit *);
    void         (*yield)          (const struct scheduler *,
                                    struct sched_unit *);
    void         (*context_saved)  (const struct scheduler *,
                                    struct sched_unit *);

    void         (*do_schedule)    (const struct scheduler *,
                                    struct sched_unit *, s_time_t,
                                    bool tasklet_work_scheduled);

    struct sched_resource * (*pick_resource) (const struct scheduler *,
                                              struct sched_unit *);
    void         (*migrate)        (const struct scheduler *,
                                    struct sched_unit *, unsigned int);
    int          (*adjust)         (const struct scheduler *, struct domain *,
                                    struct xen_domctl_scheduler_op *);
    void         (*adjust_affinity)(const struct scheduler *,
                                    struct sched_unit *,
                                    const struct cpumask *,
                                    const struct cpumask *);
    int          (*adjust_global)  (const struct scheduler *,
                                    struct xen_sysctl_scheduler_op *);
    void         (*dump_settings)  (const struct scheduler *);
    void         (*dump_cpu_state) (const struct scheduler *, int);

    void         (*tick_suspend)    (const struct scheduler *, unsigned int);
    void         (*tick_resume)     (const struct scheduler *, unsigned int);
};

static inline int sched_init(struct scheduler *s)
{
    return s->init(s);
}

static inline void sched_deinit(struct scheduler *s)
{
    s->deinit(s);
}

static inline spinlock_t *sched_switch_sched(struct scheduler *s,
                                             unsigned int cpu,
                                             void *pdata, void *vdata)
{
    return s->switch_sched(s, cpu, pdata, vdata);
}

static inline void sched_dump_settings(const struct scheduler *s)
{
    if ( s->dump_settings )
        s->dump_settings(s);
}

static inline void sched_dump_cpu_state(const struct scheduler *s, int cpu)
{
    if ( s->dump_cpu_state )
        s->dump_cpu_state(s, cpu);
}

static inline void sched_do_tick_suspend(const struct scheduler *s, int cpu)
{
    if ( s->tick_suspend )
        s->tick_suspend(s, cpu);
}

static inline void sched_do_tick_resume(const struct scheduler *s, int cpu)
{
    if ( s->tick_resume )
        s->tick_resume(s, cpu);
}

static inline void *sched_alloc_domdata(const struct scheduler *s,
                                        struct domain *d)
{
    return s->alloc_domdata ? s->alloc_domdata(s, d) : NULL;
}

static inline void sched_free_domdata(const struct scheduler *s,
                                      void *data)
{
    ASSERT(s->free_domdata || !data);
    if ( s->free_domdata )
        s->free_domdata(s, data);
}

static inline void *sched_alloc_pdata(const struct scheduler *s, int cpu)
{
    return s->alloc_pdata ? s->alloc_pdata(s, cpu) : NULL;
}

static inline void sched_free_pdata(const struct scheduler *s, void *data,
                                    int cpu)
{
    ASSERT(s->free_pdata || !data);
    if ( s->free_pdata )
        s->free_pdata(s, data, cpu);
}

static inline void sched_init_pdata(const struct scheduler *s, void *data,
                                    int cpu)
{
    if ( s->init_pdata )
        s->init_pdata(s, data, cpu);
}

static inline void sched_deinit_pdata(const struct scheduler *s, void *data,
                                      int cpu)
{
    if ( s->deinit_pdata )
        s->deinit_pdata(s, data, cpu);
}

static inline void *sched_alloc_vdata(const struct scheduler *s,
                                      struct sched_unit *unit, void *dom_data)
{
    return s->alloc_vdata(s, unit, dom_data);
}

static inline void sched_free_vdata(const struct scheduler *s, void *data)
{
    s->free_vdata(s, data);
}

static inline void sched_insert_unit(const struct scheduler *s,
                                     struct sched_unit *unit)
{
    if ( s->insert_unit )
        s->insert_unit(s, unit);
}

static inline void sched_remove_unit(const struct scheduler *s,
                                     struct sched_unit *unit)
{
    if ( s->remove_unit )
        s->remove_unit(s, unit);
}

static inline void sched_sleep(const struct scheduler *s,
                               struct sched_unit *unit)
{
    if ( s->sleep )
        s->sleep(s, unit);
}

static inline void sched_wake(const struct scheduler *s,
                              struct sched_unit *unit)
{
    if ( s->wake )
        s->wake(s, unit);
}

static inline void sched_yield(const struct scheduler *s,
                               struct sched_unit *unit)
{
    if ( s->yield )
        s->yield(s, unit);
}

static inline void sched_context_saved(const struct scheduler *s,
                                       struct sched_unit *unit)
{
    if ( s->context_saved )
        s->context_saved(s, unit);
}

static inline void sched_migrate(const struct scheduler *s,
                                 struct sched_unit *unit, unsigned int cpu)
{
    if ( s->migrate )
        s->migrate(s, unit, cpu);
    else
        sched_set_res(unit, get_sched_res(cpu));
}

static inline struct sched_resource *sched_pick_resource(
    const struct scheduler *s, struct sched_unit *unit)
{
    return s->pick_resource(s, unit);
}

static inline void sched_adjust_affinity(const struct scheduler *s,
                                         struct sched_unit *unit,
                                         const cpumask_t *hard,
                                         const cpumask_t *soft)
{
    if ( s->adjust_affinity )
        s->adjust_affinity(s, unit, hard, soft);
}

static inline int sched_adjust_dom(const struct scheduler *s, struct domain *d,
                                   struct xen_domctl_scheduler_op *op)
{
    return s->adjust ? s->adjust(s, d, op) : 0;
}

static inline int sched_adjust_cpupool(const struct scheduler *s,
                                       struct xen_sysctl_scheduler_op *op)
{
    return s->adjust_global ? s->adjust_global(s, op) : 0;
}

static inline void sched_unit_pause_nosync(struct sched_unit *unit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        vcpu_pause_nosync(v);
}

static inline void sched_unit_unpause(struct sched_unit *unit)
{
    struct vcpu *v;

    for_each_sched_unit_vcpu ( unit, v )
        vcpu_unpause(v);
}

#define REGISTER_SCHEDULER(x) static const struct scheduler *x##_entry \
  __used_section(".data.schedulers") = &x;

struct cpupool
{
    int              cpupool_id;
    unsigned int     n_dom;
    cpumask_var_t    cpu_valid;      /* all cpus assigned to pool */
    cpumask_var_t    res_valid;      /* all scheduling resources of pool */
    struct cpupool   *next;
    struct scheduler *sched;
    atomic_t         refcnt;
    unsigned int     granularity;
    enum sched_gran  opt_granularity;
};

#define cpupool_online_cpumask(_pool) \
    (((_pool) == NULL) ? &cpu_online_map : (_pool)->cpu_valid)

static inline cpumask_t* cpupool_domain_cpumask(const struct domain *d)
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
static inline int has_soft_affinity(const struct sched_unit *unit)
{
    return unit->soft_aff_effective &&
           !cpumask_subset(cpupool_domain_cpumask(unit->vcpu->domain),
                           unit->cpu_soft_affinity);
}

/*
 * This function copies in mask the cpumask that should be used for a
 * particular affinity balancing step. For the soft affinity one, the pcpus
 * that are not part of vc's hard affinity are filtered out from the result,
 * to avoid running a vcpu where it would like, but is not allowed to!
 */
static inline void
affinity_balance_cpumask(const struct sched_unit *unit, int step,
                         cpumask_t *mask)
{
    if ( step == BALANCE_SOFT_AFFINITY )
    {
        cpumask_and(mask, unit->cpu_soft_affinity, unit->cpu_hard_affinity);

        if ( unlikely(cpumask_empty(mask)) )
            cpumask_copy(mask, unit->cpu_hard_affinity);
    }
    else /* step == BALANCE_HARD_AFFINITY */
        cpumask_copy(mask, unit->cpu_hard_affinity);
}

#endif /* __XEN_SCHED_IF_H__ */
