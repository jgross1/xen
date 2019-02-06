/******************************************************************************
 * sched_arinc653.c
 *
 * An ARINC653-compatible scheduling algorithm for use in Xen.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright (c) 2010, DornerWorks, Ltd. <DornerWorks.com>
 */

#include <xen/lib.h>
#include <xen/sched.h>
#include <xen/sched-if.h>
#include <xen/timer.h>
#include <xen/softirq.h>
#include <xen/time.h>
#include <xen/errno.h>
#include <xen/list.h>
#include <xen/guest_access.h>
#include <public/sysctl.h>

/**************************************************************************
 * Private Macros                                                         *
 **************************************************************************/

/**
 * Default timeslice for domain 0.
 */
#define DEFAULT_TIMESLICE MILLISECS(10)

/**
 * Retrieve the idle ITEM for a given physical CPU
 */
#define IDLETASK(cpu)  (sched_idle_item(cpu))

/**
 * Return a pointer to the ARINC 653-specific scheduler data information
 * associated with the given ITEM (item)
 */
#define AITEM(item) ((arinc653_item_t *)(item)->priv)

/**
 * Return the global scheduler private data given the scheduler ops pointer
 */
#define SCHED_PRIV(s) ((a653sched_priv_t *)((s)->sched_data))

/**************************************************************************
 * Private Type Definitions                                               *
 **************************************************************************/

/**
 * The arinc653_item_t structure holds ARINC 653-scheduler-specific
 * information for all non-idle ITEMs
 */
typedef struct arinc653_item_s
{
    /* item points to Xen's struct sched_item so we can get to it from an
     * arinc653_item_t pointer. */
    struct sched_item * item;
    /* awake holds whether the ITEM has been woken with vcpu_wake() */
    bool_t              awake;
    /* list holds the linked list information for the list this ITEM
     * is stored in */
    struct list_head    list;
} arinc653_item_t;

/**
 * The sched_entry_t structure holds a single entry of the
 * ARINC 653 schedule.
 */
typedef struct sched_entry_s
{
    /* dom_handle holds the handle ("UUID") for the domain that this
     * schedule entry refers to. */
    xen_domain_handle_t dom_handle;
    /* item_id holds the ITEM number for the ITEM that this schedule
     * entry refers to. */
    int                 item_id;
    /* runtime holds the number of nanoseconds that the ITEM for this
     * schedule entry should be allowed to run per major frame. */
    s_time_t            runtime;
    /* item holds a pointer to the Xen sched_item structure */
    struct sched_item * item;
} sched_entry_t;

/**
 * This structure defines data that is global to an instance of the scheduler
 */
typedef struct a653sched_priv_s
{
    /* lock for the whole pluggable scheduler, nests inside cpupool_lock */
    spinlock_t lock;

    /**
     * This array holds the active ARINC 653 schedule.
     *
     * When the system tries to start a new ITEM, this schedule is scanned
     * to look for a matching (handle, ITEM #) pair. If both the handle (UUID)
     * and ITEM number match, then the ITEM is allowed to run. Its run time
     * (per major frame) is given in the third entry of the schedule.
     */
    sched_entry_t schedule[ARINC653_MAX_DOMAINS_PER_SCHEDULE];

    /**
     * This variable holds the number of entries that are valid in
     * the arinc653_schedule table.
     *
     * This is not necessarily the same as the number of domains in the
     * schedule. A domain could be listed multiple times within the schedule,
     * or a domain with multiple ITEMs could have a different
     * schedule entry for each ITEM.
     */
    unsigned int num_schedule_entries;

    /**
     * the major frame time for the ARINC 653 schedule.
     */
    s_time_t major_frame;

    /**
     * the time that the next major frame starts
     */
    s_time_t next_major_frame;

    /**
     * pointers to all Xen ITEM structures for iterating through
     */
    struct list_head item_list;
} a653sched_priv_t;

/**************************************************************************
 * Helper functions                                                       *
 **************************************************************************/

/**
 * This function compares two domain handles.
 *
 * @param h1        Pointer to handle 1
 * @param h2        Pointer to handle 2
 *
 * @return          <ul>
 *                  <li> <0:  handle 1 is less than handle 2
 *                  <li>  0:  handle 1 is equal to handle 2
 *                  <li> >0:  handle 1 is greater than handle 2
 *                  </ul>
 */
static int dom_handle_cmp(const xen_domain_handle_t h1,
                          const xen_domain_handle_t h2)
{
    return memcmp(h1, h2, sizeof(xen_domain_handle_t));
}

/**
 * This function searches the item list to find a ITEM that matches
 * the domain handle and ITEM ID specified.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param handle    Pointer to handler
 * @param item_id   ITEM ID
 *
 * @return          <ul>
 *                  <li> Pointer to the matching ITEM if one is found
 *                  <li> NULL otherwise
 *                  </ul>
 */
static struct sched_item *find_item(
    const struct scheduler *ops,
    xen_domain_handle_t handle,
    int item_id)
{
    arinc653_item_t *aitem;

    /* loop through the item_list looking for the specified ITEM */
    list_for_each_entry ( aitem, &SCHED_PRIV(ops)->item_list, list )
        if ( (dom_handle_cmp(aitem->item->domain->handle, handle) == 0)
             && (item_id == aitem->item->item_id) )
            return aitem->item;

    return NULL;
}

/**
 * This function updates the pointer to the Xen ITEM structure for each entry
 * in the ARINC 653 schedule.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @return          <None>
 */
static void update_schedule_items(const struct scheduler *ops)
{
    unsigned int i, n_entries = SCHED_PRIV(ops)->num_schedule_entries;

    for ( i = 0; i < n_entries; i++ )
        SCHED_PRIV(ops)->schedule[i].item =
            find_item(ops,
                      SCHED_PRIV(ops)->schedule[i].dom_handle,
                      SCHED_PRIV(ops)->schedule[i].item_id);
}

/**
 * This function is called by the adjust_global scheduler hook to put
 * in place a new ARINC653 schedule.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 *
 * @return          <ul>
 *                  <li> 0 = success
 *                  <li> !0 = error
 *                  </ul>
 */
static int
arinc653_sched_set(
    const struct scheduler *ops,
    struct xen_sysctl_arinc653_schedule *schedule)
{
    a653sched_priv_t *sched_priv = SCHED_PRIV(ops);
    s_time_t total_runtime = 0;
    unsigned int i;
    unsigned long flags;
    int rc = -EINVAL;

    spin_lock_irqsave(&sched_priv->lock, flags);

    /* Check for valid major frame and number of schedule entries. */
    if ( (schedule->major_frame <= 0)
         || (schedule->num_sched_entries < 1)
         || (schedule->num_sched_entries > ARINC653_MAX_DOMAINS_PER_SCHEDULE) )
        goto fail;

    for ( i = 0; i < schedule->num_sched_entries; i++ )
    {
        /* Check for a valid run time. */
        if ( schedule->sched_entries[i].runtime <= 0 )
            goto fail;

        /* Add this entry's run time to total run time. */
        total_runtime += schedule->sched_entries[i].runtime;
    }

    /*
     * Error if the major frame is not large enough to run all entries as
     * indicated by comparing the total run time to the major frame length.
     */
    if ( total_runtime > schedule->major_frame )
        goto fail;

    /* Copy the new schedule into place. */
    sched_priv->num_schedule_entries = schedule->num_sched_entries;
    sched_priv->major_frame = schedule->major_frame;
    for ( i = 0; i < schedule->num_sched_entries; i++ )
    {
        memcpy(sched_priv->schedule[i].dom_handle,
               schedule->sched_entries[i].dom_handle,
               sizeof(sched_priv->schedule[i].dom_handle));
        sched_priv->schedule[i].item_id =
            schedule->sched_entries[i].vcpu_id;
        sched_priv->schedule[i].runtime =
            schedule->sched_entries[i].runtime;
    }
    update_schedule_items(ops);

    /*
     * The newly-installed schedule takes effect immediately. We do not even
     * wait for the current major frame to expire.
     *
     * Signal a new major frame to begin. The next major frame is set up by
     * the do_schedule callback function when it is next invoked.
     */
    sched_priv->next_major_frame = NOW();

    rc = 0;

 fail:
    spin_unlock_irqrestore(&sched_priv->lock, flags);
    return rc;
}

/**
 * This function is called by the adjust_global scheduler hook to read the
 * current ARINC 653 schedule
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @return          <ul>
 *                  <li> 0 = success
 *                  <li> !0 = error
 *                  </ul>
 */
static int
arinc653_sched_get(
    const struct scheduler *ops,
    struct xen_sysctl_arinc653_schedule *schedule)
{
    a653sched_priv_t *sched_priv = SCHED_PRIV(ops);
    unsigned int i;
    unsigned long flags;

    spin_lock_irqsave(&sched_priv->lock, flags);

    schedule->num_sched_entries = sched_priv->num_schedule_entries;
    schedule->major_frame = sched_priv->major_frame;
    for ( i = 0; i < sched_priv->num_schedule_entries; i++ )
    {
        memcpy(schedule->sched_entries[i].dom_handle,
               sched_priv->schedule[i].dom_handle,
               sizeof(sched_priv->schedule[i].dom_handle));
        schedule->sched_entries[i].vcpu_id = sched_priv->schedule[i].item_id;
        schedule->sched_entries[i].runtime = sched_priv->schedule[i].runtime;
    }

    spin_unlock_irqrestore(&sched_priv->lock, flags);

    return 0;
}

/**************************************************************************
 * Scheduler callback functions                                           *
 **************************************************************************/

/**
 * This function performs initialization for an instance of the scheduler.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 *
 * @return          <ul>
 *                  <li> 0 = success
 *                  <li> !0 = error
 *                  </ul>
 */
static int
a653sched_init(struct scheduler *ops)
{
    a653sched_priv_t *prv;

    prv = xzalloc(a653sched_priv_t);
    if ( prv == NULL )
        return -ENOMEM;

    ops->sched_data = prv;

    prv->next_major_frame = 0;
    spin_lock_init(&prv->lock);
    INIT_LIST_HEAD(&prv->item_list);

    return 0;
}

/**
 * This function performs deinitialization for an instance of the scheduler
 *
 * @param ops       Pointer to this instance of the scheduler structure
 */
static void
a653sched_deinit(struct scheduler *ops)
{
    xfree(SCHED_PRIV(ops));
    ops->sched_data = NULL;
}

/**
 * This function allocates scheduler-specific data for a ITEM
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param item      Pointer to struct sched_item
 *
 * @return          Pointer to the allocated data
 */
static void *
a653sched_alloc_vdata(const struct scheduler *ops, struct sched_item *item,
                      void *dd)
{
    a653sched_priv_t *sched_priv = SCHED_PRIV(ops);
    arinc653_item_t *svc;
    unsigned int entry;
    unsigned long flags;

    /*
     * Allocate memory for the ARINC 653-specific scheduler data information
     * associated with the given ITEM (item).
     */
    svc = xmalloc(arinc653_item_t);
    if ( svc == NULL )
        return NULL;

    spin_lock_irqsave(&sched_priv->lock, flags);

    /*
     * Add every one of dom0's items to the schedule, as long as there are
     * slots available.
     */
    if ( item->domain->domain_id == 0 )
    {
        entry = sched_priv->num_schedule_entries;

        if ( entry < ARINC653_MAX_DOMAINS_PER_SCHEDULE )
        {
            sched_priv->schedule[entry].dom_handle[0] = '\0';
            sched_priv->schedule[entry].item_id = item->item_id;
            sched_priv->schedule[entry].runtime = DEFAULT_TIMESLICE;
            sched_priv->schedule[entry].item = item;

            sched_priv->major_frame += DEFAULT_TIMESLICE;
            ++sched_priv->num_schedule_entries;
        }
    }

    /*
     * Initialize our ARINC 653 scheduler-specific information for the ITEM.
     * The ITEM starts "asleep." When Xen is ready for the ITEM to run, it
     * will call the vcpu_wake scheduler callback function and our scheduler
     * will mark the ITEM awake.
     */
    svc->item = item;
    svc->awake = 0;
    if ( !is_idle_item(item) )
        list_add(&svc->list, &SCHED_PRIV(ops)->item_list);
    update_schedule_items(ops);

    spin_unlock_irqrestore(&sched_priv->lock, flags);

    return svc;
}

/**
 * This function frees scheduler-specific ITEM data
 *
 * @param ops       Pointer to this instance of the scheduler structure
 */
static void
a653sched_free_vdata(const struct scheduler *ops, void *priv)
{
    arinc653_item_t *av = priv;

    if (av == NULL)
        return;

    if ( !is_idle_item(av->item) )
        list_del(&av->list);

    xfree(av);
    update_schedule_items(ops);
}

/**
 * Xen scheduler callback function to sleep a ITEM
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param item      Pointer to struct sched_item
 */
static void
a653sched_item_sleep(const struct scheduler *ops, struct sched_item *item)
{
    if ( AITEM(item) != NULL )
        AITEM(item)->awake = 0;

    /*
     * If the ITEM being put to sleep is the same one that is currently
     * running, raise a softirq to invoke the scheduler to switch domains.
     */
    if ( per_cpu(sched_res, sched_item_cpu(item))->curr == item )
        cpu_raise_softirq(sched_item_cpu(item), SCHEDULE_SOFTIRQ);
}

/**
 * Xen scheduler callback function to wake up a ITEM
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param item      Pointer to struct sched_item
 */
static void
a653sched_item_wake(const struct scheduler *ops, struct sched_item *item)
{
    if ( AITEM(item) != NULL )
        AITEM(item)->awake = 1;

    cpu_raise_softirq(sched_item_cpu(item), SCHEDULE_SOFTIRQ);
}

/**
 * Xen scheduler callback function to select a ITEM to run.
 * This is the main scheduler routine.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param now       Current time
 */
static void
a653sched_do_schedule(
    const struct scheduler *ops,
    struct sched_item *prev,
    s_time_t now,
    bool tasklet_work_scheduled)
{
    struct sched_item *new_task = NULL;
    static unsigned int sched_index = 0;
    static s_time_t next_switch_time;
    a653sched_priv_t *sched_priv = SCHED_PRIV(ops);
    const unsigned int cpu = sched_get_resource_cpu(smp_processor_id());
    unsigned long flags;

    spin_lock_irqsave(&sched_priv->lock, flags);

    if ( sched_priv->num_schedule_entries < 1 )
        sched_priv->next_major_frame = now + DEFAULT_TIMESLICE;
    else if ( now >= sched_priv->next_major_frame )
    {
        /* time to enter a new major frame
         * the first time this function is called, this will be true */
        /* start with the first domain in the schedule */
        sched_index = 0;
        sched_priv->next_major_frame = now + sched_priv->major_frame;
        next_switch_time = now + sched_priv->schedule[0].runtime;
    }
    else
    {
        while ( (now >= next_switch_time)
                && (sched_index < sched_priv->num_schedule_entries) )
        {
            /* time to switch to the next domain in this major frame */
            sched_index++;
            next_switch_time += sched_priv->schedule[sched_index].runtime;
        }
    }

    /*
     * If we exhausted the domains in the schedule and still have time left
     * in the major frame then switch next at the next major frame.
     */
    if ( sched_index >= sched_priv->num_schedule_entries )
        next_switch_time = sched_priv->next_major_frame;

    /*
     * If there are more domains to run in the current major frame, set
     * new_task equal to the address of next domain's sched_item structure.
     * Otherwise, set new_task equal to the address of the idle task's
     * sched_item structure.
     */
    new_task = (sched_index < sched_priv->num_schedule_entries)
        ? sched_priv->schedule[sched_index].item
        : IDLETASK(cpu);

    /* Check to see if the new task can be run (awake & runnable). */
    if ( !((new_task != NULL)
           && (AITEM(new_task) != NULL)
           && AITEM(new_task)->awake
           && item_runnable(new_task)) )
        new_task = IDLETASK(cpu);
    BUG_ON(new_task == NULL);

    /*
     * Check to make sure we did not miss a major frame.
     * This is a good test for robust partitioning.
     */
    BUG_ON(now >= sched_priv->next_major_frame);

    spin_unlock_irqrestore(&sched_priv->lock, flags);

    /* Tasklet work (which runs in idle ITEM context) overrides all else. */
    if ( tasklet_work_scheduled )
        new_task = IDLETASK(cpu);

    /* Running this task would result in a migration */
    if ( !is_idle_item(new_task)
         && (sched_item_cpu(new_task) != cpu) )
        new_task = IDLETASK(cpu);

    /*
     * Return the amount of time the next domain has to run and the address
     * of the selected task's ITEM structure.
     */
    prev->next_time = next_switch_time - now;
    prev->next_task = new_task;
    new_task->migrated = false;

    BUG_ON(prev->next_time <= 0);
}

/**
 * Xen scheduler callback function to select a resource for the ITEM to run on
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param item      Pointer to struct sched_item
 *
 * @return          Scheduler resource to run on
 */
static struct sched_resource *
a653sched_pick_resource(const struct scheduler *ops, struct sched_item *item)
{
    cpumask_t *online;
    unsigned int cpu;

    /*
     * If present, prefer item's current processor, else
     * just find the first valid item.
     */
    online = cpupool_domain_cpumask(item->domain);

    cpu = cpumask_first(online);

    if ( cpumask_test_cpu(sched_item_cpu(item), online)
         || (cpu >= nr_cpu_ids) )
        cpu = sched_item_cpu(item);

    return per_cpu(sched_res, cpu);
}

/**
 * Xen scheduler callback to change the scheduler of a cpu
 *
 * @param new_ops   Pointer to this instance of the scheduler structure
 * @param cpu       The cpu that is changing scheduler
 * @param pdata     scheduler specific PCPU data (we don't have any)
 * @param vdata     scheduler specific ITEM data of the idle item
 */
static void
a653_switch_sched(struct scheduler *new_ops, unsigned int cpu,
                  void *pdata, void *vdata)
{
    struct sched_resource *sd = per_cpu(sched_res, cpu);
    arinc653_item_t *svc = vdata;

    ASSERT(!pdata && svc && is_idle_item(svc->item));

    sched_idle_item(cpu)->priv = vdata;

    per_cpu(scheduler, cpu) = new_ops;
    per_cpu(sched_res, cpu)->sched_priv = NULL; /* no pdata */

    /*
     * (Re?)route the lock to its default location. We actually do not use
     * it, but if we leave it pointing to where it does now (i.e., the
     * runqueue lock for this PCPU in the default scheduler), we'd be
     * causing unnecessary contention on that lock (in cases where it is
     * shared among multiple PCPUs, like in Credit2 and RTDS).
     */
    sd->schedule_lock = &sd->_lock;
}

/**
 * Xen scheduler callback function to perform a global (not domain-specific)
 * adjustment. It is used by the ARINC 653 scheduler to put in place a new
 * ARINC 653 schedule or to retrieve the schedule currently in place.
 *
 * @param ops       Pointer to this instance of the scheduler structure
 * @param sc        Pointer to the scheduler operation specified by Domain 0
 */
static int
a653sched_adjust_global(const struct scheduler *ops,
                        struct xen_sysctl_scheduler_op *sc)
{
    struct xen_sysctl_arinc653_schedule local_sched;
    int rc = -EINVAL;

    switch ( sc->cmd )
    {
    case XEN_SYSCTL_SCHEDOP_putinfo:
        if ( copy_from_guest(&local_sched, sc->u.sched_arinc653.schedule, 1) )
        {
            rc = -EFAULT;
            break;
        }

        rc = arinc653_sched_set(ops, &local_sched);
        break;
    case XEN_SYSCTL_SCHEDOP_getinfo:
        memset(&local_sched, -1, sizeof(local_sched));
        rc = arinc653_sched_get(ops, &local_sched);
        if ( rc )
            break;

        if ( copy_to_guest(sc->u.sched_arinc653.schedule, &local_sched, 1) )
            rc = -EFAULT;
        break;
    }

    return rc;
}

/**
 * This structure defines our scheduler for Xen.
 * The entries tell Xen where to find our scheduler-specific
 * callback functions.
 * The symbol must be visible to the rest of Xen at link time.
 */
static const struct scheduler sched_arinc653_def = {
    .name           = "ARINC 653 Scheduler",
    .opt_name       = "arinc653",
    .sched_id       = XEN_SCHEDULER_ARINC653,
    .sched_data     = NULL,

    .init           = a653sched_init,
    .deinit         = a653sched_deinit,

    .free_vdata     = a653sched_free_vdata,
    .alloc_vdata    = a653sched_alloc_vdata,

    .insert_item    = NULL,
    .remove_item    = NULL,

    .sleep          = a653sched_item_sleep,
    .wake           = a653sched_item_wake,
    .yield          = NULL,
    .context_saved  = NULL,

    .do_schedule    = a653sched_do_schedule,

    .pick_resource  = a653sched_pick_resource,

    .switch_sched   = a653_switch_sched,

    .adjust         = NULL,
    .adjust_global  = a653sched_adjust_global,

    .dump_settings  = NULL,
    .dump_cpu_state = NULL,

    .tick_suspend   = NULL,
    .tick_resume    = NULL,
};

REGISTER_SCHEDULER(sched_arinc653_def);

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
