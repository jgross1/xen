/******************************************************************************
 * arch/x86/pv/domain.c
 *
 * PV domain handling
 */

#include <xen/domain_page.h>
#include <xen/errno.h>
#include <xen/lib.h>
#include <xen/sched.h>

#include <asm/cpufeature.h>
#include <asm/invpcid.h>
#include <asm/spec_ctrl.h>
#include <asm/pv/domain.h>
#include <asm/shadow.h>

static __read_mostly enum {
    PCID_OFF,
    PCID_ALL,
    PCID_XPTI,
    PCID_NOXPTI
} opt_pcid = PCID_XPTI;

static int parse_pcid(const char *s)
{
    int rc = 0;

    switch ( parse_bool(s, NULL) )
    {
    case 0:
        opt_pcid = PCID_OFF;
        break;

    case 1:
        opt_pcid = PCID_ALL;
        break;

    default:
        switch ( parse_boolean("xpti", s, NULL) )
        {
        case 0:
            opt_pcid = PCID_NOXPTI;
            break;

        case 1:
            opt_pcid = PCID_XPTI;
            break;

        default:
            rc = -EINVAL;
            break;
        }
        break;
    }

    return rc;
}
custom_runtime_param("pcid", parse_pcid);

static void noreturn continue_nonidle_domain(struct vcpu *v)
{
    check_wakeup_from_wait();
    reset_stack_and_jump(ret_from_intr);
}

static int setup_compat_l4(struct vcpu *v)
{
    struct page_info *pg;
    l4_pgentry_t *l4tab;
    mfn_t mfn;

    pg = alloc_domheap_page(v->domain, MEMF_no_owner);
    if ( pg == NULL )
        return -ENOMEM;

    mfn = page_to_mfn(pg);
    l4tab = map_domain_page(mfn);
    clear_page(l4tab);
    init_xen_l4_slots(l4tab, mfn, v->domain, INVALID_MFN, false);
    unmap_domain_page(l4tab);

    /* This page needs to look like a pagetable so that it can be shadowed */
    pg->u.inuse.type_info = PGT_l4_page_table | PGT_validated | 1;

    v->arch.guest_table = pagetable_from_page(pg);
    v->arch.guest_table_user = v->arch.guest_table;

    return 0;
}

static void release_compat_l4(struct vcpu *v)
{
    if ( !pagetable_is_null(v->arch.guest_table) )
        free_domheap_page(pagetable_get_page(v->arch.guest_table));
    v->arch.guest_table = pagetable_null();
    v->arch.guest_table_user = pagetable_null();
}

int switch_compat(struct domain *d)
{
    struct vcpu *v;
    int rc;

    if ( is_hvm_domain(d) || d->tot_pages != 0 )
        return -EACCES;
    if ( is_pv_32bit_domain(d) )
        return 0;

    d->arch.has_32bit_shinfo = 1;
    d->arch.is_32bit_pv = 1;

    for_each_vcpu( d, v )
    {
        if ( (rc = setup_compat_arg_xlat(v)) ||
             (rc = setup_compat_l4(v)) )
            goto undo_and_fail;
    }

    domain_set_alloc_bitsize(d);
    recalculate_cpuid_policy(d);

    d->arch.x87_fip_width = 4;

    d->arch.pv.xpti = false;
    d->arch.pv.pcid = false;

    return 0;

 undo_and_fail:
    d->arch.is_32bit_pv = d->arch.has_32bit_shinfo = 0;
    for_each_vcpu( d, v )
    {
        free_compat_arg_xlat(v);
        release_compat_l4(v);
    }

    return rc;
}

static int pv_create_gdt_ldt_l1tab(struct vcpu *v)
{
    return create_perdomain_mapping(v->domain, GDT_VIRT_START(v),
                                    1U << GDT_LDT_VCPU_SHIFT,
                                    v->domain->arch.pv.gdt_ldt_l1tab,
                                    NULL);
}

static void pv_destroy_gdt_ldt_l1tab(struct vcpu *v)
{
    destroy_perdomain_mapping(v->domain, GDT_VIRT_START(v),
                              1U << GDT_LDT_VCPU_SHIFT);
}

void pv_vcpu_destroy(struct vcpu *v)
{
    if ( is_pv_32bit_vcpu(v) )
    {
        free_compat_arg_xlat(v);
        release_compat_l4(v);
    }

    pv_destroy_gdt_ldt_l1tab(v);
    XFREE(v->arch.pv.trap_ctxt);
}

int pv_vcpu_initialise(struct vcpu *v)
{
    struct domain *d = v->domain;
    int rc;

    ASSERT(!is_idle_domain(d));

#ifdef CONFIG_PV_LDT_PAGING
    spin_lock_init(&v->arch.pv.shadow_ldt_lock);
#endif

    rc = pv_create_gdt_ldt_l1tab(v);
    if ( rc )
        return rc;

    BUILD_BUG_ON(NR_VECTORS * sizeof(*v->arch.pv.trap_ctxt) >
                 PAGE_SIZE);
    v->arch.pv.trap_ctxt = xzalloc_array(struct trap_info, NR_VECTORS);
    if ( !v->arch.pv.trap_ctxt )
    {
        rc = -ENOMEM;
        goto done;
    }

    /* PV guests by default have a 100Hz ticker. */
    v->periodic_period = MILLISECS(10);

    v->arch.pv.ctrlreg[4] = real_cr4_to_pv_guest_cr4(mmu_cr4_features);

    if ( is_pv_32bit_domain(d) )
    {
        if ( (rc = setup_compat_arg_xlat(v)) )
            goto done;

        if ( (rc = setup_compat_l4(v)) )
            goto done;
    }

    if ( d->arch.pv.l4tab_idle )
    {
        v->arch.guest_table = pagetable_from_paddr(__pa(d->arch.pv.l4tab_idle));
        update_cr3(v);
    }

 done:
    if ( rc )
        pv_vcpu_destroy(v);
    return rc;
}

void pv_domain_destroy(struct domain *d)
{
    pv_l1tf_domain_destroy(d);

    destroy_perdomain_mapping(d, GDT_LDT_VIRT_START,
                              GDT_LDT_MBYTES << (20 - PAGE_SHIFT));

    XFREE(d->arch.pv.cpuidmasks);

    FREE_XENHEAP_PAGE(d->arch.pv.gdt_ldt_l1tab);
    FREE_XENHEAP_PAGE(d->arch.pv.l4tab_idle);
}


int pv_domain_initialise(struct domain *d)
{
    static const struct arch_csw pv_csw = {
        .from = paravirt_ctxt_switch_from,
        .to   = paravirt_ctxt_switch_to,
        .tail = continue_nonidle_domain,
    };
    int rc = -ENOMEM;

    pv_l1tf_domain_init(d);

    d->arch.pv.gdt_ldt_l1tab =
        alloc_xenheap_pages(0, MEMF_node(domain_to_node(d)));
    if ( !d->arch.pv.gdt_ldt_l1tab )
        goto fail;
    clear_page(d->arch.pv.gdt_ldt_l1tab);

    if ( levelling_caps & ~LCAP_faulting &&
         (d->arch.pv.cpuidmasks = xmemdup(&cpuidmask_defaults)) == NULL )
        goto fail;

    rc = create_perdomain_mapping(d, GDT_LDT_VIRT_START,
                                  GDT_LDT_MBYTES << (20 - PAGE_SHIFT),
                                  NULL, NULL);
    if ( rc )
        goto fail;

    d->arch.ctxt_switch = &pv_csw;

    if ( sched_granularity > 1 )
    {
        l4_pgentry_t *l4;

        l4 = alloc_xenheap_pages(0, MEMF_node(domain_to_node(d)));
        if ( !l4 )
            goto fail;
        clear_page(l4);
        init_xen_l4_slots(l4, _mfn(virt_to_mfn(l4)), d, INVALID_MFN, true);
        d->arch.pv.l4tab_idle = l4;
    }

    /* 64-bit PV guest by default. */
    d->arch.is_32bit_pv = d->arch.has_32bit_shinfo = 0;

    d->arch.pv.xpti = is_hardware_domain(d) ? opt_xpti_hwdom : opt_xpti_domu;

    if ( !is_pv_32bit_domain(d) && use_invpcid && cpu_has_pcid )
        switch ( ACCESS_ONCE(opt_pcid) )
        {
        case PCID_OFF:
            break;

        case PCID_ALL:
            d->arch.pv.pcid = true;
            break;

        case PCID_XPTI:
            d->arch.pv.pcid = d->arch.pv.xpti;
            break;

        case PCID_NOXPTI:
            d->arch.pv.pcid = !d->arch.pv.xpti;
            break;

        default:
            ASSERT_UNREACHABLE();
            break;
        }

    return 0;

  fail:
    pv_domain_destroy(d);

    return rc;
}

bool __init xpti_pcid_enabled(void)
{
    return use_invpcid && cpu_has_pcid &&
           (opt_pcid == PCID_ALL || opt_pcid == PCID_XPTI);
}

static void _toggle_guest_pt(struct vcpu *v)
{
    const struct domain *d = v->domain;

    v->arch.flags ^= TF_kernel_mode;
    update_cr3(v);
    if ( d->arch.pv.xpti )
    {
        struct cpu_info *cpu_info = get_cpu_info();

        cpu_info->root_pgt_changed = true;
        cpu_info->pv_cr3 = __pa(this_cpu(root_pgt)) |
                           (d->arch.pv.pcid ? get_pcid_bits(v, true) : 0);
    }

    /* Don't flush user global mappings from the TLB. Don't tick TLB clock. */
    write_cr3(v->arch.cr3);

    if ( !(v->arch.flags & TF_kernel_mode) )
        return;

    if ( v->arch.pv.need_update_runstate_area && update_runstate_area(v) )
        v->arch.pv.need_update_runstate_area = 0;

    if ( v->arch.pv.pending_system_time.version &&
         update_secondary_system_time(v, &v->arch.pv.pending_system_time) )
        v->arch.pv.pending_system_time.version = 0;
}

void toggle_guest_mode(struct vcpu *v)
{
    ASSERT(!is_pv_32bit_vcpu(v));

    if ( cpu_has_fsgsbase )
    {
        if ( v->arch.flags & TF_kernel_mode )
            v->arch.pv.gs_base_kernel = __rdgsbase();
        else
            v->arch.pv.gs_base_user = __rdgsbase();
    }
    asm volatile ( "swapgs" );

    _toggle_guest_pt(v);
}

void toggle_guest_pt(struct vcpu *v)
{
    if ( !is_pv_32bit_vcpu(v) )
        _toggle_guest_pt(v);
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
