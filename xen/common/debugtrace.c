/******************************************************************************
 * debugtrace.c
 *
 * Debugtrace for Xen
 */


#include <xen/console.h>
#include <xen/cpu.h>
#include <xen/init.h>
#include <xen/keyhandler.h>
#include <xen/lib.h>
#include <xen/mm.h>
#include <xen/serial.h>
#include <xen/spinlock.h>
#include <xen/watchdog.h>

#define DEBUG_TRACE_ENTRY_SIZE   1024

/* Send output direct to console, or buffer it? */
static volatile bool debugtrace_send_to_console;

struct debugtrace_data {
    unsigned long bytes; /* Size of buffer. */
    unsigned long prd;   /* Producer index. */
    char          buf[];
};

static struct debugtrace_data *dt_data;
static DEFINE_PER_CPU(struct debugtrace_data *, dt_cpu_data);

static unsigned long debugtrace_bytes = 128 << 10;
static bool debugtrace_per_cpu;
static bool debugtrace_used;
static char debugtrace_last_entry_buf[DEBUG_TRACE_ENTRY_SIZE];
static DEFINE_SPINLOCK(debugtrace_lock);

static int __init debugtrace_parse_param(const char *s)
{
    if ( !strncmp(s, "cpu:", 4) )
    {
        debugtrace_per_cpu = true;
        s += 4;
    }
    debugtrace_bytes = parse_size_and_unit(s, NULL);
    return 0;
}
custom_param("debugtrace", debugtrace_parse_param);

static void debugtrace_dump_buffer(struct debugtrace_data *data,
                                   const char *which)
{
    if ( !data )
        return;

    printk("debugtrace_dump() %s buffer starting\n", which);

    /* Print oldest portion of the ring. */
    if ( data->buf[data->prd] != '\0' )
        console_serial_puts(&data->buf[data->prd], data->bytes - data->prd);

    /* Print youngest portion of the ring. */
    data->buf[data->prd] = '\0';
    console_serial_puts(&data->buf[0], data->prd);

    memset(data->buf, '\0', data->bytes);
    data->prd = 0;

    printk("debugtrace_dump() %s buffer finished\n", which);
}

static void debugtrace_dump_worker(void)
{
    unsigned int cpu;
    char buf[16];

    if ( !debugtrace_used )
        return;

    debugtrace_dump_buffer(dt_data, "global");

    for ( cpu = 0; cpu < nr_cpu_ids; cpu++ )
    {
        snprintf(buf, sizeof(buf), "cpu %u", cpu);
        debugtrace_dump_buffer(per_cpu(dt_cpu_data, cpu), buf);
    }

    debugtrace_last_entry_buf[0] = 0;
}

static void debugtrace_toggle(void)
{
    unsigned long flags;

    watchdog_disable();
    spin_lock_irqsave(&debugtrace_lock, flags);

    /*
     * Dump the buffer *before* toggling, in case the act of dumping the
     * buffer itself causes more printk() invocations.
     */
    printk("debugtrace_printk now writing to %s.\n",
           !debugtrace_send_to_console ? "console": "buffer");
    if ( !debugtrace_send_to_console )
        debugtrace_dump_worker();

    debugtrace_send_to_console = !debugtrace_send_to_console;

    spin_unlock_irqrestore(&debugtrace_lock, flags);
    watchdog_enable();
}

void debugtrace_dump(void)
{
    unsigned long flags;

    watchdog_disable();
    spin_lock_irqsave(&debugtrace_lock, flags);

    debugtrace_dump_worker();

    spin_unlock_irqrestore(&debugtrace_lock, flags);
    watchdog_enable();
}

static void debugtrace_add_to_buf(char *buf)
{
    struct debugtrace_data *data;
    char *p;

    data = debugtrace_per_cpu ? this_cpu(dt_cpu_data) : dt_data;

    for ( p = buf; *p != '\0'; p++ )
    {
        data->buf[data->prd++] = *p;
        if ( data->prd == data->bytes )
            data->prd = 0;
    }
}

void debugtrace_printk(const char *fmt, ...)
{
    static char buf[DEBUG_TRACE_ENTRY_SIZE];
    static unsigned int count, last_count, last_cpu;
    static unsigned long last_prd;

    char          cntbuf[24];
    va_list       args;
    unsigned long flags;
    unsigned int nr;
    struct debugtrace_data *data;
    unsigned int cpu;

    data = debugtrace_per_cpu ? this_cpu(dt_cpu_data) : dt_data;
    cpu = debugtrace_per_cpu ? smp_processor_id() : 0;
    if ( !data )
        return;

    debugtrace_used = true;

    spin_lock_irqsave(&debugtrace_lock, flags);

    va_start(args, fmt);
    nr = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if ( debugtrace_send_to_console )
    {
        unsigned int n = scnprintf(cntbuf, sizeof(cntbuf), "%u ", ++count);

        console_serial_puts(cntbuf, n);
        console_serial_puts(buf, nr);
    }
    else
    {
        if ( strcmp(buf, debugtrace_last_entry_buf) || cpu != last_cpu )
        {
            last_prd = data->prd;
            last_count = ++count;
            last_cpu = cpu;
            safe_strcpy(debugtrace_last_entry_buf, buf);
            snprintf(cntbuf, sizeof(cntbuf), "%u ", count);
        }
        else
        {
            data->prd = last_prd;
            snprintf(cntbuf, sizeof(cntbuf), "%u-%u ", last_count, ++count);
        }
        debugtrace_add_to_buf(cntbuf);
        debugtrace_add_to_buf(buf);
    }

    spin_unlock_irqrestore(&debugtrace_lock, flags);
}

static void debugtrace_key(unsigned char key)
{
    debugtrace_toggle();
}

static void debugtrace_alloc_buffer(struct debugtrace_data **ptr,
                                    unsigned int cpu)
{
    int order;
    struct debugtrace_data *data;

    if ( !debugtrace_bytes || *ptr )
        return;

    order = get_order_from_bytes(debugtrace_bytes);
    data = alloc_xenheap_pages(order, 0);
    if ( !data )
    {
        if ( debugtrace_per_cpu )
            printk("failed to allocate debugtrace buffer for cpu %u\n", cpu);
        else
            printk("failed to allocate debugtrace buffer\n");
        return;
    }

    debugtrace_bytes = PAGE_SIZE << order;
    memset(data, '\0', debugtrace_bytes);
    data->bytes = debugtrace_bytes - sizeof(*data);

    *ptr = data;
}

static int debugtrace_cpu_callback(struct notifier_block *nfb,
                                   unsigned long action, void *hcpu)
{
    unsigned int cpu = (unsigned long)hcpu;

    /* Buffers are only ever allocated, never freed. */
    if ( action == CPU_UP_PREPARE )
        debugtrace_alloc_buffer(&per_cpu(dt_cpu_data, cpu), cpu);

    return 0;
}

static struct notifier_block debugtrace_nfb = {
    .notifier_call = debugtrace_cpu_callback
};

static int __init debugtrace_init(void)
{
    unsigned long bytes;
    unsigned int cpu;

    /* Round size down to next power of two. */
    while ( (bytes = (debugtrace_bytes & (debugtrace_bytes - 1))) != 0 )
        debugtrace_bytes = bytes;

    register_keyhandler('T', debugtrace_key,
                        "toggle debugtrace to console/buffer", 0);

    if ( debugtrace_per_cpu )
    {
        for_each_online_cpu ( cpu )
            debugtrace_alloc_buffer(&per_cpu(dt_cpu_data, cpu), cpu);
        register_cpu_notifier(&debugtrace_nfb);
    }
    else
        debugtrace_alloc_buffer(&dt_data, 0);

    return 0;
}
__initcall(debugtrace_init);

