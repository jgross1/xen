/******************************************************************************
 * debugtrace.c
 *
 * Debugtrace for Xen
 */


#include <xen/console.h>
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

static unsigned int debugtrace_kilobytes = 128;
static bool debugtrace_used;
static char debugtrace_last_entry_buf[DEBUG_TRACE_ENTRY_SIZE];
static DEFINE_SPINLOCK(debugtrace_lock);
integer_param("debugtrace", debugtrace_kilobytes);

static void debugtrace_dump_worker(void)
{
    if ( !debugtrace_used )
        return;

    printk("debugtrace_dump() starting\n");

    /* Print oldest portion of the ring. */
    if ( dt_data->buf[dt_data->prd] != '\0' )
        console_serial_puts(&dt_data->buf[dt_data->prd],
                            dt_data->bytes - dt_data->prd);

    /* Print youngest portion of the ring. */
    dt_data->buf[dt_data->prd] = '\0';
    console_serial_puts(&dt_data->buf[0], dt_data->prd);

    memset(dt_data->buf, '\0', dt_data->bytes);
    dt_data->prd = 0;
    debugtrace_last_entry_buf[0] = 0;

    printk("debugtrace_dump() finished\n");
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
    char *p;

    for ( p = buf; *p != '\0'; p++ )
    {
        dt_data->buf[dt_data->prd++] = *p;
        if ( dt_data->prd == dt_data->bytes )
            dt_data->prd = 0;
    }
}

void debugtrace_printk(const char *fmt, ...)
{
    static char buf[DEBUG_TRACE_ENTRY_SIZE];
    static unsigned int count, last_count;
    static unsigned long last_prd;

    char          cntbuf[24];
    va_list       args;
    unsigned long flags;
    unsigned int nr;

    if ( !dt_data )
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
        if ( strcmp(buf, debugtrace_last_entry_buf) )
        {
            last_prd = dt_data->prd;
            last_count = ++count;
            safe_strcpy(debugtrace_last_entry_buf, buf);
            snprintf(cntbuf, sizeof(cntbuf), "%u ", count);
        }
        else
        {
            dt_data->prd = last_prd;
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

static int __init debugtrace_init(void)
{
    int order;
    unsigned long kbytes, bytes;
    struct debugtrace_data *data;

    /* Round size down to next power of two. */
    while ( (kbytes = (debugtrace_kilobytes & (debugtrace_kilobytes-1))) != 0 )
        debugtrace_kilobytes = kbytes;

    bytes = debugtrace_kilobytes << 10;
    if ( bytes == 0 )
        return 0;

    order = get_order_from_bytes(bytes);
    data = alloc_xenheap_pages(order, 0);
    if ( !data )
        return -ENOMEM;

    bytes = PAGE_SIZE << order;
    memset(data, '\0', bytes);

    data->bytes = bytes - sizeof(*data);
    dt_data = data;

    register_keyhandler('T', debugtrace_key,
                        "toggle debugtrace to console/buffer", 0);

    return 0;
}
__initcall(debugtrace_init);

