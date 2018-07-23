#ifndef _LINUX_INIT_H
#define _LINUX_INIT_H

#include <asm/init.h>

/*
 * Mark functions and data as being only used at initialization
 * or exit time.
 */
#define __init            __text_section(".init.text")
#define __exit            __text_section(".exit.text")
#define __initdata        __section(".init.data")
#define __initconst       __section(".init.rodata")
#define __initconstrel    __section(".init.rodata.rel")
#define __exitdata        __used_section(".exit.data")
#define __initsetup       __used_section(".init.setup")
#define __init_call(lvl)  __used_section(".initcall" lvl ".init")
#define __exit_call       __used_section(".exitcall.exit")

/* These macros are used to mark some functions or 
 * initialized data (doesn't apply to uninitialized data)
 * as `initialization' functions. The kernel can take this
 * as hint that the function is used only during the initialization
 * phase and free up used memory resources after
 *
 * Usage:
 * For functions:
 * 
 * You should add __init immediately before the function name, like:
 *
 * static void __init initme(int x, int y)
 * {
 *    extern int z; z = x * y;
 * }
 *
 * If the function has a prototype somewhere, you can also add
 * __init between closing brace of the prototype and semicolon:
 *
 * extern int initialize_foobar_device(int, int, int) __init;
 *
 * For initialized data:
 * You should insert __initdata between the variable name and equal
 * sign followed by value, e.g.:
 *
 * static int init_variable __initdata = 0;
 * static char linux_logo[] __initdata = { 0x32, 0x36, ... };
 *
 * Don't forget to initialize data not at file scope, i.e. within a function,
 * as gcc otherwise puts the data into the bss section and not into the init
 * section.
 * 
 * Also note, that this data cannot be "const".
 */

#ifndef __ASSEMBLY__

struct cpupool;
struct domain;

/*
 * Used for initialization calls..
 */
typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

#define presmp_initcall(fn) \
    const static initcall_t __initcall_##fn __init_call("presmp") = fn
#define __initcall(fn) \
    const static initcall_t __initcall_##fn __init_call("1") = fn
#define __exitcall(fn) \
    static exitcall_t __exitcall_##fn __exit_call = fn

void do_presmp_initcalls(void);
void do_initcalls(void);

/*
 * Used for kernel command line parameter setup
 */
enum param_scope {
    SCOPE_GLOBAL,
    SCOPE_CPUPOOL,
    SCOPE_DOMAIN
};

struct kernel_param {
    const char *name;
    enum {
        OPT_STR,
        OPT_UINT,
        OPT_BOOL,
        OPT_SIZE,
        OPT_CUSTOM
    } type;
    unsigned int len;
    enum param_scope scope;
    unsigned int flags;
#define PARAM_FLAG_RUNTIME    0x80000000
    union {
        void *var;
        int (*func)(const char *);
        int (*func_cpupool)(const char *, struct cpupool *);
        int (*func_domain)(const char *, struct domain *);
        int (*call)(const char *, void *);
    } par;
};

extern const struct kernel_param __setup_start[], __setup_end[];
extern const struct kernel_param __param_start[], __param_end[];

#define __dataparam       __used_section(".data.param")

#define __param(att)      static const att \
    __attribute__((__aligned__(sizeof(void *)))) struct kernel_param

#define __setup_str static const __initconst \
    __attribute__((__aligned__(1))) char
#define __kparam          __param(__initsetup)

#define def_custom_param(_name, _scope, _flags, _ptr, _func) \
    { .name = _name, \
      .type = OPT_CUSTOM, \
      .scope = _scope, \
      .flags = _flags, \
      .par._ptr = _func }
#define def_var_param(_name, _type, _scope, _flags, _var) \
    { .name = _name, \
      .type = _type, \
      .len = sizeof(_var), \
      .scope = _scope, \
      .flags = _flags, \
      .par.var = &_var }

#define custom_param(_name, _var) \
    __setup_str __setup_str_##_var[] = _name; \
    __kparam __setup_##_var = \
        def_custom_param(__setup_str_##_var, SCOPE_GLOBAL, 0, func, _var)
#define boolean_param(_name, _var) \
    __setup_str __setup_str_##_var[] = _name; \
    __kparam __setup_##_var = \
        def_var_param(__setup_str_##_var, OPT_BOOL, SCOPE_GLOBAL, 0, _var)
#define integer_param(_name, _var) \
    __setup_str __setup_str_##_var[] = _name; \
    __kparam __setup_##_var = \
        def_var_param(__setup_str_##_var, OPT_UINT, SCOPE_GLOBAL, 0, _var)
#define size_param(_name, _var) \
    __setup_str __setup_str_##_var[] = _name; \
    __kparam __setup_##_var = \
        def_var_param(__setup_str_##_var, OPT_SIZE, SCOPE_GLOBAL, 0, _var)
#define string_param(_name, _var) \
    __setup_str __setup_str_##_var[] = _name; \
    __kparam __setup_##_var = \
        def_var_param(__setup_str_##_var, OPT_STR, SCOPE_GLOBAL, 0, _var)

#define __rtparam         __param(__dataparam)

#define custom_runtime_only_param(_name, _var) \
    __rtparam __rtpar_##_var = \
        def_custom_param(_name, SCOPE_GLOBAL, 0, func, _var)
#define boolean_runtime_only_param(_name, _var) \
    __rtparam __rtpar_##_var = \
        def_var_param(_name, OPT_BOOL, SCOPE_GLOBAL, 0, _var)
#define integer_runtime_only_param(_name, _var) \
    __rtparam __rtpar_##_var = \
        def_var_param(_name, OPT_UINT, SCOPE_GLOBAL, 0, _var)
#define size_runtime_only_param(_name, _var) \
    __rtparam __rtpar_##_var = \
        def_var_param(_name, OPT_SIZE, SCOPE_GLOBAL, 0, _var)
#define string_runtime_only_param(_name, _var) \
    __rtparam __rtpar_##_var = \
        def_var_param(_name, OPT_STR, SCOPE_GLOBAL, 0, _var)

#define custom_runtime_param(_name, _var) \
    custom_param(_name, _var); \
    custom_runtime_only_param(_name, _var)
#define boolean_runtime_param(_name, _var) \
    boolean_param(_name, _var); \
    boolean_runtime_only_param(_name, _var)
#define integer_runtime_param(_name, _var) \
    integer_param(_name, _var); \
    integer_runtime_only_param(_name, _var)
#define size_runtime_param(_name, _var) \
    size_param(_name, _var); \
    size_runtime_only_param(_name, _var)
#define string_runtime_param(_name, _var) \
    string_param(_name, _var); \
    string_runtime_only_param(_name, _var)

#define custom_cpupool_param(_name, _flags, _var) \
    __rtparam __cpupool_par_##_var = \
        def_custom_param(_name, SCOPE_CPUPOOL, _flags, func_cpupool, _var)
#define boolean_cpupool_param(_name, _flags, _var) \
    __rtparam __cpupool_par_##_var = \
        def_var_param(_name, OPT_BOOL, SCOPE_CPUPOOL, _flags, \
                      (struct cpupool *)NULL->_var)
#define integer_cpupool_param(_name, _flags, _var) \
    __rtparam __cpupool_par_##_var = \
        def_var_param(_name, OPT_UINT, SCOPE_CPUPOOL, _flags, \
                      (struct cpupool *)NULL->_var)
#define size_cpupool_param(_name, _flags, _var) \
    __rtparam __cpupool_par_##_var = \
        def_var_param(_name, OPT_SIZE, SCOPE_CPUPOOL, _flags, \
                      (struct cpupool *)NULL->_var)
#define string_cpupool_param(_name, _flags, _var) \
    __rtparam __cpupool_par_##_var = \
        def_var_param(_name, OPT_STR, SCOPE_CPUPOOL, _flags, \
                      (struct cpupool *)NULL->_var)

#define custom_domain_param(_name, _flags, _var) \
    __rtparam __domain_par_##_var = \
        def_custom_param(_name, SCOPE_DOMAIN, _flags, func_domain, _var)
#define boolean_domain_param(_name, _flags, _var) \
    __rtparam __domain_par_##_var = \
        def_var_param(_name, OPT_BOOL, SCOPE_DOMAIN, _flags, \
                      (struct domain *)NULL->_var)
#define integer_domain_param(_name, _flags, _var) \
    __rtparam __domain_par_##_var = \
        def_var_param(_name, OPT_UINT, SCOPE_DOMAIN, _flags, \
                      (struct domain *)NULL->_var)
#define size_domain_param(_name, _flags, _var) \
    __rtparam __domain_par_##_var = \
        def_var_param(_name, OPT_SIZE, SCOPE_DOMAIN, _flags, \
                      (struct domain *)NULL->_var)
#define string_domain_param(_name, _flags, _var) \
    __rtparam __domain_par_##_var = \
        def_var_param(_name, OPT_STR, SCOPE_DOMAIN, _flags, \
                      (struct domain *)NULL->_var)

#endif /* __ASSEMBLY__ */

#ifdef CONFIG_LATE_HWDOM
#define __hwdom_init
#define __hwdom_initdata  __read_mostly
#else
#define __hwdom_init      __init
#define __hwdom_initdata  __initdata
#endif

#endif /* _LINUX_INIT_H */
