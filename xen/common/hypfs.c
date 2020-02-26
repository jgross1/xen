/******************************************************************************
 *
 * hypfs.c
 *
 * Simple sysfs-like file system for the hypervisor.
 */

#include <xen/err.h>
#include <xen/guest_access.h>
#include <xen/hypercall.h>
#include <xen/hypfs.h>
#include <xen/lib.h>
#include <xen/param.h>
#include <xen/rwlock.h>
#include <public/hypfs.h>

#ifdef CONFIG_COMPAT
#include <compat/hypfs.h>
CHECK_hypfs_direntry;
#undef CHECK_hypfs_direntry
#define CHECK_hypfs_direntry struct xen_hypfs_direntry
CHECK_hypfs_dirlistentry;
#endif

#define DIRENTRY_NAME_OFF offsetof(struct xen_hypfs_dirlistentry, name)
#define DIRENTRY_SIZE(name_len) \
    (DIRENTRY_NAME_OFF +        \
     ROUNDUP((name_len) + 1, alignof(struct xen_hypfs_direntry)))

static DEFINE_RWLOCK(hypfs_lock);

HYPFS_DIR_INIT(hypfs_root, "");

static int add_entry(struct hypfs_entry_dir *parent, struct hypfs_entry *new)
{
    int ret = -ENOENT;
    struct hypfs_entry *e;

    write_lock(&hypfs_lock);

    list_for_each_entry ( e, &parent->dirlist, list )
    {
        int cmp = strcmp(e->name, new->name);

        if ( cmp > 0 )
        {
            ret = 0;
            list_add_tail(&new->list, &e->list);
            break;
        }
        if ( cmp == 0 )
        {
            ret = -EEXIST;
            break;
        }
    }

    if ( ret == -ENOENT )
    {
        ret = 0;
        list_add_tail(&new->list, &parent->dirlist);
    }

    if ( !ret )
    {
        unsigned int sz = strlen(new->name);

        parent->e.size += DIRENTRY_SIZE(sz);
    }

    write_unlock(&hypfs_lock);

    return ret;
}

int hypfs_add_dir(struct hypfs_entry_dir *parent,
                  struct hypfs_entry_dir *dir, bool nofault)
{
    int ret;

    ret = add_entry(parent, &dir->e);
    BUG_ON(nofault && ret);

    return ret;
}

int hypfs_add_leaf(struct hypfs_entry_dir *parent,
                   struct hypfs_entry_leaf *leaf, bool nofault)
{
    int ret;

    if ( !leaf->content )
        ret = -EINVAL;
    else
        ret = add_entry(parent, &leaf->e);
    BUG_ON(nofault && ret);

    return ret;
}

static int hypfs_get_path_user(char *buf,
                               XEN_GUEST_HANDLE_PARAM(const_char) uaddr,
                               unsigned long ulen)
{
    if ( ulen > XEN_HYPFS_MAX_PATHLEN )
        return -EINVAL;

    if ( copy_from_guest(buf, uaddr, ulen) )
        return -EFAULT;

    if ( memchr(buf, 0, ulen) != buf + ulen - 1 )
        return -EINVAL;

    return 0;
}

static struct hypfs_entry *hypfs_get_entry_rel(struct hypfs_entry_dir *dir,
                                               const char *path)
{
    const char *end;
    struct hypfs_entry *entry;
    unsigned int name_len;

    if ( dir->e.type != XEN_HYPFS_TYPE_DIR )
        return NULL;

    if ( !*path )
        return &dir->e;

    end = strchr(path, '/');
    if ( !end )
        end = strchr(path, '\0');
    name_len = end - path;

    list_for_each_entry ( entry, &dir->dirlist, list )
    {
        int cmp = strncmp(path, entry->name, name_len);
        struct hypfs_entry_dir *d = container_of(entry,
                                                 struct hypfs_entry_dir, e);

        if ( cmp < 0 )
            return NULL;
        if ( !cmp && strlen(entry->name) == name_len )
            return *end ? hypfs_get_entry_rel(d, end + 1) : entry;
    }

    return NULL;
}

struct hypfs_entry *hypfs_get_entry(const char *path)
{
    if ( path[0] != '/' )
        return NULL;

    return hypfs_get_entry_rel(&hypfs_root, path + 1);
}

int hypfs_read_dir(const struct hypfs_entry *entry,
                   XEN_GUEST_HANDLE_PARAM(void) uaddr)
{
    const struct hypfs_entry_dir *d;
    const struct hypfs_entry *e;
    unsigned int size = entry->size;

    d = container_of(entry, const struct hypfs_entry_dir, e);

    list_for_each_entry ( e, &d->dirlist, list )
    {
        struct xen_hypfs_dirlistentry direntry;
        unsigned int e_namelen = strlen(e->name);
        unsigned int e_len = DIRENTRY_SIZE(e_namelen);

        direntry.e.flags = e->write ? XEN_HYPFS_WRITEABLE : 0;
        direntry.e.type = e->type;
        direntry.e.encoding = e->encoding;
        direntry.e.content_len = e->size;
        direntry.off_next = list_is_last(&e->list, &d->dirlist) ? 0 : e_len;
        if ( copy_to_guest(uaddr, &direntry, 1) )
            return -EFAULT;

        if ( copy_to_guest_offset(uaddr, DIRENTRY_NAME_OFF,
                                  e->name, e_namelen + 1) )
            return -EFAULT;

        guest_handle_add_offset(uaddr, e_len);

        ASSERT(e_len <= size);
        size -= e_len;
    }

    return 0;
}

int hypfs_read_leaf(const struct hypfs_entry *entry,
                    XEN_GUEST_HANDLE_PARAM(void) uaddr)
{
    const struct hypfs_entry_leaf *l;

    l = container_of(entry, const struct hypfs_entry_leaf, e);

    return copy_to_guest(uaddr, l->content, entry->size) ? -EFAULT: 0;
}

static int hypfs_read(const struct hypfs_entry *entry,
                      XEN_GUEST_HANDLE_PARAM(void) uaddr, unsigned long ulen)
{
    struct xen_hypfs_direntry e;
    long ret = -EINVAL;

    if ( ulen < sizeof(e) )
        goto out;

    e.flags = entry->write ? XEN_HYPFS_WRITEABLE : 0;
    e.type = entry->type;
    e.encoding = entry->encoding;
    e.content_len = entry->size;

    ret = -EFAULT;
    if ( copy_to_guest(uaddr, &e, 1) )
        goto out;

    ret = -ENOBUFS;
    if ( ulen < entry->size + sizeof(e) )
        goto out;

    guest_handle_add_offset(uaddr, sizeof(e));

    ret = entry->read(entry, uaddr);

 out:
    return ret;
}

int hypfs_write_leaf(struct hypfs_entry_leaf *leaf,
                     XEN_GUEST_HANDLE_PARAM(void) uaddr, unsigned long ulen)
{
    char *buf;
    int ret;

    if ( ulen > leaf->e.size )
        return -ENOSPC;

    if ( leaf->e.type != XEN_HYPFS_TYPE_STRING &&
         leaf->e.type != XEN_HYPFS_TYPE_BLOB && ulen != leaf->e.size )
        return -EDOM;

    buf = xmalloc_array(char, ulen);
    if ( !buf )
        return -ENOMEM;

    ret = -EFAULT;
    if ( copy_from_guest(buf, uaddr, ulen) )
        goto out;

    ret = -EINVAL;
    if ( leaf->e.type == XEN_HYPFS_TYPE_STRING && !memchr(buf, 0, ulen) )
        goto out;

    ret = 0;
    memcpy(leaf->write_ptr, buf, ulen);

 out:
    xfree(buf);
    return ret;
}

int hypfs_write_bool(struct hypfs_entry_leaf *leaf,
                     XEN_GUEST_HANDLE_PARAM(void) uaddr, unsigned long ulen)
{
    bool buf;

    ASSERT(leaf->e.type == XEN_HYPFS_TYPE_BOOL && leaf->e.size == sizeof(bool));

    if ( ulen != leaf->e.size )
        return -EDOM;

    if ( copy_from_guest(&buf, uaddr, ulen) )
        return -EFAULT;

    *(bool *)leaf->write_ptr = buf;

    return 0;
}

int hypfs_write_custom(struct hypfs_entry_leaf *leaf,
                       XEN_GUEST_HANDLE_PARAM(void) uaddr, unsigned long ulen)
{
    struct param_hypfs *p;
    char *buf;
    int ret;

    buf = xzalloc_array(char, ulen);
    if ( !buf )
        return -ENOMEM;

    ret = -EFAULT;
    if ( copy_from_guest(buf, uaddr, ulen) )
        goto out;

    ret = -EDOM;
    if ( !memchr(buf, 0, ulen) )
        goto out;

    p = container_of(leaf, struct param_hypfs, hypfs);
    ret = p->func(buf);

 out:
    xfree(buf);
    return ret;
}

static int hypfs_write(struct hypfs_entry *entry,
                       XEN_GUEST_HANDLE_PARAM(void) uaddr, unsigned long ulen)
{
    struct hypfs_entry_leaf *l;

    if ( !entry->write )
        return -EACCES;

    l = container_of(entry, struct hypfs_entry_leaf, e);

    return entry->write(l, uaddr, ulen);
}

long do_hypfs_op(unsigned int cmd,
                 XEN_GUEST_HANDLE_PARAM(const_char) arg1, unsigned long arg2,
                 XEN_GUEST_HANDLE_PARAM(void) arg3, unsigned long arg4)
{
    int ret;
    struct hypfs_entry *entry;
    static char path[XEN_HYPFS_MAX_PATHLEN];

    if ( xsm_hypfs_op(XSM_PRIV) )
        return -EPERM;

    if ( cmd == XEN_HYPFS_OP_get_version )
        return XEN_HYPFS_VERSION;

    if ( cmd == XEN_HYPFS_OP_write_contents )
        write_lock(&hypfs_lock);
    else
        read_lock(&hypfs_lock);

    ret = hypfs_get_path_user(path, arg1, arg2);
    if ( ret )
        goto out;

    entry = hypfs_get_entry(path);
    if ( !entry )
    {
        ret = -ENOENT;
        goto out;
    }

    switch ( cmd )
    {
    case XEN_HYPFS_OP_read:
        ret = hypfs_read(entry, arg3, arg4);
        break;

    case XEN_HYPFS_OP_write_contents:
        ret = hypfs_write(entry, arg3, arg4);
        break;

    default:
        ret = -EOPNOTSUPP;
        break;
    }

 out:
    if ( cmd == XEN_HYPFS_OP_write_contents )
        write_unlock(&hypfs_lock);
    else
        read_unlock(&hypfs_lock);

    return ret;
}
