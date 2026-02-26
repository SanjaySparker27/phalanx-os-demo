#include "ipc.h"
#include "mm/mm.h"
#include <string.h>

static struct list_head msg_queues;
static uint32_t next_msgq_id = 1;
static spinlock_t msgq_lock;

int msg_init(void)
{
    INIT_LIST_HEAD(&msg_queues);
    spin_lock_init(&msgq_lock);
    return 0;
}

int msgget(uint32_t key, int flags)
{
    struct msg_queue *mq;
    
    spin_lock(&msgq_lock);
    
    list_for_each_entry(mq, &msg_queues, list) {
        if (mq->id == key) {
            spin_unlock(&msgq_lock);
            return mq->id;
        }
    }
    
    if (!(flags & 0x200)) {
        spin_unlock(&msgq_lock);
        return -ENOENT;
    }
    
    mq = kmalloc(sizeof(*mq));
    if (!mq) {
        spin_unlock(&msgq_lock);
        return -ENOMEM;
    }
    
    mq->id = next_msgq_id++;
    mq->permissions = flags & 0x1FF;
    mq->num_bytes = 0;
    mq->num_msgs = 0;
    INIT_LIST_HEAD(&mq->messages);
    spin_lock_init(&mq->lock);
    list_add_tail(&mq->list, &msg_queues);
    
    spin_unlock(&msgq_lock);
    
    return mq->id;
}

int msgsnd(int msqid, const struct msgbuf *msgp, size_t msgsz, int flags)
{
    struct msg_queue *mq;
    struct msg_msg *msg;
    
    if (msgsz > MSG_MAX_SIZE)
        return -EINVAL;
    
    spin_lock(&msgq_lock);
    
    list_for_each_entry(mq, &msg_queues, list) {
        if (mq->id == msqid)
            goto found;
    }
    
    spin_unlock(&msgq_lock);
    return -EINVAL;
    
found:
    spin_lock(&mq->lock);
    spin_unlock(&msgq_lock);
    
    if (mq->num_bytes + msgsz > MSG_QUEUE_MAX * MSG_MAX_SIZE) {
        if (!(flags & 0x800)) {
            spin_unlock(&mq->lock);
            return -EAGAIN;
        }
    }
    
    msg = kmalloc(sizeof(*msg) + msgsz);
    if (!msg) {
        spin_unlock(&mq->lock);
        return -ENOMEM;
    }
    
    msg->mtype = msgp->mtype;
    msg->size = msgsz;
    memcpy(msg->data, msgp->mtext, msgsz);
    INIT_LIST_HEAD(&msg->list);
    
    list_add_tail(&msg->list, &mq->messages);
    mq->num_msgs++;
    mq->num_bytes += msgsz;
    
    spin_unlock(&mq->lock);
    
    return 0;
}

int msgrcv(int msqid, struct msgbuf *msgp, size_t msgsz, long msgtyp, int flags)
{
    struct msg_queue *mq;
    struct msg_msg *msg, *tmp;
    int ret = -EINTR;
    
    spin_lock(&msgq_lock);
    
    list_for_each_entry(mq, &msg_queues, list) {
        if (mq->id == msqid)
            goto found;
    }
    
    spin_unlock(&msgq_lock);
    return -EINVAL;
    
found:
    spin_lock(&mq->lock);
    spin_unlock(&msgq_lock);
    
try_again:
    list_for_each_entry_safe(msg, tmp, &mq->messages, list) {
        if (msgtyp == 0 || msg->mtype == msgtyp || 
            (msgtyp < 0 && msg->mtype <= -msgtyp)) {
            
            size_t copy_size = (msgsz < msg->size) ? msgsz : msg->size;
            
            msgp->mtype = msg->mtype;
            memcpy(msgp->mtext, msg->data, copy_size);
            
            list_del(&msg->list);
            mq->num_msgs--;
            mq->num_bytes -= msg->size;
            
            kfree(msg);
            
            spin_unlock(&mq->lock);
            return copy_size;
        }
    }
    
    if (flags & 0x800) {
        spin_unlock(&mq->lock);
        return -ENOMSG;
    }
    
    spin_unlock(&mq->lock);
    task_sleep(1000);
    spin_lock(&mq->lock);
    goto try_again;
}