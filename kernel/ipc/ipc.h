#ifndef _IPC_H
#define _IPC_H

#include "linux/types.h"
#include "linux/list.h"
#include "linux/spinlock.h"

#define MSG_QUEUE_MAX       32
#define MSG_MAX_SIZE        1024
#define SHM_MAX_REGIONS     16
#define SEM_MAX_COUNT       256

struct msgbuf {
    long mtype;
    char mtext[MSG_MAX_SIZE];
};

struct msg_queue {
    uint32_t id;
    uint32_t permissions;
    struct list_head messages;
    uint32_t num_bytes;
    uint32_t num_msgs;
    spinlock_t lock;
    struct list_head list;
};

struct msg_msg {
    struct list_head list;
    long mtype;
    size_t size;
    char data[];
};

struct shm_region {
    uint32_t id;
    void *addr;
    size_t size;
    uint32_t ref_count;
    uint32_t permissions;
    bool attached;
    spinlock_t lock;
    struct list_head list;
};

struct semaphore {
    int32_t count;
    spinlock_t lock;
    struct list_head wait_list;
};

struct sem_waiter {
    struct list_head list;
    struct task_struct *task;
    bool up;
};

int ipc_init(void);
int msgget(uint32_t key, int flags);
int msgsnd(int msqid, const struct msgbuf *msgp, size_t msgsz, int flags);
int msgrcv(int msqid, struct msgbuf *msgp, size_t msgsz, long msgtyp, int flags);
void *shmget(uint32_t key, size_t size, int flags);
int shmat(uint32_t shmid, void *addr, int flags);
int shmdt(uint32_t shmid);
int sem_init(struct semaphore *sem, int32_t value);
void sem_wait(struct semaphore *sem);
void sem_post(struct semaphore *sem);
bool sem_trywait(struct semaphore *sem);
int sem_getvalue(struct semaphore *sem, int32_t *value);

#endif