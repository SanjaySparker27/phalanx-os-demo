#include "ipc.h"

extern int msg_init(void);
extern int shm_init(void);

int ipc_init(void)
{
    int ret;
    
    ret = msg_init();
    if (ret < 0)
        return ret;
    
    ret = shm_init();
    if (ret < 0)
        return ret;
    
    return 0;
}