#ifndef THREAD_SUPPORT_H
#define THREAD_SUPPORT_H

typedef int carbon_thread_t;

typedef void *(*thread_func_t)(void *);

typedef struct
{
    SInt32 msg_type;
    thread_func_t func;
    void *arg;
    SInt32 requester;
    core_id_t core_id;
} ThreadSpawnRequest;

typedef struct 
{
    SInt32 msg_type;
    SInt32 sender;
    core_id_t core_id;
} ThreadJoinRequest;

SInt32 CarbonSpawnThread(thread_func_t func, void *arg);
void CarbonJoinThread(SInt32 tid);

#endif
