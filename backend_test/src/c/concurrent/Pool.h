#ifndef FORCEATLAS_POOL_H
#define FORCEATLAS_POOL_H

#include <stdbool.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// Job Queue
typedef void (*job)(void*);

struct Job {
    job j;
    void* args;
};

struct Queue{

    int capacity, size;
    int head, tail;
    struct Job* file;

    pthread_mutex_t mtx;
    pthread_cond_t isFull, isEmpty;
    bool isFinished;

};

typedef struct Queue* queue;

extern void QueueInit(queue q, int capacity);
extern bool Push(queue q, struct Job j);
extern bool Pop(queue q, struct Job* j);
extern void isFinished(queue q);

extern void QueueFree(queue q);

// Thread Pool
struct Pool {

    _Atomic int nb_op;
    queue      job_queue;

    int nb_threads;
    pthread_t* threads;

    bool working;
    
};

typedef struct Pool* thr_Pool;

void InitPool(thr_Pool, int, int);
void submit(thr_Pool, struct Job task);
int GetOp(thr_Pool);

void * work(void* arg);

void FreePool(thr_Pool);

#endif