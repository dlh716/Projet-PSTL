#ifndef FORCEATLAS_BARRIER_H
#define FORCEATLAS_BARRIER_H

#include <pthread.h>
#include <stdatomic.h>

#include <stdio.h>

/* This data type serves as a synchronization mecanism
 * threads decrease the counter until it is less than 0.
 * Meanwhile, one thread (usually the main thread) waits 
 * for the counter to be less than 0. This thread is then 
 * notified and can continue its execution. 
 * The mutex and condition variable of this struct are
 * destroyed when the waiting thread stops wainting.
*/
struct barrier {
    int counter;
    pthread_mutex_t mtx;
    pthread_cond_t wait;
};

typedef struct barrier* Barrier;

void new_barrier(Barrier b, int c);
void destroy_barrier(Barrier b);

void decrement_barrier(Barrier b, int c);
void wait_barrier(Barrier b);

#endif