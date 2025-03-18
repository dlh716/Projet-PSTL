#include "Barrier.h"


void new_barrier(Barrier b, int c)
{
    b->counter = c;
    pthread_mutex_init(&b->mtx, NULL);
    pthread_cond_init(&b->wait, NULL);
}

void destroy_barrier(Barrier b)
{
    pthread_mutex_destroy(&b->mtx);
    pthread_cond_destroy(&b->wait);
}

void decrement_barrier(Barrier b, int c)
{
    pthread_mutex_lock(&b->mtx);
    b->counter -= c;
    pthread_mutex_unlock(&b->mtx);

    if ( b->counter <= 0 )
    {
        pthread_cond_signal(&b->wait);
    }
}

void wait_barrier(Barrier b)
{
    pthread_mutex_lock(&b->mtx);
    while ( b->counter > 0 )
    {
        pthread_cond_wait(&b->wait, &b->mtx);
    }
    pthread_mutex_unlock(&b->mtx);

    destroy_barrier(b);
}