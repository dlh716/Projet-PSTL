#include "Pool.h"

void QueueInit(queue q, int capacity)
{
    q->file = (struct Job*) malloc(sizeof(struct Job) * capacity);

    q->capacity = capacity;
    q->size = 0;

    q->tail = capacity - 1;
    q->head = capacity - 1;

    pthread_mutex_init(&q->mtx, NULL);
    pthread_cond_init(&q->isFull, NULL);
    pthread_cond_init(&q->isEmpty, NULL);
    q->isFinished = false;
}

bool Push(queue q, struct Job task)
{
    pthread_mutex_lock(&q->mtx);
    while ( q->size >= q->capacity && ! q->isFinished ) {
        pthread_cond_wait(&q->isFull, &q->mtx);
    }

    bool res = false;
    if ( ! q->isFinished ) {

        q->file[q->head].j = task.j;
        q->file[q->head].args = task.args;

        if ( q->head <= 0 ){
            q->head = q->capacity - 1;
        } else --(q->head);
        ++(q->size);
        res = true;
    }

    pthread_cond_signal (&q->isEmpty);
    pthread_mutex_unlock(&q->mtx); 
    return res;
}

bool Pop(queue q, struct Job* task)
{
    pthread_mutex_lock(&q->mtx);

    while ( ! q->isFinished && q->size <= 0){
        pthread_cond_wait(&q->isEmpty, &q->mtx);
    }
    bool res = false;
    if ( ! (q->isFinished) ) {
        task->j    = q->file[q->tail].j;
        task->args = q->file[q->tail].args;
        
        if ( q->tail <= 0 ) {
            q->tail = q->capacity - 1;
        } else --(q->tail);
        --(q->size);
        res = true;
    }

    pthread_mutex_unlock(&q->mtx);
    pthread_cond_signal (&q->isFull);
    return res;
}

void isFinished(queue q)
{
    q->isFinished = true;

    pthread_cond_broadcast (&q->isEmpty);
}

void QueueFree(queue q) 
{
    free(q->file);
    q->size = 0;
    q->capacity = 0;
    q->head = 0;
    q->tail = 0;

    pthread_mutex_destroy(&q->mtx);
    pthread_cond_destroy(&q->isFull);
    pthread_cond_destroy(&q->isEmpty);
    q->isFinished = true;
}

void * work(void* arg)
{

    thr_Pool pool = (thr_Pool) arg;

    struct Job task;
    while( pool->working )
    {
        if ( Pop(pool->job_queue, &task) ){
            task.j(task.args);
            pool->nb_op += 1;
            free(task.args);
        }
    }
    pthread_exit(NULL);
}

int GetOp(thr_Pool pool)
{
    return pool->nb_op;
}

void InitPool(thr_Pool pool, int job_capacity, int nb_threads)
{
    pool->job_queue = (queue) malloc(sizeof(struct Queue));
    QueueInit(pool->job_queue, job_capacity);

    pool->working = true;
    pool->nb_op = 0;
    pool->nb_threads = nb_threads;
    pool->threads = (pthread_t*) malloc(sizeof(pthread_t) * nb_threads);
    for (int i = 0; i < nb_threads; ++i)
    {
        pthread_create(&pool->threads[i], NULL, work, pool);
    }
}

void submit(thr_Pool pool, struct Job j)
{
    Push(pool->job_queue, j);
}

void FreePool(thr_Pool pool)
{
    pool->working = false;
    isFinished(pool->job_queue);
    for (int i = 0; i < pool->nb_threads; ++i)
    {
        pthread_join(pool->threads[i], NULL);
    }
    free(pool->threads);

    QueueFree(pool->job_queue);
}