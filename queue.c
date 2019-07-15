#include "queue.h"


uint8_t queuePush(volatile queue_t* queue, uint8_t newElement)
{
    // Check to make sure the queue isn't full before adding something
    if(queue->quantity >= QUEUE_SIZE)
    {
        // If queue is full, return 0 for failure
        return 0;
    }

    // If the queue isn't full, add the new element to the queue
    queue->elements[queue->head] = newElement;
    queue->head = queue->head + 1;
    queue->head = queue->head % QUEUE_SIZE;
    queue->quantity = queue->quantity + 1;

    // Return 1 for success
    return 1;
}


uint8_t queuePop(volatile queue_t* queue)
{
    // Check to make sure the queue isn't empty before removing something
    if(queue->quantity <= 0)
    {
        // If queue is empty, return 0 for failure
        return 0;
    }

    // If the queue isn't empty, remove the front element from the queue
    uint8_t ret = queue->elements[queue->tail];
    queue->tail = queue->tail + 1;
    queue->tail = queue->tail % QUEUE_SIZE;
    queue->quantity = queue->quantity - 1;
    return ret;
}


uint8_t queueEmpty(volatile queue_t* queue)
{
    // If queue is empty, return 1
    if(queue->quantity <= 0)
    {
        return 1;
    }

    // Otherwise, return 0
    else
    {
        return 0;
    }
}


uint8_t queueFull(volatile queue_t* queue)
{
    // If queue is full, return 1
    if(queue->quantity >= QUEUE_SIZE)
    {
        return 1;
    }

    // Otherwise, return 0
    else
    {
        return 0;
    }
}
