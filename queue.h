#ifndef QUEUE_H_
#define QUEUE_H_


#include "main.h"


#define QUEUE_SIZE  20


typedef struct
{
    uint8_t elements[STACK_SIZE];   // Actual data in the queue
    uint8_t head;                   // Head of queue
    uint8_t tail;                   // Tail of queue
    uint8_t quantity;               // Number of elements in the queue
} queue_t;


/**********************************************************************
 * FUNCTION NAME:       queuePush
 * FUNCTION PURPOSE:    Adds an element to the back of a queue
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue being added to
 *  -uint8_t newElement = The new item being added to the queue
 * OUTPUTS:
 *  -uint8_t            = 1 or 0, returns 0 on failure and 1 on success
 *********************************************************************/
uint8_t queuePush(queue_t* queue, uint8_t newElement);


/**********************************************************************
 * FUNCTION NAME:       queuePop
 * FUNCTION PURPOSE:    Removes and returns the front value of a queue
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue being removed from
 * OUTPUTS:
 *  -uint8_t            = The front element of the queue which was
 *                        removed, returns 0 on failure
 *********************************************************************/
uint8_t queuePop(queue_t* queue);


/**********************************************************************
 * FUNCTION NAME:       queueEmpty
 * FUNCTION PURPOSE:    Checks if a queue is empty
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue
 * OUTPUTS:
 *  -uint8_t            = Returns 1 if empty, 0 if not empty
 *********************************************************************/
uint8_t queueEmpty(queue_t* queue);


/**********************************************************************
 * FUNCTION NAME:       queueFull
 * FUNCTION PURPOSE:    Checks if a queue is full
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue
 * OUTPUTS:
 *  -uint8_t            = Returns 1 if full, 0 if not full
 *********************************************************************/
uint8_t queueFull(queue_t* queue);


#endif /* QUEUE_H_ */
