#ifndef QUEUE_H_
#define QUEUE_H_


#include "msp.h"
#include "stdint.h"


/* A queue size of 35 bytes is currently selected because the most data we attempt to send at
 * once is 16 data bytes. If all 16 data bytes are a start/stop frame or an escape frame,
 * this size will double to 32 bytes. We also need two more bytes for the start and stop
 * frames, which means the largest transmission we will every make is 34 bytes. It is
 * rounded to 35 because 35 bytes because it is a better number than 34.
 */
#define QUEUE_SIZE  35


typedef struct
{
    uint8_t elements[QUEUE_SIZE];   // Actual data in the queue
    uint8_t head;               // Head of queue
    uint8_t tail;               // Tail of queue
    uint8_t quantity;           // Number of elements in the queue
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
uint8_t queuePush(volatile queue_t* queue, uint8_t newElement);


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
uint8_t queuePop(volatile queue_t* queue);


/**********************************************************************
 * FUNCTION NAME:       queueEmpty
 * FUNCTION PURPOSE:    Checks if a queue is empty
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue
 * OUTPUTS:
 *  -uint8_t            = Returns 1 if empty, 0 if not empty
 *********************************************************************/
uint8_t queueEmpty(volatile queue_t* queue);


/**********************************************************************
 * FUNCTION NAME:       queueFull
 * FUNCTION PURPOSE:    Checks if a queue is full
 *
 * INPUTS:
 *  -queue_t* queue     = A pointer to the queue
 * OUTPUTS:
 *  -uint8_t            = Returns 1 if full, 0 if not full
 *********************************************************************/
uint8_t queueFull(volatile queue_t* queue);


#endif /* QUEUE_H_ */
