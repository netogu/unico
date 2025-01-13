#ifndef _UNILIB_H_
#define _UNILIB_H_
#include "stddef.h"

/* Memory and Data Structures */

typedef struct {
  void *buffer_ptr;
  size_t element_size;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
} ul_fifo_t;

int ul_fifo_init(ul_fifo_t *self, void *buffer, size_t capacity,
                 size_t element_size);
int ul_fifo_enqueue(ul_fifo_t *self, const void *element);
int ul_fifo_dequeue(ul_fifo_t *self, void *element);
int ul_fifo_clear(ul_fifo_t *self);
size_t ul_fifo_get_linear_size(ul_fifo_t *self);
size_t ul_fifo_count(ul_fifo_t *self);
size_t ul_fifo_capacity(ul_fifo_t *self);

#endif
