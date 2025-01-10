#ifndef _UNILIB_H_
#define _UNILIB_H_
#include "stddef.h"

/* Memory */

typedef struct {
  void *buffer;
  size_t elementSize;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
} ul_fifo_t;

int ul_fifo_init(ul_fifo_t *self, void *buffer, size_t capacity,
                 size_t elementSize);
int ul_fifo_put(ul_fifo_t *self, void *element);
int ul_fifo_get(ul_fifo_t *self, void *element);
size_t ul_fifo_count(ul_fifo_t *self);
size_t ul_fifo_capacity(ul_fifo_t *self);

#endif
