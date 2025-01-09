#ifndef _UNILIB_H_
#define _UNILIB_H_
#include "stddef.h"

/* Memory */

void ULIB_memcpy(void *dest, const void *src, size_t n);

typedef struct {
  // private
  int allocated_memory[6];
} ULIB_fifo_t;

int ULIB_fifo_init(ULIB_fifo_t *self, void *buffer, size_t capacity,
                   size_t elementSize);
int ULIB_fifo_put(ULIB_fifo_t *self, void *element);
int ULIB_fifo_get(ULIB_fifo_t *self, void *element);
size_t ULIB_fifo_count(ULIB_fifo_t *self);
size_t ULIB_fifo_capacity(ULIB_fifo_t *self);

#endif
