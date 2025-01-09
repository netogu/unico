#include "unilib.h"
#include <assert.h>

void ULIB_memcpy(void *dest, const void *src, size_t n) {

  unsigned char *d = dest;
  const unsigned char *s = src;

  for (size_t i = 0; i < n; i++) {
    d[i] = s[i];
  }
}

typedef struct {
  void *buffer;
  size_t elementSize;
  size_t capacity;
  size_t head;
  size_t tail;
  size_t count;
} fifo_def_t;

assert(sizeof(fifo_def_t) <= sizeof(ULIB_fifo_t));
// ULIB_fifo_t allocation is too small for fifo_det_t

int ULIB_fifo_init(ULIB_fifo_t *self, void *buffer, size_t capacity,
                   size_t elementSize) {

  if (!self || !buffer || capacity == 0 || elementSize == 0) {
    return -1;
  }

  fifo_def_t *fifo = (fifo_def_t *)self;
  fifo->buffer = buffer;
  fifo->capacity = capacity;
  fifo->head = 0;
  fifo->tail = 0;
  fifo->count = 0;

  return 0;
}

int ULIB_fifo_get(ULIB_fifo_t *self, void *element) {

  fifo_def_t *fifo = (fifo_def_t *)self;

  if (!fifo || !fifo->buffer || fifo->count == 0) {
    return -1;
  }

  void *fifo_head = (char *)fifo->buffer + (fifo->head * fifo->elementSize);
  ULIB_memcpy(element, fifo_head, fifo->elementSize);
  fifo->head = (fifo->head + 1) % fifo->capacity;
  fifo->count--;

  return 0;
}

int ULIB_fifo_put(ULIB_fifo_t *self, void *element) {
  fifo_def_t *fifo = (fifo_def_t *)self;

  if (!fifo || !fifo->buffer || fifo->count == fifo->capacity) {
    return -1;
  }

  void *fifo_tail = (char *)fifo->buffer + (fifo->tail * fifo->elementSize);
  ULIB_memcpy(fifo_tail, element, fifo->elementSize);
  fifo->tail = (fifo->tail + 1) % fifo->capacity;
  fifo->count++;

  return 0;
}

size_t ULIB_fifo_count(ULIB_fifo_t *self) { return self ? self->count : 0; }
size_t ULIB_fifo_capacity(ULIB_fifo_t *self) {
  return self ? self->capacity : 0;
}
