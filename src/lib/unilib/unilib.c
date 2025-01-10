#include "unilib.h"
#include <string.h>

int ul_fifo_init(ul_fifo_t *self, void *buffer, size_t capacity,
                 size_t elementSize) {

  if (!self || !buffer || capacity == 0 || elementSize == 0) {
    return -1;
  }

  self->buffer = buffer;
  self->capacity = capacity;
  self->head = 0;
  self->tail = 0;
  self->count = 0;

  return 0;
}

int ul_fifo_get(ul_fifo_t *self, void *element) {

  if (!self || !self->buffer || self->count == 0) {
    return -1;
  }

  void *fifo_head = (char *)self->buffer + (self->head * self->elementSize);
  memcpy(element, fifo_head, self->elementSize);
  self->head = (self->head + 1) % self->capacity;
  self->count--;

  return 0;
}

int ul_fifo_put(ul_fifo_t *self, void *element) {

  if (!self || !self->buffer || self->count == self->capacity) {
    return -1;
  }

  void *fifo_tail = (char *)self->buffer + (self->tail * self->elementSize);
  memcpy(fifo_tail, element, self->elementSize);
  self->tail = (self->tail + 1) % self->capacity;
  self->count++;

  return 0;
}

size_t ul_fifo_count(ul_fifo_t *self) { return self ? self->count : 0; }
size_t ul_fifo_capacity(ul_fifo_t *self) { return self ? self->capacity : 0; }
