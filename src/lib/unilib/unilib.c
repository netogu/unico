#include "unilib.h"
#include <string.h>

int ul_fifo_init(ul_fifo_t *self, void *buffer, size_t capacity,
                 size_t elementSize) {

  if (!self || !buffer || capacity == 0 || elementSize == 0) {
    return -1;
  }

  self->buffer_ptr = buffer;
  self->capacity = capacity;
  self->element_size = elementSize;
  self->head = 0;
  self->tail = 0;
  self->count = 0;

  return 0;
}

int ul_fifo_dequeue(ul_fifo_t *self, void *element) {

  if (!self || !self->buffer_ptr) {
    return -1; // error
  }

  if (self->count == 0) {
    return 1; // fifo empty
  }

  void *fifo_head =
      (char *)self->buffer_ptr + (self->head * self->element_size);
  memcpy(element, fifo_head, self->element_size);
  self->head = (self->head + 1) % self->capacity;
  self->count--;

  return 0;
}

size_t ul_fifo_dequeue_n(ul_fifo_t *self, size_t n, void *element) {

  size_t elements_processed = 0;

  if (!self || !self->buffer_ptr || n == 0) {
    return 0; // error
  }
  for (size_t i = 0; i < n; i++) {
    if (ul_fifo_dequeue(self, element + i * self->element_size) == 0) {
      elements_processed++;
    }
  }

  return elements_processed;
}

int ul_fifo_enqueue(ul_fifo_t *self, const void *element) {

  if (!self || !self->buffer_ptr) {
    return -1; // error
  }

  if (self->count == self->capacity) {
    return 1; // fifo is full
  }

  void *fifo_tail =
      (char *)self->buffer_ptr + (self->tail * self->element_size);
  memcpy(fifo_tail, element, self->element_size);
  self->tail = (self->tail + 1) % self->capacity;
  self->count++;

  return 0;
}

int ul_fifo_enqueue_n(ul_fifo_t *self, size_t n, const void *element) {
  size_t elements_processed = 0;

  if (!self || !self->buffer_ptr || n == 0) {
    return 0; // error
  }
  for (size_t i = 0; i < n; i++) {
    if (ul_fifo_enqueue(self, element + (i * self->element_size)) == 0) {
      elements_processed++;
    }
  }

  return elements_processed;
}

size_t ul_fifo_count(ul_fifo_t *self) { return self ? self->count : 0; }
size_t ul_fifo_capacity(ul_fifo_t *self) { return self ? self->capacity : 0; }
