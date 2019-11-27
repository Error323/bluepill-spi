#include "utils.h"

void SwapArrayU16(uint16_t **a, uint16_t **b) {
  uint16_t *tmp = *a;
  *a = *b;
  *b = tmp;
}
