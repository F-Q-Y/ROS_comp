#include "mem.h"

void * surely_malloc(size_t n) {
  void * p = malloc(n);
  if (!p)
    exit(1);
  return p;
}

// void * memset(void * s, int c, size_t n){
//   int * p = (int *)s;
//   for(size_t i = 0; i < n / 4; i++)
//     p[i] = c;
//   return s;
// }
