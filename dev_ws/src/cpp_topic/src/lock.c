#include "atomic_op.h"

// void my_acquire(int *l, lock_t *ll){
//   int r = 1;
//   while(r)
//     r = atomic_or_32(l, 1, ll);
// }

// void my_release(int *l, lock_t *ll){
//   atomic_and_32(l, 0, ll);
// }
