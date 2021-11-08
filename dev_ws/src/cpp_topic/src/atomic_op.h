#include <stdlib.h>
#include "extern_def.h"
#include "threads.h"

int atomic_or_32(int * tgt, int v, lock_t * l);
int atomic_and_32(int * tgt, int v, lock_t * l);
int atomic_exchange_32(int * p, int v, lock_t * l);
int atomic_compare_swap_32(int * p, int exp, int des, lock_t * l);
int atomic_inc_32(int * p, int v, lock_t * l);
int atomic_dec_32(int * p, int v, lock_t * l);
int atomic_xor_32(int * p, int v, lock_t * l);

int64 atomic_exchange_64(int64 * p, int64 v, lock_t * l);
int64 atomic_compare_swap_64(int64 * p, int64 exp, int64 des, lock_t * l);
int64 atomic_inc_64(int64 * p, int64 v, lock_t * l);
int64 atomic_dec_64(int64 * p, int64 v, lock_t * l);
int64 atomic_and_64(int64 * p, int64 v, lock_t * l);
int64 atomic_or_64(int64 * p, int64 v, lock_t * l);
int64 atomic_xor_64(int64 * p, int64 v, lock_t * l);
