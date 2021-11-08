#include "atomic_op.h"

// This could be replaced by an external call.

int atomic_or_32(int * tgt, int v, lock_t * l) {
  int ret, tmp;
  acquire(l);
  ret = *tgt;
  tmp = ret | v;
  *tgt = tmp;
  release(l);
  return ret;
}

int atomic_and_32(int * tgt, int v, lock_t * l) {
  int ret, tmp;
  acquire(l);
  ret = *tgt;
  tmp = ret & v;
  *tgt = tmp;
  release(l);
  return ret;
}

int atomic_exchange_32(int * p, int v, lock_t * l) {
  int ret = *p;
  *p = v;
  return ret;
}

int atomic_compare_swap_32(int * tgt, int exp, int new, lock_t * l) {
  int ret;
  acquire(l);
  ret = *tgt;
  if (ret == exp)
    *tgt = new;
  release(l);
  return ret;
}

int atomic_inc_32(int * p, int v, lock_t * l) {
  int ret = *p;
  *p += v;
  return ret;
}

int atomic_dec_32(int * p, int v, lock_t * l) {
  int ret = *p;
  *p -= v;
  return ret;
}

int atomic_xor_32(int * p, int v, lock_t * l) {
  int ret = *p;
  *p ^= v;
  return ret;
}

int64 atomic_exchange_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p = v;
  return ret;
}

int64 atomic_compare_swap_64(int64 * tgt, int64 exp, int64 new, lock_t * l) {
  long long ret;
  acquire(l);
  ret = *tgt;
  if (ret == exp)
    *tgt = new;
  release(l);
  return new;
}

int64 atomic_inc_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p += v;
  return ret;
}

int64 atomic_dec_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p -= v;
  return ret;
}

int64 atomic_and_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p &= v;
  return ret;
}

int64 atomic_or_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p |= v;
  return ret;
}

int64 atomic_xor_64(int64 * p, int64 v, lock_t * l) {
  long long ret = *p;
  *p ^= v;
  return ret;
}


#define N (4)
static lock_t lock_thread[N];
static struct args_t {
  int wid;
  lock_t * l;
} args[N];
static int * pv;
static lock_t * plock_v;

void * task_or(void * arg) {
  struct args_t * a = (struct args_t *)arg;
  int wid = a->wid;
  lock_t * l = a->l;
  atomic_or_32(pv, (1 << wid), plock_v);
  release((void *)l);
  return (void *)0;
}

int atomic_test() {
  int v = 0;
  lock_t lock_v;
  pv = &v;
  plock_v = &lock_v;
  makelock((void *)&lock_v);
  release((void *)&lock_v);
  for (int w = 0; w < N; w++) {
    lock_t * l = &lock_thread[w];
    makelock((void *)l);
    struct args_t * a = &args[w];
    a->wid = w;
    a->l = l;
    spawn(&task_or, (void *)a);
  }
  for (int w = 0; w < N; w++) {
    lock_t * l = args[w].l;
    acquire((void *)l);
    freelock((void*)l);
  }
  acquire((void *)&lock_v);
  freelock((void *)&lock_v);
  return v;
}
