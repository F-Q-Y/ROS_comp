#include <stdlib.h>
#include <string>
#include <atomic>
#include <chrono>
#include <fstream>
#include "def.h"
#include "mem.h"
#include "atomic_op.h"
#include "bits_op.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

// #define N 2
// #define M 3
extern int N;
extern int M;
#define B (N+2*M)
#define MASK_B (((1) << B) - 1)
#define MASK_N (((1) << N) - 1)
#define Empty (-1)
#define First 0
typedef int buf_id;


static const unsigned Mod37BitPosition[37] = {
  32, 0, 1, 26, 2, 23, 27, 32, 3, 16, 24, 30, 28, 11, 32, 13, 4, 7, 17, 32, 25, 22, 31, 15, 29, 10, 12, 6, 32, 21, 14, 9, 5, 20, 8, 19, 18
};

static const unsigned BitsSetTable256[256] = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};
unsigned get_lowest(unsigned value) {
  unsigned l = -value & value;
  unsigned m = l % 37;
  unsigned res = Mod37BitPosition[m];
  return res;
}

unsigned count_one(unsigned value) {
  unsigned b00 = BitsSetTable256[value & 0xff];
  value = (value >> 8);
  unsigned b08 = BitsSetTable256[value & 0xff];
  value = (value >> 8);
  unsigned b16 = BitsSetTable256[value & 0xff];
  value = (value >> 8);
  unsigned b24 = BitsSetTable256[value];
  return (b00 + b08 + b16 + b24);
}

void spawn(void* (*f)(void*), void* args) {
  pthread_t t;
  pthread_create(&t, NULL, f, args);
  pthread_detach(t);
  return;
}

void *surely_malloc (size_t n) {
  void *p = malloc(n);
  if (!p) exit(1);
  return p;
}

//This will be replaced by an external call eventually.
void *memset(void *s, int c, size_t n){
  int *p = (int *)s;
  for(size_t i = 0; i < n / 4; i++)
    p[i] = c;
  return s;
}

int64 shrl(int64 v, int n) {
  int64 _v;
  _v = v >> n;
  return _v;
}

/* buffer resources */
typedef struct buffer { int data; } buffer;
// buffer *bufs[B];
buffer **bufs;

/* shareable resources */
std::atomic<int> *avail_map; // one bit for each buffer, 0: occupied, 1: available
// std::atomic<int> *read_count[B];
// std::atomic<int> *comm[M];
std::atomic<int> **read_count;
std::atomic<int> **comm;

/* locks for shareable resources */
// lock_t *avail_lock;
// lock_t *rc_lock[B];
// lock_t *comm_lock[M];

/* registrar function */
// buf_id *reading[N];
// buf_id *writing[M];
buf_id **reading;
buf_id **writing;

void initialize_topic() {
  scanf("%d %d",&M,&N);
  bufs = new buffer*[B];
  reading = new buf_id*[N];
  writing = new buf_id*[M];
  read_count = new std::atomic<int>*[B];
  comm = new std::atomic<int>*[M];
  for (int i = 0; i < B; i++) {
    bufs[i] = (buffer*)surely_malloc(sizeof(buffer));
    memset(bufs[i], 0, sizeof(buffer));
  }
  avail_map=(std::atomic<int> *)surely_malloc(sizeof(std::atomic<int>));
  *avail_map = MASK_B - ((1 << M) - 1);
  for (int i = 0; i < B; i++) {
    read_count[i] = (std::atomic<int> *)surely_malloc(sizeof(std::atomic<int>));
    *read_count[i] = (i < M) ? 1 : 0;
  }
  for (int i = 0; i < M; i++) {
    comm[i] = (std::atomic<int>*)surely_malloc(sizeof(std::atomic<int>));
    *comm[i] = i << N;
  }
  // avail_lock = (lock_t*)surely_malloc(sizeof(lock_t));
  // makelock(avail_lock);
  // release(avail_lock);
  // for (int i = 0; i < B; i++) {
  //   rc_lock[i] = (lock_t*)surely_malloc(sizeof(lock_t));
  //   makelock(rc_lock[i]);
  //   release(rc_lock[i]);
  // }
  // for (int i = 0; i < M; i++) {
  //   comm_lock[i] = (lock_t*)surely_malloc(sizeof(lock_t));
  //   makelock(comm_lock[i]);
  //   release(comm_lock[i]);
  // }
  for (int i = 0; i < N; i++) {
    reading[i] = (buf_id*)surely_malloc(sizeof(buf_id));
    *reading[i] = Empty;
  }
  for (int i = 0; i < M; i++) {
    writing[i] = (buf_id*)surely_malloc(sizeof(buf_id));
    *writing[i] = Empty;
  }
}

void avail_push(int b) {
  avail_map->fetch_or(1 << b);
  // atomic_or_32(avail_map, 1 << b, avail_lock);
}

buf_id avail_pop() {
  buf_id b;
  int _v, v, v_; // pre-val, cur-val, post-val
  do {
      v = *avail_map; // atomic load
      b = get_lowest((uint)v);
      if (b >= B)
        exit(1);
      _v = avail_map->fetch_and(~(1<<b));
      // _v = atomic_and_32(avail_map, ~(1 << b), avail_lock);
  } while ((_v & (1 << b)) == 0);
  return b;
}

int rc_set(int b, int v) {
  int _v;
  _v = read_count[b]->exchange(v);
  // _v = atomic_exchange_32(read_count[b], v, rc_lock[b]);
  return _v;
}

int rc_dec(int b, int n) {
  int _v;
  _v = read_count[b]->fetch_sub(n);
  // _v = atomic_dec_32(read_count[b], n, rc_lock[b]);
  return _v;
}

int comm_set(int w, int v) {
  int _v;
  _v = comm[w]->exchange(v);
  // _v = atomic_exchange_32(comm[w], v, comm_lock[w]);
  return _v;
}

int comm_get(int w, int r) {
  int _v;
  _v = comm[w]->fetch_and(~(1<<r));
  // _v = atomic_and_32(comm[w], ~(1 << r), comm_lock[w]);
  return _v;
}

buf_id start_read(int r, int w) {
  // assert (*reading[r] == Empty)
  int mask = 1 << r;
  //
  int _v = comm_get(w, r);
  buf_id b;
  int flag = _v & mask;
  if (flag == 0) {
    b = Empty;
  } else {
    b = _v >> N;
  }
  *reading[r] = b;
  return b;
}

void finish_read(int r) {
  // assert (*reading[r] != Empty);
  buf_id b = *reading[r];
  if (rc_dec(b, 1) == 1)
    avail_push(b);
  *reading[r] = Empty;
}
buf_id get_loan(int w) {
  // assert (*writing[w] == Empty)
  int b = avail_pop();
  *writing[w] = b;
  rc_set(b, N + 1);
  return b;
}
void publish(int w) {
  // assert (*writing[w] != Empty)
  int b = *writing[w];
  int v = (b << N) | MASK_N;
  int _v = comm_set(w, v);
  int _b = (_v >> N);
  int n = count_one(_v & MASK_N) + 1;
  if (rc_dec(_b, n) == n)//g_writer_rdc
    avail_push(_b);
  *writing[w] = Empty;
}

void spin_once(int r) {
  for (int w = 0; w < M; w++) {
    buf_id b = start_read(r, w);
    if (b == Empty)
      continue;
    int v = bufs[b]->data;
    printf("Reader %d read %d\n", r, v);
    finish_read(r);
  }
}

void * subscriber(void * arg) {
  int r = *(int *)arg;
  while (true) {
    spin_once(r);
  }
  printf("reader_message_done:%d\n",r);
  return NULL;
}

void * publisher(void * arg) {
  std::ofstream w_out_get_loan,w_out_publish,w_out;
  int v = 0;
  int w = *(int *)arg;
  w_out_get_loan.open("ans/get_loan/writer_"+std::to_string(w)+".out",std::ios::out);
  w_out_publish.open("ans/publish/writer_"+std::to_string(w)+".out",std::ios::out);
  w_out.open("ans/sum/writer_"+std::to_string(w)+".out",std::ios::out);
  int count = 0;
  while (true) {
    auto start = std::chrono::system_clock::now();
    buf_id b = get_loan(w);
    auto end = std::chrono::system_clock::now();
    bufs[b]->data = v;
    auto start1 = std::chrono::system_clock::now();
    publish(w);
    auto end1 = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    auto all_time = double(duration.count()) * std::chrono::nanoseconds::period::num / std::chrono::nanoseconds::period::den;
    auto average_time_0 = (double)all_time;
    duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - start1);
    all_time = double(duration.count()) * std::chrono::nanoseconds::period::num/std::chrono::nanoseconds::period::den;
    auto average_time_1 = (double)all_time;
    auto average_time_added = (average_time_0+average_time_1);
    w_out_get_loan << double(average_time_0) << std::endl;
    w_out_publish << double(average_time_1) << std::endl;
    w_out << double(average_time_added) << std::endl;
    v++;
  }
  printf("writer_message done:%d\n",w);
  w_out.close();
  return NULL;
}
