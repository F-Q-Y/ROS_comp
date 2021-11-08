#include <iostream>
#include <stdlib.h>
#include <string>
#include <atomic>
#include <chrono>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
typedef unsigned uint;
typedef long long int64;
typedef unsigned long long uint64;

//general data
// #define N 3 //number of readers
#define B (N + 2) //number of buffers
#define Empty (100)
#define First 0
typedef int buf_id;
typedef struct buffer {int data;} buffer;
int N,M;

void spawn(void* (*f)(void*), void* args) {
        pthread_t t;
        pthread_create(&t, NULL, f, args);
        pthread_detach(t);
        return;
}

class topic_base{
public:
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

    buffer **bufs;
    // lock_t *lock[N];
    std::atomic<buf_id> **comm;

    //registrar function
    std::atomic<buf_id> **reading, **last_read;

    void initialize_channels(){
        bufs = new buffer*[B];
        comm = new std::atomic<buf_id>*[N];
        reading = new std::atomic<buf_id>*[N];
        last_read = new std::atomic<buf_id>*[N];
        for(int i = 0; i < B; i++){
            buffer *b = (buffer*)surely_malloc(sizeof(buffer));
            memset(b, 0, sizeof(buffer));
            bufs[i] = b;
        }
        for(int r = 0; r < N; r++){
            std::atomic<buf_id> *c = (std::atomic<buf_id>*)surely_malloc(sizeof(std::atomic<buf_id>));
            *c = First;
            comm[r] = c;
            c = (std::atomic<buf_id>*)surely_malloc(sizeof(std::atomic<buf_id>));
            reading[r]=c;
            c = (std::atomic<buf_id>*)surely_malloc(sizeof(std::atomic<buf_id>));
            last_read[r] = c;
            // lock_t *l = surely_malloc(sizeof(lock_t));
            // lock[r] = l;
            // makelock(l);
            // release(l);
        }
    }

    //reader functions
    void initialize_reader(int r){
        std::atomic<buf_id> *rr = reading[r];
        std::atomic<buf_id> *lr = last_read[r];
        rr->exchange(Empty);
        lr->exchange(1);
    }

    buf_id start_read(int r){
        buf_id b;
        std::atomic<buf_id> *c = comm[r];
        //   lock_t *l = lock[r];
        std::atomic<buf_id> *rr = reading[r];
        std::atomic<buf_id> *lr = last_read[r];
        b = c->exchange(Empty);
        //   b = std::atomic_exchange(c, l, Empty);
        if(b >= 0 && b < B)
            *lr = b;
        else
            b = *lr;
        *rr = b;
        return b;
    }

    void finish_read(int r){
    //r is no longer using the buffer
    std::atomic<buf_id> *rr = reading[r];
    *rr = Empty;
    }

    //writer functions
    buf_id *last_taken;
    buf_id writing, last_given;

    void initialize_writer(){
        last_taken = new buf_id[N];
        last_given = First;
        writing = Empty;
        for(int i = 0; i < N; i++)
            last_taken[i] = 1;
    }

    buf_id start_write(){
        //get a buffer not in use by any reader
        int available[B];
        for(int i = 0; i < B; i++)
            available[i] = 1;
        buf_id last = last_given;
        available[last] = 0;
        for(int r = 0; r < N; r++){
            last = last_taken[r];
            if(last != Empty)
            available[last] = 0;
        }
        for(int i = 0; i < B; i++){
            int avail = available[i];
            if(avail){
            writing = i;
            return i;
            }
        }
        exit(1); //no available buffer, should be impossible
    }

    void finish_write(){
        //make current buffer available to all readers
        buf_id last = last_given;
        buf_id w = writing;
        for(int r = 0; r < N; r++){
            std::atomic<buf_id> *c = comm[r];
            // lock_t *l = lock[r];
            buf_id b = c->exchange(w);
            // buf_id b = simulate_atomic_exchange(c, l, w);
            if(b == Empty)
            last_taken[r] = last;
        }
        last_given = w;
        writing = Empty;
    }
};

topic_base *T;

void init_topics(){
    T = new topic_base[M];
    for(int i=0;i<M;i++){
        T[i].initialize_channels();
    }
}

void *reader(void *arg){
    int r = *(int *)arg;
    for(int i=0;i<M;i++){
        T[i].initialize_reader(r);
    }
    while(true){
        for(int i=0;i<M;i++){
            buf_id b = T[i].start_read(r);
            buffer *buf = T[i].bufs[b];
            int v=buf->data;
            T[i].finish_read(r);
        }
    }
    return NULL;
}

void *writer(void *arg){
    std::ofstream w_out_get_loan,w_out_publish,w_out;
    int w =*(int*)arg;
    std::string obj="M_"+std::to_string(M)+"_N_"+std::to_string(N);
    w_out_get_loan.open("../ans_mail/ans/"+obj+"_get_loan_writer_"+std::to_string(w)+".out",std::ios::out);
    w_out_publish.open("../ans_mail/ans/"+obj+"_publish_writer_"+std::to_string(w)+".out",std::ios::out);
    w_out.open("../ans_mail/ans/"+obj+"_sum_writer_"+std::to_string(w)+".out",std::ios::out);
    T[w].initialize_writer();
    unsigned v = 0;
    while(true){
        auto start = std::chrono::system_clock::now();
        buf_id b = T[w].start_write();
        auto end = std::chrono::system_clock::now();
        buffer *buf = T[w].bufs[b];
        buf->data = v;
        auto start1 = std::chrono::system_clock::now();
        T[w].finish_write();
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
        std::cout << double(average_time_added) << std::endl;
        v++;
    }
    return NULL;
}


