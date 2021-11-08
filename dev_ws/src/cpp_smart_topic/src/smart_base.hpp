#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <fstream>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

void spawn(void* (*f)(void*), void* args) {
  pthread_t t;
  pthread_create(&t, NULL, f, args);
  pthread_detach(t);
  return;
}

int M;
int N;

std::shared_ptr<int> *writing;
std::mutex * writing_mutex;
std::shared_ptr<int> *last_write;
std::mutex * last_write_mutex;
std::shared_ptr<int> *reading;
std::mutex * reading_mutex;
std::mutex m;

void *surely_malloc (size_t n) {
  void *p = malloc(n);
  if (!p) exit(1);
  return p;
}

void init_topic(){
    writing = new std::shared_ptr<int>[M];
    writing_mutex = new std::mutex[M];
    last_write = new std::shared_ptr<int>[M];
    last_write_mutex = new std::mutex[M];
    reading = new std::shared_ptr<int>[N];
    reading_mutex = new std::mutex[N];
}

void start_write(int w){
    writing_mutex[w].lock();
    writing[w] = std::make_shared<int>(0);//malloc a new buffer
    writing_mutex[w].unlock();
    return;
}

void writer_atomic_load_last(int w){
    writing_mutex[w].lock();
    last_write_mutex[w].lock();
    last_write[w] = std::move(writing[w]);
    last_write_mutex[w].unlock();
    writing_mutex[w].unlock();
}

void finish_write(int w){
    //delete old?
    //last_write release old and get new and new ref++, old ref--
    writer_atomic_load_last(w);
}

void reader_atomic_store(int r,int w){
    reading_mutex[r].lock();
    last_write_mutex[w].lock();
    reading[r] = last_write[w];
    last_write_mutex[w].unlock();
    reading_mutex[r].unlock();
}

void start_read(int r,int w){
    reader_atomic_store(r,w);
}

void reader_atomic_done(int r){
    reading_mutex[r].lock();
    reading[r] = NULL;
    reading_mutex[r].unlock();
}

void finish_read(int r){
    reader_atomic_done(r);
}

void writer_set(int w,int num){
    writing_mutex[w].lock();
    *(writing[w].get()) = num;
    writing_mutex[w].unlock();
}

void *publ(void * arg){
    std::ofstream w_out_get_loan,w_out_publish,w_out;
    int num = 0;
    int w = *(int *)arg;
    std::string obj="M_"+std::to_string(M)+"_N_"+std::to_string(N);
    w_out_get_loan.open("../ans_smart/ans/"+obj+"_get_loan_writer_"+std::to_string(w)+".out",std::ios::out);
    w_out_publish.open("../ans_smart/ans/"+obj+"_publish_writer_"+std::to_string(w)+".out",std::ios::out);
    w_out.open("../ans_smart/ans/"+obj+"_sum_writer_"+std::to_string(w)+".out",std::ios::out);
    while(true){
        auto start = std::chrono::system_clock::now();
        start_write(w);
        auto end = std::chrono::system_clock::now();
        // set value;
        writer_set(w,num);
        auto start1 = std::chrono::system_clock::now();
        finish_write(w);
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
        std::cout << double(average_time_added) <<std::endl;
        num++;
    }
    return NULL;
}

void reader_get(int r,int& value){
    reading_mutex[r].lock();
    if(reading[r]!=NULL){
        value = *(reading[r]);
    }
    else{
        value = 0;
    }
    reading_mutex[r].unlock();
}

void *subs(void * arg){
    int r = *(int *)arg;
    while(true){
        for(int i=0;i<M;i++){
            start_read(r,i);
            int value;
            reader_get(r,value);
            finish_read(r);
        }
    }
    return NULL;
}