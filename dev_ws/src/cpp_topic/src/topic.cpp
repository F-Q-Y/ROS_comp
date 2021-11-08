#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include "topic_base.hpp"

using namespace std::chrono_literals;

struct Producer : public rclcpp::Node
{
public:
    Producer(const std::string & name, const std::string & output, size_t w)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        count_ = 0;
        w_ = w;
    }
    void run(){
        spawn(&publisher, (void*)&w_);
    }
private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    size_t w_;
};

struct Consumer : public rclcpp::Node
{
public:
    Consumer(const std::string & name, const std::string & input, size_t r)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        r_ = r;
    }
    void run(){
        spawn(&subscriber, (void*)&r_);
    }
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    size_t r_;
};

struct Topic{
public:
    Topic(size_t m=M,size_t n=N){
        producers = new Producer*[m];
        consumers = new Consumer*[n];
        m_ = m;
        n_ = n;
        for(int i=0;i<m;i++){
            producers[i] = new Producer("producer"+std::to_string(i),"number",i);
        }
        for(int j=0;j<n;j++){
            consumers[j] = new Consumer("consumer"+std::to_string(j),"number",j);
        }
        initialize_topic();
    }
    void run(){
        for(int i=0;i<m_;i++){
            producers[i]->run();
        }
        for(int j=0;j<n_;j++){
            consumers[j]->run();
        }
    }
private:
    size_t m_;
    size_t n_;
    Producer** producers;
    Consumer** consumers;
};

int main(int argc, char* argv[]){
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    Topic topic;
    while(true){
        printf("a");
    };
    topic.run();
    // rclcpp::shutdown();
    return 0;
}