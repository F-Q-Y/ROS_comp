// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int M,N;

//pub int to string(longer)
//print sub_ptr and pub_ptr assert same

using namespace std::chrono_literals;

// Node that produces messages.
struct Producer : public rclcpp::Node
{
  Producer(const std::string & name, const std::string & output)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    counter = 0;
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output,10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        std::ofstream w_out_get_loan,w_out_publish,w_out;
        auto start = std::chrono::system_clock::now();
        auto pub_ptr = captured_pub.lock();
        auto end = std::chrono::system_clock::now();
        if (!pub_ptr) {
          return;
        }
        std::string obj = "M_"+std::to_string(M)+"_N_"+std::to_string(N);
        w_out_get_loan.open("../ans_comp/ans/"+obj+"_get_loan_writer.out",std::ios::out|std::ios::app);
        w_out_publish.open("../ans_comp/ans/"+obj+"_publish_writer.out",std::ios::out|std::ios::app);
        w_out.open("../ans_comp/ans/"+obj+"_sum_writer.out",std::ios::out|std::ios::app);

        //get count
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = count++;

        // printf(
        //   "Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        //   reinterpret_cast<std::uintptr_t>(msg.get()));

        //publish time
        auto start1 = std::chrono::system_clock::now();
        pub_ptr->publish(std::move(msg));
        auto end1 = std::chrono::system_clock::now();

        //time counter
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
        w_out_get_loan.close();
        w_out_publish.close();
        w_out.close();
      };
    timer_ = this->create_wall_timer(20ms, callback);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t counter;
};

// Node that consumes messages.
struct Consumer : public rclcpp::Node
{
  Consumer(const std::string & name, const std::string & input)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input,
      10,
      [](std_msgs::msg::Int32::ConstSharedPtr msg) {
        // printf(
        //   " Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
        //   reinterpret_cast<std::uintptr_t>(msg.get()));
      });
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  
  scanf("%d %d",&M,&N);

  auto producers = new std::shared_ptr<Producer>[M];
  auto consumers = new std::shared_ptr<Consumer>[N];

  for(int i=0;i<M;i++){
      producers[i] = std::make_shared<Producer>("producer"+std::to_string(i), "number");
  }
  for(int j=0;j<N;j++){
      consumers[j] = std::make_shared<Consumer>("consumer"+std::to_string(j), "number");
  }
  
  for(int i=0;i<M;i++){
      executor.add_node(producers[i]);
  }

  for(int j=0;j<N;j++){
      executor.add_node(consumers[j]);
  }

  executor.spin();

  rclcpp::shutdown();

  return 0;
}