/*
* foxbot_tool
* heart_beat.cpp
*
* https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
* https://github.com/ros2/demos/blob/master/intra_process_demo/src/two_node_pipeline/two_node_pipeline.cpp

1. build
$ colcon build --symlink-install --parallel-workers 1 --packages-select foxbot_tool
$ . install/setup.bash


*/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class HeartBeat : public rclcpp::Node
{
  public:
    HeartBeat()
    : Node("foxbot_heart_beat"), no_(0)
    {
      //publisher_uint32_ = this->create_publisher<std_msgs::msg::UInt32>("fox_beat", 10);
      publisher_uint32_ = this->create_publisher<std_msgs::msg::UInt32>("pc_beat", 10);   // changed by nishi 2023.3.15
      timer_ = this->create_wall_timer(
        250ms, std::bind(&HeartBeat::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto m_uint32 = std_msgs::msg::UInt32();
      m_uint32.data = no_;
      no_++;
      publisher_uint32_->publish(m_uint32);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_uint32_;
    u_int32_t no_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartBeat>());
  rclcpp::shutdown();
  return 0;
}
