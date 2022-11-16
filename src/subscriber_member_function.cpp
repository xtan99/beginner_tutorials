/**
 * @file subscriber_member_function.cpp
 * @author Tanmay (tanmay.n.haldankar99@gmail.com)
 * @brief modified simple subscriber from ROS2 Tutorials
 * @version 0.1
 * @date 2022-11-16
 * 
 * @copyright MIT License (c)
 * 
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Class to create Subscriber node
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Subscriber object
  * 
  */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:

 /**
  * @brief Callback function that looks for and outputs the message it receives from the topic
  * 
  * @param msg mesasge that is received
  */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard:" <<msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
