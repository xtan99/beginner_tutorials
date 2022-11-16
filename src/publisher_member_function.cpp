#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "beginner_tutorials/srv/print_new_string.hpp"

using namespace std::chrono_literals; // for use of time units: "ms", "s"
using std::placeholders::_1;          // for use with binding Class member
using std::placeholders::_2;          // callback function

// topic types
using STRING    = std_msgs::msg::String;
using PUBLISHER = rclcpp::Publisher<STRING>::SharedPtr;
using TIMER     = rclcpp::TimerBase::SharedPtr;

using SERVICE    = rclcpp::Service<beginner_tutorials::srv::PrintNewString>::SharedPtr;
using PRINTNEWSTRING = beginner_tutorials::srv::PrintNewString;
using REQUEST    = const std::shared_ptr<beginner_tutorials::srv::PrintNewString::Request>;
using RESPONSE    = std::shared_ptr<beginner_tutorials::srv::PrintNewString::Response>;

class MinimalPublisher : public rclcpp::Node {
public:
  //////////////////////////
  // Constructor
  //////////////////////////
  MinimalPublisher() :
    Node("minimal_publisher"),
    m_count_(0) {

    /* 
     * Create publisher with buffer size of 10 and frequency = 2 hz
     */
    auto topicName = "topic";
    m_publisher_ = this->create_publisher<STRING> (topicName, 10);
    auto topicCallbackPtr = std::bind (&MinimalPublisher::timer_callback, this);
    m_timer_ = this->create_wall_timer (500ms, topicCallbackPtr);

    /*
     * Creates a service server (with a service name = "print_new_string")
     */
    auto serviceName = "print_new_string";
    auto serviceCallbackPtr = std::bind (&MinimalPublisher::print_new, this, _1, _2);
    m_service_ = create_service <PRINTNEWSTRING> (serviceName, serviceCallbackPtr);

    this->declare_parameter("my_parameter", "Hello");
    RCLCPP_DEBUG_STREAM (this->get_logger(), "Testing initial parameter");
    RCLCPP_DEBUG_STREAM (this->get_logger(), "my_parameter");
  }

  private:
  //////////////////////////
  // Member Variables:
  //////////////////////////
  size_t    m_count_;
  PUBLISHER m_publisher_;
  TIMER     m_timer_;
  SERVICE   m_service_;
  
  //////////////////////////
  // Member Functions:
  //////////////////////////

  void print_new (REQUEST request,
            RESPONSE response) {
    response->new_string = request->s;
    RCLCPP_INFO_STREAM (this->get_logger(), "Incoming request: " << response->new_string);
    RCLCPP_INFO_STREAM (this->get_logger(), "Sending back response:" <<response->new_string);
    std::vector<rclcpp::Parameter> new_parameter{rclcpp::Parameter("my_parameter", response->new_string)};
    this->set_parameters(new_parameter);
  }

  void timer_callback()  {
    // Create the message to publish
    std::string my_param =
      this->get_parameter("my_parameter").get_parameter_value().get<std::string>();
    while((this->get_parameter("my_parameter").get_parameter_value().get<std::string>()) == "None")
    {
      RCLCPP_FATAL_STREAM (this->get_logger(), "NOT A STRING");
      RCLCPP_FATAL_STREAM (this->get_logger(), "Program shutting down");
      exit(1);
    }

    while((this->get_parameter("my_parameter").get_parameter_value().get<std::string>()) == "Hello")
    {
      RCLCPP_WARN_STREAM (this->get_logger(), "Same string, try another maybe?");
      break;
    }
    auto message = STRING();
    message.data = my_param + std::to_string (m_count_++);
    RCLCPP_INFO_STREAM (this->get_logger(), "Publishing: " << message.data.c_str());
    // Publish the message
    m_publisher_->publish (message);
  }

    
  
};

int main(int argc, char* argv[]) {
  
  // 1.) Initialize ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // 2.) Start processing
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  // 3.) Shutdown ROS 2
  rclcpp::shutdown();
  
  return 0;
}
