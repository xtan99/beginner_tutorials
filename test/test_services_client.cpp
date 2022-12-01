/**
 * @file test_services_client.cpp
 * @author Tanmay Haldankar (tanmay.n.haldankar99@gmail.com)
 * @brief  modified test client file to test service
 * @version 0.1
 * @date 2022-11-30
 *
 * @copyright MIT License (c)
 *
 */

#include <chrono>
#include <memory>

#include "beginner_tutorials/srv/print_new_string.hpp"
#include "gtest/gtest.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef RMW_IMPLEMENTATION
#define CLASSNAME_(NAME, SUFFIX) NAME##__##SUFFIX
#define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
#define CLASSNAME(NAME, SUFFIX) NAME
#endif

using namespace std::chrono_literals;

/**
 * @brief Class to setup test cases for service
 *
 */
class CLASSNAME(test_services_client, RMW_IMPLEMENTATION)
    : public ::testing::Test {
 public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

/**
 * @brief Create test to test service and client
 *
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_add_noreqid) {
  auto node = rclcpp::Node::make_shared("test_services_client_no_reqid");

  auto client = node->create_client<beginner_tutorials::srv::PrintNewString>(
      "add_two_ints_noreqid");
  auto request =
      std::make_shared<beginner_tutorials::srv::PrintNewString::Request>();
  request->s = "Message";

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result,
                                                5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ("Message", result.get()->new_string);
}

/**
 * @brief Create test to test service and client
 *
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_add_reqid) {
  auto node = rclcpp::Node::make_shared("test_services_client_add_reqid");

  auto client = node->create_client<beginner_tutorials::srv::PrintNewString>(
      "add_two_ints_reqid");
  auto request =
      std::make_shared<beginner_tutorials::srv::PrintNewString::Request>();
  request->s = "Message";

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result,
                                                5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ("Message", result.get()->new_string);
}

/**
 * @brief Create test to test return request
 *
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION),
       test_return_request) {
  auto node = rclcpp::Node::make_shared("test_services_client_return_request");

  auto client = node->create_client<beginner_tutorials::srv::PrintNewString>(
      "add_two_ints_reqid_return_request");
  auto request =
      std::make_shared<beginner_tutorials::srv::PrintNewString::Request>();
  request->s = "Message";

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(
      request, [](rclcpp::Client<beginner_tutorials::srv::PrintNewString>::
                      SharedFutureWithRequest future) {
        EXPECT_EQ("Message", future.get().first->s);
        EXPECT_EQ("Message", future.get().second->new_string);
      });

  auto ret = rclcpp::spin_until_future_complete(node, result,
                                                5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
}

/**
 * @brief Create test to test the message being recieved across service and
 * client
 *
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION),
       test_add_two_ints_defered_cb) {
  auto node =
      rclcpp::Node::make_shared("test_services_client_add_two_ints_defered_cb");

  auto client = node->create_client<beginner_tutorials::srv::PrintNewString>(
      "add_two_ints_defered_cb");
  auto request =
      std::make_shared<beginner_tutorials::srv::PrintNewString::Request>();
  request->s = "Message";

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(
      request, [](rclcpp::Client<beginner_tutorials::srv::PrintNewString>::
                      SharedFutureWithRequest future) {
        EXPECT_EQ("Message", future.get().first->s);
        EXPECT_EQ("Message", future.get().second->new_string);
      });

  auto ret = rclcpp::spin_until_future_complete(node, result,
                                                5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
}

/**
 * @brief Create test to test the message being recieved across service and
 * client
 *
 */
TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION),
       test_add_two_ints_defcb_with_handle) {
  auto node = rclcpp::Node::make_shared(
      "test_services_client_add_two_ints_defered_cb_with_handle");

  auto client = node->create_client<beginner_tutorials::srv::PrintNewString>(
      "add_two_ints_defered_cb_with_handle");
  auto request =
      std::make_shared<beginner_tutorials::srv::PrintNewString::Request>();
  request->s = "Message";

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(
      request, [](rclcpp::Client<beginner_tutorials::srv::PrintNewString>::
                      SharedFutureWithRequest future) {
        EXPECT_EQ("Message", future.get().first->s);
        EXPECT_EQ("Message", future.get().second->new_string);
      });

  auto ret = rclcpp::spin_until_future_complete(node, result,
                                                5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
}
