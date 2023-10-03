#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "rcl/init_options.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <cstddef>
#include <custom_interfaces/srv/get_direction.hpp>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class TestClient : public rclcpp::Node {

    public:
        TestClient() : Node("test_subscribe_node") {
            
            rclcpp::SubscriptionOptions sub_thread;
            sub_thread.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10, 
            std::bind(&TestClient::sub_callback, this, std::placeholders::_1),sub_thread);

            client = this->create_client<custom_interfaces::srv::GetDirection>(
            "/direction_service");      

            while (!client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "service not available, waiting again...");
            }
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
        rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client;
        std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request;

        void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request;
            auto request =std::make_shared<custom_interfaces::srv::GetDirection::Request>();
            request->laser_data = *msg;

            client->async_send_request(request, std::bind(&TestClient::service_response, this, std::placeholders::_1));
        }
        void service_response(rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture fut){
            auto status = fut.wait_for(std::chrono::seconds(1));
            if(status == std::future_status::ready){
                auto response = fut.get();
                RCLCPP_INFO(this->get_logger(), "Service was called");
                RCLCPP_INFO(this->get_logger(), "Recevied direction: %s", response->direction.c_str());
            }
            else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            }
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestClient>());
    rclcpp::shutdown();
  return 0;
}