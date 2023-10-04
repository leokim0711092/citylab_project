#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <custom_interfaces/srv/get_direction.hpp>

class TurtleServiceClient: public rclcpp::Node{

    public:
        TurtleServiceClient(): Node("Robot_patrol_service_node"){
            callback_group_ = this->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant);
            auto sub1_ = rclcpp::SubscriptionOptions();
            sub1_.callback_group = callback_group_;

            Pub = this ->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            Sub = this-> create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&TurtleServiceClient::turtle_callback,this, std::placeholders::_1),sub1_);    
            timer = this->create_wall_timer(std::chrono::milliseconds(180), std::bind(&TurtleServiceClient::timer_callback,this));
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
        
        rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Sub;
        geometry_msgs::msg::Twist vel;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        std::shared_ptr<sensor_msgs::msg::LaserScan> last_laser_ ;

        void turtle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            auto request =std::make_shared<custom_interfaces::srv::GetDirection::Request>();
            request->laser_data = *msg;
            last_laser_ = msg;
            client->async_send_request(request, std::bind(&TurtleServiceClient::service_response, this, std::placeholders::_1));
        }

        void service_response(rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture fut){
            auto status = fut.wait_for(std::chrono::seconds(1));
            if(status == std::future_status::ready){
                auto response = fut.get();
                RCLCPP_INFO(this->get_logger(), "Service was called");
                RCLCPP_INFO(this->get_logger(), "Recevied direction: %s", response->direction.c_str());

                if (response->direction == "left") {
                    vel.linear.x = 0.1*response->proximity_scale;
                    vel.angular.z = 0.5;
                }

                if (response->direction == "forward") {
                    vel.linear.x = 0.1*response->proximity_scale;
                    vel.angular.z = 0;
                }
                if (response->direction == "right") {
                    vel.linear.x = 0.1*response->proximity_scale;
                    vel.angular.z = -0.5;
                }
            }
            else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            }
        }

        void timer_callback(){
            RCLCPP_INFO(this->get_logger(),"%f",vel.linear.x);
            Pub->publish(vel);
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);  
    std::shared_ptr<TurtleServiceClient> node = std::make_shared<TurtleServiceClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}