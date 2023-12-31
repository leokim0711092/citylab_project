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

class Turtle_bot_mv: public rclcpp::Node{

    public:
        Turtle_bot_mv(): Node("Move_turtlebot_node"){

        
            callback_group_1 = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            callback_group_2 = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            auto sub1_ = rclcpp::SubscriptionOptions();
            sub1_.callback_group = callback_group_1;

            Pub = this ->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            Sub = this-> create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Turtle_bot_mv::turtle_callback,this, std::placeholders::_1),sub1_);    
            timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Turtle_bot_mv::timer_callback,this),callback_group_2);
        }
    private:
        void turtle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            rc = msg->range_min;
            size_t angle = 0;
            bool rg_ck = true;
            const float safe_dist = 0.47;
            for(size_t i=0; i< msg->ranges.size();i++){

                    if (msg->ranges[719-i] > safe_dist && rg_ck) {
                        
                        if(rc< msg->ranges[719-i] && msg->ranges[719-i] != INFINITY ){
                            rc = msg->ranges[719-i];
                            std::cout << i <<"1  :" << rc << std::endl;
                            angle = 719-i;
                        }

                    }
                    else if ( msg->ranges[719-i] <= safe_dist || !rg_ck) {
                    // else if ( msg->range_min <= safe_dist || !rg_ck) {
                        rg_ck = false;
                        if (angle<360 && 719-i<360 && rc< msg->ranges[719-i] && msg->ranges[719-i] != INFINITY) {
                            rc = msg->ranges[719-i];
                            std::cout << i <<"2  :" << rc << std::endl;
                            angle = 719-i;

                        }else if (angle>359 && 719-i>359 && rc< msg->ranges[719-i] && msg->ranges[719-i] != INFINITY) {
                            rc = msg->ranges[719-i];
                            std::cout << i <<"3  :" << rc << std::endl;
                            angle = 719-i;
                        }
                    }
            }
            direction_ = M_PI/2 - (angle/719.0)*M_PI;
            
        }
        void timer_callback(){
            vel.linear.x = 0.1;
            vel.angular.z = direction_*0.5;
            RCLCPP_INFO(this->get_logger(),"%f",vel.angular.z);
            Pub->publish(vel);
        }
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Sub;
        geometry_msgs::msg::Twist vel;
        rclcpp::TimerBase::SharedPtr timer;
        float rc = 0.0;
        rclcpp::CallbackGroup::SharedPtr callback_group_1;
        rclcpp::CallbackGroup::SharedPtr callback_group_2;
        float direction_;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);  
    std::shared_ptr<Turtle_bot_mv> node = std::make_shared<Turtle_bot_mv>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}