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
            Pub = this ->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            Sub = this-> create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Turtle_bot_mv::turtle_callback,this, std::placeholders::_1));    
            timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Turtle_bot_mv::timer_callback,this));
        }
    private:
        void turtle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            rc = msg->range_min;
            size_t angle = 0;
            bool ck =true;
            bool rg_ck = true;
            for(size_t i=0; i< msg->ranges.size();i++){
                if (msg->ranges[i] > 0.3 && rg_ck) {
                    
                    if(rc< msg->ranges[i] && msg->ranges[i] != INFINITY ){
                        rc = msg->ranges[i];
                        std::cout << i <<"1  :" << rc << std::endl;
                        angle = i;
                    }

                }else if ( msg->ranges[i] <= 0.3 || !rg_ck) {
                    rg_ck = false;
                    if (angle<360 && i<360 && rc< msg->ranges[i] && msg->ranges[i] != INFINITY) {
                        rc = msg->ranges[i];
                        std::cout << i <<"2  :" << rc << std::endl;
                        angle = i;
                        ck = false;
                    }else if (angle>359 && i>359 && ck && rc< msg->ranges[i] && msg->ranges[i] != INFINITY) {
                        rc = msg->ranges[i];
                        std::cout << i <<"3  :" << rc << std::endl;
                        angle = i;
                    }
                }
                // std::cout << i <<":" << msg->ranges[i] << std::endl;
            }
            direction_ = M_PI/2 - (angle/719.0)*M_PI;
            RCLCPP_INFO(this->get_logger(),"%f",direction_);
            
        }
        void timer_callback(){
            vel.linear.x = 0.05;
            vel.angular.z = direction_*0.5;
            Pub->publish(vel);
        }
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Sub;
        geometry_msgs::msg::Twist vel;
        rclcpp::TimerBase::SharedPtr timer;
        float rc = 0.0;
        
        float direction_;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Turtle_bot_mv>());
    rclcpp::shutdown();
}