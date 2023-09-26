#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Turtle_bot_mv: public rclcpp::Node{

    public:
        Turtle_bot_mv(): Node("Move_turtlebot_node"){
            Pub = this ->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            Sub = this-> create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Turtle_bot_mv::turtle_callback,this, std::placeholders::_1));    
        }
    private:
        void turtle_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            rc = msg->range_min;
            rclcpp::WallRate r(10);
            for(int i=0;i<720;i++){
                if (rc< msg->ranges[i]) {
                    rc = msg->ranges[i];
                    angle = i;
                }
            }
            vel.linear.x = 0.1;
            rad = angle/720.0*M_PI;
            vel.angular.z = rad*0.5;
            Pub->publish(vel);
            r.sleep();
        }
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr Sub;
        geometry_msgs::msg::Twist vel;
        float rc = 0.0;
        int angle;
        float rad;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Turtle_bot_mv>());
    rclcpp::shutdown();
}