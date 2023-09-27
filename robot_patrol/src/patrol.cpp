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
            int angle = 0;
            bool ck =true;
            bool rg_ck = false;
            std::vector<int> str;
            // bool middle_ck =true;
            const float sf = 0.45;
            for(int i= 719 ; i >= 0;i--){
                    if (msg->ranges[i] > sf || rg_ck) {
                        
                        if(rc< msg->ranges[i] && msg->ranges[i] != INFINITY){
                            rc = msg->ranges[i];
                            std::cout << "1  "<<i <<":" << rc << std::endl;
                            angle = i;
                        }

                    }else if ( msg->ranges[i] <= sf && !rg_ck) {
                        rg_ck = false;
                        str.push_back(i);
                    }
            }
            direction_ = -M_PI/2 + (angle/719.0)*M_PI;
            if(!rg_ck){
                int rcrd[720];
                for(int j=0;j<720;j++) rcrd[j] =0;
                for(int i=0;i<str.size();i++){
                    rcrd[str[i]] = 1;
                }
               std::vector<int> len_number_store[10]; //record the number of each start and end
               int record_len = 0; // record the length
               int record_hole_amount =0;
               // know the direction
               for(int j=0;j<720;j++){

                    if(rcrd[j] == 1 && record_len ==0){
                        len_number_store[record_hole_amount].push_back(j);
                        record_len++;
                    }else if (rcrd[j] ==1 ) record_len++; 
                    else if (rcrd[j] ==0 && record_len > 0) {
                        len_number_store[record_hole_amount].push_back(j-1);
                        record_hole_amount++;
                        record_len = 0;
                    }
               }
                angle = Turtle_bot_mv::direction(len_number_store)*2;
            }
            
        }
        void timer_callback(){
            vel.linear.x = 0.1;
            vel.angular.z = direction_*0.5;
            RCLCPP_INFO(this->get_logger(),"%f",vel.angular.z);
            Pub->publish(vel);
        }
        int direction(std::vector<int> len_store[10]){
            int sz=0;
            int hole_size=0;
            int ang = 0;
            int vec_number =0;
                while(!len_number_store[++sz].empty()){

                    hole_size = len_number_store[sz-1][1] -len_number_store[sz-1][0];
                    if(hole_size < len_number_store[sz][1] -len_number_store[sz][0]){
                        hole_size = len_number_store[sz][1] -len_number_store[sz][0];
                        vec_number = sz;
                    }
                    ang = (len_number_store[vec_number][1] + len_number_store[vec_number][0])/2
                }
            return ang;
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