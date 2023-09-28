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

struct ray_len{
    int fst;
    int lst;
};

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
            bool rg_ck = false;
            std::vector<int> str;
            // bool middle_ck =true;
            const float sf = 0.6;
            for(int i= 719 ; i >= 0;i--){
                std::cout << i <<": "<< msg->ranges[i] << std::endl; 
                    if (msg->ranges[i] > sf || rg_ck) {
                        
                        if(rc< msg->ranges[i] && msg->ranges[i] != INFINITY){
                            rc = msg->ranges[i];
                            angle = i;
                        }

                    }else if ( msg->ranges[i] <= sf && !rg_ck) {
                        rg_ck = false;
                        str.push_back(i);
                    }
            }
            
            if(!rg_ck){
                int rcrd[720];
                for(int j=0;j<720;j++) rcrd[j] =0;
                for(size_t i=0;i<str.size();i++){
                    rcrd[str[i]] = 1;
                }
               std::vector<int> len_number_store[720]; //record the number of each start and end
               int record_len = 0; // record the length
               int record_hole_amount =0;
               // know the direction
               for(int j=0;j<720;j++){
                
                    if(rcrd[j] == 0 && record_len ==0 && j!= 719){
                        len_number_store[record_hole_amount].push_back(j);
                        record_len++;
                    }else if (rcrd[j] == 0 && j!= 719) record_len++; 
                    else if (rcrd[j] ==1 && record_len > 0) {
                        len_number_store[record_hole_amount].push_back(j-1);
                        // std::cout <<"record_amout: " << record_hole_amount << std::endl;
                        // std::cout << "j: " << j-1<< std::endl;
                        record_hole_amount++;
                        record_len = 0;
                    }else if (j == 719 && record_len >0) {
                        len_number_store[record_hole_amount].push_back(j);
                        record_len++;
                        // std::cout <<"record_amout: " << record_hole_amount << std::endl;
                        // std::cout << "j: " << j<< std::endl;
                        record_hole_amount++;
                        record_len = 0;
                    }
               }
               ray_len ang;
               ang = Turtle_bot_mv::direction(len_number_store);
               angle = acc_direction(ang, msg);  
               std::cout << "angle: "<< angle << std::endl;
            }
            direction_ = -M_PI/2 + (angle/719.0)*M_PI;
        }
        void timer_callback(){
            vel.linear.x = 0.1;
            vel.angular.z = direction_*0.5;
            RCLCPP_INFO(this->get_logger(),"%f",vel.angular.z);
            Pub->publish(vel);
        }
        ray_len direction(std::vector<int> len_store[10]){
            int sz=0;
            int hole_size_final=0;
            int hole_size =0;
            ray_len ang;
            int vec_number =0;
                while(!len_store[sz].empty()){

                    hole_size = len_store[sz][1] -len_store[sz][0];
                    if(hole_size_final < hole_size){
                        hole_size_final = hole_size;
                        vec_number = sz;
                    }
                    std::cout << "hole size: "<<hole_size_final <<" sz: "<< vec_number <<std::endl;
                    sz++;
                    ang.fst = len_store[vec_number][0]; 
                    ang.lst = len_store[vec_number][1];
                }
            return ang;
        }
        int acc_direction(ray_len ang, const sensor_msgs::msg::LaserScan::SharedPtr msg){
            int angle = 0;
            float rr = msg->range_min;
            for(int i = ang.fst; i<= ang.lst;i++){        
                if(rr< msg->ranges[i] && msg->ranges[i] != INFINITY){
                    rr = msg->ranges[i];
                    angle = i;
                }
            }
            return angle;
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