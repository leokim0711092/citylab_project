#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "rcl/init_options.h"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

struct response_struct{
    std::string direction;
    float proximity_scale;
};

class DirectionService :public rclcpp::Node{

    public:
        DirectionService(): Node("Direction_service_node"){

            Srv_ = this->create_service<custom_interfaces::srv::GetDirection>("/direction_service",std::bind(&DirectionService::direction_callback,this, std::placeholders::_1,std::placeholders::_2));        
        }
    
    private:
        rclcpp::Service<custom_interfaces::srv::GetDirection>::SharedPtr Srv_;

        float total_dist_sec_right ;
        float total_dist_sec_front ;
        float total_dist_sec_left ;

        void direction_callback(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> Req,
        const std::shared_ptr<custom_interfaces::srv::GetDirection::Response> Res){
            
            Res->direction = find_direction(Req).direction;
            Res->proximity_scale = find_direction(Req).proximity_scale;
        }

        response_struct find_direction(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> Req){
            
            total_dist_sec_right = 0.0 ;
            total_dist_sec_front = 0.0;
            total_dist_sec_left = 0.0;
            int cal_right =0;
            int cal_front = 0;
            int cal_left = 0;
            float threshold = 0.4;
            response_struct resp;
            int consecutive_obstacle_left;
            int consecutive_obstacle_right;
            int consecutive_obstacle_front;
            bool right = true;
            bool left = true;
            bool front = true;
            int obstacle_size = 18;
            int front_obstacle_size = 5;
            float allow_maximum_speed_distance = 1.6;
            float allow_maximum_speed_distance_front = 1;
            bool determine_left = true;   
            bool determine_right = true;

            for(int i = 0 ; i<310;i++){

                if(!std::isinf(Req->laser_data.ranges[i])){
                total_dist_sec_right += Req->laser_data.ranges[i]; 
                cal_right++;
                }
                if (Req->laser_data.ranges[i] < threshold) {
                    consecutive_obstacle_right++;
                    if (consecutive_obstacle_right >= obstacle_size) {
                        right = false;
                    }
                }else {
                    consecutive_obstacle_right =0;
                }
            }
            
            for(int i = 310; i<410;i++){
                if(!std::isinf(Req->laser_data.ranges[i])){
                total_dist_sec_front += Req->laser_data.ranges[i]; 
                cal_front++;
                }
                if (Req->laser_data.ranges[i] < threshold) {
                    consecutive_obstacle_front++;
                    if (consecutive_obstacle_front >= front_obstacle_size) {
                        front = false;
                    }
                }else {
                    consecutive_obstacle_front =0;
                }
            }

            for(int i = 410; i<720;i++){
                if(!std::isinf(Req->laser_data.ranges[i])){
                total_dist_sec_left += Req->laser_data.ranges[i]; 
                cal_left++;
                }
                if (Req->laser_data.ranges[i] < threshold) {
                    consecutive_obstacle_left++;
                    if (consecutive_obstacle_left >= obstacle_size) {
                        left = false;
                    }
                }else {
                    consecutive_obstacle_left =0;
                }

            }


            float left_avg = cal_left>0 ? total_dist_sec_left/cal_left : 0;
            float front_avg = cal_front>0 ? total_dist_sec_front/cal_front : 0;
            float right_avg = cal_right>0 ? total_dist_sec_right/cal_right : 0;

            // Right, Left and Front have no obstacles
            if( right && left && front){
                if (left_avg > right_avg && left_avg > front_avg) {
                    resp.direction = "left";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                }else if (front_avg > right_avg && front_avg > left_avg) {
                    resp.direction = "forward";
                    resp.proximity_scale = front_avg/allow_maximum_speed_distance_front ;
                }else{
                    resp.direction = "right";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                }
                std::cout << "left and right and forward are open" <<std::endl;

            // Left and Front have no obstacle
            }else if (left && !right && front) {

                 if (left_avg > front_avg) {
                    resp.direction = "left";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg),allow_maximum_speed_distance) / allow_maximum_speed_distance;

                }else if (front_avg > left_avg) {
                    resp.direction = "forward";
                    resp.proximity_scale = front_avg/allow_maximum_speed_distance_front ;
                }
                std::cout << "left and front are open, right is not" <<std::endl;
            // Left have no obstacle
            }else if (left && !right && !front) {
                    resp.direction = "left";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                    std::cout << "left is open, right and front are not" <<std::endl;
            // Right and Front have no obstacle
            }else if (right && !left && front) {
                if(front_avg > right_avg) {
                    resp.direction = "forward";
                    resp.proximity_scale = front_avg/allow_maximum_speed_distance_front ;
                }else{
                    resp.direction = "right";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                }
                std::cout << "right and front are open, left is not" <<std::endl;
            //Right have no obstacle
            }else if (right && !left && !front) {
                    resp.direction = "right";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                    std::cout << "right is open, left and front are not" <<std::endl;
            //Front have no obstacle
            }else if (!right && !left && front) {
                    resp.direction = "forward";
                    resp.proximity_scale = front_avg/allow_maximum_speed_distance_front ;
                    std::cout << "both of right and left are not open, but front is open" <<std::endl;
            
            //Three direction have obstacle or only front have obstacle
            }else if ((!right && !left && !front)) {
                if (left_avg > right_avg ) {
                    resp.direction = "left";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                }else if (right_avg > left_avg ) {
                    resp.direction = "right";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                }
                std::cout << "both of right, left and front are not open, choose one way to go  " <<resp.direction.c_str() <<std::endl;
            }else if ((right && left && !front) ) {
                if (left_avg > right_avg && determine_left ) {
                    resp.direction = "left";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                    determine_right = false;
                }else if (right_avg > left_avg && determine_right) {
                    resp.direction = "right";
                    resp.proximity_scale = std::min(std::min(left_avg, right_avg), allow_maximum_speed_distance) / allow_maximum_speed_distance;
                    determine_left = false;
                }
                std::cout << "both of right and left are open, but front is not open" <<std::endl;
            }
            
            if(!(right && left && !front)) {
                determine_left = true ;
                determine_right = true;
            }

            std::string front_status = front == true ? "true" : "false";

            std::cout << resp.proximity_scale <<std::endl;
            std::cout << "left_avg: " << left_avg <<std::endl;
            std::cout << "right_avg: " << right_avg <<std::endl;
            std::cout << "front_avg: " << front_avg <<std::endl;
            std::cout << front_status.c_str()  << std::endl;  
            
            return resp;
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    
    rclcpp::shutdown();
    return 0;

}