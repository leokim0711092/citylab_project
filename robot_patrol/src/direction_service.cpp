#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "rcl/init_options.h"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cstddef>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

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
            
            Res->direction = find_direction(Req);


        }

        std::string find_direction(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> Req){
            
            total_dist_sec_right = 0.0 ;
            total_dist_sec_front = 0.0;
            total_dist_sec_left = 0.0;
            
            for(int i = 0; i<240;i++){
                total_dist_sec_left += Req->laser_data.ranges[i];
            }
            
            float max = total_dist_sec_left;
            std::string direction = "left";

            for(int i = 240; i<480;i++){
                total_dist_sec_front += Req->laser_data.ranges[i];
            }

            if (total_dist_sec_front > max) {
                max = total_dist_sec_front;
                direction = "forward";
            }

            for(int i = 480; i<720;i++){
                total_dist_sec_right += Req->laser_data.ranges[i];
            }

            if (total_dist_sec_right > max) {
                max = total_dist_sec_right;
                direction = "right";
            }
            return direction;
        }

};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    
    rclcpp::shutdown();
    return 0;

}