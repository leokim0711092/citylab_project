#include "custom_interfaces/action/detail/go_to_pose__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rcl/publisher.h"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <custom_interfaces/action/go_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
#include <cmath>
#include <math.h>

using namespace std::placeholders;

class GoToPose : public rclcpp::Node{

    public:
        using GTP = custom_interfaces::action::GoToPose;
        using GoalHandleGTP = rclcpp_action::ServerGoalHandle<GTP>;

        explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):
            Node("Go_to_pose_node", options){

            callback_group_ = this->create_callback_group(
                rclcpp::CallbackGroupType::Reentrant);
            auto sub1_ = rclcpp::SubscriptionOptions();
            sub1_.callback_group = callback_group_;
            

            this->action_ser_ = rclcpp_action::create_server<custom_interfaces::action::GoToPose>(
                this,
                "/go_to_pose",
                std::bind(&GoToPose::goal_response, this, _1, _2),
                std::bind(&GoToPose::goal_cancel, this, _1),
                std::bind(&GoToPose::handle_accepted, this, _1)
            );
            pub_ = this ->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10, std::bind(&GoToPose::sub_callback,this,std::placeholders::_1));
            timer = this->create_wall_timer(std::chrono::milliseconds(180), std::bind(&GoToPose::timer_callback,this));
        }
    private:
        rclcpp_action::Server<custom_interfaces::action::GoToPose>::SharedPtr action_ser_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        std::shared_ptr<nav_msgs::msg::Odometry> odom_msg;
        geometry_msgs::msg::Twist vel;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        rclcpp_action::GoalResponse goal_response(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GTP::Goal> goal){
            RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->goal_pos);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
        }

        rclcpp_action::CancelResponse goal_cancel(const std::shared_ptr<GoalHandleGTP> go_cancel){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)go_cancel;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleGTP> goal_handle){

            std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
        
        }

        void execute( const std::shared_ptr<GoalHandleGTP> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<GTP::Feedback>();
            auto result = std::make_shared<GTP::Result>();
            bool x_pos = false;
            bool y_pos = false;
            bool orient = false;
            float error_x = 0.0;
            float error_y = 0.0;
            float error_orient = 0.0;
            
            goal->goal_pos.x;
            rclcpp::Rate lr(1);
            float direction = 0;
            float rad_theta = 0;

            while (rclcpp::ok() && (!x_pos || !y_pos || !orient)) {
                
                if (goal_handle->is_canceling()) {
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                
                rad_theta =  goal->goal_pos.theta/180.0*M_PI;

                error_x = std::fabs(goal->goal_pos.x - odom_msg->pose.pose.position.x);
                error_y = std::fabs(goal->goal_pos.y - odom_msg->pose.pose.position.y);
                error_orient = std::fabs(rad_theta - euler_degree_transform(odom_msg));

                if (error_x < 0.05) {
                    x_pos = true;
                    if(x_pos) std::cout << "X_pos: true" <<std::endl;
                }

                if(error_y < 0.05){
                    y_pos =true;
                    if(y_pos) std::cout << "Y_pos: true" <<std::endl;
                }
                if(error_orient < 0.05){
                    orient = true;
                    if(orient) std::cout << "Orient_pos: true" <<std::endl;
                }
                
                direction = rad_theta - euler_degree_transform(odom_msg);
                determine_velocity(error_x, error_y, error_orient, direction, goal_handle);
                
                if(x_pos && y_pos && orient){
                    vel.linear.x = 0;
                    vel.angular.z = 0;
                }

                feedback->current_pos.x = odom_msg->pose.pose.position.x;
                feedback->current_pos.y = odom_msg->pose.pose.position.y;
                feedback->current_pos.theta = euler_degree_transform(odom_msg);
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback:");
                RCLCPP_INFO(this->get_logger(), "X: %f", feedback->current_pos.x);
                RCLCPP_INFO(this->get_logger(), "Y: %f", feedback->current_pos.y);
                RCLCPP_INFO(this->get_logger(), "Theta: %f", feedback->current_pos.theta);
                lr.sleep();
            }
            if (rclcpp::ok()) {
                result->status = true;
                goal_handle->succeed(result);
            }
        }

        void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            this->odom_msg = msg;
        }

        float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg){
                float x = msg->pose.pose.orientation.z;
                float y = msg->pose.pose.orientation.y;
                float z = msg->pose.pose.orientation.z;
                float w = msg->pose.pose.orientation.w;
            return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
        }

        void timer_callback(){
            RCLCPP_INFO(this->get_logger(),"%f",vel.angular.z);
            pub_->publish(vel);
        }
        
        void determine_velocity(float error_x, float error_y, float error_orient, float direction, const std::shared_ptr<GoalHandleGTP> goal_handle){
            const auto goal = goal_handle->get_goal();
            bool orient_avaliable = false;
            float dir_of_xy = 0;
            float dist_diff =0;
            if (error_x < 0.05 && error_y < 0.05 ) {
                orient_avaliable = true;
            }else {
                dir_of_xy = atan2(goal->goal_pos.y - odom_msg->pose.pose.position.y, goal->goal_pos.x - odom_msg->pose.pose.position.x);
                dist_diff = std::sqrt(std::pow(goal->goal_pos.y - odom_msg->pose.pose.position.y,2) + std::pow(goal->goal_pos.x - odom_msg->pose.pose.position.x,2));
                vel.angular.z  = dir_of_xy*0.5;
                vel.linear.x =  0.2 * std::min(dist_diff, 0.4f)/0.4f;
            }
            if(error_orient > 0.05 && orient_avaliable ){
                vel.linear.x = 0;
                vel.angular.z = direction*0.5;
            }
        
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<GoToPose>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}