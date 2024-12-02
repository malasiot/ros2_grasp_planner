#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/subscriber.h"
#include "image_transport/image_transport.hpp"
#include "grasp_planner_msgs/srv/grasp_candidates.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "grasp_planner_interfaces/srv/grasp_net.hpp"
#include "grasp_planner_interfaces/srv/grasp_box.hpp"
#include "cv_bridge/cv_bridge.h"

class MaskedPointCloud ;

class GraspCandidatesNode:  public rclcpp::Node {
public:
    GraspCandidatesNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void setup(const std::shared_ptr<MaskedPointCloud> &pcl);
private:

 void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
 void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
 void frameCallback(sensor_msgs::msg::Image::ConstSharedPtr colorMsg, sensor_msgs::msg::Image::ConstSharedPtr depthMsg, sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo) ;
  
private:
   
    using GraspCandidatesSrv = grasp_planner_msgs::srv::GraspCandidates ;
    using GraspNet = grasp_planner_interfaces::srv::GraspNet;
    using GraspBox = grasp_planner_interfaces::srv::GraspBox;

    rclcpp::Service<GraspCandidatesSrv>::SharedPtr service_ ;
    
    std::shared_ptr<rclcpp::Client<GraspNet>> graspnet_client_ ;
    std::shared_ptr<rclcpp::Client<GraspBox>> graspbox_client_ ;
    rclcpp::CallbackGroup::SharedPtr graspnet_client_group_, service_group_ ;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> grasps_rviz_pub_ ;

    void candidates(const std::shared_ptr<GraspCandidatesSrv::Request> request, std::shared_ptr<GraspCandidatesSrv::Response> response) ;
    
    rclcpp::Client<GraspNet>::FutureAndRequestId callGraspNet(const cv::Mat &, const cv::Mat &, const cv::Mat &, const sensor_msgs::msg::CameraInfo &) ; 
    rclcpp::Client<GraspBox>::FutureAndRequestId callGraspBox(const cv::Mat &, const cv::Mat &, const cv::Mat &, const sensor_msgs::msg::CameraInfo &, const geometry_msgs::msg::Transform &) ; 
    
    std::shared_ptr<MaskedPointCloud> masked_point_cloud_ ;
};