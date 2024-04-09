#include "grasp_planner_service.hpp"
#include "move_group_interface.hpp"
#include "masked_point_cloud.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std ;
using namespace Eigen ;

GraspPlannerService::GraspPlannerService(const rclcpp::NodeOptions &options): rclcpp::Node("grasp_planner_service", options) {

    
    grasps_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visual_grasps", 10);

    graspnet_client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    graspnet_client_ = create_client<GraspNet>("graspnet", rmw_qos_profile_services_default, graspnet_client_group_);
    service_ = create_service<GraspPlannerSrv>("grasp_planner_service", std::bind(&GraspPlannerService::plan, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_group_);



}

void GraspPlannerService::setup(const std::shared_ptr<MoveGroupInterfaceNode> &mgi, const std::shared_ptr<MaskedPointCloud> &pcl)
{
    move_group_interface_ = mgi ;
    masked_point_cloud_ = pcl ;
}


extern visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
  const std::vector<grasp_planner_interfaces::msg::Grasp> & hands,
  double hand_length, double finger_width, double finger_height,
  const std::string & frame_id, const Vector4f &clr, const std::string &ns) ;

static void convertToWorldCoordinates(const Isometry3d &camera_tr, std::vector<grasp_planner_interfaces::msg::Grasp> &grasps) {
      for( auto &g: grasps ) {
            auto &trv = g.translation ;
            auto &rotv = g.rotation ;

            Eigen::Vector3d c(trv.x, trv.y, trv.z) ;
            Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z) ;
          
            Isometry3d p ;
            p.setIdentity() ;
            p.linear() = rot.toRotationMatrix() ;
            p.translation() = c ;

            p = camera_tr * p ;

            c = p.translation() ;
            rot = p.linear() ;

            trv.x = c.x() ; trv.y = c.y() ; trv.z = c.z() ;
            rotv.x = rot.x() ; rotv.y = rot.y() ; rotv.z = rot.z() ;  rotv.w = rot.w() ;
        }
}


using namespace std::literals::chrono_literals;

void GraspPlannerService::plan(const std::shared_ptr<GraspPlannerSrv::Request> request, std::shared_ptr<GraspPlannerSrv::Response> response) {
   while (! masked_point_cloud_->hasFrame() ) {
      RCLCPP_INFO(this->get_logger(), "still waiting for frame");
      rclcpp::sleep_for(1s);
    }   

    
    auto [rgb, depth, caminfo] = masked_point_cloud_->getFrame() ; 
   
    auto graspnet_request = std::make_shared<GraspNet::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, rgb);
    rgb_bridge->toImageMsg(graspnet_request->rgb);

    // Convert OpenCV Mat to ROS Image

    auto width = rgb.cols;
    auto height = rgb.rows;

    auto depth_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO16, depth.clone());
    depth_bridge->toImageMsg(graspnet_request->depth);

    graspnet_request->camera_info = caminfo ;

    while (!graspnet_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            response->result = 0 ;
            return ;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto graspnet_result = graspnet_client_->async_send_request(graspnet_request);

   
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    geometry_msgs::msg::TransformStamped tr  = tf_buffer->lookupTransform(
                     "world", "camera_optical_frame", tf2::TimePointZero, tf2::durationFromSec(5));

    auto camera_tr = tf2::transformToEigen(tr);
 
  
    // Wait for the result.
     auto status = graspnet_result.wait_for(10s)  ;
   
    if ( status == std::future_status::ready )
    {
        auto resp = graspnet_result.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received %d candidate grasps", resp->grasps.size()) ;

        // transform to world coordinate frame
        convertToWorldCoordinates(camera_tr, resp->grasps) ;

        // publish markers
        grasps_rviz_pub_->publish(convertToVisualGraspMsg(resp->grasps, 0.05, 0.01, 0.01, "world", {0, 0, 1.0f, 0.5f}, "candidates"));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Filtering non-reachable candidates") ;
        vector<grasp_planner_interfaces::msg::Grasp> grasps_filtered ;
        vector<GraspCandidate> results ;
        move_group_interface_->filterGrasps(resp->grasps, 0.02, 0.01, grasps_filtered, results) ;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Found %d reachable grasps", results.size()) ;

        grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps_filtered, 0.05, 0.01, 0.01, "world", {1, 0, 0.0f, 0.5f}, "filtered"));


        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response ok");
        response->result = !results.empty() ;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Graspnet service did not respond");
        response->result = 0 ;
    }
}
