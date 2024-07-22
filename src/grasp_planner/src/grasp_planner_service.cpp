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

    string viz_topic = declare_parameter("viz_topic", "/visual_grasps") ;
    declare_parameter("camera_frame", "camera_color_optical_frame") ;
    declare_parameter("gripper_offset", 0.05f) ;
    declare_parameter("finger_width", 0.01f) ;
    declare_parameter("max_results", 3) ;
    declare_parameter("clearance", 0.01f) ;
       
    grasps_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic, 10);

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

    auto camera_frame = masked_point_cloud_->getCameraFrame() ;

    auto [rgb, depth, mask, caminfo] = masked_point_cloud_->getFrame() ; 
RCLCPP_INFO(this->get_logger(), "saving images");
    cv::imwrite("/workspaces/ros2_grasp_planner/rgb.png", rgb) ;
    cv::imwrite("/workspaces/ros2_grasp_planner/depth.png", depth) ;
    cv::imwrite("/workspaces/ros2_grasp_planner/mask.png", mask) ;
   
    auto graspnet_request = std::make_shared<GraspNet::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, rgb);
    rgb_bridge->toImageMsg(graspnet_request->rgb);

    // Convert OpenCV Mat to ROS Image
   
    auto depth_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO16, depth.clone());
    depth_bridge->toImageMsg(graspnet_request->depth);

    auto mask_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO8, mask);
    mask_bridge->toImageMsg(graspnet_request->mask);

    graspnet_request->camera_info = caminfo ;

    while (!graspnet_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            response->result = GraspPlannerSrv::Response::RESULT_ERROR ;
            return ;
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    auto graspnet_result = graspnet_client_->async_send_request(graspnet_request);

   
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    Isometry3d camera_tr ;

    try {
        geometry_msgs::msg::TransformStamped tr  = tf_buffer->lookupTransform(
                     "world", camera_frame, tf2::TimePointZero, tf2::durationFromSec(5));
        camera_tr = tf2::transformToEigen(tr);

        ofstream strm("/workspaces/ros2_grasp_planner/camera.txt") ;
        strm << camera_tr.matrix() << endl ;
 
    } catch ( tf2::TransformException &e ) {
        RCLCPP_ERROR(get_logger(), "Failed to read transform between 'world' and '%s': %s", camera_frame.c_str(), e.what()) ;
        response->result = GraspPlannerSrv::Response::RESULT_ERROR ;
        return ;
    }
 
    // Wait for the result.
     auto status = graspnet_result.wait_for(10s)  ;
   
    if ( status == std::future_status::ready )
    {
        auto resp = graspnet_result.get();
        RCLCPP_INFO(get_logger(), "Received %ld candidate grasps", resp->grasps.size()) ;

        // transform to world coordinate frame
        convertToWorldCoordinates(camera_tr, resp->grasps) ;

        // publish markers
        grasps_rviz_pub_->publish(convertToVisualGraspMsg(resp->grasps, 0.05, 0.01, 0.01, "world", {0, 0, 1.0f, 0.5f}, "candidates"));

        RCLCPP_INFO(get_logger(), "Filtering non-reachable candidates") ;
        vector<grasp_planner_interfaces::msg::Grasp> grasps_filtered ;
        vector<GraspCandidate> results ;

        double gripper_offset = get_parameter("gripper_offset").as_double() ;
        double finger_width = get_parameter("finger_width").as_double() ;
        int max_results = get_parameter("max_results").as_int() ;
        double clearance = get_parameter("clearance").as_double() ;

        move_group_interface_->filterGrasps(resp->grasps, gripper_offset, finger_width, clearance, grasps_filtered, results) ;
        RCLCPP_INFO(get_logger(), "Found %ld reachable grasps", results.size()) ;

        grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps_filtered, 0.05, 0.01, 0.01, "world", {1, 0, 0.0f, 0.5f}, "filtered"));

        move_group_interface_->computeMotionPlans(results, request->start_state, max_results) ;

        RCLCPP_INFO(get_logger(), "Finished");

        for( const auto &result: results ) {
            response->trajectories.push_back(result.trajectory_.joint_trajectory) ;
        }
        response->result = ( results.empty() ) ? GraspPlannerSrv::Response::RESULT_NOT_FOUND : GraspPlannerSrv::Response::RESULT_OK ; 
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Graspnet service did not respond");
        response->result = GraspPlannerSrv::Response::RESULT_ERROR ;
    }
}
