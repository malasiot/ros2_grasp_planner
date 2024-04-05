#include "grasp_planner_service.hpp"
#include "move_group_interface.hpp"

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

    
    camera_info_topic_ = declare_parameter("camera_info_topic", "/virtual_camera/color/camera_info");
    rgb_topic_ = declare_parameter("rgb_topic", "/virtual_camera/color/image_raw") ;
    depth_topic_ = declare_parameter("depth_topic", "/virtual_camera/depth/image_raw") ;
    mask_topic_ = declare_parameter("mask_topic", "/robot_mask/image_raw") ;

    graspnet_client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    graspnet_client_ = create_client<GraspNet>("graspnet", rmw_qos_profile_services_default, graspnet_client_group_);
    service_ = create_service<GraspPlannerSrv>("grasp_planner_service", std::bind(&GraspPlannerService::plan, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_group_);


}

void GraspPlannerService::setup(const std::shared_ptr<MoveGroupInterfaceNode> &mgi)
{
    move_group_interface_ = mgi ;
    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    caminfo_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data) ;

    sync_.reset(new Synchronizer(SyncPolicy(10), rgb_sub_, depth_sub_, caminfo_sub_));
    sync_->registerCallback(std::bind(&GraspPlannerService::frameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    mask_sub_ = std::make_shared<image_transport::Subscriber>(image_transport_->subscribe(mask_topic_, 10,
                                                                                          std::bind(&GraspPlannerService::maskCallback, this, std::placeholders::_1)));
}

void GraspPlannerService::frameCallback(sensor_msgs::msg::Image::ConstSharedPtr colorMsg, 
    sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camInfo)
{
    std::lock_guard<std::mutex> frame_lock_(frame_mutex_);

    camera_info_ = camInfo ;

    try
    {
        auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);

        rgb_ = colorPtr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        // display the error at most once per 10 seconds
        RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                              __FUNCTION__, __FILE__);
        return;
    }

    try
    {
        auto depthPtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);

        if (depthMsg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depthMsg->encoding == sensor_msgs::image_encodings::MONO16)
        {
            depth_ = depthPtr->image; // no conversion needed
        }
    }
    catch (cv_bridge::Exception &e)
    {
        // display the error at most once per 10 seconds
        RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                              __FUNCTION__, __FILE__);
        return;
    }

    frame_ready_ = true ;
}

void GraspPlannerService::maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    std::lock_guard<std::mutex> mask_lock_(mask_mutex_);
    auto depthPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == sensor_msgs::image_encodings::MONO16)
    {
        mask_ = depthPtr->image; // no conversion needed
    }

    mask_ready_ = true ;
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

void maskDepth(cv::Mat &depth, const cv::Mat &mask) {
    assert(depth.cols == mask.cols && depth.rows == mask.rows) ;

    cv::Mat_<ushort> depthx(depth), maskx(mask) ;

    for( int i=0 ; i<depthx.rows ; i++ )
        for( int j=0 ; j<depthx.cols ; j++ ) {
            ushort &dv = depthx[i][j] ;
            ushort maskv = maskx[i][j] ;

            if ( maskv == 0 || dv == 0 ) continue ;
            if ( abs(dv - maskv) < 10 ) dv = 0 ;

        }
}

using namespace std::literals::chrono_literals;

void GraspPlannerService::plan(const std::shared_ptr<GraspPlannerSrv::Request> request, std::shared_ptr<GraspPlannerSrv::Response> response) {
   while (!frame_ready_ && !mask_ready_) {
      RCLCPP_INFO(this->get_logger(), "still waiting for frame");
      rclcpp::sleep_for(1s);
    }   

    cv::Mat rgb, depth ;
    {
        std::lock_guard<std::mutex> frame_lock(frame_mutex_) ;
        rgb = rgb_.clone() ;
        depth = depth_.clone() ;
    }

    cv::Mat mask ;
    {
        std::lock_guard<std::mutex> mask_lock(mask_mutex_) ;   
        mask = mask_.clone() ;
    }

    maskDepth(depth, mask) ;

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

    graspnet_request->camera_info = *camera_info_ ;

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
        move_group_interface_->filterGrasps(resp->grasps, grasps_filtered, results) ;
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
