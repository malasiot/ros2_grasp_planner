#include "grasp_box_service.hpp"
#include "box_detector.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include "image_geometry/pinhole_camera_model.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cvx/math/rng.hpp>

#include <grasp_planner_msgs/msg/grasp.hpp>
#include <grasp_planner_msgs/msg/box3_d.hpp>

using namespace std;
using namespace Eigen;

static cvx::RNG g_rng ;


GraspBoxService::GraspBoxService(const rclcpp::NodeOptions &options) : rclcpp::Node("grasp_box_service", options)
{

    string viz_topic_boxes = declare_parameter("viz_topic_boxes", "/visual_boxes");

    declare_parameter("gripper_offset", 0.05f);
    declare_parameter("finger_width", 0.01f);

    declare_parameter("clearance", 0.01f);

    boxes_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic_boxes, 10);

    service_ = create_service<GraspBoxSrv>("grasp_box_service", std::bind(&GraspBoxService::callback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);

    segmentation_client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    segmentation_client_ = create_client<SegmentationSrv>("segmentation", rmw_qos_profile_services_default, segmentation_client_group_);
}

using namespace std::literals::chrono_literals;

void GraspBoxService::visualizeBoxes(const std::vector<Box> &boxes)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    int32_t id = 0;
    for (uint32_t i = 0; i < boxes.size(); i++)
    {
        const auto &box = boxes[i] ;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        marker.ns = "boxes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = box.center_.x() ;
        marker.pose.position.y = box.center_.y() ;
        marker.pose.position.z = box.center_.z() ;
        marker.lifetime = rclcpp::Duration(0, 0);

        float theta2 = box.theta_/2.0 ;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = sin(theta2);
        marker.pose.orientation.w = cos(theta2);

        // these scales are relative to the hand frame (unit: meters)
        marker.scale.x = box.sz_.x() ;
        marker.scale.y = box.sz_.y() ;
        marker.scale.z = box.sz_.z() ;

        marker.color.a = 0.5f;
        marker.color.r = g_rng.uniform<float>();
        marker.color.g = g_rng.uniform<float>();
        marker.color.b = g_rng.uniform<float>();

        ++id;

        marker_array.markers.push_back(marker);
    }

    boxes_rviz_pub_->publish(marker_array);
}

void serializeBoxes(const std::vector<Box> &boxes, std::vector<grasp_planner_msgs::msg::Box3D> &msgs) {
    for( const auto &box: boxes ) {
        grasp_planner_msgs::msg::Box3D msg ;
        msg.center.position.x = box.center_.x() ;
        msg.center.position.y = box.center_.y() ;
        msg.center.position.z = box.center_.z() ;
        
        float theta2 = box.theta_/2.0 ;
        msg.center.orientation.x = 0;
        msg.center.orientation.y = 0;
        msg.center.orientation.z = sin(theta2);
        msg.center.orientation.w = cos(theta2);

        // these scales are relative to the hand frame (unit: meters)
        msg.size.x = box.sz_.x() ;
        msg.size.y = box.sz_.y() ;
        msg.size.z = box.sz_.z() ;

        msgs.emplace_back(msg) ;
    }
}

static void makeSideGrasp(uint box_id, const Box &box, std::vector<grasp_planner_msgs::msg::Grasp> &grasps, float dist, float width, float height, float offset) {
    grasp_planner_msgs::msg::Grasp grasp ;

    Vector3f center{-box.sz_.x()/2, 0, offset} ;
    
    Matrix3f rot(AngleAxisf(box.theta_, Vector3f::UnitZ()) * AngleAxisf(M_PI, Vector3f::UnitX())) ;
    Affine3f tr = Translation3f(box.center_) * rot;

    Quaternionf q(rot) ;
    Vector3f c = tr * center ;
    
    grasp.width = width ;
    grasp.height = height ;
    grasp.depth = dist ;
    grasp.rotation.x = q.x() ;
    grasp.rotation.y = q.y() ;
    grasp.rotation.z = q.z() ;
    grasp.rotation.w = q.w() ;
    grasp.translation.x = c.x() ;
    grasp.translation.y = c.y() ;
    grasp.translation.z = c.z() ;

    grasp.score = 1 - 0.5 * fabs(offset)/box.sz_.z() ;
    grasp.box_id = box_id ;

    grasps.emplace_back(std::move(grasp)) ;
}

static void makeGrasps(const std::vector<Box> &boxes, std::vector<grasp_planner_msgs::msg::Grasp> &grasps, float dist, float height, float clearance) {
    const float offset_step = 0.05f ;
    uint count = 0 ;
    for(const auto &box: boxes) {
        grasp_planner_msgs::msg::Grasp g ;
        for( float offset = -box.sz_.z()/2 ; offset < box.sz_.z()/2 ; offset += offset_step ) {
            makeSideGrasp(count, box, grasps, dist, box.sz_.y() + clearance * 2, height, offset) ;
        }
        ++count ;
    }
}

void GraspBoxService::callback(const std::shared_ptr<GraspBoxSrv::Request> request, std::shared_ptr<GraspBoxSrv::Response> response)
{

    const auto &cam_info = request->camera_info;

    cv::Mat depth = cv_bridge::toCvCopy(request->depth, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    cv::Mat clr = cv_bridge::toCvCopy(request->rgb, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat mask = cv_bridge::toCvCopy(request->mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    
    while (!segmentation_client_->wait_for_service(1s)) {
        if ( !rclcpp::ok() ) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for segmentation service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    auto seg_request = std::make_shared<SegmentationSrv::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, clr);
    rgb_bridge->toImageMsg(seg_request->image);
    
    auto result = segmentation_client_->async_send_request(seg_request);

    auto status = result.wait_for(60s);

    if (status == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "Segmentation succesfull");
    } else {
        RCLCPP_ERROR(get_logger(), "Segmentation failed");
    }

    auto seg_result = result.get() ;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(seg_result->segmented_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat seg_mask = cv_ptr->image ;

    cv::imwrite("segmented_images/rgb.png", seg_mask) ;

    Camera cam;

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cam_info);

    cam.cx_ = model.cx();
    cam.cy_ = model.cy();
    cam.fx_ = model.fx();
    cam.fy_ = model.fy();

    cam.tr_ = tf2::transformToEigen(request->camera_pose).matrix().cast<float>();

    BoxDetector detector;

   // cv::Mat seg_mask = cv::imread("segmented_images/rgb.png");

    auto boxes = detector.detect(cam, depth, mask, seg_mask);

    makeGrasps(boxes, response->grasps, 0.05, 0.05, 0.05) ;

    visualizeBoxes(boxes) ;

    serializeBoxes(boxes, response->boxes) ;
}
