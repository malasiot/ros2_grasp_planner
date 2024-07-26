#include "grasp_box_service.hpp"
#include "box_detector.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include "image_geometry/pinhole_camera_model.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cvx/math/rng.hpp>

#include <grasp_planner_interfaces/msg/grasp.hpp>

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

static void makeSideGrasp(const Box &box, std::vector<grasp_planner_interfaces::msg::Grasp> &grasps, float dist, float width, float height, float offset) {
    grasp_planner_interfaces::msg::Grasp grasp ;

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

    grasps.emplace_back(std::move(grasp)) ;
}

static void makeGrasps(const std::vector<Box> &boxes, std::vector<grasp_planner_interfaces::msg::Grasp> &grasps, float dist, float height) {
    const float offset_step = 0.05f ;
    for(const auto &box: boxes) {
        grasp_planner_interfaces::msg::Grasp g ;
        for( float offset = -box.sz_.z()/2 ; offset < box.sz_.z()/2 ; offset += offset_step ) {
            makeSideGrasp(box, grasps, dist, box.sz_.y(), height, offset) ;
        }
    }
}

void GraspBoxService::callback(const std::shared_ptr<GraspBoxSrv::Request> request, std::shared_ptr<GraspBoxSrv::Response> response)
{

    const auto &cam_info = request->camera_info;

    cv::Mat depth = cv_bridge::toCvCopy(request->depth, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    cv::Mat clr = cv_bridge::toCvCopy(request->rgb, sensor_msgs::image_encodings::BGR8)->image;
    cv::Mat mask = cv_bridge::toCvCopy(request->mask, sensor_msgs::image_encodings::TYPE_8UC1)->image;

    Camera cam;

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cam_info);

    cam.cx_ = model.cx();
    cam.cy_ = model.cy();
    cam.fx_ = model.fx();
    cam.fy_ = model.fy();

    cam.tr_ = tf2::transformToEigen(request->camera_pose).matrix().cast<float>();

    BoxDetector detector;

    cv::Mat seg_mask = cv::imread("segmented_images/rgb.png");

    auto boxes = detector.detect(cam, depth, mask, seg_mask);

    makeGrasps(boxes, response->grasps, 0.05, 0.05) ;

    visualizeBoxes(boxes) ;
}
