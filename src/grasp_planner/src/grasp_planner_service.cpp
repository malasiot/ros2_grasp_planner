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

using namespace std;
using namespace Eigen;

GraspPlannerService::GraspPlannerService(const rclcpp::NodeOptions &options) : rclcpp::Node("grasp_planner_service", options)
{

    string viz_topic = declare_parameter("viz_topic", "/visual_grasps");
    declare_parameter("camera_frame", "camera_color_optical_frame");
    declare_parameter("gripper_offset", -0.0f);
    declare_parameter("finger_width", 0.01f);
    declare_parameter("max_results", 3);
    declare_parameter("clearance", 0.01f);
    declare_parameter("n_attempts_6dof", 15);
    declare_parameter("n_attempts_7dof", 1);
    declare_parameter("tol_6dof", std::vector<double>{0.02, 0.02, 0.02, 0.05, 0.2, 0.05});
    declare_parameter("tol_7dof", std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});

    grasps_rviz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic, 10);

    graspnet_client_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    graspnet_client_ = create_client<GraspNet>("graspnet", rmw_qos_profile_services_default, graspnet_client_group_);
    graspbox_client_ = create_client<GraspBox>("grasp_box_service", rmw_qos_profile_services_default, graspnet_client_group_);
    service_ = create_service<GraspPlannerSrv>("grasp_planner_service", std::bind(&GraspPlannerService::plan, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_group_);
}

void GraspPlannerService::setup(const std::shared_ptr<MoveGroupInterfaceNode> &mgi, const std::shared_ptr<MaskedPointCloud> &pcl)
{
    move_group_interface_ = mgi;
    masked_point_cloud_ = pcl;
}

extern visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
    const std::vector<grasp_planner_msgs::msg::Grasp> &hands,
    double hand_length, double finger_width, double finger_height,
    const std::string &frame_id, const Vector4f &clr, const std::string &ns);

static void convertToWorldCoordinates(const Isometry3d &camera_tr, std::vector<grasp_planner_msgs::msg::Grasp> &grasps)
{
    for (auto &g : grasps)
    {
        auto &trv = g.translation;
        auto &rotv = g.rotation;

        Eigen::Vector3d c(trv.x, trv.y, trv.z);
        Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z);

        Isometry3d p;
        p.setIdentity();
        p.linear() = rot.toRotationMatrix();
        p.translation() = c;

        p = camera_tr * p;

        c = p.translation();
        rot = p.linear();

        trv.x = c.x();
        trv.y = c.y();
        trv.z = c.z();
        rotv.x = rot.x();
        rotv.y = rot.y();
        rotv.z = rot.z();
        rotv.w = rot.w();
    }
}

using namespace std::literals::chrono_literals;

rclcpp::Client<GraspPlannerService::GraspNet>::FutureAndRequestId GraspPlannerService::callGraspNet(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask, const sensor_msgs::msg::CameraInfo &caminfo)
{
    auto graspnet_request = std::make_shared<GraspNet::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, rgb);
    rgb_bridge->toImageMsg(graspnet_request->rgb);

    // Convert OpenCV Mat to ROS Image

    auto depth_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO16, depth);
    depth_bridge->toImageMsg(graspnet_request->depth);

    auto mask_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO8, mask);
    mask_bridge->toImageMsg(graspnet_request->mask);

    graspnet_request->camera_info = caminfo;

    return graspnet_client_->async_send_request(graspnet_request);
}

rclcpp::Client<GraspPlannerService::GraspBox>::FutureAndRequestId GraspPlannerService::callGraspBox(const cv::Mat &rgb, const cv::Mat &depth, const cv::Mat &mask, const sensor_msgs::msg::CameraInfo &caminfo, const geometry_msgs::msg::Transform &tr)
{
    auto graspbox_request = std::make_shared<GraspBox::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, rgb);
    rgb_bridge->toImageMsg(graspbox_request->rgb);

    // Convert OpenCV Mat to ROS Image

    auto depth_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO16, depth);
    depth_bridge->toImageMsg(graspbox_request->depth);

    auto mask_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO8, mask);
    mask_bridge->toImageMsg(graspbox_request->mask);

    graspbox_request->camera_info = caminfo;
    graspbox_request->camera_pose = tr;

    return graspbox_client_->async_send_request(graspbox_request);
}

void GraspPlannerService::plan(const std::shared_ptr<GraspPlannerSrv::Request> request, std::shared_ptr<GraspPlannerSrv::Response> response)
{
    bool tactile = request->end_effector == GraspPlannerSrv::Request::GRASP_WITH_TACTILE;
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    uint8_t algorithm = request->algorithm;

    while (!masked_point_cloud_->hasFrame())
    {
        RCLCPP_INFO(this->get_logger(), "still waiting for frame");
        rclcpp::sleep_for(1s);
    }

    auto camera_frame = masked_point_cloud_->getCameraFrame();

    auto [rgb, depth, mask, caminfo] = masked_point_cloud_->getFrame();

    Isometry3d camera_tr;
    geometry_msgs::msg::TransformStamped tr;

    try
    {
        tr = tf_buffer->lookupTransform(
            "world", camera_frame, tf2::TimePointZero, tf2::durationFromSec(5));
        camera_tr = tf2::transformToEigen(tr);

        ofstream strm("/workspaces/ros2_grasp_planner/camera.txt");
        strm << camera_tr.matrix() << endl;
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to read transform between 'world' and '%s': %s", camera_frame.c_str(), e.what());
        response->result = GraspPlannerSrv::Response::RESULT_ERROR;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "saving images");
    cv::imwrite("/workspaces/ros2_grasp_planner/rgb.png", rgb);
    cv::imwrite("/workspaces/ros2_grasp_planner/depth.png", depth);
    cv::imwrite("/workspaces/ros2_grasp_planner/mask.png", mask);

    size_t max_results = get_parameter("max_results").as_int();

    GraspCandidateFilterParams fparams;
    fparams.clearence_thresh_ = get_parameter("clearance").as_double();
    fparams.offset_ = get_parameter("gripper_offset").as_double();
    fparams.finger_width_ = get_parameter("finger_width").as_double();
    fparams.n_attempts_6dof_ = get_parameter("n_attempts_6dof").as_int();
    fparams.n_attempts_7dof_ = get_parameter("n_attempts_7dof").as_int();
    fparams.tol_6dof_ = get_parameter("tol_6dof").as_double_array();
    fparams.tol_7dof_ = get_parameter("tol_7dof").as_double_array();

    std::future_status status;
    std::vector<GraspCandidate> results ;

    if ( algorithm == GraspPlannerSrv::Request::ALGORITHM_GRASPNET ) {
        while (!graspnet_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for graspnet the service. Exiting.");
                response->result = GraspPlannerSrv::Response::RESULT_ERROR;
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto graspnet_result = callGraspNet(rgb, depth, mask, caminfo);

        status = graspnet_result.wait_for(10s);

        if (status == std::future_status::ready) {
            auto resp_gnet = graspnet_result.get();    

             RCLCPP_INFO(get_logger(), "Received %ld candidate grasps from graspnet", resp_gnet->grasps.size());

        // transform to world coordinate frame
            convertToWorldCoordinates(camera_tr, resp_gnet->grasps);

#ifdef DEBUG
        resp_gnet->grasps.clear();

        grasp_planner_interfaces::msg::Grasp g;
        g.depth = 0.05;
        g.height = 0.05;
        g.score = 0.5;
        g.width = 0.3;
        g.translation.x = 0.75;
        g.translation.y = 0;
        g.translation.z = 0.5;
        resp_gnet->grasps.emplace_back(g);
        // publish markers
#endif
            grasps_rviz_pub_->publish(convertToVisualGraspMsg(resp_gnet->grasps, 0.05, 0.01, 0.01, "world", {0, 0, 1.0f, 0.5f}, "candidates:graspnet"));

            RCLCPP_INFO(get_logger(), "Filtering non-reachable candidates");
            vector<grasp_planner_msgs::msg::Grasp> grasps_filtered;
     
            move_group_interface_->filterGrasps(resp_gnet->grasps, fparams, tactile, grasps_filtered, results);
            grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps_filtered, 0.05, 0.01, 0.01, "world", {1, 0, 0.0f, 0.5f}, "filtered:graspnet"));
        } else  {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Graspnet service did not respond");
            response->result = GraspPlannerSrv::Response::RESULT_ERROR;
            return ;
        }

    } else {
        while (!graspbox_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for graspbox service. Exiting.");
                response->result = GraspPlannerSrv::Response::RESULT_ERROR;
                return;
            }
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        auto graspbox_result = callGraspBox(rgb, depth, mask, caminfo, tr.transform);

        status = graspbox_result.wait_for(60s);

        if (status == std::future_status::ready) {
            auto resp_gbox = graspbox_result.get();    

            RCLCPP_INFO(get_logger(), "Received %ld candidate grasps from graspbox", resp_gbox->grasps.size());

        // transform to world coordinate frame
          //  convertToWorldCoordinates(camera_tr, resp_gbox->grasps);

            grasps_rviz_pub_->publish(convertToVisualGraspMsg(resp_gbox->grasps, 0.05, 0.01, 0.01, "world", {0, 0, 1.0f, 0.5f}, "candidates:graspbox"));

            RCLCPP_INFO(get_logger(), "Filtering non-reachable candidates");
            vector<grasp_planner_msgs::msg::Grasp> grasps_filtered;
     
            move_group_interface_->filterGrasps(resp_gbox->grasps, fparams, tactile, grasps_filtered, results);
            grasps_rviz_pub_->publish(convertToVisualGraspMsg(grasps_filtered, 0.05, 0.01, 0.01, "world", {1, 0, 0.0f, 0.5f}, "filtered:graspbox"));

        } else  {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Graspbox service did not respond");
            response->result = GraspPlannerSrv::Response::RESULT_ERROR;
            return ;
        }
    
        
    }

        // sort candidates
    std::sort(results.begin(), results.end(), [](const GraspCandidate &a, const GraspCandidate &b)  {
        double ma = a.lm_ + a.rm_, mb = b.lm_ + b.rm_;
        return ma > mb; // sort based on manipulability
    });

    // visualize robot state for best result
    if (!results.empty())
        move_group_interface_->visualizeResult(results[0]);

    RCLCPP_INFO(get_logger(), "Found %ld reachable grasps", results.size());

    // motion planning to compute trajectories
    move_group_interface_->computeMotionPlans(results, request->start_state, max_results);

    RCLCPP_INFO(get_logger(), "Finished");

    for (uint i = 0; i < std::min(results.size(), max_results); i++) {
        const auto &result = results[i];
        response->trajectories.push_back(result.trajectory_.joint_trajectory);
    }

    response->result = (results.empty()) ? GraspPlannerSrv::Response::RESULT_NOT_FOUND : GraspPlannerSrv::Response::RESULT_OK;
    
    
}
