#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <grasp_planner_interfaces/srv/grasp_net.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>


#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

extern visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
  const std::vector<grasp_planner_interfaces::msg::Grasp> & hands,
  double hand_length, double finger_width, double finger_height,
  const std::string & frame_id) ;

sensor_msgs::msg::CameraInfo getCameraInfo(uint32_t width, uint32_t height, float f)
{
    sensor_msgs::msg::CameraInfo info;
    info.width = width;
    info.height = height;

    info.k.at(0) = f;
    info.k.at(2) = width / 2.0;
    info.k.at(4) = f;
    info.k.at(5) = height / 2.0;
    info.k.at(8) = 1;

    info.p.at(0) = info.k.at(0);
    info.p.at(1) = 0;
    info.p.at(2) = info.k.at(2);
    info.p.at(3) = 0;
    info.p.at(4) = 0;
    info.p.at(5) = info.k.at(4);
    info.p.at(6) = info.k.at(5);
    info.p.at(7) = 0;
    info.p.at(8) = 0;
    info.p.at(9) = 0;
    info.p.at(10) = 1;
    info.p.at(11) = 0;

    // set R (rotation matrix) values to identity matrix
    info.r.at(0) = 1.0;
    info.r.at(1) = 0.0;
    info.r.at(2) = 0.0;
    info.r.at(3) = 0.0;
    info.r.at(4) = 1.0;
    info.r.at(5) = 0.0;
    info.r.at(6) = 0.0;
    info.r.at(7) = 0.0;
    info.r.at(8) = 1.0;

    int coeff_size(5);
    info.distortion_model = "plumb_bob";

    info.d.resize(coeff_size);
    for (int i = 0; i < coeff_size; i++)
    {
        info.d.at(i) = 0.0;
    }
    return info;
}


using namespace std::literals::chrono_literals;
using GraspNet = grasp_planner_interfaces::srv::GraspNet;

void validateGrasps(const std::vector<grasp_planner_interfaces::msg::Grasp> &grasps) {
}
  

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

 cout << argv[1] << ' ' << argv[2] << endl ;

    cv::Mat rgb = cv::imread(argv[1]);
    cv::Mat depth = cv::imread(argv[2], -1);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("graspnet_client_node");

   robot_model_loader::RobotModelLoader robot_model_loader(node);
   moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("l_iiwa_arm");

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d &end_effector_state = kinematic_state->getGlobalLinkTransform("l_tool0");

  /* Print end-effector pose. Remember that this is in the model frame */
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Translation: " << end_effector_state.translation());
RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Rotation: " << end_effector_state.rotation());

bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10);


if (found_ik)
{
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i=0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
}
else
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Did not find IK solution");
}

    auto grasps_rviz_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
     "/visual_grasps", 10);

    auto client = node->create_client<GraspNet>("graspnet");

    auto request = std::make_shared<GraspNet::Request>();

    std_msgs::msg::Header header; // empty header
    header.stamp = node->get_clock()->now();
    header.frame_id = "camera";

    auto rgb_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::BGR8, rgb.clone());
    rgb_bridge->toImageMsg(request->rgb);

    // Convert OpenCV Mat to ROS Image

    auto width = rgb.cols;
    auto height = rgb.rows;

    auto depth_bridge = std::make_shared<cv_bridge::CvImage>(header, sensor_msgs::image_encodings::MONO16, depth.clone());
    depth_bridge->toImageMsg(request->depth);

    float f = 690.0f;

    request->camera_info = getCameraInfo(width, height, f);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

   
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        cout << response->grasps.size() << endl ;

            grasps_rviz_pub->publish(convertToVisualGraspMsg(response->grasps, 0.05, 0.01, 0.01, "camera_optical_frame"));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response ok");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response error");
    }

    rclcpp::shutdown();
}