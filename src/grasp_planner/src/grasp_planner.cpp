#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <grasp_planner_interfaces/srv/grasp_net.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;

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

visualization_msgs::msg::Marker createHandBaseMarker(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & end, const Eigen::Matrix3d & frame, double length, double height, int id,
  const std::string & frame_id)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = rclcpp::Duration(0, 0);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length;  // forward direction
  marker.scale.y = (end - start).norm();  // hand closing direction
  marker.scale.z = height;  // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  return marker;
}


visualization_msgs::msg::Marker createFingerMarker(
  const Eigen::Vector3d & center,
  const Eigen::Matrix3d & frame, double length, double width, double height, int id,
  const std::string & frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = rclcpp::Duration(0, 0);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length;  // forward direction
  marker.scale.y = width;  // hand closing direction
  marker.scale.z = height;  // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  return marker;
}



visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
  const std::vector<grasp_planner_interfaces::msg::Grasp> & hands,
  double outer_diameter, double hand_depth, double finger_width, double hand_height,
  const std::string & frame_id)
{
  double width = outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker left_finger, right_finger, base, hand;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center,
    approach_center,
    base_center;

  for (uint32_t i = 0; i < hands.size(); i++) {
    const auto &trv = hands[i].translation ;
    const auto &rotv = hands[i].rotation ;

    Eigen::Vector3d tr(trv[0], trv[1], trv[2]) ;
    Eigen::Matrix3d rot ;
    rot << rotv[0], rotv[1], rotv[2], 
            rotv[3], rotv[4], rotv[5],
            rotv[6], rotv[7], rotv[8] ;

   
    auto approach = rot.col(0) ;
    auto binormal = -rot.col(1) ;
 double hw = 0.5 * hands[i].width;
    left_bottom =  tr - (hw - 0.5 * finger_width) * binormal;
    right_bottom = tr + (hw - 0.5 * finger_width) * binormal;
     left_top = left_bottom + hand_depth * approach;
    right_top = right_bottom + hand_depth * approach;
    left_center = left_bottom + 0.5 * (left_top - left_bottom);
    right_center = right_bottom + 0.5 * (right_top - right_bottom);
    base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * approach;
    approach_center = base_center - 0.04 * approach;

    base = createHandBaseMarker(left_bottom, right_bottom,
        rot, 0.02, hand_height, i, frame_id);

         left_finger = createFingerMarker(left_center,
        rot, hand_depth, finger_width, hand_height, i * 3, frame_id);
    right_finger = createFingerMarker(right_center,
        rot, hand_depth, finger_width, hand_height, i * 3 + 1, frame_id);
    hand = createFingerMarker(approach_center,
        rot, 0.08, finger_width, hand_height, i * 3 + 2, frame_id);
  /*  marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.ns = "finger";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = tr.x();
    marker.pose.position.y = tr.y();
    marker.pose.position.z = tr.z();
    marker.lifetime = rclcpp::Duration(0, 0);

  // use orientation of hand frame
    Eigen::Quaterniond quat(rot);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // these scales are relative to the hand frame (unit: meters)
    marker.scale.x = 0.01;  // forward direction
    marker.scale.y = hands[i].width;  // hand closing direction
    marker.scale.z = 0.01;  // hand vertical direction

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
*/
    marker_array.markers.push_back(base);
     marker_array.markers.push_back(left_finger);
      marker_array.markers.push_back(right_finger);
       marker_array.markers.push_back(hand);

  /*  left_bottom = hands[i].getGraspBottom() - (hw - 0.5 * finger_width) * hands[i].getBinormal();
    right_bottom = hands[i].getGraspBottom() + (hw - 0.5 * finger_width) * hands[i].getBinormal();
    left_top = left_bottom + hand_depth * hands[i].getApproach();
    right_top = right_bottom + hand_depth * hands[i].getApproach();
    left_center = left_bottom + 0.5 * (left_top - left_bottom);
    right_center = right_bottom + 0.5 * (right_top - right_bottom);
    base_center = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * hands[i].getApproach();
    approach_center = base_center - 0.04 * hands[i].getApproach();

    base = createHandBaseMarker(left_bottom, right_bottom,
        hands[i].getFrame(), 0.02, hand_height, i, frame_id);
    left_finger = createFingerMarker(left_center,
        hands[i].getFrame(), hand_depth, finger_width, hand_height, i * 3, frame_id);
    right_finger = createFingerMarker(right_center,
        hands[i].getFrame(), hand_depth, finger_width, hand_height, i * 3 + 1, frame_id);
    approach = createFingerMarker(approach_center,
        hands[i].getFrame(), 0.08, finger_width, hand_height, i * 3 + 2, frame_id);

    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
    */
  }

  return marker_array;
}


using namespace std::literals::chrono_literals;
using GraspNet = grasp_planner_interfaces::srv::GraspNet;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    cv::Mat rgb = cv::imread(argv[1]);
    cv::Mat depth = cv::imread(argv[2], -1);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("graspnet_client_node");

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

            grasps_rviz_pub->publish(convertToVisualGraspMsg(response->grasps, 0.1,
          0.1, 0.01, 0.01, "camera_optical_frame"));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response ok");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response error");
    }

    rclcpp::shutdown();
}