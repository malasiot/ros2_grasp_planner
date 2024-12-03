#include <grasp_planner_msgs/msg/grasp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

using namespace Eigen ;


visualization_msgs::msg::Marker createHandBaseMarker(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & end, const Eigen::Quaterniond & quat, double width, double height, int32_t &id,
  const std::string & frame_id, const Vector4f &clr, const std::string &ns)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = rclcpp::Duration(0, 0);

  // use orientation of hand frame
 
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = width;  // forward direction
  marker.scale.y = (end - start).norm();  // hand closing direction
  marker.scale.z = height;  // hand vertical direction

  marker.color.a = clr.w();
  marker.color.r = clr.x();
  marker.color.g = clr.y();
  marker.color.b = clr.z();

  ++id ;

  return marker;
}


visualization_msgs::msg::Marker createFingerMarker(
  const Eigen::Vector3d & center,
  const Eigen::Quaterniond & quat, double length, double width, double height, int32_t &id,
  const std::string & frame_id, const Vector4f &clr, const std::string &ns)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);
    marker.lifetime = rclcpp::Duration(0, 0);
    
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // these scales are relative to the hand frame (unit: meters)
    marker.scale.x = length;  // forward direction
    marker.scale.y = width;  // hand closing direction
    marker.scale.z = height;  // hand vertical direction

    marker.color.a = clr.w();
    marker.color.r = clr.x();
    marker.color.g = clr.y();
    marker.color.b = clr.z();

    ++id ;
    return marker;
}



visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
  const std::vector<grasp_planner_msgs::msg::Grasp> & hands,
  double hand_length, double finger_width, double finger_height,
  const std::string & frame_id, const Vector4f &clr, const std::string &ns) {

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker left_finger, right_finger, base, hand;
    Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center,
    approach_center,  base_center;
    
    int32_t id = 0 ;
    for (uint32_t i = 0; i < hands.size(); i++) {
      const auto &trv = hands[i].translation ;
      const auto &rotv = hands[i].rotation ;
 
      Eigen::Vector3d c(trv.x, trv.y, trv.z) ;
      Eigen::Quaterniond rot(rotv.w, rotv.x, rotv.y, rotv.z) ;
      Eigen::Matrix3d m = rot.toRotationMatrix() ;

      static const double base_depth = 0.02 ;
   
      auto approach = m.col(0) ; 
      auto binormal = m.col(1) ;
      double hw = 0.5 * hands[i].width;
      double hand_depth = hands[i].depth ;

      left_bottom =  c - (hw + finger_width) * binormal - approach * (base_depth + finger_width/2);
      right_bottom = c + (hw + finger_width) * binormal - approach * (base_depth + finger_width/2);
      left_center = c - (hw + finger_width/2) * binormal + approach * ( hand_depth - base_depth )/2.0 ;
      right_center = c + (hw + finger_width/2) * binormal + approach * ( hand_depth - base_depth )/2.0 ;
      approach_center = c - approach * ( base_depth + finger_width + hand_length/2);

      base = createHandBaseMarker(left_bottom, right_bottom, rot, finger_width, finger_height, id, frame_id, clr, ns);
      left_finger = createFingerMarker(left_center, rot, hand_depth + base_depth, finger_width, finger_height, id, frame_id, clr, ns);
      right_finger = createFingerMarker(right_center, rot, hand_depth + base_depth, finger_width, finger_height, id, frame_id, clr, ns);
      hand = createFingerMarker(approach_center, rot, hand_length, finger_width, finger_height, id, frame_id, clr, ns);

      marker_array.markers.push_back(base);
      marker_array.markers.push_back(left_finger);
      marker_array.markers.push_back(right_finger);
      marker_array.markers.push_back(hand);
  }

  return marker_array;
}
