#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <grasp_planner_interfaces/srv/grasp_net.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include <Eigen/Geometry>

#include "move_group_interface.hpp"

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

    rclcpp::NodeOptions options ;
    options.automatically_declare_parameters_from_overrides(true) ;
    std::shared_ptr<MoveGroupInterfaceNode> mgi(new MoveGroupInterfaceNode(options)) ;
    mgi->setup() ;

    std::thread t = std::thread([&]() {
        rclcpp::spin(mgi) ;
        rclcpp::shutdown() ;

    }) ;

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

   
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    geometry_msgs::msg::TransformStamped tr  = tf_buffer->lookupTransform(
                     "world", "camera_optical_frame", tf2::TimePointZero, tf2::durationFromSec(5));

    auto camera_tr = tf2::transformToEigen(tr);
 
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        cout << response->grasps.size() << endl ;

        for( auto &g: response->grasps ) {
            auto &trv = g.translation ;
            auto &rotv = g.rotation ;

            Eigen::Vector3d c(trv[0], trv[1], trv[2]) ;
            Eigen::Matrix3d rot ;
            rot << rotv[0], rotv[1], rotv[2], 
            rotv[3], rotv[4], rotv[5],
            rotv[6], rotv[7], rotv[8] ;

            Isometry3d p ;
            p.setIdentity() ;
            p.linear() = rot ;
            p.translation() = c ;

            p = camera_tr * p ;

            c = p.translation() ;
            rot = p.linear() ;

            trv[0] = c.x() ; trv[1] = c.y() ; trv[2] = c.z() ;
            rotv[0] = rot(0, 0) ; rotv[1] = rot(0, 1) ; rotv[2] = rot(0, 2) ;
            rotv[3] = rot(1, 0) ; rotv[4] = rot(1, 1) ; rotv[5] = rot(1, 2) ;
            rotv[6] = rot(2, 0) ; rotv[7] = rot(2, 1) ; rotv[8] = rot(2, 2) ;
        }

            grasps_rviz_pub->publish(convertToVisualGraspMsg(response->grasps, 0.05, 0.01, 0.01, "world"));

            vector<grasp_planner_interfaces::msg::Grasp> grasps_filtered ;
            mgi->filterGrasps(response->grasps, grasps_filtered) ;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response ok");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "response error");
    }

    rclcpp::shutdown();
}