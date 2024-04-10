#include "masked_point_cloud.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_geometry/image_geometry/pinhole_camera_model.h>
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <chrono>
#include <Eigen/Geometry>


MaskedPointCloud::MaskedPointCloud(const rclcpp::NodeOptions &options) : rclcpp::Node("masked_point_cloud", options)
{
    camera_info_topic_ = declare_parameter("camera_info_topic", "/virtual_camera/color/camera_info");
    rgb_topic_ = declare_parameter("rgb_topic", "/virtual_camera/color/image_raw");
    depth_topic_ = declare_parameter("depth_topic", "/virtual_camera/depth/image_raw");
    mask_topic_ = declare_parameter("mask_topic", "robot_mask/image_raw");
    pcl_topic_ = declare_parameter("pcl_topic", "masked/points");
    target_frame_ = declare_parameter("target_frame", "camera_color_optical_frame");
    depth_threshold_ = declare_parameter("depth_threshold", 10) ;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}
using namespace std::literals::chrono_literals ;

void MaskedPointCloud::setup() {
    rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    caminfo_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);

    sync_.reset(new Synchronizer(SyncPolicy(10), rgb_sub_, depth_sub_, caminfo_sub_));
    sync_->registerCallback(std::bind(&MaskedPointCloud::frameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    image_transport_.reset(new image_transport::ImageTransport(shared_from_this()));
    
    mask_sub_ = std::make_shared<image_transport::Subscriber>(image_transport_->subscribe(mask_topic_, 10,
                                                                                          std::bind(&MaskedPointCloud::maskCallback, this, std::placeholders::_1)));
    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic_, 10) ;

    geometry_msgs::msg::TransformStamped t;

        try {
          t = tf_buffer_->lookupTransform(
            "world", target_frame_, 
            tf2::TimePointZero, tf2::Duration(10s));
            camera_transform_ = tf2::transformToEigen(t);
            has_camera_transform_ = true ;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            target_frame_.c_str(), "world", ex.what());
          return;
        }

}

static void maskDepth(const cv::Mat &depth, const cv::Mat &mask, cv::Mat &dmasked, uint depth_thresh) {
    assert(depth.cols == mask.cols && depth.rows == mask.rows) ;

    dmasked = depth.clone() ;
    cv::Mat_<ushort> depthx(depth), maskx(mask), maskedx(dmasked) ;

    for( int i=0 ; i<depthx.rows ; i++ )
        for( int j=0 ; j<depthx.cols ; j++ ) {
            ushort dv = depthx[i][j] ;
            ushort maskv = maskx[i][j] ;
            ushort &masked = maskedx[i][j] ;

            if ( maskv == 0 || dv == 0 ) continue ;
            if ( abs(dv - maskv) < depth_thresh ) masked = 0 ;

        }
}

void MaskedPointCloud::frameCallback(sensor_msgs::msg::Image::ConstSharedPtr colorMsg, 
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

    if ( mask_.data != nullptr ) {
        maskDepth(depth_, mask_, depth_masked_, depth_threshold_) ;
        publishCloud(depth_masked_, *camera_info_);
        frame_ready_ = true ;
    }
}

static void convert(
        const cv::Mat & depth,
        const image_geometry::PinholeCameraModel & model,
        const Eigen::Isometry3d &tr, 
        sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
        double range_max = 0.0,
        bool use_quiet_nan = false
        )
{
    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = 0.001f ;
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
   
    const uint16_t * depth_row = reinterpret_cast<const uint16_t *>(&depth.data[0]);
    int row_step = depth.step / sizeof(uint16_t);
    for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
            uint16_t depth = depth_row[u];

            // Missing points denoted by NaNs
            if ( depth == 0 ) {
                *iter_x = *iter_y = *iter_z = bad_point;
                continue;
            }

            float X = (u - center_x) * depth * constant_x; 
            float Y = (v - center_y) * depth * constant_y;
            float Z = depth * unit_scaling ;


            Eigen::Vector3d tp = tr * Eigen::Vector3d(X, Y, Z) ;

            if ( tp.z() < 0.05 ) {
                 *iter_x = *iter_y = *iter_z = bad_point;
                continue;
            }

            // Fill in XYZ
            *iter_x = tp.x() ;
            *iter_y = tp.y() ;
            *iter_z = tp.z() ;

        }
    }
}

void MaskedPointCloud::publishCloud(const cv::Mat &dim, const sensor_msgs::msg::CameraInfo &caminfo) {
  if ( pcl_pub_->get_subscription_count() > 0 ) {

       
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
                std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = std_msgs::msg::Header();
        cloud_msg->header.stamp = get_clock()->now();
        cloud_msg->header.frame_id = "world" ;
        cloud_msg->height = dim.rows;
        cloud_msg->width = dim.cols;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;
        cloud_msg->fields.clear();
        cloud_msg->fields.reserve(2);


        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        // g_cam_info here is a sensor_msg::msg::CameraInfo::SharedPtr,
        // which we get from the depth_camera_info topic.
        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(caminfo);

        convert(dim, model, camera_transform_, cloud_msg);

        pcl_pub_->publish(*cloud_msg) ;

    }
}

std::tuple<cv::Mat, cv::Mat, sensor_msgs::msg::CameraInfo> MaskedPointCloud::getFrame() {
        std::lock_guard<std::mutex> frame_lock_(frame_mutex_);

        return { rgb_.clone(), depth_masked_.clone(), *camera_info_} ;
}


void MaskedPointCloud::maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    auto depthPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == sensor_msgs::image_encodings::MONO16)
    {
        mask_ = depthPtr->image; // no conversion needed
    }

}