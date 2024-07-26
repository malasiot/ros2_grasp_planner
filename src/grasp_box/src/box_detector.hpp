#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <unordered_map>

struct Box
{
    Eigen::Vector3f center_, sz_;
    float theta_;
};

struct Camera
{
    Eigen::Matrix4f tr_;
    float fx_, fy_, cx_, cy_;
};

using PointCloud = std::vector<Eigen::Vector3f> ;

struct BoxCluster {
    std::vector<Eigen::Vector3f> coords_ ;
    float z_ ;
    Eigen::Vector3f center_ ;
    Eigen::Vector3f sz_ ;
    float theta_ ;
};

class BoxDetector
{
public:
    struct Parameters {
        float minz_ = -0.05;        // support surface z value
        float nrm_radius_ = 0.02;   // radius of support neighborhood for normal vector computation
        float min_cluster_sz_ = 0.05;   // detection of small clusters (percent of all points)
        float depth_thresh_ = 0.01; // distance from average z of top surface
        float nrm_sampling_prob_ = 0.05 ; // probability of computing normal for each point in the cloud
        float up_facing_thresh_ = 0.1 ; // dot product between normal and z vector threshold
    };

    BoxDetector() = default ;
    BoxDetector(const Parameters &params) : params_(params) {}

    std::vector<Box> detect(const Camera &cam, const cv::Mat &depth, const cv::Mat &mask, const cv::Mat &seg_mask);

protected:
    Parameters params_;

};