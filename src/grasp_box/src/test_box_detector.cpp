#include "box_detector.hpp"

int main(int argc, char *argv[])
{
    Camera cam;
    cam.cx_ = 648;
    cam.cy_ = 369;
    cam.fx_ = 911;
    cam.fy_ = 910;

    cam.tr_ << 0.0654116, -0.743699, 0.665306, -0.0302594,
        -0.997056, -0.0754376, 0.0137022, 0.0385303,
        0.0399988, -0.664244, -0.746445, 1.0363,
        0, 0, 0, 1;

    auto depth = cv::imread(argv[1], -1);
    auto seg_mask = cv::imread(argv[2]);
    auto mask = cv::imread(argv[3], -1);

    BoxDetector detector ;
    
    auto boxes = detector.detect(cam, depth, mask, seg_mask);
}