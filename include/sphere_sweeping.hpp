/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:45 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:13:52
 */

#ifndef SPHERE_SWEEPING_HPP
#define SPHERE_SWEEPING_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <stereo_processor.h>
#include <iostream>

#include <vector>

#define USE_NAIVE

class CameraModel;


class SphereSweeping : public StereoProcessor{
    
public:
    SphereSweeping() : StereoProcessor(3,
        "/tiscamera_ros/left/image_rect_raw",
        "/tiscamera_ros/right/image_rect_raw",
        "/tiscamera_ros/left/camera_info",
        "/tiscamera_ros/right/camera_info"), initialised(false)
    {

    }

    ~SphereSweeping()
    {
        std::cout << "~SphereSweeping()" << std::endl;
    }

private:

    bool initialised;
    bool visualisation = true;

    static constexpr int KERNEL_SIZE = 7;
    typedef Eigen::Matrix<float, KERNEL_SIZE, KERNEL_SIZE> KernelType;

    static constexpr int depthN = 20;
    std::array<double, depthN> depth_candidates;

    // store the list of keypoints detected
    std::vector<cv::KeyPoint> keys;
    
    typedef std::array<std::vector<double>, depthN> CostMapType;
    // each entry is a vector of all points' cost for one particular depth candidates
    // index is the same as keys.
    CostMapType cost_map;

    CameraModel *caml, *camr; // camera model for both left and right cameras

    void initialiseDepthCandidates(const sensor_msgs::CameraInfoConstPtr l_info_msg, const sensor_msgs::CameraInfoConstPtr r_info_msg);

    // Extract a square kernel, around a image plane point
    KernelType squareKernelImagePlane(const cv::Point2f pt, const cv::Mat img);

    void imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg,
								const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg);

    void showDebugImage(const std::string title, const cv::Mat img);
};

#endif /* SPHERE_SWEEPING_HPP */
