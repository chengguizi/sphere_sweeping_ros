/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:45 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:13:52
 */

#ifndef SPHERE_SWEEPING_HPP
#define SPHERE_SWEEPING_HPP

#include <stereo_processor.h>
#include <iostream>


class SphereSweeping : public StereoProcessor{
    
public:
    SphereSweeping() : StereoProcessor(3,
        "/tiscamera_ros/left/image_rect_raw",
        "/tiscamera_ros/right/image_rect_raw",
        "/tiscamera_ros/left/camera_info",
        "/tiscamera_ros/right/camera_info")
    {

    }

    ~SphereSweeping()
    {
        std::cout << "~SphereSweeping()" << std::endl;
    }

private:

    void imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg,
								const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg);
};

#endif /* SPHERE_SWEEPING_HPP */
