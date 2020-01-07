/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:31 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:15:20
 */


#include<sphere_sweeping.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// #include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>

class CameraModel : aslam::cameras::DoubleSphereProjection<aslam::cameras::NoDistortion>{

public:
	CameraModel(double xi, double alpha, double focalLengthU, double focalLengthV,
                 double imageCenterU, double imageCenterV, int resolutionU,
                 int resolutionV) : 
				 DoubleSphereProjection(xi,alpha,focalLengthU,focalLengthV,imageCenterU,imageCenterV,resolutionU,resolutionV){}
};

void SphereSweeping::imageCallback(	const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg,
								const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg)
{
    std::cout  << "image " << l_info_msg->header.seq << std::endl;

	if(!initialised){
		std::cout << "Initialising Double Sphere Models" << std::endl;
		initialiseDepthCandidates(l_image_msg, r_image_msg);
	}

	// convert to cvImage bridge
	auto cvImage_l = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO16); // CvImageConstPtr 
	auto cvImage_r = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO16);


	cv::Mat cv_leftImg_source = cvImage_l->image;
	cv::Mat cv_rightImg_source = cvImage_r->image;

	cv::Mat cv_leftImg_8uc1, cv_rightImg_8uc1;
	
	cv_leftImg_source.convertTo(cv_leftImg_8uc1, CV_8UC1);
	cv_rightImg_source.convertTo(cv_rightImg_8uc1, CV_8UC1);

	// Fast detector
	const int fast_th = 20;
	auto detector = cv::FastFeatureDetector::create(fast_th,true,cv::FastFeatureDetector::TYPE_9_16);

}

void SphereSweeping::initialiseDepthCandidates(const sensor_msgs::ImageConstPtr l_image_msg,
								const sensor_msgs::ImageConstPtr r_image_msg)
{	
	// Initialised Camera Models

	struct Parameters{
		double xi;
		double alpha;
		double focalLengthU;
		double focalLengthV;
		double imageCenterU;
		double imageCenterV;
		int resolutionU;
		int resolutionV;
	};

	Parameters pl, pr;

	caml = new CameraModel(pl.xi, pl.alpha, pl.focalLengthU, pl.focalLengthV, pl.imageCenterU, pl.imageCenterV, pl.resolutionU, pl.resolutionV);

	//   DoubleSphereProjection(double xi, double alpha, double focalLengthU, double focalLengthV,
    //              double imageCenterU, double imageCenterV, int resolutionU,
    //              int resolutionV);

	// Initialise all depth candidates, based on unit pixel disparity between adjacent spheres

	initialised = true;
}