/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:31 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:15:20
 */


#include<sphere_sweeping.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>

using namespace std;
using namespace Eigen;
class CameraModel : public aslam::cameras::DoubleSphereProjection<aslam::cameras::NoDistortion>{

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
		initialiseDepthCandidates(l_info_msg, r_info_msg);
	}

	// convert to cvImage bridge
	auto cvImage_l = cv_bridge::toCvShare(l_image_msg, "mono8"); // CvImageConstPtr 
	auto cvImage_r = cv_bridge::toCvShare(r_image_msg, "mono8");


	cv::Mat cv_leftImg_source = cvImage_l->image;
	cv::Mat cv_rightImg_source = cvImage_r->image;

	cv::Mat cv_leftImg_8uc1, cv_rightImg_8uc1;
	
	// cv_leftImg_source.convertTo(cv_leftImg_8uc1, CV_RGB, 0.00390625);
	// cv_rightImg_source.convertTo(cv_rightImg_8uc1, CV_RGB, 0.00390625);

	// std::cout << "size" << cv_leftImg_8uc1.cols << "," << cv_leftImg_8uc1.rows << std::endl;

	// Fast detector
	const int fast_th = 20;
	auto detector = cv::FastFeatureDetector::create(fast_th,true,cv::FastFeatureDetector::TYPE_9_16);

	std::vector< cv::KeyPoint > keysl;

	

	detector->detect(cv_leftImg_source, keysl);

	// std::cout << "FAST feature no. = " << keysl.size() << std::endl;

	// cv::imshow("Left", cv_leftImg_source);
	// cv::waitKey(1);
	

}

struct DSParameters{
		double xi;
		double alpha;
		double focalLengthU;
		double focalLengthV;
		double imageCenterU;
		double imageCenterV;
		int resolutionU;
		int resolutionV;
		Eigen::Affine3d T_cn_cnm1 = Eigen::Affine3d::Identity();
};



void cameraInfo2DSParam(const sensor_msgs::CameraInfoConstPtr info, DSParameters& param)
{
	// Intrinsics are stored in the K matrix
	param.xi = info->K[0];
	param.alpha = info->K[1];
	param.focalLengthU = info->K[2];
	param.focalLengthV = info->K[3];
	param.imageCenterU = info->K[4];
	param.imageCenterV = info->K[5];
	param.resolutionU = info->width;
	param.resolutionV = info->height;
	if(info->P[11]){
		param.T_cn_cnm1(0,0) = info->P[0]; param.T_cn_cnm1(0,1) = info->P[1]; param.T_cn_cnm1(0,2) = info->P[2]; param.T_cn_cnm1(0,3) = info->P[3];
		param.T_cn_cnm1(1,0) = info->P[4]; param.T_cn_cnm1(1,1) = info->P[5]; param.T_cn_cnm1(1,2) = info->P[6]; param.T_cn_cnm1(1,3) = info->P[7];
		param.T_cn_cnm1(2,0) = info->P[8]; param.T_cn_cnm1(2,1) = info->P[9]; param.T_cn_cnm1(2,2) = info->P[10]; param.T_cn_cnm1(2,3) = info->P[11];
	}
	cout<<"camera_parameters: "<<endl;
	cout<<param.xi<<", "<<param.alpha<<", "<<param.focalLengthU<<", "<<param.focalLengthV<<", "
	<<param.imageCenterU<<", "<<param.imageCenterV<<", "<<param.resolutionU<<", "<<param.resolutionV<<endl;
	cout<<"T_cn_cm1: "<<endl;
	cout<<param.T_cn_cnm1.matrix()<<endl;
}

void SphereSweeping::initialiseDepthCandidates(const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg)
{	

	cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);


	// Initialised Camera Models
	DSParameters pl, pr;
	
	cameraInfo2DSParam(l_info_msg, pl);
	cameraInfo2DSParam(r_info_msg, pr);

	caml = new CameraModel(pl.xi, pl.alpha, pl.focalLengthU, pl.focalLengthV, pl.imageCenterU, pl.imageCenterV, pl.resolutionU, pl.resolutionV);
	camr = new CameraModel(pr.xi, pr.alpha, pr.focalLengthU, pr.focalLengthV, pr.imageCenterU, pr.imageCenterV, pr.resolutionU, pr.resolutionV);
	// Initialise all depth candidates, based on unit pixel disparity between adjacent spheres
	constexpr int N = 20;
	depth_candidates.resize(N);

	cv::waitKey(1);
	initialised = true;
}