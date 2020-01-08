/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:31 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:15:20
 */


#include<sphere_sweeping.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/features2d.hpp>

#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>

#include <algorithm>
#include <numeric> 

template <typename T>
std::vector<size_t> sort_indexes(const std::array<T, SphereSweeping::depthN> &v) {

  // initialize original index locations
  std::vector<size_t> idx(SphereSweeping::depthN);
  std::iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}


class CameraModel : public aslam::cameras::DoubleSphereProjection<aslam::cameras::NoDistortion>{

public:
	CameraModel(double xi, double alpha, double focalLengthU, double focalLengthV,
                 double imageCenterU, double imageCenterV, int resolutionU,
                 int resolutionV) : 
				 DoubleSphereProjection(xi,alpha,focalLengthU,focalLengthV,imageCenterU,imageCenterV,resolutionU,resolutionV){}
};

SphereSweeping::KernelType SphereSweeping::squareKernelImagePlane(const cv::Point2f pt, const cv::Mat img)
{
	KernelType kernel;

	constexpr int HALF_SIZE = KERNEL_SIZE / 2;
	const float anchor_x = pt.x - HALF_SIZE;
	const float anchor_y = pt.y - HALF_SIZE;
	for (int i = 0; i < KERNEL_SIZE; i++)
	{
		for (int j = 0; j < KERNEL_SIZE; j++)
		{
			int x = i + anchor_x;
			int y = j + anchor_y;
			// check for bound
			if (x < 0 || x >= img.cols || y < 0 || y >= img.rows)
				kernel(i,j) = 0;
			else
			{
				// at<>(row, col)
				kernel(i,j) = img.at<unsigned char>(cv::Point(x,y));
			}
			
		}
	}

	return kernel;
}


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


	// Fast detector
	const int fast_th = 30;
	auto detector = cv::FastFeatureDetector::create(fast_th,true,cv::FastFeatureDetector::TYPE_9_16);

	// detect FAST features into the class keys variable
	detector->detect(cv_leftImg_source, keys);
	std::cout << "FAST feature no. = " << keys.size() << std::endl;

	cost_map.resize(keys.size());

	assert(caml != nullptr && camr != nullptr);

	#ifdef USE_NAIVE
	CostMapType::iterator cost = cost_map.begin();

	for (auto& key : keys){

		

		for (size_t idx = 0; idx < depthN; idx++){

			KernelType kernel_left = squareKernelImagePlane(key.pt, cv_leftImg_source);

			// Use of template argument deduction 
			Eigen::Vector3d point_caml, point_camr;
			caml->keypointToEuclidean(Eigen::Vector2d(key.pt.x, key.pt.y), point_caml);

			// std::cout << point_caml.transpose() << std::endl;

			// Perform basis transformation from left camera to right camera
			point_camr = pr.T_cn_cnm1 * (point_caml * depth_candidates[idx]);
			Eigen::Vector2d right_point;
			caml->euclideanToKeypoint(point_camr, right_point);

			KernelType kernel_right = squareKernelImagePlane(cv::Point2f(right_point[0], right_point[1]), cv_rightImg_source);

			// Frobenius norm
			(*cost)[idx] = (kernel_right - kernel_left).norm() / (KERNEL_SIZE * KERNEL_SIZE);

			// std::cout  << " " << (*cost)[idx];
			
		}

		auto sorted_idx = sort_indexes<double>((*cost));

		key.response = depth_candidates[sorted_idx[0]];
		// std::partial_sort((*cost).begin(), (*cost).begin()+1, (*cost).end());

		// std::cout  << std::endl << sorted_idx;

		cost++;
		// std::cout  << "done " << std::endl;
	}

	
	#endif

	// Updated cost_map

	// for ()

	showDebugImage("Left", cv_leftImg_source);
	

}




void cameraInfo2DSParam(const sensor_msgs::CameraInfoConstPtr info, SphereSweeping::DSParameters& param)
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
	std::cout<<"camera_parameters: "<<std::endl;
	std::cout<<param.xi<<", "<<param.alpha<<", "<<param.focalLengthU<<", "<<param.focalLengthV<<", "
	<<param.imageCenterU<<", "<<param.imageCenterV<<", "<<param.resolutionU<<", "<<param.resolutionV<<std::endl;
	std::cout<<"T_cn_cm1: "<<std::endl;
	std::cout<<param.T_cn_cnm1.matrix()<<std::endl;
}

void SphereSweeping::initialiseDepthCandidates(const sensor_msgs::CameraInfoConstPtr l_info_msg,
								const sensor_msgs::CameraInfoConstPtr r_info_msg)
{	

	if (visualisation)
		cv::namedWindow("Left", cv::WINDOW_AUTOSIZE);


	// Initialised Camera Models
	
	
	cameraInfo2DSParam(l_info_msg, pl);
	cameraInfo2DSParam(r_info_msg, pr);

	caml = new CameraModel(pl.xi, pl.alpha, pl.focalLengthU, pl.focalLengthV, pl.imageCenterU, pl.imageCenterV, pl.resolutionU, pl.resolutionV);
	camr = new CameraModel(pr.xi, pr.alpha, pr.focalLengthU, pr.focalLengthV, pr.imageCenterU, pr.imageCenterV, pr.resolutionU, pr.resolutionV);
	// Initialise all depth candidates, based on unit pixel disparity between adjacent spheres
	int N = depth_candidates.size();
	for(int i = 1; i <= N; i++){
		Eigen::Vector2d keyPoint(pr.imageCenterU - i ,pr.imageCenterV);
		Eigen::Vector3d outPoint;
		camr->keypointToEuclidean(keyPoint,outPoint);
		depth_candidates[i-1] = outPoint[2]/outPoint[0] * pr.T_cn_cnm1(0,3);
		std::cout<<"depth"<<i<<": "<< depth_candidates[i-1]<< std::endl;
	}

	cv::waitKey(1);
	initialised = true;
}

inline void SphereSweeping::showDebugImage(const std::string title, const cv::Mat img)
{
	if (!visualisation)
		return;
	
	cv::Mat outImg;
	if (img.type() != CV_8UC3)
		cv::cvtColor(img, outImg, CV_GRAY2BGR);
	else 
		outImg = img;

	cv::Mat keysMat;
	cv::Mat colorCoding;
	for(auto & key:keys){
		keysMat.push_back(1.0/key.response);
	}
	double min = 1.0/depth_candidates[0], max = 1.0/depth_candidates[depthN-1];
	keysMat.convertTo(keysMat,CV_8UC3, 255 / (max-min), -min);
	cv::applyColorMap(keysMat,colorCoding,cv::COLORMAP_HOT);
	int color_index = 0;
	for(auto & key:keys){
		rectangle(outImg, cv::Point(key.pt.x-KERNEL_SIZE/2,key.pt.y-KERNEL_SIZE/2), cv::Point(key.pt.x+KERNEL_SIZE/2,key.pt.y+KERNEL_SIZE/2), colorCoding.at<cv::Vec3b>(cv::Point(0, color_index)), -1);
		color_index++;
	}
	cv::resize(outImg, outImg, cv::Size(), 0.5, 0.5);
	cv::imshow(title, outImg);
	cv::waitKey(1);
}