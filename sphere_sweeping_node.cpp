/*
 * @Author: Cheng Huimin 
 * @Date: 2020-01-07 17:57:57 
 * @Last Modified by: Cheng Huimin
 * @Last Modified time: 2020-01-07 18:16:38
 */
#include <sphere_sweeping.hpp>

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sphere_sweeping");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

    std::cout  << "main" << std::endl;

    SphereSweeping sw ; // DO NOT INCLUDE PARENTHESIS

    std::cout  << "SphereSweeping" << std::endl;

    ros::spin();


}