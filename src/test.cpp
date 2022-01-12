//
// Created by wisdom on 1/12/22.
//

#include <iostream>

using namespace std;

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ratslam_ros/ViewTemplate.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>


#include "ratslam/local_view_match.h"

#if HAVE_IRRLICHT

#include "graphics/local_view_scene.h"

ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif

using namespace ratslam;

void image_feedback(sensor_msgs::ImageConstPtr image){
    ROS_INFO_STREAM("recied the images");
    cv::Mat img;
    try {
        //将ros::msg转化为cv::mat

        img = cv_bridge::toCvShare(image, "mono8")->image;
    } catch (...) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", image->encoding.c_str());
        return;
    }

    cv::imshow("img_view", img);
    cv::waitKey(2);
}



int main(int argc, char **argv){
    ROS_INFO_STREAM("Begin to test");

    if (!ros::isInitialized()) //判断是否ros::init()
    {
        ros::init(argc, argv,"RatSLAMTest"); //初始化一个“RatSLAMViewTemplate”节点
    }
    ros::NodeHandle test_node; //新建一个NodeHandle节点，调用节点函数
    image_transport::ImageTransport it(test_node);
    image_transport::Subscriber sub = it.subscribe("test/camera/image", 1,
                                                   image_feedback);
    cv::namedWindow("img_view");
    ros::spin();
    cv::destroyWindow("img_view");
    return 0;



}