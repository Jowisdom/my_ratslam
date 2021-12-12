/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

//ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif

using namespace ratslam;
ratslam::LocalViewMatch *lv = NULL;


/*函数功能：回调函数，处理订阅的图像信息
 * input: image
 * output: vt_output.current_id
 *          vt_output.relative_rad
 *          */
void image_callback(sensor_msgs::ImageConstPtr image, ros::Publisher *pub_vt) {
    ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);


//    try {
//        cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
//        cv::waitKey(30);
//    }
//    catch (cv_bridge::Exception &e) {
//        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
//    }



    static ratslam_ros::ViewTemplate vt_output;//创建一个ViewTemplate类型的消息vt_output

    /*
     * on_image:
     * 1、将原始图片转化为灰度图，
     * 2、裁切图片，
     * 3、并池化为指定大小的view_image,
     * 4、global normalization and patch normalization
     * 5、然后与存储的template对比，返回匹配到的模板id或新建立的模板id*/
//    lv->on_image(&image->data[0], (image->encoding == "bgr8" ? false : true), image->width, image->height);

    lv->on_image_ORB(image);
//    vt_output.header.stamp = ros::Time::now();
//    vt_output.header.seq++;
//    vt_output.current_id = lv->get_current_vt(); //得到当前激活的模板id
//    vt_output.relative_rad = lv->get_relative_rad(); //得到模板对应的角度
//
//    pub_vt->publish(vt_output); //发布消息

//#ifdef HAVE_IRRLICHT
//    if (use_graphics) {
//        lvs->draw_all();
//    }
//#endif
}



/*---------------------------------------------------------------------------------------------------------
函数功能：订阅topic_root + "/camera/image"，接受topic_root + "/camera/image/compressed"消息，
        然后在话题topic_root + "/LocalView/Template"发布ratslam_ros::ViewTemplate类型的消息
        
*/
int main(int argc, char *argv[]) {

    //输出一些无聊的信息
    ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
    ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
    ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
    //判断是否输入了一个参数
//    if (argc < 2) {
//        ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
//        exit(-1);
//    }

    //-----------------------------------------------------------------------------------------------
    //读取配置文件
    //----------------------------------------------------------------------------------------------
    std::string topic_root = "irat_red";
//    // boost库的ptree是一种树形结构，参见https://www.boost.org/doc/libs/1_65_1/doc/html/property_tree/tutorial.html
//    boost::property_tree::ptree settings, ratslam_settings, general_settings;
//
//    read_ini(argv[1], settings); //读入传入的文件名对应的文件，并保存到setting ptree中
//    //读取setting中的子对象["general"]并存入到general_setting ptree中
//    get_setting_child(general_settings, settings, "general", true);
//    //从general_setting中得到["topic_root"]的值并存入topic_root
//    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
//    get_setting_child(ratslam_settings, settings, "ratslam", true);
    //------------------------------------------------------------------------------------------------

    lv = new ratslam::LocalViewMatch();

    if (!ros::isInitialized()) //判断是否ros::init()
    {
        ros::init(argc, argv, "RatSLAMViewTemplate"); //初始化一个“RatSLAMViewTemplate”节点
    }
    ros::NodeHandle node; //新建一个NodeHandle节点，调用节点函数

    //使用上一步的NodeHandl对象node在Master端注册一个Publisher,其中发布消息的类型为ratslam_ros::ViewTemplate，topic名为topic_root + "/LocalView/Template"
    ros::Publisher pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);

    image_transport::ImageTransport it(node); //定义一个专用于图像的NodeHandle的对象

    cv::namedWindow("view");
    //使用上一步创建的节点处理对象创建一个subscriber
    //订阅topic name:topic_root + "/camera/image",
    //回调函数为boost::bind(image_callback, _1, &pub_vt),接受订阅话题的参数
    //boost::bind为绑定函数，参见https://blog.csdn.net/byxdaz/article/details/71527369
    //举个例子
    //void test(int a, int b, int c)
    //boost::bind(test, 1, _1, _2)得到一个函数对象b，当我们调用b(3,4)时，相当于调用test(1,3,4)
    //boost::bind(test, _2, 3, _1)得到一个函数对象b，当我们调用b(3,4)时，相当于调用test(4,3,3)

    image_transport::Subscriber sub = it.subscribe(topic_root + "/camera/image", 1,
                                                   boost::bind(image_callback, _1, &pub_vt));

//#ifdef HAVE_IRRLICHT
//    boost::property_tree::ptree draw_settings;
//    get_setting_child(draw_settings, settings, "draw", true);
//    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
//    if (use_graphics)
//        lvs = new ratslam::LocalViewScene(draw_settings, lv);
//#endif


    ros::spin();
    cv::destroyWindow("view");
    return 0;
}

