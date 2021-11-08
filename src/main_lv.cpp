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

ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif

using namespace ratslam;
ratslam::LocalViewMatch *lv = NULL;

void image_callback(sensor_msgs::ImageConstPtr image, ros::Publisher *pub_vt) {
    ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

    static ratslam_ros::ViewTemplate vt_output;

    lv->on_image(&image->data[0], (image->encoding == "bgr8" ? false : true), image->width, image->height);

    vt_output.header.stamp = ros::Time::now();
    vt_output.header.seq++;
    vt_output.current_id = lv->get_current_vt();
    vt_output.relative_rad = lv->get_relative_rad();

    pub_vt->publish(vt_output);

#ifdef HAVE_IRRLICHT
    if (use_graphics) {
        lvs->draw_all();
    }
#endif
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
    if (argc < 2) {
        ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
        exit(-1);
    }

    //-----------------------------------------------------------------------------------------------
    //读取配置文件
    //----------------------------------------------------------------------------------------------
    std::string topic_root = "";
    // boost库的ptree是一种树形结构，参见https://www.boost.org/doc/libs/1_65_1/doc/html/property_tree/tutorial.html
    boost::property_tree::ptree settings, ratslam_settings, general_settings;

    read_ini(argv[1], settings); //读入传入的文件名对应的文件，并保存到setting ptree中
    //读取setting中的子对象["general"]并存入到general_setting ptree中
    get_setting_child(general_settings, settings, "general", true);
    //从general_setting中得到["topic_root"]的值并存入topic_root
    get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");
    get_setting_child(ratslam_settings, settings, "ratslam", true);
    //------------------------------------------------------------------------------------------------

    lv = new ratslam::LocalViewMatch(ratslam_settings);

    if (!ros::isInitialized()) //判断是否ros::init()
    {
        ros::init(argc, argv, "RatSLAMViewTemplate"); //初始化一个“RatSLAMViewTemplate”节点
    }
    ros::NodeHandle node; //新建一个NodeHandle节点，调用节点函数

    //使用上一步的NodeHandl对象node在Master端注册一个Publisher,其中发布消息的类型为ratslam_ros::ViewTemplate，topic名为topic_root + "/LocalView/Template"
    ros::Publisher pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);

    image_transport::ImageTransport it(node); //定义一个专用于图像的NodeHandle的对象
    //使用上一步创建的节点处理对象创建一个subscriber
    //订阅topic name:topic_root + "/camera/image",
    //回调函数为boost::bind(image_callback, _1, &pub_vt),接受订阅话题的参数
    //boost::bind为绑定函数，参见https://blog.csdn.net/byxdaz/article/details/71527369
    image_transport::Subscriber sub = it.subscribe(topic_root + "/camera/image", 0,
                                                   boost::bind(image_callback, _1, &pub_vt));

#ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (use_graphics)
        lvs = new ratslam::LocalViewScene(draw_settings, lv);
#endif

    ros::spin();

    return 0;
}
