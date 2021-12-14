#include "local_view_match.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>

using namespace std;

#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>


namespace ratslam {

    LocalViewMatch::LocalViewMatch() {

    }


    LocalViewMatch::~LocalViewMatch() {

    }


    void LocalViewMatch::on_image_ORB(sensor_msgs::ImageConstPtr &image) {
        cv::Mat img;
        try {
            //将ros::msg转化为cv::mat
            img = cv_bridge::toCvShare(image, "mono8")->image;
        } catch (...) {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.", image->encoding.c_str());
            return;
        }

        cv::imshow("img_view", img);

        //提取图像ORB特征点和计算描述子

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        try {
            detector->detect(img, keypoints);
            descriptor->compute(img, keypoints, descriptors);
            if (keypoints.size()<=100){
                return;
            }
            cv::Mat outimg_with_keypoints;
            drawKeypoints(img, keypoints, outimg_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
            imshow("ORB features", outimg_with_keypoints);
        } catch (...) {
            ROS_INFO_STREAM("提取特征点失败");
            return;
        }

        //如果size(ORB_vt_set)=0，则保存特征点和描述子ORB_vt到ORB_vt_set
        if (keyPoint_pool.empty() && descriptor_pool.empty()) {
            keyPoint_pool.push_back(keypoints);
            descriptor_pool.push_back(descriptors);
            return;
        }

        ROS_INFO("开始匹配......");
        ROS_INFO_STREAM("The size of descriptor_pool: =" << descriptor_pool.size() << endl);

        //如果size(ORB_vt_set)！=0，与ORB_vt_set进行匹配
        std::vector<int> num_match;
        for (int i = 0; i < descriptor_pool.size(); ++i) {
            //ROS_INFO_STREAM("与第["<<i<<"]个模板进行匹配"<<endl);
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
            vector<cv::DMatch> matches;
            try {
                matcher->match(descriptors, descriptor_pool[i], matches);
                //计算最小距离和最大距离
                auto min_max = minmax_element(matches.begin(), matches.end(),
                                              [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
                double min_dist = min_max.first->distance;
                double max_dist = min_max.second->distance;
                //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
                std::vector<cv::DMatch> good_matches;
                for (auto &item_match : matches) {
                    if (item_match.distance <= max(1.5*min_dist,20.0)) {
                        good_matches.push_back(item_match);
                    }
                }
                num_match.push_back(good_matches.size());
            } catch (...) {
                ROS_INFO_STREAM("匹配失败");
                num_match.push_back(0);
            }
        }


        ROS_INFO_STREAM("num_match_size=" << num_match.size() << endl;);
        auto biggest_num = std::max_element(std::begin(num_match), std::end(num_match));

        if (*biggest_num > 350) {
            // 匹配到或者在ORB_vt_set找到匹配到的id
            current_vt = biggest_num - num_match.begin();
//            ROS_INFO_STREAM("big_id=" << current_vt << endl);
//            ROS_INFO_STREAM("The size of descriptor_pool:" << descriptor_pool.size() << endl);
        } else {
            //没匹配到创建新的ORB_vt到ORB_vt_set,
            keyPoint_pool.push_back(keypoints);
            descriptor_pool.push_back(descriptors);
            current_vt = descriptor_pool.size() - 1;
        }
        vt_relative_rad = 0;
        ROS_INFO_STREAM("激活的id:current_vt = " << current_vt);
        ROS_INFO_STREAM("The size of descriptor_pool:" << descriptor_pool.size() << endl);

        cv::waitKey(30);

    }
}
//                    try {

//                    ROS_INFO("匹配特征点成功");
//                        ROS_INFO_STREAM("The size of matches:" << matches.size() << endl);
//                 -- 第四步:匹配点对筛选

//
//                    //    printf("-- Max dist : %f \n", max_dist);
//                    //    printf("-- Min dist : %f \n", min_dist);
//
//                    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
//                    std::vector<cv::DMatch> good_matches;
//                    for (int j = 0; j < descriptors.rows; j++) {
//                        if (matches[j].distance <= max(2 * min_dist, 40.0)) {
//                            good_matches.push_back(matches[j]);
//                        }
//                    }
//                        num_match.push_back(matches.size());

//                    catch (...) {
//                        ROS_INFO("match error");




//                    ROS_INFO_STREAM("The size of num_match:" << num_match.size() << endl);
//
//

//
//                auto biggest_num = std::max_element(std::begin(num_match), std::end(num_match));
//
//                if (*biggest_num > 400) {
//                    // 匹配到或者在ORB_vt_set找到匹配到的id
//                    current_vt = biggest_num - num_match.begin();
//                    ROS_INFO_STREAM("big_id="<<current_vt<<endl);
//                    ROS_INFO_STREAM("The size of descriptor_pool:" << descriptor_pool.size() << endl);
//                } else {
//                    //没匹配到创建新的ORB_vt到ORB_vt_set,
//                    keyPoint_pool.push_back(keypoints);
//                    descriptor_pool.push_back(descriptors);
//                    current_vt = descriptor_pool.size() - 1;
//                }
//                vt_relative_rad = 0;
//                ROS_INFO_STREAM("激活的id:current_vt = " << current_vt << endl);
//                ROS_INFO_STREAM("The size of descriptor_pool:" << descriptor_pool.size() << endl);

//
//            }



//
//        //如果size(ORB_vt_set)=0，则保存特征点和描述子ORB_vt到ORB_vt_set
//        if (keyPoint_pool.empty() && descriptor_pool.empty()) {
//            keyPoint_pool.push_back(keypoints);
//            descriptor_pool.push_back(descriptors);
//        } else {
//            //如果size(ORB_vt_set)！=0，与ORB_vt_set进行匹配
//            std::vector<int> num_match;
//            for (int i = 0; i < descriptor_pool.size(); ++i) {
//                cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//                vector<cv::DMatch> matches;
//                matcher->match(descriptors, descriptor_pool[i], matches);
//                //    fmt::print("old_matches.size()={}\n",matches.size());
//                //-- 第四步:匹配点对筛选
//                // 计算最小距离和最大距离
//                auto min_max = minmax_element(matches.begin(), matches.end(),
//                                              [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
//                double min_dist = min_max.first->distance;
//                double max_dist = min_max.second->distance;
//
//                //    printf("-- Max dist : %f \n", max_dist);
//                //    printf("-- Min dist : %f \n", min_dist);
//
//                //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
//                std::vector<cv::DMatch> good_matches;
//                for (int i = 0; i < descriptors.rows; i++) {
//                    if (matches[i].distance <= max(2 * min_dist, 40.0)) {
//                        good_matches.push_back(matches[i]);
//                    }
//                }
//                num_match.push_back(good_matches.size());
//            }
//            auto biggest_num = std::max_element(std::begin(num_match), std::end(num_match));
//
//            if (*biggest_num > 100){
//                // 匹配到或者在ORB_vt_set找到匹配到的id
//                current_vt = biggest_num - num_match.begin();
//            }else{
//                //没匹配到创建新的ORB_vt到ORB_vt_set,
//                keyPoint_pool.push_back(keypoints);
//                descriptor_pool.push_back(descriptors);
//                current_vt = descriptor_pool.size()-1;
//            }
//        }
//        vt_relative_rad = 0;











//    void LocalViewMatch::clip_view_x_y(int &x, int &y) {
//        if (x < 0)
//            x = 0;
//        else if (x > TEMPLATE_X_SIZE - 1)
//            x = TEMPLATE_X_SIZE - 1;
//
//        if (y < 0)
//            y = 0;
//        else if (y > TEMPLATE_Y_SIZE - 1)
//            y = TEMPLATE_Y_SIZE - 1;
//
//    }
//
///* void LocalViewMatch::convert_view_to_view_template(bool grayscale)函数功能
// * 将rgb图像转为current_view模板:
// * 1、将原图片池化为指定大小的模板current_view，彩色图片顺带转为灰度图
// * 2、将current_view进行 global normalization和 path normalizaiton
// * 3、计算current_view的均值current_mean
// * */
//    void LocalViewMatch::convert_view_to_view_template(bool grayscale) {
//        int data_next = 0;
//        //图片大小
//        int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;
//        int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;
//        //每个要池化的block的大小
//        //template_size为模板个数,也是block的个数，每个池化后的block都是template的一个像素
//        int x_block_size = sub_range_x / TEMPLATE_X_SIZE;
//        int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;
//        int pos;
//
//        for (unsigned int i; i < current_view.size(); i++)
//            current_view[i] = 0;
//        //--------------------------------------------------------------------------------------------------------------
//        //池化为模板大小
//        //判断图片是否为灰度图：
//        //      if 是：池化后保存到current_view
//        //      if not:将所有像素的RGB值进行累加转化为灰度图，池化后保存到current_view
//        if (grayscale) {
//            for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
//                                                                                                                 y_block_size, y_block_count++) {
//                for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
//                                                                                                                     x_block_size, x_block_count++) {
//                    for (int x = x_block; x < (x_block + x_block_size); x++) {
//                        for (int y = y_block; y < (y_block + y_block_size); y++) {
//                            pos = (x + y * IMAGE_WIDTH);
//                            current_view[data_next] += (double) (view_rgb[pos]);
//                        }
//                    }
//                    current_view[data_next] /= (255.0);
//                    current_view[data_next] /= (x_block_size * y_block_size);
//                    data_next++;
//                }
//            }
//        } else {
//            for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block +=
//                                                                                                                 y_block_size, y_block_count++) {
//                for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block +=
//                                                                                                                     x_block_size, x_block_count++) {
//                    // block中像素累加，池化，block_size表示池化核的size
//                    for (int x = x_block; x < (x_block + x_block_size); x++) {
//                        for (int y = y_block; y < (y_block + y_block_size); y++) {
//                            //计算在block中的像素在原图片中的坐标，view_rgb为按一维矩阵排列，所以（x,y) = (x + y * IMAGE_WIDTH) * 3
//                            pos = (x + y * IMAGE_WIDTH) * 3;
//                            //将RGB像素值累加并保存到current_view到转为灰度图
//                            current_view[data_next] += ((double) (view_rgb[pos]) + (double) (view_rgb[pos + 1])
//                                                        + (double) (view_rgb[pos + 2]));
//                        }
//                    }
//                    //全局平均
//                    current_view[data_next] /= (255.0 * 3.0);
//                    current_view[data_next] /= (x_block_size * y_block_size);
//
//                    data_next++;
//                }
//            }
//        }
//        //--------------------------------------------------------------------------------------------------------------
//        //global normalization
//        if (VT_NORMALISATION > 0) {
//            double avg_value = 0;
//
//            for (unsigned int i = 0; i < current_view.size(); i++) {
//                avg_value += current_view[i];
//            }
//
//            avg_value /= current_view.size();
//            //将current_view像素值均放置于【0，1】之间
//            for (unsigned int i = 0; i < current_view.size(); i++) {
//                current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));
//            }
//        }
//
//        // now do patch normalisation
//        // +- patch size on the pixel, ie 4 will give a 9x9
//        if (VT_PATCH_NORMALISATION > 0) {
//            int patch_size = VT_PATCH_NORMALISATION;
//            int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);
//            double patch_sum;
//            double patch_mean;
//            double patch_std;
//            int patch_x_clip;
//            int patch_y_clip;
//
//            // first make a copy of the view
//            std::vector<double> current_view_copy;
//            current_view_copy.resize(current_view.size());
//            for (unsigned int i = 0; i < current_view.size(); i++)
//                current_view_copy[i] = current_view[i];
//
//            // this code could be significantly optimimised ....
//            // patch normalisation
//            for (int x = 0; x < TEMPLATE_X_SIZE; x++) {
//                for (int y = 0; y < TEMPLATE_Y_SIZE; y++) {
//                    patch_sum = 0;
//                    for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++) {
//                        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++) {
//                            patch_x_clip = patch_x;
//                            patch_y_clip = patch_y;
//                            clip_view_x_y(patch_x_clip, patch_y_clip);
//
//                            patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];
//                        }
//                    }
//                    patch_mean = patch_sum / patch_total;
//
//                    patch_sum = 0;
//                    for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++) {
//                        for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++) {
//                            patch_x_clip = patch_x;
//                            patch_y_clip = patch_y;
//                            clip_view_x_y(patch_x_clip, patch_y_clip);
//
//                            patch_sum += (
//                                    (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean)
//                                    * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
//                        }
//                    }
//
//                    patch_std = sqrt(patch_sum / patch_total);
//
//                    if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)
//                        current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
//                    else {
//                        current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y *
//                                                                                                                  TEMPLATE_X_SIZE] -
//                                                                                            patch_mean) / patch_std) +
//                                                                                          3.0) / 6.0));
//                    }
//                }
//            }
//        }
//
//        double sum = 0;
//
//        // find the mean of the data
//        for (int i = 0; i < current_view.size(); i++)
//            sum += current_view[i];
//
//        current_mean = sum / current_view.size();
//
//    }
//
//// create and add a visual template to the collection
//    int LocalViewMatch::create_template() {
//        templates.resize(templates.size() + 1);
//        VisualTemplate *new_template = &(*(templates.end() - 1));
//
//        new_template->id = templates.size() - 1;
//        double *data_ptr = &current_view[0];
//        new_template->data.reserve(TEMPLATE_SIZE);
//        for (int i = 0; i < TEMPLATE_SIZE; i++)
//            new_template->data.push_back(*(data_ptr++));
//
//        new_template->mean = current_mean;
//
//        return templates.size() - 1;
//    }
//
//// compare a visual template to all the stored templates, allowing for
//// slen pixel shifts in each direction
//// returns the matching template and the MSE
//    void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id) {
//        //如果没有模板，则返回
//        if (templates.size() == 0) {
//            vt_err = DBL_MAX;
//            vt_error = vt_err;
//            return;
//        }
//
//        double *data = &current_view[0];
//        double mindiff, cdiff;
//        mindiff = DBL_MAX;
//
//        vt_err = DBL_MAX;
//        int min_template = 0;
//
//        double *template_ptr;
//        double *column_ptr;
//        double *template_row_ptr;
//        double *column_row_ptr;
//        double *template_start_ptr;
//        double *column_start_ptr;
//        int row_size;
//        int sub_row_size;
//        double *column_end_ptr;
//        VisualTemplate vt;
//        int min_offset;
//
//        int offset;
//        double epsilon = 0.005;
//
//        if (VT_PANORAMIC) {
//
//            BOOST_FOREACH(vt, templates) {
//                            //如果当前current_tempate的模板的均值与匹配的模板的均值相差太大，则认为其不匹配；
//                            if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
//                                continue;
//
//                            // for each vt try matching the view at different offsets
//                            // 错位匹配 at different offsets, 忽略y方向上的偏移，仅在x方向上偏移
//                            // try to fast break based on error already great than previous errors
//                            // handles 2d images shifting only in the x direction
//                            // note I haven't tested on a 1d yet.
//                            for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH) {
//                                cdiff = 0;
//                                template_start_ptr = &vt.data[0] + offset;
//                                column_start_ptr = &data[0];
//                                row_size = TEMPLATE_X_SIZE;
//                                column_end_ptr = &data[0] + TEMPLATE_SIZE - offset;
//                                sub_row_size = TEMPLATE_X_SIZE - offset;
//
//                                // do from offset to end.....0
//                                for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr;
//                                     column_row_ptr <
//                                     column_end_ptr; column_row_ptr += row_size, template_row_ptr += row_size) {
//                                    for (column_ptr = column_row_ptr, template_ptr = template_row_ptr;
//                                         column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++) {
//                                        cdiff += abs(*column_ptr - *template_ptr);
//                                    }
//
//                                    // fast breaks
//                                    if (cdiff > mindiff)
//                                        break;
//                                }
//
//                                // do from start to offset
//                                template_start_ptr = &vt.data[0];
//                                column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;
//                                row_size = TEMPLATE_X_SIZE;
//                                column_end_ptr = &data[0] + TEMPLATE_SIZE;
//                                sub_row_size = offset;
//                                for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr;
//                                     column_row_ptr <
//                                     column_end_ptr; column_row_ptr += row_size, template_row_ptr += row_size) {
//                                    for (column_ptr = column_row_ptr, template_ptr = template_row_ptr;
//                                         column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++) {
//                                        cdiff += abs(*column_ptr - *template_ptr);
//                                    }
//
//                                    // fast breaks
//                                    if (cdiff > mindiff)
//                                        break;
//                                }
//
//
//                                if (cdiff < mindiff) {
//                                    mindiff = cdiff;
//                                    min_template = vt.id;
//                                    min_offset = offset;
//                                }
//                            }
//
//                        }
//
//            vt_relative_rad = (double) min_offset / TEMPLATE_X_SIZE * 2.0 * M_PI;
//            if (vt_relative_rad > M_PI)
//                vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
//            vt_err = mindiff / (double) TEMPLATE_SIZE;
//            vt_match_id = min_template;
//
//            vt_error = vt_err;
//
//        } else {
//
//            BOOST_FOREACH(vt, templates) {
//
//                            if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
//                                continue;
//
//                            // for each vt try matching the view at different offsets
//                            // try to fast break based on error already great than previous errors
//                            // handles 2d images shifting only in the x direction
//                            // note I haven't tested on a 1d yet.
//                            for (offset = 0; offset < VT_SHIFT_MATCH * 2 + 1; offset += VT_STEP_MATCH) {
//                                cdiff = 0;
//                                template_start_ptr = &vt.data[0] + offset;
//                                column_start_ptr = &data[0] + VT_SHIFT_MATCH;
//                                row_size = TEMPLATE_X_SIZE;
//                                column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
//                                sub_row_size = TEMPLATE_X_SIZE - 2 * VT_SHIFT_MATCH;
//
//                                for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr;
//                                     column_row_ptr <
//                                     column_end_ptr; column_row_ptr += row_size, template_row_ptr += row_size) {
//                                    for (column_ptr = column_row_ptr, template_ptr = template_row_ptr;
//                                         column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++) {
//                                        cdiff += abs(*column_ptr - *template_ptr);
//                                    }
//
//                                    // fast breaks
//                                    if (cdiff > mindiff)
//                                        break;
//                                }
//
//                                if (cdiff < mindiff) {
//                                    mindiff = cdiff;
//                                    min_template = vt.id;
//                                    min_offset = 0;
//                                }
//                            }
//
//                        }
//
//            vt_relative_rad = 0;
//            vt_err = mindiff / (double) (TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
//            vt_match_id = min_template;
//
//            vt_error = vt_err;
//
//        }
//    }
//