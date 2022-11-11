/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-10-28 16:31:37
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-11-11 09:47:20
 * @FilePath     : /middleware_benchmark/ros/point_cloud/point_cloud_snd/src/point_cloud_snd.cpp
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author
 * information and source code according to the license.
 */

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tclap/CmdLine.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <queue>

#include "ros_helper.h"

static size_t g_msg_num = 0;

// warmup runs not to measure
const int warmups(20);

static inline void evaluate(std::vector<float>& arr)
{

    if (arr.size() >= warmups) {
        arr.erase(arr.begin(), arr.begin() + warmups);
    }

    std::stringstream ss;
    size_t sum_msg = arr.size();
    ss << "------------------------------------------------------" << std::endl;
    ss << "Messages send                                 : " << sum_msg << std::endl;
    auto sum_time = std::accumulate(arr.begin(), arr.end(), 0.0f);
    auto avg_time = sum_time / (sum_msg * 1.0);
    auto min_it = std::min_element(arr.begin(), arr.end());
    auto max_it = std::max_element(arr.begin(), arr.end());
    size_t min_pos = min_it - arr.begin();
    size_t max_pos = max_it - arr.begin();
    auto min_time = *min_it;
    auto max_time = *max_it;
    ss << "ROS to eCAL PointCloud2 average latency       : " << avg_time << " ms" << std::endl;
    ss << "ROS to eCAL PointCloud2 min latency           : " << min_time << " ms @ " << min_pos << std::endl;
    ss << "ROS to eCAL PointCloud2 max latency           : " << max_time << " ms @ " << max_pos << std::endl;
    ss << "------------------------------------------------------" << std::endl;
    std::cout << ss.str();
}

static void ros_to_ecal_point_cloud_send(std::queue<sensor_msgs::PointCloud2::ConstPtr>& ros_point_cloud_queue,
                                         std::string topic, int runs)
{
    // generate a class instance of PointCloud2

    sensor_msgs::PointCloud2 point_cloud;
    std::vector<float> ros_to_ecal_latency_array;
    uint32_t seq = 0;
    std::string frame_id = std::to_string(runs + warmups);
    while (1) {
        if (ros_point_cloud_queue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto ros_point_cloud = ros_point_cloud_queue.front();

        seq++;
        auto start_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        point_cloud.header.seq = seq;            // 用于记录发送序号
        point_cloud.header.frame_id = frame_id;  // 用于统计发送次数
        point_cloud.header.stamp = ros_point_cloud->header.stamp;
        point_cloud.height = ros_point_cloud->height;
        point_cloud.width = ros_point_cloud->width;
        point_cloud.fields = ros_point_cloud->fields;
        point_cloud.is_bigendian = ros_point_cloud->is_bigendian;
        point_cloud.point_step = ros_point_cloud->point_step;
        point_cloud.row_step = ros_point_cloud->row_step;
        point_cloud.data = ros_point_cloud->data;
        point_cloud.is_dense = ros_point_cloud->is_dense;

        auto end_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        // 计算时间戳

        // 记录发送时间,这个可能是有问题的，但在测试过程中，应该是可以的，如果测试出现问题，可以考虑使用frame id来替代
        point_cloud.header.stamp.sec = end_time & 0xffffffff;
        point_cloud.header.stamp.nsec = (end_time - start_time);  // 记录转换时间差

        // send the point_cloud object
        ros::ros_helper::instance().publish(topic, point_cloud);

        ros_to_ecal_latency_array.push_back((end_time - start_time) / 1000.0f);

        ros_point_cloud_queue.pop();

        if (seq > runs + warmups) {
            evaluate(ros_to_ecal_latency_array);
            g_msg_num += runs;
            return;
        }
    }
}

int main(int argc, char** argv)
{
    TCLAP::CmdLine cmd("point_cloud_publisher");

    TCLAP::ValueArg<int> runs("r", "runs", "Number of messages to send.", false, 1000, "int");
    TCLAP::ValueArg<int> point_cloud_num("n", "number", "Number of PointCloud2.", false, 3, "int");
    TCLAP::ValueArg<int> mem_buffer("b", "mem_buffer", "Number of memory files per connection.", false, 1, "int");
    TCLAP::SwitchArg zero_copy("z", "zero_copy", "Switch zero copy mode on.");

    cmd.add(runs);
    cmd.add(point_cloud_num);
    cmd.add(mem_buffer);
    cmd.add(zero_copy);
    cmd.parse(argc, argv);

    std::vector<std::thread> thread_vec;

    std::queue<sensor_msgs::PointCloud2::ConstPtr> mid_point_cloud_queue;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> left_point_cloud_queue;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> right_point_cloud_queue;

    if (point_cloud_num.getValue() == 1) {
        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/left_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { mid_point_cloud_queue.push(msg); });
        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), "/ros/mid_point_cloud",
                               runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
    } else if (point_cloud_num.getValue() == 2) {
        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/mid_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { mid_point_cloud_queue.push(msg); });

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/left_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { left_point_cloud_queue.push(msg); });

        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), "/ros/mid_point_cloud",
                               runs.getValue());
        std::thread left_thread(ros_to_ecal_point_cloud_send, std::ref(left_point_cloud_queue), "/ros/left_point_cloud",
                                runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
    } else if (point_cloud_num.getValue() == 3) {
        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/mid_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { mid_point_cloud_queue.push(msg); });

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/left_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { left_point_cloud_queue.push(msg); });

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/right_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { right_point_cloud_queue.push(msg); });

        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), "/ros/mid_point_cloud",
                               runs.getValue());
        std::thread left_thread(ros_to_ecal_point_cloud_send, std::ref(left_point_cloud_queue), "/ros/left_point_cloud",
                                runs.getValue());
        std::thread right_thread(ros_to_ecal_point_cloud_send, std::ref(right_point_cloud_queue),
                                 "/ros/right_point_cloud", runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
        thread_vec.push_back(std::move(right_thread));
    } else {
        std::cout << "Invalid point_cloud number." << std::endl;
        exit(0);
    }

    while (ros::ok()) {
        // sleep 500 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (g_msg_num >= runs.getValue()) {
            break;
        }
    }

    for (auto& th : thread_vec) {
        if (th.joinable()) {
            th.join();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std_msgs::String str_msg;
    str_msg.data = "stop";
    ros::ros_helper::instance().publish("/ros/point_cloud_stop_signal", str_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ros::ros_helper::instance().publish("/ros/point_cloud_stop_signal", str_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return (0);
}
