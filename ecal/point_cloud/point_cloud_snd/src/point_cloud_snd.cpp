/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-10-28 16:31:37
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-11-03 11:28:31
 * @FilePath     : /middleware_benchmark/ecal/point_cloud/point_cloud_snd/src/point_cloud_snd.cpp
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author
 * information and source code according to the license.
 */

#include <cv_bridge/cv_bridge.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <tclap/CmdLine.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <queue>

#include "point_cloud.pb.h"
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
                                         eCAL::protobuf::CPublisher<pb::PointCloud2>* ecal_point_cloud_pub, int runs)
{
    // generate a class instance of PointCloud2

    pb::PointCloud2 point_cloud;
    std::vector<float> ros_to_ecal_latency_array;

    while (1) {
        if (ros_point_cloud_queue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto ros_point_cloud = ros_point_cloud_queue.front();

        auto start_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        point_cloud.set_seq(ros_point_cloud->header.seq);
        point_cloud.set_timestamp(ros_point_cloud->header.stamp.sec * 1000000
                                  + ros_point_cloud->header.stamp.nsec / 1000);  // us
        point_cloud.set_frame_id(ros_point_cloud->header.frame_id);
        point_cloud.set_height(ros_point_cloud->height);
        point_cloud.set_width(ros_point_cloud->width);
        point_cloud.set_is_bigendian(ros_point_cloud->is_bigendian);
        point_cloud.set_point_step(ros_point_cloud->point_step);
        point_cloud.set_row_step(ros_point_cloud->row_step);
        point_cloud.set_is_dense(ros_point_cloud->is_dense);

        point_cloud.set_data(ros_point_cloud->data.data(), ros_point_cloud->data.size());

        if (ros_point_cloud->fields.size() > 0) {
            for (auto& field : ros_point_cloud->fields) {
                auto point_field = point_cloud.add_fields();
                point_field->set_name(field.name);
                point_field->set_offset(field.offset);
                point_field->set_datatype(field.datatype);
                point_field->set_count(field.count);
            }
        }
        auto end_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        // 计算时间戳
        auto diff = (end_time - start_time) / 1000.0;

        point_cloud.set_send_time(end_time);
        point_cloud.set_convert_time(diff);

        // send the point_cloud object
        ecal_point_cloud_pub->Send(point_cloud, end_time);

        ros_to_ecal_latency_array.push_back(diff);

        ros_point_cloud_queue.pop();

        point_cloud.clear_fields();

        if (ros_to_ecal_latency_array.size() >= runs + warmups) {
            evaluate(ros_to_ecal_latency_array);
            g_msg_num += runs;
            return;
        }
    }
}

int main(int argc, char** argv)
{
    // initialize eCAL API
    eCAL::Initialize(argc, argv, "point_cloud_publisher");
    TCLAP::CmdLine cmd("point_cloud_publisher");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

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
    std::vector<eCAL::protobuf::CPublisher<pb::PointCloud2>*> ecal_point_cloud_pub_vec;

    eCAL::CPublisher stop_signal_pub("point_cloud_stop_signal");

    std::queue<sensor_msgs::PointCloud2::ConstPtr> mid_point_cloud_queue;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> left_point_cloud_queue;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> right_point_cloud_queue;

    if (point_cloud_num.getValue() == 1) {
        // create a publisher (topic name "point_cloud")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("mid_point_cloud");
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_point_cloud_pub_vec.push_back(mid_pub);

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/left_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { mid_point_cloud_queue.push(msg); });
        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), std::ref(mid_pub),
                               runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
    } else if (point_cloud_num.getValue() == 2) {
        // create a publisher (topic name "point_cloud")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("mid_point_cloud");
        auto left_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("left_point_cloud");

        // set number of publisher memory buffers
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        left_pub->ShmSetBufferCount(mem_buffer.getValue());
        // enable zero copy mode
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());
        left_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_point_cloud_pub_vec.push_back(mid_pub);
        ecal_point_cloud_pub_vec.push_back(left_pub);

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/mid_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { mid_point_cloud_queue.push(msg); });

        ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
            "/left_point_cloud", 10,
            (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
                const sensor_msgs::PointCloud2::ConstPtr& msg) { left_point_cloud_queue.push(msg); });
        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), std::ref(mid_pub),
                               runs.getValue());
        std::thread left_thread(ros_to_ecal_point_cloud_send, std::ref(left_point_cloud_queue), std::ref(left_pub),
                                runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
    } else if (point_cloud_num.getValue() == 3) {

        // create a publisher (topic name "point_cloud")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("mid_point_cloud");
        auto left_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("left_point_cloud");
        auto right_pub = new eCAL::protobuf::CPublisher<pb::PointCloud2>("right_point_cloud");

        // set number of publisher memory buffers
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        left_pub->ShmSetBufferCount(mem_buffer.getValue());
        right_pub->ShmSetBufferCount(mem_buffer.getValue());

        // enable zero copy mode
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());
        left_pub->ShmEnableZeroCopy(zero_copy.getValue());
        right_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_point_cloud_pub_vec.push_back(mid_pub);
        ecal_point_cloud_pub_vec.push_back(left_pub);
        ecal_point_cloud_pub_vec.push_back(right_pub);

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
        std::thread mid_thread(ros_to_ecal_point_cloud_send, std::ref(mid_point_cloud_queue), std::ref(mid_pub),
                               runs.getValue());
        std::thread left_thread(ros_to_ecal_point_cloud_send, std::ref(left_point_cloud_queue), std::ref(left_pub),
                                runs.getValue());
        std::thread right_thread(ros_to_ecal_point_cloud_send, std::ref(right_point_cloud_queue), std::ref(right_pub),
                                 runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
        thread_vec.push_back(std::move(right_thread));
    } else {
        std::cout << "Invalid point_cloud number." << std::endl;
        exit(0);
    }

    while (ros::ok()) {
        // sleep 500 ms
        eCAL::Process::SleepMS(500);
        if (g_msg_num >= runs.getValue()) {
            break;
        }
    }

    for (auto& th : thread_vec) {
        if (th.joinable()) {
            th.join();
        }
    }

    stop_signal_pub.Send("stop");
    eCAL::Process::SleepMS(1000);
    stop_signal_pub.Send("stop");
    eCAL::Process::SleepMS(1000);

    // finalize eCAL API
    eCAL::Finalize();

    return (0);
}
