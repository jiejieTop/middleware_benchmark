/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-10-28 16:31:37
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-11-03 10:05:08
 * @FilePath     : /middleware_benchmark/ecal/image/image_snd/src/image_snd.cpp
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author
 * information and source code according to the license.
 */

#include <cv_bridge/cv_bridge.h>
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <sensor_msgs/Image.h>
#include <tclap/CmdLine.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <queue>

#include "image.pb.h"
#include "ros_helper.h"

static size_t g_msg_num = 0;

// warmup runs not to measure
const int warmups(10);

static inline void evaluate(std::vector<float>& arr)
{

    if (arr.size() >= warmups) {
        arr.erase(arr.begin(), arr.begin() + warmups);
    }

    std::stringstream ss;
    size_t sum_msg = arr.size();
    ss << "------------------------------------------------------" << std::endl;
    ss << "Messages send                           : " << sum_msg << std::endl;
    auto sum_time = std::accumulate(arr.begin(), arr.end(), 0.0f);
    auto avg_time = sum_time / (sum_msg * 1.0);
    auto min_it = std::min_element(arr.begin(), arr.end());
    auto max_it = std::max_element(arr.begin(), arr.end());
    size_t min_pos = min_it - arr.begin();
    size_t max_pos = max_it - arr.begin();
    auto min_time = *min_it;
    auto max_time = *max_it;
    ss << "ROS to eCAL image average latency       : " << avg_time << " ms" << std::endl;
    ss << "ROS to eCAL image min latency           : " << min_time << " ms @ " << min_pos << std::endl;
    ss << "ROS to eCAL image max latency           : " << max_time << " ms @ " << max_pos << std::endl;
    ss << "------------------------------------------------------" << std::endl;
    std::cout << ss.str();
}

static void ros_to_ecal_image_send(std::queue<sensor_msgs::Image::ConstPtr>& ros_image_queue,
                                   eCAL::protobuf::CPublisher<pb::Image>* ecal_image_pub, int runs)
{
    // generate a class instance of Image

    pb::Image image;
    std::vector<float> ros_to_ecal_latency_array;

    while (1) {
        if (ros_image_queue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto ros_image = ros_image_queue.front();

        auto start_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        image.set_seq(ros_image->header.seq);
        image.set_timestamp(ros_image->header.stamp.sec * 1000000 + ros_image->header.stamp.nsec / 1000);  // us
        image.set_frame_id(ros_image->header.frame_id);
        image.set_height(ros_image->height);
        image.set_width(ros_image->width);
        image.set_encoding(ros_image->encoding);
        image.set_is_bigendian(ros_image->is_bigendian);
        image.set_step(ros_image->step);
        image.set_data(ros_image->data.data(), ros_image->data.size());
        auto end_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();

        // 计算时间戳
        auto diff = (end_time - start_time) / 1000.0;

        image.set_send_time(end_time);
        image.set_convert_time(diff);

        // send the image object
        ecal_image_pub->Send(image, end_time);

        ros_to_ecal_latency_array.push_back(diff);
        ros_image_queue.pop();

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
    eCAL::Initialize(argc, argv, "image_publisher");
    TCLAP::CmdLine cmd("image_publisher");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    TCLAP::ValueArg<int> runs("r", "runs", "Number of messages to send.", false, 1000, "int");
    TCLAP::ValueArg<int> image_num("n", "number", "Number of image.", false, 3, "int");
    TCLAP::ValueArg<int> mem_buffer("b", "mem_buffer", "Number of memory files per connection.", false, 1, "int");
    TCLAP::SwitchArg zero_copy("z", "zero_copy", "Switch zero copy mode on.");

    cmd.add(runs);
    cmd.add(image_num);
    cmd.add(mem_buffer);
    cmd.add(zero_copy);
    cmd.parse(argc, argv);

    std::vector<std::thread> thread_vec;
    std::vector<eCAL::protobuf::CPublisher<pb::Image>*> ecal_image_pub_vec;

    eCAL::CPublisher stop_signal_pub("image_stop_signal");

    std::queue<sensor_msgs::Image::ConstPtr> mid_image_queue;
    std::queue<sensor_msgs::Image::ConstPtr> left_image_queue;
    std::queue<sensor_msgs::Image::ConstPtr> right_image_queue;

    if (image_num.getValue() == 1) {
        // create a publisher (topic name "image")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::Image>("mid_image");
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_image_pub_vec.push_back(mid_pub);

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/mid_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                mid_image_queue.push(msg);
            });
        std::thread mid_thread(ros_to_ecal_image_send, std::ref(mid_image_queue), std::ref(mid_pub), runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
    } else if (image_num.getValue() == 2) {
        // create a publisher (topic name "image")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::Image>("mid_image");
        auto left_pub = new eCAL::protobuf::CPublisher<pb::Image>("left_image");

        // set number of publisher memory buffers
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        left_pub->ShmSetBufferCount(mem_buffer.getValue());
        // enable zero copy mode
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());
        left_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_image_pub_vec.push_back(mid_pub);
        ecal_image_pub_vec.push_back(left_pub);

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/mid_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                mid_image_queue.push(msg);
            });

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/left_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                left_image_queue.push(msg);
            });
        std::thread mid_thread(ros_to_ecal_image_send, std::ref(mid_image_queue), std::ref(mid_pub), runs.getValue());
        std::thread left_thread(ros_to_ecal_image_send, std::ref(left_image_queue), std::ref(left_pub),
                                runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
    } else if (image_num.getValue() == 3) {

        // create a publisher (topic name "image")
        auto mid_pub = new eCAL::protobuf::CPublisher<pb::Image>("mid_image");
        auto left_pub = new eCAL::protobuf::CPublisher<pb::Image>("left_image");
        auto right_pub = new eCAL::protobuf::CPublisher<pb::Image>("right_image");

        // set number of publisher memory buffers
        mid_pub->ShmSetBufferCount(mem_buffer.getValue());
        left_pub->ShmSetBufferCount(mem_buffer.getValue());
        right_pub->ShmSetBufferCount(mem_buffer.getValue());

        // enable zero copy mode
        mid_pub->ShmEnableZeroCopy(zero_copy.getValue());
        left_pub->ShmEnableZeroCopy(zero_copy.getValue());
        right_pub->ShmEnableZeroCopy(zero_copy.getValue());

        ecal_image_pub_vec.push_back(mid_pub);
        ecal_image_pub_vec.push_back(left_pub);
        ecal_image_pub_vec.push_back(right_pub);

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/mid_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                mid_image_queue.push(msg);
            });

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/left_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                left_image_queue.push(msg);
            });

        ros::ros_helper::instance().subscribe<sensor_msgs::Image>(
            "/right_cam", 10,
            (boost::function<void(const sensor_msgs::Image::ConstPtr&)>)[&](const sensor_msgs::Image::ConstPtr& msg) {
                right_image_queue.push(msg);
            });
        std::thread mid_thread(ros_to_ecal_image_send, std::ref(mid_image_queue), std::ref(mid_pub), runs.getValue());
        std::thread left_thread(ros_to_ecal_image_send, std::ref(left_image_queue), std::ref(left_pub),
                                runs.getValue());
        std::thread right_thread(ros_to_ecal_image_send, std::ref(right_image_queue), std::ref(right_pub),
                                 runs.getValue());
        thread_vec.push_back(std::move(mid_thread));
        thread_vec.push_back(std::move(left_thread));
        thread_vec.push_back(std::move(right_thread));
    } else {
        std::cout << "Invalid image number." << std::endl;
        exit(0);
    }

    while (ros::ok()) {
        // sleep 500 ms
        eCAL::Process::SleepMS(100);
        if (g_msg_num >= runs.getValue()) {
            stop_signal_pub.Send("stop");
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
