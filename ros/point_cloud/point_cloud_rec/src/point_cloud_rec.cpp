/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-03-16 13:56:02
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-11-11 09:55:23
 * @FilePath     : /middleware_benchmark/ros/point_cloud/point_cloud_rec/src/point_cloud_rec.cpp
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author
 * information and source code according to the license.
 */

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <tclap/CmdLine.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <queue>
#include <thread>

#include "ros_helper.h"

// warmup runs not to measure
const int warmups(5);

// data structure for later evaluation
struct evaluate_data {
    evaluate_data()
    {
        seq_array.reserve(10000);
        comm_latency_array.reserve(10000);
        ecal_to_ros_latency_array.reserve(10000);
        ros_to_ecal_latency_array.reserve(10000);
    };
    uint32_t send_count;
    std::vector<uint32_t> seq_array;
    std::vector<float> comm_latency_array;
    std::vector<float> ecal_to_ros_latency_array;
    std::vector<float> ros_to_ecal_latency_array;
    size_t rec_size = 0;
    size_t msg_num = 0;
};

struct recv_data {
    int64_t rec_time;
    sensor_msgs::PointCloud2::ConstPtr point_cloud;
};

bool stop = false;
std::map<std::string, evaluate_data*> evaluate_map;

void on_point_cloud(std::queue<recv_data>& ros_point_cloud_queue, const char* topic_name_)
{

    evaluate_data* data = nullptr;
    data = new evaluate_data();
    evaluate_map[topic_name_] = data;
    sensor_msgs::PointCloud2 point_cloud;

    while (1) {
        if (ros_point_cloud_queue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (stop) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto ros_point_cloud_it = ros_point_cloud_queue.front();
        auto ros_point_cloud = ros_point_cloud_it.point_cloud;

        // get receive time stamp
        auto rec_time = ros_point_cloud_it.rec_time;

        // get send time stamp
        auto send_time = ros_point_cloud->header.stamp.sec;

        // get convert time diff
        auto convert_time = ros_point_cloud->header.stamp.nsec;

        if (data->send_count == 0) {
            data->send_count = std::stoi(ros_point_cloud->header.frame_id);
        }

        auto start_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();

        point_cloud.header = ros_point_cloud->header;
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

        ros_point_cloud_queue.pop();

        // update latency, size and msg number
        data->comm_latency_array.push_back(((rec_time & 0xffffffff) - send_time) / 1000.0f);
        data->rec_size += point_cloud.data.size();
        data->msg_num++;

        // 计算时间戳
        data->ecal_to_ros_latency_array.push_back((end_time - start_time) / 1000.0f);
        data->ros_to_ecal_latency_array.push_back(convert_time / 1000.0f);

        if (data->send_count < ros_point_cloud->header.seq) {
            data->send_count = ros_point_cloud->header.seq;
        }

        while ((data->seq_array.size() < ros_point_cloud->header.seq - 1)
               && (data->seq_array.size() < data->send_count)) {
            data->seq_array.push_back(0);
        }
        if ((data->seq_array.size() >= ros_point_cloud->header.seq - 1)
            && (data->seq_array.size() < data->send_count)) {
            data->seq_array[ros_point_cloud->header.seq - 1] = ros_point_cloud->header.seq;
        }
        data->seq_array.push_back(ros_point_cloud->header.seq);

        ros::ros_helper::instance().publish("/ros_rec" + std::string(topic_name_), point_cloud);
    }
}

bool mkdirs(std::string path)
{
    std::string p = "";
    std::string _path = "";
    std::istringstream ss;

    if (path.at(0) == '~') {
        std::string home = std::string(getenv("HOME"));
        ss.str(home + path.substr(1, path.size() - 1));
    } else {
        ss.str(path);
    }

    while (std::getline(ss, p, '/')) {
        _path += p + "/";
        if (access(_path.c_str(), F_OK) != 0) {
            if (::mkdir(_path.c_str(), 0777) == -1) {
                return false;
            }
        }
    }

    return true;
}

// evaluation
void evaluate(evaluate_data* data_, const std::string& file_name_)
{
    uint32_t loss = 0;
    uint32_t seq_start = 0;
    std::stringstream total_ss;
    std::stringstream ss;
    float total_latency = 0.0;

    // remove warmup runs
    if (data_->comm_latency_array.size() >= warmups) {
        data_->comm_latency_array.erase(data_->comm_latency_array.begin(), data_->comm_latency_array.begin() + warmups);
    }

    if (data_->ecal_to_ros_latency_array.size() >= warmups) {
        data_->ecal_to_ros_latency_array.erase(data_->ecal_to_ros_latency_array.begin(),
                                               data_->ecal_to_ros_latency_array.begin() + warmups);
    }

    if (data_->ros_to_ecal_latency_array.size() >= warmups) {
        data_->ros_to_ecal_latency_array.erase(data_->ros_to_ecal_latency_array.begin(),
                                               data_->ros_to_ecal_latency_array.begin() + warmups);
    }

    for (int i = 0; i < data_->send_count; i++) {
        if (data_->seq_array[i] != 0) {
            if (seq_start == 0) {
                seq_start = i;
                break;
            }
        }
    }

    std::cout << "seq start : " << seq_start << std::endl;

    std::cout << "loss data seq: ";
    for (int i = 0; i < data_->send_count; i++) {
        if (data_->seq_array[i] == 0) {
            std::cout << i + 1 << " ";
            loss++;
        }
    }

    std::cout << std::endl;

    auto loss_rate = (((loss - seq_start) * 1.0) / ((data_->send_count - seq_start) * 1.0)) * 100;

    std::cout << "loss message count : " << (((loss - seq_start) < 0 ? 0 : (loss - seq_start))) << std::endl;
    std::cout << "loss rate : " << loss_rate << " %" << std::endl;

    // evaluate all
    size_t sum_msg = data_->comm_latency_array.size();

    ss << "Messages received                            : " << sum_msg + warmups << std::endl;

    // evaluate ros to ecal
    sum_msg = data_->ros_to_ecal_latency_array.size();
    auto sum_time =
        std::accumulate(data_->ros_to_ecal_latency_array.begin(), data_->ros_to_ecal_latency_array.end(), 0.0f);
    auto avg_time = sum_time / sum_msg;
    auto min_it = std::min_element(data_->ros_to_ecal_latency_array.begin(), data_->ros_to_ecal_latency_array.end());
    auto max_it = std::max_element(data_->ros_to_ecal_latency_array.begin(), data_->ros_to_ecal_latency_array.end());
    auto min_pos = min_it - data_->ros_to_ecal_latency_array.begin();
    auto max_pos = max_it - data_->ros_to_ecal_latency_array.begin();
    auto min_time = *min_it;
    auto max_time = *max_it;
    total_latency += avg_time;

    ss << "ROS to eCAL PointCloud average latency       : " << avg_time << " ms" << std::endl;
    ss << "ROS to eCAL PointCloud min latency           : " << min_time << " ms @ " << min_pos << std::endl;
    ss << "ROS to eCAL PointCloud max latency           : " << max_time << " ms @ " << max_pos << std::endl;

    total_ss << data_->send_count << " " << sum_msg + warmups << " " << seq_start << " ";

    total_ss << avg_time << " " << min_time << " " << max_time << " ";

    sum_msg = data_->comm_latency_array.size();
    sum_time = std::accumulate(data_->comm_latency_array.begin(), data_->comm_latency_array.end(), 0.0f);
    avg_time = sum_time / (sum_msg * 1.0);
    min_it = std::min_element(data_->comm_latency_array.begin(), data_->comm_latency_array.end());
    max_it = std::max_element(data_->comm_latency_array.begin(), data_->comm_latency_array.end());
    min_pos = min_it - data_->comm_latency_array.begin();
    max_pos = max_it - data_->comm_latency_array.begin();
    min_time = *min_it;
    max_time = *max_it;
    total_latency += avg_time;

    auto size_per_msg = data_->rec_size / ((sum_msg * 1024.0) * 1.0);
    auto kb_per_sec = static_cast<int>(((data_->rec_size) / 1024.0) / (sum_time / 1000.0));
    auto mb_per_sec = static_cast<int>(((data_->rec_size) / 1024.0 / 1024.0) / (sum_time / 1000.0));
    auto msg_per_sec = static_cast<int>((sum_msg * 1.0) / (sum_time / 1000.0));

    ss << "Message average latency                      : " << avg_time << " ms" << std::endl;
    ss << "Message min latency                          : " << min_time << " ms @ " << min_pos << std::endl;
    ss << "Message max latency                          : " << max_time << " ms @ " << max_pos << std::endl;

    total_ss << avg_time << " " << min_time << " " << max_time << " ";

    // evaluate ecal to ros
    sum_msg = data_->ecal_to_ros_latency_array.size();
    sum_time = std::accumulate(data_->ecal_to_ros_latency_array.begin(), data_->ecal_to_ros_latency_array.end(), 0.0f);
    avg_time = sum_time / sum_msg;
    min_it = std::min_element(data_->ecal_to_ros_latency_array.begin(), data_->ecal_to_ros_latency_array.end());
    max_it = std::max_element(data_->ecal_to_ros_latency_array.begin(), data_->ecal_to_ros_latency_array.end());
    min_pos = min_it - data_->ecal_to_ros_latency_array.begin();
    max_pos = max_it - data_->ecal_to_ros_latency_array.begin();
    min_time = *min_it;
    max_time = *max_it;
    total_latency += avg_time;

    ss << "eCAL to ROS PointCloud average latency       : " << avg_time << " ms" << std::endl;
    ss << "eCAL to ROS PointCloud min latency           : " << min_time << " ms @ " << min_pos << std::endl;
    ss << "eCAL to ROS PointCloud max latency           : " << max_time << " ms @ " << max_pos << std::endl;

    ss << "Message size received                        : " << data_->rec_size / 1024 << " kB" << std::endl;
    ss << "Size of each message                         : " << size_per_msg << " kB" << std::endl;
    ss << "Throughput                                   : " << kb_per_sec << " kB/s" << std::endl;
    ss << "                                             : " << mb_per_sec << " MB/s" << std::endl;
    ss << "                                             : " << msg_per_sec << " Msg/s" << std::endl;

    total_ss << avg_time << " " << min_time << " " << max_time << " ";

    total_ss << total_latency << " " << mb_per_sec << " " << msg_per_sec << " " << loss_rate << std::endl << std::endl;

    ss << "--------------------------------------------" << std::endl;

    // log to console
    std::cout << ss.str();
    std::cout << total_ss.str();

    // write to file

    timeval tv;
    ::gettimeofday(&tv, nullptr);
    time_t t = (tv.tv_sec * 1000000 + tv.tv_usec) / 1000000;
    tm* tm = localtime(&t);
    char str[100];
    int slen = strftime(str, 100, "%Y-%m-%d-%H:%M:%S", tm);
    auto time = std::string(str, slen);

    mkdirs("./evaluate/ros");

    FILE* csv_file = fopen(("evaluate/" + file_name_ + "-" + time + ".csv").c_str(), "w");
    std::string info("序号, ros转ecal耗时(ms), 传输耗时(ms), ecal转ros耗时(ms), 总耗时(ms)\n");
    fwrite(info.c_str(), 1, info.size(), csv_file);

    for (int i = 0; i < data_->comm_latency_array.size(); i++) {
        std::stringstream data_ss;
        auto total =
            data_->ros_to_ecal_latency_array[i] + data_->comm_latency_array[i] + data_->ecal_to_ros_latency_array[i];
        data_ss << i << ", " << data_->ros_to_ecal_latency_array[i] << ", " << data_->comm_latency_array[i] << ", "
                << data_->ecal_to_ros_latency_array[i] << ", " << total << std::endl;
        fwrite(data_ss.str().c_str(), 1, data_ss.str().size(), csv_file);
    }

    fclose(csv_file);
}

int main(int argc, char** argv)
{
    TCLAP::CmdLine cmd("point_cloud_subscriber");

    TCLAP::ValueArg<int> point_cloud_num("n", "number", "Number of point_cloud.", false, 3, "int");

    cmd.add(point_cloud_num);
    cmd.parse(argc, argv);

    // subscriber
    std::queue<recv_data> mid_point_cloud_queue;
    std::queue<recv_data> left_point_cloud_queue;
    std::queue<recv_data> right_point_cloud_queue;

    ros::ros_helper::instance().subscribe<std_msgs::String>(
        "/ros/point_cloud_stop_signal", 10,
        (boost::function<void(const std_msgs::String::ConstPtr&)>)[&](const std_msgs::String::ConstPtr& msg) {
            if ("stop" == msg->data) {
                std::cout << "stop signal received" << std::endl;
                stop = true;
            }
        });

    ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
        "/ros/mid_point_cloud", 10,
        (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
            const sensor_msgs::PointCloud2::ConstPtr& msg) {
            recv_data data;
            data.point_cloud = msg;
            data.rec_time = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            mid_point_cloud_queue.push(data);
        });

    ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
        "/ros/left_point_cloud", 10,
        (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
            const sensor_msgs::PointCloud2::ConstPtr& msg) {
            recv_data data;
            data.point_cloud = msg;
            data.rec_time = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            left_point_cloud_queue.push(data);
        });

    ros::ros_helper::instance().subscribe<sensor_msgs::PointCloud2>(
        "/ros/right_point_cloud", 10,
        (boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>)[&](
            const sensor_msgs::PointCloud2::ConstPtr& msg) {
            recv_data data;
            data.point_cloud = msg;
            data.rec_time = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            right_point_cloud_queue.push(data);
        });

    std::thread mid_thread(on_point_cloud, std::ref(mid_point_cloud_queue), "/ros/mid_point_cloud");
    std::thread left_thread(on_point_cloud, std::ref(left_point_cloud_queue), "/ros/left_point_cloud");
    std::thread right_thread(on_point_cloud, std::ref(right_point_cloud_queue), "/ros/right_point_cloud");

    while (!stop) {
        // sleep 500 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (mid_thread.joinable()) {
        mid_thread.join();
    }

    if (left_thread.joinable()) {
        left_thread.join();
    }

    if (right_thread.joinable()) {
        right_thread.join();
    }

    for (auto eva : evaluate_map) {
        if (eva.second == nullptr) {
            continue;
        }
        if (eva.second->comm_latency_array.size() >= warmups) {
            std::cout << "-------------------- " << eva.first << " ----------------------" << std::endl;
            evaluate(eva.second, eva.first);
        }
        delete eva.second;
    }

    return (0);
}
