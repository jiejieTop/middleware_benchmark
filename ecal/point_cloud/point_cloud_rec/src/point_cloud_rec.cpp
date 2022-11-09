/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-03-16 13:56:02
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-11-09 16:56:12
 * @FilePath     : /middleware_benchmark/ecal/point_cloud/point_cloud_rec/src/point_cloud_rec.cpp
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author
 * information and source code according to the license.
 */

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
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

#include "point_cloud.pb.h"
#include "ros_helper.h"

// warmup runs not to measure
const int warmups(5);

// data structure for later evaluation
struct evaluate_data {
    evaluate_data(){};
    uint32_t send_count;
    std::vector<uint32_t> seq_array;
    std::vector<float> comm_latency_array;
    std::vector<float> ecal_to_ros_latency_array;
    std::vector<float> ros_to_ecal_latency_array;
    size_t rec_size = 0;
    size_t msg_num = 0;
};

std::map<std::string, evaluate_data*> evaluate_map;

void OnPointCloud2(const char* topic_name_, const pb::PointCloud2& point_cloud_, const long long time_,
                   const long long clock_)
{
    // get receive time stamp
    auto rec_time =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    evaluate_data* data = nullptr;
    auto it = evaluate_map.find(topic_name_);
    if (it == evaluate_map.end()) {
        data = new evaluate_data();
        data->comm_latency_array.reserve(point_cloud_.send_count());
        data->ecal_to_ros_latency_array.reserve(point_cloud_.send_count());
        data->ros_to_ecal_latency_array.reserve(point_cloud_.send_count());
        data->seq_array.reserve(point_cloud_.send_count());
        evaluate_map[topic_name_] = data;
    } else {
        data = it->second;
    }

    auto start_time =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    static thread_local sensor_msgs::PointCloud2 ros_point_cloud = [&]() {
        sensor_msgs::PointCloud2 point_cloud;
        ros_point_cloud.fields.resize(point_cloud_.fields_size());
        point_cloud.data.resize(point_cloud_.data().size());
        return point_cloud;
    }();

    ros_point_cloud.header.seq = point_cloud_.seq();
    ros_point_cloud.header.stamp.sec = point_cloud_.timestamp() / 1000000;
    ros_point_cloud.header.stamp.nsec = point_cloud_.timestamp() * 1000;
    ros_point_cloud.header.frame_id = point_cloud_.frame_id();
    ros_point_cloud.height = point_cloud_.height();
    ros_point_cloud.width = point_cloud_.width();

    if (ros_point_cloud.fields.size() != point_cloud_.fields_size()) {
        ros_point_cloud.fields.resize(point_cloud_.fields_size());
    }

    for (int i = 0; i < point_cloud_.fields_size(); i++) {
        ros_point_cloud.fields[i].name = point_cloud_.fields(i).name();
        ros_point_cloud.fields[i].offset = point_cloud_.fields(i).offset();
        ros_point_cloud.fields[i].datatype = point_cloud_.fields(i).datatype();
        ros_point_cloud.fields[i].count = point_cloud_.fields(i).count();
    }

    ros_point_cloud.is_bigendian = point_cloud_.is_bigendian();
    ros_point_cloud.point_step = point_cloud_.point_step();
    ros_point_cloud.row_step = point_cloud_.row_step();
    ros_point_cloud.is_dense = point_cloud_.is_dense();

    if (ros_point_cloud.data.size() != point_cloud_.data().size()) {
        ros_point_cloud.data.resize(point_cloud_.data().size());
    }

    memcpy(ros_point_cloud.data.data(), point_cloud_.data().data(), point_cloud_.data().size());

    auto end_time =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();

    // update latency, size and msg number
    data->comm_latency_array.push_back((rec_time - time_) / 1000.0);
    data->rec_size += point_cloud_.ByteSize();
    data->msg_num++;

    // 计算时间戳
    data->ecal_to_ros_latency_array.push_back((end_time - start_time) / 1000.0);
    data->ros_to_ecal_latency_array.push_back(point_cloud_.convert_time());

    while (data->seq_array.size() < point_cloud_.seq() - 1) {
        data->seq_array.push_back(0);
    }
    if (data->seq_array.size() >= point_cloud_.seq() - 1) {
        data->seq_array[point_cloud_.seq() - 1] = point_cloud_.seq();
    }
    data->seq_array.push_back(point_cloud_.seq());
    if (data->send_count < point_cloud_.send_count()) {
        data->send_count = point_cloud_.send_count();
    }

    ros::ros_helper::instance().publish("/ecal/" + std::string(topic_name_), ros_point_cloud);
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

    std::cout << "loss message count : " << (loss - seq_start) << std::endl;
    std::cout << "loss rate : " << loss_rate << " %" << std::endl;

    // evaluate all
    size_t sum_msg = data_->comm_latency_array.size();

    ss << "Messages received                            : " << sum_msg << std::endl;

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

    mkdirs("./evaluate/");

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
    bool stop = false;

    // initialize eCAL API
    eCAL::Initialize(argc, argv, "point_cloud_subscriber");

    TCLAP::CmdLine cmd("point_cloud_subscriber");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "I feel good !");

    TCLAP::ValueArg<int> point_cloud_num("n", "number", "Number of point_cloud.", false, 3, "int");

    cmd.add(point_cloud_num);
    cmd.parse(argc, argv);

    std::vector<eCAL::protobuf::CSubscriber<pb::PointCloud2>> subscribers;

    // subscriber
    eCAL::CSubscriber sub("point_cloud_stop_signal");
    sub.AddReceiveCallback([&](const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_) {
        if ("stop" == std::string((char*)data_->buf, data_->size)) {
            std::cout << "stop signal received" << std::endl;
            stop = true;
        }
    });

    // add receive callback function (_1 = topic_name, _2 = msg, _3 = time, _4 =
    // clock, _5 = id)
    auto callback = std::bind(OnPointCloud2, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                              std::placeholders::_4);
    if (point_cloud_num.getValue() == 1) {
        eCAL::protobuf::CSubscriber<pb::PointCloud2> mid_sub("mid_point_cloud");
        mid_sub.AddReceiveCallback(callback);
        subscribers.push_back(std::move(mid_sub));
    } else if (point_cloud_num.getValue() == 2) {
        eCAL::protobuf::CSubscriber<pb::PointCloud2> mid_sub("mid_point_cloud");
        eCAL::protobuf::CSubscriber<pb::PointCloud2> left_sub("left_point_cloud");
        mid_sub.AddReceiveCallback(callback);
        left_sub.AddReceiveCallback(callback);
        subscribers.push_back(std::move(mid_sub));
        subscribers.push_back(std::move(left_sub));
    } else if (point_cloud_num.getValue() == 3) {
        eCAL::protobuf::CSubscriber<pb::PointCloud2> mid_sub("mid_point_cloud");
        eCAL::protobuf::CSubscriber<pb::PointCloud2> left_sub("left_point_cloud");
        eCAL::protobuf::CSubscriber<pb::PointCloud2> right_sub("right_point_cloud");
        mid_sub.AddReceiveCallback(callback);
        left_sub.AddReceiveCallback(callback);
        right_sub.AddReceiveCallback(callback);
        subscribers.push_back(std::move(mid_sub));
        subscribers.push_back(std::move(left_sub));
        subscribers.push_back(std::move(right_sub));
    }

    while (!stop) {
        // sleep 500 ms
        eCAL::Process::SleepMS(500);
    }

    eCAL::Process::SleepMS(1000);

    for (auto eva : evaluate_map) {
        std::cout << "-------------------- " << eva.first << " ----------------------" << std::endl;
        evaluate(eva.second, eva.first);
    }

    // finalize eCAL API
    eCAL::Finalize();

    return (0);
}
