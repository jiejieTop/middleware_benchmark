/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-03-29 17:52:23
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-10-28 18:03:44
 * @FilePath     : /ecal/samples/cpp/image/image_snd/src/ros_helper.cc
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author information and source code according to the
 * license.
 */

#include "ros_helper.h"
#include <signal.h>

namespace ros {

ros_helper::ros_helper()
{
    int argc = 0;
    char** argv = nullptr;
    std::string node_name = "rs_ros_helper";

    ros::Time::init();
    ros::init(argc, argv, node_name + "_" + std::to_string(ros::Time::now().toNSec()));

    nh_ = new ros::NodeHandle;

    shutdown_ = false;

    thread_ = std::move(std::thread([&]() {
        ros::Rate rate(50);
        printf("ros helper thread running ...\n");
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
        printf("ros helper thread exitting ...\n");
    }));
}

ros_helper::~ros_helper()
{
    shutdown_ = true;
    for (auto& it : publisher_) {
        auto publisher = it.second;
        printf("publisher shutdown [%s] ... \n", publisher.getTopic().c_str());
        publisher.shutdown();
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

}  // namespace ros
