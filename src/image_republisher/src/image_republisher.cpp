//
// Created by csl on 11/8/22.
//

#include "ros/ros.h"
#include "thirdparty/logger/src/include/logger.h"
#include <filesystem>
#include <algorithm>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

/**
 * \brief a function to get all the filenames in the directory
 * \param directory the directory
 * \return the filenames in the directory
 */
std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem: std::filesystem::directory_iterator(directory))
        if (elem.status().type() != std::filesystem::file_type::directory)
            files.emplace_back(std::filesystem::canonical(elem.path()).c_str());
    std::sort(files.begin(), files.end());
    return files;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_republisher_node");
    // get ros params
    LOG_PROCESS("read ros params...")
    std::string imageDir, imageTopic;
    ros::param::get("/image_republisher_node/image_dir", imageDir);
    LOG_PLAINTEXT("config image directory: ", imageDir)
    if (!std::filesystem::exists(imageDir)) {
        LOG_ERROR("the config image directory '", imageDir, "' is not exists!")
        ros::shutdown();
    }
    ros::param::get("/image_republisher_node/image_topic", imageTopic);
    int rate;
    ros::param::get("/image_republisher_node/image_rate", rate);

    auto imageNames = filesInDir(imageDir);
    LOG_PLAINTEXT("image count: ", imageNames.size())
    LOG_PLAINTEXT("image span: from '", imageNames.front(), "' to '", imageNames.back(), "'")
    LOG_PLAINTEXT("image rate: ", rate)

    ros::NodeHandle nodeHandle;
    auto publisher = nodeHandle.advertise<sensor_msgs::Image>(imageTopic, 10);

    int idx = 0;
    ros::Rate r(rate);
    LOG_PROCESS("start to publish images to topic '", imageTopic, "'...")
    while (ros::ok() && idx != imageNames.size()) {
        LOG_PLAINTEXT("publish the image: ", imageNames.at(idx))
        // organize the cv image
        cv_bridge::CvImage cvImage;
        cvImage.image = cv::imread(imageNames.at(idx), cv::ImreadModes::IMREAD_UNCHANGED);
        cvImage.header.stamp = ros::Time::now();
        cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        // cv::imshow("win", image);
        // cv::waitKey(1);

        // publish
        sensor_msgs::Image sensorImage;
        cvImage.toImageMsg(sensorImage);
        publisher.publish(sensorImage);

        // sleep
        r.sleep();
        ++idx;
    }
    ros::shutdown();
    return 0;
}