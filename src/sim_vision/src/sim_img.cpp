//
// Created by csl on 11/19/22.
//
#include "ros/ros.h"
#include "thirdparty/logger/src/include/logger.h"
#include "filesystem"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "nav_msgs/Odometry.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_img_node");
    LOG_PROCESS("loading ros params...")

    std::string bagPath, imgTopic, odomTopic;

    ros::param::get("/sim_img_node/bag_path", bagPath);
    ros::param::get("/sim_img_node/img_topic", imgTopic);
    ros::param::get("/sim_vision_node/odom_topic", odomTopic);

    LOG_VAR(bagPath)
    LOG_VAR(imgTopic)
    LOG_VAR(odomTopic)

    if (!std::filesystem::exists(bagPath)) {
        LOG_ERROR("the readBag path '", bagPath, "' is not exists...")
        ros::shutdown();
        return 0;
    }

    std::vector<std::pair<ros::Time, std::string>> timestampVec;

    std::shared_ptr<rosbag::Bag> readBag(new rosbag::Bag()), writeBag(new rosbag::Bag());
    readBag->open(bagPath, rosbag::BagMode::Read);
    rosbag::View bagView;
    bagView.addQuery(*readBag);
    LOG_PLAINTEXT("readBag start time: ", bagView.getBeginTime(), ", readBag end time: ", bagView.getEndTime())
    // get message data items and save them
    for (rosbag::MessageInstance const m: bagView) {
        if (m.getTopic() != odomTopic) {
            continue;
        }
        auto odom = m.instantiate<nav_msgs::Odometry>();
        timestampVec.push_back({odom->header.stamp, odom->header.frame_id});

    }
    readBag->close();
    writeBag->open(bagPath, rosbag::BagMode::Append);

    for (const auto &[timestamp, frameId]: timestampVec) {
        cv_bridge::CvImage cvImage;
        cvImage.image = cv::Mat(10, 10, CV_8UC1, cv::Scalar(0));
        cvImage.header.stamp = timestamp;
        cvImage.header.frame_id = frameId;
        cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        auto msg = cvImage.toImageMsg();
        writeBag->write(imgTopic, msg->header.stamp, msg);
    }
    writeBag->close();
    ros::shutdown();
    return 0;
}