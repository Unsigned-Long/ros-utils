//
// Created by csl on 11/9/22.
//

#ifndef ROS_UTILS_HELPER_H
#define ROS_UTILS_HELPER_H

#include "artwork/logger/logger.h"
#include "artwork/csv/csv.h"
#include "filesystem"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

struct IMUFrame {
    double timeStamp;
    double gx;
    double gy;
    double gz;
    double ax;
    double ay;
    double az;

    [[nodiscard]] sensor_msgs::Imu ToSensorMsg(const std::string &frameId) const {
        sensor_msgs::Imu imuMsg;

        imuMsg.header.stamp = ros::Time(timeStamp);
        imuMsg.header.frame_id = frameId;

        imuMsg.linear_acceleration.x = ax;
        imuMsg.linear_acceleration.y = ay;
        imuMsg.linear_acceleration.z = az;

        imuMsg.angular_velocity.x = gx;
        imuMsg.angular_velocity.y = gy;
        imuMsg.angular_velocity.z = gz;

        return imuMsg;
    }

    friend std::ostream &operator<<(std::ostream &os, const IMUFrame &frame) {
        os << "timeStamp: " << frame.timeStamp << " gx: " << frame.gx << " gy: " << frame.gy << " gz: " << frame.gz
           << " ax: " << frame.ax << " ay: " << frame.ay << " az: " << frame.az;
        return os;
    }
};

struct CameraFrame {
    double timeStamp{};
    std::string filename;

    friend std::ostream &operator<<(std::ostream &os, const CameraFrame &frame) {
        os << "timeStamp: " << frame.timeStamp << " filename: " << frame.filename;
        return os;
    }

    [[nodiscard]] sensor_msgs::Image::Ptr ToSensorMsg(const std::string &prefixDir, const std::string &frameId) const {
        cv_bridge::CvImage cvImage;
        cvImage.image = cv::imread(prefixDir + "/" + filename, cv::ImreadModes::IMREAD_UNCHANGED);
        cvImage.header.stamp = ros::Time(timeStamp);
        cvImage.header.frame_id = frameId;
        cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        if (cvImage.image.empty()) {
            LOG_WARNING("empty image: ", *this)
            return nullptr;
        }
        // cv::imshow("win", cvImage.image);
        // cv::waitKey(1);
        return cvImage.toImageMsg();
    }
};

constexpr double UNIX_GPS_DIF = 315964800;
constexpr double GPS_TIME_LEAPSEC = 18;
constexpr double SECOND_PER_WEEK = 604800;

std::pair<int, double> ToGPST(double UNIXT) {
    std::pair<int, double> GPST;

    double gpsSec = UNIXT + GPS_TIME_LEAPSEC - UNIX_GPS_DIF;
    int gWeek = int(gpsSec / SECOND_PER_WEEK);
    double sow = gpsSec - gWeek * SECOND_PER_WEEK;
    GPST.first = gWeek;
    GPST.second = sow;
    return GPST;
}

long double ToUNIXT(int GWeek, double SOW) {
    long double UNIXT = GWeek * SECOND_PER_WEEK + SOW + UNIX_GPS_DIF - GPS_TIME_LEAPSEC;
    return UNIXT;
}

std::vector<std::string> split(const std::string &str, char splitor, bool ignoreEmpty = true) {
    std::vector<std::string> vec;
    auto iter = str.cbegin();
    while (true) {
        auto pos = std::find(iter, str.cend(), splitor);
        auto elem = std::string(iter, pos);
        if (!(elem.empty() && ignoreEmpty)) {
            vec.push_back(elem);
        }
        if (pos == str.cend()) {
            break;
        }
        iter = ++pos;
    }
    return vec;
}

std::vector<CameraFrame> LoadCameraTimeStampFile(const std::string &filename) {
    std::vector<CameraFrame> imageFrames;
    std::ifstream imageTimeStampFile(filename, std::ios::in);
    std::string strLine;
    while (std::getline(imageTimeStampFile, strLine)) {
        try {
            auto items = split(strLine, ',');
            CameraFrame cameraFrame;
            cameraFrame.timeStamp = ToUNIXT(2235, std::stod(items.at(0)));
            cameraFrame.filename = items.at(1);
            imageFrames.push_back(cameraFrame);
        } catch (const std::exception &e) {
            LOG_WARNING(e.what())
        }
    }
    return imageFrames;
}


#endif //ROS_UTILS_HELPER_H
