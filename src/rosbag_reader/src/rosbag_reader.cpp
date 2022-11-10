#include "memory"
#include "string"
#include "vector"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv2/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include <filesystem>
#include <iomanip>
#include <utility>
#include "thirdparty/logger/src/include/logger.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>

template<class DataType>
struct TopicInfo {
public:
    using data_type = DataType;

private:
    const std::string topicName;

public:
    explicit TopicInfo(std::string topicName) : topicName(std::move(topicName)) {}

    [[nodiscard]] const std::string &getTopicName() const {
        return this->topicName;
    }
};

class RosBagReader {
public:
    RosBagReader() = default;

    template<class DataType>
    std::vector<typename DataType::ConstPtr> static
    read(const std::string &bagPath, const TopicInfo<DataType> &topicInfo,
         const double bagStart = 0.0, const double bagDuration = -1.0) {

        LOG_INFO("find the bag '", bagPath, "', ready to read the bag")
        std::shared_ptr<rosbag::Bag> bag(new rosbag::Bag());
        // open the bag
        bag->open(bagPath, rosbag::BagMode::Read);

        // choose the time piece
        rosbag::View tempView;
        tempView.addQuery(*bag);

        ros::Time beginTime = tempView.getBeginTime() + ros::Duration(bagStart);
        ros::Time endTime = (bagDuration < 0) ? tempView.getEndTime() : tempView.getBeginTime() +
                                                                        ros::Duration(bagDuration);

        // query messages
        rosbag::View view;
        view.addQuery(*bag, [&topicInfo](const rosbag::ConnectionInfo *info) -> bool {
            return info->topic == topicInfo.getTopicName();
        }, beginTime, endTime);

        std::vector<typename DataType::ConstPtr> dataVec;

        LOG_INFO("read the bag from '", beginTime, "' to '", endTime, "' on topic: '", topicInfo.getTopicName(), "'");

        // get message data items and save them
        for (rosbag::MessageInstance const m: view) {
            dataVec.push_back(m.instantiate<DataType>());
        }

        LOG_INFO("read data finished, total: '", dataVec.size(), "' item(s)");
        return dataVec;
    }

    static bool write(const std::vector<sensor_msgs::ImageConstPtr> &vec, const std::string &outputPath) {

        // prepare the directory
        std::string absolutePath = std::filesystem::canonical(outputPath).c_str();
        std::filesystem::create_directory(absolutePath + "/images");
        auto digNum = std::to_string(vec.size()).size();

        // prepare the imgPathWithTimeFile
        std::fstream file(absolutePath + "/imgInfo.txt", std::ios::out);
        file << "format ascii 1.0\n"
             << "element imgInfo " + std::to_string(vec.size()) + '\n'
             << "property double timeStamp\n"
             << "property string imagePath\n"
             << "end_header\n";

        for (int i = 0; i != vec.size(); ++i) {
            const auto &imgMsg = vec[i];
            cv::Mat img;
            cv_bridge::toCvCopy(imgMsg)->image.copyTo(img);

            // generate the image name and image time stamp string
            std::string curImgName, curImgTimeStamp;
            std::stringstream stream1, stream2;
            stream1 << std::fixed << std::setprecision(10);
            stream1 << imgMsg->header.stamp.toSec();
            stream1 >> curImgTimeStamp;

            stream2 << std::setfill('0') << std::setw(static_cast<int>(digNum));
            stream2 << i;
            stream2 >> curImgName;
            curImgName = absolutePath + "/images/" + curImgName + ".png";

            // save date
            cv::imwrite(curImgName, img);
            file << curImgTimeStamp << ' ' << curImgName << std::endl;
        }

        file.close();

        return true;
    }

    static bool write(const std::vector<sensor_msgs::PointCloud2ConstPtr> &vec, const std::string &outputPath) {

        // prepare the directory
        std::string absolutePath = std::filesystem::canonical(outputPath).c_str();
        std::filesystem::create_directory(absolutePath + "/pointclouds");
        auto digNum = std::to_string(vec.size()).size();

        // prepare the imgPathWithTimeFile
        std::fstream file(absolutePath + "/imgInfo.txt", std::ios::out);
        file << "format ascii 1.0\n"
             << "element scanInfo " + std::to_string(vec.size()) + '\n'
             << "property double timeStamp\n"
             << "property string scaPath\n"
             << "property int width\n"
             << "property int height\n"
             << "end_header\n";

        for (int i = 0; i != vec.size(); ++i) {
            const auto &pcMsg = vec[i];

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*pcMsg, *cloud);

            // generate the image name and image time stamp string
            std::string curScanName, curTimeStamp;
            std::stringstream stream1, stream2;
            stream1 << std::fixed << std::setprecision(10);
            stream1 << pcMsg->header.stamp.toSec();
            stream1 >> curTimeStamp;

            stream2 << std::setfill('0') << std::setw(static_cast<int>(digNum));
            stream2 << i;
            stream2 >> curScanName;
            curScanName = absolutePath + "/pointclouds/" + curScanName + ".pcd";

            // save date
            pcl::io::savePCDFile(curScanName, *cloud, true);
            file << curTimeStamp << ' ' << curScanName << ' ' << pcMsg->width << ' ' << pcMsg->height << std::endl;
        }

        file.close();

        return true;
    }

    static bool write(const std::vector<sensor_msgs::ImuConstPtr> &vec, const std::string &outputPath) {
        // prepare the directory
        std::string absolutePath = std::filesystem::canonical(outputPath).c_str();

        std::fstream file(absolutePath + "/imuInfo.txt", std::ios::out);
        file << "format ascii 1.0\n"
             << "element imuInfo " + std::to_string(vec.size()) + '\n'
             << "property double timeStamp\n"
             << "property double gx\n"
             << "property double gy\n"
             << "property double gz\n"
             << "property double ax\n"
             << "property double ay\n"
             << "property double az\n"
             << "end_header\n";

        file << std::fixed << std::setprecision(10);
        for (const auto &imu: vec) {
            const auto &gyro = imu->angular_velocity;
            const auto &acce = imu->linear_acceleration;

            file << imu->header.stamp.toSec() << ' ';
            file << gyro.x << ' ' << gyro.y << ' ' << gyro.z << ' ';
            file << acce.x << ' ' << acce.y << ' ' << acce.z << '\n';
        }
        file.close();
        return true;
    }

    static bool write(const std::vector<nav_msgs::OdometryConstPtr> &vec, const std::string &outputPath) {
        // prepare the directory
        std::string absolutePath = std::filesystem::canonical(outputPath).c_str();

        std::fstream file(absolutePath + "/odometryInfo.txt", std::ios::out);
        file << "format ascii 1.0\n"
             << "element odometryInfo " + std::to_string(vec.size()) + '\n'
             << "property double timeStamp\n"
             << "property double qx\n"
             << "property double qy\n"
             << "property double qz\n"
             << "property double qw\n"
             << "property double x\n"
             << "property double y\n"
             << "property double z\n"
             << "end_header\n";

        file << std::fixed << std::setprecision(10);
        for (const auto &item: vec) {
            const auto &quaternion = item->pose.pose.orientation;
            const auto &position = item->pose.pose.position;
            file << item->header.stamp << ' ' << quaternion.x << ' ' << quaternion.y << ' '
                 << quaternion.z << ' ' << quaternion.w << ' ';
            file << position.x << ' ' << position.y << ' ' << position.z << std::endl;
        }
        file.close();
        return true;
    }
};

int main(int argc, char **argv) {

    const std::string bagPath = "/home/csl/dataset/LIC-Calib/simu_bag.bag";
    const std::string outputDir = "/home/csl/dataset/LIC-Calib/scan_sim";
    const std::string topicName = "/scan_sim";
    using msg_type = sensor_msgs::PointCloud2;

    ros::init(argc, argv, "node_rosbag_reader");

    if (!exists(std::filesystem::path(outputDir))) {
        create_directory(std::filesystem::path(outputDir));
    }

    try {
        auto vec = RosBagReader::read(bagPath, TopicInfo<msg_type>(topicName));
        RosBagReader::write(vec, outputDir);
    } catch (std::exception &e) {
        LOG_ERROR(e.what());
    }

    ros::shutdown();
    return 0;
}
