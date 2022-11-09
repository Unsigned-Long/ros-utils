//
// Created by csl on 11/9/22.
//
#include <ostream>
#include "ros/ros.h"
#include "artwork/logger/logger.h"
#include "artwork/csv/csv.h"
#include "filesystem"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Image.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

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

std::vector<CameraFrame> LoadCameraTimeStampFile(const std::string &filename) {
    std::vector<CameraFrame> imageFrames;
    std::ifstream imageTimeStampFile(filename, std::ios::in);
    std::string strLine;
    while (std::getline(imageTimeStampFile, strLine)) {
        auto items = split(strLine, ',');
        CameraFrame cameraFrame;
        cameraFrame.timeStamp = ToUNIXT(2235, std::stod(items.at(0)));
        cameraFrame.filename = items.at(1);
        imageFrames.push_back(cameraFrame);
    }
    return imageFrames;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_frame_inserter_node");
    LOG_PROCESS("loading ros params...")

    std::string srcBagPath, dstBagPath, imageDir, imageTimeStampFilename;
    std::string lidarSrcTopic, lidarDstTopic, lidarDstFrameId;
    std::string cameraDstTopic, cameraDstFrameId;

    ros::param::get("/camera_frame_inserter_node/src_bag_path", srcBagPath);
    ros::param::get("/camera_frame_inserter_node/dst_bag_path", dstBagPath);
    ros::param::get("/camera_frame_inserter_node/image_timestamp_filename", imageTimeStampFilename);
    ros::param::get("/camera_frame_inserter_node/image_dir", imageDir);

    ros::param::get("/camera_frame_inserter_node/lidar_src_topic", lidarSrcTopic);
    ros::param::get("/camera_frame_inserter_node/lidar_dst_topic", lidarDstTopic);
    ros::param::get("/camera_frame_inserter_node/lidar_dst_frame_id", lidarDstFrameId);

    ros::param::get("/camera_frame_inserter_node/camera_dst_topic", cameraDstTopic);
    ros::param::get("/camera_frame_inserter_node/camera_dst_frame_id", cameraDstFrameId);

    LOG_VAR(srcBagPath)
    LOG_VAR(dstBagPath)
    LOG_ENDL()
    LOG_VAR(imageTimeStampFilename)
    LOG_VAR(imageDir)
    LOG_ENDL()
    LOG_VAR(lidarSrcTopic, lidarDstTopic, lidarDstFrameId)
    LOG_VAR(cameraDstTopic, cameraDstFrameId)

    if (!std::filesystem::exists(srcBagPath)) {
        LOG_ERROR("the src srcBag path: '", srcBagPath, "' is not exists...")
        ros::shutdown();
    }
    if (!std::filesystem::exists(imageTimeStampFilename)) {
        LOG_ERROR("the image timestamp filename: '", imageTimeStampFilename, "' is not exists...")
        ros::shutdown();
    }
    if (!std::filesystem::exists(imageDir)) {
        LOG_ERROR("the image directory: '", imageDir, "' is not exists...")
        ros::shutdown();
    }
    LOG_PROCESS("loading camera timestamp file...")

    std::vector<CameraFrame> cameraFrames = LoadCameraTimeStampFile(imageTimeStampFilename);
    LOG_PLAINTEXT("camera frames count: ", cameraFrames.size())
    LOG_PLAINTEXT("camera first frame: ", cameraFrames.front())
    LOG_PLAINTEXT("camera last frame: ", cameraFrames.back())

    LOG_PROCESS("loading srcBag data...")
    std::shared_ptr<rosbag::Bag> srcBag(new rosbag::Bag()), dstBag(new rosbag::Bag);
    // open the srcBag
    srcBag->open(srcBagPath, rosbag::BagMode::Read);
    LOG_PLAINTEXT("srcBag size: ", srcBag->getSize() / 1000.0 / 1024, "(MB)")
    LOG_PLAINTEXT("srcBag mode: ", srcBag->getMode())
    // open the dst bag
    dstBag->open(dstBagPath, rosbag::BagMode::Write);
    {
        rosbag::View srcBagView;
        srcBagView.addQuery(*srcBag);
        LOG_PLAINTEXT("srcBag start time: ", srcBagView.getBeginTime(), ", srcBag end time: ", srcBagView.getEndTime())
        // get message data items and save them
        LOG_PROCESS("write lidar data to dstBag...")
        for (rosbag::MessageInstance const m: srcBagView) {
            if (m.getTopic() != lidarSrcTopic) {
                continue;
            }
            auto scanMsg = m.instantiate<sensor_msgs::PointCloud2>();
            scanMsg->header.frame_id = lidarDstFrameId;
            dstBag->write(lidarDstTopic, scanMsg->header.stamp, scanMsg);
        }
    }

    LOG_PROCESS("write camera data to dstBag...")
    for (const auto &frame: cameraFrames) {
        auto camMsg = frame.ToSensorMsg(imageDir, cameraDstFrameId);
        if (camMsg == nullptr) {
            continue;
        }
        dstBag->write(cameraDstTopic, camMsg->header.stamp, camMsg);
    }

    LOG_PLAINTEXT("dstBag size: ", dstBag->getSize() / 1000.0 / 1024, "(MB)")

    srcBag->close();
    dstBag->close();

    ros::shutdown();
    return 0;
}