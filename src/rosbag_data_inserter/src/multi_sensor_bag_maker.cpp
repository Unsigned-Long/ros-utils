//
// Created by csl on 11/9/22.
//
#include <ostream>
#include "ros/ros.h"
#include "rosbag_data_inserter/helper.h"

auto LoadIMUFrames(const std::string &filename, int gps_week) {
    LOG_PROCESS("loading IMU frames...")
    auto imuFrames = ns_csv::CSVReader::read<CSV_STRUCT(IMUFrame, timeStamp, gx, gy, gz, ax, ay, az) >(
            filename, ' '
    );
    LOG_PLAINTEXT("imu frames count: ", imuFrames.size())
    // unit cast
    for (auto &frame: imuFrames) {
        static const double DEG_2_RAD = M_PI / 180.0;
        frame.gx *= DEG_2_RAD;
        frame.gy *= DEG_2_RAD;
        frame.gz *= DEG_2_RAD;
        frame.timeStamp = ToUNIXT(gps_week, frame.timeStamp);
    }
    LOG_PLAINTEXT("first imu frame: ", imuFrames.front())
    LOG_PLAINTEXT(" last imu frame: ", imuFrames.back())
    return imuFrames;
}

template<class MsgType>
void InsertOldBagToNewBag(const std::string &odlBagPath, const std::string &oldTopic,
                          const std::string &newTopic, const std::string &newFrameId,
                          std::shared_ptr<rosbag::Bag> &newBag) {
    LOG_VAR(odlBagPath, oldTopic, newTopic, newFrameId)

    std::shared_ptr<rosbag::Bag> oldBag(new rosbag::Bag());
    // open the bag
    oldBag->open(odlBagPath, rosbag::BagMode::Read);
    LOG_PLAINTEXT("oldBag size: ", oldBag->getSize() / 1000.0 / 1024, "(MB)")
    LOG_PLAINTEXT("oldBag mode: ", oldBag->getMode())

    rosbag::View bagBagView;
    bagBagView.addQuery(*oldBag, rosbag::TopicQuery({oldTopic}));
    LOG_PLAINTEXT("oldBag start time: ", bagBagView.getBeginTime(), ", oldBag end time: ", bagBagView.getEndTime())

    // get message data items and save them
    LOG_PROCESS("write lidar data to dstBag, size: ", bagBagView.size())
    for (rosbag::MessageInstance const m: bagBagView) {
        auto scanMsg = m.instantiate<MsgType>();
        if (scanMsg->header.stamp < ros::Time(1000000000.0)) { continue; }
        scanMsg->header.frame_id = newFrameId;
        // LOG_VAR(scanMsg->header.stamp)
        newBag->write(newTopic, scanMsg->header.stamp, scanMsg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multi_sensor_bag_maker_node");
    LOG_PROCESS("loading ros params...")

#define LOAD_STR_ROS_PARAM(paramName)                                                          \
    std::string paramName = []() {                                                             \
        std::string paramName;                                                                 \
        ros::param::get(std::string("/multi_sensor_bag_maker_node/") + #paramName, paramName); \
        LOG_PLAINTEXT(#paramName, ": ", paramName)                                             \
        return paramName;                                                                      \
    }();

#define LOAD_INT_ROS_PARAM(paramName)                                                          \
    int paramName = []() {                                                                     \
        int paramName;                                                                         \
        ros::param::get(std::string("/multi_sensor_bag_maker_node/") + #paramName, paramName); \
        LOG_PLAINTEXT(#paramName, ": ", paramName)                                             \
        return paramName;                                                                      \
    }();

#define CHECK_PATH_EXIST(path)                               \
    if (!std::filesystem::exists(path)) {                    \
        LOG_ERROR("the path: '", path, "' is not exists...") \
        ros::shutdown();                                     \
    }

    LOAD_STR_ROS_PARAM(src_velodyne_bag_path)
    CHECK_PATH_EXIST(src_velodyne_bag_path)
    LOAD_STR_ROS_PARAM(src_livox_bag_path)
    CHECK_PATH_EXIST(src_livox_bag_path)

    LOAD_STR_ROS_PARAM(dst_bag_path)

    LOAD_STR_ROS_PARAM(imu0_filename)
    CHECK_PATH_EXIST(imu0_filename)
    LOAD_STR_ROS_PARAM(imu1_filename)
    CHECK_PATH_EXIST(imu1_filename)

    LOAD_STR_ROS_PARAM(gs_image_timestamp_filename)
    CHECK_PATH_EXIST(gs_image_timestamp_filename)
    LOAD_STR_ROS_PARAM(gs_image_dir)
    CHECK_PATH_EXIST(gs_image_dir)

    LOAD_STR_ROS_PARAM(rs_image_timestamp_filename)
    CHECK_PATH_EXIST(rs_image_timestamp_filename)
    LOAD_STR_ROS_PARAM(rs_image_dir)
    CHECK_PATH_EXIST(rs_image_dir)

    LOAD_STR_ROS_PARAM(velodyne_src_topic)
    LOAD_STR_ROS_PARAM(velodyne_dst_topic)
    LOAD_STR_ROS_PARAM(velodyne_dst_frame_id)

    LOAD_STR_ROS_PARAM(livox_src_topic)
    LOAD_STR_ROS_PARAM(livox_dst_topic)
    LOAD_STR_ROS_PARAM(livox_dst_frame_id)

    LOAD_STR_ROS_PARAM(imu0_dst_topic)
    LOAD_STR_ROS_PARAM(imu0_dst_frame_id)

    LOAD_STR_ROS_PARAM(imu1_dst_topic)
    LOAD_STR_ROS_PARAM(imu1_dst_frame_id)

    LOAD_STR_ROS_PARAM(gs_camera_dst_topic)
    LOAD_STR_ROS_PARAM(gs_camera_dst_frame_id)

    LOAD_STR_ROS_PARAM(rs_camera_dst_topic)
    LOAD_STR_ROS_PARAM(rs_camera_dst_frame_id)

    LOAD_INT_ROS_PARAM(gps_week)


    auto imu0Frames = LoadIMUFrames(imu0_filename, gps_week);
    auto imu1Frames = LoadIMUFrames(imu1_filename, gps_week);

    LOG_PROCESS("loading camera timestamp file...")

    auto gsCameraFrames = LoadCameraTimeStampFile(gs_image_timestamp_filename, gps_week);
    LOG_PLAINTEXT("gs camera frames count: ", gsCameraFrames.size())
    LOG_PLAINTEXT("gs camera first frame: ", gsCameraFrames.front())
    LOG_PLAINTEXT("gs camera last frame: ", gsCameraFrames.back())

    auto rsCameraFrames = LoadCameraTimeStampFile(rs_image_timestamp_filename, gps_week);
    LOG_PLAINTEXT("rs camera frames count: ", rsCameraFrames.size())
    LOG_PLAINTEXT("rs camera first frame: ", rsCameraFrames.front())
    LOG_PLAINTEXT("rs camera last frame: ", rsCameraFrames.back())


    // open the dst bag
    std::shared_ptr<rosbag::Bag> dstBag(new rosbag::Bag);
    dstBag->open(dst_bag_path, rosbag::BagMode::Write);

    InsertOldBagToNewBag<sensor_msgs::PointCloud2>(
            src_velodyne_bag_path, velodyne_src_topic, velodyne_dst_topic,
            velodyne_dst_frame_id, dstBag
    );
    InsertOldBagToNewBag<sensor_msgs::PointCloud2>(
            src_livox_bag_path, livox_src_topic, livox_dst_topic,
            livox_dst_frame_id, dstBag
    );

    LOG_PROCESS("write imu 0 data to dstBag...")
    LOG_VAR(imu0Frames.size())
    for (const auto &frame: imu0Frames) {
        auto imuMsg = frame.ToSensorMsg(imu0_dst_frame_id);
        dstBag->write(imu0_dst_topic, imuMsg.header.stamp, imuMsg);
    }
    LOG_PROCESS("write imu 1 data to dstBag...")
    LOG_VAR(imu1Frames.size())
    for (const auto &frame: imu1Frames) {
        auto imuMsg = frame.ToSensorMsg(imu1_dst_frame_id);
        dstBag->write(imu1_dst_topic, imuMsg.header.stamp, imuMsg);
    }

    LOG_PROCESS("write gs camera data to dstBag...")
    LOG_VAR(gsCameraFrames.size())
    for (const auto &frame: gsCameraFrames) {
        auto gsCamMsg = frame.ToSensorMsg(gs_image_dir, gs_camera_dst_frame_id);
        if (gsCamMsg == nullptr) { continue; }
        dstBag->write(gs_camera_dst_topic, gsCamMsg->header.stamp, gsCamMsg);
    }
    LOG_PROCESS("write rs camera data to dstBag...")
    LOG_VAR(rsCameraFrames.size())
    for (const auto &frame: rsCameraFrames) {
        auto rsCamMsg = frame.ToSensorMsg(rs_image_dir, rs_camera_dst_frame_id);
        if (rsCamMsg == nullptr) {
            continue;
        }
        dstBag->write(rs_camera_dst_topic, rsCamMsg->header.stamp, rsCamMsg);
    }

    LOG_PLAINTEXT("dstBag size: ", dstBag->getSize() / 1000.0 / 1024, "(MB)")

    dstBag->close();

    ros::shutdown();
    return 0;
}