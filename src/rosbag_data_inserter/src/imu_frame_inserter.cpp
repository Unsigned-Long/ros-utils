//
// Created by csl on 11/8/22.
//
#include <ostream>
#include "ros/ros.h"
#include "artwork/logger/logger.h"
#include "artwork/csv/csv.h"
#include "filesystem"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_bridge/cv_bridge.h"

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_frame_inserter_node");
    LOG_PROCESS("loading ros params...")

    std::string srcBagPath, dstBagPath, imuFilename;
    std::string lidarSrcTopic, lidarDstTopic, lidarDstFrameId;
    std::string imuDstTopic, imuDstFrameId;

    ros::param::get("/imu_frame_inserter_node/src_bag_path", srcBagPath);
    ros::param::get("/imu_frame_inserter_node/dst_bag_path", dstBagPath);
    ros::param::get("/imu_frame_inserter_node/imu_filename", imuFilename);

    ros::param::get("/imu_frame_inserter_node/lidar_src_topic", lidarSrcTopic);
    ros::param::get("/imu_frame_inserter_node/lidar_dst_topic", lidarDstTopic);
    ros::param::get("/imu_frame_inserter_node/lidar_dst_frame_id", lidarDstFrameId);
    ros::param::get("/imu_frame_inserter_node/imu_dst_topic", imuDstTopic);
    ros::param::get("/imu_frame_inserter_node/imu_dst_frame_id", imuDstFrameId);

    LOG_VAR(srcBagPath)
    LOG_VAR(dstBagPath)
    LOG_ENDL()
    LOG_VAR(imuFilename)
    LOG_ENDL()
    LOG_VAR(lidarSrcTopic, lidarDstTopic, lidarDstFrameId)
    LOG_VAR(imuDstTopic, imuDstFrameId)

    if (!std::filesystem::exists(srcBagPath)) {
        LOG_ERROR("the src srcBag path: '", srcBagPath, "' is not exists...")
        ros::shutdown();
    }
    if (!std::filesystem::exists(imuFilename)) {
        LOG_ERROR("the imu filename: '", imuFilename, "' is not exists...")
        ros::shutdown();
    }
    LOG_PROCESS("loading imu frames...")
    auto imuFrames = ns_csv::CSVReader::read<CSV_STRUCT(IMUFrame, timeStamp, gx, gy, gz, ax, ay, az) >(
            imuFilename, ' '
    );
    LOG_PLAINTEXT("imu frames count: ", imuFrames.size())
    // unit cast
    for (auto &frame: imuFrames) {
        static const double DEG_2_RAD = M_PI / 180.0;
        frame.gx *= DEG_2_RAD;
        frame.gy *= DEG_2_RAD;
        frame.gz *= DEG_2_RAD;
        frame.timeStamp = ToUNIXT(2235, frame.timeStamp);
    }
    LOG_PLAINTEXT("first imu frame: ", imuFrames.front())
    LOG_PLAINTEXT("last imu frame: ", imuFrames.back())

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

    LOG_PROCESS("write imu data to dstBag...")
    for (const auto &frame: imuFrames) {
        auto imuMsg = frame.ToSensorMsg(imuDstFrameId);
        dstBag->write(imuDstTopic, imuMsg.header.stamp, imuMsg);
    }

    LOG_PLAINTEXT("dstBag size: ", dstBag->getSize() / 1000.0 / 1024, "(MB)")

    srcBag->close();
    dstBag->close();

    ros::shutdown();
    return 0;
}