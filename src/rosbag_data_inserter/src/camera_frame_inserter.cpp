//
// Created by csl on 11/9/22.
//
#include <ostream>
#include "ros/ros.h"
#include "rosbag_data_inserter/helper.h"

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

    int gpsWeek;
    ros::param::get("/camera_frame_inserter_node/gps_week", gpsWeek);

    LOG_VAR(srcBagPath)
    LOG_VAR(dstBagPath)
    LOG_ENDL()
    LOG_VAR(imageTimeStampFilename)
    LOG_VAR(imageDir)
    LOG_ENDL()
    LOG_VAR(lidarSrcTopic, lidarDstTopic, lidarDstFrameId)
    LOG_VAR(cameraDstTopic, cameraDstFrameId)
    LOG_VAR(gpsWeek)

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

    std::vector<CameraFrame> cameraFrames = LoadCameraTimeStampFile(imageTimeStampFilename, gpsWeek);
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