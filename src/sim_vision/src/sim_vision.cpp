//
// Created by csl on 11/19/22.
//
#include <utility>

#include "ros/ros.h"
#include "thirdparty/logger/src/include/logger.h"
#include "filesystem"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Geometry"
#include "sophus/se3.hpp"
#include "pcl/io/pcd_io.h"
#include "slam-scene-viewer/scene_viewer.h"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/cameras/Camera_Pinhole_Brown.hpp"

struct Landmark {
    Eigen::Vector3d x;
    // view id, feature
    std::map<std::uint32_t, Eigen::Vector2d> features;

    Landmark(Eigen::Vector3d x, std::map<std::uint32_t, Eigen::Vector2d> features)
            : x(std::move(x)), features(std::move(features)) {}
};

struct Frame {
    double timestamp;
    Sophus::SE3d CurToW;

    Frame(double timestamp, const Sophus::SE3d &pose) : timestamp(timestamp), CurToW(pose) {}

};

std::vector<Frame>
GenerateBiasFrames(const Sophus::SE3d &CurToRef, const std::string &bagPath, const std::string &odomTopic) {
    std::vector<Frame> frames;

    std::shared_ptr<rosbag::Bag> bag(new rosbag::Bag());
    bag->open(bagPath, rosbag::BagMode::Read);
    rosbag::View bagView;
    bagView.addQuery(*bag);
    LOG_PLAINTEXT("bag start time: ", bagView.getBeginTime(), ", bag end time: ", bagView.getEndTime())
    // get message data items and save them
    for (rosbag::MessageInstance const m: bagView) {
        if (m.getTopic() != odomTopic) {
            continue;
        }
        auto odom = m.instantiate<nav_msgs::Odometry>();

        double timestamp = odom->header.stamp.toSec();
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;
        double qw = odom->pose.pose.orientation.w;
        double qx = odom->pose.pose.orientation.x;
        double qy = odom->pose.pose.orientation.y;
        double qz = odom->pose.pose.orientation.z;

        Eigen::Vector3d t(x, y, z);
        Eigen::Quaterniond q(qw, qx, qy, qz);

        Sophus::SE3d RefToW(q, t);
        Sophus::SE3d CurToW = RefToW * CurToRef;
        frames.emplace_back(timestamp, CurToW);
    }

    return frames;
}

void visual(const std::map<std::uint32_t, Frame> &views, const std::map<std::uint32_t, Landmark> &landmarks) {
    ns_viewer::SceneViewer viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (const auto &[trackId, lm]: landmarks) {
        pcl::PointXYZRGBA p;
        p.x = lm.x(0);
        p.y = lm.x(1);
        p.z = lm.x(2);
        p.r = p.g = p.b = 0;
        p.a = 255;
        cloud->points.push_back(p);
    }
    viewer.AddFeatures(cloud);
    for (const auto &[viewId, view]: views) {
        viewer.AddCamera(
                ns_viewer::Posed(view.CurToW.so3().matrix(), view.CurToW.translation()).cast<float>(),
                ns_viewer::Colour(0.0f, 1.0f, 0.0f, 0.3f), 0.15f
        );
    }
    viewer.RunMultiThread();
    for (const auto &[viewId, view]: views) {
        viewer.Lock();
        std::vector<std::string> lineNames;
        for (const auto &[trackId, lm]: landmarks) {
            for (const auto &feat: lm.features) {
                if (feat.first == viewId) {
                    pcl::PointXYZ p1, p2;
                    auto t = view.CurToW.translation();
                    p1.x = t(0);
                    p1.y = t(1);
                    p1.z = t(2);
                    p2.x = lm.x(0);
                    p2.y = lm.x(1);
                    p2.z = lm.x(2);
                    auto newNames = viewer.AddLine(p1, p2);
                    lineNames.resize(lineNames.size() + newNames.size());
                    std::copy_n(newNames.cbegin(), newNames.size(), lineNames.end() - newNames.size());
                }
            }
        }
        auto cameraNames = viewer.AddCamera(
                ns_viewer::Posed(view.CurToW.so3().matrix(), view.CurToW.translation()).cast<float>(),
                ns_viewer::Colour::Red(), 0.15f
        );
        viewer.UnLock();
        std::this_thread::sleep_for(std::chrono::seconds(1));
        viewer.Lock();
        viewer.RemoveEntities({cameraNames, lineNames});
        if (viewer.GetViewer()->wasStopped()) {
            viewer.UnLock();
            break;
        }
        viewer.UnLock();
    }
}

openMVG::sfm::SfM_Data
OrganizeSfMData(const std::map<std::uint32_t, Frame> &views,
                const std::map<std::uint32_t, Landmark> &landmarks,
                int width, int height, double f) {
    openMVG::sfm::SfM_Data sfMData;
    sfMData.s_root_path = "undefined";

    std::shared_ptr<openMVG::cameras::IntrinsicBase> intri =
            std::make_shared<openMVG::cameras::Pinhole_Intrinsic_Brown_T2>(
                    width, height, f, width / 2.0, height / 2.0
            );
    sfMData.intrinsics.insert(std::make_pair(0, intri));
    auto digNum = std::to_string(views.size()).size();
    for (const auto &[viewId, view]: views) {
        std::string curImageName;
        std::stringstream stream2;
        stream2 << std::setfill('0') << std::setw(static_cast<int>(digNum)) << viewId;
        stream2 >> curImageName;
        curImageName = "/" + curImageName + ".png";

        std::shared_ptr<openMVG::sfm::View> sfmView = std::make_shared<openMVG::sfm::View>(
                curImageName, viewId, 0, viewId, width, height
        );
        sfMData.views.insert(std::make_pair(viewId, sfmView));
        openMVG::geometry::Pose3 pose(view.CurToW.so3().inverse().matrix(), view.CurToW.translation());
        sfMData.poses.insert(std::make_pair(viewId, pose));
    }
    for (const auto &[trackId, lm]: landmarks) {
        openMVG::sfm::Landmark landmark;
        landmark.X = lm.x;
        for (const auto &[viewId, feat]: lm.features) {
            landmark.obs.insert(std::make_pair(viewId, openMVG::sfm::Observation(feat, openMVG::UndefinedIndexT)));
        }

        sfMData.structure.insert(std::make_pair(trackId, landmark));
    }

    return sfMData;
}

void OutputTimestamps(const std::map<std::uint32_t, Frame> &views, const std::string &path) {
    std::ofstream file(path, std::ios::out);
    // header
    file << "# element " + std::to_string(views.size()) + '\n'
         << "# property double timeStamp\n"
         << "# property string imagePath\n"
         << "# end header\n";
    // data and info
    auto digNum = std::to_string(views.size()).size();

    file << std::fixed << std::setprecision(12);
    for (const auto &[viewId, view]: views) {
        // generate the image name
        std::string curImageName;
        std::stringstream stream2;
        stream2 << std::setfill('0') << std::setw(static_cast<int>(digNum)) << viewId;
        stream2 >> curImageName;
        curImageName = "/" + curImageName + ".png";

        file << view.timestamp << ' ' << curImageName << std::endl;
    }

    file.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_vision_node");
    LOG_PROCESS("loading ros params...")

    std::string bagPath, sfmDataOPath, odomTopic, timestampOPath;

    ros::param::get("/sim_vision_node/bag_path", bagPath);
    ros::param::get("/sim_vision_node/sfm_data_output_path", sfmDataOPath);
    ros::param::get("/sim_vision_node/timestamp_output_path", timestampOPath);
    ros::param::get("/sim_vision_node/odom_topic", odomTopic);

    LOG_VAR(bagPath)
    LOG_VAR(sfmDataOPath)
    LOG_VAR(timestampOPath)
    LOG_VAR(odomTopic)

    if (!std::filesystem::exists(bagPath)) {
        LOG_ERROR("the bag path '", bagPath, "' is not exists...")
        ros::shutdown();
        return 0;
    }

    const double RAD_TO_DEG = 180.0 / M_PI;
    const double DEG_TO_RAD = M_PI / 180.0;

    Sophus::SE3d CtoL, LtoI, CtoI;
    {
        Eigen::AngleAxisd a1(10.0 * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd a2(20.0 * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd a3(30.0 * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
        CtoL = Sophus::SE3d((a1 * a2 * a3).toRotationMatrix(), Eigen::Vector3d(0.3, 0.2, 0.1));
        LOG_INFO("Camera To LiDAR:")
        LOG_INFO("Quaternion: ", CtoL.unit_quaternion().coeffs().transpose())
        LOG_INFO("Transpose: ", CtoL.translation().transpose())
        auto angles = CtoL.so3().matrix().eulerAngles(0, 1, 2) * RAD_TO_DEG;
        LOG_INFO("euler angles: ", angles.transpose())
    }
    {
        Eigen::AngleAxisd a1(5.0 * DEG_TO_RAD, Eigen::Vector3d(0, 0, 1));
        Eigen::AngleAxisd a2(2.0 * DEG_TO_RAD, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd a3(1.0 * DEG_TO_RAD, Eigen::Vector3d(1, 0, 0));
        LtoI = Sophus::SE3d((a3 * a2 * a1).toRotationMatrix(), Eigen::Vector3d(0.3, 0.15, 0.05));
        LOG_INFO("LiDAR To IMU:")
        LOG_INFO("Quaternion: ", LtoI.unit_quaternion().coeffs().transpose())
        LOG_INFO("Transpose: ", LtoI.translation().transpose())
        auto angles = LtoI.so3().matrix().eulerAngles(0, 1, 2) * RAD_TO_DEG;
        LOG_INFO("euler angles: ", angles.transpose())
    }
    CtoI = LtoI * CtoL;
    LOG_INFO("Camera To IMU:")
    LOG_INFO("Quaternion: ", CtoI.unit_quaternion().coeffs().transpose())
    LOG_INFO("Transpose: ", CtoI.translation().transpose())
    auto angles = CtoI.so3().matrix().eulerAngles(0, 1, 2) * RAD_TO_DEG;
    LOG_INFO("euler angles: ", angles.transpose())

    // frames
    std::vector<Frame> frames = GenerateBiasFrames(CtoL, bagPath, odomTopic);
    LOG_PROCESS("frame size: ", frames.size())
    // features
    pcl::PointCloud<pcl::PointXYZ> feature;
    std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<float> ux(-3.0f, 2.5f), uy(0.0f, 2.5f), uz(0.0f, 2.5f);
    for (int i = 0; i < 1000; ++i) {
        pcl::PointXYZ p;
        p.x = ux(engine);
        p.y = uy(engine);
        p.z = uz(engine);
        feature.points.push_back(p);
    }
    LOG_PROCESS("feature size: ", feature.size())

    int width = 400, height = 400;
    double fx = 450.0, fy = 450.0;
    Eigen::Matrix3d intrinsic;
    intrinsic << fx, 0, width / 2.0,
            0, fy, height / 2.0,
            0, 0, 1;

    // ready for sfm data
    std::map<std::uint32_t, Frame> views;
    std::map<std::uint32_t, Landmark> landmarks;

    for (int i = 0; i < frames.size(); ++i) {
        views.insert(std::make_pair(i, frames.at(i)));
    }

    for (int i = 0; i < feature.size(); ++i) {
        Eigen::Vector3d p;
        p(0) = feature.at(i).x;
        p(1) = feature.at(i).y;
        p(2) = feature.at(i).z;
        landmarks.insert(std::make_pair(i, Landmark(p, {})));
    }

    for (auto &[trackId, lm]: landmarks) {
        for (int i = 0; i < views.size(); ++i) {
            const auto &view = views.at(i);
            Eigen::Vector3d pInCam = view.CurToW.inverse() * lm.x;
            if (pInCam(2) < 0.0) {
                continue;
            }
            Eigen::Vector3d pInCamPlane = pInCam / pInCam(2);
            Eigen::Vector3d pInImgPlane = intrinsic * pInCamPlane;

            if (pInImgPlane(0) < 0.0 || pInImgPlane(0) > width) {
                continue;
            }
            if (pInImgPlane(1) < 0.0 || pInImgPlane(1) > height) {
                continue;
            }
            Eigen::Vector2d pixel(pInImgPlane(0), pInImgPlane(1));

            lm.features.insert(std::make_pair(i, pixel));
        }
    }

    // visual(views, landmarks);
    auto sfmData = OrganizeSfMData(views, landmarks, width, height, (fx + fy) / 2.0);
    if (!openMVG::sfm::Save(sfmData, sfmDataOPath, openMVG::sfm::ESfM_Data::ALL)) {
        LOG_ERROR("save sfm data failed!!!")
    }
    OutputTimestamps(views, timestampOPath);
    ros::shutdown();
    return 0;
}