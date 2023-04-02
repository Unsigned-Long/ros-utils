//
// Created by csl on 4/2/23.
//
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include <filesystem>
#include "opencv2/highgui.hpp"
#include "thirdparty/logger/src/include/logger.h"
#include "opencv2/calib3d.hpp"

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

/**
 * @brief a function to split a string to some string elements according the splitor
 * @param str the string to be splited
 * @param splitor the splitor char
 * @param ignoreEmpty whether ignoring the empty string element or not
 * @return the splited string vector
 */
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "undisto_images_node");

    // get ros params
    LOG_PROCESS("read ros params...")
    std::string image_src_dir, image_dst_dir, intrinsics, distortion_coeffs;
    ros::param::get("/undisto_images_node/image_src_dir", image_src_dir);
    ros::param::get("/undisto_images_node/image_dst_dir", image_dst_dir);
    ros::param::get("/undisto_images_node/intrinsics", intrinsics);
    ros::param::get("/undisto_images_node/distortion_coeffs", distortion_coeffs);
    LOG_VAR(image_src_dir, image_dst_dir, intrinsics, distortion_coeffs)
    if (!std::filesystem::exists(image_src_dir)) {
        LOG_ERROR("the config image source directory '", image_src_dir, "' is not exists!")
        ros::shutdown();
    }
    if (!std::filesystem::exists(image_dst_dir)) {
        std::filesystem::create_directories(image_dst_dir);
    }

    auto intrinsicsElems = split(intrinsics, ',');
    auto distortionCoeffs = split(distortion_coeffs, ',');
    double fx = std::stod(intrinsicsElems.at(0)), fy = std::stod(intrinsicsElems.at(1));
    double cx = std::stod(intrinsicsElems.at(2)), cy = std::stod(intrinsicsElems.at(3));
    double k1 = std::stod(distortionCoeffs.at(0)), k2 = std::stod(distortionCoeffs.at(1));
    double p1 = std::stod(distortionCoeffs.at(2)), p2 = std::stod(distortionCoeffs.at(3));

    LOG_VAR(fx, fy, cx, cy)
    LOG_VAR(k1, k2, p1, p2)

    const cv::Mat KMat = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    const cv::Mat DMat = (cv::Mat_<double>(5, 1) << k1, k2, 0.0, 0.0, 0.0);

    auto filenames = filesInDir(image_src_dir);

    for (const auto &filename: filenames) {
        LOG_VAR(filename)
        std::string undistoFilename = image_dst_dir + "/" + filename.substr(filename.find_last_of('/') + 1);
        LOG_VAR(undistoFilename)

        cv::Mat imgSrc = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        cv::Mat imgDst;
        cv::undistort(imgSrc, imgDst, KMat, DMat);
        // cv::imshow(filename, imgSrc);
        // cv::imshow(filename + "-undisto", imgDst);
        // cv::waitKey(0);
        cv::imwrite(undistoFilename, imgDst);
    }

    ros::shutdown();
    return 0;
}