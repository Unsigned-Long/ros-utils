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


int main(int argc, char **argv) {
    ros::init(argc, argv, "images_thinning_node");

    // get ros params
    LOG_PROCESS("read ros params...")
    std::string image_src_dir, image_dst_dir;
    int thinning_step;
    ros::param::get("/images_thinning_node/image_src_dir", image_src_dir);
    ros::param::get("/images_thinning_node/image_dst_dir", image_dst_dir);
    ros::param::get("/images_thinning_node/thinning_step", thinning_step);
    LOG_VAR(image_src_dir, image_dst_dir, thinning_step)
    if (!std::filesystem::exists(image_src_dir)) {
        LOG_ERROR("the config image source directory '", image_src_dir, "' is not exists!")
        ros::shutdown();
    }
    if (!std::filesystem::exists(image_dst_dir)) {
        std::filesystem::create_directories(image_dst_dir);
    }

    auto filenames = filesInDir(image_src_dir);

    for (int i = 0; i < filenames.size(); i += thinning_step) {
        auto filename = filenames.at(i);
        std::string dstFilename = image_dst_dir + "/" + filename.substr(filename.find_last_of('/') + 1);

        LOG_VAR(filename)
        LOG_VAR(dstFilename)
        LOG_ENDL()

        cv::Mat imgSrc = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        cv::imwrite(dstFilename, imgSrc);
    }

    ros::shutdown();
    return 0;
}