//
// Created by csl on 11/10/22.
//
#include "ros/ros.h"
#include <filesystem>
#include <algorithm>
#include <utility>
#include <ostream>
#include "thirdparty/csv/src/include/csv.h"
#include "thirdparty/logger/src/include/logger.h"

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

struct RawIMUData {
public:
    /**
     * @brief the members
     */
    std::string name;
    int gpsw{};
    double sow{};
    bool flag{};
    double az{};
    double ax{};
    double ay{};
    double gz{};
    double gx{};
    double gy{};

    [[nodiscard]] double TotalSeconds() const {
        return sow + gpsw * 604800;
    }

    void AdjustData() {
        static const double UNICORE_ACC_SCALE = 400.0 / std::pow(2, 31);
        static const double UNICORE_GYRO_SCALE = 2160.0 / pow(2, 31);

        ax *= -UNICORE_ACC_SCALE / 0.002;
        ay *= UNICORE_ACC_SCALE / 0.002;
        az *= UNICORE_ACC_SCALE / 0.002;

        gx *= -UNICORE_GYRO_SCALE / 0.002;
        gy *= UNICORE_GYRO_SCALE / 0.002;
        gz *= UNICORE_GYRO_SCALE / 0.002;
    }

    friend std::ostream &operator<<(std::ostream &os, const RawIMUData &data) {
        os << "name: " << data.name << " gpsw: " << data.gpsw << " sow: " << data.sow << " flag: " << data.flag
           << " az: " << data.az << " ax: " << data.ax << " ay: " << data.ay << " gz: " << data.gz << " gx: " << data.gx
           << " gy: " << data.gy;
        return os;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "raw_imu_decode_node");
    LOG_PROCESS("load ros params...")
    std::string rawIMUDir, imuFilename;
    ros::param::get("/raw_imu_decode_node/raw_imu_dir", rawIMUDir);
    ros::param::get("/raw_imu_decode_node/output_filename", imuFilename);
    LOG_VAR(rawIMUDir)
    LOG_VAR(imuFilename)

    LOG_PROCESS("find raw imu files...")
    auto files = filesInDir(rawIMUDir);
    for (auto iter = files.cbegin(); iter != files.cend();) {
        if (std::filesystem::path(*iter).extension() != ".txt") {
            LOG_WARNING("the file '", *iter, "' may be invalid, jump it.")
            iter = files.erase(iter);
        } else {
            ++iter;
        }
    }
    LOG_PLAINTEXT("the total useful raw imu files count: ", files.size())
    LOG_PROCESS("start loading raw imu data...")
    std::vector<RawIMUData> totalData;
    {
        std::vector<std::vector<RawIMUData>> data;
        for (const auto &item: files) {
            auto rawImu = ns_csv::CSVReader::read<CSV_STRUCT(
                    RawIMUData, name, gpsw, sow, flag, az, ax, ay, gz, gx, gy) >(item, ',');
            LOG_INFO("file '", item, "'")
            LOG_PLAINTEXT("range from ", rawImu.front().gpsw, ':', rawImu.front().sow, " to ",
                          rawImu.back().gpsw, ':', rawImu.back().sow)
            data.push_back(rawImu);
        }
        std::sort(data.begin(), data.end(), [](const std::vector<RawIMUData> &v1, const std::vector<RawIMUData> &v2) {
            return v1.back().TotalSeconds() < v2.front().TotalSeconds();
        });
        LOG_PROCESS("combine all data...")
        for (const auto &item: data) {
            totalData.resize(totalData.size() + item.size());
            std::copy_n(item.cbegin(), item.size(), totalData.end() - item.size());
        }
        LOG_PLAINTEXT("total data from ", totalData.front().gpsw, ':', totalData.front().sow, " to ",
                      totalData.back().gpsw, ':', totalData.back().sow)
        LOG_PLAINTEXT("time span from ", totalData.front().TotalSeconds(), " to ", totalData.front().TotalSeconds())
    }

    for (auto &frame: totalData) { frame.AdjustData(); }

    LOG_PROCESS("write to file: ", imuFilename, "...")
    ns_csv::CSVWriter::write<CSV_STRUCT(RawIMUData, sow, gx, gy, gz, ax, ay, az) >(
            imuFilename, ' ', totalData
    );
    LOG_PROCESS("finished.")

    ros::shutdown();
    return 0;
}