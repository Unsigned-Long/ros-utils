//
// Created by csl on 11/10/22.
//
#include <ostream>
#include "ros/ros.h"
#include "thirdparty/csv/src/include/csv.h"
#include "thirdparty/logger/src/include/logger.h"
#include "filesystem"

struct RawIMUData {
public:
    double sow{};

    double gx{};
    double gy{};
    double gz{};

    double ax{};
    double ay{};
    double az{};

    friend std::ostream &operator<<(std::ostream &os, const RawIMUData &data) {
        os << "sow: " << data.sow << " gx: " << data.gx << " gy: " << data.gy << " gz: " << data.gz << " ax: "
           << data.ax << " ay: " << data.ay << " az: " << data.az;
        return os;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "raw_imu_reconstructor_node");
    LOG_PROCESS("load ros params...")
    std::string rawImuFilename, srcAdisIMUFilename, dstAdisIMUFilename;
    ros::param::get("/raw_imu_reconstructor_node/raw_imu_filename", rawImuFilename);
    ros::param::get("/raw_imu_reconstructor_node/src_imu_adis", srcAdisIMUFilename);
    ros::param::get("/raw_imu_reconstructor_node/dst_imu_adis", dstAdisIMUFilename);
    LOG_VAR(rawImuFilename)
    LOG_VAR(srcAdisIMUFilename)
    LOG_VAR(dstAdisIMUFilename)
    if (!std::filesystem::exists(rawImuFilename)) {
        LOG_ERROR("the raw imu file: ", rawImuFilename, " is not exists!")
        ros::shutdown();
        return 0;
    }
    if (!std::filesystem::exists(srcAdisIMUFilename)) {
        LOG_ERROR("the src imu adis file: ", srcAdisIMUFilename, " is not exists!")
        ros::shutdown();
        return 0;
    }
    LOG_PROCESS("loading raw imu data...")
    auto rawImuData = ns_csv::CSVReader::read<CSV_STRUCT(RawIMUData, sow, gx, gy, gz, ax, ay, az) >(
            rawImuFilename, ' ');
    LOG_PLAINTEXT("raw imu data size: ", rawImuData.size(), ", time span from ", rawImuData.front().sow, " to ",
                  rawImuData.back().sow)
    LOG_PROCESS("loading adis imu data...")
    auto srcAdisIMUData = ns_csv::CSVReader::read<CSV_STRUCT(RawIMUData, sow, gx, gy, gz, ax, ay, az) >(
            srcAdisIMUFilename, ' ');
    LOG_PLAINTEXT("adis imu data size: ", srcAdisIMUData.size(), ", time span from ", srcAdisIMUData.front().sow,
                  " to ", srcAdisIMUData.back().sow)
    double startTimestamp = srcAdisIMUData.front().sow;
    double endTimestamp = srcAdisIMUData.back().sow;
    if (startTimestamp > rawImuData.back().sow || endTimestamp < rawImuData.front().sow) {
        LOG_ERROR("the source adis imu data time span is out of the raw imu data range!!!")
    }
    if (startTimestamp < rawImuData.front().sow) {
        startTimestamp = rawImuData.front().sow;
        LOG_WARNING("adjust the adis imu data's start time stamp.")
    }
    if (endTimestamp > rawImuData.back().sow) {
        endTimestamp = rawImuData.back().sow;
        LOG_WARNING("adjust the adis imu data's end time stamp.")
    }
    LOG_INFO("the dst adis imu data time span: ", startTimestamp, " to ", endTimestamp)
    LOG_PROCESS("combine dst adis data...")
    auto sIter = std::find_if(rawImuData.cbegin(), rawImuData.cend(), [startTimestamp](const RawIMUData &data) {
        return data.sow > startTimestamp;
    });
    auto eIter = std::find_if(rawImuData.rbegin(), rawImuData.rend(), [endTimestamp](const RawIMUData &data) {
        return data.sow < endTimestamp;
    });
    std::vector<RawIMUData> dstAdisIMUData(eIter.base() - sIter);
    std::copy_n(sIter, dstAdisIMUData.size(), dstAdisIMUData.begin());

    LOG_VAR(dstAdisIMUData.front())
    LOG_VAR(dstAdisIMUData.back())
    LOG_VAR(dstAdisIMUData.size())

    LOG_INFO("add ", dstAdisIMUData.size() - srcAdisIMUData.size(), " new imu frames.")
    LOG_PROCESS("finished")
    ns_csv::CSVWriter::write<CSV_STRUCT(RawIMUData, sow, gx, gy, gz, ax, ay, az) >(
            dstAdisIMUFilename, ' ', dstAdisIMUData
    );
    ros::shutdown();
    return 0;
}
