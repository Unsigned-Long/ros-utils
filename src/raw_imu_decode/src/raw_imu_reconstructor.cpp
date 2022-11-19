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
    int gpsw{};
    double sow{};

    double gx{};
    double gy{};
    double gz{};

    double ax{};
    double ay{};
    double az{};

    [[nodiscard]] double TotalSeconds() const {
        return sow + gpsw * 604800;
    }

    friend std::ostream &operator<<(std::ostream &os, const RawIMUData &data) {
        os << " gpsw: " << data.gpsw << " sow: " << data.sow << " gx: " << data.gx << " gy: " << data.gy << " gz: "
           << data.gz << " ax: " << data.ax << " ay: " << data.ay << " az: " << data.az;
        return os;
    }
};

struct Frame {
    double timeStamp{};
    std::string filename;

    friend std::ostream &operator<<(std::ostream &os, const Frame &frame) {
        os << "timeStamp: " << frame.timeStamp << " filename: " << frame.filename;
        return os;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "raw_imu_reconstructor_node");

    // ------------------------
    LOG_PROCESS("load ros params...")
    std::string rawImuFilename, dstAdisIMUFilename, imgTimestampFilename;
    int gpsWeek;
    ros::param::get("/raw_imu_reconstructor_node/raw_imu_filename", rawImuFilename);
    ros::param::get("/raw_imu_reconstructor_node/dst_imu_adis", dstAdisIMUFilename);
    ros::param::get("/raw_imu_reconstructor_node/image_timestamp", imgTimestampFilename);
    ros::param::get("/raw_imu_reconstructor_node/gps_week", gpsWeek);
    LOG_VAR(rawImuFilename)
    LOG_VAR(dstAdisIMUFilename)
    LOG_VAR(imgTimestampFilename)
    LOG_VAR(gpsWeek)

    // ------------------------
    if (!std::filesystem::exists(rawImuFilename)) {
        LOG_ERROR("the raw imu file: ", rawImuFilename, " is not exists!")
        ros::shutdown();
        return 0;
    }
    if (!std::filesystem::exists(imgTimestampFilename)) {
        LOG_ERROR("the image timestamp file: ", imgTimestampFilename, " is not exists!")
        ros::shutdown();
        return 0;
    }

    // ------------------------
    LOG_PROCESS("loading raw imu data...")
    auto rawImuData = ns_csv::CSVReader::read<CSV_STRUCT(RawIMUData, gpsw, sow, gx, gy, gz, ax, ay, az) >(
            rawImuFilename, ' ');
    LOG_PLAINTEXT("raw imu data size: ", rawImuData.size(), ", time span from ", rawImuData.front().TotalSeconds(),
                  " to ", rawImuData.back().TotalSeconds())

    // ------------------------
    LOG_PROCESS("loading camera timestamp data...")
    auto imgTimestamp = ns_csv::CSVReader::read<CSV_STRUCT(Frame, timeStamp, filename) >(
            imgTimestampFilename, ','
    );
    for (auto &item: imgTimestamp) {
        // week seconds -> gsp seconds
        item.timeStamp = item.timeStamp + gpsWeek * 604800;
    }
    LOG_PLAINTEXT("image timestamp data size: ", imgTimestamp.size(), ", time span from ",
                  imgTimestamp.front().timeStamp, " to ", imgTimestamp.back().timeStamp)

    // ------------------------
    double startTimestamp = imgTimestamp.front().timeStamp;
    double endTimestamp = imgTimestamp.back().timeStamp;
    if (startTimestamp > rawImuData.back().TotalSeconds() || endTimestamp < rawImuData.front().TotalSeconds()) {
        LOG_ERROR("the source adis imu data time span is out of the raw imu data range!!! Please the gps week setting!!!")
        ros::shutdown();
        return 0;
    }
    if (startTimestamp < rawImuData.front().TotalSeconds()) {
        startTimestamp = rawImuData.front().TotalSeconds();
        LOG_WARNING("adjust the adis imu data's start time stamp.")
    }
    if (endTimestamp > rawImuData.back().TotalSeconds()) {
        endTimestamp = rawImuData.back().TotalSeconds();
        LOG_WARNING("adjust the adis imu data's end time stamp.")
    }
    LOG_INFO("the dst adis imu data time span: ", startTimestamp, " to ", endTimestamp)

    // ------------------------
    LOG_PROCESS("combine dst adis data...")
    auto sIter = std::find_if(rawImuData.cbegin(), rawImuData.cend(), [startTimestamp](const RawIMUData &data) {
        return data.TotalSeconds() > startTimestamp;
    });
    auto eIter = std::find_if(rawImuData.rbegin(), rawImuData.rend(), [endTimestamp](const RawIMUData &data) {
        return data.TotalSeconds() < endTimestamp;
    });
    std::vector<RawIMUData> dstAdisIMUData(eIter.base() - sIter);
    std::copy_n(sIter, dstAdisIMUData.size(), dstAdisIMUData.begin());

    LOG_VAR(dstAdisIMUData.size())
    LOG_VAR(dstAdisIMUData.front())
    LOG_VAR(dstAdisIMUData.back())

    LOG_PROCESS("finished")
    ns_csv::CSVWriter::write<CSV_STRUCT(RawIMUData, sow, gx, gy, gz, ax, ay, az) >(
            dstAdisIMUFilename, ' ', dstAdisIMUData
    );
    ros::shutdown();
    return 0;
}
