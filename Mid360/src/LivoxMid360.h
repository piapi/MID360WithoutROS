#ifndef LIVOX_MID_360_H
#define LIVOX_MID_360_H

#include <chrono>
#include <vector>
#include <csignal>
#include <future>
#include <thread>
#include "lddc.h"
// #include "lds_lvx_mid360.h"
#include "livox_lidar_api.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include "comm/lidar_imu_data_queue.h"
#define LIVOX_MID_DRIVER_VER_MAJOR 2
#define LIVOX_MID_DRIVER_VER_MINOR 7
#define LIVOX_MID_DRIVER_VER_PATCH 0

#define GET_STRING(n) GET_STRING_DIRECT(n)
#define GET_STRING_DIRECT(n) #n

#define LIVOX_MID_DRIVER_VERSION_STRING                        \
    GET_STRING(LIVOX_MID_DRIVER_VER_MAJOR)                     \
    "." GET_STRING(LIVOX_MID_DRIVER_VER_MINOR) "." GET_STRING( \
        LIVOX_MID_DRIVER_VER_PATCH)
class LivoxMid360
{
    const int32_t kSdkVersionMajorLimit = 2;

public:
    LivoxMid360();
    ~LivoxMid360();
    void run();
    livox_mid::Lddc *lddc;
    bool saveCfgFile();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloudInFov(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
    // std::shared_ptr<std::thread> pointclouddata_poll_thread_;
    // std::shared_ptr<std::thread> imudata_poll_thread_;
    void PointCloudDataPollThread();
    void ImuDataPollThread();

private:
    inline void SignalHandler(int signum)
    {
        printf("livox ros driver will exit\r\n");

        exit(signum);
    }
};

#endif
