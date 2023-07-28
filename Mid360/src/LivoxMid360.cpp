#include "lds_lidar.h"
#include <iostream>
#include "LivoxMid360.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"

#include <fstream>
#include <iostream>

// #include <pcl/visualization/cloud_viewer.h>

using namespace livox_mid;

LivoxMid360::LivoxMid360()
{
}

LivoxMid360::~LivoxMid360()
{
  printf("delete livox mid360");
}
void LivoxMid360::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do
  {
    std::vector<PointXyzlt> PointCloud;
    PointCloud.clear();
    uint64_t timestamp = 0;
    uint64_t timestampPrev=0;
    lddc->DistributePointCloudData(PointCloud, timestamp);
    if (timestampPrev!=timestamp)
    {
      std::cout << "PointCloud------:" << PointCloud.size() << std::endl;
    }
    timestampPrev=timestamp;
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void LivoxMid360::ImuDataPollThread()
{
  std::future_status status;
  u_int64_t timestamp = 0;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do
  {
    livox_mid::ImuData imu_data;
        // std::vecto<livox_mid::ImuData> imu_data;
    lddc->DistributeImuData(imu_data);
    
    if (timestamp != imu_data.time_stamp)
    {
      // std::cout << "ImuData------:" << imu_data.time_stamp << std::endl;
    }
    timestamp = imu_data.time_stamp;
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}
void LivoxMid360::run()
{
  //    std::cout << "Livox Ros Driver Version: " << LIVOX_MID_DRIVER_VERSION_STRING << "\n";
  // signal(SIGINT, SignalHandler);
  LivoxLidarSdkVer _sdkversion;
  GetLivoxLidarSdkVer(&_sdkversion);

  if (_sdkversion.major < kSdkVersionMajorLimit)
  {
    printf("The SDK version is too low");
  }
  // saveCfgFile();
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag = false;
  std::string m_IP = "192.168.192.46";
  int ret = 0;
  double mMaxLength = 1.2;

  const std::string &path = "Mid360/config/MID360_config.json";
  lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                  publish_freq, frame_id, lidar_bag, imu_bag, mMaxLength, m_IP);
  LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
  lddc->RegisterLds(static_cast<Lds *>(read_lidar));
  bool flag = true;
  if (!read_lidar->isInitialized())
    flag = read_lidar->InitLdsLidar(path);

  future_ = exit_signal_.get_future();
  if (flag)
  {
    printf("lds lidar has init!");
  }
  else
  {
    printf("Init lds lidar fail!");
    // SLEEP(500);
  }

  std::thread pointclouddata_poll_thread_(&LivoxMid360::PointCloudDataPollThread, this);
  std::thread imudata_poll_thread_(&LivoxMid360::ImuDataPollThread, this);
  while (true)
  {
  }
}

int main()
{
  LivoxMid360 lm360;
  lm360.run();

  return 0;
}
