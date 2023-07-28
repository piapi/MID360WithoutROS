#ifndef LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_
#define LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_	

#include "../comm/comm.h"

namespace livox_mid {

class LidarCommonCallback {
 public:
  static void OnLidarPointClounCb(PointFrame* frame, void* client_data);
  static void LidarImuDataCallback(ImuData* imu_data, void *client_data);
};

} // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_
