#ifndef LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_
#define LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_

#include "../lds.h"
#include "../comm/comm.h"
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"


namespace livox_mid {

class LivoxLidarCallback {
 public:
  static void LidarInfoChangeCallback(const uint32_t handle,
                                      const LivoxLidarInfo* info,
                                      void* client_data);
  static void WorkModeChangedCallback(livox_status status,
                                      uint32_t handle,
                                      LivoxLidarAsyncControlResponse *response,
                                      void *client_data);
  static void SetDataTypeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetPatternModeCallback(livox_status status, uint32_t handle,
                                     LivoxLidarAsyncControlResponse *response,
                                     void *client_data);
  static void SetBlindSpotCallback(livox_status status, uint32_t handle,
                                   LivoxLidarAsyncControlResponse *response,
                                   void *client_data);
  static void SetDualEmitCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void SetAttitudeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  static void EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);

 private:
  static LidarDevice* GetLidarDevice(const uint32_t handle, void* client_data);
};

} // namespace livox_ros

#endif  // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_
