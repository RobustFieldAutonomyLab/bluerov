/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IMU_VN_100_ROS_H_
#define IMU_VN_100_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include <vn100.h>

namespace imu_vn_100 {

/**
 * @brief ImuVn100 The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 {
 public:
  static constexpr int kBaseImuRate = 800;
  static constexpr int kDefaultImuRate = 100;
  static constexpr int kDefaultSyncOutRate = 20;

  explicit ImuVn100(const ros::NodeHandle& pnh);
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100();

  void Initialize();

  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData& data);

  void RequestOnce();

  void Idle(bool need_reply = true);

  void Resume(bool need_reply = true);

  void Disconnect();

  void Configure();

  struct SyncInfo {
    unsigned count = 0;
    ros::Time time;

    int rate = -1;
    double rate_double = -1;
    int pulse_width_us = 1000;
    int skip_count = 0;

    void Update(const unsigned sync_count, const ros::Time& sync_time);
    void FixSyncRate();
    bool SyncEnabled() const;
  };

  const SyncInfo sync_info() const { return sync_info_; }

 private:
  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Settings
  std::string port_;
  int baudrate_ = 921600;
  int imu_rate_ = kDefaultImuRate;
  double imu_rate_double_ = kDefaultImuRate;
  std::string frame_id_;

  bool enable_mag_ = true;
  bool enable_pres_ = true;
  bool enable_temp_ = true;
  bool enable_rpy_ = false;

  bool binary_output_ = true;
  int binary_async_mode_ = BINARY_ASYNC_MODE_SERIAL_2;

  bool imu_compensated_ = false;

  bool tf_ned_to_enu_ = false;

  bool vpe_enable_ = true;
  int vpe_heading_mode_ = 1;
  int vpe_filtering_mode_ = 1;
  int vpe_tuning_mode_ = 1;
  VnVector3 vpe_mag_base_tuning_;
  VnVector3 vpe_mag_adaptive_tuning_;
  VnVector3 vpe_mag_adaptive_filtering_;
  VnVector3 vpe_accel_base_tuning_;
  VnVector3 vpe_accel_adaptive_tuning_;
  VnVector3 vpe_accel_adaptive_filtering_;

  SyncInfo sync_info_;

  ros::Publisher imu_pub_;

  void FixImuRate();
  void LoadParameters();
};

// Just don't like type that is ALL CAP
using VnErrorCode = VN_ERROR_CODE;
void VnEnsure(const VnErrorCode& error_code);

}  // namespace imu_vn_100

#endif  // IMU_VN_100_ROS_H_
