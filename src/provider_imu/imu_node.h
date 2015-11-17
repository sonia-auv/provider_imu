/**
 * \file	imu_node.h
 * \copyright Copyright (c) 2008-2010, Willow Garage, Inc. All rights reserved.
 * \section LICENSE
 *
 * Software License Agreement (BSD License)
 *
 *  Microstrain 3DM-GX2 node
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROVIDER_IMU_IMU_NODE_H_
#define PROVIDER_IMU_IMU_NODE_H_

#include <assert.h>
#include <math.h>
#include <iostream>
#include <boost/format.hpp>
#include <ros/time.h>
#include <self_test/self_test.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <provider_imu/AddOffset.h>
#include "imu_driver.h"

namespace provider_imu {

class ImuNode {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  ImuNode(ros::NodeHandle h);

  ~ImuNode();

  //============================================================================
  // P U B L I C   M E T H O D S

  void SetErrorStatusF(const char *format, ...);

  // Prints an error message if it isn't the same old error message.
  void SetErrorStatus(const std::string msg);

  void ClearErrorStatus();

  int Start();

  std::string getID(bool output_info = false);

  int Stop();

  int PublishData();

  bool Spin();

  void PublishIsCalibrated();

  void PreTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void InterruptionTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void ReadIDTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void GyroBiasTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void GetData(sensor_msgs::Imu &data);

  tf::Quaternion GetQuaternion(sensor_msgs::Imu &data);

  void StreamedDataTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void GravityTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void DisconnectTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void ResumeTest(diagnostic_updater::DiagnosticStatusWrapper &status);

  void GetDeviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status);

  void GetCalibrationStatus(
      diagnostic_updater::DiagnosticStatusWrapper &status);

  bool AddOffsetCallback(provider_imu::AddOffset::Request &req,
                         provider_imu::AddOffset::Response &resp);

  bool CalibrateCallback(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &resp);

  void CheckCalibration();

 private:
  //============================================================================
  // P R I V A T E  M E M B E R S

  /**
   * The driver that handle the communication between this software and the IMU.
   */
  provider_imu::ImuDriver imu;

  /**
   * ROS Message that contains the IMU informations.
   * This message will be published on the data topic.
   */
  sensor_msgs::Imu reading;

  /**
   * The serial port to connect to.
   * This information is being used by the serial driver implementation.
   */
  std::string port;

  /**
   * All possible command that the IMU Node can send to the IMU.
   */
  provider_imu::ImuDriver::cmd cmd;

  self_test::TestRunner self_test_;

  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;

  ros::NodeHandle private_node_handle_;

  ros::Publisher imu_data_pub_;

  ros::ServiceServer add_offset_serv_;

  ros::ServiceServer calibrate_serv_;

  ros::Publisher is_calibrated_pub_;

  bool running;

  bool autocalibrate_;

  bool calibrate_requested_;

  bool calibrated_;

  int error_count_;

  int slow_count_;

  std::string was_slow_;

  std::string error_status_;

  std::string frameid_;

  double offset_;

  double bias_x_;

  double bias_y_;

  double bias_z_;

  double angular_velocity_stdev_;

  double angular_velocity_covariance_;

  double linear_acceleration_covariance_;

  double linear_acceleration_stdev_;

  double orientation_covariance_;

  double orientation_stdev_;

  double max_drift_rate_;

  double desired_freq_;

  diagnostic_updater::FrequencyStatus freq_diag_;
};

}  // namespace provider_imu

#endif  // PROVIDER_IMU_IMU_NODE_H_