/**
 * \file	imu_node.cc
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

#include "provider_imu/imu_node.h"

namespace provider_imu {

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ImuNode::ImuNode(ros::NodeHandle h)
    : self_test_(),
      diagnostic_(),
      node_handle_(h),
      private_node_handle_("~"),
      calibrate_requested_(false),
      error_count_(0),
      slow_count_(0),
      desired_freq_(100),
      freq_diag_(diagnostic_updater::FrequencyStatusParam(
          &desired_freq_, &desired_freq_, 0.05)) {
  ros::NodeHandle imu_node_handle(node_handle_, "provider_imu");

  private_node_handle_.param("autocalibrate", autocalibrate_, false);
  private_node_handle_.param("assume_calibrated", calibrated_, false);
  private_node_handle_.param("port", port_, std::string("/dev/ttyACM0"));
  private_node_handle_.param("max_drift_rate", max_drift_rate_, 0.0002);

  imu_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("imu", 100);
  accel_pub_ = imu_node_handle.advertise<geometry_msgs::AccelWithCovarianceStamped>("acceleration", 100);
  twist_pub_ = imu_node_handle.advertise<geometry_msgs::TwistWithCovarianceStamped>("twist", 100);
  magnetic_pub_ = imu_node_handle.advertise<sensor_msgs::MagneticField>("magnetic_field", 100);

  add_offset_serv_ = private_node_handle_.advertiseService(
      "add_offset", &ImuNode::AddOffsetCallback, this);
  calibrate_serv_ = imu_node_handle.advertiseService(
      "calibrate", &ImuNode::CalibrateCallback, this);
  is_calibrated_pub_ =
      imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);

  PublishIsCalibrated();

  cmd_ = provider_imu::ImuDriver::CMD_ACCEL_ANGRATE_MAG_ORIENT;

  running_ = false;

  bias_x_ = bias_y_ = bias_z_ = 0;

  private_node_handle_.param("driver/frame_id", frameid_, std::string("imu"));
  imu_msg_.header.frame_id = frameid_;

  private_node_handle_.param("driver/time_offset", offset_, 0.0);

  private_node_handle_.param("imu/linear_acceleration_stdev",
                             linear_acceleration_stdev_, 0.098);
  private_node_handle_.param("imu/orientation_stdev", orientation_stdev_, 0.035);
  private_node_handle_.param("imu/angular_velocity_stdev", angular_velocity_stdev_,
                             0.012);

  angular_velocity_covariance_ =
      angular_velocity_stdev_ * angular_velocity_stdev_;
  orientation_covariance_ = orientation_stdev_ * orientation_stdev_;
  linear_acceleration_covariance_ =
      linear_acceleration_stdev_ * linear_acceleration_stdev_;

  imu_msg_.linear_acceleration_covariance[0] = linear_acceleration_covariance_;
  imu_msg_.linear_acceleration_covariance[4] = linear_acceleration_covariance_;
  imu_msg_.linear_acceleration_covariance[8] = linear_acceleration_covariance_;
  accel_msg_.accel.covariance[0] = linear_acceleration_covariance_;
  accel_msg_.accel.covariance[4] = linear_acceleration_covariance_;
  accel_msg_.accel.covariance[8] = linear_acceleration_covariance_;

  imu_msg_.angular_velocity_covariance[0] = angular_velocity_covariance_;
  imu_msg_.angular_velocity_covariance[4] = angular_velocity_covariance_;
  imu_msg_.angular_velocity_covariance[8] = angular_velocity_covariance_;
  twist_msg_.twist.covariance[0] = angular_velocity_covariance_;
  twist_msg_.twist.covariance[4] = angular_velocity_covariance_;
  twist_msg_.twist.covariance[8] = angular_velocity_covariance_;

  imu_msg_.orientation_covariance[0] = orientation_covariance_;
  imu_msg_.orientation_covariance[4] = orientation_covariance_;
  imu_msg_.orientation_covariance[8] = orientation_covariance_;

  self_test_.add("Close Test", this, &ImuNode::PreTest);
  self_test_.add("Interruption Test", this, &ImuNode::InterruptionTest);
  self_test_.add("Connect Test", this, &ImuNode::ConnectTest);
  self_test_.add("Read ID Test", this, &ImuNode::ReadIDTest);
  self_test_.add("Gyro Bias Test", this, &ImuNode::GyroBiasTest);
  self_test_.add("Streamed Data Test", this, &ImuNode::StreamedDataTest);
  self_test_.add("Gravity Test", this, &ImuNode::GravityTest);
  self_test_.add("Disconnect Test", this, &ImuNode::DisconnectTest);
  self_test_.add("Resume Test", this, &ImuNode::ResumeTest);

  diagnostic_.add(freq_diag_);
  diagnostic_.add("Calibration Status", this, &ImuNode::GetCalibrationStatus);
  diagnostic_.add("IMU Status", this, &ImuNode::GetDeviceStatus);
}

ImuNode::~ImuNode() { Stop(); }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ImuNode::SetErrorStatusF(const char *format, ...) {
  va_list va;
  char buff[1000];
  va_start(va, format);
  if (vsnprintf(buff, 1000, format, va) >= 1000)
    ROS_DEBUG("Really long string in setErrorStatus, it was truncated.");
  std::string value = std::string(buff);
  SetErrorStatus(buff);
}

//------------------------------------------------------------------------------
//
void ImuNode::SetErrorStatus(const std::string msg) {
  if (error_status_ != msg) {
    error_status_ = msg;
    ROS_ERROR(
        "%s You may find further details at "
            "http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting",
        msg.c_str());
  } else {
    ROS_DEBUG(
        "%s You may find further details at "
            "http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting",
        msg.c_str());
  }
}

//------------------------------------------------------------------------------
//
void ImuNode::ClearErrorStatus() { error_status_.clear(); }

//------------------------------------------------------------------------------
//
int ImuNode::Start() {
  Stop();

  try {
    try {
      imu_driver_.OpenPort(port_.c_str());
    } catch (std::runtime_error &e) {
      error_count_++;
      SetErrorStatus(e.what());
      diagnostic_.broadcast(2, e.what());
      return -1;
    }

    diagnostic_.setHardwareID(getID(true));

    ROS_INFO("Initializing IMU time with offset %f.", offset_);

    imu_driver_.InitTime(offset_);

    if (autocalibrate_ || calibrate_requested_) {
      CheckCalibration();
      calibrate_requested_ = false;
      autocalibrate_ =
          false;  // No need to do this each time we reopen the device.
    } else {
      ROS_INFO(
          "Not calibrating the IMU sensor. Use the calibrate service to "
              "calibrate it before use.");
    }

    ROS_INFO("IMU sensor initialized.");

    imu_driver_.SetContinuous(cmd_);

    freq_diag_.clear();

    running_ = true;

  } catch (std::runtime_error &e) {
    error_count_++;
    usleep(100000);                // Give isShuttingDown a chance to go true.
    if (!ros::isShuttingDown()) {  // Don't warn if we are shutting down.
      SetErrorStatusF(
          "Exception thrown while starting IMU. This sometimes happens if "
              "you are not connected to an IMU or if another process is trying "
              "to access the IMU port. You may try 'lsof|grep %s' to see if "
              "other processes have the port open. %s",
          port_.c_str(), e.what());
      diagnostic_.broadcast(2, "Error opening IMU.");
    }
    return -1;
  }

  return (0);
}

//------------------------------------------------------------------------------
//
std::string ImuNode::getID(bool output_info) {
  char dev_name[17];
  char dev_model_num[17];
  char dev_serial_num[17];
  char dev_opt[17];
  imu_driver_.GetDeviceIdentifierString(provider_imu::ImuDriver::ID_DEVICE_NAME,
                                        dev_name);
  imu_driver_.GetDeviceIdentifierString(provider_imu::ImuDriver::ID_MODEL_NUMBER,
                                        dev_model_num);
  imu_driver_.GetDeviceIdentifierString(provider_imu::ImuDriver::ID_SERIAL_NUMBER,
                                        dev_serial_num);
  imu_driver_.GetDeviceIdentifierString(provider_imu::ImuDriver::ID_DEVICE_OPTIONS,
                                        dev_opt);

  if (output_info)
    ROS_INFO("Connected to IMU [%s] model [%s] s/n [%s] options [%s]", dev_name,
             dev_model_num, dev_serial_num, dev_opt);

  char *dev_name_ptr = dev_name;
  char *dev_model_num_ptr = dev_model_num;
  char *dev_serial_num_ptr = dev_serial_num;

  while (*dev_name_ptr == ' ') dev_name_ptr++;
  while (*dev_model_num_ptr == ' ') dev_model_num_ptr++;
  while (*dev_serial_num_ptr == ' ') dev_serial_num_ptr++;

  return (boost::format("%s_%s-%s") % dev_name_ptr % dev_model_num_ptr %
      dev_serial_num_ptr)
      .str();
}

//------------------------------------------------------------------------------
//
int ImuNode::Stop() {
  if (running_) {
    try {
      imu_driver_.ClosePort();
    } catch (std::runtime_error &e) {
      error_count_++;
      ROS_INFO("Exception thrown while stopping IMU. %s", e.what());
    }
    running_ = false;
  }

  return (0);
}

//------------------------------------------------------------------------------
//
int ImuNode::PublishData() {
  try {
    static double prevtime = 0;
    double starttime = ros::Time::now().toSec();
    if (prevtime && prevtime - starttime > 0.05) {
      ROS_WARN("Full IMU loop took %f ms. Nominal is 10ms.",
               1000 * (prevtime - starttime));
      was_slow_ = "Full IMU loop was slow.";
      slow_count_++;
    }
    BuildRosMessages();
    double endtime = ros::Time::now().toSec();
    if (endtime - starttime > 0.05) {
      ROS_WARN("Gathering data took %f ms. Nominal is 10ms.",
               1000 * (endtime - starttime));
      was_slow_ = "Full IMU loop was slow.";
      slow_count_++;
    }
    prevtime = starttime;
    starttime = ros::Time::now().toSec();

    imu_pub_.publish(imu_msg_);
    accel_pub_.publish(accel_msg_);
    twist_pub_.publish(twist_msg_);
    magnetic_pub_.publish(magnetic_field_msg_);

    endtime = ros::Time::now().toSec();
    if (endtime - starttime > 0.05) {
      ROS_WARN("Publishing took %f ms. Nominal is 10 ms.",
               1000 * (endtime - starttime));
      was_slow_ = "Full IMU loop was slow.";
      slow_count_++;
    }

    freq_diag_.tick();

    // If we got here, then the IMU really is working.
    // Next time an error occurs, we want to print it.
    ClearErrorStatus();
  } catch (std::runtime_error &e) {
    error_count_++;

    // Give isShuttingDown a chance to go true.
    usleep(100000);
    if (!ros::isShuttingDown()) {
      // Don't warn if we are shutting down.
      ROS_WARN(
          "Exception thrown while trying to get the IMU reading. This "
              "sometimes happens due to a communication glitch, or if another "
              "process is trying to access the IMU port. You may try 'lsof|grep "
              "%s' to see if other processes have the port open. %s",
          port_.c_str(), e.what());
    }
    return -1;
  }

  return (0);
}

//------------------------------------------------------------------------------
//
bool ImuNode::Spin() {
  // Using ros::isShuttingDown to avoid
  // restarting the node during a shutdown.
  while (!ros::isShuttingDown()) {
    if (Start() == 0) {
      while (node_handle_.ok()) {
        if (PublishData() < 0) break;
        self_test_.checkTest();
        diagnostic_.update();
        ros::spinOnce();
      }
    } else {
      // No need for diagnostic here since a broadcast occurs in start
      // when there is an error.
      usleep(1000000);
      self_test_.checkTest();
      ros::spinOnce();
    }
  }

  Stop();

  return true;
}

//------------------------------------------------------------------------------
//
void ImuNode::PublishIsCalibrated() {
  std_msgs::Bool msg;
  msg.data = calibrated_;
  is_calibrated_pub_.publish(msg);
}

//------------------------------------------------------------------------------
//
void ImuNode::PreTest(diagnostic_updater::DiagnosticStatusWrapper &status) {
  try {
    imu_driver_.ClosePort();

    status.summary(0, "Device closed successfully.");
  } catch (std::runtime_error &e) {
    status.summary(1, "Failed to close device.");
  }
}

//------------------------------------------------------------------------------
//
void ImuNode::InterruptionTest(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (imu_pub_.getNumSubscribers() == 0)
    status.summary(0, "No operation interrupted.");
  else
    status.summary(1,
                   "There were active subscribers.  Running of self test "
                       "interrupted operations.");
}

//------------------------------------------------------------------------------
//
void ImuNode::ConnectTest(diagnostic_updater::DiagnosticStatusWrapper &status) {
  imu_driver_.OpenPort(port_.c_str());

  status.summary(0, "Connected successfully.");
}

//------------------------------------------------------------------------------
//
void ImuNode::ReadIDTest(diagnostic_updater::DiagnosticStatusWrapper &status) {
  self_test_.setID(getID());

  status.summary(0, "Read Successfully");
}

//------------------------------------------------------------------------------
//
void ImuNode::GyroBiasTest(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  imu_driver_.InitGyros(&bias_x_, &bias_y_, &bias_z_);

  status.summary(0, "Successfully calculated gyro biases.");

  status.add("Bias_X", bias_x_);
  status.add("Bias_Y", bias_y_);
  status.add("Bias_Z", bias_z_);
}

//------------------------------------------------------------------------------
//
void ImuNode::BuildRosMessages() {
  uint64_t time;
  double accel[3];
  double angrate[3];
  double mag[3];
  double orientation[9];

  imu_driver_.ReceiveAccelAngrateMagOrientation(&time, accel, angrate, mag, orientation);

  imu_msg_.linear_acceleration.x = accel[0];
  imu_msg_.linear_acceleration.y = accel[1];
  imu_msg_.linear_acceleration.z = accel[2];

  accel_msg_.accel.accel.linear.x = accel[0];
  accel_msg_.accel.accel.linear.x = accel[1];
  accel_msg_.accel.accel.linear.x = accel[2];

  imu_msg_.angular_velocity.x = angrate[0];
  imu_msg_.angular_velocity.y = angrate[1];
  imu_msg_.angular_velocity.z = angrate[2];

  twist_msg_.twist.twist.angular.x = angrate[0];
  twist_msg_.twist.twist.angular.x = angrate[1];
  twist_msg_.twist.twist.angular.x = angrate[2];

  tf::Quaternion quat;
  (tf::Matrix3x3(-1, 0, 0, 0, 1, 0, 0, 0, -1) *
      tf::Matrix3x3(orientation[0], orientation[3], orientation[6], orientation[1],
                    orientation[4], orientation[7], orientation[2], orientation[5],
                    orientation[8]))
      .getRotation(quat);

  tf::quaternionTFToMsg(quat, imu_msg_.orientation);

  magnetic_field_msg_.magnetic_field.x = mag[0];
  magnetic_field_msg_.magnetic_field.y = mag[1];
  magnetic_field_msg_.magnetic_field.z = mag[2];

  imu_msg_.header.stamp = ros::Time::now().fromNSec(time);
  accel_msg_.header.stamp = ros::Time::now().fromNSec(time);
  twist_msg_.header.stamp = ros::Time::now().fromNSec(time);
  magnetic_field_msg_.header.stamp = ros::Time::now().fromNSec(time);
}

//------------------------------------------------------------------------------
//
void ImuNode::StreamedDataTest(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  uint64_t time;
  double accel[3];
  double angrate[3];

  if (!imu_driver_.SetContinuous(provider_imu::ImuDriver::CMD_ACCEL_ANGRATE)) {
    status.summary(2, "Could not start streaming data.");
  } else {
    for (int i = 0; i < 100; i++) {
      imu_driver_.ReceiveAccelAngrate(&time, accel, angrate);
    }

    imu_driver_.StopContinuous();

    status.summary(0, "Data streamed successfully.");
  }
}

//------------------------------------------------------------------------------
//
void ImuNode::GravityTest(diagnostic_updater::DiagnosticStatusWrapper &status) {
  uint64_t time;
  double accel[3];
  double angrate[3];

  double grav = 0.0;

  double grav_x = 0.0;
  double grav_y = 0.0;
  double grav_z = 0.0;

  if (!imu_driver_.SetContinuous(provider_imu::ImuDriver::CMD_ACCEL_ANGRATE)) {
    status.summary(2, "Could not start streaming data.");
  } else {
    int num = 200;

    for (int i = 0; i < num; i++) {
      imu_driver_.ReceiveAccelAngrate(&time, accel, angrate);

      grav_x += accel[0];
      grav_y += accel[1];
      grav_z += accel[2];
    }

    imu_driver_.StopContinuous();

    grav += sqrt(pow(grav_x / (double) (num), 2.0) +
        pow(grav_y / (double) (num), 2.0) +
        pow(grav_z / (double) (num), 2.0));

    // double err = (grav - microstrain_3dmgx2_imu::G);
    double err = (grav - 9.796);

    if (fabs(err) < .05) {
      status.summary(0, "Gravity detected correctly.");
    } else {
      status.summaryf(2, "Measured gravity deviates by %f", err);
    }

    status.add("Measured gravity", grav);
    status.add("Gravity error", err);
  }
}

//------------------------------------------------------------------------------
//
void ImuNode::DisconnectTest(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  imu_driver_.ClosePort();

  status.summary(0, "Disconnected successfully.");
}

//------------------------------------------------------------------------------
//
void ImuNode::ResumeTest(diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (running_) {
    imu_driver_.OpenPort(port_.c_str());
    freq_diag_.clear();

    if (imu_driver_.SetContinuous(cmd_) != true) {
      status.summary(2, "Failed to resume previous mode of operation.");
      return;
    }
  }

  status.summary(0, "Previous operation resumed successfully.");
}

//------------------------------------------------------------------------------
//
void ImuNode::GetDeviceStatus(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (!running_) {
    status.summary(2, "IMU is stopped");
  } else if (!was_slow_.empty()) {
    status.summary(1, "Excessive delay");
    was_slow_.clear();
  } else {
    status.summary(0, "IMU is running");
  }

  status.add("Device", port_);
  status.add("TF frame", frameid_);
  status.add("Error count", error_count_);
  status.add("Excessive delay", slow_count_);
}

//------------------------------------------------------------------------------
//
void ImuNode::GetCalibrationStatus(
    diagnostic_updater::DiagnosticStatusWrapper &status) {
  if (calibrated_) {
    status.summary(0, "Gyro is calibrated");
    status.add("X bias", bias_x_);
    status.add("Y bias", bias_y_);
    status.add("Z bias", bias_z_);
  } else {
    status.summary(2, "Gyro not calibrated");
  }
}

//------------------------------------------------------------------------------
//
bool ImuNode::AddOffsetCallback(sonia_msgs::AddOffset::Request &req,
                                sonia_msgs::AddOffset::Response &resp) {
  double offset = req.add_offset;
  offset_ += offset;

  ROS_INFO("Adding %f to existing IMU time offset.", offset);
  ROS_INFO("Total IMU time offset is now %f.", offset_);

  // send changes to imu driver
  imu_driver_.SetFixedOffset(offset_);

  // write changes to param server
  private_node_handle_.setParam("time_offset", offset_);

  // set response
  resp.total_offset = offset_;

  return true;
}

//------------------------------------------------------------------------------
//
bool ImuNode::CalibrateCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &resp) {
  bool old_running = running_;

  try {
    calibrate_requested_ = true;
    if (old_running) {
      Stop();
      Start();  // Start will do the calibration.
    } else {
      imu_driver_.OpenPort(port_.c_str());
      CheckCalibration();
      imu_driver_.ClosePort();
    }
  } catch (std::runtime_error &e) {
    error_count_++;
    calibrated_ = false;
    PublishIsCalibrated();
    ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
    Stop();
    if (old_running)
      Start();  // Might throw, but we have nothing to lose... Needs
    // restructuring.
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------
//
void ImuNode::CheckCalibration() {  // Expects to be called with the IMU
  // stopped.
  ROS_INFO("Calibrating IMU gyros.");
  imu_driver_.InitGyros(&bias_x_, &bias_y_, &bias_z_);

  // check calibration
  if (!imu_driver_.SetContinuous(provider_imu::ImuDriver::CMD_ACCEL_ANGRATE_ORIENT)) {
    ROS_ERROR("Could not start streaming data to verify calibration");
  } else {
    double x_rate = 0.0;
    double y_rate = 0.0;
    double z_rate = 0.0;
    size_t count = 0;
    BuildRosMessages();
    ros::Time start_time = imu_msg_.header.stamp;

    while (imu_msg_.header.stamp - start_time < ros::Duration(2.0)) {
      BuildRosMessages();
      x_rate += imu_msg_.angular_velocity.x;
      y_rate += imu_msg_.angular_velocity.y;
      z_rate += imu_msg_.angular_velocity.z;
      ++count;
    }

    double average_rate =
        sqrt(x_rate * x_rate + y_rate * y_rate + z_rate * z_rate) / count;

    if (count < 200) {
      ROS_WARN("Imu: calibration check acquired fewer than 200 samples.");
    }

    // calibration succeeded
    if (average_rate < max_drift_rate_) {
      ROS_INFO(
          "Imu: calibration check succeeded: average angular drift %f "
              "mdeg/sec < %f mdeg/sec",
          average_rate * 180 * 1000 / M_PI,
          max_drift_rate_ * 180 * 1000 / M_PI);
      calibrated_ = true;
      PublishIsCalibrated();
      ROS_INFO("IMU gyro calibration completed.");
      freq_diag_.clear();
    }
      // calibration failed
    else {
      calibrated_ = false;
      PublishIsCalibrated();
      ROS_ERROR(
          "Imu: calibration check failed: average angular drift = %f "
              "mdeg/sec > %f mdeg/sec",
          average_rate * 180 * 1000 / M_PI,
          max_drift_rate_ * 180 * 1000 / M_PI);
    }
    imu_driver_.StopContinuous();
  }
}

} //namespace provider_imu
