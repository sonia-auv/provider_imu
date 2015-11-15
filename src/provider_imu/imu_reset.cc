/**
 * \file	imu_reset.cc
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	02/11/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "ros/console.h"
#include "provider_imu/3dmgx2.h"

int main(int argc, char **argv) {
  if (argv[2] != "reset") {
    ROS_WARN(
        "You are about to reset the imu software! Usage:  imu_reset "
        "/dev/ttyUSB? reset.");
  } else if (argv == NULL) {
    ROS_WARN(
        "You are about to reset the imu software! Usage:  imu_reset "
        "/dev/ttyUSB? reset.");
  }

  microstrain_3dmgx2_imu::IMU imu;

  try {
    imu.openPort(argv[1]);
  } catch (microstrain_3dmgx2_imu::Exception &e) {
    std::cout << "unable to communicate with th imuon port: " << argv[1] << "\n"
              << e.what() << std::endl;
    return 1;
  }

  imu.initTime(0.0);

  imu.reset();

  ROS_INFO("Imu has been reset!");

  return 0;
}