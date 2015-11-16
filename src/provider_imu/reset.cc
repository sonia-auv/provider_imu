/**
 * \file	imu_reset.cc
 * \author	Antoine Dozois <dozois.a@gmail.com>
 * \date	02/11/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include "ros/console.h"
#include "imu_driver.h"

int main(int argc, char **argv) {
  if (argv[2] != "reset") {
    ROS_WARN(
        "You are about to reset the imu software! Usage:  imu_reset "
        "/dev/ttyUSB? reset.");
  } else if (argv == nullptr) {
    ROS_WARN(
        "You are about to reset the imu software! Usage:  imu_reset "
        "/dev/ttyUSB? reset.");
  }

  provider_imu::ImuDriver imu;

  try {
    imu.OpenPort(argv[1]);
  } catch (std::runtime_error &e) {
    std::cout << "unable to communicate with the IMU on port: " << argv[1]
              << "\n" << e.what() << std::endl;
    return 1;
  }

  imu.InitTime(0.0);

  imu.Reset();

  ROS_INFO("Imu has been reset!");

  return 0;
}
