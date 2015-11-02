/**
 * \file	main.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	02/11/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <ros/ros.h>
#include "provider_imu/imu_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "microstrain_3dmgx2_node");

  ros::NodeHandle nh;

  ImuNode in(nh);
  in.spin();

  return (0);
}
