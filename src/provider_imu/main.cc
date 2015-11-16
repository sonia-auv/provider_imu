/**
 * \file	main.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	02/11/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "provider_imu/imu_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "provider_imu_node");

  ros::NodeHandle nh;

  provider_imu::ImuNode in(nh);
  in.Spin();

  return (0);
}
