#include <ros/ros.h>
#include "provider_imu_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "provider_imu");

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    provider_IMU::ProviderIMUNode interface_node{nh};
    interface_node.Spin();
}
