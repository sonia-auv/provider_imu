//
// Created by coumarc9 on 7/24/17. and modified by Lucas ^^
//

#include "Configuration.h"

namespace provider_IMU
{

    Configuration::Configuration(const ros::NodeHandlePtr &nh)
        : nh(nh),
          ttyPort("/dev/IMU"),
          settingsFile("settings.txt")
    {
        Deserialize();
    }

    Configuration::~Configuration() {}

    void Configuration::Deserialize() {

        ROS_INFO("Deserialize params");

        FindParameter("/connection/tty_port", ttyPort);
        FindParameter("/settings/setting_file", settingsFile);

        ROS_INFO("End deserialize params");
    }

    template <typename TType>
    void Configuration::FindParameter(const std::string &paramName, TType &attribute) {
        if (nh->hasParam("/provider_imu" + paramName)) {
            nh->getParam("/provider_imu" + paramName, attribute);
        } else {
            ROS_WARN_STREAM("Did not find /provider_imu" + paramName
                                    << ". Using default.");
        }
    }

}
