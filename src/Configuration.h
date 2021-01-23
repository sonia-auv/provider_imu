//
// Created by coumarc9 on 7/24/17. and modified by Lucas ^^
//

#ifndef INTERFACE_CONFIGURATION_H
#define INTERFACE_CONFIGURATION_H

#include <cstdint>
#include <cmath>
#include <ros/ros.h>

namespace provider_IMU
{
    class Configuration {

    public:

        Configuration(const ros::NodeHandlePtr &nh);
        ~Configuration();

        std::string getTtyPort() const {return ttyPort;}
        std::string getSettingsFile() const {return settingsFile;}

    private:

        ros::NodeHandlePtr nh;

        std::string ttyPort;
        std::string settingsFile;

        void Deserialize();
        void SetParameter();

        template <typename TType>
        void FindParameter(const std::string &paramName, TType &attribute);


        };
}




#endif
