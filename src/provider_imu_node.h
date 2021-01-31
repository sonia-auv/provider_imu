#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <sonia_common/ImuInformation.h>
#include <sonia_common/ImuTare.h>
#include <sonia_common/ImuDisturbance.h>
#include <sonia_common/ImuResetSettings.h>
#include <ros/ros.h>
#include <string>
#include "Configuration.h"

namespace provider_IMU
{
    class ProviderIMUNode
    {
    public:

        ProviderIMUNode(const ros::NodeHandlePtr &_nh);
        ~ProviderIMUNode();

        void Spin();

    private:
	    Configuration configuration;

        uint8_t calculateCheckSum(std::string data);
        void appendChecksum(std::string& data);
        bool confirmChecksum(std::string& data);

        bool tare(sonia_common::ImuTare::Request &tareRsq, sonia_common::ImuTare::Response &tareRsp);
        bool disturbance(sonia_common::ImuDisturbance::Request &disturbanceRsq, sonia_common::ImuDisturbance::Response &disturbanceRsp);
        bool reset_settings(sonia_common::ImuResetSettings::Request &settingsRsq, sonia_common::ImuResetSettings::Response &settingsRsp);
        void send_information();

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        ros::ServiceServer tare_srv;
        ros::ServiceServer disturbance_srv;
        ros::ServiceServer reset_srv;
        ros::Publisher publisher;
    };
}

#endif
