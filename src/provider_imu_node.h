#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <sonia_common/ImuInformation.h>
#include <sonia_common/ImuTare.h>
#include <sonia_common/ImuDisturbance.h>
#include <sonia_common/ImuResetSettings.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <sharedQueue.h>
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

        uint8_t calculateCheckSum(uint8_t nbByte, std::vector<uint8_t> data);
        uint8_t calculateCheckSum(uint8_t nbByte, char* data);

        void tare(const sonia_common::ImuTare::ConstPtr &tare);
        void disturbance(const sonia_common::ImuDisturbance::ConstPtr &disturbance);
        void reset_settings(const sonia_common::ImuResetSettings::ConstPtr &settings);
        void send_information();

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        ros::Subscriber subscriber;
        ros::Publisher publisher;
    };
}

#endif
