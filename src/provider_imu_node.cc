#include "provider_imu_node.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>

namespace provider_IMU
{

    // node Construtor
    ProviderIMUNode::ProviderIMUNode(const ros::NodeHandlePtr &_nh)
    : nh(_nh), configuration(_nh), serialConnection(configuration.getTtyPort())
    {
        publisher = nh->advertise<sonia_common::ImuInformation>("/provider_imu/imu_info", 100);
        tare_srv = nh->advertiseService("/provider_imu/tare", &ProviderIMUNode::tare, this);
        disturbance_srv = nh->advertiseService("/provider_imu/disturbance", &ProviderIMUNode::disturbance, this);
        reset_srv = nh->advertiseService("/provider_imu/reset_settings", &ProviderIMUNode::reset_settings, this);
    }

    // node destructor
    ProviderIMUNode::~ProviderIMUNode()
    {

    }

    // node spin
    void ProviderIMUNode::Spin()
    {
        ros::Rate r(40);
        while(ros::ok())
        {
            ros::spinOnce();
            send_information();
            r.sleep();
        }
    }

    /**
     * @brief calculate the checksum for a data string
     * 
     * @param data the string to calculate the checksum
     * @return uint8_t the checksum value
     */
    uint8_t ProviderIMUNode::calculateCheckSum(std::string data)
    {
        uint8_t check = 0;

        for(unsigned int i = 1; i < data.size(); i++)
            check ^= data[i];
        
        return check;
    }

    /**
     * @brief append the checksum at the end of the string data. 
     *        ex: $VNTAR -> $VNTAR*5F
     * 
     * @param data the string to append the checksum
     */
    void ProviderIMUNode::appendChecksum(std::string& data)
    {
        std::stringstream ss;
        uint8_t checksum = calculateCheckSum(data);
        ss << data << std::string("*") << std::hex << checksum;
        data = ss.str();
    }

    /**
     * @brief confirm the checksum of a transmission string
     * 
     * @param data the string to confirm
     */
    bool ProviderIMUNode::confirmChecksum(std::string& data)
    {
        std::string checksumData = data.substr(0, data.find("*", 0));
        uint8_t calculatedChecksum = calculateCheckSum(checksumData);
        uint8_t originalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
        return originalChecksum == calculatedChecksum;
    }

    /**
     * @brief tare the IMU
     * 
     * @param tare contains the tare service
     */
    bool ProviderIMUNode::tare(sonia_common::ImuTare::Request &tareRsq, sonia_common::ImuTare::Response &tareRsp)
    {
        serialConnection.transmit("$VNTAR*5F");
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU tare finished");
        return true;
    }

    /**
     * @brief set the disturbance for the accelometer and magnetometer
     * 
     * @param disturbance contains the disturbance service
     */
    bool ProviderIMUNode::disturbance(sonia_common::ImuDisturbance::Request &disturbanceRsq, sonia_common::ImuDisturbance::Response &disturbanceRsp)
    {
        std::string magneticDisturbanceCommand = std::string("$VNKMD,");
        std::string accelerationDisturbanceCommand = std::string("$VNKAD,");

        magneticDisturbanceCommand += disturbanceRsq.magnetometerDisturbance ? std::string("1") : std::string("0");
        accelerationDisturbanceCommand += disturbanceRsq.accelerationDisturbance ? std::string("1") : std::string("0");

        appendChecksum(magneticDisturbanceCommand);
        appendChecksum(accelerationDisturbanceCommand);

        serialConnection.transmit(magneticDisturbanceCommand);
        ros::Duration(0.1).sleep();
        serialConnection.transmit(accelerationDisturbanceCommand);
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU set disturbance settings finished");
        return true;
    }

    /**
     * @brief reset all the settings
     * 
     * @param settings contains the settings service
     */
    bool ProviderIMUNode::reset_settings(sonia_common::ImuResetSettings::Request &settingsRsq, sonia_common::ImuResetSettings::Response &settingsRsp)
    {
        std::string line;
        std::ifstream inputFile(configuration.getSettingsFile());

        if (!inputFile)
        {
            ROS_ERROR("IMU unable to open the file %s", configuration.getSettingsFile().c_str());
        }

        else
        {
            // factory reset
            serialConnection.transmit("$VNRFS*5F");
            ros::Duration(1).sleep();

            while (inputFile)
            {
                std::getline(inputFile, line);
                
                appendChecksum(line);
                serialConnection.transmit(line);
                ros::Duration(0.1).sleep();
            }
            serialConnection.transmit("$VNWNV*57");
            ros::Duration(0.1).sleep();
            serialConnection.transmit("$VNRST*4D");
            ros::Duration(0.1).sleep();
            ROS_INFO("IMU settings reset finished");
        }

        inputFile.close();
        return true;
    }

    /**
     * @brief send the imu information.
     * 
     */
    void ProviderIMUNode::send_information()
    {
        sonia_common::ImuInformation msg;
        std::string buffer = "";
        std::string parameter = "";

        serialConnection.flush();
        
        // filter status information
        serialConnection.transmit("$VNRRG,42*75");
        buffer = serialConnection.receive(1024);
        if(confirmChecksum(buffer))
        {
            std::stringstream ss(buffer);

            std::getline(ss, parameter, ',');
            std::getline(ss, parameter, ',');
            
            std::getline(ss, parameter, ',');
            msg.filterStatus = std::stoul(parameter, 0, 16);
            
            std::getline(ss, parameter, ',');
            msg.yawUncertainty = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.pitchUncertainty = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.rollUncertainty = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.gyroBiasUncertainty = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.magUncertainty = std::stof(parameter);

            std::getline(ss, parameter, '*');
            msg.accelUncertainty = std::stof(parameter);
        }

        // quaternion, magnetic, acceleration and angular rates information
        serialConnection.transmit("$VNRRG,15*77");
        buffer = serialConnection.receive(1024);
        if(confirmChecksum(buffer))
        {
             std::stringstream ss(buffer);

            std::getline(ss, parameter, ',');
            std::getline(ss, parameter, ',');

            std::getline(ss, parameter, ',');
            msg.quaternion.x = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.quaternion.y = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.quaternion.z = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.quaternion.w = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.magnetometer.x = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.magnetometer.y = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.magnetometer.z = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.acceleration.linear.x = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.acceleration.linear.y = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.acceleration.linear.z = std::stof(parameter);

            std::getline(ss, parameter, ',');
            msg.acceleration.angular.x = std::stof(parameter);
            
            std::getline(ss, parameter, ',');
            msg.acceleration.angular.y = std::stof(parameter);
            
            std::getline(ss, parameter, '*');
            msg.acceleration.angular.z = std::stof(parameter);

            publisher.publish(msg);
        }
    }
}
