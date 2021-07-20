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
        publisher = nh->advertise<sensor_msgs::Imu>("/provider_imu/imu_info", 100);
        dvl_subscriber = nh->subscribe<geometry_msgs::Twist>("/proc_nav/dvl_velocity", 100, &ProviderIMUNode::dvl_velocity, this);
        tare_srv = nh->advertiseService("/provider_imu/tare", &ProviderIMUNode::tare, this);

        reader_thread = std::thread(std::bind(&ProviderIMUNode::reader, this));
    }

    // node destructor
    ProviderIMUNode::~ProviderIMUNode()
    {

    }

    // node spin
    void ProviderIMUNode::Spin()
    {
        ros::Rate r(200);
        while(ros::ok())
        {
            ros::spinOnce();
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
        serialConnection.transmit("$VNTAR*5F\n");
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU tare finished");
        return true;
    }

    /**
     * @brief read the IMU serial port
     * 
     */
    void ProviderIMUNode::reader()
    {
        char buffer[1024];

        // find the message beginning
        while(buffer[0] != '$')
        {
            serialConnection.readOnce(buffer, 0);
        }

        for(int i = 1; buffer[i] != '\n'; ++i)
        {
            serialConnection.readOnce(buffer, i);
        }

        if(!strncmp(&buffer[6], REG_15, 2))
        {
            std::unique_lock<std::mutex> mlock(register_15_mutex);
            register_15_str = std::string(buffer);
            register_15_cond.notify_one();
        }
    }

    /**
     * @brief send the imu information.
     * 
     */
    void ProviderIMUNode::send_register_15()
    {
        while(!register_15_stop_thread)
        {
            sensor_msgs::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(register_15_mutex);
            register_15_cond.wait(mlock);

            // quaternion, magnetic, acceleration and angular rates information
            if((!register_15_str.empty()) && confirmChecksum(register_15_str))
            {
                std::stringstream ss(register_15_str);

                std::getline(ss, parameter, ',');
                std::getline(ss, parameter, ',');

                std::getline(ss, parameter, ',');
                msg.orientation.x = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.orientation.y = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.orientation.z = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.orientation.w = std::stof(parameter);

                std::getline(ss, parameter, ',');
                //msg.magnetometer.x = std::stof(parameter);

                std::getline(ss, parameter, ',');
                //msg.magnetometer.y = std::stof(parameter);

                std::getline(ss, parameter, ',');
                //msg.magnetometer.z = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.linear_acceleration.x = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.linear_acceleration.y = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.linear_acceleration.z = std::stof(parameter);

                std::getline(ss, parameter, ',');
                msg.angular_velocity.x = std::stof(parameter);
                
                std::getline(ss, parameter, ',');
                msg.angular_velocity.y = std::stof(parameter);
                
                std::getline(ss, parameter, '*');
                msg.angular_velocity.z = std::stof(parameter);
                publisher.publish(msg);
            }
        }
    }

    void ProviderIMUNode::dvl_velocity(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::stringstream ss;

        std::unique_lock<std::mutex> mlock(writer_mutex);
        ss << "$VNWRG,50," << std::to_string((float)msg->linear.x) << "," << std::to_string((float)msg->linear.y) << "," << std::to_string((float)msg->linear.z);
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
    }
}
