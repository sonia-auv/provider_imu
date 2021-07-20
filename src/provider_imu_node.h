#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <sonia_common/ImuTare.h>
#include <ros/ros.h>

#include <mutex>
#include <condition_variable>
#include <thread>
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

        const char* REG_15 = "15";

	    Configuration configuration;

        uint8_t calculateCheckSum(std::string data);
        void appendChecksum(std::string& data);
        bool confirmChecksum(std::string& data);

        std::thread reader_thread;
        std::thread reg_15_thread;

        std::string register_15_str = "";
        std::mutex register_15_mutex;
        std::condition_variable register_15_cond;

        std::mutex writer_mutex;

        bool register_15_stop_thread = false;
        
        void send_information();
        void tare(sonia_common::ImuTare::Request &tareRsq, sonia_common::ImuTare::Response &tareRsp);
        void dvl_velocity(const geometry_msgs::Twist::ConstPtr& msg);
        void send_register_15();
        void reader();

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        ros::ServiceServer tare_srv;
        ros::Publisher publisher;
        ros::Subscriber dvl_subscriber;
    };
}

#endif
