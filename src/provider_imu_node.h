#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"

#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
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

        const char* REG_15 = "QMR";
        const char* REG_239 = "YBA";
        const char* REG_240 = "YIA";

        const char* ERR_STR = "ERR";

	    Configuration configuration;

        std::thread reader_thread;
        std::thread register_15_thread;
        std::thread register_239_thread;
        std::thread register_240_thread;
        std::thread error_thread;

        std::string register_15_str = "";
        std::mutex register_15_mutex;
        std::condition_variable register_15_cond;

        std::string register_239_str = "";
        std::mutex register_239_mutex;
        std::condition_variable register_239_cond;

        std::string register_240_str = "";
        std::mutex register_240_mutex;
        std::condition_variable register_240_cond;

        std::string error_str = "";
        std::mutex error_mutex;
        std::condition_variable error_cond;

        std::mutex writer_mutex;

        bool register_15_stop_thread = false;
        bool register_239_stop_thread = false;
        bool register_240_stop_thread = false;
        bool error_stop_thread = false;
        bool reader_stop_thread = false;

        uint8_t calculateCheckSum(std::string data);
        void appendChecksum(std::string& data);
        bool confirmChecksum(std::string& data);
        
        void send_information();
        bool tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);
        void dvl_velocity(const geometry_msgs::Twist::ConstPtr& msg);
        void send_register_15();
        void send_register_239();
        void send_register_240();
        void send_error();
        void reader();

        ros::NodeHandlePtr nh;
        Serial serialConnection;

        ros::ServiceServer tare_srv;
        ros::Publisher publisher;
        ros::Subscriber dvl_subscriber;
    };
}

#endif
