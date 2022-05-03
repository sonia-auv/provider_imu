#ifndef INTERFACE_RS485_NODE_H
#define INTERFACE_RS485_NODE_H

#include "driver/serial.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
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

        ros::NodeHandlePtr nh;
        Configuration configuration;
        Serial serialConnection;
	    
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
        // bool indoormode(std_srvs::SetBool::Request &indoormodeRsq, std_srvs::SetBool::Response &indoormodeRsp);
        bool tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);
        bool reset(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);
        bool factory_reset(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);
        bool magnetic_disturbance(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp);
        bool acceleration_disturbance(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp);
        bool velocity_compensation(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp);
        bool asyn_output_pause(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp);

        void dvl_velocity(const geometry_msgs::Twist::ConstPtr& msg);
        void asyn_data_frequency_callback(const std_msgs::UInt8::ConstPtr& msg);
        void vpe_basic_control_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
        void magnetometer_calibration_control_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg);

        void send_register_15();
        void send_register_239();
        void send_register_240();
        void send_error();
        void reader();

        ros::ServiceServer tare_srv;
        ros::ServiceServer reset_srv;
        ros::ServiceServer factory_reset_srv;
        ros::ServiceServer magnetic_disturbance_srv;
        ros::ServiceServer acceleration_disturbance_srv;
        ros::ServiceServer velocity_compensation_srv;
        ros::ServiceServer asyn_output_pause_srv;

        ros::Publisher publisher;

        ros::Subscriber dvl_subscriber;
        ros::Subscriber asyn_data_frequency;
        ros::Subscriber vpe_basic_control;
        ros::Subscriber magnetometer_calibration_control;
    };
}

#endif
