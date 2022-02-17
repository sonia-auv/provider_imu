#include "provider_imu_node.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>

#define BUFFER_SIZE 4096

namespace provider_IMU
{

    // node Construtor
    ProviderIMUNode::ProviderIMUNode(const ros::NodeHandlePtr &_nh)
    : nh(_nh), configuration(_nh), serialConnection(configuration.getTtyPort())
    {
        publisher = nh->advertise<sensor_msgs::Imu>("/provider_imu/imu_info", 100);
        dvl_subscriber = nh->subscribe<geometry_msgs::Twist>("/proc_nav/dvl_velocity", 100, &ProviderIMUNode::dvl_velocity, this);
        tare_srv = nh->advertiseService("/provider_imu/tare", &ProviderIMUNode::tare, this);
        indoor_srv = nh->advertiseService("/provider_imu/indoor", &ProviderIMUNode::indoormode, this);

        reader_thread = std::thread(std::bind(&ProviderIMUNode::reader, this));
        error_thread = std::thread(std::bind(&ProviderIMUNode::send_error, this));
        register_15_thread = std::thread(std::bind(&ProviderIMUNode::send_register_15, this));
        register_239_thread = std::thread(std::bind(&ProviderIMUNode::send_register_239, this));
        register_240_thread = std::thread(std::bind(&ProviderIMUNode::send_register_240, this));
    }

    // node destructor
    ProviderIMUNode::~ProviderIMUNode()
    {
        register_15_stop_thread = true;
        register_239_stop_thread = true;
        register_240_stop_thread = true;
        error_stop_thread = true;
        reader_stop_thread = true;
    }

    // node spin
    void ProviderIMUNode::Spin()
    {
        ros::Rate r(20);
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
        try
        {
            std::string checksumData = data.substr(0, data.find("*", 0));
            uint8_t calculatedChecksum = calculateCheckSum(checksumData);
            uint8_t originalChecksum = std::stoi(data.substr(data.find("*", 0)+1, 2), nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            ROS_INFO("imu: bad packet checksum");
            return false;
        }
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
     * @brief Switch the transmission mode to indoor or outdoor
     * 
     * @param std_srvs contains the std_srvs SetBool service
     */
    bool ProviderIMUNode::indoormode(std_srvs::SetBool::Request &indoormodeRsq, std_srvs::SetBool::Response &indoormodeRsp)
    {
            
        if (indoormodeRsq.data) {

            serialConnection.transmit("$VNTAR*5F\n"); //Modifier le string
            ros::Duration(0.1).sleep();

            indoormodeRsp.message = "IMU in indoor mode";
            ROS_INFO_STREAM("IMU in indoor mode");

        } else {

            serialConnection.transmit("$VNTAR*5F\n"); //Modifier le string
            ros::Duration(0.1).sleep();

            indoormodeRsp.message = "IMU in outdoor mode";
            ROS_INFO_STREAM("IMU in outdoor mode");
            
        }
        return true;
    }

    /**
     * @brief read the IMU serial port
     * 
     */
    void ProviderIMUNode::reader()
    {
        ROS_INFO("reader thread started");
        char buffer[BUFFER_SIZE];

        while(!reader_stop_thread)
        {
            // find the message beginning
            do
            {
                serialConnection.readOnce(buffer, 0);
            }while(buffer[0] != '$');

            int i;

            for(i = 1; buffer[i-1] != '\n' && i < BUFFER_SIZE; i++)
            {
                serialConnection.readOnce(buffer, i);
            }

            if(i >= BUFFER_SIZE)
            {
                continue;
            }


            buffer[i] = 0;

            if(!strncmp(&buffer[3], REG_15, 3))
            {
                std::unique_lock<std::mutex> mlock(register_15_mutex);
                register_15_str = std::string(buffer);
                register_15_cond.notify_one();
            }
            else if(!strncmp(&buffer[3], REG_239, 3))
            {
                std::unique_lock<std::mutex> mlock(register_239_mutex);
                register_239_str = std::string(buffer);
                register_239_cond.notify_one();
            }
            else if(!strncmp(&buffer[3], REG_240, 3))
            {
                std::unique_lock<std::mutex> mlock(register_240_mutex);
                register_240_str = std::string(buffer);
                register_240_cond.notify_one();
            }
            else if(!strncmp(&buffer[3], ERR_STR, 3))
            {
                std::unique_lock<std::mutex> mlock(error_mutex);
                error_str = std::string(buffer);
                error_cond.notify_one();
            }
        }        
    }

    /**
     * @brief thread to send the error values.
     * 
     */
    void ProviderIMUNode::send_error()
    {
        ROS_INFO("error thread started");
        while(!error_stop_thread)
        {
            sensor_msgs::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(error_mutex);
            error_cond.wait(mlock);

            try
            {
                // quaternion, magnetic, acceleration and angular rates information
                if((!error_str.empty()) && confirmChecksum(error_str))
                {
                    std::stringstream ss(error_str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, '*');
                    
                    ROS_INFO("error: %s", parameter.c_str());
                }   
            }
            catch(...)
            {
                ROS_INFO("imu: bad packet error");
            }
        }
    }

    /**
     * @brief thread to send the register 15 values.
     * 
     */
    void ProviderIMUNode::send_register_15()
    {
        ROS_INFO("register 15 thread started");
        while(!register_15_stop_thread)
        {
            sensor_msgs::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(register_15_mutex);
            register_15_cond.wait(mlock);

            try
            {
                // quaternion, magnetic, acceleration and angular rates information
                if((!register_15_str.empty()) && confirmChecksum(register_15_str))
                {
                    std::stringstream ss(register_15_str);

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
            catch(...)
            {
                ROS_INFO("imu: bad packet register 15");
            }
        }
    }

    

    /**
     * @brief thread to send the register 239 values.
     * 
     */
    void ProviderIMUNode::send_register_239()
    {
        ROS_INFO("register 239 thread started");
        while(!register_239_stop_thread)
        {
            sensor_msgs::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(register_239_mutex);
            register_239_cond.wait(mlock);

            try
            {
                // quaternion, magnetic, acceleration and angular rates information
                if((!register_239_str.empty()) && confirmChecksum(register_239_str))
                {
                    std::stringstream ss(register_239_str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');
                    msg.orientation.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.z = std::stof(parameter);

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
            catch(...)
            {
                ROS_DEBUG("imu: bad packet");
            }
        }
    }

    /**
     * @brief thread to send the register 240 values.
     * 
     */
    void ProviderIMUNode::send_register_240()
    {
        ROS_INFO("register 240 thread started");
        while(!register_240_stop_thread)
        {
            sensor_msgs::Imu msg;
            std::string parameter = "";

            std::unique_lock<std::mutex> mlock(register_240_mutex);
            register_240_cond.wait(mlock);

            try
            {
                // quaternion, magnetic, acceleration and angular rates information
                if((!register_240_str.empty()) && confirmChecksum(register_240_str))
                {
                    std::stringstream ss(register_240_str);

                    std::getline(ss, parameter, ',');

                    std::getline(ss, parameter, ',');
                    msg.orientation.x = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.y = std::stof(parameter);

                    std::getline(ss, parameter, ',');
                    msg.orientation.z = std::stof(parameter);

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
            catch(...)
            {
                ROS_DEBUG("imu: bad packet");
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
