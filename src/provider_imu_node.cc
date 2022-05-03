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
        // Publisher
        publisher = nh->advertise<sensor_msgs::Imu>("/provider_imu/imu_info", 100);

        // Subscribers
        dvl_subscriber = nh->subscribe<geometry_msgs::Twist>("/proc_nav/dvl_velocity", 100, 
            &ProviderIMUNode::dvl_velocity, this);
        vpe_basic_control = nh->subscribe<std_msgs::UInt8MultiArray>("/provider_imu/vpe_basic_control", 10, 
            &ProviderIMUNode::vpe_basic_control_callback, this);
        magnetometer_calibration_control = nh->subscribe<std_msgs::UInt8MultiArray>("/provider_imu/magnetometer_calibration_control", 10, 
            &ProviderIMUNode::magnetometer_calibration_control_callback, this);
        delta_theta_delta_velocity = nh->subscribe<std_msgs::UInt8MultiArray>("/provider_imu/delta_theta_delta_velocity", 10,
            &ProviderIMUNode::delta_theta_delta_velocity_callback, this);
        imu_filtering_configuration = nh->subscribe<std_msgs::UInt8MultiArray>("/provider_imu/imu_filtering_configuration", 10,
            &ProviderIMUNode::imu_filtering_configuration_callback, this);

        // Service
        tare_srv = nh->advertiseService("/provider_imu/tare", &ProviderIMUNode::tare, this);
        reset_srv = nh->advertiseService("/provider_imu/reset", &ProviderIMUNode::reset, this);
        factory_reset_srv = nh->advertiseService("/provider_imu/factory_reset", &ProviderIMUNode::factory_reset, this);
        magnetic_disturbance_srv = nh->advertiseService("/provider_imu/magnetic_disturbance", &ProviderIMUNode::magnetic_disturbance, this);
        acceleration_disturbance_srv = nh->advertiseService("/provider_imu/acceleration_disturbance", &ProviderIMUNode::acceleration_disturbance, this);
        velocity_compensation_srv = nh->advertiseService("/provider_imu/velocity_compensation", &ProviderIMUNode::velocity_compensation, this);
        asyn_output_pause_srv = nh->advertiseService("/provider_imu/pause", &ProviderIMUNode::asyn_output_pause, this);

        // Threads
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
    bool ProviderIMUNode::tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp)
    {
        serialConnection.transmit("$VNTAR*5F\n");
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU tare finished");
        return true;
    }

    bool ProviderIMUNode::reset(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp)
    {
        serialConnection.transmit("$VNRST*4D\n");
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU reset finished");
        return true;
    }

    bool ProviderIMUNode::factory_reset(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp)
    {
        serialConnection.transmit("$VNRFS*5F\n");
        ros::Duration(0.1).sleep();

        ROS_INFO("IMU factory reset finished");
        return true;
    }

    bool ProviderIMUNode::magnetic_disturbance(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp)
    {
        std::stringstream ss;

        writer_mutex.lock();
        ss << "$VNKMD," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.1).sleep();

        writer_mutex.unlock();
        return true;
    }

    bool ProviderIMUNode::acceleration_disturbance(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp)
    {
        std::stringstream ss;

        writer_mutex.lock();
        ss << "$VNKAD," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.1).sleep();
        
        writer_mutex.unlock();
        return true;
    }

     bool ProviderIMUNode::velocity_compensation(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp)
    {
        std::stringstream ss;

        writer_mutex.lock();
        ss << "$VNWNV,51," << std::to_string((uint8_t)rsq.data) << ",0.1,0.01";
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.1).sleep();
        
        writer_mutex.unlock();
        return true;    
    }

    bool ProviderIMUNode::asyn_output_pause(std_srvs::SetBool::Request &rsq, std_srvs::SetBool::Response &rsp)
    {
        std::stringstream ss;

        writer_mutex.lock();
        ss << "$VNASY," << std::to_string((uint8_t)rsq.data);
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.1).sleep();
        
        writer_mutex.unlock();
        return true;
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

    void ProviderIMUNode::asyn_data_frequency_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
        std::stringstream ss;

        writer_mutex.lock();

        ss << "$VNWNV,07," << std::to_string(msg->data);
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.5).sleep();

        writer_mutex.unlock();        
    }

    void ProviderIMUNode::vpe_basic_control_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {
        std::stringstream ss;

        writer_mutex.lock();

        ss << "$VNWNV,35," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2)) 
            << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.5).sleep();

        writer_mutex.unlock();
    }

    void ProviderIMUNode::magnetometer_calibration_control_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {
        std::stringstream ss;

        writer_mutex.lock();

        ss << "$VNWNV,44," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2));
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.5).sleep();

        writer_mutex.unlock();
    }

    void ProviderIMUNode::delta_theta_delta_velocity_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {
        std::stringstream ss;

        writer_mutex.lock();

        ss << "$VNWNV,82," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2)) 
            << "," << std::to_string(msg->data.at(3));
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.5).sleep();

        writer_mutex.unlock();        
    }

    void ProviderIMUNode::imu_filtering_configuration_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {
        std::stringstream ss;

        writer_mutex.lock();

        ss << "$VNWNV,85," << std::to_string(msg->data.at(0)) << "," << std::to_string(msg->data.at(1)) << "," << std::to_string(msg->data.at(2)) 
            << "," << std::to_string(msg->data.at(4)) << "," << std::to_string(msg->data.at(5)) << "," << std::to_string(msg->data.at(6))
            << "," << std::to_string(msg->data.at(7)) << "," << std::to_string(msg->data.at(8)) << "," << std::to_string(msg->data.at(9));
        std::string send_data = ss.str();
        appendChecksum(send_data);

        serialConnection.transmit(send_data);
        ros::Duration(0.5).sleep();

        writer_mutex.unlock();        
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
            else
            {
                ROS_INFO_STREAM(buffer);
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
}
