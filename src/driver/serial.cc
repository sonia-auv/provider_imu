//
// Created by dev on 10/26/17.
//

#include "serial.h"
#include <fcntl.h>
#include <ros/ros.h>
#include <sys/ioctl.h>

Serial::Serial(std::string port)
{
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if(fd == -1)
    {
        ROS_ERROR("unable to connect to %s", port.c_str());
        ros::shutdown();
    }
    else
    {
        ROS_INFO("connection to %s succeed", port.c_str());
    }

    tcgetattr(fd, &options);

    // setup le baudrate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~(PARENB | PARODD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    //Input Flags
    options.c_iflag &= ~IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    //Local Flags
    options.c_lflag  = 0;

    //Output Flags
    options.c_oflag  = 0;


    tcsetattr(fd, TCSANOW, &options);
}

Serial::~Serial()
{
    close(fd);
}

std::string Serial::receive(size_t count)
{
    ROS_DEBUG("provider_imu receive data");
    char data[1024];
    data[0] = 0;

    read(fd, data, count);
    return std::string(data);
}

void Serial::readOnce(char* data, int offset)
{
    ROS_DEBUG("provider_imu receive Once");
    read(fd, (data+offset), 1);
}

void Serial::flush()
{
    ROS_DEBUG("provider_imu flush data");
    tcflush(fd,TCIOFLUSH);
}

ssize_t Serial::transmit(const std::string data)
{
    ROS_DEBUG("provider_imu transmit data");
    return write(fd, data.c_str(), data.size());
}
