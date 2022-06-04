#!/usr/bin/env python3
import rospy
import numpy
import struct
import os

from sonia_common.msg import *
from sensor_msgs.msg import *

def sim_imu():
    rospy.init_node('provider_imu_simulation', anonymous=True)
    pub = rospy.Publisher('/provider_imu/imu_info', Imu, queue_size=100)
    rate = rospy.Rate(1)  # 10hz
    msg = Imu()
    while not rospy.is_shutdown():
        
        msg.linear_acceleration.x = numpy.random.uniform(0, 1)
        msg.linear_acceleration.y = numpy.random.uniform(0, 1)
        msg.linear_acceleration.z = numpy.random.uniform(0, 1)
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        auv = os.getenv('LOCAL_AUV',"AUV7")
        print("LOCAL_AUV set on : " + auv)
        sim_imu()
    except rospy.ROSInterruptException:
        pass

