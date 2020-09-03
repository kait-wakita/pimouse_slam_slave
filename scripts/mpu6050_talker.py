#! /usr/bin/env python
# coding:utf-8

# talker for mpu6050 gyro
#     read the value of mpu6050, publish the valude to '/gyro'
#        (message type 'Twist' is used for easy communication with MATLAB)
#

import smbus
import math
import time
import rospy
from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

##############################################################
#アドレスの設定
##############################################################
DEV_ADDR = 0x68

ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

PWR_MGMT_1 = 0x6b # set clock
PWR_MGMT_2 = 0x6c



bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):#minus
        return -((65535 - val) + 1)
    else:#plus
        return val

def getAccel():
    # unit is 'g'
    x = read_word_sensor(ACCEL_XOUT)/16384.0
    y = read_word_sensor(ACCEL_YOUT)/16384.0
    z = read_word_sensor(ACCEL_ZOUT)/16384.0
    return [x, y, z]

def getGyro():
    # unit is 'deg/sec'
    gx = read_word_sensor(GYRO_XOUT)/131.0
    gy = read_word_sensor(GYRO_YOUT)/131.0
    gz = read_word_sensor(GYRO_ZOUT)/131.0
    return [gx, gy, gz]

def calcEuler(x,y,z):
    theta = math.atan( x / math.sqrt(y*y + z*z) )
    psi = math.atan( y / math.sqrt(x*x + z*z) )
    phi = math.atan( math.sqrt( x*x + y*y ) / z)
    return [theta, psi, phi]


if __name__ == "__main__":
    rospy.init_node("gyro_talker")
    gyro_pub = rospy.Publisher('gyro',Twist,queue_size=1)
    twist = Twist()

    rate = rospy.Rate(40)

    while not rospy.is_shutdown():
        ax, ay, az = getAccel()
        gx, gy, gz = getGyro()
        #roll, pitch, yaw = calcEuler(ax,ay,az)


        twist.linear.x = ax
        twist.linear.y = ay
        twist.linear.z = az
        twist.angular.x = gx
        twist.angular.y = gy
        twist.angular.z = gz + 0.3  # for mouse002
        
        
        gyro_pub.publish(twist)

        # print "ACC[%5.2f,"%ax, "%5.2f,"%ay, "%5.2f]"%az, "   GYRO[%5.0f,"%gx, "%5.0f,"%gy, "%5.0f]"%gz

        rate.sleep()
   
