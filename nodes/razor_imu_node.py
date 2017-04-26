#!/usr/bin/env python

import rospy
from razor_imu_m0.msg import RazorIMURaw
from sensor_msgs.msg import Imu
import tf

imu_msg = Imu()
imu_msg.header.frame_id = 'imu'

imu_msg.orientation_covariance = [
    0.0025, 0, 0,
    0, 0.0025, 0,
    0, 0, 0.0025
]

imu_msg.angular_velocity_covariance = [
    0.02, 0, 0,
    0, 0.02, 0,
    0, 0, 0.02
]

imu_msg.linear_acceleration_covariance = [
    0.04, 0, 0,
    0, 0.04, 0,
    0, 0, 0.04
]

pub = rospy.Publisher('imu', Imu, queue_size=1)


def raw_imu_cb(msg):
    assert (isinstance(msg, RazorIMURaw))
    imu_msg.header.stamp = msg.header.stamp
    imu_msg.header.seq = msg.header.seq
    imu_msg.orientation.x = msg.quat[0]
    imu_msg.orientation.y = msg.quat[1]
    imu_msg.orientation.z = msg.quat[2]
    imu_msg.orientation.w = msg.quat[3]

    imu_msg.linear_acceleration.x = msg.acc[0]
    imu_msg.linear_acceleration.y = msg.acc[1]
    imu_msg.linear_acceleration.z = msg.acc[2]

    imu_msg.angular_velocity.x = msg.gyr[0]
    imu_msg.angular_velocity.y = msg.gyr[1]
    imu_msg.angular_velocity.z = msg.gyr[2]
    pub.publish(imu_msg)


rospy.init_node("razor_imu_node", anonymous=True)
sub = rospy.Subscriber('razor_imu_raw', RazorIMURaw, raw_imu_cb)

tf_broadcast = tf.TransformBroadcaster()
r = rospy.Rate(100)
while not rospy.is_shutdown():
    # tf_broadcast.sendTransform((0, 0, 0),
    #                            (imu_msg.orientation.x, imu_msg.orientation.y,
    #                             imu_msg.orientation.z, imu_msg.orientation.w),
    #                            rospy.Time.now(),
    #                            'imu_link',
    #                            "world")
    r.sleep()
