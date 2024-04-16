#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

def broadcast_tf(odom_frame, target_frame, position, orientation):
    broadcaster = tf2_ros.TransformBroadcaster()
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = odom_frame
    transform.child_frame_id = target_frame
    transform.transform.translation.x = position[0]
    transform.transform.translation.y = position[1]
    transform.transform.translation.z = position[2]
    transform.transform.rotation.x = orientation[0]
    transform.transform.rotation.y = orientation[1]
    transform.transform.rotation.z = orientation[2]
    transform.transform.rotation.w = orientation[3]

    # 発行するトランスフォームを送信
    broadcaster.sendTransform(transform)

def main():
    rospy.init_node('tf2_broadcaster_example')

    # ループの間隔を設定
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # ここで位置と姿勢を設定
        position = [0.0, 0.40, 0.9]  # 位置: x, y, z
        orientation = [0.0, 0.0, 0.0, 1.0]  # 姿勢: 四元数 qx, qy, qz, qw
        broadcast_tf("odom", "target_frame", position, orientation)
        rate.sleep()

if __name__ == '__main__':
    main()