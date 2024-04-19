#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations
from tamlib.utils import Logger
from geometry_msgs.msg import TransformStamped


class TestTfPublissher(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

    def broadcast_tf(self, odom_frame, target_frame, position, orientation):
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
        self.logdebug(transform)
        broadcaster.sendTransform(transform)

    def quaternion_norm(self, x, y, z, w):
        return math.sqrt(x**2 + y**2 + z**2 + w**2)

    def run(self, position, orientation):
        # ループの間隔を設定
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
        # for i in range(10):
            # ここで位置と姿勢を設定
            position = [3.45, 1.2, 0.05]  # 位置: x, y, z
            # 正面
            orientation = tf.transformations.quaternion_from_euler(3.14, -1.57, 0)
            # 上から
            # orientation = tf.transformations.quaternion_from_euler(3.14, 0, 0)
            # orientation = tf.transformations.quaternion_from_euler(3.14, 1.57, 0)
            # orientation = tf.transformations.quaternion_from_euler(0, 0, 0)
            # orientation = [0.0, 0.0, 0.0, 1.0]  # 姿勢: 四元数 qx, qy, qz, qw
            nolm = self.quaternion_norm(orientation[0], orientation[1], orientation[2], orientation[3])
            self.logdebug(nolm)
            self.broadcast_tf("odom", "target_frame", position, orientation)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_example')
    cls = TestTfPublissher()
    position = [1.9, 2.2, 0.02]  # 位置: x, y, z
    orientation = tf.transformations.quaternion_from_euler(3.14, -1.57, 0)

    cls.run(position=position, orientation=orientation)
