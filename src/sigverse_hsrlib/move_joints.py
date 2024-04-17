#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from tamlib.utils import Logger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveJoints(Logger):
    def __init__(self):
        Logger.__init__(self)
        self.head_pub = rospy.Publisher('/hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.base_pub = rospy.Publisher('/hsrb/omni_base_controller/command', JointTrajectory, queue_size=1)        
        self.gripper_pub = rospy.Publisher('/hsrb/gripper_controller/command', JointTrajectory, queue_size=1)

    def move_head(self, pan: float, tilt: float) -> bool:
        """頭部のジョイント角を指定して移動
        Args:
            pan(float): 横向きの角度（rad）
            tilt(float): 縦向きの角度(rad)
        Returns:
            bool: 移動に成功したか
        """
        try:
            traj = JointTrajectory()
            traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
            p = JointTrajectoryPoint()
            p.positions = [pan, tilt]
            p.velocities = [0, 0]
            p.time_from_start = rospy.Duration(1)
            traj.points = [p]

            self.head_pub.publish(traj)
            self.loginfo("Send move HEAD command")
            return True
        except Exception as e:
            self.logwarn(e)
            return False

    def move_arm(self, arm_lift: float, arm_flex: float, arm_roll: float, wrist_flex: float, wrist_roll: float) -> bool:
        """腕のジョイント角を指定して移動
        Args:

        Returns:

        """
        traj = JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = JointTrajectoryPoint()
        p.positions = [arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll]
        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(1)
        traj.points = [p]

        self.arm_pub.publish(traj)
        self.loginfo("Send move ARM command")

    def move_base_joint(self, x, y, yaw, duration=5):
        """ベースの移動
        """
        traj = JointTrajectory()
        traj.header.frame_id = "base_link"
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = JointTrajectoryPoint()
        p.positions = [x, y, yaw]
        p.velocities = [0, 0, 0]
        p.time_from_start = rospy.Duration(duration)
        traj.points = [p]

        self.base_pub.publish(traj)

    def gripper(self, angle: float):
        """
        Args:
            angle(float): ハンドの開閉角を指定
                angle = 0 -> close
                angle =3.14 -> open
        Returns:
            bool: 成功したかどうか
        """
        traj = JointTrajectory()
        traj.joint_names = ["hand_motor_joint"]
        p = JointTrajectoryPoint()
        p.positions = [angle]
        p.time_from_start = rospy.Duration(1)
        traj.points = [p]

        self.gripper_pub.publish(traj)
        self.loginfo("Send move GRIPPER command")

    def neutral(self):
        """ニュートラルの姿勢に戻る"""
        self.move_arm(0.0, 0.0, 0.0, -1.57, 0.0)
        self.move_head(0.0, 0.0)
        self.loginfo("Send NEUTRAL command")

    def go(self):
        """移動姿勢になる"""
        self.move_arm(0.0, 0.0, -1.57, -1.57, 0.0)
        self.move_head(0.0, 0.0)
        self.loginfo("Send GO command")

    # def detection(self):
    #     self.go()
    #     self.move_head(0.0,-1.0)
    #     sleep(2)

    # def grasp_from_front(self,al):
    #     self.move_arm(al,-3.0,0.0,1.5,0.0)
    #     #self.move_arm(0.0,-3.0,0.0,1.5,0.0)

    # def horizontal(self,al):
    #     self.move_arm(al,-1.57,0.0,0.0,0.0)


if __name__ == "__main__":
    rospy.init_node('test_move_joints')
    mj = MoveJoints()
    mj.neutral()
