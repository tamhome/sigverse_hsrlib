#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
import tf, tf2_ros
import tf_conversions
import numpy as np
from time import sleep
from tamlib.utils import Logger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped, Twist


class MoveJoints(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")
        self.head_pub = rospy.Publisher('/hsrb/head_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.arm_pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', JointTrajectory, queue_size=1)
        self.base_pub = rospy.Publisher('/hsrb/omni_base_controller/command', JointTrajectory, queue_size=1)        
        self.gripper_pub = rospy.Publisher('/hsrb/gripper_controller/command', JointTrajectory, queue_size=1)
        self.joint_topic_name = rospy.get_param("~joint_topic_name", "/hsrb/joint_states")

        self.sub_joint_state = rospy.Subscriber(self.joint_topic_name, JointState, self._joint_state_callback)
        self.pub_base_rot = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

        self.broadcaster = tf2_ros.TransformBroadcaster()
        # Create a tf listener
        self.listener = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.listener)

        # HSRBの現在のjointを指定
        self.arm_lift_joint_pos1_ = None
        self.arm_lift_joint_pos2_ = None
        self.arm_roll_joint_pos_ = None
        self.arm_flex_joint_pos_ = None
        self.wrist_flex_joint_pos_ = None
        self.wrist_roll_joint_pos_ = None
        self.head_pan_joint_pos_ = None
        self.head_tilt_joint_pos_ = None
        self.hand_l_joint_pos_ = None
        self.hand_r_joint_pos_ = None

    def _joint_state_callback(self, joint_state):
        for i in range(len(joint_state.name)):
            if joint_state.name[i] == "arm_lift_joint":
                self.arm_lift_joint_pos2_ = self.arm_lift_joint_pos1_
                self.arm_lift_joint_pos1_ = joint_state.position[i]
            elif joint_state.name[i] == "arm_flex_joint":
                self.arm_flex_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "arm_roll_joint":
                self.arm_roll_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "wrist_flex_joint":
                self.wrist_flex_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "wrist_roll_joint":
                self.wrist_roll_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "head_pan_joint":
                self.head_pan_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "head_tilt_joint":
                self.head_tilt_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "hand_l_spring_proximal_joint":
                self.hand_l_joint_pos_ = joint_state.position[i]
            elif joint_state.name[i] == "hand_r_spring_proximal_joint":
                self.hand_r_joint_pos_ = joint_state.position[i]

    def get_current_joint(self):
        """現在のHSRのジョイント角を取得
        Args:
            None
        Returns:
            dict
            sample {"arm_lift_joint", 0.2...}
        """
        current_joint = {}
        current_joint["arm_lift_joint"] = self.arm_lift_joint_pos1_
        current_joint["arm_flex_joint"] = self.arm_flex_joint_pos_
        current_joint["arm_roll_joint"] = self.arm_roll_joint_pos_
        current_joint["wrist_flex_joint"] = self.wrist_flex_joint_pos_
        current_joint["wrist_roll_joint"] = self.wrist_roll_joint_pos_
        current_joint["head_pan_joint"] = self.head_pan_joint_pos_
        current_joint["head_tilt_joint"] = self.head_tilt_joint_pos_
        current_joint["hand_l_spring_proximal_joint"] = self.hand_l_joint_pos_
        current_joint["hand_r_spring_proximal_joint"] = self.hand_r_joint_pos_

        return current_joint

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

    def move_arm_by_line(self, movement_value: float, target_joint="arm_lift_joint", duration_sec=20) -> bool:
        """腕の移動量を指定して移動
        Args:
            movement_value(float): 移動量
            target_joint
        """
        traj = JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = JointTrajectoryPoint()

        if target_joint == "arm_lift_joint":
            p.positions = [self.arm_lift_joint_pos1_ + movement_value, self.arm_flex_joint_pos_, self.arm_roll_joint_pos_, self.wrist_flex_joint_pos_, self.wrist_roll_joint_pos_]
        elif target_joint == "arm_flex_joint":
            p.positions = [self.arm_lift_joint_pos1_, self.arm_flex_joint_pos_ + movement_value, self.arm_roll_joint_pos_, self.wrist_flex_joint_pos_, self.wrist_roll_joint_pos_]
        elif target_joint == "arm_roll_joint":
            p.positions = [self.arm_lift_joint_pos1_, self.arm_flex_joint_pos_, self.arm_roll_joint_pos_ + movement_value, self.wrist_flex_joint_pos_, self.wrist_roll_joint_pos_]
        elif target_joint == "wrist_flex_joint":
            p.positions = [self.arm_lift_joint_pos1_, self.arm_flex_joint_pos_, self.arm_roll_joint_pos_, self.wrist_flex_joint_pos_ + movement_value, self.wrist_roll_joint_pos_]
        elif target_joint == "wrist_roll_joint":
            p.positions = [self.arm_lift_joint_pos1_, self.arm_flex_joint_pos_, self.arm_roll_joint_pos_, self.wrist_flex_joint_pos_, self.wrist_roll_joint_pos_ + movement_value]
        else:
            self.logwarn("指定されたジョイントが正しくありません．")
            self.logwarn("指定できるジョイントは以下のとおりです [arm_lift_joint, arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint]")

        p.velocities = [0, 0, 0, 0, 0]
        p.time_from_start = rospy.Duration(1)
        traj.points = [p]
        self.loginfo(f"Traget pose is \n {p.positions}")

        self.arm_pub.publish(traj)
        self.loginfo("send move ARM by line command.")
        return True

    def move_arm_by_pose(self, arm_lift: float, arm_flex: float, arm_roll: float, wrist_flex: float, wrist_roll: float) -> bool:
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
        self.loginfo("Send move base command")

    def gripper(self, angle: float):
        """
        Args:
            angle(float): ハンドの開閉角を指定
                angle = 0 -> close
                angle = 3.14 -> open
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
        self.move_arm_by_pose(0.0, 0.0, 0.0, -1.57, 0.0)
        self.move_head(0.0, 0.0)
        self.loginfo("Send NEUTRAL command")

    def go(self):
        """移動姿勢になる"""
        self.move_arm_by_pose(0.0, 0.0, -1.57, -1.57, 0.0)
        self.move_head(0.0, 0.0)
        self.loginfo("Send GO command")

    def gaze_point(self, target_pose: Pose, frame: str = "odom", timeout=10) -> bool:
        """台車を回転させることによるgaze_point
        Args:
            target_pose(Pose): 目標位置
            frame(str): 目標位置のベースフレーム
                odom以外では動かないので要修正
            timeout(int): タイムアウト
                defaults to 10
        Returns:
            bool: 移動に成功したかどうか
        """
        self.loginfo("start gaze point")
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if current_time - start_time > timeout_duration:
                twist_msg = Twist()
                twist_msg.angular.z = 0
                self.pub_base_rot.publish(twist_msg)

                self.logwarn("cannot reach goal")
                return False

            target_position = target_pose.position
            transform = TransformStamped()
            # Set the translation and rotation of the transform
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = frame
            transform.child_frame_id = "target_position"
            transform.transform.translation.x = target_position.x
            transform.transform.translation.y = target_position.y
            transform.transform.translation.z = target_position.z
            transform.transform.rotation.x = 0
            transform.transform.rotation.y = 0
            transform.transform.rotation.z = 0
            transform.transform.rotation.w = 1

            # Send the transformation
            self.broadcaster.sendTransform(transform)

            # Check if the transform is possible
            if not self.listener.can_transform("base_footprint", "target_position", rospy.Time(0)):
                self.logwarn("cannot get transform tf")
                continue

            try:
                # Lookup the transformation
                trans = self.listener.lookup_transform("base_footprint", "target_position", rospy.Time(0))
            except tf2_ros.TransformException as e:
                self.logerr("TF lookup error: %s" % str(e))
                continue

            # Compute the angle
            angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            # Prepare the twist message
            twist_msg = Twist()

            # Publish the twist message based on the angle
            if -2.5 * math.pi / 180 < angle < 2.5 * math.pi / 180:
                self.loginfo("goal reached")
                twist_msg.angular.z = 0
                self.pub_base_rot.publish(twist_msg)
                return True
            else:
                if angle <= -2.5 * math.pi / 180:
                    twist_msg.angular.z = -10 * math.pi / 180
                    self.pub_base_rot.publish(twist_msg)

                elif angle >= 2.5 * math.pi / 180:
                    twist_msg.angular.z = 10 * math.pi / 180
                    self.pub_base_rot.publish(twist_msg)

            self.logdebug(angle)
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node('test_move_joints')
    mj = MoveJoints()
    mj.neutral()
