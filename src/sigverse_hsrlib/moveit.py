#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
# import moveit_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from typing import Tuple, List
from tamlib.utils import Logger


from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import DisplayTrajectory


class HSRBMoveIt(Logger):

    def __init__(self):
        Logger.__init__(self, loglevel="DEBUG")

        moveit_commander.roscpp_initialize(sys.argv)

        # MoveItのインターフェースとロボットの情報を初期化
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        arm = moveit_commander.MoveGroupCommander("arm", wait_for_servers=0.0)
        base = moveit_commander.MoveGroupCommander("base", wait_for_servers=0.0)
        gripper = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=0.0)
        head = moveit_commander.MoveGroupCommander("head", wait_for_servers=0.0)
        whole_body = moveit_commander.MoveGroupCommander("whole_body", wait_for_servers=0.0)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', DisplayTrajectory, queue_size=20)
        self.move_group = whole_body
        self.move_group.set_planning_time(10.0)  # 最大計画時間を10秒に設定

        # TF2リスナーのセットアップ
        self.tf_listener = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_listener)
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def broadcast_tf(self, position: List[float, float, float], orientation: List[float, float, float, float], target_frame: str, base_frame="odom") -> None:
        """TFの配信をする関数
        Args:
            position (List[float, float, float]): 目標フレームの位置を表す3要素のタプル (x, y, z)
            orientation (List[float, float, float, float]): 目標フレームの姿勢を表すクォータニオン (x, y, z, w)
            target_frame (str): 変換情報が適用される子フレームの名前
            base_frame (str, optional): 変換情報の基準となる親フレームの名前。デフォルトは 'odom'

        Example:
            broadcast_tf([1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0], "target_frame", "odom")
        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = base_frame
        transform.child_frame_id = target_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]

        # 発行するトランスフォームを送信
        self.broadcaster.sendTransform(transform)

    def move_to_tf_target(self, target_frame: str, base_frame="odom", timeout=30) -> bool:
        """全身駆動を用いて目標とするTFへエンドエフェクタを移動させる関数
        Args:
            target_frame(str): 目標とするTFの名前
            base_frame(str): 指定したTFの親リンク
                defaults to odom
            timeout(int): executeに関するタイムアウト
                defaults to 30
        Returns:
            bool: 移動に成功したかどうか
        """
        try:
            # リスナーを使用して、base_frameからtarget_frameへの変換を取得します
            trans = self.tf_listener.lookup_transform(base_frame, target_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.logerr("TF transform error: {}".format(e))
            return False

        # TFから取得した変換をPoseに変換
        target_pose = tf2_geometry_msgs.do_transform_pose(PoseStamped(), trans)
        self.loginfo("get target pose")

        # MoveItを使用して目標位置に移動
        self.move_group.set_pose_target(target_pose.pose)

        # 目標位置への移動
        status = False
        start_time = rospy.Time.now()
        timeout_duration = rospy.Duration(timeout)

        while status is False:
            current_time = rospy.Time.now()
            elapsed_time = current_time - start_time

            # 動作計画できない場合を考慮したタイムアウトの実装
            if elapsed_time > timeout_duration:
                self.logwarn("TIMEOUT cannot reach target")
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                status = False
                break

            status = self.move_group.go(wait=True)

        if status is True:
            self.logsuccess("reached target tf")

        return status

    # def all_close(self, goal, actual, tolerance):
    #     """
    #     目標位置と現在位置の差を比較
    #     """
    #     if type(goal) is list:
    #         for g, a in zip(goal, actual):
    #             if abs(a - g) > tolerance:
    #                 return False
    #     elif type(goal) is Pose:
    #         return self.all_close([goal.position.x, goal.position.y, goal.position.z], [actual.position.x, actual.position.y, actual.position.z], tolerance) and \
    #             self.all_close([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w], [actual.orientation.x, actual.orientation.y, actual.orientation.z, actual.orientation.w], tolerance)
    #     return True


if __name__ == '__main__':
    try:
        rospy.init_node("moveit_test")
        cls = HSRBMoveIt()
        target_frame = "target_frame"
        cls.move_to_tf_target(target_frame)

    except rospy.ROSInterruptException:
        pass
