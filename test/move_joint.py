import rospy
from sigverse_hsrlib import MoveJoints
from sigverse_hsrlib import HSRBMoveIt
from geometry_msgs.msg import Pose


def test_move_base():
    move_joint = MoveJoints()
    move_joint.move_base_joint(1, 1, 0)


def test_gaze(pose, frame):
    move_joint = MoveJoints()
    move_joint.gaze_point(pose, frame)


if __name__ == "__main__":
    rospy.init_node("test_movejoint")
    # test_move_base()

    pose = Pose()
    pose.position.x = 2
    pose.position.y = 2
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    test_gaze(pose, "odom")
