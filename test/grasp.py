#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sigverse_hsrlib import HSRBMoveIt
from sigverse_hsrlib import MoveJoints
rospy.init_node("test")
moveit = HSRBMoveIt()
move_joint = MoveJoints()
# move_joint.neutral()
moveit.move_to_tf_target("target_frame")
