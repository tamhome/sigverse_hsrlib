#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sigverse_hsrlib import HSRBMoveIt

rospy.init_node("test")
moveit = HSRBMoveIt()
moveit.move_to_tf_target("target_frame")
