#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

if __name__ == "__main__":
    move_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers=0.0)
    current_state = move_group.get_current_state()
    print(current_state)
