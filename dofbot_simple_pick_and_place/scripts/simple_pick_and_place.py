#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_pick_and_place", anonymous=True)
    
    robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    

    # # We can also print the name of the end-effector link for this group:
    # eef_link = move_group.get_end_effector_link()
    # print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
        
    # rospy.sleep(10)
    group_name = "arm_group"
    move_group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=30)
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)