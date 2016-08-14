#!/usr/bin/env python

import roslib; roslib.load_manifest('turtlebot_sim_2dnav')
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib
from actionlib_msgs.msg import GoalStatus

import std_srvs
from std_srvs.srv import Empty

def navigationGoals():
  move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

  rospy.loginfo("Wait for the action server to come up")
  move_base.wait_for_server(rospy.Duration(30))

  target = MoveBaseGoal()
  target.target_pose.header.frame_id = 'map'
  target.target_pose.header.stamp = rospy.Time.now()

  target.target_pose.pose.position.x = 4.000
  target.target_pose.pose.position.y = 0.000
  target.target_pose.pose.orientation.z = 0.5708
  target.target_pose.pose.orientation.w = 0.8211

  rospy.loginfo("Sending goal A")
  move_base.send_goal(target)

  success = move_base.wait_for_result()

  if not success:
    rospy.loginfo("The turtlebot failed to reach A")
    return False
  else:
    state = move_base.get_state()
    if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("The turtlebot moved to A")
      return image_capture()

def image_capture():
  rospy.wait_for_service('image_saver/save')
  try:
    rospy.loginfo("Telling the turtlebot to take a picture")
    save_image = rospy.ServiceProxy('image_saver/save', Empty)
    resp = save_image()
    return True
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

if __name__ == '__main__':
  try:
    rospy.init_node('utias_navigation_goals_py', anonymous=True)
    navigationGoals()

  except rospy.ROSInterruptException: 
    print "program interrupted before completion"
