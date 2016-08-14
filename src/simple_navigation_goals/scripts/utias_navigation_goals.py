#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus

def navigationGoals():
  move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

  rospy.loginfo("Wait for the action server to come up")
  move_base.wait_for_server(rospy.Duration(30))

  target = MoveBaseGoal()
  target.target_pose.header.frame_id = 'map'
  target.target_pose.header.stamp = rospy.Time.now()

  target.target_pose.pose.position.x = -0.34
  target.target_pose.pose.position.y = 0.35
  target.target_pose.pose.orientation.z = 0.00
  target.target_pose.pose.orientation.w = 1.00

  rospy.loginfo("Sending goal UTIAS 189")
  move_base.send_goal(target)

  success = move_base.wait_for_result()

  if not success:
    rospy.loginfo("The turtlebot failed to reach UTIAS 189")
    return False
  else:
    state = move_base.get_state()
    if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("The turtlebot moved to UTIAS 189")
      return dock_drive_client()
  

def doneCb(status, result):
  if 0: print ''
  elif status == GoalStatus.PENDING   : state='PENDING'
  elif status == GoalStatus.ACTIVE    : state='ACTIVE'
  elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
  elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
  elif status == GoalStatus.ABORTED   : state='ABORTED'
  elif status == GoalStatus.REJECTED  : state='REJECTED'
  elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
  elif status == GoalStatus.RECALLING : state='RECALLING'
  elif status == GoalStatus.RECALLED  : state='RECALLED'
  elif status == GoalStatus.LOST      : state='LOST'
  # Print state of action server
  print 'Result - [ActionServer: ' + state + ']: ' + result.text

def activeCb():
  if 0: print 'Action server went active.'

def feedbackCb(feedback):
  # Print state of dock_drive module (or node.)
  print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

def dock_drive_client():
  # add timeout setting
  client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
  while not client.wait_for_server(rospy.Duration(5.0)):
    if rospy.is_shutdown(): return
    print 'Action server is not connected yet. still waiting...'

  goal = AutoDockingGoal();
  client.send_goal(goal, doneCb, activeCb, feedbackCb)
  print 'Goal: Sent.'
  rospy.on_shutdown(client.cancel_goal)
  client.wait_for_result()

  #print '    - status:', client.get_goal_status_text()
  return client.get_result()

if __name__ == '__main__':
  try:
    rospy.init_node('utias_navigation_goals_py', anonymous=True)
    navigationGoals()
    #print ''
    #print "Result: ", result
  except rospy.ROSInterruptException: 
    print "program interrupted before completion"
