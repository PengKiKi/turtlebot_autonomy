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
  i = 0

  while i<8:
    target.target_pose.header.frame_id = 'map'
    target.target_pose.header.stamp = rospy.Time.now()

    target.target_pose.pose.position.x = -2.53
    target.target_pose.pose.position.y = -1.91
    target.target_pose.pose.orientation.z = 1.00
    target.target_pose.pose.orientation.w = 0.06

    rospy.loginfo("Sending goal A")
    move_base.send_goal(target)

    success = move_base.wait_for_result()

    if not success:
      rospy.loginfo("The turtlebot failed to reach A")
      break
    else:
      state = move_base.get_state()
      if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("The turtlebot moved to A")
        i = i + 1

        target.target_pose.header.frame_id = 'map'
        target.target_pose.header.stamp = rospy.Time.now()

        target.target_pose.pose.position.x = -2.22
        target.target_pose.pose.position.y = -0.44
        target.target_pose.pose.orientation.z = 0.54
        target.target_pose.pose.orientation.w = -0.84

        rospy.loginfo("Sending goal B")
        move_base.send_goal(target)

        success = move_base.wait_for_result()

        if not success:
          rospy.loginfo("The turtlebot failed to reach B")
          break
        else:
          state = move_base.get_state()
          if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("The turtlebot moved to B")
            i = i + 1

            target.target_pose.header.frame_id = 'map'
            target.target_pose.header.stamp = rospy.Time.now()

            target.target_pose.pose.position.x = -1.95
            target.target_pose.pose.position.y = 3.23
            target.target_pose.pose.orientation.z = 0.99
            target.target_pose.pose.orientation.w = -0.14

            rospy.loginfo("Sending goal C")
            move_base.send_goal(target)

            success = move_base.wait_for_result()

            if not success:
              rospy.loginfo("The turtlebot failed to reach C")
              break
            else:
              state = move_base.get_state()
              if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("The turtlebot moved to C")
                i = i + 1

                target.target_pose.header.frame_id = 'map'
                target.target_pose.header.stamp = rospy.Time.now()

                target.target_pose.pose.position.x = -2.25
                target.target_pose.pose.position.y = 0.62
                target.target_pose.pose.orientation.z = -0.76
                target.target_pose.pose.orientation.w = 0.65l

                rospy.loginfo("Sending goal D")
                move_base.send_goal(target)

                success = move_base.wait_for_result()

                if not success:
                  rospy.loginfo("The turtlebot failed to reach D")
                  break
                else:
                  state = move_base.get_state()
                  if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("The turtlebot moved to D")
                    i = i + 1

  # move close to dock
  target.target_pose.header.frame_id = 'map'
  target.target_pose.header.stamp = rospy.Time.now()

  target.target_pose.pose.position.x = -1.050
  target.target_pose.pose.position.y = 0.009
  target.target_pose.pose.orientation.z = 0.009
  target.target_pose.pose.orientation.w = 1.000

  rospy.loginfo("Sending goal 'dock'")
  move_base.send_goal(target)

  success = move_base.wait_for_result()

  if not success:
    rospy.loginfo("The turtlebot failed to reach 'dock'")
    return False
  else:
    state = move_base.get_state()
    if state == GoalStatus.SUCCEEDED:
      rospy.loginfo("The turtlebot moved to 'dock'")
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
    rospy.init_node('utias_145_navigation_goals_py', anonymous=True)
    navigationGoals()
    #print ''
    #print "Result: ", result
  except rospy.ROSInterruptException: 
    print "program interrupted before completion"
