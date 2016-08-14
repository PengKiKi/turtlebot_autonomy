#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  int i = 0;

  //repeat until 100 goals have been sent
  while(i < 100)
  {

    //we'll send a goal to the robot to move to A
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.seq = i;
    i++;

    goal.target_pose.pose.position.x = 4.000;
    goal.target_pose.pose.position.y = 0.000;
    goal.target_pose.pose.orientation.z = 0.5708;
    goal.target_pose.pose.orientation.w = 0.8211;

    ROS_INFO("Sending goal A");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to A");

      //we'll send a goal to the robot to move to B
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.header.seq = i;
      i++;

      goal.target_pose.pose.position.x = 4.000;
      goal.target_pose.pose.position.y = 4.000;
      goal.target_pose.pose.orientation.z = 1.0000;
      goal.target_pose.pose.orientation.w = 0.0000;

      ROS_INFO("Sending goal B");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base moved to B");

        //we'll send a goal to the robot to move to C
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.seq = i;
        i++;

        goal.target_pose.pose.position.x = -4.000;
        goal.target_pose.pose.position.y = 4.000;
        goal.target_pose.pose.orientation.z = -0.7687;
        goal.target_pose.pose.orientation.w = 0.6396;

        ROS_INFO("Sending goal C");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          ROS_INFO("Hooray, the base moved to C");

          //we'll send a goal to the robot to move to D
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.header.stamp = ros::Time::now();
          goal.target_pose.header.seq = i;
          i++;

          goal.target_pose.pose.position.x = 0.000;
          goal.target_pose.pose.position.y = 0.000;
          goal.target_pose.pose.orientation.z = 0.0123;
          goal.target_pose.pose.orientation.w = 1.0000;

          ROS_INFO("Sending goal D");
          ac.sendGoal(goal);

          ac.waitForResult();

          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray, the base moved to D");
          }
          else
            ROS_INFO("The base failed to move to D for some reason");
        }
        else
          ROS_INFO("The base failed to move to C for some reason");
      }
      else
        ROS_INFO("The base failed to move to B for some reason");
    }
    else
      ROS_INFO("The base failed to move to A for some reason");

  }

  return 0;
}
