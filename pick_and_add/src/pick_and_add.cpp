#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "pick_and_add");
  
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  // Wait 5 sec for move_base action server to come up
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "pick_and_addss";
  marker.id = 0;
  marker.type=visualization_msgs::Marker::CYLINDER;
  //marker.type=visualization_msgs::Marker::CYLINDER;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER


  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0.0f;
  marker.pose.position.y = -1.6f;
  marker.pose.position.z = 0.0f;
  marker.pose.orientation.x = 0.0f;
  marker.pose.orientation.y = 0.0f;
  marker.pose.orientation.z = 0.0f;
  marker.pose.orientation.w = 1.0f;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();

  // Publish the marker
  
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
  // Cycle between different shapes
  ROS_INFO("Marker is in the first place");
//*****************************************************************************
  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);
 
  // Wait 5 sec for move_base action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}

//*****************************************************************************
  //move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = -1.6;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  ros::Duration(5.0).sleep();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
      ROS_INFO("Hooray, the base moved 1 meter forward");
      ROS_INFO("Waiting for 5 seconds");
      ros::Duration(5.0).sleep();

      ROS_INFO("Marker has been picked up ");
      marker.action = visualization_msgs::Marker::DELETE;
      ros::Duration(5.0).sleep();

      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 5.5;
      marker.pose.position.y = -2.0;
      marker_pub.publish(marker);
      ROS_INFO("Marker is in the second place");

      goal.target_pose.pose.position.x = 5.5;
      goal.target_pose.pose.position.y = -2.0;
      goal.target_pose.header.stamp = ros::Time::now();
      ROS_INFO("Sending second goal");
      ac.sendGoal(goal);
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	  ROS_INFO("Hooray, moved to second goal");
      }
      else
      {
          ROS_INFO("The base failed to move forward to second goal");
      }
  }
  else
  {
          ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
  ros::spin();
  return 0;
}
