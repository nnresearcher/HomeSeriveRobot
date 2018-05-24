/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <cfloat>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
// %EndTag(INCLUDES)%

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
double scav_distance = 0.0;

#include <cmath>
#include <limits>

bool AreSame(double a, double b) {
    return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
}

void odom_callback (const nav_msgs::Odometry::ConstPtr& msg)
   {
   /*ROS_INFO("Seq: [%d]", msg->header.seq);*/
   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
   /*ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);*/
      if ((msg->pose.pose.position.x > 0.3) && (msg->pose.pose.position.y < -1.7) ) {
      //if (AreSame(msg->pose.pose.position.x,0.4)){
	marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        sleep(2);
        }
      if (msg->pose.pose.position.x > 4.7)
        {
         marker.pose.position.x = 4.8;
         marker.pose.position.y = 0.2;
         marker.pose.orientation.w = 1.0;
         marker.action = visualization_msgs::Marker::ADD;
         sleep(20);
         marker_pub.publish(marker);
        }
  }




void amcl_callback (const geometry_msgs::PoseWithCovarianceStamped & pose)
   {
     printf ("Received pose %f, %f, %f\n", pose.pose.pose.position.x,
             pose.pose.pose.position.y, pose.pose.pose.position.z);
      geometry_msgs::Point p;	
	  p.x = pose.pose.pose.position.x; 
	  p.y = pose.pose.pose.position.y; 
	  p.z = pose.pose.pose.position.z; 
 
          if (false  == marker.points.empty()) {
	      double dis = pow((p.x-marker.points.back().x), 2.0) + pow((p.y-marker.points.back().y), 2.0); 
	      scav_distance += pow(dis, 0.5); 
	      printf ("Robot-distance %f\n ",scav_distance);
	    }
      if (scav_distance > 0.3 ) {
	marker.points.push_back(p);
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker); 
        sleep(2);
	}
      if (scav_distance > 1.8)
	{
        marker.pose.position.x = 4.8;
        marker.pose.position.y = 0.2;
        marker.pose.orientation.w = 1.0;
        marker.action = visualization_msgs::Marker::ADD;
        sleep(20);
        marker_pub.publish(marker);
	}
   }


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
// %EndTag(INIT)%
 // odom_subscriber = n.subscribe("/odom", 10, odom_callback);
  //geometry_msgs::PoseWithCovarianceStamped::ConstPtr&msgAMCL{}; 	
  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  //uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t shape = visualization_msgs::Marker::SPHERE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    //visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    //marker.ns = "basic_shapes";
    marker.ns = "add_markers";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = 0.2;
    marker.pose.position.y = -1.80;
    marker.pose.position.z = 1.20;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%
    //sleep(2);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
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
// %EndTag(PUBLISH)%

    // Cycle between different shapes
// %Tag(CYCLE_SHAPES)%
    /*switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }*/
// %EndTag(CYCLE_SHAPES)%

// %Tag(SLEEP_END)%
    r.sleep();
  //}
// %EndTag(SLEEP_END)%
     //ros::Subscriber < geometry_msgs::PoseWithCovarianceStamped > 
     //poseSub ("/odom", &odom_callback);
     //n.subscribe (poseSub);
    //ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 100, odom_callback);
    ros::Subscriber sub_odom = n.subscribe("odom", 10, odom_callback);

    while (1)
    {
      ros::spinOnce();
      ROS_WARN_ONCE("Keep checking robot at marker");
      sleep(1);
      /*marker.action = visualization_msgs::Marker::DELETE;
      sleep(20);
      marker.pose.position.x = 4.8;
      marker.pose.position.y = 0.2;
      marker.pose.orientation.w = 1.0;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);*/
   }
  }	
}
// %EndTag(FULLTEXT)%
