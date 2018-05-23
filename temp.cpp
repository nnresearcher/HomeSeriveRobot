#include <ros/ros.h>
#include <visualization_msgs/Marker.h>



int main( int argc, char** argv )
{
// We initialize ROS, and create a ros::Publisher on the visualization_marker topic.
ros::init(argc, argv, "add_markers");
ros::NodeHandle n;
ros::Rate r(1);

// This call connects to the master to publicize that the node will be publishing messages on the given topic. 
// This method returns a Publisher that allows you to publish a message on this topic.
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

// Set our initial shape type to be a cube
// uint32_t shape = visualization_msgs::Marker::CUBE;

while (ros::ok())
{
visualization_msgs::Marker marker;
// Set the frame ID and timestamp. See the TF tutorials for information on these.
marker.header.frame_id = "/map";
marker.header.stamp = ros::Time::now();

// Set the namespace and id for this marker. This serves to create a unique ID
// Any marker sent with the same namespace and id will overwrite the old one
marker.ns = "basic_shapes";
marker.id = 0;

// Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
marker.type = visualization_msgs::Marker::CUBE;

// Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
marker.action = visualization_msgs::Marker::ADD;

// Set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
marker.pose.position.x = 0.0;
marker.pose.position.y = -1.6;
marker.pose.position.z = 0;
marker.pose.orientation.x = 5.5;
marker.pose.orientation.y = -2.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;

// Set the scale of the marker -- 1x1x1 here means 1m on a side

marker.scale.x = 0.2;
marker.scale.y = 0.2;
marker.scale.z = 0.2;

// Set the color -- be sure to set alpha to something non-zero!
marker.color.r = 0.0f;
marker.color.g = 0.843f;
marker.color.b = 0.0f;
marker.color.a = 1.0;

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
ROS_INFO("Marker is in the pickup place now");
ros::Duration(5.0).sleep();

marker.action = visualization_msgs::Marker::DELETE;
marker_pub.publish(marker);
ROS_INFO("Robot pick off the Marker and transport");
ros::Duration(5.0).sleep();

marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 5.5;
marker.pose.position.y = -2.0;
marker_pub.publish(marker);
ROS_INFO("Marker is in the drop off place");
// spin so we can see the messages...
ros::spin(); 
return 0;
// r.sleep();
}
}
