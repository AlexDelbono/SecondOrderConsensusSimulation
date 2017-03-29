/*
* Publish tf info
*
* Code taken from:
* http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
*
* Changes done by Alex Delbono
*
*/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

const double extraTime = 0.1;

void poseCallback(const nav_msgs::OdometryConstPtr& msg){
  static tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;

  transform.header.stamp = ros::Time(msg->header.stamp.toSec()+extraTime);
  transform.header.frame_id =   msg->header.frame_id;
  transform.child_frame_id = msg->child_frame_id;

  transform.transform.translation.x = msg->pose.pose.position.x;
  transform.transform.translation.y = msg->pose.pose.position.y;
  transform.transform.translation.z = msg->pose.pose.position.z;
  transform.transform.rotation = msg->pose.pose.orientation;

  br.sendTransform(transform);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simulator_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber subSim1 = node.subscribe("DelbonoKobraCompl4/simulator_odometry", 10, &poseCallback);
  ros::Subscriber subSim2 = node.subscribe("DelbonoKobraCompl3/simulator_odometry", 10, &poseCallback);
  ros::Subscriber subSim3 = node.subscribe("DelbonoKobraCompl2/simulator_odometry", 10, &poseCallback);
  ros::Subscriber subSim4 = node.subscribe("DelbonoKobraCompl1/simulator_odometry", 10, &poseCallback);

  ros::spin();

  
  return 0;
};
