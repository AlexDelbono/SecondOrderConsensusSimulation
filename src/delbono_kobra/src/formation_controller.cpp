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
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <math.h>

#define WAITING 0.500
#define ALPHA 0.1
#define GAMMA 10.0

#define LINK_WEIGHT 0.1
#define LINK_WEIGHT_ANG 0.01

#define ALPHA_ANG 3.0
#define GAMMA_ANG 1.0

#define K 5.0

#define SHRINK 1.0

#define INDEX 0.1

#define MAX_VEL 0.2
#define MAX_ANG_VEL 0.8

#define DESTX 8.0
#define DESTY 8.0
#define HALFDIST 4.0

const double DEST1X = DESTX-HALFDIST;
const double DEST1Y = DESTY+HALFDIST;
const double DEST2X = DESTX+HALFDIST;
const double DEST2Y = DESTY+HALFDIST;
const double DEST3X = DESTX+HALFDIST;
const double DEST3Y = DESTY-HALFDIST;
const double DEST4X = DESTX-HALFDIST;
const double DEST4Y = DESTY-HALFDIST;

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

double goalAng(double x1, double y1, double x2, double y2) {
  double x = x2-x1;
  double y = y2-y1;

  return atan2(y,x);
}

int netStructure = 0;

double A0[4][4] = {{0.0, 1.0, 1.0, 1.0},
                 {1.0, 0.0, 1.0, 1.0},
                 {1.0, 1.0, 0.0, 1.0},
                 {1.0, 1.0, 1.0, 0.0}};

double A1[4][4] = {{0.0, 1.0, 1.0, 1.0},
                  {0.0, 0.0, 0.0, 0.0},
                  {0.0, 0.0, 0.0, 1.0},
                  {0.0, 0.0, 0.0, 0.0}};

double A2[4][4] = {{0.0, 1.0, 1.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0}};

double A3[4][4] = {{0.0, 1.0, 0.0, 0.0},
                  {0.0, 0.0, 1.0, 0.0},
                  {1.0, 0.0, 0.0, 0.0},
                  {0.0, 0.0, 0.0, 0.0}};

void getGraphMatrix(double a[][4]){

  switch (netStructure) {
    case 0: for(int i=0; i<4; ++i)for(int j=0; j<4; ++j) a[i][j]=A0[i][j];
            break;
    case 1: for(int i=0; i<4; ++i)for(int j=0; j<4; ++j) a[i][j]=A1[i][j];
            break;
    case 2: for(int i=0; i<4; ++i)for(int j=0; j<4; ++j) a[i][j]=A2[i][j];
            break;
    case 3: for(int i=0; i<4; ++i)for(int j=0; j<4; ++j) a[i][j]=A3[i][j];
            break;
    default: for(int i=0; i<4; ++i)for(int j=0; j<4; ++j) a[i][j]=0;
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "formation_controller");

  ros::NodeHandle node;

  usleep(5000000);

  if(!ros::param::get("/net_structure", netStructure))
    netStructure = 0;

  ros::Publisher velPub1 = node.advertise<geometry_msgs::Twist>("/DelbonoKobraCompl1/cmd_velocity", 100);
  ros::Publisher velPub2 = node.advertise<geometry_msgs::Twist>("/DelbonoKobraCompl2/cmd_velocity", 100);
  ros::Publisher velPub3 = node.advertise<geometry_msgs::Twist>("/DelbonoKobraCompl3/cmd_velocity", 100);
  ros::Publisher velPub4 = node.advertise<geometry_msgs::Twist>("/DelbonoKobraCompl4/cmd_velocity", 100);

  double a1=0, a2=0, a3=0, a4=0;
  double g1=0, g2=0, g3=0, g4=0;
  double a[4][4];

  while(ros::ok())
  {
    ros::Time t0 = ros::Time::now();

    const std::string str1("DelbonoKobraCompl1/simulator_odometry");
    const std::string str2("DelbonoKobraCompl2/simulator_odometry");
    const std::string str3("DelbonoKobraCompl3/simulator_odometry");
    const std::string str4("DelbonoKobraCompl4/simulator_odometry");


    nav_msgs::OdometryConstPtr msg1 = ros::topic::waitForMessage<nav_msgs::Odometry>(str1, ros::Duration(WAITING));
    nav_msgs::OdometryConstPtr msg2 = ros::topic::waitForMessage<nav_msgs::Odometry>(str2, ros::Duration(WAITING));
    nav_msgs::OdometryConstPtr msg3 = ros::topic::waitForMessage<nav_msgs::Odometry>(str3, ros::Duration(WAITING));
    nav_msgs::OdometryConstPtr msg4 = ros::topic::waitForMessage<nav_msgs::Odometry>(str4, ros::Duration(WAITING));


    getGraphMatrix(a);

    double dbef1 = distance(msg1->pose.pose.position.x, msg1->pose.pose.position.y, DEST1X, DEST1Y );
    double dbef2 = distance(msg2->pose.pose.position.x, msg2->pose.pose.position.y, DEST2X, DEST2Y );
    double dbef3 = distance(msg3->pose.pose.position.x, msg3->pose.pose.position.y, DEST3X, DEST3Y );
    double dbef4 = distance(msg4->pose.pose.position.x, msg4->pose.pose.position.y, DEST4X, DEST4Y );

    double d1 = -dbef1 * cos(tf::getYaw(msg1->pose.pose.orientation)-goalAng(msg1->pose.pose.position.x, msg1->pose.pose.position.y, DEST1X, DEST1Y ));
    double d2 = -dbef2 * cos(tf::getYaw(msg2->pose.pose.orientation)-goalAng(msg2->pose.pose.position.x, msg2->pose.pose.position.y, DEST2X, DEST2Y ));
    double d3 = -dbef3 * cos(tf::getYaw(msg3->pose.pose.orientation)-goalAng(msg3->pose.pose.position.x, msg3->pose.pose.position.y, DEST3X, DEST3Y ));
    double d4 = -dbef4 * cos(tf::getYaw(msg4->pose.pose.orientation)-goalAng(msg4->pose.pose.position.x, msg4->pose.pose.position.y, DEST4X, DEST4Y ));

    double v1 = msg1->twist.twist.linear.x;
    double v2 = msg2->twist.twist.linear.x;
    double v3 = msg3->twist.twist.linear.x;
    double v4 = msg4->twist.twist.linear.x;

    a1 = -ALPHA*d1 - GAMMA * ALPHA * v1
          -LINK_WEIGHT*(a[1][0]*(d1-d2)+a[2][0]*(d1-d3)+a[3][0]*(d1-d4))
          -LINK_WEIGHT*GAMMA*(a[1][0]*(v1-v2)+a[2][0]*(v1-v3)+a[3][0]*(v1-v4));
    a2 = -ALPHA*d2 - GAMMA * ALPHA * v2
          -LINK_WEIGHT*(a[0][1]*(d2-d1)+a[2][1]*(d2-d3)+a[3][1]*(d2-d4))
          -LINK_WEIGHT*GAMMA*(a[0][1]*(v2-v1)+a[2][1]*(v2-v3)+a[3][1]*(v2-v4));
    a3 = -ALPHA*d3 - GAMMA * ALPHA * v3
          -LINK_WEIGHT*(a[0][2]*(d3-d1)+a[1][2]*(d3-d2)+a[3][2]*(d3-d4))
          -LINK_WEIGHT*GAMMA*(a[0][2]*(v3-v1)+a[1][2]*(v3-v2)+a[3][2]*(v3-v4));
    a4 = -ALPHA*d4 - GAMMA * ALPHA * v4
          -LINK_WEIGHT*(a[0][3]*(d4-d1)+a[1][3]*(d4-d2)+a[2][3]*(d4-d3))
          -LINK_WEIGHT*GAMMA*(a[0][3]*(v4-v1)+a[1][3]*(v4-v2)+a[2][3]*(v4-v3));

    double s1=1, s2=1, s3=1, s4=1;

    if(dbef1<SHRINK ) s1=0;
    if(dbef2<SHRINK ) s2=0;
    if(dbef3<SHRINK ) s3=0;
    if(dbef4<SHRINK ) s4=0;

    double o1 = s1*(tf::getYaw(msg1->pose.pose.orientation)-goalAng(msg1->pose.pose.position.x, msg1->pose.pose.position.y, DEST1X, DEST1Y));
    double o2 = s2*(tf::getYaw(msg2->pose.pose.orientation)-goalAng(msg2->pose.pose.position.x, msg2->pose.pose.position.y, DEST2X, DEST2Y ));
    double o3 = s3*(tf::getYaw(msg3->pose.pose.orientation)-goalAng(msg3->pose.pose.position.x, msg3->pose.pose.position.y, DEST3X, DEST3Y ));
    double o4 = s4*(tf::getYaw(msg4->pose.pose.orientation)-goalAng(msg4->pose.pose.position.x, msg4->pose.pose.position.y, DEST4X, DEST4Y ));

    double w1 = msg1->twist.twist.angular.z;
    double w2 = msg2->twist.twist.angular.z;
    double w3 = msg3->twist.twist.angular.z;
    double w4 = msg4->twist.twist.angular.z;

    g1 = -ALPHA_ANG*o1 - GAMMA_ANG * ALPHA_ANG * w1
          -LINK_WEIGHT_ANG*(a[1][0]*(o1-o2)+a[2][0]*(o1-o3)+a[3][0]*(o1-o4))
          -LINK_WEIGHT_ANG*GAMMA*(a[1][0]*(w1-w2)+a[2][0]*(w1-w3)+a[3][0]*(w1-w4));
    g2 = -ALPHA_ANG*o2 - GAMMA_ANG * ALPHA_ANG * w2
          -LINK_WEIGHT_ANG*(a[0][1]*(o2-o1)+a[2][1]*(o2-o3)+a[3][1]*(o2-o4))
          -LINK_WEIGHT_ANG*GAMMA*(a[0][1]*(w2-w1)+a[2][1]*(w2-w3)+a[3][1]*(w2-w4));
    g3 = -ALPHA_ANG*o3 - GAMMA_ANG * ALPHA_ANG * w3
          -LINK_WEIGHT_ANG*(a[0][2]*(o3-o1)+a[1][2]*(o3-o2)+a[3][2]*(o3-o4))
          -LINK_WEIGHT_ANG*GAMMA*(a[0][2]*(w3-w1)+a[1][2]*(w3-w2)+a[3][2]*(w3-w4));
    g4 = -ALPHA_ANG*o4 - GAMMA_ANG * ALPHA_ANG * w4
          -LINK_WEIGHT_ANG*(a[0][3]*(o4-o1)+a[1][3]*(o4-o2)+a[2][3]*(o4-o3))
          -LINK_WEIGHT_ANG*GAMMA*(a[0][3]*(w4-w1)+a[1][3]*(w4-w2)+a[2][3]*(w4-w3));

    ros::Time t1 = ros::Time::now();
    double dt = (t1-t0).toSec();

    geometry_msgs::Twist ctrl1;
    geometry_msgs::Twist ctrl2;
    geometry_msgs::Twist ctrl3;
    geometry_msgs::Twist ctrl4;

    ctrl1.linear.x = v1 + dt * a1;
    ctrl1.linear.x = ctrl1.linear.x < MAX_VEL ? ctrl1.linear.x : MAX_VEL;
    ctrl1.linear.x = ctrl1.linear.x > -MAX_VEL ? ctrl1.linear.x : -MAX_VEL;
    ctrl1.angular.z = w1 + dt * g1;
    ctrl1.angular.z = ctrl1.angular.z < MAX_ANG_VEL ? ctrl1.angular.z : MAX_ANG_VEL;
    ctrl1.angular.z = ctrl1.angular.z > -MAX_ANG_VEL ? ctrl1.angular.z : -MAX_ANG_VEL;

    ctrl2.linear.x = v2 + dt * a2;
    ctrl2.linear.x = ctrl2.linear.x < MAX_VEL ? ctrl2.linear.x : MAX_VEL;
    ctrl2.linear.x = ctrl2.linear.x > -MAX_VEL ? ctrl2.linear.x : -MAX_VEL;
    ctrl2.angular.z = w2 + dt * g2;
    ctrl2.angular.z = ctrl2.angular.z < MAX_ANG_VEL ? ctrl2.angular.z : MAX_ANG_VEL;
    ctrl2.angular.z = ctrl2.angular.z > -MAX_ANG_VEL ? ctrl2.angular.z : -MAX_ANG_VEL;

    ctrl3.linear.x = v3 + dt * a3;
    ctrl3.linear.x = ctrl3.linear.x < MAX_VEL ? ctrl3.linear.x : MAX_VEL;
    ctrl3.linear.x = ctrl3.linear.x > -MAX_VEL ? ctrl3.linear.x : -MAX_VEL;
    ctrl3.angular.z = w3 + dt * g3;
    ctrl3.angular.z = ctrl3.angular.z < MAX_ANG_VEL ? ctrl3.angular.z : MAX_ANG_VEL;
    ctrl3.angular.z = ctrl3.angular.z > -MAX_ANG_VEL ? ctrl3.angular.z : -MAX_ANG_VEL;

    ctrl4.linear.x = v4 + dt * a4;
    ctrl4.linear.x = ctrl4.linear.x < MAX_VEL ? ctrl4.linear.x : MAX_VEL;
    ctrl4.linear.x = ctrl4.linear.x > -MAX_VEL ? ctrl4.linear.x : -MAX_VEL;
    ctrl4.angular.z = w4 + dt * g4;
    ctrl4.angular.z = ctrl4.angular.z < MAX_ANG_VEL ? ctrl4.angular.z : MAX_ANG_VEL;
    ctrl4.angular.z = ctrl4.angular.z > -MAX_ANG_VEL ? ctrl4.angular.z : -MAX_ANG_VEL;

    velPub1.publish(ctrl1);
    velPub2.publish(ctrl2);
    velPub3.publish(ctrl3);
    velPub4.publish(ctrl4);

    ros::spinOnce();
  }



  return 0;
}
