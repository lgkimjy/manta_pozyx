/*
 * manta_positioning_association.h
 *      Author: junyoung kim / lgkimjy
 */

#include <iostream>
#include <string>
#include <cmath>
#include <sstream>
#include <random>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <manta_positioning/mqtt_msg.h>

using namespace std;

manta_positioning::mqtt_msg filtered_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub;

void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg);