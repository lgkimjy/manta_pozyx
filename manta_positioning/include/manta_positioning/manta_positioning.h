/*
 * manta_positioning.h
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

#include <manta_positioning/mqtt_msg.h>

using namespace std;

struct tag{
    float x;
    float y;
    float z;
};

manta_positioning::mqtt_msg filtered_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub;

float raw_value, prev_raw_value, alpha;
float xoffset, yoffset, zoffset;
int flag=0;

void readConfigData();
void setOffset(tag data);
float LowPassFilter(float raw_value);
float MovingAvgeFilter();
void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg);