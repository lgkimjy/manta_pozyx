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
    string id;
    float x;
    float y;
    float z;
    float x_alpha;
    float y_alpha;
    float z_alpha;
    float prev_raw_value;
};

vector<tag> tags;

// manta_positioning::mqtt_msg filtered_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub;

float prev_avg_value;
float x_alpha, y_alpha, z_alpha;
float xoffset, yoffset, zoffset;
int flag=0;
int count=0;
vector<float> data_buff;

void readConfigData();
float LowPassFilter(tag tag_data, int index);
float MovingAvgeFilter();
void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg);