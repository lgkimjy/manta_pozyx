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
    vector<float> pose = {0, 0, 0};
    vector<float> alpha = {0, 0, 0};
    vector<float> prev_raw_value = {0, 0, 0};
    vector<float> offset = {0, 0, 0};
};

vector<tag> tags;


YAML::Node lpf_doc;
YAML::Node offset_doc;
std::string path_lpf;
std::string path_offset;
vector<string> docs;

manta_positioning::mqtt_msg pose_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub;

int flag=0;
float prev_avg_value;
vector<float> data_buff;

void readConfigData();
void updateConfigData();
float setoffset(tag tag_data, int index, int num);
float LowPassFilter(tag tag_data, int index, int num);
float MovingAvgeFilter(float raw_value, int n_samples);
void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg);