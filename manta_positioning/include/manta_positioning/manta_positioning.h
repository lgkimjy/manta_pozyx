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
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mqtt2ros/mqtt_msg.h>
#include <mqtt2ros/mqtt_mag.h>
#include <mqtt2ros/mqtt_imu.h>

using namespace std;
using namespace message_filters;

namespace WorldFrame{
    enum WorldFrame{
        ENU, NED, NWU
    };
}

struct tag{
    string id;
    ros::Time time;
    vector<float> pose = {0, 0, 0};
    vector<float> alpha = {0, 0, 0};
    vector<float> prev_raw_value = {0, 0, 0};
    vector<float> offset = {0, 0, 0};
};

// tag structs
vector<tag> tags;

// config data
YAML::Node lpf_doc;
YAML::Node offset_doc;
std::string path_lpf;
std::string path_offset;
vector<string> docs;

// ros publishers
mqtt2ros::mqtt_msg pose_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher pose_pub;

// variables for filter position data
int flag = 0;
float prev_avg_value;
vector<float> data_buff;

bool ahrs_flag = false;

template <typename T>
static inline void crossProduct(
    T ax, T ay, T az,
    T bx, T by, T bz,
    T &rx, T &ry, T &rz)
{
    rx = ay * bz - az * by;
    ry = az * bx - ax * bz;
    rz = ax * by - ay * bx;
}

template <typename T>
static inline T normalizeVector(T &vx, T &vy, T &vz)
{
    T norm = sqrt(vx * vx + vy * vy + vz * vz);
    T inv = 1.0 / norm;
    vx *= inv;
    vy *= inv;
    vz *= inv;
    return norm;
}

/* functions */
void readConfigData();
void updateConfigData();
float setoffset(tag tag_data, int index, int num);
float LowPassFilter(tag tag_data, int index, int num);
float MovingAvgeFilter(float raw_value, int n_samples);
double Convert(double radian);
bool computeOrientation(
    WorldFrame::WorldFrame frame,
    geometry_msgs::Vector3 A,
    geometry_msgs::Vector3 E,
    geometry_msgs::Quaternion &orientation);

/* callback */
void poseCallback(const mqtt2ros::mqtt_msg::ConstPtr &msg);
void imuMagPoseCallback(
    const mqtt2ros::mqtt_imu::ConstPtr &imu_msg_raw, 
    const mqtt2ros::mqtt_mag::ConstPtr &mag_msg,
    const mqtt2ros::mqtt_msg::ConstPtr &msg);