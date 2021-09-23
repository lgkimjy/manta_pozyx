/*
 * manta_positioning.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "manta_positioning/manta_positioning.h"

void readConfigData()
{
    YAML::Node lpf_doc;
    YAML::Node offset_doc;
    std::string path_lpf = ros::package::getPath("manta_positioning")+"/config/LPF.yaml";
    std::string path_offset = ros::package::getPath("manta_positioning")+"/config/offset.yaml";

    try{
        lpf_doc = YAML::LoadFile(path_lpf.c_str());
        offset_doc = YAML::LoadFile(path_offset.c_str());
    }
    catch(const std::exception& e){
        ROS_ERROR("Fail to load Config yaml file!");
        return;
    }
    x_alpha = lpf_doc["x_alpha"].as<float>();
    y_alpha = lpf_doc["y_alpha"].as<float>();
    z_alpha = lpf_doc["z_alpha"].as<float>();
    xoffset = offset_doc["x"].as<float>();
    yoffset = offset_doc["y"].as<float>();
    zoffset = offset_doc["z"].as<float>();
}

void setOffset(tag data)
{
    pose.pose.position.x = data.x - xoffset;
    pose.pose.position.y = data.y - yoffset;
    pose.pose.position.z = data.z - zoffset;
}

float LowPassFilter(float raw_value, float alpha)
{
    float output;
    
    output = alpha*raw_value + (1.0 - alpha)*prev_raw_value;
    prev_raw_value = output;

    return output;
}

float MovingAvgeFilter()
{
    return 0.0;
}

void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg)
{
    tag offset;
    
    readConfigData();
    filtered_msg.data.clear();
    
    if(flag == 0){
        prev_raw_value = msg->data[0].pose.position.z;
        flag++;
    }

    for(int i=0; i<msg->data.size(); i++){
        
        // Filtering
        pose = msg->data[i];                                                                  // data copy (deep copy)
        pose.pose.position.z = LowPassFilter(msg->data[0].pose.position.z, z_alpha);                   // Low Pass Filter

        // Data Offset
        setOffset(tag{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});

        filtered_msg.data.push_back(pose);
    }
    pose_pub.publish(filtered_msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "manta_positioning");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(10);

    ros::Subscriber obstacle_sub = nh.subscribe("/mqtt_coord", 10, poseCallback);  // Data from mqtt protocol
    // ros::Subscriber obstacle_sub = nh.subscribe("/coord", 10, poseCallback);    // Data from raspberry pi / arduino
    
    pose_pub = nh.advertise<manta_positioning::mqtt_msg>("/filtered/coord", 10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}