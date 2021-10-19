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

float MovingAvgeFilter(float raw_value, int n_samples)
{
    float output;
    float samples;

    data_buff.push_back(raw_value);

    if(data_buff.size() == 1){
        output = raw_value;
    }
    else if(data_buff.size() < n_samples){
        output = (accumulate(data_buff.begin(), data_buff.end(), 0)) / data_buff.size();
    }
    else{

        samples = data_buff.front();
        data_buff.erase(data_buff.begin());

        output = prev_avg_value + (raw_value - samples) / n_samples;
        prev_avg_value = output;
    }

    return output;
}

void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg)
{
    tag offset;
    
    readConfigData();
    
    if(flag == 0){
        prev_raw_value = msg->data[0].pose.position.z;
        prev_avg_value = msg->data[0].pose.position.z;

        flag++;
    }

    for(int i=0; i<msg->data.size(); i++){
        pose = msg->data[i];                                                                 // data copy (deep copy)
        if(to_string(tag_id) == pose.header.frame_id)
        {
            // Filtering
            pose.pose.position.z = LowPassFilter(msg->data[i].pose.position.z, z_alpha);          // Low Pass Filter
            // pose.pose.position.z = MovingAvgeFilter(msg->data[i].pose.position.z, 500);
            // Data Offset
            setOffset(tag{pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});

            pose_pub.publish(pose);
            break;
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "manta_positioning");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(10);

    string topic = "coord";
    nh.getParam("tag_id", tag_id);

    ros::Subscriber obstacle_sub = nh.subscribe("/mqtt_coord", 10, poseCallback);  // Data from mqtt protocol
    // ros::Subscriber obstacle_sub = nh.subscribe("/coord", 10, poseCallback);    // Data from raspberry pi / arduino

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic, 10);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}