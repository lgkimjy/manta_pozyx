/*
 * manta_positioning.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "manta_positioning/manta_positioning.h"

void readConfigData()
{
    try{
        lpf_doc = YAML::LoadFile(path_lpf.c_str());
        offset_doc = YAML::LoadFile(path_offset.c_str());
    }
    catch(const std::exception& e){
        ROS_ERROR("Fail to load Config yaml file!");
        return;
    }

    tags.clear();
    for(int i=0; i<docs.size(); i++)
    {
        tag temp;
        temp.id = lpf_doc[docs[i]]["id"].as<string>();
        temp.alpha[0] = lpf_doc[docs[i]]["x_alpha"].as<float>();
        temp.alpha[1] = lpf_doc[docs[i]]["y_alpha"].as<float>();
        temp.alpha[2] = lpf_doc[docs[i]]["z_alpha"].as<float>();
        temp.offset[0] = offset_doc[docs[i]]["x"].as<float>();
        temp.offset[1] = offset_doc[docs[i]]["y"].as<float>();
        temp.offset[2] = offset_doc[docs[i]]["z"].as<float>();
        tags.push_back(temp);
    }
}

void updateConfigData()
{
    try{
        lpf_doc = YAML::LoadFile(path_lpf.c_str());
        offset_doc = YAML::LoadFile(path_offset.c_str());
    }
    catch(const std::exception& e){
        ROS_ERROR("Fail to load Config yaml file!");
        return;
    }

    for(int i=0; i<docs.size(); i++)
    {
        tags[i].alpha[0] = lpf_doc[docs[i]]["x_alpha"].as<float>();
        tags[i].alpha[1] = lpf_doc[docs[i]]["y_alpha"].as<float>();
        tags[i].alpha[2] = lpf_doc[docs[i]]["z_alpha"].as<float>();
        tags[i].offset[0] = offset_doc[docs[i]]["x"].as<float>();
        tags[i].offset[1] = offset_doc[docs[i]]["y"].as<float>();
        tags[i].offset[2] = offset_doc[docs[i]]["z"].as<float>();
    }
}

float LowPassFilter(tag tag_data, int index, int num)
{
    float output;
    output = tag_data.alpha[num] * tag_data.pose[num] + (1.0 - tag_data.alpha[num]) * tag_data.prev_raw_value[num];
    tags[index].pose[num] = output;
    tags[index].prev_raw_value[num] = output;

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

float setoffset(tag tag_data, int index, int num)
{
    float output;
    output = tag_data.pose[num] - tag_data.offset[num];
    tags[index].pose[num] = output;

    return output;
}

void poseCallback(const manta_positioning::mqtt_msg::ConstPtr& msg)
{
    updateConfigData();
    pose_msg.data.clear();

    if(flag == 0){
        for(int i=0; i<msg->data.size(); i++){
            for(int j=0; j<tags.size(); j++){
                if(tags[j].id == msg->data[i].header.frame_id){
                    tags[j].prev_raw_value[0] = msg->data[i].pose.position.x;
                    tags[j].prev_raw_value[1] = msg->data[i].pose.position.y;
                    tags[j].prev_raw_value[2] = msg->data[i].pose.position.z;
                }
            }
        }
        flag++;
    }

    for(int i=0; i<msg->data.size(); i++){
        pose = msg->data[i];                                                  // data copy (deep copy)
        for(int j=0; j<tags.size(); j++){
            if(tags[j].id == msg->data[i].header.frame_id){
                tags[j].pose[0] = msg->data[i].pose.position.x;
                tags[j].pose[1] = msg->data[i].pose.position.y;
                tags[j].pose[2] = msg->data[i].pose.position.z;

                // Filtering
                pose.pose.position.x = LowPassFilter(tags[j], j, 0);          // Low Pass Filter x
                pose.pose.position.y = LowPassFilter(tags[j], j, 1);          // Low Pass Filter y
                pose.pose.position.z = LowPassFilter(tags[j], j, 2);          // Low Pass Filter z

                // Offset
                pose.pose.position.x = setoffset(tags[j], j, 0);              // offset x
                pose.pose.position.y = setoffset(tags[j], j, 1);              // offset y
                pose.pose.position.z = setoffset(tags[j], j, 2);              // offset z
            }
        }
        pose_msg.data.push_back(pose);
    }
    pose_pub.publish(pose_msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "manta_positioning");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(10);

    ros::Subscriber obstacle_sub = nh.subscribe("/mqtt_coord", 10, poseCallback);  // Data from mqtt protocol
    pose_pub = nh.advertise<manta_positioning::mqtt_msg>("/filtered/mqtt_coord", 10);

    path_lpf = ros::package::getPath("manta_positioning")+"/config/LPF.yaml";
    path_offset = ros::package::getPath("manta_positioning")+"/config/offset.yaml";
    docs = {"tag 0", "tag 1", "tag 2", "tag 3", "tag 4", "tag 5", "tag 6", "tag 7", "tag 8", "tag 9"};
    readConfigData();

     while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}