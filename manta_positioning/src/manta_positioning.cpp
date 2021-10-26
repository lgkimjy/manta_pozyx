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
    vector<string> docs = {"tag 0", "tag 1", "tag 2"};

    tags.clear();
    for(int i=0; i<docs.size(); i++)
    {
        tag temp;
        temp.id = lpf_doc[docs[i]]["id"].as<string>();
        temp.x_alpha = lpf_doc[docs[i]]["x_alpha"].as<float>();
        temp.y_alpha = lpf_doc[docs[i]]["y_alpha"].as<float>();
        temp.z_alpha = lpf_doc[docs[i]]["z_alpha"].as<float>();
        tags.push_back(temp);
    }
}

void updateConfigData()
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
    vector<string> docs = {"tag 0", "tag 1", "tag 2"};

    for(int i=0; i<docs.size(); i++)
    {
        tags[i].x_alpha = lpf_doc[docs[i]]["x_alpha"].as<float>();
        tags[i].y_alpha = lpf_doc[docs[i]]["y_alpha"].as<float>();
        tags[i].z_alpha = lpf_doc[docs[i]]["z_alpha"].as<float>();
    }
}

float LowPassFilter(tag tag_data, int index)
{
    float output;
    // cout << "tag id : " << tag_data.z << " : " << tag_data.z_alpha << " : " << tag_data.prev_raw_value << " " << index << endl;
    output = tag_data.z_alpha * tag_data.z + (1.0 - tag_data.z_alpha) * tag_data.prev_raw_value;
    tags[index].prev_raw_value = output;

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
    manta_positioning::mqtt_msg pose_msg;

    updateConfigData();

    if(flag == 0){
        for(int i=0; i<msg->data.size(); i++){
            for(int j=0; j<tags.size(); j++){
                if(tags[j].id == msg->data[i].header.frame_id){
                    tags[j].prev_raw_value = msg->data[i].pose.position.z;
                }
            }
        }
        flag++;
    }

    for(int i=0; i<msg->data.size(); i++){
        for(int j=0; j<tags.size(); j++){
            if(tags[j].id == msg->data[i].header.frame_id){
                tags[j].x = msg->data[i].pose.position.x;
                tags[j].y = msg->data[i].pose.position.y;
                tags[j].z = msg->data[i].pose.position.z;
            }
            // cout << j << endl;
        }
    }

    for(int i=0; i<msg->data.size(); i++){
        pose = msg->data[i];                                                            // data copy (deep copy)
        for(int j=0; j<tags.size(); j++){
            if(tags[j].id == msg->data[i].header.frame_id){
                // Filtering
                pose.pose.position.z = LowPassFilter(tags[j], j);                       // Low Pass Filter z
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

    readConfigData();

     while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}