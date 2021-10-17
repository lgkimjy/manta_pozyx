/*
 * manta_positioning_association.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "manta_positioning/manta_positioning_association.h"

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
    filtered_msg.data.push_back(pose);
}

void controlFunction(const ros::TimerEvent&)
{
    pose_pub.publish(filtered_msg);
    filtered_msg.data.clear();
}

int main(int argc, char **argv){

    ros::init(argc, argv, "manta_positioning_association");
    ros::NodeHandle nh; 
    ros::Rate loop_rate(10);

    ros::Subscriber sub1, sub2;
    vector<ros::Subscriber> list;

    list.push_back(sub1);
    list.push_back(sub2);

    for(int i=0; i<list.size(); i++){
        string data = "coord_" + to_string(i);
        list[i] = nh.subscribe(data, 10, poseCallback);
    }

    ros::Timer timer_control = nh.createTimer(ros::Duration(0.025), controlFunction); // 25ms

    pose_pub = nh.advertise<manta_positioning::mqtt_msg>("/filtered/coord_list", 10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}