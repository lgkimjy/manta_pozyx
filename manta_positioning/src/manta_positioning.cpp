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
        ROS_INFO("[Positioning Node] Read Config Data");
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

bool computeOrientation(
    WorldFrame::WorldFrame frame,
    geometry_msgs::Vector3 A,
    geometry_msgs::Vector3 E,
    geometry_msgs::Quaternion &orientation)
{
    float Hx, Hy, Hz;
    float Mx, My, Mz;
    float normH;

    // A: pointing up 
    // E: pointing down/north
    float Ax = A.x, Ay = A.y, Az = A.z;
    float Ex = E.x, Ey = E.y, Ez = E.z;
    // H: vector horizontal, pointing east
    // H = E x A
    crossProduct(Ex, Ey, Ez, Ax, Ay, Az, Hx, Hy, Hz);

    // normalize H
    normH = normalizeVector(Hx, Hy, Hz);
    if (normH < 1E-7) return false;       // device is close to free fall or close to magnetic north pole. typical threshold values are > 1E-5.
    // normalize A
    normalizeVector(Ax, Ay, Az);
    // M: vector horizontal, pointing north
    // M = A x H
    crossProduct(Ax, Ay, Az, Hx, Hy, Hz, Mx, My, Mz);

    // Create matrix for basis transformation
    tf2::Matrix3x3 R;
    switch (frame)
    {
    case WorldFrame::NED:
        // R: Transform Matrix local => world equals basis of L, because basis of W is I
        R[0][0] = Mx;  R[0][1] = Hx;  R[0][2] = -Ax;
        R[1][0] = My;  R[1][1] = Hy;  R[1][2] = -Ay;
        R[2][0] = Mz;  R[2][1] = Hz;  R[2][2] = -Az;
        break;
    case WorldFrame::NWU:
        // R: Transform Matrix local => world equals basis of L, because basis of W is I
        R[0][0] = Mx;  R[0][1] = -Hx;  R[0][2] = Ax;
        R[1][0] = My;  R[1][1] = -Hy;  R[1][2] = Ay;
        R[2][0] = Mz;  R[2][1] = -Hz;  R[2][2] = Az;
        break;
    default:
    case WorldFrame::ENU:
        // R: Transform Matrix local => world equals basis of L, because basis of W is I
        R[0][0] = Hx;  R[0][1] = Mx;  R[0][2] = Ax;
        R[1][0] = Hy;  R[1][1] = My;  R[1][2] = Ay;
        R[2][0] = Hz;  R[2][1] = Mz;  R[2][2] = Az;
        break;
    }
    // Matrix.getRotation assumes vector rotation, but we're using coordinate systems. Thus negate rotation angle (inverse).
    tf2::Quaternion q;
    R.getRotation(q);
    tf2::convert(q.inverse(), orientation);
    return true;
}

double Convert(double radian)
{
    double pi = 3.14159;
    return(radian * (180 / pi));
}

void imuMagPoseCallback(
    const manta_positioning::mqtt_imu::ConstPtr &imu_msg_raw, 
    const manta_positioning::mqtt_mag::ConstPtr &mag_msg,
    const manta_positioning::mqtt_msg::ConstPtr &msg)
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

    ROS_INFO("----------------------------------------------------------");
    for(int i=0; i<msg->data.size(); i++){
        pose = msg->data[i];                                                  // data copy (deep copy)
        for(int j=0; j<tags.size(); j++){
            if(tags[j].id == msg->data[i].header.frame_id){
                tags[j].pose[0] = msg->data[i].pose.position.x;
                tags[j].pose[1] = msg->data[i].pose.position.y;
                tags[j].pose[2] = msg->data[i].pose.position.z;

                // Filtering (LPF)
                pose.pose.position.x = LowPassFilter(tags[j], j, 0);          // Low Pass Filter x
                pose.pose.position.y = LowPassFilter(tags[j], j, 1);          // Low Pass Filter y
                pose.pose.position.z = LowPassFilter(tags[j], j, 2);          // Low Pass Filter z

                // Offset
                pose.pose.position.x = setoffset(tags[j], j, 0);              // offset x
                pose.pose.position.y = setoffset(tags[j], j, 1);              // offset y
                pose.pose.position.z = setoffset(tags[j], j, 2);              // offset z
            
                geometry_msgs::Vector3 ang_vel = imu_msg_raw->data[i].angular_velocity;
                geometry_msgs::Vector3 lin_acc = imu_msg_raw->data[i].linear_acceleration;
                geometry_msgs::Vector3 mag_fld = mag_msg->data[i].magnetic_field;

                double roll, pitch, yaw = 0.0;
                // wait for mag message without NaN / inf
                if (!isfinite(mag_fld.x) || !isfinite(mag_fld.y) || !isfinite(mag_fld.z)) return;

                geometry_msgs::Quaternion init_q;
                if (!computeOrientation(WorldFrame::ENU, lin_acc, mag_fld, init_q))
                {
                    ROS_WARN_THROTTLE(5.0, "[%s] The IMU seems to be in free fall or close to magnetic north pole, cannot determine gravity direction!", msg->data[i].header.frame_id.c_str());
                    return;
                }
                // ROS_INFO("[%s] %f | %f | %f | %f ", msg->data[i].header.frame_id.c_str(), init_q.x, init_q.y, init_q.z,init_q.w);
                tf2::Matrix3x3(tf2::Quaternion(init_q.x, init_q.y, init_q.z,init_q.w)).getRPY(roll, pitch, yaw, 0);
                ROS_INFO("[%s] %f | %f | %f ", msg->data[i].header.frame_id.c_str() ,Convert(roll), Convert(pitch), Convert(yaw));

                /* 
                  have to apply madgwickAHRSupdateIMU function to calculate absolute heading 
                */

                pose.pose.orientation.x = Convert(roll);
                pose.pose.orientation.y = Convert(pitch);
                pose.pose.orientation.z = Convert(yaw);
            }
        }
        pose_msg.data.push_back(pose);
    }
    pose_pub.publish(pose_msg);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manta_positioning_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ROS_INFO("[Positioning Node] Initialize");

    int queue_size = 5;
    // ros::Subscriber obstacle_sub = nh.subscribe("/mqtt_coord", queue_size, poseCallback); // Data from mqtt protocol
    message_filters::Subscriber<manta_positioning::mqtt_imu> image_sub(nh, "/mqtt_imu", 1);
    message_filters::Subscriber<manta_positioning::mqtt_mag> mag_sub(nh, "/mqtt_mag", 1);
    message_filters::Subscriber<manta_positioning::mqtt_msg> pose_sub(nh, "/mqtt_coord", 1);
    TimeSynchronizer<manta_positioning::mqtt_imu, manta_positioning::mqtt_mag, manta_positioning::mqtt_msg> sync(image_sub, mag_sub, pose_sub, 10);
    sync.registerCallback(boost::bind(imuMagPoseCallback, _1, _2, _3));

    pose_pub = nh.advertise<manta_positioning::mqtt_msg>("/filtered/mqtt_coord", 10);

    path_lpf = ros::package::getPath("manta_positioning") + "/config/LPF.yaml";
    path_offset = ros::package::getPath("manta_positioning") + "/config/offset.yaml";
    docs = {"tag 101", "tag 102", "tag 103", "tag 104", "tag 105", "tag 106", "tag 107", "tag 108", "tag 109", "tag 110",
            "tag 111", "tag 112", "tag 113", "tag 114", "tag 115", "tag 116", "tag 117", "tag 118", "tag 119", "tag 120",
            "tag 121", "tag 122", "tag 123", "tag 124", "tag 125", "tag 126", "tag 127", "tag 128", "tag 129", "tag 130",
            "tag 131", "tag 132", "tag 133", "tag 134", "tag 135", "tag 136", "tag 137", "tag 138", "tag 139", "tag 140",
            "tag 141", "tag 142", "tag 143", "tag 144", "tag 145", "tag 146", "tag 147", "tag 148", "tag 149", "tag 150"};
    readConfigData();

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}