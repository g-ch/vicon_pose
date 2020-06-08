#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>
#include "px4_gcs/GCSMessage.h"
#include "tf/transform_datatypes.h"

using namespace std;
string node_name;
float start_angle[3];
float cur_pos[3];
float sp_pos[3];
float sp_rpyt[4];
double cur_angle[3];
double send_angle[3];
double init_q_data[4];


static int _count = 0;
tf::Quaternion cur_q;
tf::Quaternion init_q;
tf::Quaternion send_q;





template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    name = node_name+"/"+name;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}


void drone_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cur_pos[0] = msg->pose.position.x;
    cur_pos[1] = msg->pose.position.y;
    cur_pos[2] = msg->pose.position.z;
    // cur_angle[0] = msg->pose.orientation.x;
    // cur_angle[1] = msg->pose.orientation.y;
    // cur_angle[2] = msg->pose.orientation.z;
    cur_q.setX(msg->pose.orientation.x);
    cur_q.setY(msg->pose.orientation.y);
    cur_q.setZ(msg->pose.orientation.z);
    cur_q.setW(msg->pose.orientation.w);
    while(_count<100){
        ++_count;
        init_q_data[0] += msg->pose.orientation.x;
        init_q_data[1] += msg->pose.orientation.y;
        init_q_data[2] += msg->pose.orientation.z;
        init_q_data[3] += msg->pose.orientation.w;

        // start_angle[0] += (float)cur_angle[0];
        // start_angle[1] += (float)cur_angle[1];
        // start_angle[2] += (float)cur_angle[2];
    }
    if(_count==100){
        ++_count;
        init_q_data[0] /= 100;
        init_q_data[3] /= 100;
        init_q_data[1] /= 100;
        init_q_data[2] /= 100;
        // start_angle[0] /=100;
        // start_angle[1] /=100;
        // start_angle[2] /=100;
        init_q.setX(init_q_data[0]);
        init_q.setY(init_q_data[1]);
        init_q.setZ(init_q_data[2]);
        init_q.setW(init_q_data[3]);
    }
    cur_q*=init_q.inverse();
    tf::Matrix3x3(cur_q).getEulerYPR(send_angle[2],send_angle[1],send_angle[0]);
    // cur_angle[0] = cur_angle[0]-start_angle[0];
    // cur_angle[1] = cur_angle[1]-start_angle[1];
    // cur_angle[2] = cur_angle[2]-start_angle[2];
    ROS_INFO("POS: x: %+2.4f,y :%+2.4f,z :%+2.4f",cur_pos[0],cur_pos[1],cur_pos[2]);
    ROS_INFO("RPY: r: %+2.4f,p :%+2.4f,y :%+2.4f",send_angle[0],send_angle[1],send_angle[2]);
    ROS_INFO("SRPY: r: %+2.4f,p :%+2.4f,y :%+2.4f",start_angle[0],start_angle[1],start_angle[2]);
    ROS_INFO("CRPY: r: %+2.4f,p :%+2.4f,y :%+2.4f",cur_angle[0],cur_angle[1],cur_angle[2]);
}

void out_sp_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    sp_pos[0] = msg->pose.position.x;
    sp_pos[1] = msg->pose.position.y;
    sp_pos[2] = msg->pose.position.z;
    sp_rpyt[0] = msg->pose.orientation.x;
    sp_rpyt[1] = msg->pose.orientation.y;
    sp_rpyt[2] = msg->pose.orientation.z;
    sp_rpyt[3] = msg->pose.orientation.w;
}



int main(int argc,char ** argv){
    ros::init(argc, argv, "drone_name");
    ros::NodeHandle nh;
    MODE drone_mode[] = {GPS,VICON,CAMERA};
    SEND_CURRENT_STATE drone_state[] = {SEND_NED_POSITION,SEND_NED_POSITION_RPY,SEND_NED_RPY};
    SETPOINT_TYPE drone_sp[] = {LOCAL_POSITION_SP,GLOBAL_POSITION_SP,ATTITUDE_SP,VELOCITY_SP};
    node_name = ros::this_node::getName();
    geometry_msgs::PoseStamped backed_pose;

    start_angle[0] = 0.0f;
    start_angle[1] = 0.0f;
    start_angle[2] = 0.0f;

    auto id = readParam<int>(nh,"id");
    auto current_mode = readParam<int>(nh,"current_mode");
    auto current_sp = readParam<int>(nh,"current_sp");
    auto current_state = readParam<int>(nh,"current_state");
    auto baudrate = readParam<int>(nh, "baudrate");
    auto serial_timeout = readParam<int>(nh,"serial_timeout");
    auto rate = readParam<int>(nh,"rate");
    auto takeoff_height = readParam<float>(nh,"takeoff_height");
    auto circle_d = readParam<float>(nh,"circle_d");
    auto init_pose_x = readParam<float>(nh,"init_pose_x");
    auto init_pose_y = readParam<float>(nh,"init_pose_x");
    auto sub_pos_topic_name = readParam<string>(nh,"sub_pos_topic_name");
    auto use_out_sp = readParam<int>(nh,"use_out_sp");
    auto out_sp_name = readParam<string>(nh,"out_sp_name");
    std::string serial_name = readParam<string>(nh,"serial_name");

    ROS_INFO("params loaded");

    string buffer;
    buffer = "drone"; buffer+=to_string(id);
    ROS_INFO("drone + %d",id);

    ros::Publisher de_pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>(buffer+"/sp",1);
    ros::Publisher drone_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(buffer+"/pose",1);
    ros::Subscriber drone_pos_sub = nh.subscribe(sub_pos_topic_name,1, drone_pos_cb);
    if(use_out_sp==1){
        ros::Subscriber out_sp_sub = nh.subscribe(out_sp_name,1, out_sp_cb);
    }


    serial::Serial drone_port;


    try{
        ROS_INFO("Open Drone %d serial device",id);
        drone_port.setPort(serial_name);
        drone_port.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(serial_timeout);
        drone_port.setTimeout(to);
        drone_port.open();
    }

    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }
    if (drone_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    } else
    {
        ROS_ERROR_STREAM("No port opened");
    }



    ROS_INFO("Open Drone %d serial device Sucessefully !!",id);

    GCSMessageManager gcsMessageManager(drone_port);
    CMDMessage cmd_msg(drone_mode[current_mode]);
    CSTMessage cst_msg(drone_state[current_state]);
    SPMessage sp_msg(drone_sp[current_sp]);
    UAVMessage recv_msg;

    ros::Rate loop_rate(rate);

        // cmd_msg.armCommand = ARM_DO_NOTHING;
        // cmd_msg.backInfo = ONLY_POSITION;
        // cmd_msg.takeoffCommand = TAKEOFF;
        // cmd_msg.offboardCommand = OFF_DO_NOTHING;
        // gcsMessageManager.sendMsg(cmd_msg);
        //
        // cmd_msg.armCommand = ARMED;
        // cmd_msg.backInfo = ONLY_POSITION;
        // cmd_msg.takeoffCommand = TAKEOFF;
        // cmd_msg.offboardCommand = TRY_IN;
        // gcsMessageManager.sendMsg(cmd_msg);
        //
        // cmd_msg.armCommand = ARMED;
        // cmd_msg.backInfo = ONLY_POSITION;
        // cmd_msg.takeoffCommand = TAKEOFF;
        // cmd_msg.offboardCommand = STAY_IN;
        // gcsMessageManager.sendMsg(cmd_msg);
        // ros::spinOnce();
        // double start = ros::Time::now().toSec();
        // float z_sp = 0.7f;

        while(ros::ok()){

            // cmd_msg.armCommand = ARMED;
            // cmd_msg.backInfo = ONLY_POSITION;
            // cmd_msg.takeoffCommand = TAKEOFF_DO_NOTHING;
            // cmd_msg.offboardCommand = TRY_IN;
            // gcsMessageManager.sendMsg(cmd_msg);

            cst_msg.sendCurrentState = SEND_NED_POSITION_RPY;
            cst_msg.position = {cur_pos[0],cur_pos[1],cur_pos[2]};
            cst_msg.rpy = {send_angle[0],send_angle[1],send_angle[2]};
            gcsMessageManager.sendMsg(cst_msg);
            recv_msg = gcsMessageManager.receiveOneMsg();
            //publish
            backed_pose.header.stamp = ros::Time::now();
            backed_pose.pose.position.x = recv_msg.position[0]+init_pose_x;
            backed_pose.pose.position.y = recv_msg.position[1]+init_pose_y;
            backed_pose.pose.position.z = recv_msg.position[2];
            drone_pos_pub.publish(backed_pose);

            // if(use_out_sp==1){
            //     sp_msg.rpyt = {sp_rpyt[0],sp_rpyt[1],sp_rpyt[2],sp_rpyt[3]};
            //     sp_msg.xyz = {sp_pos[0]-init_pose_x,sp_pos[1]-init_pose_y,sp_pos[2]};
            // }else{
            // sp_msg.xyz = {0,0,z_sp};
            // sp_msg.setpointType = ATTITUDE_SP;
            // sp_msg.rpyt = {0,0,0,0.2};
            // // }
            // //
            // gcsMessageManager.sendMsg(sp_msg);
            // recv_msg = gcsMessageManager.receiveOneMsg();
            // //publish
            // backed_pose.header.stamp = ros::Time::now();
            // backed_pose.pose.position.x = recv_msg.position[0]+init_pose_x;
            // backed_pose.pose.position.y = recv_msg.position[1]+init_pose_y;
            // backed_pose.pose.position.z = recv_msg.position[2];
            // drone_pos_pub.publish(backed_pose);
            // if(ros::Time::now().toSec()-start>20.0f){
            //     z_sp = 0.0;
            // }

            loop_rate.sleep();
            ros::spinOnce();
        }
}





