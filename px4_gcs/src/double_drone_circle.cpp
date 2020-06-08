#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <iostream>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>
#include "px4_gcs/GCSMessage.h"

using namespace std;

#define REFRESH_RATE 30



bool tryOpenSerial(serial::Serial &serialport,string &dev_name,int baudrate = 57600,int _timeout = 1000){
    serialport.setPort(dev_name);
    serialport.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(_timeout);
    serialport.setTimeout(to);
    serialport.open();
    return serialport.isOpen();
}

int main(int argc,char ** argv){
    ros::init(argc, argv, "double_drone_circle");
    ros::NodeHandle nh;
    MODE drone_mode[] = {GPS,VICON,CAMERA};
    SEND_CURRENT_STATE drone_state[] = {SEND_NED_POSITION,SEND_NED_POSITION_RPY,SEND_NED_RPY};
    SETPOINT_TYPE drone_sp[] = {LOCAL_POSITION_SP,GLOBAL_POSITION_SP,ATTITUDE_SP,VELOCITY_SP};

    int drone_num;
    int takeoff_height;
    int circle_d;
    int current_mode;
    int current_sp;
    int current_state;
    int all_buadrate;
    int serial_timeout;
    vector<string> serial_name;


    nh.getParam("drone_num",drone_num);
    nh.getParam("takeoff_height",takeoff_height);
    nh.getParam("circle_d",circle_d);
    nh.getParam("serial_name",serial_name);
    nh.getParam("current_mode",current_mode);
    nh.getParam("current_sp",current_sp);
    nh.getParam("all_buadrate",all_buadrate);
    nh.getParam("serial_timeout",serial_timeout);
    nh.getParam("current_state",current_state);


    vector<ros::Publisher> drone_pos_pub_by_id(drone_num,ros::Publisher());
    vector<ros::Publisher> drone_pos_sp_pub_by_id(drone_num,ros::Publisher());
    vector<CMDMessage> CMDMessage_in_drone_id(drone_num,CMDMessage(drone_mode[current_mode]));
    vector<CSTMessage> CSTMessage_in_drone_id(drone_num,CSTMessage(drone_state[current_state]));
    vector<SPMessage> SPMessage_in_drone_id(drone_num,SPMessage(drone_sp[current_sp]));
    vector<serial::Serial> drone_port(drone_num);



    ROS_INFO("Now, Init %d Drone serial",drone_num);
    for (int j = 0; j < drone_num; ++j) {
        ROS_INFO("Please plug in computer Drone %d serial device",j);
        if(tryOpenSerial(drone_port[j],serial_name[j],57600,serial_timeout)){
            ROS_INFO(" Drone %d serial device open sucessed!",j);
        } else{
            do{
                ROS_ERROR("Reopen Drone %d device",j);
                tryOpenSerial(drone_port[j],serial_name[j],57600,serial_timeout);
            }while (!drone_port[j].isOpen());
        }
    }

    ROS_INFO("Now, Process publish msg");
    for (int j = 0; j < drone_num; ++j) {
        string buffer;
        buffer = "drone";buffer+=std::to_string(j);
        buffer += "/pose";
        drone_pos_pub_by_id[j] = nh.advertise<geometry_msgs::PoseStamped>(buffer,1);
        drone_pos_sp_pub_by_id[j] = nh.advertise<geometry_msgs::PoseStamped>(buffer,1);
    }

    ros::spinOnce();
    while (ros::ok()) {



    }


}






