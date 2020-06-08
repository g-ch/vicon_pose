#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <iostream>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>
#include "px4_gcs/GCSMessage.h"


using namespace std;

#define REFRESH_RATE 30






int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_test");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(REFRESH_RATE);

    serial::Serial serialport;
    std::string dev_name = "/dev/ttyUSB0";
    int baudrate = 57600;


    
    try
    {
        serialport.setPort(dev_name);
        serialport.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serialport.setTimeout(to);
        serialport.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }
    if (serialport.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    } else
    {
        ROS_ERROR_STREAM("No port opened");
    }

    GCSMessageManager gcsMessageManager(serialport);
    gcsMessageManager.showOutput = false;

    ros::Publisher cam_reset_origin_pub = nodeHandle.advertise<geometry_msgs::PoseStamped> ("/cam_reset_origin/pose", 1);
    ros::Publisher vicon_reset_origin_pub = nodeHandle.advertise<geometry_msgs::PoseStamped> ("/vicon_reset_origin/pose", 1);
    ros::Publisher pos_setpoint_pub = nodeHandle.advertise<geometry_msgs::PoseStamped> ("/pos_setpoint/pose", 1);


    ros::spinOnce();
    CMDMessage cmd(VICON);
    CSTMessage cstMessage(SEND_NED_POSITION_RPY);
    SPMessage spMessage(ATTITUDE_SP);

    while (ros::ok())
    {

        cmd.backInfo = POSITION_AND_IMU;
        gcsMessageManager.sendMsg(cmd);
        cstMessage.position = {1,2,3};
        cstMessage.rpy = {0.1,-0.2,0.3};
        gcsMessageManager.sendMsg(cstMessage);
        auto msg = gcsMessageManager.receiveOneMsg();
        cout << msg.position[0] << " " << msg.position[1] << " " << msg.position[2] << " "
                << msg.acceleration[0] << " " << msg.acceleration[1] << " " << msg.acceleration[2] << "   ###   ";
        spMessage.rpyt = {-4,-5,-6,0.5};
        gcsMessageManager.sendMsg(spMessage);
        msg = gcsMessageManager.receiveOneMsg();
        cout << msg.position[0] << " " << msg.position[1] << " " << msg.position[2] << " "
             << msg.acceleration[0] << " " << msg.acceleration[1] << " " << msg.acceleration[2] << endl;
        loop_rate.sleep();
        ros::spinOnce();

    }

    return 0;
}
