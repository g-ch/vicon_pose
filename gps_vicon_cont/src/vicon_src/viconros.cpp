#include "ros/ros.h"
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h> 
#include <geometry_msgs/TwistStamped.h> 
#include <sstream>
#include <string.h>
#include "CFetchViconData.h"
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;

string node_name;

int main(int argc, char **argv) {
	ros::init(argc, argv, "viconros_201");
	ros::NodeHandle n("~");
    //>>>>>>>>>>>>初始化<<<<<<<<<<<<<
    std::string ip;
	std::string model;
	std::string segment;
	std::string pose_topic_name;
	std::string vel_topic_name;

	bool undo_correct_q = true;
	int corret_num = 0;
	int x_convert,y_convert,z_convert;
	int sgn_x,sgn_y,sgn_z,x_index,y_index,z_index;
	int print_origin_data,print_convert_data,print_origin_angle,print_q_angle;
    tf2::Quaternion q;
    tf2::Quaternion _q;
    tf2::Quaternion init_invq;
    float init_q[4] = {0.0,0.0,0.0,0.0};
    n.param<std::string>("host",ip,std::string(""));
    n.param<std::string>("segment",segment,std::string(""));
    n.param<std::string>("model",model,std::string(""));
    n.param<std::string>("pose_topic_name",pose_topic_name,std::string(""));
    n.param<std::string>("vel_topic_name",vel_topic_name,std::string(""));
	n.getParam("x_convert",x_convert);
	n.getParam("y_convert",y_convert);
	n.getParam("z_convert",z_convert);
	n.getParam("print_origin_data",print_origin_data);
	n.getParam("print_convert_data",print_convert_data);
	n.getParam("print_origin_angle",print_origin_angle);
	n.getParam("print_q_angle",print_q_angle);
	ROS_INFO("HOST:%s",ip.c_str());
	ROS_INFO("MODEL:%s; SEGMENT:%s",model.c_str(),segment.c_str());
	ROS_INFO_STREAM("x,y,z convert : "<<x_convert<<","<<y_convert<<","<<z_convert);

	ros::Publisher vicon_pub = n.advertise<geometry_msgs::PoseStamped> (pose_topic_name, 1);
	ros::Publisher vicon_pub1 = n.advertise<geometry_msgs::TwistStamped> (vel_topic_name, 1);
	sgn_x = x_convert>0?1:-1;
	sgn_y = y_convert>0?1:-1;
    sgn_z = z_convert>0?1:-1;
	ROS_INFO_STREAM("sgn_x,y,z: "<< sgn_x<<sgn_y<<sgn_z);
    x_index = abs(x_convert)-1;y_index = abs(y_convert)-1;z_index = abs(z_convert)-1;

	ros::Rate loop_rate(40);

	auto * vicon=new CFetchViconData();
	const char * host=ip.c_str();
	ObjStatus objs;
	
	if(!(vicon->IsConnected))
    { 
            ROS_INFO("Connecting to %s",host);
            bool res=vicon->Connect(host);
            if(!res)
            {
                ROS_INFO("Failed to connect!\r\n");
                    return 0;
            }
            else
            {
                ROS_INFO("Successfully connected!\r\n");
            }

    }
	while (ros::ok()) {
        vicon->GetStatus(objs, model.c_str(),segment.c_str());
        if(undo_correct_q){
            if(corret_num<9){
                init_q[0]+=objs.ort[0];//x
                init_q[1]+=objs.ort[1];//y
                init_q[2]+=objs.ort[2];//z
                init_q[3]+=objs.ort[3];//w
                ++corret_num;
                continue;
            }else{
                init_invq[0] = -init_q[0];
                init_invq[1] = -init_q[1];
                init_invq[2] = -init_q[2];
                init_invq[3] = init_q[3];
                init_invq.normalize();
                undo_correct_q = false;
            }
        }else{
            geometry_msgs::PoseStamped msg;
            geometry_msgs::TwistStamped msg1;
            msg.header.stamp.sec=(int)objs.tm;
            msg.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;

            msg.pose.position.x = (sgn_x)*objs.pos[x_index];
            msg.pose.position.y = (sgn_y)*objs.pos[y_index];
            msg.pose.position.z = (sgn_z)*objs.pos[z_index];

            msg1.header.stamp.sec=(int)objs.tm;
            msg1.header.stamp.nsec=(objs.tm-msg.header.stamp.sec)*10000*100000;

            _q[0] = objs.ort[0];
            _q[1] = objs.ort[1];
            _q[2] = objs.ort[2];
            _q[3] = objs.ort[3];
            _q.normalize();
            q = _q*init_invq;
            q.normalize();
            msg.pose.orientation.x = sgn_x*_q[x_index];
            msg.pose.orientation.y = sgn_y*_q[y_index];
            msg.pose.orientation.z = sgn_z*_q[z_index];
            msg.pose.orientation.w = _q[3];
            // msg.pose.orientation.x = sgn_x*objs.euler[x_index];
            // msg.pose.orientation.y = sgn_y*objs.euler[y_index];
            // msg.pose.orientation.z = sgn_z*objs.euler[z_index];
            // msg.pose.orientation.w = 0;

            msg1.twist.linear.x = (sgn_x)*objs.vel[x_index];
            msg1.twist.linear.y = (sgn_y)*objs.vel[y_index];
            msg1.twist.linear.z = (sgn_z)*objs.vel[z_index];

            if(print_convert_data==1)ROS_INFO_STREAM(std::setprecision(5)
                                                             <<"Convert pos:  X:"<< fixed << setprecision(5)<<msg.pose.position.x<<
                                                             " Y: "<< fixed << setprecision(5)<<msg.pose.position.y<<
                                                             " Z: "<< fixed << setprecision(5)<<msg.pose.position.z);
            if(print_origin_data==1)ROS_INFO_STREAM("origin data"<<objs.pos[0]<<
                                                                 " Y: "<<objs.pos[1]<<" Z: "<<objs.pos[2]);
            if(print_origin_angle==1)ROS_INFO_STREAM("origin angle"<<showpos<<fixed << setprecision(5)<<objs.euler[0]<<
                                                                   " Y: "<<showpos<<fixed << setprecision(5)<<objs.euler[1]<<
                                                                   " Z: "<<showpos<<fixed << setprecision(5)<<objs.euler[2]);
            if(print_q_angle==1)ROS_INFO_STREAM("q angle X:" <<showpos<<fixed << setprecision(5)<<sgn_x*q[x_index]<<
                                                            " Y: "<<showpos<<fixed << setprecision(5)<<sgn_y*q[y_index]<<
                                                            " Z: "<<showpos<<fixed << setprecision(5)<<sgn_z*q[z_index]<<
                                                            " w: "<<showpos<<fixed << setprecision(5)<<q[3]);
            vicon_pub.publish(msg);
            vicon_pub1.publish(msg1);
            ros::spinOnce();
            loop_rate.sleep();
        }

	}
	return 0;
}
