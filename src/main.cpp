#include "InputWindow.h"
#include "ManualControl.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include <ros/ros.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "bebop_teleop");
	ros::NodeHandle nh;
	ManualControl control;
	control.setPub( VELOCITY, nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1) );
	control.setPub( TAKEOFF, nh.advertise<std_msgs::Empty>("bebop/takeoff", 1) );
	control.setPub( LAND, nh.advertise<std_msgs::Empty>("bebop/land", 1) );
	control.setPub( RESET, nh.advertise<std_msgs::Empty>("bebop/reset", 1) );
	control.setPub( CAMERA, nh.advertise<geometry_msgs::Twist>("bebop/camera_control", 1) );
	control.setPub( SNAPSHOT, nh.advertise<std_msgs::Empty>("bebop/snapshot", 1) );
	control.setPub( RECORD, nh.advertise<std_msgs::Bool>("bebop/record", 1) );
	control.setPub( FLIP, nh.advertise<std_msgs::UInt8>("bebop/flip", 1) );
	control.setPub( HOME, nh.advertise<std_msgs::Bool>("bebop/autoflight/navigate_home", 1) );

	ros::Rate r(60);

	bool pressed;
	InputWindow input(pressed);
	if(pressed) {
		ROS_ERROR("SDL INITIALIZER ERROR");
		ros::shutdown();
		return -1;
	}


	// fprintf(stdout, "\nKeys:\nW: forward\tS: backward\nA: left\t\tD: right\nSPACE: up\tLSHIFT: down\nCTRL: land\tRSHIFT: takeoff\nUP: camera up\tDOWN: camera down\nLEFT: rot left\tRIGHT: rot right\nENTER: emergency rotor shutdown\n2: start video\t3: end video\n1: Take a camera snapshot\nUse I, J, K, and L sparingly for arial flips. You can also use '[' and ']' to start and stop autohome navigation.\nEnsure SDL Window is focused for input to be processed!\n");
	while( ros::ok() ) {
		ros::spinOnce();
		eventPoll();
		control.publishVel();
		control.publishCam();
		r.sleep();
	}

	ros::shutdown();
	return 0;
}
