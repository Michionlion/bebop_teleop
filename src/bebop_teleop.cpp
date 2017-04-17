#include "InputWindow.h"
#include "ManualControl.h"
#include "bebop_teleop.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#define CAM_ROTATE_SPEED 2.5
#define CAM_MAX_UP 20.0
#define CAM_MAX_DOWN -70.0
#define SPEED_INCREMENT 0.05
#define ROTATE_INCREMENT 0.125


/*
  * b0=w -- forward -- 119
  * b1=a -- left -- 97
  * b2=s -- backward -- 115
  * b3=d -- right -- 100
  * b4=space -- up -- 32
  * b5=lshift -- down -- 304
  * b6=ctrl -- land -- 306
  * b7=enter -- reset -- 13
  * b8=rshift -- takeoff -- 303
  * b9=up -- camera up -- 273
  * b10=down -- camera down -- 274
  * b11=left -- rotate left -- 276
  * b12=right -- rotate right -- 275
  *
  * b13=1 -- 49
  * b14=2 -- 50
  * b15=3 -- 51
  */
uint16_t keysDown;
bool flying = false;


bool sendVel = true;
double speed = 1;
double rotSpeed = 1;
double camCurrentRot = 0.0;
int counter;

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

	ros::Rate r(20);

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
		control.publishVel();
		control.publishCam();
		r.sleep();
	}

	ros::shutdown();
	return 0;
}
