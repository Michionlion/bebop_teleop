#include "ManualControl.h"
#include "StateTracker.h"
#include "Window.h"
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <string.h>

// utility
void bebopImage(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv) {
	ros::init(argc, argv, "bebop_teleop");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	ManualControl control;

	control.advertise(nh);

	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints("compressed", ros::TransportHints(), local_nh);
	image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, bebopImage, hints);

	stats.subscribe(nh);

	ros::Rate r(60);

	// bool pressed;
	// InputWindow input(pressed);
	// if(pressed) {
	// ROS_ERROR("SDL INITIALIZER ERROR");
	// ros::shutdown();
	// return -1;
	// }


	// fprintf(stdout, "\nKeys:\nW: forward\tS: backward\nA: left\t\tD: right\nSPACE: up\tLSHIFT: down\nCTRL: land\tRSHIFT: takeoff\nUP: camera up\tDOWN: camera down\nLEFT: rot left\tRIGHT: rot right\nENTER: emergency rotor shutdown\n2: start video\t3: end video\n1: Take a camera snapshot\nUse I, J, K, and L sparingly for arial flips. You can also use '[' and ']' to start and stop autohome navigation.\nEnsure SDL Window is focused for input to be processed!\n");
	while( ros::ok() && window.ready() ) {
		ros::spinOnce();
		eventPoll();
		window.update();
		ROS_INFO("STATS: ");
		control.publishVel();
		control.publishCam();
		r.sleep();
	}
	window.destroy();
	ros::shutdown();
	return 0;
}

void bebopImage(const sensor_msgs::ImageConstPtr& msg) {
	window.updateVideoTexture(msg);
}
