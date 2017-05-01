#include "ManualControl.h"
#include "Patroller.h"
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

int main(int argc, char** argv) {
	ros::init(argc, argv, "bebop_teleop");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	ManualControl control;

	control.advertise(nh);

	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints("compressed", ros::TransportHints(), local_nh);
	image_transport::Subscriber sub = it.subscribe("bebop/image_raw", 1, &Window::updateVideoTexture, &window, hints);

	stats.subscribe(nh);

	ros::Rate r(30);

	// bool pressed;
	// InputWindow input(pressed);
	// if(pressed) {
	// ROS_ERROR("SDL INITIALIZER ERROR");
	// ros::shutdown();
	// return -1;
	// }


	// fprintf(stdout, "\nKeys:\nW: forward\tS: backward\nA: left\t\tD: right\nSPACE: up\tLSHIFT: down\nCTRL: land\tRSHIFT: takeoff\nUP: camera up\tDOWN: camera down\nLEFT: rot left\tRIGHT: rot right\nENTER: emergency rotor shutdown\n2: start video\t3: end video\n1: Take a camera snapshot\nUse I, J, K, and L sparingly for arial flips. You can also use '[' and ']' to start and stop autohome navigation.\nEnsure SDL Window is focused for input to be processed!\n");
	while( ros::ok() && window.ok() ) {
		ros::spinOnce();
		eventPoll();
		window.update();
		patroller.patrol();


		// ROS_INFO( "STATS: Batt: %d%% Wifi: %d GPS: %s\nGPS: (Latitude: %0.6f Longitude: %0.6f)\nVELX: %0.3f VELY: %0.3f VELZ: %0.3f", stats.getBattery(), stats.getWifiStrength(), stats.hasFix() ? "Has Fix" : "No Fix", stats.getLatitude(), stats.getLongitude(), stats.getXVelocity(), stats.getYVelocity(), stats.getZVelocity() );

		control.publishVel();
		control.publishCam();
		r.sleep();
	}

	ros::shutdown();
	window.destroy();
	stats.destroy();
	return 0;
}
