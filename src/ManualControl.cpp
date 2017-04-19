#include "Input.h"
#include "ManualControl.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"

#define CAM_ROTATE_SPEED 2.5
#define CAM_MAX_UP 20.0
#define CAM_MAX_DOWN -70.0
#define SPEED_INCREMENT 0.05
#define ROTATE_INCREMENT 0.125


ManualControl::ManualControl() {}

ManualControl::~ManualControl() {}

void ManualControl::setPub(int index, ros::Publisher publisher) {
	pub[index] = publisher;
}

void ManualControl::toggle() {
	enabled = !enabled;
	ROS_INFO("%s velocity publishing!", enabled ? "Enabled" : "Disabled");
	geometry_msgs::Twist vel;
	pub[VELOCITY].publish(vel);
}

void ManualControl::publishVel() {
	// if( input->isKeyDown(6) ) {
	// // land
	// flying = false;
	// std_msgs::Empty m;
	// land.publish(m);
	// ROS_INFO("EXECUTING LAND!!");
	// return;
	// }
	// if( input->isKeyDown(7) ) {
	// flying = false;
	// std_msgs::Empty m;
	// reset.publish(m);
	// ROS_INFO("EXECUTING EMERGENCY ROTOR STOP!!");
	// return;
	// }
	// if( input->isKeyDown(8) ) {
	// // land
	// // flying = true;
	// std_msgs::Empty m;
	//
	// // takeoff.publish(m);
	// ROS_INFO("EXECUTING TAKEOFF!!");
	// return;
	// }

	// if( input->isKeyDown(13) && !input->isKeyDown(14) ) {
	// speed -= SPEED_INCREMENT;
	// goto CHECK_SPEED;
	// } else if( !input->isKeyDown(13) && input->isKeyDown(14) ) {
	// speed += SPEED_INCREMENT;
	goto CHECK_SPEED;

	// }
	goto END_CHECK_SPEED;

CHECK_SPEED:

	// if(speed > 1 - SPEED_INCREMENT / 2) speed = 1;
	// else if(speed < SPEED_INCREMENT / 2) speed = SPEED_INCREMENT;
	// ROS_INFO("Speed: %f", speed);
END_CHECK_SPEED:

	// if(isKeyDown(15) && counter % 4 == 0) {
	// rotSpeed -= ROTATE_INCREMENT;
	// if(rotSpeed < -1 - ROTATE_INCREMENT / 2) rotSpeed = 1;
	// if(rotSpeed == 0.0) rotSpeed = -ROTATE_INCREMENT;
	// ROS_INFO("Rotation speed: %f", rotSpeed);
// }

	geometry_msgs::Twist vel;

// if( input->isKeyDown(0) && !input->isKeyDown(2) )
// // forward
// vel.linear.x = speed;
// else if( !input->isKeyDown(0) && input->isKeyDown(2) )
// // backward
// vel.linear.x = -speed;
//
// if( input->isKeyDown(1) && !input->isKeyDown(3) )
// // left
// vel.linear.y = speed;
// else if( !input->isKeyDown(1) && input->isKeyDown(3) )
// // right
// vel.linear.y = -speed;

// if( input->isKeyDown(4) && !input->isKeyDown(5) )
// // up
// vel.linear.z = speed;
// else if( !input->isKeyDown(4) && input->isKeyDown(5) )
// // down
// vel.linear.z = -speed;
//
// if( input->isKeyDown(11) && !input->isKeyDown(12) )
// // rot left
// vel.angular.z = rotSpeed;
// else if( !input->isKeyDown(11) && input->isKeyDown(12) )
// // right
// vel.angular.z = -rotSpeed;

// camera control
	geometry_msgs::Twist cam;

// if( input->isKeyDown(9) && !input->isKeyDown(10) ) {
// // cam up
// cam.angular.y = (camCurrentRot += CAM_ROTATE_SPEED);
// goto STARTCAM;
// } else if( !input->isKeyDown(9) && input->isKeyDown(10) ) {
// // cam down
// cam.angular.y = (camCurrentRot -= CAM_ROTATE_SPEED);
// goto STARTCAM;
// }
// goto ENDCAM;// skip cam publishing if no keys pressed
// STARTCAM:

// if(camCurrentRot >= CAM_MAX_UP) {
// camCurrentRot = CAM_MAX_UP;
// cam.angular.y = camCurrentRot;
// } else if(camCurrentRot <= CAM_MAX_DOWN) {
// camCurrentRot = CAM_MAX_DOWN;
// cam.angular.y = camCurrentRot;
// }

// pub[CAMERA].publish(cam);
//
// ENDCAM:
// pub[VELOCITY].publish(vel);
}

void ManualControl::publishCam() {
	// if(code == 49) {
	// // move 1 (snapshot)
	// std_msgs::Empty m;
	// snapshot.publish(m);
	// } else if(code == 50) {
	// // move 2 (start recording)
	// std_msgs::Bool m;
	// m.data = true;
	// record.publish(m);
	// } else if(code == 51) {
	// // move 3 (stop recording)
	// std_msgs::Bool m;
	// m.data = false;
	// record.publish(m);
	// }
}

void ManualControl::doFlip(short type) {
	// std_msgs::UInt8 m;
	// m.data = data;
	// flip.publish(m);
}

void ManualControl::navHome(bool state) {
	// std_msgs::Bool m;
	// m.data = (code == 91 ? true : false);
	// home.publish(m);
}
