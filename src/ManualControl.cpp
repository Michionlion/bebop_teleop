#include "Input.h"
#include "ManualControl.h"
#include "Patroller.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

#define CAM_ROTATE_SPEED 2.0
#define CAM_MAX_UP 18.0
#define CAM_MAX_DOWN -70.0
#define CAM_MAX_RIGHT 35.0
#define CAM_MAX_LEFT -35.0
#define SPEED_INCREMENT 0.0125
#define ROTATE_INCREMENT 0.00625
#define START_RECORDING 1
#define STOP_RECORDING 2
#define TAKE_SNAPSHOT 3
#define DO_LAND 4
#define DO_TAKEOFF 5
#define DO_RESET 6

ManualControl control;

ManualControl::ManualControl() {
	input.registerKeyListener(this);
	enabled = true;
}

ManualControl::~ManualControl() {
	input.unregisterKeyListener(this);
}

void ManualControl::key(SDL_KeyboardEvent* event) {
	if(event->type == SDL_KEYDOWN)
		switch(event->keysym.scancode) {
		case SDL_SCANCODE_RIGHTBRACKET:
			doMisc(START_RECORDING);
			break;

		case SDL_SCANCODE_LEFTBRACKET:
			doMisc(STOP_RECORDING);
			break;

		case SDL_SCANCODE_BACKSLASH:
			doMisc(TAKE_SNAPSHOT);
			break;

		case SDL_SCANCODE_0:
			toggle();
			break;

		case SDL_SCANCODE_I:
			doFlip(0);
			break;

		case SDL_SCANCODE_K:
			doFlip(1);
			break;

		case SDL_SCANCODE_L:
			doFlip(2);
			break;

		case SDL_SCANCODE_J:
			doFlip(3);
			break;

		case SDL_SCANCODE_LCTRL:
			doMisc(DO_LAND);
			break;

		case SDL_SCANCODE_RSHIFT:
			doMisc(DO_TAKEOFF);
			break;

		case SDL_SCANCODE_RETURN:
			doMisc(DO_RESET);
			break;

		case SDL_SCANCODE_7:
			patroller.start(2, 0.25, 0.08);
			break;

		case SDL_SCANCODE_8:
			patroller.stop();
			break;

		default:
			return;
		}

}

void ManualControl::advertise(ros::NodeHandle& nh) {
	pub[VELOCITY] = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	pub[TAKEOFF] = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
	pub[LAND] = nh.advertise<std_msgs::Empty>("bebop/land", 1);
	pub[RESET] = nh.advertise<std_msgs::Empty>("bebop/reset", 1);
	pub[CAMERA] = nh.advertise<geometry_msgs::Twist>("bebop/camera_control", 1);
	pub[SNAPSHOT] = nh.advertise<std_msgs::Empty>("bebop/snapshot", 1);
	pub[RECORD] = nh.advertise<std_msgs::Bool>("bebop/record", 1);
	pub[FLIP] = nh.advertise<std_msgs::UInt8>("bebop/flip", 1);
	pub[HOME] = nh.advertise<std_msgs::Bool>("bebop/autoflight/navigate_home", 1);
}

void ManualControl::toggle() {
	enabled = !enabled;
	ROS_INFO("%s manual control!", enabled ? "Enabled" : "Disabled");
	geometry_msgs::Twist vel;
	pub[VELOCITY].publish(vel);
}

void ManualControl::send(geometry_msgs::Twist* msg) {
	ROS_INFO("PUBLISH MSG");
	pub[VELOCITY].publish(*msg);
}

bool ManualControl::isEnabled() {
	return enabled;
}

void ManualControl::publishVel() {
	if(!enabled) return;

	if( input.isKeyDown(SDL_SCANCODE_1) && !input.isKeyDown(SDL_SCANCODE_2) ) {
		speed -= SPEED_INCREMENT;
		goto CHECK_SPEED;
	} else if( !input.isKeyDown(SDL_SCANCODE_1) && input.isKeyDown(SDL_SCANCODE_2) ) {
		speed += SPEED_INCREMENT;
		goto CHECK_SPEED;
	}
	goto END_CHECK_SPEED;

CHECK_SPEED:
	if(speed >= 1 - SPEED_INCREMENT / 2) speed = 1;
	else if(speed < SPEED_INCREMENT / 2) speed = SPEED_INCREMENT;
	ROS_INFO("Speed: %f", speed);

END_CHECK_SPEED:
	if( input.isKeyDown(SDL_SCANCODE_3) && !input.isKeyDown(SDL_SCANCODE_4) ) {
		rotSpeed -= ROTATE_INCREMENT;
		goto CHECK_ROT_SPEED;
	} else if( !input.isKeyDown(SDL_SCANCODE_3) && input.isKeyDown(SDL_SCANCODE_4) ) {
		rotSpeed += ROTATE_INCREMENT;
		goto CHECK_ROT_SPEED;
	}
	goto END_CHECK_ROT_SPEED;

CHECK_ROT_SPEED:
	if(rotSpeed >= 1 - ROTATE_INCREMENT / 2) rotSpeed = 1;
	else if(rotSpeed < ROTATE_INCREMENT / 2) rotSpeed = ROTATE_INCREMENT;
	ROS_INFO("Rotation speed: %f", rotSpeed);

END_CHECK_ROT_SPEED:
	geometry_msgs::Twist vel;

	if( input.isKeyDown(SDL_SCANCODE_W) && !input.isKeyDown(SDL_SCANCODE_S) ) vel.linear.x = speed;
	else if( !input.isKeyDown(SDL_SCANCODE_W) && input.isKeyDown(SDL_SCANCODE_S) ) vel.linear.x = -speed;

	if( input.isKeyDown(SDL_SCANCODE_A) && !input.isKeyDown(SDL_SCANCODE_D) ) vel.linear.y = speed;
	else if( !input.isKeyDown(SDL_SCANCODE_A) && input.isKeyDown(SDL_SCANCODE_D) ) vel.linear.y = -speed;

	if( input.isKeyDown(SDL_SCANCODE_SPACE) && !input.isKeyDown(SDL_SCANCODE_LSHIFT) ) vel.linear.z = speed;
	else if( !input.isKeyDown(SDL_SCANCODE_SPACE) && input.isKeyDown(SDL_SCANCODE_LSHIFT) ) vel.linear.z = -speed;

	if( input.isKeyDown(SDL_SCANCODE_Q) && !input.isKeyDown(SDL_SCANCODE_E) ) vel.angular.z = rotSpeed;
	else if( !input.isKeyDown(SDL_SCANCODE_Q) && input.isKeyDown(SDL_SCANCODE_E) ) vel.angular.z = -rotSpeed;

	pub[VELOCITY].publish(vel);
}

void ManualControl::publishCam() {
	// camera control
	geometry_msgs::Twist cam;

	if( input.isKeyDown(SDL_SCANCODE_UP) && !input.isKeyDown(SDL_SCANCODE_DOWN) ) camY += CAM_ROTATE_SPEED;
	else if( !input.isKeyDown(SDL_SCANCODE_UP) && input.isKeyDown(SDL_SCANCODE_DOWN) ) camY -= CAM_ROTATE_SPEED;

	if( input.isKeyDown(SDL_SCANCODE_RIGHT) && !input.isKeyDown(SDL_SCANCODE_LEFT) ) camX += CAM_ROTATE_SPEED;
	else if( !input.isKeyDown(SDL_SCANCODE_RIGHT) && input.isKeyDown(SDL_SCANCODE_LEFT) ) camX -= CAM_ROTATE_SPEED;

	if(camY >= CAM_MAX_UP - CAM_ROTATE_SPEED / 2) camY = CAM_MAX_UP;
	else if(camY <= CAM_MAX_DOWN + CAM_ROTATE_SPEED / 2) camY = CAM_MAX_DOWN;
	if(camX >= CAM_MAX_RIGHT - CAM_ROTATE_SPEED / 2) camX = CAM_MAX_RIGHT;
	else if(camX <= CAM_MAX_LEFT + CAM_ROTATE_SPEED / 2) camX = CAM_MAX_LEFT;

	cam.angular.z = camX;
	cam.angular.y = camY;
	pub[CAMERA].publish(cam);
}

void ManualControl::doMisc(short type) {
	if(type == START_RECORDING) {
		std_msgs::Bool m;
		m.data = true;
		pub[RECORD].publish(m);
	} else if(type == STOP_RECORDING) {
		std_msgs::Bool m;
		m.data = false;
		pub[RECORD].publish(m);
	} else if(type == TAKE_SNAPSHOT) {
		std_msgs::Empty m;
		pub[SNAPSHOT].publish(m);
	} else if(type == DO_LAND) {
		std_msgs::Empty m;
		pub[LAND].publish(m);
		ROS_INFO("EXECUTING LAND!!");
	} else if(type == DO_RESET) {
		std_msgs::Empty m;
		pub[RESET].publish(m);
		ROS_INFO("EXECUTING EMERGENCY ROTOR STOP!!");
	} else if(type == DO_TAKEOFF) {
		std_msgs::Empty m;
		pub[TAKEOFF].publish(m);
		ROS_INFO("EXECUTING TAKEOFF!!");
	}
}

void ManualControl::doFlip(short type) {
	std_msgs::UInt8 m;
	m.data = type;
	pub[FLIP].publish(m);
}

void ManualControl::navHome(bool state) {
	std_msgs::Bool m;
	m.data = state;
	pub[HOME].publish(m);
}
