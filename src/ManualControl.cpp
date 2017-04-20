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

#define START_RECORDING 1
#define STOP_RECORDING 2
#define TAKE_SNAPSHOT 3


ManualControl::ManualControl() {
	input.registerKeyListener(this);
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

		default:
			return;
		}

}

void ManualControl::setPub(int index, ros::Publisher publisher) {
	pub[index] = publisher;
}

void ManualControl::toggle() {
	enabled = !enabled;
	ROS_INFO("%s manual control!", enabled ? "Enabled" : "Disabled");
	geometry_msgs::Twist vel;
	pub[VELOCITY].publish(vel);
}

bool ManualControl::isEnabled() {
	return enabled;
}

void ManualControl::publishVel() {
	if( input.isKeyDown(SDL_SCANCODE_LCTRL) ) {
		std_msgs::Empty m;
		pub[LAND].publish(m);
		ROS_INFO("EXECUTING LAND!!");
		return;
	} else if( input.isKeyDown(SDL_SCANCODE_RETURN) ) {
		std_msgs::Empty m;
		pub[RESET].publish(m);
		ROS_INFO("EXECUTING EMERGENCY ROTOR STOP!!");
		return;
	} else if( input.isKeyDown(SDL_SCANCODE_LSHIFT) ) {
		std_msgs::Empty m;
		pub[TAKEOFF].publish(m);
		ROS_INFO("EXECUTING TAKEOFF!!");
		return;
	}

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

	if( input.isKeyDown(SDL_SCANCODE_D) && !input.isKeyDown(SDL_SCANCODE_A) ) vel.linear.y = speed;
	else if( !input.isKeyDown(SDL_SCANCODE_D) && input.isKeyDown(SDL_SCANCODE_A) ) vel.linear.y = -speed;

	if( input.isKeyDown(SDL_SCANCODE_SPACE) && !input.isKeyDown(SDL_SCANCODE_RSHIFT) ) vel.linear.z = speed;
	else if( !input.isKeyDown(SDL_SCANCODE_SPACE) && input.isKeyDown(SDL_SCANCODE_RSHIFT) ) vel.linear.z = -speed;

	if( input.isKeyDown(SDL_SCANCODE_LEFT) && !input.isKeyDown(SDL_SCANCODE_RIGHT) ) vel.angular.z = rotSpeed;
	else if( !input.isKeyDown(SDL_SCANCODE_LEFT) && input.isKeyDown(SDL_SCANCODE_RIGHT) ) vel.angular.z = -rotSpeed;

	pub[VELOCITY].publish(vel);
}

void ManualControl::publishCam() {
	// camera control
	geometry_msgs::Twist cam;

	if( input.isKeyDown(SDL_SCANCODE_UP) && !input.isKeyDown(SDL_SCANCODE_DOWN) ) {
		cam.angular.y = (camCurrentRot += CAM_ROTATE_SPEED);
		goto CHECK_CAM;
	} else if( !input.isKeyDown(SDL_SCANCODE_UP) && input.isKeyDown(SDL_SCANCODE_DOWN) ) {
		cam.angular.y = (camCurrentRot -= CAM_ROTATE_SPEED);
		goto CHECK_CAM;
	}

	// skip cam publishing if no keys pressed
	return;

CHECK_CAM:
	if(camCurrentRot >= CAM_MAX_UP - CAM_ROTATE_SPEED / 2) {
		camCurrentRot = CAM_MAX_UP;
		cam.angular.y = camCurrentRot;
	} else if(camCurrentRot <= CAM_MAX_DOWN + CAM_ROTATE_SPEED / 2) {
		camCurrentRot = CAM_MAX_DOWN;
		cam.angular.y = camCurrentRot;
	}

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
