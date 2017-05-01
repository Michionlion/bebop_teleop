#include "ManualControl.h"
#include "Patroller.h"
#include "StateTracker.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

Patroller patroller;

Patroller::Patroller() {}

Patroller::~Patroller() {
	destroy();
}

void Patroller::destroy() {}

void Patroller::checkState() {
	// switch statement in case we need to check signage
	ROS_INFO( "OFF: %f %f", fabs(stats.getOdom()->pose.pose.position.x - current_state.start.x), fabs(stats.getOdom()->pose.pose.position.y - current_state.start.y) );
	switch(current_state.direction) {
	case FORWARD:
		if(fabs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case RIGHT:
		if(fabs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;

	case BACKWARD:
		if(fabs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case LEFT:
		if(fabs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;
	}
}

void Patroller::nextState() {
	current_state.start = stats.getOdom()->pose.pose.position;

	if(current_state.direction == RIGHT || current_state.direction == LEFT) current_state.distance = current_state.distance + spacing;
	else current_state.distance = current_state.distance;
	current_state.direction = (current_state.direction + 1) % 4;

	if(current_state.direction == BACKWARD && current_state.distance > radius) patrolling = false;
}

// simple reflex function that responds to current altitude of bebop and state information
void Patroller::patrol() {
	if(!patrolling) return;

	checkState();
	geometry_msgs::Twist vel;

	ROS_INFO("STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);

	switch(current_state.direction) {
	case FORWARD:
		vel.linear.x = speed;
		control.send(&vel);
		return;

	case RIGHT:
		vel.linear.y = -speed;
		control.send(&vel);
		return;

	case BACKWARD:
		vel.linear.x = -speed;
		control.send(&vel);
		return;

	case LEFT:
		vel.linear.y = speed;
		control.send(&vel);
		return;
	}
}

void Patroller::stop() {
	ROS_INFO("STOPPING PATROL");
	patrolling = false;
	control.toggle();
}

void Patroller::start(double altitude, double spacing, double speed) {
	ROS_INFO("STARTING PATROL");
	if( control.isEnabled() ) control.toggle();
	radius = (int) ceil(altitude);
	this->spacing = spacing;
	this->speed = speed;
	patrolling = true;

	current_state.distance = spacing;
	current_state.direction = FORWARD;
	current_state.start = stats.getOdom()->pose.pose.position;
}
