#include "ManualControl.h"
#include "Patroller.h"
#include "StateTracker.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

Patroller* patroller;

Patroller::Patroller() {}

Patroller::~Patroller() {
	destroy();
}

void Patroller::destroy() {}

void Patroller::checkState() {
	// switch statement in case we need to check signage
	ROS_INFO( "OFF: x: %f Y: %f", fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x), fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) );
	switch(current_state.direction) {
	case FORWARD:
	case BACKWARD:
		if(fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case RIGHT:
	case LEFT:
		if(fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;
	}
}

void Patroller::nextState() {
	current_state.start = stats->getOdom()->pose.pose.position;

	if(current_state.direction == RIGHT || current_state.direction == LEFT) current_state.distance = current_state.distance + spacing;
	else current_state.distance = current_state.distance;
	current_state.direction = (current_state.direction + 1) % 4;

	if(current_state.direction == BACKWARD && current_state.distance > radius) patrolling = false;
	ROS_INFO("SWITCHING TO STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);
}

// simple reflex function that responds to current altitude of bebop and state information
void Patroller::patrol() {
	if(!patrolling) return;

	double off = 0;

	ROS_INFO("PATROLLING WITH STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);

	checkState();
	geometry_msgs::Twist vel;
	speed = control->getSpeed();
	switch(current_state.direction) {
	case FORWARD:
		ROS_INFO("EXECUTING FORWARD");
		vel.linear.x = speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case RIGHT:
		ROS_INFO("EXECUTING RIGHT");
		vel.linear.y = -speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case BACKWARD:
		ROS_INFO("EXECUTING BACKWARD");
		vel.linear.x = -speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case LEFT:
		ROS_INFO("EXECUTING LEFT");
		vel.linear.y = speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;
	}
	ROS_INFO("SENT X: %f Y: %f", vel.linear.x, vel.linear.y);
}

void Patroller::stop() {
	ROS_INFO("STOPPING PATROL");
	patrolling = false;
	if( !control->isEnabled() ) control->toggle();
}

void Patroller::start(double altitude, double spacing) {
	ROS_INFO("STARTING PATROL");
	if( control->isEnabled() ) control->toggle();
	radius = (int) ceil(altitude);
	this->spacing = spacing;
	patrolling = true;

	current_state.distance = spacing;
	current_state.direction = FORWARD;
	current_state.start = stats->getOdom()->pose.pose.position;
}
