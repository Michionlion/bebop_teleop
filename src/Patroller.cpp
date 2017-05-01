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

void Patroller::destroy() {
	pub.shutdown();
}

void Patroller::checkState() {
	// switch statement in case we need to check signage
	switch(current_state.direction) {
	case FORWARD:
		if(abs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case RIGHT:
		if(abs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;

	case BACKWARD:
		if(abs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case LEFT:
		if(abs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
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

	switch(current_state.direction) {
	case FORWARD:
		vel.linear.x = speed;
		pub.publish(vel);
		return;

	case RIGHT:
		vel.linear.y = -speed;
		pub.publish(vel);
		return;

	case BACKWARD:
		vel.linear.x = -speed;
		pub.publish(vel);
		return;

	case LEFT:
		vel.linear.y = speed;
		pub.publish(vel);
		return;
	}
}

void Patroller::start(double altitude, double spacing, double speed) {
	radius = (int) ceil(altitude);
	this->spacing = spacing;
	this->speed = speed;
	patrolling = true;

	current_state.distance = spacing;
	current_state.direction = FORWARD;
	current_state.start = stats.getOdom()->pose.pose.position;
}
