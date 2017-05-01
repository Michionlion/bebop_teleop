#include "Patroller.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

Patroller patroller;

Patroller::Patroller() {
}

Patroller::~Patroller() {
    destroy();
}

void Patroller::destroy() {
    pub.shutdown();
}

void Patroller::checkState() {

    switch(current_state.direction) {
        case FORWARD:
            if (std::abs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) {
                nextState();
            }
            break;
        case RIGHT:
            if (std::abs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) {
                nextState();
            }
            break;
        case BACKWARD:
            if (std::abs(stats.getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) {
                nextState();
            }
            break;
        case LEFT:
            if (std::abs(stats.getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) {
                nextState();
            }
            break;
    }


}

void Patroller::nextState() {
    current_state.start = stats.getOdom()->post.pose.position;

    if (current_state.direction == RIGHT || current_state.direction == LEFT) {
        current_state.distance = current_state.distance + spacing;
    }
    else {
        current_state.distance = current_state.distance;
    }

    current_state.direction = (current_state.direction + 1) % 4;

    if (current_state.direction == BACKWARD && current_state.distance > radius) {
        patrolling = false;
    }

}

// simple reflex function that responds to current altitude of bebop
void Patroller::patrol() {

    if (!patrolling) return;

    checkState();
    geometry_msgs::Twist vel;

    switch(current_state.direction) {
        case FORWARD:
            vel.linear.x = spacing;
            pub.publish(vel);
            return;
        case RIGHT:
            vel.linear.y = -spacing;
            pub.publish(vel);
            return;
        case BACKWARD:
            vel.linear.x = -spacing;
            pub.publish(vel);
            return;
        case LEFT:
            vel.linear.y = spacing;
            pub.publish(vel);
            return;
    }

}

void Patroller::start(double altitude, double spacing) {
    
    radius = (int) ceil(altitude);
    this->spacing = spacing;
    patrolling = true;

    current_state.distance = spacing;
    current_state.direction = FORWARD;
    current_state.start = stats.getOdom()->pose.pose.position;

}
