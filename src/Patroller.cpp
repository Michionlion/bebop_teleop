#include "Patroller.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

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
            if (std::abs(stats.getOdom()->pose.pose.position.x))
            pub.publish(vel);
            return;
        case RIGHT:
            vel.linear.y = -1;
            pub.publish(vel);
            return;
        case BACKWARD:
            vel.linear.x = -1;
            pub.publish(vel);
            return;
        case LEFT:
            vel.linear.y = 1;
            pub.publish(vel);
            return;
    }


}

State Patroller::nextState() {

    State next_state;
    if (current_state.direction == RIGHT || current_state.direction == LEFT) {
        next_state.distance = current_state.distance + 1;
    }
    else {
        next_state.distance = current_state.distance;
    }

    next_state.direction = (current_state.direction + 1) % 4;

    return next_state;
}

// simple reflex function that responds to current altitude of bebop
bool Patroller::patrol() {

    checkState();
    geometry_msgs::Twist vel;

    switch(current_state.direction) {
        case FORWARD:
            vel.linear.x = 1;
            pub.publish(vel);
            return;
        case RIGHT:
            vel.linear.y = -1;
            pub.publish(vel);
            return;
        case BACKWARD:
            vel.linear.x = -1;
            pub.publish(vel);
            return;
        case LEFT:
            vel.linear.y = 1;
            pub.publish(vel);
            return;
    }


}
