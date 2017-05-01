#ifndef __PATROL_H__
#define __PATROL_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#define FORWARD 0
#define RIGHT 1
#define BACKWARD 2
#define LEFT 3
#define DIR uint8_t;

struct State {
    double distance;
    DIR direction;
    geometry_msgs::Point start;
};

class Patroller {
public:
    Patroller(void);
    ~Patroller(void);
    void destroy(void);

    void patrol();
    void start(double, double);

private:
    ros::Publisher pub;
    State current_state;
    int radius;
    double spacing;
    bool patrolling;

    void checkState();
    void nextState();

};

extern Patroller patroller;

#endif