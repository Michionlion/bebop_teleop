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
    int distance;
    DIR direction;
    geometry_msgs::Point start;
};

class Patroller {
public:
    Patroller(void);
    ~Patroller(void);
    void destroy(void);

    void patrol();

private:
    ros::Publisher pub;
    State current_state;

    void checkState();
    State nextState();

};

extern Patroller patroller;

#endif