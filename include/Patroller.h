#ifndef __PATROL_H__
#define __PATROL_H__

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#define FORWARD 0
#define RIGHT 1
#define BACKWARD 2
#define LEFT 3

struct State {
	double distance;
	short direction;
	geometry_msgs::Point start;
};

class Patroller {
public:
	Patroller(void);
	~Patroller(void);
	void destroy(void);

	void patrol(void);
	void start(double, double, double);
	void stop(void);

private:
	State current_state;
	int radius = 0;
	double spacing = 0;
	double speed = 0;
	bool patrolling = false;

	void checkState(void);
	void nextState(void);
};

extern Patroller* patroller;

#endif
