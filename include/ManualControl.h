#ifndef __MANUAL_H__
#define __MANUAL_H__

#include "Input.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#define VELOCITY 0
#define TAKEOFF 1
#define LAND 2
#define RESET 3
#define CAMERA 4
#define SNAPSHOT 5
#define RECORD 6
#define FLIP 7
#define HOME 8

class ManualControl : public KeyListener {
public:
	ManualControl(void);
	~ManualControl(void);
	void toggle(void);
	bool isEnabled(void);
	void publishVel(void);
	void publishCam(void);
	void doMisc(short);
	void doFlip(short);
	void navHome(bool);
	void send(geometry_msgs::Twist*);
	void advertise(ros::NodeHandle&);
	void key(SDL_KeyboardEvent*);
	geometry_msgs::Twist* getLast(void);

	void incSpeed(void);
	void decSpeed(void);
	void incRotSpeed(void);
	void decRotSpeed(void);
	double getSpeed(void);
	double getRotSpeed(void);

private:
	bool enabled = true;
	ros::Publisher pub[9];
	double speed = 0.7;
	double rotSpeed = 0.7;
	double camX = 0;
	double camY = 0;
	geometry_msgs::Twist last;
};

extern ManualControl* control;

#endif
