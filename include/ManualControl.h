#ifndef __MANUAL_H__
#define __MANUAL_H__

#include "Input.h"
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
	void advertise(ros::NodeHandle&);
	void key(SDL_KeyboardEvent*);

private:
	bool enabled;
	ros::Publisher pub[9];
	double speed = 0.7;
	double rotSpeed = 0.7;
	double camCurrentRot;
};

#endif
