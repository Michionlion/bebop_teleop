#ifndef __MANUAL_H__
#define __MANUAL_H__
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

class ManualControl {
public:
	ManualControl();
	~ManualControl();
	void toggle();
	void publishVel();
	void publishCam();
	void doFlip(short);
	void navHome(bool);
	void setPub(int, ros::Publisher);

private:
	bool enabled;
	ros::Publisher pub[9];
};

#endif
