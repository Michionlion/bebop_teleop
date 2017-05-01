#ifndef __STATE_H__
#define __STATE_H__

#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/CommonCommonStateWifiSignalChanged.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class StateTracker {
public:
	StateTracker(void);
	~StateTracker(void);
	void destroy(void);

	short getBattery(void);
	short getWifiStrength(void);

	double getXVelocity(void);
	double getYVelocity(void);
	double getZVelocity(void);

	bool hasFix(void);
	double getLatitude(void);
	double getLongitude(void);
	double getAltitude(void);
	nav_msgs::Odometry* getOdom(void);

	void setBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr&);
	void setWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr&);
	void setOdom(const nav_msgs::OdometryConstPtr&);
	void setPos(const sensor_msgs::NavSatFixConstPtr&);

	void subscribe(ros::NodeHandle&);

private:
	short bat = 95;
	short wifi = 100;
	nav_msgs::Odometry odom;
	sensor_msgs::NavSatFix pos;
	ros::Subscriber sub[4];
};

extern StateTracker* stats;

#endif
