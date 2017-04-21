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

	short getBattery(void);
	short getWifiStrength(void);

	double getXVelocity(void);
	double getYVelocity(void);
	double getZVelocity(void);

	bool hasFix(void);
	double getLatitude(void);
	double getLongitude(void);
	double getAltitude(void);

	void setBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr&);
	void setWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr&);
	void setOdom(const nav_msgs::OdometryConstPtr&);
	void setPos(const sensor_msgs::NavSatFixConstPtr&);

	void subscribe(ros::NodeHandle&);

private:
	bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr bat;
	bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr wifi;
	nav_msgs::OdometryConstPtr odom;
	sensor_msgs::NavSatFixConstPtr pos;
};

extern StateTracker stats;

#endif
