#include "StateTracker.h"
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/CommonCommonStateWifiSignalChanged.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

// declare callbacks
void updateBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr& msg);
void updateWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr& msg);
void updateOdom(const nav_msgs::OdometryConstPtr& msg);
void updatePos(const sensor_msgs::NavSatFixConstPtr& msg);

StateTracker stats;

StateTracker::StateTracker() {}

StateTracker::~StateTracker() {}

void StateTracker::subscribe(ros::NodeHandle& nh) {
	nh.subscribe("/bebop/states/common/CommonState/BatteryStateChanged", 1, updateBattery);
	nh.subscribe("/bebop/states/common/CommonState/WifiSignalChanged", 1, updateWifi);
	nh.subscribe("/bebop/odom", 1, updateOdom);
	nh.subscribe("/bebop/fix", 1, updatePos);
}

short StateTracker::getBattery() {
	if(bat != NULL) return bat->percent;
	else return -1;
}

short StateTracker::getWifiStrength() {
	if(wifi != NULL) return wifi->rssi;
	else return -10000;
}

double StateTracker::getXVelocity() {
	if(odom != NULL) return odom->twist.twist.linear.x;
	else return 0;
}

double StateTracker::getYVelocity() {
	if(odom != NULL) return odom->twist.twist.linear.y;
	else return 0;
}

double StateTracker::getZVelocity() {
	if(odom != NULL) return odom->twist.twist.linear.z;
	else return 0;
}

bool StateTracker::hasFix() {
	if(pos != NULL) return pos->status.status != -1;
	else return false;
}

double StateTracker::getLatitude() {
	if( hasFix() ) return pos->latitude;
	else return 0;
}

double StateTracker::getLongitude() {
	if( hasFix() ) return pos->longitude;
	else return 0;
}

double StateTracker::getAltitude() {
	if( hasFix() ) return pos->altitude;
	else return 0;
}

void StateTracker::setBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr& battery) {
	bat = battery;
}

void StateTracker::setWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr& wi) {
	wifi = wi;
}

void StateTracker::setOdom(const nav_msgs::OdometryConstPtr& od) {
	odom = od;
}

void StateTracker::setPos(const sensor_msgs::NavSatFixConstPtr& po) {
	pos = po;
}

void updateBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr& msg) {
	stats.setBattery(msg);
}

void updateWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr& msg) {
	stats.setWifi(msg);
}

void updateOdom(const nav_msgs::OdometryConstPtr& msg) {
	stats.setOdom(msg);
}

void updatePos(const sensor_msgs::NavSatFixConstPtr& msg) {
	stats.setPos(msg);
}
