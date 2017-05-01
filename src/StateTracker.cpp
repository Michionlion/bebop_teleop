#include "StateTracker.h"
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
#include <bebop_msgs/CommonCommonStateWifiSignalChanged.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

StateTracker stats;

StateTracker::StateTracker() {
	// ensure we start off with no fix
	pos.status.status = -1;
}

StateTracker::~StateTracker() {
	destroy();
}

void StateTracker::destroy() {
	for(int i = 0; i < 4; i++) sub[i].shutdown();
}

void StateTracker::subscribe(ros::NodeHandle& nh) {
	sub[0] = nh.subscribe("bebop/states/common/CommonState/BatteryStateChanged", 100, &StateTracker::setBattery, this);
	sub[1] = nh.subscribe("bebop/states/common/CommonState/WifiSignalChanged", 100, &StateTracker::setWifi, this);
	sub[2] = nh.subscribe("bebop/odom", 100, &StateTracker::setOdom, this);
	sub[3] = nh.subscribe("bebop/fix", 100, &StateTracker::setPos, this);

	// ROS_INFO("SUBSCRIBED");
}

short StateTracker::getBattery() {
	return bat;
}

short StateTracker::getWifiStrength() {
	return wifi;
}

double StateTracker::getXVelocity() {
	return odom.twist.twist.linear.x;
}

double StateTracker::getYVelocity() {
	return odom.twist.twist.linear.y;
}

double StateTracker::getZVelocity() {
	return odom.twist.twist.linear.z;
}

nav_msgs::Odometry* StateTracker::getOdom() {
	return &odom;
}

bool StateTracker::hasFix() {
	return pos.status.status != -1;
}

double StateTracker::getLatitude() {
	if( hasFix() ) return pos.latitude;
	else return nan("No Fix");
}

double StateTracker::getLongitude() {
	if( hasFix() ) return pos.longitude;
	else return nan("No Fix");
}

double StateTracker::getAltitude() {
	if( hasFix() ) return pos.altitude;
	else return nan("No Fix");
}

void StateTracker::setBattery(const bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr& battery) {
	bat = battery->percent;

	// ROS_INFO("GOT BATT, %d", bat);
}

void StateTracker::setWifi(const bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr& wi) {
	wifi = wi->rssi;

	// ROS_INFO("GOT WIFI, %d", wifi);
}

void StateTracker::setOdom(const nav_msgs::OdometryConstPtr& od) {
	odom = *od;
}

void StateTracker::setPos(const sensor_msgs::NavSatFixConstPtr& po) {
	pos = *po;

	// ROS_INFO("GOT NAVFIX");
}
