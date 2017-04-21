#ifndef __STATE_H__
#define __STATE_H__

#include <ros/ros.h>

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

    void setBattery();
    void setWifi();
    void setOdom();
    void setPos();

    void subscribe(ros::NodeHandle&);

private:
    bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr bat;
    bebop_msgs::CommonCommonStateWifiSignalChangedConstPtr wifi;
    nav_msgs::OdometryConstPtr odom;
    sensor_msgs::NavSatFixConstPtr pos;

}

extern StateTracker stats;

#endif
