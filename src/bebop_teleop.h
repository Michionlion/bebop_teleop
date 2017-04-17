#include <ros/ros.h>

void doPub();

bool isKeyDown(uint8_t);
void doFlip(uint8_t);
void doHome(uint16_t);
void doCamera(uint16_t);

ros::Publisher velocity;
ros::Publisher takeoff;
ros::Publisher land;
ros::Publisher reset;

ros::Publisher camera;
ros::Publisher snapshot;
ros::Publisher record;
ros::Publisher flip;
ros::Publisher home;
