#include "ManualControl.h"

ManualControl::ManualControl() {}

ManualControl::~ManualControl() {}

void ManualControl::setPub(int index, ros::Publisher publisher) {
	pub[index] = publisher;
}

void ManualControl::toggle() {
	enabled = !enabled;
	ROS_INFO("%s velocity publishing!", enabled ? "Enabled" : "Disabled");
	geometry_msgs::Twist vel;
	pub[VELOCITY].publish(vel);
}

void ManualControl::publishVel() {
	// char* s = (char*) malloc(17);
	// for(int i = 0; i < 16; i++) {
	// if(((keysDown >> (15-i)) & 1) == 1) {
	// s[i] = '1';
	// } else {
	// s[i] = '0';
	// }
	// }
	// s[16] = '\0';
	//
	//
	//
	//
	// ROS_INFO("PUBLISHED VEL WITH INFO: %s", s);
	if( isKeyDown(6) ) {
		// land
		flying = false;
		std_msgs::Empty m;
		land.publish(m);
		ROS_INFO("EXECUTING LAND!!");
		return;
	}
	if( isKeyDown(7) ) {
		flying = false;
		std_msgs::Empty m;
		reset.publish(m);
		ROS_INFO("EXECUTING EMERGENCY ROTOR STOP!!");
		return;
	}
	if( isKeyDown(8) ) {
		// land
		flying = true;
		std_msgs::Empty m;
		takeoff.publish(m);
		ROS_INFO("EXECUTING TAKEOFF!!");
		return;
	}

	if( isKeyDown(13) && !isKeyDown(14) ) {
		speed -= SPEED_INCREMENT;
		goto CHECK_SPEED;
	} else if( !isKeyDown(13) && isKeyDown(14) ) {
		speed += SPEED_INCREMENT;
		goto CHECK_SPEED;
	}
	goto END_CHECK_SPEED;

CHECK_SPEED:
	if(speed > 1 - SPEED_INCREMENT / 2) speed = 1;
	else if(speed < SPEED_INCREMENT / 2) speed = SPEED_INCREMENT;
	ROS_INFO("Speed: %f", speed);
END_CHECK_SPEED:
	if(isKeyDown(15) && counter % 4 == 0) {
		rotSpeed -= ROTATE_INCREMENT;
		if(rotSpeed < -1 - ROTATE_INCREMENT / 2) rotSpeed = 1;
		if(rotSpeed == 0.0) rotSpeed = -ROTATE_INCREMENT;
		ROS_INFO("Rotation speed: %f", rotSpeed);
	}

	geometry_msgs::Twist vel;

	if( isKeyDown(0) && !isKeyDown(2) )
		// forward
		vel.linear.x = speed;
	else if( !isKeyDown(0) && isKeyDown(2) )
		// backward
		vel.linear.x = -speed;

	if( isKeyDown(1) && !isKeyDown(3) )
		// left
		vel.linear.y = speed;
	else if( !isKeyDown(1) && isKeyDown(3) )
		// right
		vel.linear.y = -speed;

	if( isKeyDown(4) && !isKeyDown(5) )
		// up
		vel.linear.z = speed;
	else if( !isKeyDown(4) && isKeyDown(5) )
		// down
		vel.linear.z = -speed;

	if( isKeyDown(11) && !isKeyDown(12) )
		// rot left
		vel.angular.z = rotSpeed;
	else if( !isKeyDown(11) && isKeyDown(12) )
		// right
		vel.angular.z = -rotSpeed;

	// camera control
	geometry_msgs::Twist cam;
	if( isKeyDown(9) && !isKeyDown(10) ) {
		// cam up
		cam.angular.y = (camCurrentRot += CAM_ROTATE_SPEED);
		goto STARTCAM;
	} else if( !isKeyDown(9) && isKeyDown(10) ) {
		// cam down
		cam.angular.y = (camCurrentRot -= CAM_ROTATE_SPEED);
		goto STARTCAM;
	}
	goto ENDCAM;// skip cam publishing if no keys pressed
STARTCAM:

	if(camCurrentRot >= CAM_MAX_UP) {
		camCurrentRot = CAM_MAX_UP;
		cam.angular.y = camCurrentRot;
	} else if(camCurrentRot <= CAM_MAX_DOWN) {
		camCurrentRot = CAM_MAX_DOWN;
		cam.angular.y = camCurrentRot;
	}

	camera.publish(cam);

ENDCAM:
	if(sendVel) velocity.publish(vel);
}

void ManualControl::publishCam() {
	if(code == 49) {
		// move 1 (snapshot)
		std_msgs::Empty m;
		snapshot.publish(m);
	} else if(code == 50) {
		// move 2 (start recording)
		std_msgs::Bool m;
		m.data = true;
		record.publish(m);
	} else if(code == 51) {
		// move 3 (stop recording)
		std_msgs::Bool m;
		m.data = false;
		record.publish(m);
	}
}

void ManualControl::doFlip(short type) {
	std_msgs::UInt8 m;
	m.data = data;
	flip.publish(m);
}

void ManualControl::navHome(bool state) {
	std_msgs::Bool m;
	m.data = (code == 91 ? true : false);
	home.publish(m);
}
