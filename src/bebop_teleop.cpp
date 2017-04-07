#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include <stdio.h>
#include <signal.h>
#include <SDL2/SDL.h>
#include <string.h>

#define CAM_ROTATE_SPEED 2.5
#define CAM_MAX_UP 20.0
#define CAM_MAX_DOWN -70.0
#define SPEED_INCREMENT 0.05
#define ROTATE_INCREMENT 0.125

void doPub();
bool isKeyDown(uint8_t);
void doFlip(uint8_t);
void doHome(uint16_t);
void doCamera(uint16_t);

class InputWindow {
	public:
		InputWindow( bool& err );
		~InputWindow(void);

		bool get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers);
		SDL_Window* getWindow();
		SDL_Renderer* getRender();
	private:
		// SDL_Surface* window;
		SDL_Window* window;
		SDL_Renderer* render;
};

InputWindow::InputWindow(bool& err) {
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		ROS_ERROR("SDL INIT FAIL: %s\n", SDL_GetError());
		err = true;
		return;
	}
	// SDL_EnableKeyRepeat( 0, SDL_DEFAULT_REPEAT_INTERVAL );
	//SDL 1.2
	// SDL_WM_SetCaption("Bebop_Teleop keyboard input", NULL);
	// window = SDL_SetVideoMode(200, 200, 0, 0);
	SDL_CreateWindowAndRenderer(200, 200, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE, &window, &render);
	if(window == NULL || render == NULL) {
		ROS_ERROR("SDL CREATE WINDOW FAIL: %s\n", SDL_GetError());
		err = true;
		return;
	}

	err = false;
}

InputWindow::~InputWindow(void) {
	SDL_DestroyRenderer(this->render);
	SDL_DestroyWindow(this->window);
	SDL_Quit();
}

bool InputWindow::get_key(bool& new_event, bool& pressed, uint16_t& code, uint16_t& modifiers) {
	new_event = false;

	SDL_Event event;
	if (SDL_PollEvent(&event)) {
		switch(event.type) {
			case SDL_KEYUP:
				pressed = false;
				code = event.key.keysym.scancode;
				modifiers = event.key.keysym.mod;
				new_event = true;
			break;
			case SDL_KEYDOWN:
				if(event.key.repeat == 0) {
					pressed = true;
					code = event.key.keysym.scancode;
					modifiers = event.key.keysym.mod;
					new_event = true;
				}
			break;
			case SDL_QUIT:
				return false;
			break;
		}
	}
	return true;
}

SDL_Window* InputWindow::getWindow() {
	return this->window;
}

SDL_Renderer* InputWindow::getRender() {
	return this->render;
}

/*
b0=w -- forward -- 119
b1=a -- left -- 97
b2=s -- backward -- 115
b3=d -- right -- 100
b4=space -- up -- 32
b5=lshift -- down -- 304
b6=ctrl -- land -- 306
b7=enter -- reset -- 13
b8=rshift -- takeoff -- 303
b9=up -- camera up -- 273
b10=down -- camera down -- 274
b11=left -- rotate left -- 276
b12=right -- rotate right -- 275

b13=1 -- 49
b14=2 -- 50
b15=3 -- 51
*/
uint16_t keysDown;
bool flying = false;

ros::Publisher velocity;
ros::Publisher takeoff;
ros::Publisher land;
ros::Publisher reset;

ros::Publisher camera;
ros::Publisher snapshot;
ros::Publisher record;
ros::Publisher flip;
ros::Publisher home;

bool sendVel = true;
double speed = 1;
double rotSpeed = 1;
double camCurrentRot = 0.0;
int counter;

int main(int argc, char** argv) {
	ros::init(argc, argv, "bebop_teleop");
	ros::NodeHandle nh;
	velocity = nh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	takeoff = nh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
	land = nh.advertise<std_msgs::Empty>("bebop/land", 1);
	reset = nh.advertise<std_msgs::Empty>("bebop/reset", 1);
	camera = nh.advertise<geometry_msgs::Twist>("bebop/camera_control", 1);
	snapshot = nh.advertise<std_msgs::Empty>("bebop/snapshot", 1);
	record = nh.advertise<std_msgs::Bool>("bebop/record", 1);
	flip = nh.advertise<std_msgs::UInt8>("bebop/flip", 1);
	home = nh.advertise<std_msgs::Bool>("bebop/autoflight/navigate_home", 1);


	//publish at 20hz, inquire at 60
	ros::Rate r(60);

	bool pressed = false, new_event;
	uint16_t code, modifiers;

	InputWindow input(pressed);
	if(pressed) {
		ROS_ERROR("SDL INITIALIZER ERROR");
		ros::shutdown();
		return -1;
	}


	fprintf(stdout, "\nKeys:\nW: forward\tS: backward\nA: left\t\tD: right\nSPACE: up\tLSHIFT: down\nCTRL: land\tRSHIFT: takeoff\nUP: camera up\tDOWN: camera down\nLEFT: rot left\tRIGHT: rot right\nENTER: emergency rotor shutdown\n2: start video\t3: end video\n1: Take a camera snapshot\nUse I, J, K, and L sparingly for arial flips. You can also use '[' and ']' to start and stop autohome navigation.\nEnsure SDL Window is focused for input to be processed!\n");
	int spinner = 5;
	while(ros::ok() && input.get_key(new_event, pressed, code, modifiers)) {
		ros::spinOnce();
		counter++;
		//modify keysDown
		if(new_event) {
			if(pressed) {
				// ROS_INFO("Pressed %d (%d)", code, modifiers);
				// SDL_FillRect(input.getSurf(), NULL, SDL_MapRGB(input.getSurf()->format, 255, 255, 255));
				// SDL_UpdateRect(input.getSurf(),0,0,0,0);
			} else {
				// ROS_INFO("Released %d (%d)", code, modifiers);
						// SDL_FillRect(input.getSurf(), NULL, SDL_MapRGB(input.getSurf()->format, 0, 0, 0));
					// SDL_UpdateRect(input.getSurf(),0,0,0,0);
			}
			spinner = 0; //force publish update
			switch(code) {
				case SDL_SCANCODE_W: //w
					keysDown = pressed ? keysDown | 0x8000 : keysDown & 0x7FFF;
				break;
				case SDL_SCANCODE_A: //a
					keysDown = pressed ? keysDown | 0x4000 : keysDown & 0xBFFF;
				break;
				case SDL_SCANCODE_S: //s
					keysDown = pressed ? keysDown | 0x2000 : keysDown & 0xDFFF;
				break;
				case SDL_SCANCODE_D: //d
					keysDown = pressed ? keysDown | 0x1000 : keysDown & 0xEFFF;
				break;
				case SDL_SCANCODE_SPACE: //space
					keysDown = pressed ? keysDown | 0x0800 : keysDown & 0xF7FF;
				break;
				case SDL_SCANCODE_LSHIFT: //lshift
					keysDown = pressed ? keysDown | 0x0400 : keysDown & 0xFBFF;
				break;
				case SDL_SCANCODE_LCTRL: //ctrl
					keysDown = pressed ? keysDown | 0x0200 : keysDown & 0xFDFF;
				break;
				case SDL_SCANCODE_RETURN: //enter
					keysDown = pressed ? keysDown | 0x0100 : keysDown & 0xFEFF;
				break;
				case SDL_SCANCODE_RSHIFT: //rshift
					keysDown = pressed ? keysDown | 0x0080 : keysDown & 0xFF7F;
				break;
				case SDL_SCANCODE_UP: //up
					keysDown = pressed ? keysDown | 0x0040 : keysDown & 0xFFBF;
				break;
				case SDL_SCANCODE_DOWN: //down
					keysDown = pressed ? keysDown | 0x0020 : keysDown & 0xFFDF;
				break;
				case SDL_SCANCODE_LEFT: //left
					keysDown = pressed ? keysDown | 0x0010 : keysDown & 0xFFEF;
				break;
				case SDL_SCANCODE_RIGHT: //right
					keysDown = pressed ? keysDown | 0x0008 : keysDown & 0xFFF7;
				break;
				case SDL_SCANCODE_1: //1
					keysDown = pressed ? keysDown | 0x0004 : keysDown & 0xFFFB;
				break;
				case SDL_SCANCODE_2: //2
					keysDown = pressed ? keysDown | 0x0002 : keysDown & 0xFFFD;
				break;
				case SDL_SCANCODE_3: //3
					keysDown = pressed ? keysDown | 0x0001 : keysDown & 0xFFFE;
					//special exception to ensure key presses are always registered for rotation speed
					counter = 0;
				break;
				//acrobatic maneuvers
				case 55:
				case 56:
				case 57:
					doCamera(code);
				break;
				case SDL_SCANCODE_I: //forward flip (i)
					doFlip(0);
				break;
				case SDL_SCANCODE_K: //backward flip (k)
					doFlip(1);
				break;
				case SDL_SCANCODE_L: //right flip (l)
					doFlip(2);
				break;
				case SDL_SCANCODE_J: //left flip (j)
					doFlip(3);
				break;
				case 91:
				case 93:
					doHome(code);
				break;
				case SDL_SCANCODE_0:
					if(pressed) {
						sendVel = !sendVel;
						ROS_INFO("%s velocity publishing!", sendVel ? "Enabled" : "Disabled");
						geometry_msgs::Twist vel;
						velocity.publish(vel);
					}
				break;
				default:
					ROS_ERROR("%d (%d) is an unbound or unrecognized key!", code, modifiers);
			}

		}

		if(spinner-- <= 0) {
			doPub();
			spinner=5;
		}

		r.sleep();
	}

	ros::shutdown();
	return 0;
}

void doPub() {
	// char* s = (char*) malloc(17);
	// for(int i = 0; i < 16; i++) {
	// 	if(((keysDown >> (15-i)) & 1) == 1) {
	// 		s[i] = '1';
	// 	} else {
	// 		s[i] = '0';
	// 	}
	// }
	// s[16] = '\0';
	//
	//
	//
	//
	// ROS_INFO("PUBLISHED VEL WITH INFO: %s", s);
	if(isKeyDown(6)) {
		//land
		flying = false;
		std_msgs::Empty m;
		land.publish(m);
		ROS_INFO("EXECUTING LAND!!");
		return;
	}
	if(isKeyDown(7)) {
		flying = false;
		std_msgs::Empty m;
		reset.publish(m);
		ROS_INFO("EXECUTING EMERGENCY ROTOR STOP!!");
		return;
	}
	if(isKeyDown(8)) {
		//land
		flying = true;
		std_msgs::Empty m;
		takeoff.publish(m);
		ROS_INFO("EXECUTING TAKEOFF!!");
		return;
	}

	if(isKeyDown(13) && !isKeyDown(14)) {
		speed -= SPEED_INCREMENT;
		goto CHECK_SPEED;
	} else if(!isKeyDown(13) && isKeyDown(14)) {
		speed += SPEED_INCREMENT;
		goto CHECK_SPEED;
	}
	goto END_CHECK_SPEED;

	CHECK_SPEED:
	if(speed > 1 - SPEED_INCREMENT/2) speed = 1;
	else if(speed < SPEED_INCREMENT/2) speed = SPEED_INCREMENT;
	ROS_INFO("Speed: %f", speed);
	END_CHECK_SPEED:
	if(isKeyDown(15) && counter % 4 == 0) {
		rotSpeed -= ROTATE_INCREMENT;
		if(rotSpeed < -1 - ROTATE_INCREMENT/2) rotSpeed = 1;
		if(rotSpeed == 0.0) rotSpeed = -ROTATE_INCREMENT;
		ROS_INFO("Rotation speed: %f", rotSpeed);
	}

	geometry_msgs::Twist vel;

	if(isKeyDown(0) && !isKeyDown(2)) {
		//forward
		vel.linear.x = speed;
	} else if(!isKeyDown(0) && isKeyDown(2)) {
		//backward
		vel.linear.x = -speed;
	}

	if(isKeyDown(1) && !isKeyDown(3)) {
		//left
		vel.linear.y = speed;
	} else if(!isKeyDown(1) && isKeyDown(3)) {
		//right
		vel.linear.y = -speed;
	}

	if(isKeyDown(4) && !isKeyDown(5)) {
		//up
		vel.linear.z = speed;
	} else if(!isKeyDown(4) && isKeyDown(5)) {
		//down
		vel.linear.z = -speed;
	}

	if(isKeyDown(11) && !isKeyDown(12)) {
		//rot left
		vel.angular.z = rotSpeed;
	} else if(!isKeyDown(11) && isKeyDown(12)) {
		//right
		vel.angular.z = -rotSpeed;
	}

	if(sendVel) velocity.publish(vel);

	//camera control
	geometry_msgs::Twist cam;
	if(isKeyDown(9) && !isKeyDown(10)) {
		//cam up
		cam.angular.y = (camCurrentRot += CAM_ROTATE_SPEED);
		goto STARTCAM;
	} else if(!isKeyDown(9) && isKeyDown(10)) {
		//cam down
		cam.angular.y = (camCurrentRot -= CAM_ROTATE_SPEED);
		goto STARTCAM;
	}
	goto ENDCAM; //skip cam publishing if no keys pressed
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

	return;
}

void doCamera(uint16_t code) {
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

void doFlip(uint8_t data) {
	std_msgs::UInt8 m;
	m.data = data;
	flip.publish(m);
}

void doHome(uint16_t code) {
	std_msgs::Bool m;
	m.data = (code == 91 ? true : false);
	home.publish(m);
}

bool isKeyDown(uint8_t index) {
	return ((keysDown >> (15-index)) & 1) == 1;
}
