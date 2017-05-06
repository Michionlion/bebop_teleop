# bebop_teleop
ROS node for the Parrot Bebop (1 or 2) teleop

To install and run this, you'll need `SDL 2`, which you may have to install (`libsdl2-dev`), and `SDL_TTF`. 

To install the `bebop_teleop` package, clone this repo into a catkin workspace's `src` directory (preferably a new/clean one), and run `$ catkin_make`. Then, source the workspace's `setup.(ba)sh`. 

To run the program, first connect to a Bebop's wireless network. Then, run the following commands:

```
$ roslaunch bebop_teleop bebop_connection.launch
$ roslaunch bebop_teleop teleop.launch
```

If a Bebop is not available, but you would still like to test the package, run `$ rosrun bebop_teleop webcam` in place of `$ roslaunch bebop_teleop bebop_connection.launch`. This will replace the Bebop camera feed with webcam feed in the GUI.
