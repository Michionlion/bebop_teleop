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

##### Configuration
Some asset files may need to be configured. You can see what can be changed by examining the `launch/teleop.launch` file, and changing the parameters. Alternatively, you can specify the font_path and circle_path parameters like so:

```
$ roslaunch bebop_teleop bebop_connection.launch font_path:="path/to/font.ttf/otf" circle_path:="path/to/circle.bmp"
```

##### Teleop Keyboard Mappings
|      Key      |        Command         |
|---------------|------------------------|
|       W       |        Forward         |
|       A       |          Left          |
|       S       |        Backward        |
|       D       |          Right         |
|     SPACE     |           Up           |
|     LSHIFT    |          Down          |
|     ENTER     |        Rotor Stop      |
|     RSHIFT    |         Takeoff        |
|      CTRL     |          Land          |
|       I       |      Forward Flip      |
|       J       |        Left Flip       |
|       K       |      Backward Flip     |
|       L       |        Right Flip      |
|    ARROW UP   |        Camera Up       |
|   ARROW DOWN  |       Camera Down      |
|   ARROW LEFT  |       Camera Left      |
|   ARROW RIGHT |       Camera Right     |
|       1       |        Inc. Speed      |
|       2       |        Dec. Speed      |
|       3       |     Inc. Rot. Speed    |
|       4       |     Dec. Rot. Speed    |
|       7       |       Start Patrol     |
|       8       |        Stop Patrol     |
|       0       |      Toggle Control    |
|       [       |    Start Nav. Home     |
|       ]       |     Stop Nav. Home     |
