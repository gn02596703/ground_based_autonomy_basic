The node is modified based on turtlebot teleoperation node.

It will output joystick message \joy from keyboard button input.

You can use this node to try out functions in ground autonomy.

## Quick Start
To test keyboard operation 
launch vehicle simulator without joystick driver node 

(you can disable joystick ps3.launch in local_planner.launch)

```roslaunch vehicle_simulator system.launch```

then, launch the keyboard teleoperation node

```keyboard_teleop keyboard_teleop.launch```

Try out functions with following command

```
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity 
a/d : push to turn robot with fixed angular velocity
space key, s : force stop

m : change autonmy/manual mode in localPlanner (default: manual mode)
o : enable or disable obstacle checking function in localPlanner (default: enable)
c : enable or disalbe clearing cloud in terrain Analysis (default: disable)

CTRL-C to quit
```
 

