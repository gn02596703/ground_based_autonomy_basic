#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from sensor_msgs.msg import Joy
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

MAX_LIN_VEL = 1 
LIN_VEL_STEP_SIZE = 0.1 

msg = """
Try out ground autonomy functions with keyboard
---------------------------
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
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop')
    pub = rospy.Publisher('joy', Joy, queue_size=10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    LT = 0    # default, manual mode in local Planner
    RT = 0    # default, check obstacle in local Planner
    RB = 1    # default, disable clearing cloud in terrain Analysis

    try:
        print(msg)
        while(1):
            key = getKey()

            # default value for angular velocity
            target_angular_vel = 0.0  

            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = 0.5
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = -0.5
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'm' :
                if LT == -1.0:
                  LT = 0.0
                  print("Manual Mode")
                else:
                  LT = -1.0
                  print("Autonomy Mode")

            elif key == 'o' :
                if RT == -1.0:
                  RT = 0.0
                  print("check obstacle!")
                else:
                  RT = -1.0
                  print("Disable obstacle checking!")

            elif key == 'c' :
                if RB == 1:
                  RB = 0
                  print("Disable clearingCloud")
                else:
                  RB = 1
                  print("clearingCloud")
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            joy = Joy()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            
            control_angular_vel = target_angular_vel
            if abs(control_angular_vel) > 0:
              control_linear_vel = control_linear_vel * 0.3/abs(control_linear_vel)

            # Microsoft Xbox 360 Wired Controller for Linux
            joy.axes.append(0.0) # Left/Right Axis stick left
            joy.axes.append(0.0) # Up/Down Axis stick left
            joy.axes.append(LT) # LT
            joy.axes.append(control_angular_vel) # Left/Right Axis stick right 
            joy.axes.append(control_linear_vel) # Up/Down Axis stick right 
            joy.axes.append(RT) # RT
            joy.axes.append(0.0) # cross key left/right 
            joy.axes.append(0.0) # cross key up/down 


            joy.buttons.append(1) # A
            joy.buttons.append(1) # B
            joy.buttons.append(1) # X
            joy.buttons.append(1) # Y 
            joy.buttons.append(1) # LB 
            joy.buttons.append(RB) # RB
            joy.buttons.append(1) # back 
            joy.buttons.append(1) # start
            joy.buttons.append(1) # power
            joy.buttons.append(1) # Button stick left
            joy.buttons.append(1) # Button stick right

            pub.publish(joy)

    except:
        print(e)

    finally:
      
        joy = Joy()
        joy.axes.append(0.0) # Left/Right Axis stick left
        joy.axes.append(0.0) # Up/Down Axis stick left
        joy.axes.append(0.0) # LT
        joy.axes.append(0.0) # Left/Right Axis stick right 
        joy.axes.append(0.0) # Up/Down Axis stick right 
        joy.axes.append(0.0) # RT
        joy.axes.append(0.0) # cross key left/right 
        joy.axes.append(0.0) # cross key up/down 


        joy.buttons.append(0.0) # A
        joy.buttons.append(0.0) # B
        joy.buttons.append(0.0) # X
        joy.buttons.append(0.0) # Y 
        joy.buttons.append(0.0) # LB 
        joy.buttons.append(0.0) # RB
        joy.buttons.append(0.0) # back 
        joy.buttons.append(0.0) # start
        joy.buttons.append(0.0) # power
        joy.buttons.append(0.0) # Button stick left
        joy.buttons.append(0.0) # Button stick right 

        pub.publish(joy)


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)