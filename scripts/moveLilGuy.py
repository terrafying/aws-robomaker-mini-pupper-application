#! /usr/bin/python
# -*- coding: utf-8 -*-
PACKAGE="maps"

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3
import sys
import select
import random
import os
if os.name == 'nt':
    import msvcrt
    import time
else:
    import tty
    import termios


msg = "Hi"
tlv = 0.0
tav = 0.0
clv = 0.0
cav = 0.0

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def randomWalking(pub):
    twist = Twist()
    nextFwd = random.randint(1, 9)
    print(nextFwd)
    nextAngle = random.randint(0, 9) * .1
    isPos=random.randint(0, 1)
    if isPos == 0:
        nextAngle = - nextAngle
    print(nextAngle)

    twist.linear.x = nextFwd
    twist.linear.y = 0.0
    twist.linear.z = 0.0


    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = nextAngle
    print(twist)
    pub.publish(twist)
    rospy.sleep(nextFwd)
    randomWalking(pub)

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while 1:
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    (rlist, _, _) = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return 'currently:\tlinear vel %s\t angular vel %s ' \
           % (target_linear_vel, target_angular_vel)


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('moveLilGuy')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    LIN_VEL_STEP_SIZE = 4.0
    ANG_VEL_STEP_SIZE = 0.2

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                target_linear_vel = +LIN_VEL_STEP_SIZE
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 's':
                target_linear_vel = -LIN_VEL_STEP_SIZE
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a':
                target_angular_vel = ANG_VEL_STEP_SIZE
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd':
                target_angular_vel = -ANG_VEL_STEP_SIZE
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ':
                print("You hit space")
                randomWalking(pub)

            elif key == '\x7f' or key == 'x':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                tlv = 0.0
                tav = 0.0
                clv = 0.0
                cav = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == '\x03':
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel,
                                                   target_linear_vel, LIN_VEL_STEP_SIZE / 2.0)
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = \
                makeSimpleProfile(control_angular_vel,
                                  target_angular_vel, ANG_VEL_STEP_SIZE
                                  / 2.0)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel
            pub.publish(twist)
    except:
        print(e)
        pass
    finally:

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
