#!/usr/bin/env python3
import rospy
from rubot_control import move_rubot
from math import sqrt,sin,cos,pi


def square_path(v,td):
    move_rubot(v,0,0,td)
    move_rubot(0,v,0,td)
    move_rubot(-v,0,0,td)
    move_rubot(0,-v,0,td)


def triangular_path(v, td):
    move_rubot(v,0,0,td)
    move_rubot(-v,v,0,td/sqrt(2))
    move_rubot(-v,-v,0,td/sqrt(2))

def figure8_path(v, td, loop_time=5):
    steps = 50  # High number of steps for smooth motion
    dt = (td / steps) / 2  # Adjust the time duration for faster motion
    scaling_factor = 0.2  # Smaller value for a significantly tighter radius

    # First half-circle (counter-clockwise loop)
    for i in range(steps):
        angle = 2 * pi * (i / steps)  # Varies from 0 to 2π
        x_vel = v * cos(angle) * scaling_factor
        y_vel = v * sin(angle) * scaling_factor
        move_rubot(x_vel, y_vel, 0, dt)  # Move with smaller adjusted velocities

    # Second half-circle (clockwise loop)
    for i in range(steps):
        angle = 2 * pi * (i / steps)  # Varies from 0 to 2π
        x_vel = v * cos(angle) * scaling_factor
        y_vel = -v * sin(angle) * scaling_factor
        move_rubot(x_vel, y_vel, 0, dt)  # Move with smaller adjusted velocities



if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        td= rospy.get_param("~td")
        path= rospy.get_param("~path")

        print(f"Received path parameter: {path}")
        print("")

        if path == "Square":
            square_path(v, td)

        elif path == "Triangular":
            triangular_path(v, td)

        elif path == "Figure8":
            figure8_path(v, td)

        else:
            print('Error: unknown movement')

    except rospy.ROSInterruptException:
        pass
