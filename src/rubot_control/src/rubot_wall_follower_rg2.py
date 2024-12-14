#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2

state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'hard left',
    4: 'hard right', 
    5: 'follow wall'
}

def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True


    front_min= int(150 * scanRangesLengthCorrectionFactor)
    front_max = int(210 * scanRangesLengthCorrectionFactor)
    fleft_min = int(210 * scanRangesLengthCorrectionFactor)
    fleft_max = int(270 * scanRangesLengthCorrectionFactor)
    left_min = int(270 * scanRangesLengthCorrectionFactor)
    left_max = int(340 * scanRangesLengthCorrectionFactor)
    right_min = int(20 * scanRangesLengthCorrectionFactor)
    right_max = int(90 * scanRangesLengthCorrectionFactor)
    fright_min = int(90 * scanRangesLengthCorrectionFactor)
    fright_max = int(150 * scanRangesLengthCorrectionFactor)

    # Handle wrapping indices
    def get_ranges(start, end):
        if start < end:
            return msg.ranges[start:end]
        else:
            return msg.ranges[start:] + msg.ranges[:end]

    # Maybe last input is 3
    regions = {
        'right': min(get_ranges(right_min, right_max), default=3),
        'fright': min(get_ranges(fright_min, fright_max), default=3),
        'front': min(get_ranges(front_min, front_max), default=3),
        'fleft': min(get_ranges(fleft_min, fleft_max), default=3),
        'left': min(get_ranges(left_min, left_max), default=3)
    }

    take_action(regions)

def change_state(state):
    global state_, state_dict_
    if state not in state_dict_:
        rospy.logerr(f"Undefined state: {state}")
        return
    if state != state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action(regions):
    msg = Twist()
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    state_description = ''

    # SPECIAL CASES - losing sight of wall
    if regions['front'] > d and regions['fright'] > d and regions['right'] < d * 1.5 and regions['left'] > d and regions['fleft'] > d:
        state_description = 'special case - back right'
        change_state(4) # hard right, no foward movement
    elif regions['front'] > d and regions['fright'] > d and regions['right'] > d and regions['left'] < d * 1.5 and regions['fleft'] > d:
        state_description = 'special case - back left'
        change_state(3)  # hard left, no forward movement

    # SPECIAL CASE - dead end
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'special case - dead end'
        change_state(3)  # hard left
    
    # SPECIAL CASE - between two walls
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] < d:
        state_description = 'special case - between 2 walls'
        change_state(0)

    #SPECIAL CASE - in corner
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] < d and regions['right'] > d:
        state_description = 'special case - in corner, right exposed'
        change_state(4) # hard right, no foward movement
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d and regions['left'] > d and regions['right'] < d:
        state_description = 'special case - in corner, left exposed'
        change_state(3) # hard left, no foward movement

    # Nothing
    elif regions['front'] > d and regions['fright'] > d and regions['right'] > d and regions['left'] > d and regions['fleft'] > d:
        state_description = 'case 0 - nothing'
        change_state(0)

    # Wall ahead, do something
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - front'
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 2 - front and fright'
        change_state(1) # turn left
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 3 - front and fleft'
        change_state(2)  # turn right

    # Wall found, follow wall
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d and regions['right'] < d:
        state_description = 'case 4 - fright'
        change_state(5) # follow the wall
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['left'] < d:
        state_description = 'case 5 - fleft'
        change_state(5) # follow the wall

    else:
        state_description = 'unknown case'

    rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()

def find_wall():
    msg = Twist()
    msg.linear.x = -vx 
    msg.angular.z = 0
    return msg

def turn_left():
    msg = Twist()
    msg.linear.x = msg.linear.x 
    msg.angular.z = wz
    return msg

def hard_left():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = wz
    return msg

def turn_right():
    msg = Twist()
    msg.linear.x = msg.linear.x
    msg.angular.z = -wz
    return msg

def hard_right():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = -wz
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = -vx 
    msg.angular.z = 0
    return msg

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(20)

    d= rospy.get_param("~distance_laser")
    vf= rospy.get_param("~speed_factor")
    vx= rospy.get_param("~forward_speed") 
    wz= rospy.get_param("~rotation_speed") 

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = turn_right()
        elif state_ == 3:
            msg = hard_left()
        elif state_ == 4:
            msg = hard_right()
        elif state_ == 5:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub.publish(msg)
        
        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()



