#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import time

pub = None
d = 0
v = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2

state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'move forward',
    4: 'move left', 
    5: 'move right',
    6: 'move back',
}

def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True


    front_min= int(338 * scanRangesLengthCorrectionFactor)
    front_max = int(23 * scanRangesLengthCorrectionFactor)
    fleft_min = int(23 * scanRangesLengthCorrectionFactor)
    fleft_max = int(68 * scanRangesLengthCorrectionFactor)
    left_min = int(68 * scanRangesLengthCorrectionFactor)
    left_max = int(113 * scanRangesLengthCorrectionFactor)
    bleft_min = int(113 * scanRangesLengthCorrectionFactor)
    bleft_max = int(158 * scanRangesLengthCorrectionFactor)
    back_min = int(158 * scanRangesLengthCorrectionFactor)
    back_max = int(203 * scanRangesLengthCorrectionFactor)
    bright_min = int(203 * scanRangesLengthCorrectionFactor)
    bright_max = int(248 * scanRangesLengthCorrectionFactor)
    right_min = int(248 * scanRangesLengthCorrectionFactor)
    right_max = int(293 * scanRangesLengthCorrectionFactor)
    fright_min = int(293 * scanRangesLengthCorrectionFactor)
    fright_max = int(338 * scanRangesLengthCorrectionFactor)

    # Handle wrapping indices
    def get_ranges(start, end):
        if start < end:
            return msg.ranges[start:end]
        else:
            return msg.ranges[start:] + msg.ranges[:end]

    # Maybe last input is 3
    regions = {
        'front': min(get_ranges(front_min, front_max), default=3),
        'fleft': min(get_ranges(fleft_min, fleft_max), default=3),
        'left': min(get_ranges(left_min, left_max), default=3),
        'bleft': min(get_ranges(bleft_min, bleft_max), default=3),
        'back': min(get_ranges(back_min, back_max), default=3),
        'bright': min(get_ranges(bright_min, bright_max), default=3),
        'right': min(get_ranges(right_min, right_max), default=3),
        'fright': min(get_ranges(fright_min, fright_max), default=3)
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
    global route, movement, wall, adjustments
    msg = Twist()
    

    # I removed 3 lines here refer back to differential drive code if needed

    state_description = ''


    # Wall found, Establish Route
    if route is None:

        if regions['front'] < d and regions['fright'] < d and regions['fleft'] < d:
            state_description = 'Case None: Route now LEFT, front and fright and fleft'
            change_state(4) # drive left
            route = 'Left'
            wall = 'Front'
            

        elif regions['front'] < d and regions['fright'] < d:
            state_description = 'Case None: Route now LEFT, front and fright'
            change_state(2)  # rotate right, no forward movement
            route = 'Left'
            wall = 'Front'
            

        elif regions['fright'] < d and regions['right'] < d:
            state_description = 'Case None: Route now LEFT, fright and right'
            change_state(2)  # rotate right, no forward movement
            route = 'Left'
            wall = 'Front'
            

        elif regions['front'] < d and  regions['fleft'] < d:
            state_description = 'Case None: Route now RIGHT, front and fleft'
            change_state(1)  # rotate left, no forward movement
            route = 'Right'
            wall = 'Front'
            

        elif regions['fleft'] < d and  regions['left'] < d:
            state_description = 'Case None: Route now RIGHT, fright and right'
            change_state(1)  # rotate left, no forward movement
            route = 'Right'
            wall = 'Front'
            

        # Nothing
        elif all(value > d for value in regions.values()):
            state_description = 'case nothing'
            change_state(0)
            route = None
            movement = None
            wall = None

    # Robot on Left first route
    elif route != None:

        if movement == None:
            movement = route

        if movement == 'Left' and wall == 'Front':

            #Specail Case - In corner 
            if regions['left'] < d and regions['front'] < d and regions['bleft'] < d * 1.5 and regions['fright'] < d * 1.5 and regions['back'] > d and regions['right'] > d and regions['bright'] > d * 2:
                state_description = 'Case Left, Front: In Top Left corner'
                movement = 'Backward'
                wall = 'Left'
                change_state(6) # drive Backward

            #Specail Case - Deadend Right
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'right') and regions['right'] > d * 1.5:
                state_description = 'Case Left, Front: Deadend'
                while regions['back'] > d:
                    change_state(6)  # drive backward
                movement = 'Right'
                wall = 'Back'
                change_state(5) # drive right

            #Specail Case - Too close to wall
            elif regions['front'] < d * 0.5 and regions['fright'] < d and regions['fleft'] < d:
                state_description = 'Case Left, Front: Too close to wall'
                change_state(6) # drive backward

            elif regions['front'] < d and regions['fright'] < d and regions['fleft'] < d:
                adjustments = 0
                state_description = 'Case Left, Front: front and fright and fleft'
                change_state(4) # drive left

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['front'] < d:
                state_description = 'Case Left, Front: Too Many Adjustments'
                adjustments = 0
                change_state(4) # drive left
            
            elif regions['front'] < d and regions['fright'] < d and regions['fleft'] > d:
                adjustments += 1
                state_description = 'Case Left, Front: No fleft reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            elif regions['front'] < d and regions['fright'] > d and regions['fleft'] < d:
                adjustments += 1
                state_description = 'Case Left, Front: No fright reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            #Special Case - Small wall
            elif regions['front'] < d * 1.25 and (regions['fleft'] > d * 2 or regions['fleft'] > d * 2):
                state_description = 'Case Left, Front: Small Wall'
                while regions['fleft'] > d * 1.25:
                    change_state(2)  # rotate right
                change_state(4) # drive left

            # Special Case - Losing Front Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'fright') and regions['fright'] < d * 1.25:
                state_description = 'Case Left, Front: Only fright, Movement is Forward'
                change_state(4)  # drive left
                rospy.sleep(0.5)
                movement = 'Forward'
                wall = 'Right'
                change_state(3) # drive forward


        elif movement == 'Left' and wall == 'Back':

            #Specail Case - In Left corner 
            if regions['left'] < d and regions['back'] < d and regions['fleft'] < d * 1.5 and regions['bright'] < d * 1.5 and regions['right'] > d and regions['front'] > d and regions['fright'] > d * 2:
                state_description = 'Case Left, Back: In Bottom Left Corner'
                movement = 'Forward'
                wall = 'Left'
                change_state(3) # drive forward

            #Specail Case - Deadend Back
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'front') and regions['front'] > d * 1.5:
                state_description = 'Case Left, Back: Deadend'
                while regions['front'] > d:
                    change_state(3)  # drive forward
                
                movement = 'Right'
                wall = 'Front'
                change_state(5) # drive right

            #Specail Case - Too close to wall
            elif regions['back'] < d * 0.5 and regions['bright'] < d and regions['bleft'] < d:
                state_description = 'Case Left, Back: Too close to wall'
                change_state(3) # drive forward

            elif regions['back'] < d and regions['bright'] < d and regions['bleft'] < d:
                adjustments = 0
                state_description = 'Case Left, Back: back and bright and bleft'
                change_state(4) # drive left

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['back'] < d:
                state_description = 'Case Left, Back: Too Many Adjustments'
                adjustments = 0
                change_state(4) # drive left

            elif regions['back'] < d and regions['bright'] < d and regions['bleft'] > d:
                adjustments += 1
                state_description = 'Case Left, Back: No bleft reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            elif regions['back'] < d and regions['bright'] > d and regions['bleft'] < d:
                adjustments += 1
                state_description = 'Case Left, Back: No bright reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            #Special Case - Small wall
            elif regions['back'] < d * 1.25 and (regions['bleft'] > d * 2 or regions['bright'] > d * 2):
                state_description = 'Case Left, Back: Small Wall'
                while regions['bleft'] > d:
                    change_state(1)  # rotate left
                change_state(4) # drive left

            # Special Case - Losing Back Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'bright') and regions['bright'] < d * 1.5:
                state_description = 'Case Left, Back: Only bright, Movement is Backward'
                change_state(4)  # drive left
                rospy.sleep(0.5)
                movement = 'Backward'
                wall = 'Right'
                change_state(6) # drive backward

        elif movement == 'Right' and wall == 'Front':

            #Specail Case - In Right corner 
            if regions['right'] < d and regions['front'] < d and regions['bright'] < d * 1.5 and regions['fleft'] < d * 1.5 and regions['left'] > d and regions['back'] > d and regions['bleft'] > d * 2:
                state_description = 'Case Right, Front: In Top Left Corner'
                movement = 'Backward'
                wall = 'Right'
                change_state(6) # drive Backward

            #Specail Case - Deadend Right
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'left') and regions['left'] > d * 1.5:
                state_description = 'Case Right, Front: Deadend'
                while regions['back'] > d:
                    change_state(6)  # drive backward
                
                movement = 'Left'
                wall = 'Back'
                change_state(4) # drive left

            #Specail Case - Too close to wall
            elif regions['front'] < d * 0.5 and regions['fright'] < d and regions['fleft'] < d:
                state_description = 'Case Right, Front: Too close to wall'
                change_state(6) # drive backward

            elif regions['front'] < d and regions['fright'] < d and regions['fleft'] < d:
                adjustments = 0
                state_description = 'Case Right, Front: front and fright and fleft'
                change_state(5) # drive right

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['front'] < d:
                state_description = 'Case Right, Front: Too Many Adjustments'
                adjustments = 0
                change_state(5) # drive right

            elif regions['front'] < d and regions['fright'] < d and regions['fleft'] > d:
                adjustments += 1
                state_description = 'Case Right, Front: No fleft reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            elif regions['front'] < d and regions['fright'] > d and regions['fleft'] < d:
                adjustments += 1
                state_description = 'Case Right, Front: No fright reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            #Special Case - Small wall
            elif regions['front'] < d * 1.25 and (regions['fright'] > d * 2 or regions['fleft'] > d * 2):
                state_description = 'Case Right, Front: Small Wall'
                while regions['fright'] > d:
                    change_state(1)  # rotate left
                change_state(5) # drive right

            # Special Case - Losing Front Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'fleft') and regions['fleft'] < d * 1.5:
                state_description = 'Case Right, Front: Only fleft, Movement is Forward'
                change_state(5)  # drive right
                rospy.sleep(0.5)
                movement = 'Forward'
                wall = 'Left'
                change_state(3) # drive forward


        elif movement == 'Right' and wall == 'Back':

            #Specail Case - In Right corner 
            if regions['right'] < d and regions['back'] < d and regions['fright'] < d * 1.5 and regions['bleft'] < d * 1.5 and regions['left'] > d and regions['front'] > d and regions['fleft'] > d * 2:
                state_description = 'Case Right, Back: In Bottom Right corner'
                movement = 'Forward'
                wall = 'Right'
                change_state(3) # drive forward

            #Specail Case - Deadend Right
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'left') and regions['left'] > d * 1.5:
                state_description = 'Case Right, Back: Deadend'
                while regions['front'] > d:
                    change_state(3)  # drive forward
                
                movement = 'Left'
                wall = 'Front'
                change_state(4) # drive left

            #Specail Case - Too close to wall
            elif regions['back'] < d * 0.5 and regions['bright'] < d and regions['bleft'] < d:
                state_description = 'Case Right, Back: Too close to wall'
                change_state(3) # drive forward

            elif regions['back'] < d and regions['bright'] < d and regions['bleft'] < d:
                adjustments = 0
                state_description = 'Case Right, Back: back and bright and bleft'
                change_state(5) # drive right

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['back'] < d:
                state_description = 'Case Right, Back: Too Many Adjustments'
                adjustments = 0
                change_state(5) # drive right

            elif regions['back'] < d and regions['bright'] < d and regions['bleft'] > d:
                adjustments += 1
                state_description = 'Case Right, Back: No bleft reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            elif regions['back'] < d and regions['bright'] > d and regions['bleft'] < d:
                adjustments += 1
                state_description = 'Case Right, Back: No bright reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            #Special Case - Small wall
            elif regions['back'] < d * 1.25 and (regions['bright'] > d * 2 or regions['bleft'] > d * 2):
                state_description = 'Case Right, Back: Small Wall'
                while regions['bright'] > d:
                    change_state(2)  # rotate right
                change_state(5) # drive right

            # Special Case - Losing Back Wall
            elif all(value > d for key, value in regions.items() if key != 'bleft') and regions['bleft'] < d * 1.5:
                state_description = 'Case Right, Back: Only bleft, Movement is Backward'
                change_state(5)  # drive right
                rospy.sleep(0.5)
                movement = 'Backward'
                wall = 'Left'
                change_state(6) # drive backward


        elif movement == 'Backward' and wall == 'Right':

            #Specail Case - In Right corner 
            if regions['right'] < d and regions['back'] < d and regions['fright'] < d * 1.5 and regions['bleft'] < d * 1.5 and regions['left'] > d and regions['back'] > d and regions['fleft'] > d * 2:
                state_description = 'Case Backward, Right: In Bottom Right corner'
                movement = 'Left'
                wall = 'Back'
                change_state(4) # drive left

            #Specail Case - Deadend Back
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'front') and regions['front'] > d * 1.5:
                state_description = 'Case Backward, Right: Deadend'
                while regions['left'] > d:
                    change_state(4)  # drive left
                
                movement = 'Forward'
                wall = 'Left'
                change_state(4) # drive left

            #Specail Case - Too close to wall
            elif regions['right'] < d * 0.5 and regions['fright'] < d and regions['bright'] < d:
                state_description = 'Case Backward, Right: Too close to wall'
                change_state(4) # drive left

            elif regions['right'] < d and regions['fright'] < d and regions['bright'] < d:
                adjustments = 0
                state_description = 'Case Backward, Right: right and fright and bright'
                change_state(6) # drive backward

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['right'] < d:
                state_description = 'Case Backward, Right: Too Many Adjustments'
                adjustments = 0
                change_state(6) # drive backwards

            elif regions['right'] < d and regions['fright'] < d and regions['bright'] > d:
                adjustments += 1
                state_description = 'Case Backward, Right: No bright reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            elif regions['right'] < d and regions['fright'] > d and regions['bright'] < d:
                adjustments += 1
                state_description = 'Case Backward, Right: No fright reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            #Special Case - Small wall
            elif regions['right'] < d * 1.25 and (regions['bright'] > d * 2 or regions['fright'] > d * 2):
                state_description = 'Case Backward, Right: Small Wall'
                while regions['bright'] > d:
                    change_state(1)  # rotate left
                change_state(6) # drive backwards

            # Special Case - Losing Right Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'fright') and regions['fright'] < d * 1.5:
                state_description = 'Case Backward, Right: Only fright, Movement is Right'
                change_state(6)  # drive backward
                rospy.sleep(0.5)
                movement = 'Right'
                wall = 'Front'
                change_state(5) # drive right


        elif movement == 'Backward' and wall == 'Left':

            #Specail Case - In Left corner 
            if regions['left'] < d and regions['back'] < d and regions['fleft'] < d * 1.5 and regions['bright'] < d * 1.5 and regions['right'] > d and regions['front'] > d and regions['fright'] > d * 2:
                state_description = 'Case Backward, Left: In Bottom Left Corner'
                movement = 'Right'
                wall = 'Back'
                change_state(5) # drive right

            #Specail Case - Deadend Back
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'front') and regions['front'] > d * 1.5:
                state_description = 'Case Backward, Left: Deadend'
                while regions['right'] > d:
                    change_state(5)  # drive right
                
                movement = 'Forward'
                wall = 'Right'
                change_state(3) # drive forward

            #Specail Case - Too close to wall
            elif regions['left'] < d * 0.5 and regions['fleft'] < d and regions['bleft'] < d:
                state_description = 'Case Backward, Left: Too close to wall'
                change_state(5) # drive right

            elif regions['left'] < d and regions['fleft'] < d and regions['bleft'] < d:
                adjustments = 0
                state_description = 'Case Backward, Left: left and fleft and bleft'
                change_state(6) # drive backward

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['left'] < d:
                state_description = 'Case Backward, Left: Too Many Adjustments'
                adjustments = 0
                change_state(6) # drive backwards

            elif regions['left'] < d and regions['fleft'] < d and regions['bleft'] > d:
                adjustments += 1
                state_description = 'Case Backward, Left: No bleft reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            elif regions['left'] < d and regions['fright'] > d and regions['bright'] < d:
                adjustments += 1
                state_description = 'Case Backward, Left: No fleft reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            #Special Case - Small wall
            elif regions['left'] < d * 1.25 and (regions['bleft'] > d * 2 or regions['fleft'] > d * 2):
                state_description = 'Case Backward, Left: Small Wall'
                while regions['bleft'] > d:
                    change_state(2)  # rotate right
                change_state(6) # drive backwards

            # Special Case - Losing Left Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'fleft') and regions['fleft'] < d * 1.5:
                state_description = 'Case Backward, Left: Only fleft, Movement is Left'
                change_state(6)  # drive backward
                rospy.sleep(0.5)
                movement = 'Left'
                wall = 'Front'
                change_state(4) # drive left


        elif movement == 'Forward' and wall == 'Right':

            #Specail Case - In Right corner 
            if regions['right'] < d and regions['front'] < d and regions['bright'] < d * 1.5 and regions['fleft'] < d * 1.5 and regions['left'] > d and regions['back'] > d and regions['bleft'] > d * 2:
                state_description = 'Case Forward, Right: In Top Left Corner'
                movement = 'Left'
                wall = 'Front'
                change_state(4) # drive left

            #Specail Case - Deadend Front
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'back') and regions['back'] > d * 1.5:
                state_description = 'Case Forward, Right: Deadend'
                while regions['left'] > d:
                    change_state(4)  # drive left
                
                movement = 'Backward'
                wall = 'Left'
                change_state(6) # drive backward

            #Specail Case - Too close to wall
            elif regions['right'] < d * 0.5 and regions['fright'] < d and regions['bright'] < d:
                state_description = 'Case Forward, Right: Too close to wall'
                change_state(4) # drive left

            elif regions['right'] < d and regions['fright'] < d and regions['bright'] < d:
                adjustments = 0
                state_description = 'Case Forward, Right: right and fright and bright'
                change_state(3) # drive forward

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['right'] < d:
                state_description = 'Case Forward, Right: Too Many Adjustments'
                adjustments = 0
                change_state(3) # drive forward

            elif regions['right'] < d and regions['fright'] < d and regions['bright'] > d:
                adjustments += 1
                state_description = 'Case Forward, Right: No bright reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            elif regions['right'] < d and regions['fright'] > d and regions['bright'] < d:
                adjustments += 1
                state_description = 'Case Forward, Right: No fright reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            #Special Case - Small wall
            elif regions['right'] < d * 1.25 and (regions['fright'] > d * 2 or regions['bright'] > d * 2):
                state_description = 'Case Forward, Right: Small Wall'
                while regions['fright'] > d:
                    change_state(2)  # rotate right
                change_state(3) # drive forward

            # Special Case - Losing Right Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'bright') and regions['bright'] < d * 1.5:
                state_description = 'Case Forward, Right: Only bright, Movement is Right'
                change_state(3)  # drive forward
                rospy.sleep(0.5)
                movement = 'Right'
                wall = 'Back'
                change_state(5) # drive right


        elif movement == 'Forward' and wall == 'Left':

            #Specail Case - In Right corner 
            if regions['left'] < d and regions['front'] < d and regions['bleft'] < d * 1.5 and regions['fright'] < d * 1.5 and regions['back'] > d and regions['right'] > d and regions['bright'] > d * 2:
                state_description = 'Case Forward, Left: In Top Left corner'
                movement = 'Right'
                wall = 'Front'
                change_state(5) # drive right

            #Specail Case - Deadend Front
            elif all(value < d * 1.25 for key, value in regions.items() if key != 'back') and regions['back'] > d * 1.5:
                state_description = 'Case Forward, Left: Deadend'
                while regions['right'] > d:
                    change_state(5)  # drive right
                
                movement = 'Backward'
                wall = 'Right'
                change_state(6) # drive backward
            
            #Specail Case - Too close to wall
            elif regions['left'] < d * 0.5 and regions['fleft'] < d and regions['bleft'] < d:
                state_description = 'Case Forward, Left: Too close to wall'
                change_state(5) # drive right

            elif regions['left'] < d and regions['fleft'] < d and regions['bleft'] < d:
                adjustments = 0
                state_description = 'Case Forward, Left: left and fleft and bleft'
                change_state(3) # drive forward

            #Special Case: Too Many Adjustments
            elif adjustments == 4 and regions['left'] < d:
                state_description = 'Case Forward, Left: Too Many Adjustments'
                adjustments = 0
                change_state(3) # drive forward

            elif regions['left'] < d and regions['fleft'] > d and regions['fleft'] < d:
                adjustments += 1
                state_description = 'Case Forward, Left: No fleft reading, Rotate left adjustment'
                change_state(1)  # rotate left, no forward movement

            elif regions['left'] < d and regions['fleft'] < d and regions['bleft'] > d:
                adjustments += 1
                state_description = 'Case Forward, Left: No bleft reading, Rotate right adjustment'
                change_state(2)  # rotate right, no forward movement

            #Special Case - Small wall
            elif regions['left'] < d * 1.25 and (regions['fleft'] > d * 2 or regions['bleft'] > d * 2):
                state_description = 'Case Forward, Left: Small Wall'
                while regions['fleft'] > d:
                    change_state(1)  # rotate left
                change_state(3) # drive forward

            # Special Case - Losing Left Wall
            elif all(value > d * 1.5 for key, value in regions.items() if key != 'bleft') and regions['bleft'] < d * 1.5:
                state_description = 'Case Forward, Left: Only bleft, Movement is Left'
                change_state(3)  # drive forward
                rospy.sleep(0.5)
                movement = 'Left'
                wall = 'Back'
                change_state(4) # drive left

    else:
        state_description = 'unknown case'
    # FOR DEBUGGING
    # rospy.loginfo(regions)

    print(state_description)
     # I removed 3 lines here refer back to differential drive code if needed
    #  pub.publish(msg)
    rate.sleep()

def find_wall():
    msg = Twist()
    msg.linear.x = v
    msg.linear.y = 0
    msg.angular.z = 0
    return msg

def turn_left():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = wz
    return msg

def turn_right():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = -wz
    return msg

def move_forward():
    msg = Twist()
    msg.linear.x = v
    msg.linear.y = 0
    msg.angular.z = 0
    return msg

def move_left():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = v
    msg.angular.z = 0
    return msg

def move_right():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = -v
    msg.angular.z = 0
    return msg

def move_back():
    msg = Twist()
    msg.linear.x = -v
    msg.linear.y = 0
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
    global v
    global wz
    global vf
    global route, movement, wall, adjustments

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(20)

    d = rospy.get_param("~distance_laser")
    vf = rospy.get_param("~speed_factor")
    v = rospy.get_param("~forward_speed") 
    wz = rospy.get_param("~rotation_speed") 

    route = None
    movement = None
    wall = None
    adjustments = 0

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = turn_right()
        elif state_ == 3:
            msg = move_forward()
        elif state_ == 4:
            msg = move_left()
        elif state_ == 5:
            msg = move_right()
        elif state_ == 6:
            msg = move_back()
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



