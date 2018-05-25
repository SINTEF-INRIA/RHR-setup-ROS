# -*- coding: utf-8 -*-
import socket


from math import pi, cos

import rospy
from std_srvs.srv import Empty

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import ForceCommand
from reflex_msgs.msg import Hand
from reflex_msgs.msg import FingerPressure
from reflex_msgs.srv import SetTactileThreshold, SetTactileThresholdRequest

hand_state = Hand()
negate = 0

def setupRospy():

    global calibrate_fingers, calibrate_tactile
    global enable_tactile_stops, disable_tactile_stops, set_tactile_threshold
    global command_pub, pos_pub, vel_pub, force_pub

    rospy.init_node('ExampleHandNode')

    # Services can automatically call hand calibration
    calibrate_fingers = rospy.ServiceProxy('/reflex_takktile/calibrate_fingers', Empty)
    calibrate_tactile = rospy.ServiceProxy('/reflex_takktile/calibrate_tactile', Empty)

    # Services can set tactile thresholds and enable tactile stops
    enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
    disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
    set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

    # This collection of publishers can be used to command the hand
    command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
    pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
    vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
    force_pub = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

def main():
    setupRospy()



    #SERVERCODE###############3

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # ports 1990-1995 can be used)
    s.bind(('', 1993))
    s.listen(1)

    print("#"*50+"\n\t\tREADY TO RUMBLE\n"+"#"*50)
    
    print("\nTACTILE-STOPS ENABLED BY DEFAULT\n\n")
    enable_tactile_stops()
    

    while True:
        conn, addr = s.accept()
        bytes_received = conn.recv(100)
        stripped = bytes_received.strip()
        split = stripped.split()
        cmnd = str(split[0])
        if not('STATS' == cmnd):
            print(stripped)

        
        if 'POSE' in str(cmnd):
            value = [float(x) for x in split[1:]]
            move_hand(value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7])
            conn.sendall('POSE-DONE\r\n')

        elif 'STATS' in str(cmnd):
            conn.sendall(get_pretty_stats())

        elif 'VEL' in str(cmnd):
            command = str(split[1])
            extra_speed = split[2]
            velocity_controlled(command, extra_speed)
            conn.sendall('VEL-DONE\r\n')

        elif 'THRESH' in str(cmnd):
            thresholds = [float(x) for x in split[1:]]
            set_thresholds(thresholds[0],thresholds[1],thresholds[2],thresholds[3],thresholds[4],thresholds[5])
            conn.sendall('THRESH-DONE\r\n')

        elif 'GOTOLIMIT' in str(cmnd):
            go_to_limit()
            conn.sendall('CLOSED-LIMIT\r\n')
            
        elif 'RESET' in str(cmnd):
            move_pos()
            conn.sendall('RESET-DONE\r\n')

        elif 'CALT' in str(cmnd):
            calibrate_tactile()
            conn.sendall('TACTILE-CALIBRATED\r\n')

        elif 'CALF' in str(cmnd):
            calibrate_fingers()
            conn.sendall('FINGERS-CALIBRATED\r\n')

        elif 'TACON' in str(cmnd):
            enable_tactile_stops()
            conn.sendall('TACTILE-ENABLED\r\n')

        elif 'TACOFF' in str(cmnd):
            disable_tactile_stops()
            conn.sendall('TACTILE-DISABLED\r\n')

        elif 'QUIT' in str(cmnd):
            pos_pub.publish(PoseCommand())
            conn.sendall('QUIT-DONE\r\n')
            break
        elif 'SERVER' in str(cmnd):
            conn.sendall('SERVER-CONNECTED\r\n')
        else:
            conn.sendall('UNKNOWN-CMD\r\n')

            
    s.close()


def hand_state_cb(data):
    global hand_state
    hand_state = data

def move_hand(f1, f2, f3, preshape, v1, v2, v3, vp):
    pose = PoseCommand(f1, f2, f3, preshape)
    velocity = VelocityCommand(v1, v2, v3, vp)
    command_pub.publish(Command(pose, velocity))

def stop_hand(var):
    speed = 0.05 * var
    vel_pub.publish(VelocityCommand())
    pos_pub.publish(PoseCommand(hand_state.motor[0].joint_angle+speed,
                                hand_state.motor[1].joint_angle+speed,
                                hand_state.motor[2].joint_angle+speed,
                                hand_state.motor[3].joint_angle))


def set_thresholds(f1_d, f1, f2_d, f2, f3_d, f3):
    # FINGERPRESSSURE(inner to outer, left to right)
    f1 = FingerPressure([f1,f1,f1,f1,f1,f1_d,f1_d,f1_d,f1_d])
    f2 = FingerPressure([f2,f2,f2,f2,f2,f2_d,f2_d,f2_d,f1_d])
    f3 = FingerPressure([f3,f3,f3,f3,f3,f3_d,f3_d,f3_d,f3_d])
    threshold = SetTactileThresholdRequest([f1,f2,f3])
    set_tactile_threshold(threshold)

def go_to_limit():
    enable_tactile_stops()
    move_hand(2,2,2,hand_state.motor[3].joint_angle,1,1,1,0)
	
    

def xgo_to_limit():
    enable_tactile_stops()
    move_vel(1,1,1,0)
    reached = [False, False, False]
    vel = [0,0,0]
    while(False in reached):
        tactiles = check_finger_true()
        for i in range(3):
            if(tactiles[i] == True):
                vel[i] = 0
                reached[i] = tactiles[i]
            else:
                vel[i] = 1
        move_vel(vel[0],vel[1], vel[2], 0)
        rospy.sleep(0.3)
    rospy.sleep(0.1)
    disable_tactile_stops()
    stop_hand(0)
    print('aegir')
           

def check_finger_true():
    tactiles = [False, False, False]
    for i in range(3):
        if(True in hand_state.finger[i].contact):
            tactiles[i] = True
    return tactiles

def move_pos(f1=0, f2=0, f3=0, preshape=0):    
    pos_pub.publish(PoseCommand(f1, f2, f3, preshape))

def move_vel(v1, v2, v3, preshape=0):
    vel_pub.publish(VelocityCommand(v1, v2, v3, preshape))

def move_force(f1, f2, f3, preshape=0):
    force_pub.publish(ForceCommand(f1,f2,f3, preshape))

def velocity_controlled(command, extra):
    global negate
    print(negate)
    constant_speed = 0.2
    set_speed = constant_speed + float(extra)
    stop_speed = 0.00
    if command == 'CLOSE':
        negate = 1
        v1 = set_speed
        v2 = set_speed
        v3 = set_speed
        preshape = stop_speed
        vel_pub.publish(VelocityCommand(v1, v2, v3, preshape))
    elif command == 'OPEN':
        negate = -1
        v1 = -set_speed
        v2 = -set_speed
        v3 = -set_speed
        preshape = stop_speed
        vel_pub.publish(VelocityCommand(v1, v2, v3, preshape))
    elif command == 'SPLIT':
        negate = 0
        v1 = stop_speed
        v2 = stop_speed
        v3 = stop_speed
        preshape = set_speed
        vel_pub.publish(VelocityCommand(v1, v2, v3, preshape))
    elif command == 'MERGE':
        negate = 0
        v1 = stop_speed
        v2 = stop_speed
        v3 = stop_speed
        preshape = -set_speed
        vel_pub.publish(VelocityCommand(v1, v2, v3, preshape))
    elif command == 'STOP':
        stop_hand(negate)
    else:
        print("WRONG COMMAND")

def get_pretty_stats():
    data = ''
    for i in range(3):
        data += str(round(hand_state.finger[i].proximal,2)) + ' ' + str(round(hand_state.finger[i].distal_approx, 2)) + ' '
        for j in range(9):
            if(hand_state.finger[i].contact[j] == True):
                data += '1'
            else:
                data += '0'
        for j in range(9):
            data += ' ' + str(round(hand_state.finger[i].pressure[j], 0))
        data += '\n'

    for i in range(4):   
        if(hand_state.motor[i].velocity >= 1000):
            fixed_velocity = -1 * (hand_state.motor[i].velocity - 1000)
        else:
            fixed_velocity = hand_state.motor[i].velocity

        data += str(round(hand_state.motor[i].joint_angle, 2)) + ' '
        data += str(fixed_velocity) + ' '
        data += str(round(hand_state.motor[i].load, 2)) + ' '
        data += str(hand_state.motor[i].temperature) + ' '
        data += str(hand_state.motor[i].error_state) + ' '
        data += str(hand_state.motor[i].voltage)
        if(i != 3):
            data += '\n'        
        
    return data + '\r\n'            

if __name__ == '__main__':
    main()

