#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from math import pi
from std_msgs.msg import String
from control_msgs.msg import *
from trajectory_msgs.msg import *
from serial import Serial
from math import sin
from math import cos
from math import atan2
from math import acos
from math import sqrt
from math import pi        

def get_theta(x,y):
    theta_2 = acos((x**2 + y**2 - l1**2 -l2**2)/(2*l1*l2)) #rad
    theta_1 = atan2(y,x) - atan2(l2*sin(theta_2), l1 + l2*cos(theta_2))
    theta_1 -= pi
    if theta_1 <= -2*pi:
        theta_1 += 2*pi
    
    if theta_1 <= -3.09:
        theta_1 = -3.09
    if theta_1 >= -0.05:
        theta_1 = -0.05
    if theta_2 <= -1.52:
        theta_2 = -1.52
    if theta_2 >= 1.52:
        theta_2 = 1.52

    result = [theta_1, theta_2]
    return result    

def transform(val_l):
    val_l = (val_l - 512.0) / 800.0
    if -0.1 <= val_l <= 0.1:
        val_l = 0
    return val_l  

def move1():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [JointTrajectoryPoint(positions=P, velocities=[0]*6, time_from_start=rospy.Duration(0.5))]
    client.send_goal(g)
    try:
        #client.wait_for_result()
        k = 1
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

global client
l1 = 24.365
l2 = 21.325
x = 0
y = 0
z = 0
x_axis = 0
y_axis = 45.68
z_axis = 0
synchro = time.time()
cnt = 0
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

try:
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"    
except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise
arduino = Serial('/dev/ttyACM0', 9600)

while True:
    if arduino.readable():
        while cnt < 10:
            arduino.readline()
            cnt += 1

        read_val = arduino.readline()
        val = read_val[:-2].decode()
        if val != '':
            x = int(val[val.find('x') + 1 : val.find('y')])
            y = int(val[val.find('y') + 1 : val.find('z')])
            z = int(val[val.find('z') + 1:])
            
            x = transform(x)
            y = transform(y)
            z = transform(z)

    if time.time() - synchro >= 0.1: #100ms
        x_axis += x
        y_axis += y
        z_axis += x
        synchro = time.time()

    try:
        theta = get_theta(x_axis, y_axis)
        theta_1 = theta[0]
        theta_2 = theta[1]
    except:
        try:
            grad = atan2(y_axis,x_axis)
            x_axis = (3.1)*cos(grad)
            y_axis = (3.1)*sin(grad)      
            theta = get_theta(x_axis, y_axis)
            theta_1 = theta[0]
            theta_2 = theta[1]
        except:            
            grad = atan2(y_axis,x_axis)
            x_axis = (45.68)*cos(grad)
            y_axis = (45.68)*sin(grad)      
            theta = get_theta(x_axis, y_axis)
            theta_1 = theta[0]
            theta_2 = theta[1] 

    P = [0., theta_1, theta_2, -pi/2, 0., 0.]
    move1()
    
