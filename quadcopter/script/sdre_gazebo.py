#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb

rospy.init_node('sdre', anonymous=True)
pub = rospy.Publisher("/drone/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0

init_height = 10

msg = AttitudeTarget()

###     Goal in gazebo frame with origin as start, from the start point of drone
goal = np.array([0.0, 0.0, 0.0])

goal_body = np.array([0.0, 0.0, 0.0])

x = 0.0
y = 0.0
z = 0.0
v_x = 0.0
v_y = 0.0
x_prev = 0.0
y_prev = 0.0
z_prev = 0.0
v_z = 0.0
area = 0.0

error_head_prev = 0.0

camera_mount = 0.785398
horizontal = 1.04719/2
vertical = 1.04719/2

vel_rover = [0,0,0]

A = np.array([[0, 1, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0]
            ,[0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1]
            ,[0, 0, 0, 0, 0, 0]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                        ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                        ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()

now_p = time.time()

####    msg.x in mavros is -y in gazebo
def land():
    set_mode = rospy.ServiceProxy('/drone/mavros/set_mode', mavros_msgs.srv.SetMode)
    print("LAND")
    #print "set mode: ", set_mode(80,'AUTO>LAND')
    land = rospy.ServiceProxy('/drone/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
    #print "land"


def sdre():
    while not rospy.is_shutdown():
        global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, error_head_prev, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body
        #rospy.loginfo("GOAL_GLOBAL %s", goal) 
        goal_body[0] = goal[0] - x
        goal_body[1] = goal[1] - y
        goal_body[2] = goal[2] - z

        ####    Global to Body rotation

        goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())
        #rospy.loginfo("GOAL_BODY %s", goal_body) 

        #vel_rover = np.dot(Rot,vel_rover)       ####    Velocity transformations to be done
        #vel_rover_body = [vel_rover[1],vel_rover[0],vel_rover[2]]

        Q = np.array([[1+goal_body[0]**2, 0, 0, 0, 0, 0]
                    ,[0, abs(25*(vel_rover[0]-v_x)/(0.001+0.01*goal_body[0]))+2, 0, 0, 0, 0]
                    ,[0, 0, 1+goal_body[1]**2, 0, 0, 0]
                    ,[0, 0, 0, abs(25*(vel_rover[1]-v_y)/(0.001+0.01*goal_body[1]))+2, 0, 0]
                    ,[0, 0, 0, 0, 1+(30*goal_body[2]/sqrt(0.01+goal_body[0]**2+goal_body[1]**2))**2, 0]
                    ,[0, 0, 0, 0, 0, abs(goal_body[0]*goal_body[1])]])

        R = np.array([[80, 0, 0]
                    ,[0, 5000, 0]   #Pitch
                    ,[0, 0, 5000]]) #Roll

        #rospy.loginfo("MATRIX %s", Q)

        ###     Calculation for control done in body fixed frame
        X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z]])

        ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
        B = np.array([[0, 0, 0], [0, -9.8, 0], [0, 0, 0], [0, 0, 9.8], [0, 0, 0], [-1, 0, 0]])

        P = la.solve_continuous_are(A, B, Q, R)

        u = np.dot(-np.linalg.inv(R),B.transpose())
        u = np.dot(u,P)
        u = np.dot(u,X)

        u[0] = (u[0]*1.5 + 14.7)/29.4

        rospy.loginfo("Inputs %s", u)
        ##15 deg max cutoff at 10
        if u[0]>1:
            u[0] = 1
        if u[0]<0:
            u[0] = 0
        if u[1]>6*np.pi/180:
            u[1] = 6*np.pi/180
        if u[1]<-6*np.pi/180:
            u[1] = -6*np.pi/180
        if u[2]>6*np.pi/180:
            u[2] = 6*np.pi/180
        if u[2]<-6*np.pi/180:
            u[2] = -6*np.pi/180

        
        now = time.time()

        quater = tf.transformations.quaternion_from_euler(u[2],u[1],yaw+np.pi/2) #0
        msg.header = Header()
        msg.type_mask = 0
        msg.orientation.x = quater[0]
        msg.orientation.y = quater[1]
        msg.orientation.z = quater[2]
        msg.orientation.w = quater[3]
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0
        msg.thrust = u[0]

        ##VELOCITIES HERE

        pub.publish(msg)
        now_p = time.time()

        #rospy.loginfo("States %s", X)
        #rospy.loginfo("positions %s", [x,y,z])

        rate = rospy.Rate(100) 
        rate.sleep


def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, error_head_prev, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body
    
    ############################        ARDUPILOT-MAVROS COORDINATE FRAME
    ###     Positions in global of mavros frame
    #x = info.pose.pose.position.x
    #y = info.pose.pose.position.y
    #z = info.pose.pose.position.z

    ###     Positions in global gazebo frame
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    ###     All linear velocities are local 
    v_x = info.twist.twist.linear.x
    v_y = info.twist.twist.linear.y
    v_z = info.twist.twist.linear.z
    
    ###     Orientations in global of mavros frame
    a1 = info.pose.pose.orientation.x
    b1 = info.pose.pose.orientation.y
    c1 = info.pose.pose.orientation.z
    d1 = info.pose.pose.orientation.w

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

    ###     Orientations in gazebo frame
    yaw = yaw-np.pi/2
    if yaw<np.pi/2:
        yaw = yaw+2*np.pi/2
    if yaw>np.pi/2:
        yaw = yaw-2*np.pi/2
    #temp = roll
    #roll = pitch
    #pitch = -temp

    #rospy.loginfo("ANGLE OUTSIDE %s", [yaw, pitch, roll])

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()


def ReceiveTar(info):
    global goal, vel_rover, Rot_inertial_to_body
    goal[0] = info.goal.x
    goal[1] = info.goal.y
    goal[2] = info.goal.z
    v1 = info.vel.x
    v2 = info.vel.y
    v = np.array([[v1]
                ,[v2]
                ,[0.0]])
    v = np.dot(Rot_inertial_to_body, v)
    vel_rover[0] = float(v[0])
    vel_rover[1] = float(v[1])
    vel_rover[2] = float(v[2])
                
def listener():
    #rospy.Subscriber("/drone/mavros/local_position/velocity_local", TwistStamped, callback_new)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    rospy.Subscriber('/kalman_filter', kalman, ReceiveTar)
    sdre()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass