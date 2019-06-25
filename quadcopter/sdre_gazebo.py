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
pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0

msg = AttitudeTarget()



###     Goal in gazebo frame with origin as start, from the start point of drone
goal = np.array([10.0, -6.0, 0.0])

#Rot_gazebo_to_mavros = np.array([[0, -1, 0]
#                                ,[1, 0, 0]
#                                ,[0, 0, 1]])
#goal = np.matmul(Rot_gazebo_to_mavros,goal.transpose())

goal_body = np.array([0.0, 0.0, 0.0])

x = 0.0
y = 0.0
z = 0.0
v_x = 0.0
v_y = 0.0
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


now_p = time.time()

####    msg.x in mavros is -y in gazebo
def land():
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    print("LAND")
    #print "set mode: ", set_mode(80,'AUTO>LAND')
    land = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
    #print "land"
'''
def callback_new(info):
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, error_head_prev, goal, goal_body, v_x, v_y, v_z
    ###     All linear velocities are local 
    v_x = info.twist.linear.x
    v_y = info.twist.linear.y
    v_z = info.twist.linear.z

    ###     All angular velocities are local
    v_roll = info.twist.angular.x
    v_pitch = info.twist.angular.y
    v_yaw = info.twist.angular.z
'''

def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, error_head_prev, goal, goal_body, v_x, v_y, v_z
    
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

    ###     All angular velocities are local
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

    #if abs(z)<0.25:
    #    land()
    
    if z!=0:
    #if z!=0:
        ###     Orientations in global of mavros frame
        a1 = info.pose.pose.orientation.x
        b1 = info.pose.pose.orientation.y
        c1 = info.pose.pose.orientation.z
        d1 = info.pose.pose.orientation.w

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([a1,b1,c1,d1])

        ###     Orientations in gazebo frame
        yaw = yaw-np.pi/2
        temp = roll
        roll = pitch
        pitch = -temp
        
        Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
        Rot_inertial_to_body = Rot_body_to_inertial.transpose()

        goal_body[0] = goal[0]-x
        goal_body[1] = goal[1]-y
        goal_body[2] = goal[2]-z

        rospy.loginfo("GOAL %s %s", goal, [x,y,z])

        #rospy.loginfo("GOA1 %s", goal_body)
        ####    Global to Body rotation


        
        goal_body = np.matmul(Rot_inertial_to_body,goal_body.transpose())
        
#########################       TRANSFORM HERE ##############################

        #rospy.loginfo("GOAL2 %s %s", Rot_inertial_to_body, goal_body)
        #rospy.loginfo("GOAL %s", goal_body)
        

        #vel_rover = np.dot(Rot,vel_rover)       ####    Velocity transformations to be done
        #vel_rover_body = [vel_rover[1],vel_rover[0],vel_rover[2]]

        Q = np.array([[1+goal_body[0]**2, 0, 0, 0, 0, 0]
                    ,[0, 1+10*(abs(vel_rover[0]-v_x)/(0.0001+0.1*abs(goal_body[0])))**2, 0, 0, 0, 0]
                    ,[0, 0, 1+goal_body[1]**2, 0, 0, 0]
                    ,[0, 0, 0, 1+10*(abs(vel_rover[1]-v_y)/(0.0001+0.1*abs(goal_body[1])))**2, 0, 0]
                    ,[0, 0, 0, 0, 50*abs(goal_body[2]/(goal_body[0]*goal_body[1])), 0]
                    ,[0, 0, 0, 0, 0, 0.1]])

        R = np.array([[100, 0, 0]
                    ,[0, 10000, 0]   #Pitch
                    ,[0, 0, 10000]]) #Roll


        ###     Calculation for control done in body fixed frame

        X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z]])

        ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
        B = np.array([[0, 0, 0], [0, -9.8, 0], [0, 0, 0], [0, 0, 9.8], [0, 0, 0], [-1, 0, 0]])

        #(P,L,G) = mb.care(A, B, Q, R)
        P = la.solve_continuous_are(A, B, Q, R)

        u = np.matmul(-np.linalg.inv(R),B.transpose())
        u = np.matmul(u,P)
        u = np.dot(u,X)
        
        #rospy.loginfo("U%s",u)

        #Rot = Rot.transpose()
        #inputs = np.dot(Rot,[[0], [0], [u[0]]])

        u[0] = (u[0]*1.5 + 14.7)/29.4
        #u[0] = (u[0]+9.8)*0.5/9.8
        #u[0] = (u[0]+9.8)*0.5/9.8

        #rospy.loginfo("Inputs %s", u)
        if u[0]>1:
            u[0] = 1
        if u[1]>15*np.pi/180:
            u[1] = 15*np.pi/180
        if u[1]<-15*np.pi/180:
            u[1] = -15*np.pi/180
        if u[2]>15*np.pi/180:
            u[2] = 15*np.pi/180
        if u[2]<-15*np.pi/180:
            u[2] = -15*np.pi/180

        
        now = time.time()

        
        ###     All quaternion info input in gazebo frame and has to be transformed to mavros frame
        #mav_roll = -u[1]
        #mav_pitch = u[2]
        #mav_yaw = yaw+np.pi/2

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
        #error_head_prev = error_head

        rospy.loginfo("States %s", X)
        #rospy.loginfo("Inputs %s", u)
        #rospy.loginfo("Time %s", now)
        #rospy.loginfo("ANGLES %s",[roll, pitch, yaw])

    rate = rospy.Rate(10) 


def ReceiveTar(data):
    global goal, x, y, z, roll, pitch, yaw, camera_mount, horizontal, vertical, area
    xt_image=data.contour.center.x
    yt_image=data.contour.center.y
    width=data.contour.width
    height=data.contour.height
    detect = data.detected

    if detect==0 or ((width<50 or height<50) and abs(width-height)>25):
        goal[0] = goal[0]
        goal[1] = goal[1]
    
    else:
        Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
        Rot_inertial_to_body = Rot_body_to_inertial.transpose()

        pos = np.array([[x]
                        ,[y]
                        ,[z]])

        new = np.matmul(Rot_inertial_to_body, pos)

        #rospy.loginfo("NEW %s", float(new[2]))
        z_n = float(new[2])
        M1 = np.array([[z_n/tan(np.pi/12), z_n*tan(np.pi/6)/sin(np.pi/12), 1, 0, 0, 0, 0, 0, 0]
                    ,[0, 0, 0, z_n/tan(np.pi/12), z_n*tan(np.pi/6)/sin(np.pi/12), 1, 0, 0, 0]
                    ,[z_n/tan(np.pi/12), 0, 1, 0, 0, 0, -250*z_n/tan(np.pi/12), 0, -250*1]
                    ,[0, 0, 0, z_n/tan(np.pi/12), 0, 1, 0, 0, 0]
                    ,[z_n/tan(np.pi/12), -z_n*tan(np.pi/6)/sin(np.pi/12), 1, 0, 0, 0, -500*z_n/tan(np.pi/12), 500*z_n*tan(np.pi/6)/sin(np.pi/12), -500*1]
                    ,[0, 0, 0, z_n/tan(np.pi/12), -z_n*tan(np.pi/6)/sin(np.pi/12), 1, 0, 0, 0]
                    ,[z_n/tan(np.pi/4), 0, 1, 0, 0, 0, -250*z_n/tan(np.pi/4), 0, -250*1]
                    ,[0, 0, 0, z_n/tan(np.pi/4), 0, 1, -250*z_n/tan(np.pi/4), 0, -250*1]
                    ,[z_n/tan(5*np.pi/12), z_n*tan(np.pi/6)/sin(5*np.pi/12), 1, 0, 0, 0, 0, 0, 0]])

        M2 = np.array([[xt_image]
                    ,[yt_image]
                    ,[1]])

        U, D, V = np.linalg.svd(M1)
        M = np.reshape(V[8], (3,3))
        #rospy.loginfo("DATA %s", M1)
        M = np.linalg.inv(M)        

        goalx = np.matmul(M[0], M2)/np.matmul(M[2], M2)
        goaly = np.matmul(M[1], M2)/np.matmul(M[2], M2)
        
        goal_prev = np.array([[float(goalx)]
                            ,[float(goaly)]
                            ,[0]])
        #rospy.loginfo("NEW %s %s", Rot_body_to_inertial, goal_prev)
        goal_new = np.matmul(Rot_body_to_inertial, goal_prev)
        
        goal[0] = x+goal_new[0]
        goal[1] = y+goal_new[1]



        #goal[0] = x + goalx*cos(yaw) - goaly*sin(yaw)
        #goal[1] = y + goalx*sin(yaw) + goaly*cos(yaw)

    rospy.loginfo("WIDTH %s, HEIGHT %s, detect%s", width, height, detect)

    
                
def listener():
    #rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback_new)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, callback)
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass