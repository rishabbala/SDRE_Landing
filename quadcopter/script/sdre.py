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
from timeit import default_timer as timer


rospy.init_node('sdre', anonymous=True)
pub = rospy.Publisher("/drone/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)


roll = 0.0
pitch = 0.0
yaw = 0.0
detect = 1
rot_flag = 0
theta = 0.0
theta_p = 0.0

yaw_dot = 0.0

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

camera_mount = 0.785398
horizontal = 1.04719/2
vertical = 1.04719/2

vel_rover = [0,0,0]

A = np.array([[0, 1, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 1, 0]
            ,[0, 0, 0, 0, 0, 0, 0]
            ,[0, 0, 0, 0, 0, 0, 0]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                        ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                        ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()

now_p = timer()
now = timer()
head = 0.0
head_prev = 0.0

####    msg.x in mavros is -y in gazebo
def land():
    set_mode = rospy.ServiceProxy('/drone/mavros/set_mode', mavros_msgs.srv.SetMode)
    print("LAND")
    #print "set mode: ", set_mode(80,'AUTO>LAND')
    land = rospy.ServiceProxy('/drone/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
    #print "land"


def sdre():
    while not rospy.is_shutdown():
        global detect, x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, head, now_p, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body, yaw_dot, theta, rot_flag, head
        #rospy.loginfo("GOAL_GLOBAL %s", goal) 
        goal_body[0] = goal[0] - x
        goal_body[1] = goal[1] - y
        goal_body[2] = goal[2] - z

        ####    Global to Body rotation

        goal_body = np.dot(Rot_inertial_to_body,goal_body.transpose())
        #rospy.loginfo("GOAL_BODY %s", goal_body) 

        #vel_rover = np.dot(Rot,vel_rover)       ####    Velocity transformations to be done
        #vel_rover_body = [vel_rover[1],vel_rover[0],vel_rover[2]]

        Q = np.array([[((4.5*goal_body[0])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0, 0, 0, 0]
                ,[0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[0]-v_x)/(0.001+0.1*abs(goal_body[0]))), 0, 0, 0, 0, 0]
                ,[0, 0, ((4.5*goal_body[1])**2)/abs(goal_body[2]+0.0001)+1, 0, 0, 0, 0]
                ,[0, 0, 0, abs(150*(0.5+abs(goal_body[2]))*(vel_rover[1]-v_y)/(0.001+0.1*abs(goal_body[1]))), 0, 0, 0]
                ,[0, 0, 0, 0, 1+(10*goal_body[2]/sqrt(0.01+0.01*(goal_body[0]**2)+0.01*(goal_body[1]**2)))**2, 0, 0]
                ,[0, 0, 0, 0, 0, 1/abs(goal_body[2]+0.001), 0]
                ,[0, 0, 0, 0, 0, 0, 3*abs((theta+0.0001)*180/np.pi)]])

        R = np.array([[800, 0, 0, 0]
                    ,[0, 75000, 0, 0]   #Pitch
                    ,[0, 0, 75000, 0]
                    ,[0, 0, 0, 1000]]) #Roll


        ###     Calculation for control done in body fixed frame
        X = np.array([[goal_body[0]],[vel_rover[0]-v_x],[goal_body[1]],[vel_rover[1]-v_y],[goal_body[2]],[vel_rover[2]-v_z],[theta]])

        ###     d2(e_x)/dt2 = 0-d2(x)/dt2 so all signs inverted
        B = np.array([[0, 0, 0, 0], [0, -9.8, 0, 0], [0, 0, 0, 0], [0, 0, 9.8, 0], [0, 0, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

        P = la.solve_continuous_are(A, B, Q, R)

        u = np.dot(-np.linalg.inv(R),B.transpose())
        u = np.dot(u,P)
        u = np.dot(u,X)

        u0 = float(u[0])
        u1 = float(u[1])
        u2 = float(u[2])
        u3 = float(u[3])

        u0 = (u0*1.5 + 14.7)/29.4
        ##15 deg max cutoff at 10
        if u0>1:
            u0 = 1
        if u0<0:
            u0 = 0

        if u1>10*np.pi/180:
            u1 = 10*np.pi/180
        if u1<-10*np.pi/180:
            u1 = -10*np.pi/180

        if u2>10*np.pi/180:
            u2 = 10*np.pi/180
        if u2<-10*np.pi/180:
            u2 = -10*np.pi/180

        u3 = abs(yaw_dot-u3)
        if rot_flag == 1:
            u3 = u3
        if rot_flag == -1:
            u3 = -u3
        rospy.loginfo("MATRIX %s %s", theta, u3)
        '''
        if Q[0][0]>Q[1][1]:
            if u1>10*np.pi/180:
                u1 = 10*np.pi/180
            if u1<-10*np.pi/180:
                u1 = -10*np.pi/180
        else:
            if u1>5*np.pi/180:
                u1 = 5*np.pi/180
            if u1<-5*np.pi/180:
                u1 = -5*np.pi/180

        if Q[2][2]>Q[3][3]:
            if u2>10*np.pi/180:
                u2 = 10*np.pi/180
            if u2<-10*np.pi/180:
                u2 = -10*np.pi/180
        else:
            if u2>5*np.pi/180:
                u2 = 5*np.pi/180
            if u2<-5*np.pi/180:
                u2 = -5*np.pi/180
        '''
        #rospy.loginfo("INFO %s %s", sqrt(goal_body[0]**2+goal_body[1]**2), abs(goal_body[2]))
        if sqrt(goal_body[0]**2+goal_body[1]**2)<0.8 and abs(goal_body[2])<1:
            rospy.loginfo("LAND")
            u0 = 0.0
            u1 = 0.0
            u2 = 0.0


        now = time.time()

        #rospy.loginfo("Inputs %s %s %s", u0, u1, u2)

        quater = tf.transformations.quaternion_from_euler(u2,u1,yaw+np.pi/2) #0
        msg.header = Header()
        msg.type_mask = 0
        msg.orientation.x = quater[0]
        msg.orientation.y = quater[1]
        msg.orientation.z = quater[2]
        msg.orientation.w = quater[3]
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = u3
        msg.thrust = u0

        ##VELOCITIES HERE

        pub.publish(msg)
        now_p = time.time()

        #rospy.loginfo("States %s", X)
        #rospy.loginfo("positions %s", [x,y,z])

        rate = rospy.Rate(100) 
        rate.sleep


def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, vel_drone_rot, vel_drone_trans, now_p, goal, goal_body, v_x, v_y, v_z, Rot_body_to_inertial, Rot_inertial_to_body, head, rot_flag, theta, theta_p
    
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
    if yaw<0:
        yaw = yaw+2*np.pi
    if yaw>2*np.pi:
        yaw = yaw-2*np.pi

    a2, b2, c2 ,d2 = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    a3, b3, c3, d3 = tf.transformations.quaternion_from_euler(0.0, 0.0, head)
    theta = acos((a2*a3+b2*b3+c2*c3+d2*d3)/(sqrt(a2**2+b2**2+c2**2+d2**2)*sqrt(a3**2+b3**2+c3**2+d3**2)))
    if theta<0:
        theta = theta+2*np.pi
    if theta>2*np.pi:
        theta = theta-2*np.pi
    if theta>np.pi:
        theta = 2*np.pi-theta

    #if abs(theta-theta_p)>np.pi/4:
    #    theta = theta_p
    #else:
    #    theta = 0.65*theta+0.35*theta_p

    yaw2 = yaw+2*np.pi/180
    if yaw2<0:
        yaw2 = yaw2+2*np.pi
    if yaw2>2*np.pi:
        yaw2 = yaw2-2*np.pi
    a4, b4, c4 ,d4 = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw2)
    theta2 = acos((a4*a3+b4*b3+c4*c3+d4*d3)/(sqrt(a4**2+b4**2+c4**2+d4**2)*sqrt(a3**2+b3**2+c3**2+d3**2)))
    if theta2<0:
        theta2 = theta2+2*np.pi
    if theta2>2*np.pi:
        theta2 = theta2-2*np.pi
    if theta2>np.pi:
        theta2 = 2*np.pi-theta2

    if theta2<theta:
       rot_flag = 1
    else:
        rot_flag = -1
       

    #temp = roll
    #roll = pitch
    #pitch = -temp

    #rospy.loginfo("ANGLE OUTSIDE %s", [yaw, pitch, roll])

    theta_p = theta

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()


def ReceiveTar(info):
    global goal, vel_rover, Rot_inertial_to_body, detect, head, head_prev, yaw_dot, now, now_p, yaw, head, theta, rot_flag
    goal[0] = info.goal.x
    goal[1] = info.goal.y
    goal[2] = 0.435
    detect = info.detected
    v1 = info.vel.x
    v2 = info.vel.y

    now = timer()
    dt = now-now_p
    head = info.heading
    #if v2<0.1 and v1<0.1:
    #    head = 0.0    
    #else:
    #    a1 = float(v1)/sqrt(float(v1)**2 + float(v2)**2)
    #    a2 = float(v2)/sqrt(float(v1)**2 + float(v2)**2)
    #    head = atan2(a2, a1)
    if head<0:
        head+=2*np.pi
    if head>2*np.pi:
        head-=2*np.pi
    yaw_dot = (head-head_prev)/dt
    v = np.array([[v1]
                ,[v2]
                ,[0.0]])
    v = np.dot(Rot_inertial_to_body, v)
    vel_rover[0] = float(v[0])
    vel_rover[1] = float(v[1])
    vel_rover[2] = float(v[2])

    head_prev = head
    now_p = timer()
                


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