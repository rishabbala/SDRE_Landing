#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import numpy as np
from math import *
import mavros_msgs.srv
from mavros_msgs.msg import *
from nav_msgs.msg import *
from std_msgs.msg import *
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *
import time
import control.matlab as mb
from timeit import default_timer as timer
import utm
from sensor_msgs.msg import *

rospy.init_node('kalman_filter', anonymous=True)
#pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

rover_utm = [0.0, 0.0]
drone_utm = [0.0, 0.0]
goal = [0.0, 0.0, 0.0]
goal_body = [0.0, 0.0, 0.0]
vel_rover = [0.0, 0.0, 0.0]
accn_rover = [0.0, 0.0, 0.0]
roll = 0.0
pitch = 0.0
yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
roll_p = 0.0
pitch_p = 0.0
x = 0.0
y = 0.0
z = 0.0
i = 0

now_p = rospy.get_time()
now = rospy.get_time()
now_p_cam = rospy.get_time()
now_cam = rospy.get_time()
now_gps = rospy.get_time()
now_p_gps = rospy.get_time()

H = np.array([[1, 0, 0, 0, 0, 0]
            ,[0, 1, 0, 0, 0, 0]
            ,[0, 0, 1, 0, 0, 0]
            ,[0, 0, 0, 1, 0, 0]
            ,[0, 0, 0, 0, 1, 0]
            ,[0, 0, 0, 0, 0, 1]])

cam_goal = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])

gps_goal = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])

cam_goal_noise = np.array([[np.random.normal(0, 1), 0, 0, 0, 0, 0]
                        ,[0, np.random.normal(0, 0.25), 0, 0, 0, 0]
                        ,[0, 0, np.random.normal(0, 0.1), 0, 0, 0]
                        ,[0, 0, 0, np.random.normal(0, 1), 0, 0]
                        ,[0, 0, 0, 0, np.random.normal(0, 0.25), 0]
                        ,[0, 0, 0, 0, 0, np.random.normal(0, 0.1)]])

gps_noise = np.array([[np.random.normal(0, 1), 0, 0, 0, 0, 0]
                    ,[0, np.random.normal(0, 0.25), 0, 0, 0, 0]
                    ,[0, 0, np.random.normal(0, 0.1), 0, 0, 0]
                    ,[0, 0, 0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, 0, 0, np.random.normal(0, 0.25), 0]
                    ,[0, 0, 0, 0, 0, np.random.normal(0, 0.1)]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()

X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])

P = np.array([[np.random.normal(0, 1), 0, 0, 0, 0, 0]
            ,[0, np.random.normal(0, 0.25), 0, 0, 0, 0]
            ,[0, 0, np.random.normal(0, 0.1), 0, 0, 0]
            ,[0, 0, 0, np.random.normal(0, 1), 0, 0]
            ,[0, 0, 0, 0, np.random.normal(0, 0.25), 0]
            ,[0, 0, 0, 0, 0, np.random.normal(0, 0.1)]])


def kalman()
    global goal_body, vel_rover, now, now_p, X, P, v_x, v_y, v_z, goal, x, y, z, roll, pitch, yaw, cam_goal, Rot_body_to_inertial, cam_goal_noise, pitch_p, roll_p, gps_goal

    while not rospy.is_shutdown():
        X = np.array([[goal[0]]
                    ,[vel_rover[0]]
                    ,[accn_rover[0]]
                    ,[goal[1]]
                    ,[vel_rover[1]]
                    ,[accn_rover[1]]])

        now_kal = rospy.get_time()
        del_t = now_kal - now_p_kal

        A = np.array([[1, (del_t), 0.5*(del_t)**2, 0, 0, 0]
                    ,[0, 1, (del_t), 0, 0, 0]
                    ,[0, 0, 1, 0, 0, 0]
                    ,[0, 0, 0, 1, (del_t), 0.5*(del_t)**2]
                    ,[0, 0, 0, 0, 1, (del_t)]
                    ,[0, 0, 0, 0, 0, 1]])

        Q = np.array([[np.random.normal(0, 1*del_t), 0, 0, 0, 0, 0]
                    ,[0, np.random.normal(0, 2*del_t), 0, 0, 0, 0]
                    ,[0, 0, np.random.normal(0, 4*del_t), 0, 0, 0]
                    ,[0, 0, 0, np.random.normal(0, 1*del_t), 0, 0]
                    ,[0, 0, 0, 0, np.random.normal(0, 2*del_t), 0]
                    ,[0, 0, 0, 0, 0, np.random.normal(0, 4*del_t)]])

        X_new_pred = np.dot(A, X)
        
        P_k = np.dot(A, P)
        P_k = np.dot(P_k, A.transpose())
        
        #rospy.loginfo("X %s", P)
        P_k = P_k + Q

        gps()
        gps_cam_filter()
        
        mu_exp = np.dot(H, X_new_pred)
        std_dev_exp = np.dot(H.transpose(), P_k)
        std_dev_exp = np.dot(std_dev_exp, H)
        KG = np.dot(std_dev_exp, np.linalg.inv(std_dev_exp + cam_goal_noise))
        
        X_new = np.dot(KG, (cam_goal - mu_exp)) + X_new_pred
        
        X = X_new
        rospy.loginfo("PRED %s",X)
        
        P = std_dev_exp - np.dot(KG, std_dev_exp)
        now_p = rospy.get_time()
        rate = rospy.Rate(10) 
        rate.sleep


def ReceiveTar(data):
    global goal_body, vel_rover, now_cam, now_p_cam, v_x, v_y, v_z, goal, x, y, z, roll, pitch, yaw, cam_goal, Rot_body_to_inertial, cam_goal_noise, pitch_p, roll_p

    xt_image=data.contour.center.x
    yt_image=data.contour.center.y
    width=data.contour.width
    height=data.contour.height
    detect = data.detected

    if detect==0:
        now_cam = rospy.get_time()
        cam_goal = cam_goal
        cam_goal_noise = cam_goal_noise
    else:
        key_points_dir_body = np.array([[cos(np.pi/12)*cos(np.pi/6), cos(np.pi/12)*cos(-np.pi/6), cos(5*np.pi/12)*cos(np.pi/6), cos(5*np.pi/12)*cos(-np.pi/6), cos(np.pi/4)]
                                        ,[sin(np.pi/6), sin(-np.pi/6), sin(np.pi/6), sin(-np.pi/6), 0]
                                        ,[-sin(np.pi/12)*cos(np.pi/6), -sin(np.pi/12)*cos(-np.pi/6), -sin(5*np.pi/12)*cos(np.pi/6), -sin(5*np.pi/12)*cos(-np.pi/6), -sin(np.pi/4)]])

        key_points_dir_global = np.dot(Rot_body_to_inertial, key_points_dir_body)

        #rospy.loginfo("KEYPOINTS_GLOBAL %s", key_points_dir_global)

        for i in range(len(key_points_dir_global[0])):
            key_points_dir_global[0][i] = -float(key_points_dir_global[0][i])*z/float(key_points_dir_global[2][i]) + x
            key_points_dir_global[1][i] = -float(key_points_dir_global[1][i])*z/float(key_points_dir_global[2][i]) + y
            key_points_dir_global[2][i] = 0

        M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                    ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                    ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -500*float(key_points_dir_global[0][1]), -500*float(key_points_dir_global[1][1]), -500*1]
                    ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                    ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                    ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -500*float(key_points_dir_global[0][2]), -500*float(key_points_dir_global[1][2]), -500*1]
                    ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -500*float(key_points_dir_global[0][3]), -500*float(key_points_dir_global[1][3]), -500*1]
                    ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -500*float(key_points_dir_global[0][3]), -500*float(key_points_dir_global[1][3]), -500*1]
                    ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -250*float(key_points_dir_global[0][4]), -250*float(key_points_dir_global[1][4]), -250*1]])

        #rospy.loginfo("DELT %s", del_t)

        M2 = np.array([[xt_image]
                    ,[yt_image]
                    ,[1]])
        
        U, D, V = np.linalg.svd(M1)
        M = np.reshape(V[8], (3,3))
        M = np.linalg.inv(M)

        x1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
        x2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

        now_cam = rospy.get_time()
        del_t = now_cam - now_p_cam

        v1 = (x1-float(cam_goal[0]))/del_t
        a1 = (v1-float(cam_goal[1]))/del_t
        v2 = (x2-float(cam_goal[3]))/del_t
        a1 = (v1-float(cam_goal[4]))/del_t

        rospy.loginfo("X1x2 %s %s",x1,x2)

        cam_goal = np.array([[x1]
                            ,[v1]
                            ,[a1]
                            ,[x2]
                            ,[v2]
                            ,[a2]])

        cam_goal_noise = np.array([[np.random.normal(0, 1*abs(pitch-pitch_p)*del_t), 0, 0, 0, 0, 0]
                                ,[0, np.random.normal(0, 2*del_t), 0, 0, 0, 0]
                                ,[0, 0, np.random.normal(0, 4*del_t), 0, 0, 0]
                                ,[0, 0, 0, np.random.normal(0, 1*abs(roll-roll_p)*del_t), 0, 0]
                                ,[0, 0, 0, 0, np.random.normal(0, 2*del_t), 0]
                                ,[0, 0, 0, 0, 0, np.random.normal(0, 4*del_t)]])
    
    now_p_cam = rospy.get_time()
    

def gps_cam_filter():
    global cam_goal, cam_goal_noise, gps_goal, gps_noise

    KG = np.dot(std_dev_exp, np.linalg.inv(std_dev_exp + cam_goal_noise))
    
    X_new = np.dot(KG, (cam_goal - mu_exp)) + X_new_pred
    
    X = X_new
    rospy.loginfo("PRED %s",X)
    
    P = std_dev_exp - np.dot(KG, std_dev_exp)


    mean_new = 
    


def callback(info):
    ##MUST GET HEADING
    global x, y, z, roll, pitch, yaw, vel_rover, goal, Rot_body_to_inertial, Rot_inertial_to_body, i, pitch_p, roll_p
    
    ############################        GAZEBO COORDINATE FRAME
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

    if i != 0:
        pitch_p = pitch
        roll_p = roll

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

    rospy.loginfo("ANGLE OUTSIDE %s", [yaw, pitch, roll])

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
    i+=1

def rover(info):
    global rover_utm
    rover_utm[0] = info.geo.latitude
    rover_utm[1] = info.geo.longitude
    rover_utm = list(rover_utm)
    rover_utm = utm.from_latlon(rover_utm[0],rover_utm[1])

def drone(info):
    global drone_utm
    drone_utm[0] = info.geo.latitude
    drone_utm[1] = info.geo.longitude
    drone_utm = list(drone_utm)
    drone_utm = utm.from_latlon(drone_utm[0],drone_utm[1])

def gps():
    global drone_utm, rover_utm, gps_goal, gps_noise, now_gps, now_p_gps
    now_gps = rospy.get_time()
    del_t = now_gps-now_p_gps
    t1 = rover_utm[1] - drone_utm[1]
    t2 = rover_utm[0] - drone_utm[0]
    v1 = (t1-gps_goal[0])/del_t
    v2 = (t2-gps_goal[3])/del_t
    a1 = (v1-gps_goal[1])/del_t
    a2 = (v2-gps_goal[4])/del_t
    gps_goal = np.array([[t1]
                        ,[v1]
                        ,[a1]
                        ,[t2]
                        ,[v2]
                        ,[a2]])

    gps_noise = np.array([[np.random.normal(0, 1*del_t), 0, 0, 0, 0, 0]
                ,[0, np.random.normal(0, 2*del_t), 0, 0, 0, 0]
                ,[0, 0, np.random.normal(0, 4*del_t), 0, 0, 0]
                ,[0, 0, 0, np.random.normal(0, 1*del_t), 0, 0]
                ,[0, 0, 0, 0, np.random.normal(0, 2*del_t), 0]
                ,[0, 0, 0, 0, 0, np.random.normal(0, 4*del_t)]])

    now_p_gps = rospy.get_time()
                
def listener():
    #rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback_new)
    rospy.Subscriber("/rover/mavros/global_position/raw/fix", NavSatFix, rover)
    rospy.Subscriber("/drone/mavros/global_position/raw/fix", NavSatFix, drone)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    rospy.Subscriber('/landing_target_info_new', TargetInfo, ReceiveTar)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass