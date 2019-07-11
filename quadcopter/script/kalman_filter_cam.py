#!/usr/bin/env python
import rospy
import tf
import scipy.linalg as la
import scipy.signal as sig
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
from quadcopter.msg import *
import timeit

rospy.init_node('kalman_filter', anonymous=True)
pub = rospy.Publisher("/kalman_filter", kalman, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
x = 0.0
y = 0.0
z = 0.0
u1_prev = 0.0
u2_prev = 0.0
u3_prev = 0.0
n_prev = 0.0
x1_prev = 0.0
x2_prev = 0.0
v_roll = 0.0
v_pitch = 0.0
v_yaw = 0.0
v1_prev = 0.0
v2_prev = 0.0
i = 0
v1 = 0.0
v2 = 0.0
fil1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fil2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#fil3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#fil4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

detect = 1

now_cam_p = timer()
now_cam = timer()
now_kal_p = timer()
now_kal = timer()
init = timer()

hori_fov = np.pi/6 #on either side
vert_fov = 2000*hori_fov/2000

H = np.array([[1, 0, 0, 0]
            ,[0, 1, 0, 0]
            ,[0, 0, 1, 0]
            ,[0, 0, 0, 1]])

goal_pred = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]
                    ,[0.0]])

goal_pred_var = np.array([[1, 0, 0, 0]
                        ,[0, 1, 0, 0]
                        ,[0, 0, 1, 0]
                        ,[0, 0, 0, 1]])

Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
Rot_inertial_to_body = Rot_body_to_inertial.transpose()


X = np.array([[0.0]
            ,[0.0]
            ,[0.0]
            ,[0.0]])

P = np.array([[np.random.normal(0, 1), 0, 0, 0]
            ,[0, np.random.normal(0, 0.25), 0, 0]
            ,[0, 0, np.random.normal(0, 1), 0]
            ,[0, 0, 0, np.random.normal(0, 0.25)]])

msg = kalman()

def kalman(timer):
    global now_kal, now_kal_p, X, P, v_x, v_y, v_z, x, y, z, goal_pred, goal_pred_var, init, detect
    #rate = rospy.Rate(200) 
    #while not rospy.is_shutdown():
    del_t = 0.01
    if detect == 0:
        #now_kal = timer()
        
        Q = np.array([[0, 0, 0, 0]
                    ,[0, 0, 0, 0]
                    ,[0, 0, 0, 0]
                    ,[0, 0, 0, 0]])
        X[0] = float(X[0]) + float(X[1])*del_t
        X[2] = float(X[2]) + float(X[3])*del_t
    else:
        v1 = float(X[1])
        v2 = float(X[3])
        #now_kal = timer()
        A = np.array([[1, (del_t), 0, 0]
                    ,[0, 1, 0, 0]
                    ,[0, 0, 1, (del_t)]
                    ,[0, 0, 0, 1]])

        X_new_pred = np.dot(A, X)
        
        P_k = np.dot(A, P)
        P_k = np.dot(P_k, A.transpose())
        Q = np.array([[np.random.normal(0, 4), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 4), 0]
                    ,[0, 0, 0, np.random.normal(0, 1)]])
            
        P_k = P_k + Q
        
        mu_exp = np.dot(H, X_new_pred)
        std_dev_exp = np.dot(H.transpose(), P_k)
        std_dev_exp = np.dot(std_dev_exp, H)

        KG = np.dot(np.dot(std_dev_exp, H.transpose()), np.linalg.inv(std_dev_exp + goal_pred_var))

        X_new = X_new_pred + np.dot(KG, (np.dot(H,goal_pred) - np.dot(H,mu_exp)))
        
        X = X_new
        #X[0] = float(X_new[0])
        #X[1] = 0.65*float(X[1])*(now_kal-init)/(now_kal+0.001) + 0.35*v1*(1-(now_kal-init)/(now_kal+0.001))
        #X[2] = float(X_new[2])
        #X[3] = 0.65*float(X[3])*(now_kal-init)/(now_kal+0.001) + 0.35*v2*(1-(now_kal-init)/(now_kal+0.001))

        #rospy.loginfo("X %s", X)
        #rospy.loginfo("DEL_T OUT %s",del_t)

        #now_kal_p = timer()
        
        P = std_dev_exp - np.dot(KG, std_dev_exp)
    msg.goal.x = float(X[0])
    msg.goal.y = float(X[2])
    msg.goal.z = 0.31
    msg.vel.x = float(X[1])
    msg.vel.y = float(X[3])
    msg.vel.z = 0.0
    msg.posn.x = x
    msg.posn.y = y
    msg.posn.z = z
    pub.publish(msg)
        #rate.sleep()


def ReceiveTar(data):
    global i, now_cam, now_cam_p, v_x, v_y, v_z, v_roll, v_pitch, v_yaw, x, y, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, goal_pred_var, detect, v1, v2, x1_prev, x2_prev
    
    R = Rot_body_to_inertial
    vx = v_x
    vy = v_y
    xn = x
    yn = y
    zn = z
    xt_image = data.center.x
    yt_image = data.center.y
    radius = data.radius
    detect = data.detected
    now_cam = data.time

    if detect==0:
        rospy.loginfo(detect)
        #u1_prev = x1 + (float(X[1])-v_x)*del_t
        #u2_prev = x2 + (float(X[3])-v_y)*del_t
        #now_cam_p = timer()
        pass
    else:
        #rospy.loginfo("FIL %s", fil1)
        del_t = now_cam-now_cam_p
        if del_t == 0:
            pass
        else:
            x1, x2 = get_position(xt_image, yt_image, xn, yn, R)    

            x1 = 0.65*x1 + 0.35*x1_prev  
            x2 = 0.65*x2 + 0.35*x2_prev   
            x1_prev = x1
            x2_prev = x2
            #w1 = x1-x
            #w2 = x2-y
            #v1, v2 = get_velocity(xt_image, yt_image, x1, x2, del_t, R)

            #v1 = x_r + vx
            #v2 = y_r + vy

            goal_pred = np.array([[x1]
                                ,[v1]
                                ,[x2]
                                ,[v2]])

            img = np.array([[x1-xn]
                        ,[x2-yn]
                        ,[0.3185]])
            img = np.dot(R.transpose(), img)
            #rospy.loginfo("GOAL VEL %s %s %s %s", v1, v2, vx, vy)
            #rospy.loginfo("GOAL POS %s %s", x1,x2)
            #goal_pred_var = np.array([[0, 0, 0, 0]
            #                        ,[0, 0, 0, 0]
            #                        ,[0, 0, 0, 0]
            #                        ,[0, 0, 0, 0]])
            #1.1**abs(float(goal_pred[0])/(z+0.00001))+abs(v_pitch*v_roll))
            goal_pred_var = np.array([[np.random.normal(0, 0.3*1.1**(float(img[0])*0.25/(z+0.0001))), 0, 0, 0]
                                    ,[0, np.random.normal(0, 1), 0, 0]
                                    ,[0, 0, np.random.normal(0, 0.3*1.1**(float(img[1])*0.25/(z+0.0001))), 0]
                                    ,[0, 0, 0, np.random.normal(0, 1)]])   

            now_cam_p = data.time
            i+=1
        
def get_position(xt, yt, xn, yn, R):
    key_points_dir_body = np.array([[cos(np.pi/4-vert_fov)*cos(hori_fov), cos(np.pi/4-vert_fov)*cos(-hori_fov), cos(np.pi/4+vert_fov)*cos(hori_fov), cos(np.pi/4+vert_fov)*cos(-hori_fov), cos(np.pi/4)]
                                    ,[sin(hori_fov), sin(-hori_fov), sin(hori_fov), sin(-hori_fov), 0]
                                    ,[-sin(np.pi/4-vert_fov)*cos(hori_fov), -sin(np.pi/4-vert_fov)*cos(-hori_fov), -sin(np.pi/4+vert_fov)*cos(hori_fov), -sin(np.pi/4+vert_fov)*cos(-hori_fov), -sin(np.pi/4)]])
    key_points_dir_global = np.dot(R, key_points_dir_body)

    for i in range(len(key_points_dir_global[0])):
        key_points_dir_global[0][i] = float(key_points_dir_global[0][i])*(0.3185-z)/float(key_points_dir_global[2][i]) + xn
        key_points_dir_global[1][i] = float(key_points_dir_global[1][i])*(0.3185-z)/float(key_points_dir_global[2][i]) + yn
        key_points_dir_global[2][i] = 0.3185

    M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -2000*float(key_points_dir_global[0][1]), -2000*float(key_points_dir_global[1][1]), -2000*1]
                ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -2000*float(key_points_dir_global[0][2]), -2000*float(key_points_dir_global[1][2]), -2000*1]
                ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -2000*float(key_points_dir_global[0][3]), -2000*float(key_points_dir_global[1][3]), -2000*1]
                ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -2000*float(key_points_dir_global[0][3]), -2000*float(key_points_dir_global[1][3]), -2000*1]
                ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -1000*float(key_points_dir_global[0][4]), -1000*float(key_points_dir_global[1][4]), -1000*1]])

    M2 = np.array([[xt]
                ,[yt]
                ,[1]])
        
    U, D, V = np.linalg.svd(M1)
    M = np.reshape(V[len(V)-1], (3,3))
    M = np.linalg.inv(M)

    w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
    w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

    return w1, w2

def get_velocity(event):
    global u1_prev, u2_prev, u3_prev, v1, v2, v1_prev, v2_prev

    dt = 0.5

    w1 = float(goal_pred[0])
    w2 = float(goal_pred[2])
    v1 = (w1-u1_prev)/dt
    v2 = (w2-u2_prev)/dt

    v1 = 0.6*v1+0.4*v1_prev
    v2 = 0.6*v2+0.4*v2_prev

    v1_prev = v1
    v2_prev = v2    

    rospy.loginfo("INFO %s %s %s", v1, w1, u1_prev)

    u1_prev = w1
    u2_prev = w2


def callback(info):
    global x, y, z, roll, pitch, yaw, Rot_body_to_inertial, Rot_inertial_to_body, v_roll, v_pitch, v_yaw, v_x, v_y, v_z
    
    ############################        GAZEBO COORDINATE FRAME
    ###     Positions in global gazebo frame
    x = info.pose.pose.position.y
    y = -info.pose.pose.position.x
    z = info.pose.pose.position.z

    ###     All linear velocities are local 
    va = info.twist.twist.linear.x
    vb = info.twist.twist.linear.y
    vc = info.twist.twist.linear.z
    
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

    Rot_body_to_inertial = np.array([[cos(yaw)*cos(pitch), -sin(yaw)*cos(roll)+sin(roll)*sin(pitch)*cos(yaw), sin(yaw)*sin(roll)+cos(roll)*cos(yaw)*sin(pitch)]
                                    ,[sin(yaw)*cos(pitch), cos(yaw)*cos(roll)+sin(roll)*sin(pitch)*sin(yaw), -sin(roll)*cos(yaw)+sin(yaw)*sin(pitch)*cos(roll)]
                                    ,[-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)]])
    Rot_inertial_to_body = Rot_body_to_inertial.transpose()
    
    ###     All angular velocities are local
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

    ###     Convert Velocities to the global frame

    v2 = np.array([[v_roll]
                ,[v_pitch]
                ,[v_yaw]])

    v1 = np.array([[va]
                ,[vb]
                ,[vc]])

    v2 = np.dot(Rot_body_to_inertial, v2)
    v1 = np.dot(Rot_body_to_inertial, v1)

    v_roll = float(v2[0])
    v_pitch = float(v2[1])
    v_yaw = float(v2[2])

    v_x = float(v1[0])
    v_y = float(v1[1])
    v_z = float(v1[2])
                
def listener():
    #rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback_new)
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    timer=rospy.Timer(rospy.Duration(10/1000.0),kalman)
    timer2=rospy.Timer(rospy.Duration(500/1000.0),get_velocity)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass