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
from quadcopter.msg import *

rospy.init_node('kalman_filter', anonymous=True)
pub = rospy.Publisher("/kalman_filter", kalman, queue_size=10)

goal = [0.0, 0.0, 0.0]
goal_body = [0.0, 0.0, 0.0]
vel_rover = [0.0, 0.0, 0.0]
roll = 0.0
pitch = 0.0
yaw = 0.0
v_x = 0.0
v_y = 0.0
v_z = 0.0
x = 0.0
y = 0.0
z = 0.0
i = 0
w1_next = 0.0
w2_next = 0.0
v_roll = 0.0
v_pitch = 0.0
v_yaw = 0.0
w1 = 0.0
w2 = 0.0

detect = 1

now_cam_p = rospy.get_time()
now_cam = rospy.get_time()
now_kal_p = rospy.get_time()
now_kal = rospy.get_time()
init = rospy.get_time()

v = np.array([[0.0]
            ,[0.0]
            ,[0.0]])

u1_prev = np.array([[0.0]
                    ,[0.0]
                    ,[0.0]])

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

Rot_body_to_inertial_vel = np.array([[-sin(yaw)*cos(pitch)*v_yaw-cos(yaw)*sin(pitch)*v_pitch, -cos(yaw)*cos(roll)*v_yaw+sin(yaw)*sin(roll)*v_roll +cos(roll)*sin(pitch)*cos(yaw)*v_roll+sin(roll)*cos(pitch)*cos(yaw)*v_pitch-sin(roll)*sin(pitch)*sin(yaw)*v_yaw, cos(yaw)*sin(roll)*v_yaw+sin(yaw)*cos(roll)*v_roll -sin(roll)*cos(yaw)*sin(pitch)*v_roll-cos(roll)*sin(yaw)*sin(pitch)*v_yaw+cos(roll)*cos(yaw)*cos(pitch)*v_pitch]
                                    ,[cos(yaw)*cos(pitch)*v_yaw-sin(yaw)*sin(pitch)*v_pitch, -sin(yaw)*cos(roll)*v_yaw-cos(yaw)*sin(roll)*v_roll +cos(roll)*sin(pitch)*sin(yaw)*v_roll+sin(roll)*cos(pitch)*sin(yaw)*v_pitch+sin(roll)*sin(pitch)*cos(yaw)*v_yaw, -cos(roll)*cos(yaw)*v_roll+sin(roll)*sin(yaw)*v_yaw +cos(yaw)*sin(pitch)*cos(roll)*v_yaw+sin(yaw)*cos(pitch)*cos(roll)*v_pitch-sin(yaw)*sin(pitch)*sin(roll)*v_roll]
                                    ,[-cos(pitch)*v_pitch, -sin(pitch)*sin(roll)*v_pitch+cos(pitch)*cos(roll)*v_roll, -sin(pitch)*cos(roll)*v_pitch-cos(pitch)*sin(roll)*v_roll]])

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
    global goal_body, vel_rover, now_kal, now_kal_p, X, P, v_x, v_y, v_z, goal, x, y, z, goal_pred, goal_pred_var, init, detect
    #rate = rospy.Rate(200) 
    #while not rospy.is_shutdown():
    del_t = 0.01
    if detect == 0:
        #now_kal = rospy.get_time()
        
        Q = np.array([[0, 0, 0, 0]
                    ,[0, 0, 0, 0]
                    ,[0, 0, 0, 0]
                    ,[0, 0, 0, 0]])
        X[0] = float(X[0]) + float(X[1])*del_t
        X[2] = float(X[2]) + float(X[3])*del_t
    else:
        v1 = float(X[1])
        v2 = float(X[3])
        #now_kal = rospy.get_time()
        A = np.array([[1, (del_t), 0, 0]
                    ,[0, 1, 0, 0]
                    ,[0, 0, 1, (del_t)]
                    ,[0, 0, 0, 1]])

        X_new_pred = np.dot(A, X)
        
        P_k = np.dot(A, P)
        P_k = np.dot(P_k, A.transpose())
        Q = np.array([[np.random.normal(0, 1.5), 0, 0, 0]
                    ,[0, np.random.normal(0, 1), 0, 0]
                    ,[0, 0, np.random.normal(0, 1.5), 0]
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

        #now_kal_p = rospy.get_time()
        
        P = std_dev_exp - np.dot(KG, std_dev_exp)
        msg.goal.x = float(X[0])
        msg.goal.y = float(X[2])
        msg.goal.z = 0.315
        msg.vel.x = float(X[1])
        msg.vel.y = float(X[3])
        msg.vel.z = 0.0
        pub.publish(msg)
        #rate.sleep()


def ReceiveTar(data):
    global v, w1, w2, goal_body, vel_rover, now_cam, now_cam_p, X, P, v_x, v_y, v_z, goal, x, y, z, roll, pitch, yaw, goal_pred, Rot_body_to_inertial, Rot_body_to_inertial_vel, goal_pred_var, pitch_p, roll_p, w1_next, w2_next, detect, v_pitch, v_roll, v_yaw, u1_prev
    now_cam = rospy.get_time()
    xt_image = data.center.x
    yt_image = data.center.y
    radius = data.radius
    detect = data.detected
    del_t = now_cam - now_cam_p

    if detect==0:
        rospy.loginfo(detect)
        w1_next = w1 + (float(X[1])-v_x)*del_t
        w2_next = w2 + (float(X[3])-v_y)*del_t
        now_cam_p = rospy.get_time()
        pass
    else:
        key_points_dir_body = np.array([[cos(np.pi/12)*cos(np.pi/6), cos(np.pi/12)*cos(-np.pi/6), cos(5*np.pi/12)*cos(np.pi/6), cos(5*np.pi/12)*cos(-np.pi/6), cos(np.pi/4)]
                                        ,[sin(np.pi/6), sin(-np.pi/6), sin(np.pi/6), sin(-np.pi/6), 0]
                                        ,[-sin(np.pi/12)*cos(np.pi/6), -sin(np.pi/12)*cos(-np.pi/6), -sin(5*np.pi/12)*cos(np.pi/6), -sin(5*np.pi/12)*cos(-np.pi/6), -sin(np.pi/4)]])

        key_points_dir_global = np.dot(Rot_body_to_inertial, key_points_dir_body)

        for i in range(len(key_points_dir_global[0])):
            key_points_dir_global[0][i] = float(key_points_dir_global[0][i])*(0.3185-z)/float(key_points_dir_global[2][i])
            key_points_dir_global[1][i] = float(key_points_dir_global[1][i])*(0.3185-z)/float(key_points_dir_global[2][i])
            key_points_dir_global[2][i] = 0.315

        M1 = np.array([[float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0, 0, 0, 0]
                    ,[0, 0, 0, float(key_points_dir_global[0][0]), float(key_points_dir_global[1][0]), 1, 0, 0, 0]
                    ,[float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0, -650*float(key_points_dir_global[0][1]), -650*float(key_points_dir_global[1][1]), -650*1]
                    ,[0, 0, 0, float(key_points_dir_global[0][1]), float(key_points_dir_global[1][1]), 1, 0, 0, 0]
                    ,[float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, 0, 0, 0, 0, 0, 0]
                    ,[0, 0, 0, float(key_points_dir_global[0][2]), float(key_points_dir_global[1][2]), 1, -650*float(key_points_dir_global[0][2]), -650*float(key_points_dir_global[1][2]), -650*1]
                    ,[float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, 0, 0, 0, -650*float(key_points_dir_global[0][3]), -650*float(key_points_dir_global[1][3]), -650*1]
                    ,[0, 0, 0, float(key_points_dir_global[0][3]), float(key_points_dir_global[1][3]), 1, -650*float(key_points_dir_global[0][3]), -650*float(key_points_dir_global[1][3]), -650*1]
                    ,[float(key_points_dir_global[0][4]), float(key_points_dir_global[1][4]), 1, 0, 0, 0, -325*float(key_points_dir_global[0][4]), -325*float(key_points_dir_global[1][4]), -325*1]])

        M2 = np.array([[xt_image]
                    ,[yt_image]
                    ,[1]])
        
        U, D, V = np.linalg.svd(M1)
        M = np.reshape(V[8], (3,3))
        M = np.linalg.inv(M)

        w1 = float(np.dot(M[0], M2)/np.dot(M[2], M2))
        w2 = float(np.dot(M[1], M2)/np.dot(M[2], M2))

        x1 = w1 + x
        x2 = w2 + y
        x3 = 0.315
        #x_vel = np.array([[x1]
        #            ,[x2]
        #            ,[x3]])

        #v_rot = np.dot(Rot_body_to_inertial_vel, x_vel)

        #rospy.loginfo("V %s", v_rot)

        if del_t == 0:
            v1 = float(X[1])
            v2 = float(X[3])
        else:
            u1 = np.array([[x1-x]
                        ,[x2-y]
                        ,[x3-z]])

            u1 = np.dot(Rot_inertial_to_body, u1)
            v_d = np.array([[v_x]
                            ,[v_y]
                            ,[v_z]])

            v = np.dot(Rot_body_to_inertial_vel, u1) + np.dot(Rot_body_to_inertial, (u1-u1_prev)/del_t) + v_d
            rospy.loginfo("VEL %s", np.dot(Rot_body_to_inertial_vel, u1))
            rospy.loginfo("DELT %s",del_t)
            #
            v1 = 0.0
            v2 = 0.0

            u1_prev = u1
            #v1 = (w1-w1_next)/del_t + float(v[0]) - float(v_rot[0])
            #v2 = (w2-w2_next)/del_t + float(v[1]) - float(v_rot[1])

        #x1 = 0.85*float(X[0]) + 0.15*x1
        #v1 = 0.85*float(X[1]) + 0.15*v1
        #x2 = 0.85*float(X[2]) + 0.15*x2
        #v2 = 0.85*float(X[3]) + 0.15*v2
        #rospy.loginfo("DEL_T IN %s", del_t)
        now_cam_p = rospy.get_time()
        #rospy.loginfo("CAM_DATA %s %s %s", v1, v2, del_t)

        goal_pred = np.array([[x1]
                            ,[v1]
                            ,[x2]
                            ,[v2]])
        w1_next = w1
        w2_next = w2

        goal_pred_var = np.array([[np.random.normal(0, 1*abs(v_pitch*v_roll)), 0, 0, 0]
                                ,[0, np.random.normal(0, 1*abs(v_pitch*v_roll)), 0, 0]
                                ,[0, 0, np.random.normal(0, 1*abs(v_pitch*v_roll)), 0]
                                ,[0, 0, 0, np.random.normal(0, 1*abs(v_pitch*v_roll))]])
        

def callback(info):
    ##MUST GET HEADING
    global v, x, y, z, roll, pitch, yaw, vel_rover, goal, Rot_body_to_inertial, Rot_body_to_inertial_vel, Rot_inertial_to_body, i, pitch_p, roll_p, v_roll, v_pitch, v_yaw
    
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

    ###     All angular velocities are local
    v_roll = info.twist.twist.angular.x
    v_pitch = info.twist.twist.angular.y
    v_yaw = info.twist.twist.angular.z

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
    Rot_body_to_inertial_vel = np.array([[-sin(yaw)*cos(pitch)*v_yaw-cos(yaw)*sin(pitch)*v_pitch, -cos(yaw)*cos(roll)*v_yaw+sin(yaw)*sin(roll)*v_roll +cos(roll)*sin(pitch)*cos(yaw)*v_roll+sin(roll)*cos(pitch)*cos(yaw)*v_pitch-sin(roll)*sin(pitch)*sin(yaw)*v_yaw, cos(yaw)*sin(roll)*v_yaw+sin(yaw)*cos(roll)*v_roll -sin(roll)*cos(yaw)*sin(pitch)*v_roll-cos(roll)*sin(yaw)*sin(pitch)*v_yaw+cos(roll)*cos(yaw)*cos(pitch)*v_pitch]
                                    ,[cos(yaw)*cos(pitch)*v_yaw-sin(yaw)*sin(pitch)*v_pitch, -sin(yaw)*cos(roll)*v_yaw-cos(yaw)*sin(roll)*v_roll +cos(roll)*sin(pitch)*sin(yaw)*v_roll+sin(roll)*cos(pitch)*sin(yaw)*v_pitch+sin(roll)*sin(pitch)*cos(yaw)*v_yaw, -cos(roll)*cos(yaw)*v_roll+sin(roll)*sin(yaw)*v_yaw +cos(yaw)*sin(pitch)*cos(roll)*v_yaw+sin(yaw)*cos(pitch)*cos(roll)*v_pitch-sin(yaw)*sin(pitch)*sin(roll)*v_roll]
                                    ,[-cos(pitch)*v_pitch, -sin(pitch)*sin(roll)*v_pitch+cos(pitch)*cos(roll)*v_roll, -sin(pitch)*cos(roll)*v_pitch-cos(pitch)*sin(roll)*v_roll]])

    v_body = np.array([[v_x]
                    ,[v_y]
                    ,[v_z]])

    v = np.dot(Rot_body_to_inertial, v_body)

                
def listener():
    #rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, callback_new)
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, callback)
    timer=rospy.Timer(rospy.Duration(10/1000.0),kalman)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass