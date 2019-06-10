#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import *
import mavros_msgs.srv
import mavros_msgs.msg
from std_msgs.msg import *
from test.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from quadcopter.msg import *

rospy.init_node('obstacle_avoidance', anonymous=True)
pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

roll = 0.0
pitch = 0.0
yaw = 0.0

msg = Twist()
#[10,-10,0]    #ardu
goal = [10.0,10.0,0.0]

error_x = 0.0
#error_x_sum = 0.0
error_x_diff = 0.0

error_y = 0.0
#error_y_sum = 0.0
error_y_diff = 0.0

error_z = 0.0
#error_z_sum = 0.0
error_z_diff = 0.0

k_px = 0.2
k_dx = 0.05
#k_ix = 0.0
k_py = 0.2
k_dy = 0.05
#k_iy = 0.0
k_pz = 0.2
k_dz = 0.05
#k_iz = 0.0

x = 0.0
y = 0.0
z = 0.0

camera_mount = 0.785398
horizontal = 1.04719/2
vertical = 1.04719/2

vel_rover = [0,0,0]

####    msg.x in mavros is -y in gazebo
def land():
    set_mode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
    print "set mode: ", set_mode(208,'GUIDED')
    land = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
    print "land: ", land(0,0,0,0,0)


def callback(info):
    global k_pz, k_px, k_py, k_dz, k_dx, k_dy, x, y, z, roll, pitch, yaw, error_x_diff, error_y_diff, error_z_diff
    x = info.pose.position.y
    y = -info.pose.position.x
    z = info.pose.position.z
    a = info.pose.orientation.x
    b = info.pose.orientation.y
    c = info.pose.orientation.z
    d = info.pose.orientation.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([a,b,c,d])
    yaw = -(-yaw+1.5707999)   #####    NED --->  ENU Check quaternions 
    
    error_x = goal[0]-x
    error_y = goal[1]-y
    error_z = goal[2]-z

    #vel_x = k_px*error_x + k_dx*(error_x-error_x_diff)
    #vel_y = k_py*error_y + k_dy*(error_y-error_y_diff)
    #vel_z = k_pz*error_z + k_dz*(error_z-error_z_diff)

    vel_x = vel_rover[0] + (k_px/(1+k_dx))*error_x
    vel_y = vel_rover[1] + (k_py/(1+k_dy))*error_y
    vel_z = vel_rover[2] + (k_pz/(1+k_dz))*error_z

    msg.linear.x = -vel_y
    msg.linear.y = vel_x
    msg.linear.z = vel_z
    if (sqrt(error_x**2 + error_y**2))>1 and error_z>-1:
        msg.linear.z = 0

    if sqrt(error_x**2 + error_y**2)<0.5 and error_z>-0.5:
        land()
#
    #req_head = atan2((goal[1]-y),(goal[0]-x))
    #if req_head>np.pi:
    #    req_head-=2*np.pi
    #if req_head<-np.pi:
    #    req_head+=2*np.pi
    #if yaw>np.pi:
    #    yaw-=2*np.pi
    #if yaw<-np.pi:
    #    yaw+=2*np.pi
    
    #error_head = req_head-yaw

    #msg.angular.z = 0.1*error_head

    rospy.loginfo("message %s %s %s %s", error_x, error_y, error_z, sqrt(error_x**2 + error_y**2))

    if msg.linear.x>1:
        msg.linear.x = 1
    if msg.linear.x<-1:
        msg.linear.x = -1
    if msg.linear.z>1:
        msg.linear.z = 1
    if msg.linear.z<-1:
        msg.linear.z = -1
    if msg.linear.y>1:
        msg.linear.y = 1
    if msg.linear.y<-1:
        msg.linear.y = -1

    error_x_diff = error_x
    error_y_diff = error_y
    error_z_diff = error_z

    pub.publish(msg)
    rate = rospy.Rate(10) 


def ReceiveTar(data):
    global goal, x, y, z, roll, pitch, yaw, camera_mount, horizontal, vertical
    xt_image=data.contour.center.x
    yt_image=data.contour.center.y
    xt_image -= 250
    yt_image -= 250
    width=data.contour.width
    height=data.contour.height

    if(width*height<10):
        goal[0] = goal[0] + vel_rover[0]*0.1
        goal[1] = goal[1] + vel_rover[1]*0.1
        #rospy.loginfo("DATA %s %s",xt_image,yt_image)

    else:
        d_xbound = 2*(z/sin(camera_mount))*tan(horizontal)
        x_ppm = d_xbound/500

        d_ybound = z/tan(camera_mount-vertical) - z/tan(camera_mount+vertical)
        y_ppm = d_ybound/500

        x_origin = x + (z/tan(camera_mount))*cos(yaw)       #In global frame
        y_origin = y + (z/tan(camera_mount))*sin(yaw)

        yt_image = -yt_image

        xt_image = xt_image*x_ppm
        yt_image = yt_image*y_ppm

        x_new = x_origin + xt_image*cos(yaw-np.pi/2) - yt_image*sin(yaw-np.pi/2)
        y_new = y_origin + xt_image*sin(yaw-np.pi/2) + yt_image*cos(yaw-np.pi/2)

        #x_new = x - x_diff*cos(yaw)
        #y_new = y - y_diff*sin(yaw)

        goal[0] = x_new
        goal[1] = y_new

        rospy.loginfo("POSN %s %s %s %s ", x_new, y_new, x, y)
        


def listener():
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback)
    rospy.Subscriber('/landing_target_info_new', TargetInfo,ReceiveTar)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass