#!/usr/bin/env python
import rospy
import tf
import numpy as np
import sys

rospy.init_node('e2q', anonymous=True)


if __name__ == '__main__':
	####	Enter the angles in deg
	roll = int(sys.argv[1])
	pitch = int(sys.argv[2])
	yaw = int(sys.argv[3])

	roll = roll*np.pi/180
	pitch = pitch*np.pi/180
	yaw = yaw*np.pi/180

	quater = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

	rospy.loginfo("QUATERNIONS %s", quater)
