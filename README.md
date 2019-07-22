# Co-operative-Landing
IIITD Summer Intern 2019


quadcopter folder for codes in the workspace
iris_standoffs_demo : place this in you gazebo models path. Changed sdf for adding camera and publishing images as ros topic

husky : place this in you gazebo models path. Changed sdf to prevent slips. Added target plate

two_bot.launch : place this in your mavros folder along with apm and apm2.launch. Used to launch a udp communication with both rover and drone

apm.launch : altered to include namespace so drone and rover messages published seperately

iris_ardupilot.world : place in world file in ardupilot_gazebo


Codes:
* roscore
* rosrun gazebo_ros gazebo iris_ardupilot.world
* sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 --console -I0
* sim_vehicle.py -v APMrover2 -f gazebo-rover -m --mav10 --console -I1
* roslaunch mavros two_bot.launch
* cd catkin_ws/
* source devel/setup.bash
* rosrun quadcopter blob.py
* rosrun quadcopter kalman_filter_cam.py
* rosrun quadcopter sdre_gazebo.py

* rosrun mavros mavsys -n drone/mavros rate --all 50

### Dont run used for plotting data
* rosrun quadcopter graph.py
* rqt_plot /rover/pose/pose/position/x
