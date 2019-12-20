# Co-operative-Landing
IIITD Summer Intern 2019

Our research has been submitted to ECC (European Control Conference).


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


In this project we use State Dependent Ricatti Equation (SDRE) based control to land a quadrotor on a moving target. We assume that the rover does not have GPS data, and no means of communication to the drone. We use an onboard camera to detect ande estimate the position and velocity of the drone with a Kalman filter, and take appropriate control

A video showing the quadrotor landing on a rover moving at 3m/s in a linear manner:
[Video for Landing on 3m/s target](https://drive.google.com/open?id=16CE4sSi6cS4Grk4E-FsCJQq4ePsW1oYf "Click to play")

A video showing the quadrotor landing on a rover in a curved path with varying velocities:
[Video for Landing target moving in a curved path](https://drive.google.com/open?id=1Wiolx_fjBPQhqBEMf5C5C_19rhqYbeJZ "Click to play")
