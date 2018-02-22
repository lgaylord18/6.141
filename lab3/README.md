# Lab-3
Green Machine's (Team 12) code for lab3

# To run this code on the robot
1. SSH into robot
2. cd into ~/racecar-ws/src
3. git clone this repo
4. cd into ~/racecar-ws
5. run [catkin_make]
6. cd into ~/racecar-ws/src/lab3/launch
7. run ```roslaunch racecar teleop.launch```
  * This turns on the mux.
8. Then run ```roslaunch wall-follow.launch```
  * This launches the wall-scan.py wall-follow.py and safety.py nodes which cause the gazebo simulated robot to follow the wall and stop if their is an obsticle.
 


# To run this code on the VM (gazebo)
1. cd into ~/racecar-ws/src
2. git clone this repo
3. cd into ~/racecar-ws
4. run [catkin_make]
5. cd into ~/racecar-ws/src/lab3/launch
6. run ```roslaunch racecar teleop.launch```
  * This turns on the mux.
7. Then run ```roslaunch wall-follow.launch```
  * This launches the wall-scan.py wall-follow.py and safety.py nodes which cause the gazebo simulated robot to follow the wall and stop if their is an obsticle.
