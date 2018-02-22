# lab6
Green Machine's (Team 12) code for lab6


# To run this code on the simulation
1. cd into ~/racecar-ws/src
2. git clone this repo
3. git clone https://github.com/mit-racecar/TA_example_labs.git
4. mv TA_example_labs/ta_lab5 ~/racecar-ws/src/
5. ```sudo rm -rf TA_example_labs```
6. cd into ~/racecar-ws
7. run [catkin_make]
8. launch gazbo
9. launch rviz
10. run ``roslaunch lab6 full_path_follower.launch```
   * this launches the TA localization code, the map server, the path planner, and the path follower
11. in rviz publish a point and wait for the path to be built (depending on what alg you set in the initilization of path_planner.py)
12. once the path is made and published in rviz the robot should run on it


# To run this code on the robot
1. cd into ~/racecar-ws/src
2. git clone this repo
3. git clone https://github.com/mit-racecar/TA_example_labs.git
4. mv TA_example_labs/ta_lab5 ~/racecar-ws/src/
5. ```sudo rm -rf TA_example_labs```
6. cd into ~/racecar-ws
7. run [catkin_make]
8. launch teleop
9. launch rviz by running ```runcar [carnum] rviz``` from the vm
10. run ``roslaunch lab6 full_path_follower.launch```
   * this launches the TA localization code, the map server, the path planner, and the path follower
   * on the car can be made faster by changing cddt to rmgpu in the localization launch file in the ta_lab5 folder
11. in rviz publish a point and wait for the path to be built (depending on what alg you set in the initilization of path_planner.py)
12. once the path is made and published in rviz the robot should run on it
