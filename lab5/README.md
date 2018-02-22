# lab 5
Green Machine's (Team 12) code for lab5


# To run this code on the simulation
1. cd into ~/racecar-ws/src
2. git clone this repo
3. cd into ~/racecar-ws
4. run [catkin_make]
5. cd into ~/racecar-ws/src/lab5/launch
6. run ```roslaunch racecar_gazebo racecar_tunnel.launch```
7. in another window run ```roslaunch racecar map_server.launch```
8. in another window run ```roslaunch racecar localize.launch```
9. now the robot pose is being published
10. you can visualize the robot in rviz
11. in rviz add by topic the robot pose and the map and the laser scan to see the robot in the tunnel
12. with the controller you can now drive the robot around in gazebo and see it update in rviz


# To run this code on the robot
1. SSH into robot
2. cd into ~/racecar-ws/src
3. git clone this repo
4. cd into ~/racecar-ws
5. run [catkin_make]
6. cd into ~/racecar-ws/src/lab5/launch
7. place the racecar in the tunnel system in the fork next to room 32-079
  * currentlly the robot has this as its start location
  * you can change this by using a clicked point in rviz
8. run ```roslaunch racecar teleop.launch```
9. in another window run ```roslaunch racecar map_server.launch```
10. in another window run ```roslaunch racecar localize.launch```
11. now the robot pose is being published
  * you can visualize this in rviz by running ```runcar [carnum] rviz``` from your local machine that is connected to the robots internet
