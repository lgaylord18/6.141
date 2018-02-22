# Lab 4
Green Machine's (Team 12) code for lab4

# To run this code on the robot
1. SSH into robot
2. cd into ~/racecar-ws/src
3. git clone this repo
4. cd into ~/racecar-ws
5. run [catkin_make]
6. cd into ~/racecar-ws/src/lab4/launch
7. run ```roslaunch racecar teleop.launch```
  * This turns on the mux.

For cone following: <br />
&nbsp;&nbsp;&nbsp;With PD Control: <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;run ```roslaunch cone_follower_proportional.launch``` <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;* This launches the cone-finder.py cone_follow_proportional.py and cone_park.py nodes <br />
&nbsp;&nbsp;&nbsp;With Pure Pursuit: <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;run ```roslaunch cone_follower.launch``` <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;* This launches the cone-finder.py cone_follow.py and cone_park.py nodes <br />

For line following: <br />
&nbsp;&nbsp;&nbsp;With Open Loop Control: <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;run ```roslaunch openloop_follower.launch``` <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;* This launches the openloop_circle.py node <br />
&nbsp;&nbsp;&nbsp;With PD Control: <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;run ```roslaunch line_follower_low.launch``` <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;* This launches the line_finder_low.py and line_follow_low.py nodes <br />
&nbsp;&nbsp;&nbsp;With Hybrid (PD + Pure Pursuit) Control: <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;run ```roslaunch line_follower_low.launch``` <br />
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;* This launches the line_finder.py and line_follow.py nodes <br />
