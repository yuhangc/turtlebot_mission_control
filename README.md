# turtlebot_mission_control

Some bare-bones starter code for the AA274 project -- feel free to use all or none of it. Clone into your `~/catkin_ws` (perhaps "Fork" this repo first, using the button in the top right of this page, if you want to use github to develop this code as a group) and run
```bash
roslaunch turtlebot_mission_control mission.launch
```
to run the `supervisor.py`, `navigator.py`, and `controller.py` scripts. To simulate receiving a mission specification, run
```bash
rosrun turtlebot_mission_control mission_publisher.py
```
