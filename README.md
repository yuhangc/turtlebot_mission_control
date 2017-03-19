# turtlebot_mission_control

Code for the AA274 project -- autonomous navigation in a maze. The robot first explores the maze and builds a map, then autonomously navigates to goal positions published to `/mission` topic. Use
```bash
roslaunch turtlebot_mission_control mission.launch
```
to run the `supervisor.py`, `navigator.py`, and `controller.py` scripts. To simulate receiving a mission specification, run
```bash
rosrun turtlebot_mission_control mission_publisher.py
```
