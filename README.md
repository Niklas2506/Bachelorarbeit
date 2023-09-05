Bachelorarbeit Niklas Lindemann
----------------------------------

Zum starten der Simulation in einer Konsole folgendes ausführen:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=~/AuNa/src/car_simulator/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
source /opt/ros/galactic/setup.bash
source ~/AuNa/install/setup.bash
ros2 launch auna_scenarios scenario_single_robot_racetrack_obstacles.launch.py
```

In einer weiteren den costmap converter starten:

```
source /opt/ros/galactic/setup.bash
source ~/AuNa/install/setup.bash
ros2 launch auna_costmap scenario_costmap_new.launch.py
```
