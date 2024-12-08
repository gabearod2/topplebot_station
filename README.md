# ToppleBot ROS2 Control Station

If you haven't already, set up your ToppleBot, and connect it to the power source. Reference this [repo](https://github.com/gabearod2/topplebot.git) for set-up.

Clone this repository in your workspaces directory:
```
https://github.com/gabearod2/topplebot_station.git &&
cd topplebot_station
```

On your control station, run the micro-ROS agent:

```
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

You should see the messages coming from the ToppleBot. The following commands will run the visualizer:

```
source /opt/ros/humble/setup.bash &&
colcon build --packages-select viz --siymlink-install &&
source install/seup.bash &&
ros2 launch viz topplebot_rviz.launch.py
```

Watch the dynamics display!
