# vision_turtlebot_HW_KihongPark
[RO:BIT] VISION with Turtlebot3 HW repo

## Clone

```bash
cd ~/{Your WorkSpace}/
gh repo clone kinesis19/vision_turtlebot_HW_KihongPark
```

## Build

```bash
cd ~/{Your WorkSpace}/
colcon build
source ~/{Your WorkSpace}/install.setup
```

## Run
### Run Package
```bash
ros2 run ros2 run qt_vision_turtlebot3_tracing qt_vision_turtlebot3_tracing
```

### Run Gazebo
```bash
ros2 launch turtlebot3_gazebo internship_vision_turtlebot3.launch.py
```