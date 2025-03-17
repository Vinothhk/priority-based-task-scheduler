# pirority-based-task-scheduler
An ROS 2 implementation of Priority Based Task Scheduler method for an Autonomous Robot

```
cd <your-workspace-name>/src
```
```
git clone https://github.com/Vinothhk/priority-based-task-scheduler.git
```

### Build the Workspace

```
colcon build --packages-select rtos_pkg
```
```
souce install/setup.bash
```
### Run the Scripts
```
ros2 run rtos_pkg rtos_delivery_scheduler
```
```
ros2 run rtos_pkg moveRobot
```

To assign goal points
```
ros2 run rtos_pkg rtos_node
```