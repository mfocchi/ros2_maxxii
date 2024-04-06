# ros2-maxxii


## Setup

In a ros2 compatible device create a workspace or use an existing workspace.
Clone the directory with 
```
https://github.com/idra-lab/ros2-maxxii.git
``` 
and [ros2 serial package](https://github.com/RoverRobotics-forks/serial-ros2).
Next connect the roboteq driver via ch340 to the ros2 device. Note down the usb port (commonly `/dev/ttyUSB0` or `/dev/ttyACM0`), and modify accordingly the parameter `port`. Finally, compile the workspace and source it with 
```
colcon build
source install/setup.bash
```

## Use

To run the node use

```
ros2 run ros2_maxxii maxxii_node
```

It is going to use the parameters specified in the node implementation.
Otherwise, launch the node with the parameters in `config/roboteq_driver.yaml` with

```
ros2 launch ros2_maxxii maxxii.launch.py
```

## Topics

- The node **Publishes** to `/enc` Joint State message containing the information from the encoders (angle in radiants).
- The node **Subscribes** to `/cmd` Joint State message. The message contains the desired velocitiy setpoint for the motors (radiants per second).