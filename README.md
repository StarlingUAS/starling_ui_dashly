# Starling Control Panel (ROS2 + Dashly)

This project serves as a mission control and configuration dashboard for the [Starling Project](https://github.com/UoBFlightLab/ProjectStarling) to replace the existing quick dashboard written in pure javascript.

The purpose of this dashboard is to provide a clearer, friendlier and more extensible dashboard that can be used within the flight arena at the Bristol Robotics Laboratory for conducting single and multi-drone experiments and operations.

This dashboard is written in Python and interfaces simultaneously with [Dash](https://dash.plotly.com/) and ROS2 Foxy. This also serves as an example project for combining Dash and ROS2 into a single project (following on from [flask_rclpy](https://github.com/codebot/flask_rclpy)).


## Setup and Installation

### Plain Installation

After install ROS2 Foxy, install the dash dependencies through pip

```
pip3 install dash dash-bootstrap-components gunicorn pandas
```

Once installed, clone this repository into the `src` folder of your `ros_ws` and build using colcon. (Make sure you have sourced ros2)

```
cd ros_ws/src
git clone https://github.com/mhl787156/starling_ui_dashly.git
cd ../ # Back into ros_ws
colcon build --packages-select starling_ui_dashly
```

Once build, the main node can be launched as follows

```
ros2 launch starling_ui_dashly dashboard_gunicorn.launxch.xml
```

which will run a user facing server on https://0.0.0.0:3000

### Docker

The project can also be run using a dockerfile, either build the local file after cloning as follows

```
docker build -t starling_ui_dashly:latest .
docker run --it -rm --network=host starling_ui_dashly:latest
```

Or use the built version on docker hub
```
docker run --it -rm --network=host mickeyli789/starling_ui_dashly:latest
```

## Usage

### Mission Control Screen (`/`)
![control panel](control_panel.png)

#### Mission Control

* Start Mission: Pressing the button will send a single `std_msgs/String` on the `/mission_start` topic.
* Cancel Mission: Pressing the button will send a single `std_msgs/String` on the `/mission_abort` topic. This topic is intended as a soft, controlled stop of the mission (e.g. hover and land, or hover, return to home then land) as determined by the connected controller

The system status will query for currently broadcasting vehicles (`/vehicle_xxx/...`)

#### Emergency Stop

The estop functions like a hardware estop. When engaged it will continuously send `std_msgs/String` on the `/emergency_stop` topic at 10hz. Pressing the ESTOP button will toggle engaged/disengaged.

Support for the `/emergency_stop` topic should be built into all running controllers

### Load Trajectory screen (`/load_trajectory`)

IN PROGRESS

## Implmenetation Details

Both Dash and ROS2 follow their own parallelism paradigms which butt heads in some ways. Currently, the server will spin up ROS2 in its own thread with Dash running in the primary thread.

Once the ROS2 node instance has been set spinning, the instance is then passed to the dash dashboard handler as a server side variable. When the dashboard needs to interface with ROS2, node instance methods can be called.

This somewhat breaks the 'stateless' property of Dash, but as this is being used in a limited capacity in a controlled environment with limited people using the dashboard at any one time (and not on the internet), we deemed that it was a good enough solution to the problem. It is safe enough to have a single version of the node running on the server machine with usually one, or maybe two users using the dashboard at any one time.

Ideally we would have used the [ros2-web-bridge](https://github.com/RobotWebTools/ros2-web-bridge) with React but this would have required a large learning curve for React which we wanted to avoid.
