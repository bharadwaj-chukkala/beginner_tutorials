# ROS2 tf2, Unit Testing, Bag Files

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

A Package built using ```colcon``` build as a part of ROS2 Tutorials. It contains the implementation of a Publisher node with a service to add two integers. It contains a parameter node that manipulates a string output. This repository also contains of a launch file that will launch all nodes at the same instant. The service will be demonstrated using a service call for testing, which then can also be called to run when we run a client node in another terminal while running the publisher node. The parameter manipulation can be done by setting a new value to the parameter from another terminal.

## Contents

<pre>
.
├── example_interfaces
├── include
│   └── beginner_tutorials
│       ├── publisher_function.hpp
│       ├── pub_param_function.hpp
│       └── subscriber_function.hpp
├── launch
│   ├── node_launcher.yaml
│   └── system_bag_launcher.py
├── LICENSE
├── package.xml
├── README.md
├── results
│   ├── cppcheck.txt
│   ├── cpplint.txt
│   ├── frames_2022-12-04_03.16.25.pdf
│   ├── LastTest_20221204-0802.log
│   └── rqt_console_output.png
├── package_bag
│   ├── metadata.yaml
│   └── package_bag_0.db3
├── src
│   ├── client_node.cpp
│   ├── publisher_member_function.cpp
│   ├── publisher_parameter_node.cpp
│   └── subscriber_member_function.cpp
├── srv
│   ├── AddTwoInts.srv
│   └── ChangeMessage.srv
└── test
    ├── main.cpp
    └── test_talker.cpp</pre>

## Assumptions

* OS: Ubuntu Jammy Jellyfish (22.04) 64-bit
* ROS2 Distro: Humble
* ROS2 Workspace name: ros2_ws
* ROS2 Installation Directory: ros2_humble [if installed through source]

## ROS2 Dependencies

* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```
* ```example_interfaces```
* ```tf2```
* ```tf2_ros```
* ```rosbag2_cpp```
* ```rosidl_default_generators```
* ```geometry_msgs```

## Instructions to Build the Package

``` cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/bharadwaj-chukkala/beginner_tutorials.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install --packages-select beginner_tutorials
```

## Instructions to Run the Package

### tf2 broadcaster implementation

#### Run the Talker Node

In a terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials talker
```

In another terminal, source your ROS workspace as mentioned before and run:
```
ros2 run tf2_ros tf2_echo world talk
```

In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files again and verify the TF frames using tf2_echo
```
ros2 run tf2_ros tf2_echo world talk
```

Open another terminal
cd to the results directory and generate the frame output.
```
cd results
ros2 run tf2_tools view_frames -o
```

The above command will generate a frames pdf which will open upon running the above command.


#### Run the Level2 Gtest Implementation

To view the gtest results for the ROS2 publisher and tf2 broadcaster, run the following in your ```ros_ws``` workspace,
```
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

You should see 1 gtest cases to pass (one for publisher and another for tf2 broadcaster)
I have saved the test log in the ```results``` directory for your convenience.

#### Recording a bag file

To launch ROS2 bag and record all topics, run the following in a terminal sourced with your ROS workspace (```ros2_ws```) and change to the ```/results``` directory:
```
cd results
ros2 launch beginner_tutorials system_bag_launcher.py record_all_topics:=true
```

here i have setup an argument in the launch file called, ```record_all_topics``` enables bag recording when set to ```true``` and disables when set to ```false```. Default value is set to ```true```.

To disable bag recording, run
```
ros2 launch beginner_tutorials system_bag_launcher.py record_all_topics:=false
```

The bag file is saved with the filename ```package_bag_0.db3``` in the ```root``` directory, if recording is enabled.

#### Bag File verification 
To see ```/chatter``` topic messages were recorded, change to the ```package_bag``` and
```
cd package_bag
ros2 bag info all_topics_bag
```

Alternatively, in one terminal run the ```listener``` using:
```
ros2 run beginner_tutorials listener
```

In another terminal, play the recorded bag files, from the ```package_bag``` directory:
```
ros2 bag play package_bag
```

## Results

### Static Code Analysis

#### cpplint

Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/beginner_tutorials/*.hpp > ./results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

#### cppcheck

Change to the root directory of the package, ```/beginner_tutorials```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/beginner_tutorials/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.

### Previous assignement resources
If you want to check the previous asignment implementations, please look at the ``` release tags ``` and follow the instructions below.

#### Run the Publisher-Server Node

In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
``` 
cd <path-to-ROS2-workspace>/ros2_ws/src
. install/setup.bash
ros2 run beginner_tutorials talker
```

#### Run the Client Node

In another terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials client
```

#### Test the Service through a Service call

```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### Using rqt_console to check the logged messages

```
ros2 run rqt_console rqt_console
```

### Testing Parameter Manipulation

```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run cpp_parameters minimal_param_node
```

In an other terminal run the following to see change in the output if terminal 1
argument manipulated [world -> universe]

```
ros2 param list
ros2 param set /minimal_param_node my_parameter universe
```

### Running all nodes using a launch file

```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 launch beginner_tutorials node_launcher.yaml my_parameter:=universe
```

sometimes if this doesn't work, you can do the following
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
cd beginner_tutorials
cd launch
ros2 launch node_launcher.yaml
```

The launch file will launch all nodes at a time and manipulations can be run in other terminals like stated above.

Enter ```Ctrl+C``` in each terminal to stop the nodes from spinning.


## Previous Results

### Outputs

#### Publisher node with service

```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials (Week10_HW) $ ros2 run beginner_tutorials talker
[INFO] [1668638511.969467841] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  0
[INFO] [1668638512.469726725] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  1
[INFO] [1668638512.969244551] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  2
[INFO] [1668638513.469225555] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  3
[INFO] [1668638513.969188680] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  4
[INFO] [1668638514.469204052] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  5
[INFO] [1668638514.969179267] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  6
[INFO] [1668638515.469343194] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  7
[INFO] [1668638515.581445903] [rclcpp]: Incoming request a: 1 b: 2
[INFO] [1668638515.581476912] [rclcpp]: sending back response: [3]
[INFO] [1668638515.969133623] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  8
```

#### Service Call Response

```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials (Week10_HW) $ ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
waiting for service to become available...
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=1, b=2)

response: example_interfaces.srv.AddTwoInts_Response(sum=3)
```
#### Logging Levels [Terminal Response]

```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials (Week10_HW) $ ros2 run beginner_tutorials talker
[INFO] [1668640039.650735595] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  0
[INFO] [1668640040.650071332] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  1
[ERROR] [1668640041.539000346] [minimal_publisher]: Response is wrong0
[INFO] [1668640041.650356586] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  2
[INFO] [1668640042.650363608] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  3
[INFO] [1668640043.650377212] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  4
[INFO] [1668640044.650387901] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  5
[WARN] [1668640044.650534359] [minimal_publisher]: Publisher has been running excessively
[INFO] [1668640045.650104364] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  6
[WARN] [1668640045.650197743] [minimal_publisher]: Publisher has been running excessively
[INFO] [1668640046.650163698] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  7
[WARN] [1668640046.650295629] [minimal_publisher]: Publisher has been running excessively
[INFO] [1668640047.650422099] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  8
[WARN] [1668640047.650596704] [minimal_publisher]: Publisher has been running excessively
[INFO] [1668640048.650229550] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  9
[WARN] [1668640048.650413024] [minimal_publisher]: Publisher has been running excessively
[INFO] [1668640049.650440634] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  10
[WARN] [1668640049.650615239] [minimal_publisher]: Publisher has been running excessively
[FATAL] [1668640049.650659658] [minimal_publisher]: Warnings were blindsided, Fatality Occured
[INFO] [1668640050.650500111] [minimal_publisher]: Publishing:Hey, This is Bharadwaj, ID:  11
[WARN] [1668640050.650630365] [minimal_publisher]: Publisher has been running excessively
[FATAL] [1668640050.650649502] [minimal_publisher]: Warnings were blindsided, Fatality Occured
```

#### Parameter Response

```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials/launch (Week10_HW) $ ros2 launch node_launcher.yaml
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [minimal_param_node-1]: process started with pid [44475]
[minimal_param_node-1] [INFO] [1668651106.180145554] [minimal_param_node]: Hello world!
[minimal_param_node-1] [INFO] [1668651107.180334288] [minimal_param_node]: Hello world!
[minimal_param_node-1] [INFO] [1668651108.180327606] [minimal_param_node]: Hello universe!
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[minimal_param_node-1] [INFO] [1668651108.430788961] [rclcpp]: signal_handler(signum=2)
[INFO] [minimal_param_node-1]: process has finished cleanly [pid 44475]
```

#### [rqt_console output](https://github.com/bharadwaj-chukkala/beginner_tutorials/blob/Week10_HW/results/rqt_console_output.png)


