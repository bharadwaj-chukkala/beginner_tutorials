[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# ROS2 Publisher and Subscriber

## Overview

A Package built using ```colcon``` build as a part of ROS2 Tutorials. It contains the implementation of a Talker and a Listener node.

## Contents


<pre>├── include
│   └── beginner_tutorials
|      ├── publisher_function.hpp
│      └── subscriber_function.hpp
├── results
│   ├── cppcheck.txt
│   └── cpplint.txt
├── package.xml
├── CMakeLists.txt
├── README.md
└── src
    ├── publisher_member_function.cpp
    └── subscriber_member_function.cpp</pre>

## Instructions to Build the Package
```
cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/bharadwaj-chukkala/beginner_tutorials.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select beginner_tutorials
```

## Instructions to Run the Package

### Run the Publisher 
In a new terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials talker
```

### Run the Subscriber
In another terminal, navigate to your ROS2 workspace (```ros2_ws```) and source the setup files,
```
cd <path-to-ROS2-workspace>/ros2_ws
. install/setup.bash
ros2 run beginner_tutorials listener
```

Enter ```Ctrl+c``` in each terminal to stop the nodes from spinning.

## Results
### Outputs
#### Talker Node
```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials (ros_pub_sub) $ ros2 run beginner_tutorials talker
[INFO] [1668062183.614769978] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  0'
[INFO] [1668062184.114694591] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  1'
[INFO] [1668062184.614729694] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  2'
[INFO] [1668062185.114778975] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  3'
[INFO] [1668062185.614698424] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  4'
[INFO] [1668062186.114692743] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  5'
[INFO] [1668062186.614639782] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  6'
[INFO] [1668062187.114668045] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  7'
[INFO] [1668062187.614657899] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  8'
[INFO] [1668062188.114706558] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  9'
[INFO] [1668062188.614695506] [minimal_publisher]: Publishing: 'Hey, This is Bharadwaj, ID:  10'
```

#### Listener Node
```
bharadwaj@Alpha-Phoenix ~/tests for ros2/beginner_tutorials (ros_pub_sub) $ ros2 run beginner_tutorials listener
[INFO] [1668062186.614874934] [minimal_subscriber]: I heard: 'Hey, This is Bharadwaj, ID:  6'
[INFO] [1668062187.115330341] [minimal_subscriber]: I heard: 'Hey, This is Bharadwaj, ID:  7'
[INFO] [1668062187.615026935] [minimal_subscriber]: I heard: 'Hey, This is Bharadwaj, ID:  8'
[INFO] [1668062188.115233154] [minimal_subscriber]: I heard: 'Hey, This is Bharadwaj, ID:  9'
[INFO] [1668062188.615349421] [minimal_subscriber]: I heard: 'Hey, This is Bharadwaj, ID:  10'
```

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
