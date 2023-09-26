---
layout: project_single
title:  "Implement global path planner as a ROS plugin"
slug: "global-path-planner-plugin"
---

## Intro

### Goal
Implement path planning algorithm as a global path planner. 

Given
* a 2D costmap of the environment
* the start pose (i.e., the current pose)
* the goal pose

The path planner would generate the global plan for the desired movement.

### Move_base
ROS navigation stack handles the navigation, from building the map, localization, and path planning. 

The move_base node is a main component of the navigation stack. It subscribes to all essential informations for path planning (ex: tf, odom, map, sensor topics, and the goal pose), and plan the path with its global planner & local planner, then publishes the planned result as a series of velocity commands (/cmd_vel), which would be sent to the controller.

There are multiple built-in global path planners provided, such as the default: NavfnROS. In order to let the move_base node use our own path planning alrgothm, it needs to be written as a ROS Plugin.


## Implement global path planner as ROS plugin

There is a [ROS Wiki tutorial](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS) on this topic.

Generally, the idea is as follows:

### 1. Implement the path planner class
The path planner needs to be implemented as a **C++ class** (See *Side Note 2* for python implementation), overriding the `nav_core::BaseGlobalPlanner`.

Two functions need to be overriden:
  * void initialization(...)
  * bool makePlan(...): where the main algorithm lies in. The planned result is stored in the `std::vector<geometry_msgs::PoseStamped>& plan`

For instance, I can defined a "MyGlobalPlanner" class in my_global_planner.cpp

```cpp
class MyGlobalPlanner : public nav_core::BaseGlobalPlanner{
    MyGlobalPlanner();
    MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        //do some initialization.....
        //ex: read costmap info
    }
    
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan){
        //main planning algotihm...
        //store result in plan
    }
}
```


### 2. Register the class as plugin 

Then we need to register this planner as plugin for nav_core::BaseGlobalPlanner of the move_base. 

Add the following to the beginning of the .cpp file where the class is defined. The planner_class_name is the name of the planner class.


```cpp
//PLUGINLIB_EXPORT_CLASS(${planner_class_name}, nav_core::BaseGlobalPlanner)
//for instance:
PLUGINLIB_EXPORT_CLASS(my_global_planner, nav_core::BaseGlobalPlanner)
```


### 3. Compile the planner library
catkin_make compiles the catkin workspace and generates binary files. To compile the planner library, we need to add the following to the `CMakeLists.txt`.

After compiling, the library file would be stored as `/catkin_ws/devel/lib/lib${your_library_name}.so`


```cmake
#add_library(${your_library_name} src/${name_of_you_cpp_file}.cpp)
#for instance: the binary file - libmy_global_planner_lib.so
add_library(my_global_planner_lib src/my_global_planner.cpp)
```

### 4. Add a description of the plugi

The plugin description file stores all the important information about a plugin in a machine readable format. 
It defines
* the library the plugin is in, 
* the name of the plugin, 
* the type of the plugin

Here I name the xml as - `mgp_plugin.xml`. Within this file:

```xml
<library path="lib/libmy_global_planner_lib">
  <class name="my_global_planner/MyGlobalPlanner" type="my_global_planner::MyGlobalPlanner" base_class_type="nav_core::BaseGlobalPlanner">
    <description>
      A global planner plugin.
    </description>
  </class>
</library>
```

Note that the library path needs to match with the library file (with the "lib" prefix) in step 3.

### 5. Register the plugin
In order to let pluginlib to discover this plugin automatically, we need to explicitly export the plugin.

In **CMakeLists.txt**, add:

```camke
<export>
  <nav_core plugin="${prefix}/mgp_plugin.xml" />
</export>
```

* As always, we would also need to add all the required dependencies (ex: nav_core, pluginlib) in the CMakeLists.txt and package.xml.


### Final file structure
The package would be finally structured as below:

```
my_global_planner
├── CMakeLists.txt
├── include
│   └── my_global_planner
│       ├── astar.h
│       ├── dijkstra.h
│       ├── my_global_planner.h
│       └── utils.h
│── src
│   ├── astar.cpp
│   ├── dijkstra.cpp
│   ├── my_global_planner.cpp
│   └── utils.cpp
├── mgp_plugin.xml
├── package.xml
```

## Result
The plugin would generate the global plan, and we can specifying which algorithms to use in the makePlan function.

Currently A* and Dijkstra algorithms are avaialble.
A sample running of Dijkstra planner:

[![Watch the video](https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/projects/dijkstra_demo.png)](https://drive.google.com/file/d/1plcW52jBbf_acsAuNExRNxo5p1KRkh4a/preview)


## Side note

### 1. Global vs. local planner

Conceptual wise, 
* global planner plans a long term path around obstacles 
* local planner adds smoothing (collision check) to the global plan, based on real-time sensor data. It produces the final velocity command that is sent to the controller.

The parameters
* planner_frequency: how frequent the global planner replans, in case of new obstacles and changes in environment.
* controller_frequency: how frequent the local planner produce velocity command (/cmd_vel)

### 2. Python implementations of path planning algorithm

As from my current understanding, the global planner plugin needs to be written as C++ class and adheres to the navcore::BaseGlobalPlanner, as described in step 1.

However, there are still certain ways to use a planning algorithm implemented in Python. For instance, we can write the python-based planning algo as a ROS service server, and create the corresponding service client in the C++ class. The makePlan method would call the ROS service whenever a goal pose is given. The request would contain all essential information for planning, and the service server would send back the planning result in the response.

### 3. Header files
To switch between different planning algorithms, I implemented the functions separately in different files, which would be called within the makePlan function of the plugin class (`MyGlobalPlanner::makePlan`).

Since they are in different files, we need to include the header files of the algorithms. I found it a little bit tricky to deal with header files for both ROS nodes and plugins. So I wrote [another note](https://yrsheld.github.io/ros/2023/09/21/ros-header-files.html) for this topic.

For our current example, the `my_global_planner.cpp` is where the plugin class defined, and it calls the "astar" and "dijkstra" functions from the `astar.cpp` and `dijkstra.cpp`. So when compiling the plugin as a library (in step 3), we also need to add those **.cpp** files (instead of .h) as sources.


```cmake
set(MY_GLOBAL_PLANNER_LIB_SOURCE 
    src/my_global_planner.cpp
    src/utils.cpp
    src/astar.cpp
    src/dijkstra.cpp
)

add_library(my_global_planner_lib ${MY_GLOBAL_PLANNER_LIB_SOURCE})
```

## Reference:

1. ROS Wiki - [move_base](http://wiki.ros.org/move_base)
2. ROS Wiki - [Write ROS Plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
3. ROS Wiki - [Write global path planner as ROS Plugin](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)
4. [Global and local path planning](https://www.hindawi.com/journals/jat/2018/6392697/)
5. [Path planner plugin in python](https://robotics.stackexchange.com/questions/96548/writing-a-planner-plugin-for-ros2-using-python)