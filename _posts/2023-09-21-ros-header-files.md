---
layout: post
title:  "Configuration of CMakeLists.txt to include header files"
date:   2023-09-21 17:57:00 +0700
categories: [ros]
---

We could build executables and plugins out of cpp files, which may also depend on other header files. The **CMakeLists.txt** would need to be configured in a way such that the headers can be discovered and linked to the executable/plugin.

## Executable 
### Without header files
For instance, I want to write a simiple talker node, which would later be built as a C++ executable.

This is easy by refering to [ROS/Tutorials/WritingPublisherSubscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

To build a .cpp as a ros node, we need to specify in the cmakeList.txt
```cmake
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
```

The target_link_libraries would link the executable - talker, to all the predefined cakin_LIBRARIES.

> `cakin_LIBRARIES`: the list of all libraries (targets and real libraries) that you need to compile against the dependencies you specified in `find_package(catkin ...)`

> ex:
The executable depends on libraries - roscpp, rospy, std_msgs. So we need to define in the `find_package()`, and then link the executable to the libraries via `target_link_libraries()`. 

> This linking is necessary, or else, there would be error - "undefined reference" to all the library functions during compiling.

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
```


### With header files


If the node wants to use some other functions defined in another file.
Example:

In `talker.cpp`

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <hello_world/my_adder.h>
#include <sstream>

int main(int argc, char** argv){
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("interesting_topic", 1000);
    ros::Rate r(10);
    int a = 1, b = 2;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Addition result: " << add_two_nums(a++, b++);
        msg.data = ss.str();

        pub.publish(msg);

        ros::spinOnce();

        r.sleep();
    }
    return 0;
}
```

where the function - `add_two_nums(int, int)` is defined in another file: `my_adder.cpp`

In my_adder.h:

```cpp
#ifndef HELLO_WORLD__MY_ADDER_H
#define HELLO_WORLD__MY_ADDER_H

int add_two_nums(int a, int b);

#endif
```

my_adder.cpp:

```cpp
#include <hello_world/my_adder.h>

int add_two_nums(int a, int b)
{
    return a+b;
}
```

Now we need to make some changes in the **CMakeLists.txt** to **let the system discover the included header file** (my_adder.h) and **link the executables to it**.


#### 1. specify the include directories where the header files are (normally in the include/${PROJECT_NAME})

```cmake
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

#### 2. Declare the cpp that contains all the utility functions (my_adder.cpp) as a C++ library

```cmake
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hello_world.cpp
# )
add_library(my_op_lib src/my_adder.cpp)
```

After compiling, the library would appear in the `/catkin_ws/devel/lib/libmy_op_lib.so`. Note that, a prefix - 'lib' is added


#### 3. Link the executable to the catkin libraries and this new library!

```cmake
add_executable(talker src/talker.cpp) 
target_link_libraries(talker ${catkin_LIBRARIES} my_op_lib)
```

The second line links the target executable to not only the catkin_LIBRARIES as always, but also to the newly defined c++ library (from step2) - my_op_lib. This way, the executable "talker" can then use functions in my_op_lib!

## Plugin

The plugin itself would be made as a library. To use functions defined in other files, they should be added to the library source.

```cmake
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
set(MY_GLOBAL_PLANNER_LIB_SOURCE 
    src/my_global_planner.cpp
    src/utils.cpp
    src/astar.cpp
    src/dijkstra.cpp
)

add_library(my_global_planner_lib ${MY_GLOBAL_PLANNER_LIB_SOURCE})
```

## Reference

* http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin

* https://www.xuningyang.com/blog/2020-05-12-ros-pluginlib/