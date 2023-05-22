---
layout: post
title:  "Using gazebo plugins (Gazebo classic)"
date:   2023-05-03 03:25:00 +0700
categories: [ros, gazebo]
---

## Intro
Gazebo plugins enables the simulation of models and sensors, such as drivers, cameras, lidars, etc. Aside from some common tags (ex: `updateRate`, `visualize`), different plugins have different custom parameters that could be configured.


Lists of available plugins could be found at the [official Gazebo Tutorial](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins).

* This is for Gazebo-classic. 
* Users of Ubuntu 18.04+ could switch to Gazebo Garden for more advanced functionalities.

## General usage
In robot URDF, insert `plugin` inside the `gazebo` tag.
```xml
<robot>
    ......
    <gazebo>
        <plugin name="${name_of_plugin}" filename="${file_name_of_plugin}">
            ... plugin paameters ...
        </plugin>
    </gazebo>
    ......
</robot>
```

## List of common plugins
### Differential Drive
* plugin filename: `libgazebo_ros_diff_drive.so`
* suitable for driving 2-wheel robots
  - listen to motion command (geometry_msgs/Twist)
     -  `<commandTopic>` : topic name, default to 'cmd_vel'
     -  could be controlled with teleop mode
    ```python
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

  - publish to transforms for wheel links & odometry, joint states of wheels
![](https://hackmd.io/_uploads/rJfrozuV2.png)

### Skid Steering Drive
* plugin filename: `libgazebo_ros_skid_steer_drive.so`
* suitable for driving 4-wheel robots
  - receive motion command (geometry_msgs/Twist)
    - `<topicName>` : default to 'cmd_vel'
  - publish transforms & joint states of wheels
    - `<broadcastTF>` : set true/false

### Lidar
* plugin filename: `libgazebo_ros_lidar.so`

### Camera
* plugin filenmae: `libgazebo_ros_camera.so`
* In order to transform from the ROS coordinate to the camera frame, it is recommended to define additional frame with a `"_optical"` suffix. (refer: [REP 103](https://www.ros.org/reps/rep-0103.html))
* The joint between **camera_link** & **camera_optical_link** would be configured as
  ```xml
  <origin xyz="0.0 0.0 0.0" rpy="-1.57 0.0 -1.57"/>
  ```
* [example - camera.xacro](https://github.com/yrsheld/ros_projects/blob/gazebo_sensors/mybot/urdf/camera.xacro)
  - In the gazebo section, the camera's parameters, including the `fov`, `width`, `height`, `format`, `far`/`near` clip distance (deciding the shape of the frustrum), `noise`, could be configured
  - The gazebo reference would still be the **camera_link**, whereas the plugin frameName tag would be the **camera_optical_link**.
```xml
<!-- camera -->
<gazebo reference="mybot/camera_link">
    <material>Gazebo/Black</material>
    <sensor type="camera" name="camera1">
        <update_rate>30.0</update_rate>
        <visualize>true</visualize>
        <camera>
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>800</width>
                <height>800</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <!-- Noise is sampled independently per pixel on each frame.
                That pixel's noise value is added to each of its color
                channels, which at that point lie in the range [0,1]. -->
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>

        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
            <cameraName>camera1</cameraName>
            <frameName>camera_optical_link</frameName>
        </plugin>
    </sensor>
</gazebo>
```

### Depth camera (Microsoft Kinect)
* plugin filename: `libgazebo_ros_openni_kinect.so`
* similar to camera, except the sensor `type`  (within `<sensor>`)would be "depth"
* publish
    - `camera/depth/image`: depth image
    - `camera/image`: RGB image
    - `camera/depth/points` (sensor_msgs/PointCloud2): the pointcloud that contains the 3d information
      * The color format need to be reconfigured in order to show correct color, or else the R & B would be messed up
      * This can be done by configuring the `<format>` (within `<image>`) **from R8G8B8 to B8G8R8**
      
### Projector
* plugin filename: `libgazebo_ros_projector.so`
* Project static image (texture) in world.
* The `<texture>` tag would be the name of the image file.
  - The image should be put in folder `/usr/share/gazebo-7/media/materials/textures` (see this [thread](https://stackoverflow.com/questions/63401355/project-a-line-laser-visible-on-camera-sensor-on-gazebo-simulation)). There also exist a couple of textures.
* Replace the `<sensor>` tag with `<projector>` tag.
* The projection would be in +Z direction.
* Projector parameters include
  - `near_clip`
  - `far_clip`
  - `fov`
  (Other tags seem to have no effect by far...)
  (`<textureTopicName>` & `<projectorTopicName>` not working as well...)


## Reference
* ros wiki - [gazebo_plugins](http://wiki.ros.org/gazebo_plugins)
* official Gazebo Tutorial - [Gazebo plugins in ROS](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
* gazebo plugin source code - [Github repo](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/noetic-devel/gazebo_plugins)
* example of xacro files with lidar, (depth) camera, projector plugin - [My project - mybot (branch "gazebo-sensors")](https://github.com/yrsheld/mybot/tree/gazebo_sensors/urdf)