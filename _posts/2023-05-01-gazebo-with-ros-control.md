---
layout: post
title:  "Control Gazebo model with ros_control"
date:   2023-05-01 20:41:00 +0700
categories: [ros, gazebo]
---
## Intro
### DataFlow

![](https://i.imgur.com/ZJlvKRI.png)*image_source: [official gazebo tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control)*

### Usage
In order to control or steer the model in Gazebo, we make use of gazebo controller plugins. Following things need to be specified.

In URDF:
* add gazebo ros control plugin (Step1)
* add transmission of each non-fixed joint (Step 2)

In config:
* add controller config, as .yaml files (Step 3), which would be initially loaded into the ROS parameter space

In launch file (or manually):
* load controller configs to ROS parameter space
* controller_manager (Step 4)


## Step1: Gazebo Plugin
Enable interaction between ROS & Gazebo.
Dynamically link to the ROS library, which will tell Gazebo what to do.

### Specify the ros control plugin in URDF

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/</robotNamespace>
    </plugin>
</gazebo>
```

If you want to specify the namespace, then fill in the `robotNamespace`.

ex:
```xml
<robotNamespace>/mybot</robotNamespace>
```

## Step2: Add transmission elements to URDF
In order to use ros_control with the robot, add transmission elements for each non-fixed joint. This links an actuator to each non-fixed joint.

### URDF tags:
* **joint** - specify the "name" as the joint name predefined in URDF
* **actuator** - specify the "name" as the name of the motor, and the "mechanicalReduction" as the reduction ratio of the motor.
* **type** - type of transmission.
  - `transmission_interface/SimpleTransmission`
* **hardwareInterface** - added in both the `joint` & `actuator` tags. It tells gazebo_ros_plugin which hardware interface to load.
  - PositionJointInterface
  - VelocityJointInterface
  - EffortJointInterface

### Note:
* The `hardwareInterface` specified here need to match the controller type we specified in Step3.
* You may get a warning of "*Deprecated syntax, please prepend 'hardware_interface/' to 'PositionJointInterface' within the <hardwareinterface> tag in joint ...*". 
To avoid this warning, simply do what it says, i.e., add "hardware_interface/" at the beginning.

### Example:

Attach a position joint controller to a revolute joint, named "joint1"
```xml
<transmission>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="joint1_motor">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
    </actuator>
</transmission>
```

## Step 3. Configure controllers in .yaml files
These controller configurations would be loaded into the ROS parameter space. 

Later in Step 4, we would load the controllers with controller_spawner (type: `controller_manager/spawner`).

### YAML keys
* type - type of controller (refer to [ros_controllers](http://wiki.ros.org/ros_controllers))
    - [DiffDriveController](https://github.com/ros/urdf_sim_tutorial/blob/master/config/diffdrive.yaml)
    - [JointPositionController](http://wiki.ros.org/robot_mechanism_controllers/JointPositionController)
    - [JointStateController](https://github.com/ros/urdf_sim_tutorial/blob/master/config/joints.yaml)
publish joint states of all joints with hardware_interfaces and advertises the topic on /joint_states
    - [JointGroupPositionController](https://github.com/ros/urdf_sim_tutorial/blob/master/config/gripper.yaml)
Group joints that need to move together. Note that, in the URDF, we still need to add transmission element (of hardwareInterface - PositionJointInterface for all the joints in here)
* publish_rate
* joints: name of the associated joints

And a lot more ROS parameters that are specific for different types of controllers.

### Note
As mentioned in Step2, the controller type specified here need to match the `hardwareInterface` in Step2.

### Example
if I have two joints, joint1 & joint2, both are revolute joints with **position controllers**. We could either define each different controller in separate files, or all in one file.

#### 1. In separate files (joint1.yaml, joint2.yaml, joints.yaml):

```yml
# joints.yaml, publish all joint states
type: "joint_state_controller/JointStateController"
publish_rate: 50

# joint1.yaml
type: "position_controllers/JointPositionController"
joint: joint1
pid:          # pid parameters could also be specified
    p: 100.0
    i: 0.01
    d: 10.0

# joint2.yaml
type: "position_controllers/JointPositionController"
joint: joint2
pid:
    p: 100
    i: 0.01
    d: 10.0
```

#### 2. In a single file (mybot_control.yaml)

The robot namespace could be specified at the top level. 
* But note that, by doing so, the `"/joint_states"` topic would now be `"mybot/joint_states"`. 
* Thus, you need to make sure to set all the related nodes (ex: `robot_state_publisher`) in the same namespace.

```yml
mybot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  

  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: position_controllers/JointPositionController
    joint: joint1
    pid: {p: 100.0, i: 0.01, d: 10.0}
  joint2_position_controller:
    type: position_controllers/JointPositionController
    joint: joint2
    pid: {p: 100.0, i: 0.01, d: 10.0}
```

## Step 4. Load controller configurations & Call service - ros_control
As mentioned in previous step, now we would load the control configs from Step3. This is usually done via a launch file.

Simply by running the node - [controller_manager/spawner](http://wiki.ros.org/controller_manager), and gives the name of the controllers as arguments, which would make a service call to the ros_control controller manager, and automatically start all controllers.

After launching this launch file, we could then control the position of joints by publishing to the controller's command topic. As for the type of topic, refer to each controller. 
(ex: [JointPositionController](http://wiki.ros.org/robot_mechanism_controllers/JointPositionController)/command - type: `std_msgs/Float64`)

Ex: Publish to the `mybot/joint1_position_controller`
```python
rostopic pub /mybot/joint1_position_controller/command std_msgs/Float64 "data: -0.707"
```
Ex: Publish to a joint group position controller, named `mybot/gripper_controller`, with three joints within.
```python
rostopic pub /mybot/gripper_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 1
 data_offset: 0
data: [0, 0.5, 0.5]"
```

### Example

#### 1. In separate files (joint1.yaml, joint2.yaml, joints.yaml):
```xml
<!-- load joint1 yaml -->
<rosparam file="$(find mybot)/config/joint1.yaml" 
        command="load" 
        ns="mybot_joint1_position_controller" />

<!-- load joint2 yaml -->
<rosparam file="$(find mybot)/config/joint2.yaml" 
        command="load" 
        ns="mybot_joint2_position_controller" />

<!-- load joints yaml -->
<rosparam file="$(find mybot)/config/joints.yaml" 
        command="load" 
        ns="mybot_joint_state_controller" />

<!-- spawn controllers -->
<node pkg="controller_manager" 
    type="spawner" 
    name="mybot_controller_spawner"
    args="mybot_joint1_position_controller 
        mybot_joint2_position_controller
        mybot_joint_state_controller"/>
```

#### 2. In a single file (mybot_control.yaml)
```xml
<!-- load mybot_control yaml -->
<rosparam file="$(find mybot)/config/mybot_control.yaml" 
         command="load"/>

<!-- spawn controllers -->
<node pkg="controller_manager" 
      type="spawner" 
      name="mybot_controller_spawner"
      ns="/mybot"
      args="joint1_position_controller 
            joint2_position_controller
            joint_state_controller"/>
```

## Step 5. Control via controllers
After starting the controllers, we could then control the position of joints by publishing to the command topic that the controllers subscribe to. As for the type of topic, refer to each controller's subscribed topics. 

### Controller subscribed topics
To name a few:
- [JointPositionController/command](http://wiki.ros.org/robot_mechanism_controllers/JointPositionController) - position command (`std_msgs/Float64`)
- [JointVelocityController/command](http://wiki.ros.org/robot_mechanism_controllers/JointVelocityController) - velocity command (`std_msgs/Float64`)
- JointGroupPositionController/command - position commands (`std_msgs/Float64MultiArray`)
- [DiffDriveController/command](http://wiki.ros.org/diff_drive_controller?distro=noetic) - velocity command (`geometry_msgs/Twist`)

### Publish commands to topics
#### 1. Publish to topic via command
Ex: Publish to the `mybot/joint1_position_controller`
```python
rostopic pub /mybot/joint1_position_controller/command std_msgs/Float64 "data: -0.707"
```
Ex: Publish to a joint group position controller, named `mybot/gripper_controller`, with three joints within.
```python
rostopic pub /mybot/gripper_controller/command std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 3
    stride: 1
 data_offset: 0
data: [0, 0.5, 0.5]"
```

#### 2. Publish to topic via rqt_robot_steering
Make use of `rqt_robot_steering/rqt_robot_steering`, which would generate a GUI for you to adjust the command values dynamically.
Specify the topic name as the param - `"default_topic"`

Ex:
```xml
<!--rqt robot steering-->
<node name="rqt_robot_steering" pkg="rqt_robot_steering" type="rqt_robot_steering">
    <param name="default_topic" value="/mybot_diff_drive_controller/cmd_vel"/>
</node>
```
    
#### 3. rqt_gui
Use rqt_gui to publish to target topics. And we could even visualize the command value & joint states and perform controller tuning with it.
Refer to the [gazebo_tutorial](https://classic.gazebosim.org/tutorials?tut=ros_control#Visualizethecontroller'sperformance).


## Summary
* To enable interaction between gazebo & ros, we add the gazebo_ros_control plugin in the URDF.
* Each non-fixed joint is linked to an actuator (motor) by specifying the transmission element and its corresponding hardwareInterface, meaning the type of controller it uses.
* Configure parameters of all controllers in .yaml files, where we specify the type, publish_rate, or even the pid parameters.
* Load controllers into parameter server &  start controllers via controller_manager/spawn, normally done in a launch file.
* Now the controllers are up, they are listening to their respective topics. To give commands to the controller, we could either publish msgs or use rqt.

## Reference
* official ros wiki - [Using a URDF in Gazebo](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo)
* official gazebo tutorial - [ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control)
* ros wiki - [ros_control](http://wiki.ros.org/ros_control)
* ros wiki - [ros_controllers](http://wiki.ros.org/ros_controllers)
* demo of using rqt-robot-steering to control gazebo model - [project - Driving with ros control](https://yrsheld.github.io/project/driving-with-ros-control/)