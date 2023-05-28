---
layout: post
title:  "Kinematics and Dynamics Library - orocos KDL"
date:   2023-04-03 14:42:00 +0700
categories: [ros]
---
## Intro
The KDL library provides a framework for modeling and computing kinematic chains, including foward/inverse kinemtaic solvers.

Required ROS packages:
* `orocos-kdl` - KDL Library
* `kdl-parser` - for processing urdf

## Common KDL geometric primitives

### `KDL::Tree`    
Parsed from urdf

### `KDL::Chain`
Kinematic chain, from robot base to tcp, generated from KDL tree `tree.getChain(root, end_link, chain)`
### `KDL::Frame`
Coordinate frame. 4x4 matrix that represents the pose of an object/frame wrt a reference frame. It contains:

* a `KDL::Rotation` M (3x3 matrix)
* a `KDL::Vector` p  (3-dim vector)
* Create Frame
``` cpp
Frame f1; //identity frame
Frame f1=Frame::Identity();
Frame f2(your_rotation); //with zero vector
Frame f3(your_vector); //with identity rotation
Frame f4(your_rotation,your_vector);
Frame f5(your_vector,your_rotation);
Frame f5(f6); //the copy constructor
```

* Access frame values

``` cpp
KDL::Vector v = f.p      //3-dim vector
KDL::Rotation R = f.M    //3x3 matrix
    
//Access each entry
//position
double x = f.p(0);  // = f(0,3)
double y = f.p(1);  // = f(1,3)
double z = f.p(2);  // = f(2,3)

//orientation
//1. raw values
double Xx = f(0,0);
double Yy = f(1,1);
double Zz = f(2,2);

//2. or get euler transform
double alpha, beta, gamma;
f.M.GetEulerZYZ(alpha, beta, gamma);

//3. or get Quaternion
geometry_msgs::Pose pose;
f.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

//4. or get RPY
double r, p, y;
f.M.GetRPY(r, p, y);
```



Further details, refer to [orocos wiki - geometric primitives](https://www.orocos.org/wiki/Geometric_primitives.html)

## Apply to robot URDF
### 0. Include header files
In order to make use of KDL library, we need to insert the source code as header files.
Below are common headers.
``` cpp
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
```
### 1. Generate `KDL::Tree`
`kdl_parser` parses the urdf file and generate KDL tree (`KDL::Tree`).
``` cpp
KDL::Tree tree;
kdl_parser::treeFromFile('/path/to/urdf', tree);
```

### 2. Generate kinematic chain `KDL::Chain`
``` cpp
KDL::Chain chain;
tree.getChain("base_link", "wrist_3_link", chain);
```

### 3. Create forward/inverse kinematics solver, based on kinematic chain.
> List of solvers at [https://www.orocos.org/wiki/Kinematic_and_Dynamic_Solvers.html](https://www.orocos.org/wiki/Kinematic_and_Dynamic_Solvers.html)
#### Fk solver
* `KDL::ChainFkSolverPos_recursive`
``` cpp
KDL::ChainFkSolverPos_recursive fk_solver(chain);
```
#### Ik solver
Need another velocity solver
* `KDL::ChainIkSolverVel_pinv`   - velocity solver
* `KDL::ChainIkSolverPos_NR `    - Ik solver, take the velocity ik and the fk solver as arguments.

``` cpp
//IK velocity solver
KDL::ChainIkSolverVel_pinv vel_ik_solver(chain, 0.0001, 1000);  // tolerance, max_num_iterations
//IK solver 
KDL::ChainIkSolerPos_NR ik_solver(chain, fk_solver, vel_ik_solver, 1000);  //max_num_iterations
```
## About FK & IK
Taking a deeper look into the the forward/inverse kinematic solvers.

### FK solver

`fk_solver.JntToCart(joint_values, tcp_pose)` - Compute TCP pose, based on Joint values
* Given: Joint values (`KD::JnTArray`)
* Output: TCP pose (`KD::Frame`) 

#### Example
Obtain current joint values through subscribing to joint controller state process value (`/xxx_controller/state/process_value`).
Then use FK solver to calculate current TCP.
``` cpp
// For instance, there are 6 joints in the kinematic chain
number_of_joints = 6;
KD::JnTArray joint_pose(number_of_joints);

// Obtaining each joint position from callback funcion
// Joint 0 value
void joint0_val_callback(const control_msgs::JointControllerState::ConstPtr& control_msg){
    joint_pose(0) = control_msg->process_value;
}
// Joint 1 value
void joint1_val_callback(const control_msgs::JointControllerState::ConstPtr& control_msg){
    joint_pose(0) = control_msg->process_value;
}

// ......

// All joint values already obtained.
// FK solver compute TCP position
// Converting KD::JnTArray (joint values) --> KD::Frame (tcp)
KD::Frame tcp_pose;
fk_solver.JntToCart(joint_pose, tcp_pose);
```

### IK Solver
`ik_solver.JntToCart(jnt_pos_start, tcp_pos_goal, jnt_pos_goal)`
* Given: 
  * Initial joint values (jnt_pos_start `KD::JnTArray`)
  * Target TCP pose (tcp_pos_goal `KD::Frame`)
* Output: 
  * Target joint values (jnt_pos_goal `KD::JnTArray`) 

#### Example
``` cpp
KDL::JntArray joint_pos_goal(number_of_joints);
ik_solver.CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal);
```
#### Algorithm of IK Solver
> This is a **numeric solver**, i.e., solves iteratively. There are also other numeric and analytical solvers from different libraries.

Having the target tcp position, we try to get closer to the goal tcp position and adjust the joint value accordingly in each iteration.
Keep iterating until joint values could lead to a tcp position that is close enough to the target tcp within a defined tolerance.

Basically, in each iteration we do the following steps

**1.** [FK] Compute current TCP position, based on current joint values (q)

**2.** Compute: Velocity ($\Delta$x) of TCP = difference between target TCP & current TCP

**3.** Compute joint velocity ($\dot{q}$), based on formula below. (Refer to [roboticscasual.com](https://roboticscasual.com/tutorial-controlling-tcp-position-of-the-ur5-robot-with-kdl-in-c-inverse-kinematics/) for detailed derivation)
    
* $v = J(q) * \dot{q}$
*    $\dot{q} = J^{+}(q) * v$, where $J^{+}(q)$ is the pseudo-inverse of Jacobian
    
**4.** Increment joint value with $\dot{q}$


And so this is why the IK Solver (`KDL::ChainIkSolverPos_NR`) requires
* KDL chain (`KDL::Chain`)
* FK Solver (`KDL::ChainFkSolverPos_recursive`)
* IK velocity solver (`KDL::ChainIkSolverVel_pinv`)
* Maximum number of iterations

as arguments.

#### Postprocessing of target joint values
Next step is to do interpolation (linear/polynomial/...) between the target & current joint values, i.e., the joint value profile within a certain time period. The interpolated values would be sent to controller as target joint values for each timestep.

## Reference
* Most contents are summarized from [roboticscasual.com](https://roboticscasual.com/tutorial-controlling-tcp-position-of-the-ur5-robot-with-kdl-in-c-inverse-kinematics/)
* [orocos manual](https://www.orocos.org/kdl/user-manual.html)
* [orocos source code](https://github.com/orocos/orocos_kinematics_dynamics)