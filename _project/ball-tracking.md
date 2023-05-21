---
layout: project_single
title:  "Ball tracking with RGB camera"
slug: "ball-tracking"
---

## Intro
### Goal
* Track a red ball with RGB camera.
* Move along to keep the ball in sight.
* If ball out of sight, rotate and search for it.
### Structure Overview
Main nodes:
- ball_detector_3d: object detection & obtain its 3D position.
- ball_tracker: given 3D position of target object, generate motion command to chase after it.
- ball_move: generate ball motion to move along a defined trajectory.

![rqt_graph](https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/ball-tracking/rqt_graph.png)

## 1.  Object detection with image processing
### Goal
Process image from RGB camera to obtain 2D position of red ball.
### Note
Lots of parameter tuning is required in case of different target object and scene.
Here the target object and the gazebo scene is rather simple, hence an uncomplicated processing is sufficient.

### Implementation
* Read in image message (sensor_msgs/Image), convert to BGR
Handled by cv_bridge
* Convert image format from BGR to HSV
Reason: HSV works better for thresholding
You may need to try out threshold values. Refer to [color_finder.py](https://github.com/yrsheld/Ball_tracking/blob/main/scripts/color_finder.py) for generating a GUI for value tuning.
* Find circle
For general geometry search, the `cv2.findContours` & `cv2.approxPolyDP` may come in handy. You may able to differentiate different shapes by the number of edges.

Here, since we are searching for a red ball, we only target at searching for circles. `cv2.HoughCircles` performs hough circle transform, and returns the center position and the radius of each circle.

The center position of the circle would now be rendered as the 2D position of the red ball.

## Coordinate transformation
### Goal
Convert 2D position (pixels) to 3D position (meters) w.r.t the camera frame.
For verifying the result, a Marker would be published to the rendered 3D position.

### Implementation
#### Required parameters
- ball actual radius
- camera information
  - height, width (pixels)
  - vertical, horizontal field of view (rad)
  - frame (usually the camera_optical_link)

For gazebo simulation, the camera information could be obatined from the model description (.xacro); for real cameras, those informations need to be measured.

#### Convert 2D pixel position to 3D position
Be careful of the coordinates. The **camera optical frame** is at the image center, and the z-axis would be pointing to the front.

![rqt_graph](https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/ball-tracking/camera_coordinate.png)

Calculate 3D coordinate, w.r.t. the camera optical frame, based on the circle center and radius in 2D image.

For easy verification, the result would be visualized as a Marker, which could be seen in rviz.
``` python
def convert3D(self, x_2d, y_2d, r_2d):
    '''
    convert 2d coords to 3d coords
    '''
    p = Point()
    theta_ball = r_2d / self.width * self.hfov
    # Calculate 3d distance to ball
    d = self.ballRadius / math.tan(theta_ball)

    theta_y = (y_2d / self.height - 0.5) * self.vfov
    p.y = d * math.sin(theta_y)
    d_ = d * math.cos(theta_y)

    theta_x = (x_2d / self.width - 0.5) * self.hfov
    p.x = d_ * math.sin(theta_x)
    p.z = d_ * math.cos(theta_x)
        
    return p       
```

## Motion control
### Goal
Keep a fixed distance to the target object. Move along with the target and try to keep it at the center of view. If out of sight, rotate and search for it.

### Implementation
#### Subscribe to object 3D position
- Update the deviation of x & z with the received position. (The y is always fixed, since the ball is always on the ground)
   - To ensure a smoother control, update the values with a **decay rate**, instead of directly overwriting with current values.
- Record the msg receive time, which would be useful later for determining whether the ball is out of sight.
``` python
def pos_cb(self, pos):
    '''
    set target distance according to ball 3D position
    '''
    self.target_xdist = self.target_xdist * self.decay + pos.x * (1-self.decay)
    self.target_zdist = self.target_zdist * self.decay + pos.z * (1-self.decay)

    self.lastTime = time.time()
```
#### Publish velocity command
Publish to the `/cmd_vel` topic every 0.1 second. Since the robot only performs planar motion, the velocity command only need to have 2 nonzero entries (linear x for moving forward, angular z for rotation)

There are basically two modes:
* **Target in sight**: Linear acceleration in case of being too far away from the target object. Rotate to keep the object at the center of view.
* **Target out of sight (Timeout)**: If the last message receiving time is too long ago (i.e., exceeding the defined timeout), then rotate to search for its appearance.

## Result
### Track orbiting ball
[![Watch the video](https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/ball-tracking/demo1.png)](https://drive.google.com/file/d/1bk_2yLANjX-Kp12dok39rbulLUgWbe0a/preview)

### Track ball at random position
[![Watch the video](https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/ball-tracking/demo2.png)](https://drive.google.com/file/d/1xikqzR-tnM_xhlAvBpf1oMSEjbFYNEum/preview)


## Summary
1. Image processing to find object 2D pose.
2. Based on camera parameters and known size of the object, calculate the 3D position.
3. Based on 3D position of object, generate velocity command to keep the object in the center of view.

This is a fairly simple demonstration of how to derive information from RGB image. 

Future goals:
* Detect more complicated objects (ex: more complex geometries in more complex scene)
* Explore different computer vision algorithms.
* Use pretrained NN to realize more powerful & robust detection.
* Try with depth camera.
* Find better motion control methods. 

## Reference
- [Tiziano Fiorenzani - ROS tutorial](https://github.com/tizianofiorenzani/ros_tutorials/tree/master/opencv)