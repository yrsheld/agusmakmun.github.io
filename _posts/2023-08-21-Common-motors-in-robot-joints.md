---
layout: post
title:  "Common motors in robot joints"
date:   2023-08-21 17:57:00 +0700
categories: [others]
---

## How to actuate robot joint

Multiple components within the joint:

* Electrical motor
* Gearbox (e.g. harmonic drive, i.e., strain wave gear)
* Encoder to measure joint position

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/joints.png" alt= "latex">


## 1. AC Motor

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/ac.png" alt= "latex">


An electric motor is a coil that is mounted between two permanent magnets. When you run current through a coil, you will get a magnetic field. In this case, the magnetic field will be repelled from the permanent magnet to the left, and on the other hand, will it be attracted from the permanent magnet to the right. 

* In order to change the orientation of the magnetic field in the coil, you will **need to change the polarity of the electric current that you provide**. 
* Hence, needs to power with **AC source**. Normally, the alternating current are of 50 or 60 hertz, and you will therefore get a motor which turns out to 50 or 60 hertz, whatever comes out of the line. 
* rotation speed controlled by the external frequency of electric field. (i.e., the frequency of AC source)
* only used for very heavy machinery. 

## 2. DC Motor

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/dc_motor_1.png" alt= "latex">

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/dc_motor_2.png" alt= "latex">

What you usually do is use something called a **commutator** in which brushes, or something like this will apply the electric current in a selective manner. 

* As the coil turns, the polarity switch as soon as the device has made 180 rotation.
* With this, you **provide the alternating current automatically** and can therefore **power the motor with the simple DC source** (ex: from battery). 
* Control the speed of a DC electric motor  by **regulating the voltage**. Usually, increasing the voltage will lead to increase current, which will lead to an increased magnetic field, which will allow the model to spin faster.
* Problem: a lot of friction and need to be replaced every now and then. 

### Direct Drive (Brushless DC motor)

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/brushless_dc.png" alt= "latex">

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/direct_drive.png" alt= "latex">

* Brushless: less friction and wear
* Hollow shaft
* High torque density
* **Does not require gears**
* Uses hall effect sensor (or something like this) to **measure the position of the coil in real time**, and then **switches the voltage electronically** and thereby creating the switching magnetic field. 
* ***Really popular in robotics!!!***

### Pros & Cons of DC motor

Efficient, but harder to control the exact speed or position

* The most common in robotic systems!

### Determine the position of DC motor

Use Encoder!

The encoder is usually a disc which has a pattern on it and an arrangement of light sensors that can measure when the disc is turning. 


<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/encoder.png" alt= "latex">

* **Incremental Encoder** (left): a disc that has pattern A and pattern B that are shifted slightly with respect to each other, which allows the controller determine the direction of the rotation. 
* **Absolute Encoder** (right): have a binary pattern that encodes an actual number and which allows the controller to read the position at every point in time. In this case, the discretization is 360 degrees divided by eight because we have 8Pi segments


### Gear reduction

Another component needed for dc motor is gear reduction. (because the motor itself rotates at a reallly high speed)

A common typ of gear is - strain wave gear, aka harmonic drive.

* Very efficient
* Very high ratios (ex: 1:50)
* Requires little space
* Not back-drivable


<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/harmonic_drive.png" alt= "latex">

## 3. Stepper motor

Easier to control the exact speed or position! Don't need gearbox and encoder.

* rotor (at the center): a gear with magnetic teeth is driving the spindle
* stator (around): multiple coils able to attract or repel the gear. They are activated one after the other. 

<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/stepping_motor.png" alt= "latex">

Here is a so-called four phase stepper motor, which has **four coils which are activated one after the other, and each time move the gear by a very small but constant increment**. (ex: 1.6 degrees, 2 degrees)

* By counting those increments, you can know exactly where the stepper motor is. 
* the first wire is on for a set amount of time while the other three are off, then the second, the third, and the fourth. 
* Here, **the period (i.e. the length) of this signal determines the stepper motor's speed**


### Pros & Cons of stepper motor

* Can control the exact position/speed better!
* Does not require gear or encoders
* More bulky and expensive.

## 4. Servo motors

* Electric motors usually **require gears, an encoder, and control electronics**. 
* **Modules that package these components (i.e., electric motor + gear + encoder + control electronics) into a convenient form-factor** are known as servo motors.  
* Servo motors have been classically used in remote controlled (RC) cars to provide a simple actuator to steer a car or move the flaps of an airplane. 
* A simple digital signal was used to set the servo angle, usually in the range of 360 degrees or less, which was then held by the integrated electronics. 
* More recently, a new class of digital servo motors has emerged that allows one to not only control the angle, but set the speed at which the servo moves as well as the maximum current (and thereby torque), as well as read information such as actual angle, temperature and other operational parameters.  
* Due to their built-in gear reduction, servo motors are usually not suitable in the drivetrain of mobile platforms, but **have become increasingly prominent to drive the joints of simple manipulating arms, articulated hands and grippers.**
* A special kind of servo motor is the ***linear actuator***. Here, a (brush-less) DC motor is driving a spindle that turns rotation into translation. Linear servo motors are available with a wide range of protocols and with or without built-in encoders that provide position feedback. 

## Summary

* Electric motors (either AC or DC) work together with other components, including gears and encoders.
* Servo motor bundles the whole as a module.
* Stepping motor can control the exact position and speed very well, without the need of gear and encoders.


Application-wise,

* AC motor is mostly seen in heavy machinery.
* Currently, the most common type of motor in robot joints is still DC motor, especially the direct drive (brushless DC motor).
* Servo motors have become increasingly popular for driving simple manipulators, articulated hands, and grippers.
* Stepping motor are more bulky and expensive, and thus less common.