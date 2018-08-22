# DogBot API Reference

## Summary

This API reference includes generated help files from the API source code

The DogBot API provides a rich interface, to drive DogBot at the level of individual servos, leg positions, or encapsulated walking gaits, as well as general monitoring and administration functions.

Currently the API includes the following capabilities, among others:
* Connect to a DogBot, query joint and leg positions, velocities, torques.
* Access voltage, temperature and other readings.
* Home motors, put motors into on/off/brake/hold modes, emergency stop.
* Control individual servos (optionally including virtual knee joints):
   * move to specified locations;
   * send a series of points along a trajectory;
   * directly control servo torque.
* Control legs:
   * move foot to a position in leg co-ordinate frame;
   * set array of joint angles;
   * compute leg force and position estimates.
* Control overall DogBot pose:
   * send single or series of trajectory values for the pose;
   * compute joint angles from pose.
* Control walking/trotting gaits.

## Using the API

The API is accessed using C++ or Python.  Note that not all methods will be available via the Python implementation.

The API should be used in conjunction with the DogBot user interface, as this contains safety features such as temperature warnings and an emergency stop button.

This document repository does not cover driving DogBot via ROS, which is abstracted across ROS controller topics.
