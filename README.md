ros_micro_machines
=======

[![Build Status](https://travis-ci.org/arnaud-ramey/ros_micro_machines.svg)](https://travis-ci.org/arnaud-ramey/ros_micro_machines)

A clone of MicroMachines to use within ROS.

Licence
=======

BSD


Authors
=======

  - Package maintainer: Arnaud Ramey (arnaud.a.ramey@gmail.com)

Compile and install
===================

ROS Kinetic + catkin
-------------------

Compile with [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make):

```bash
$ rosdep install ros_micro_machines --ignore-src
$ catkin_make --only-pkg-with-deps ros_micro_machines
```

Run
===

```bash
$ rosrun ros_micro_machines ros_micro_machines.exe
```

Parameters
==========

 * ```~ivona_credentials``` [std_msgs/String]
  a text file containing two lines,
  the first being the access key, the second the secret key.

Subscriptions
=============

 * ```/tts``` [std_msgs/String]
 Sentences to be said.

Publications
============

None.
