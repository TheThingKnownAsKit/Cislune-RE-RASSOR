TODO:
1. Fix the launch files. All gazebo stuff should be in sim, all ROS2 Control in control, etc. Make it modular
    Done

2. Make groundstation and jetson setup scripts for things Docker can't do automatically
3. Make full setup scripts that do literally everything from computer setup to docker setup to launching program
4. Make subpackage readme's
5. Add support for remotely uploading to Teensy
6. Support all needed commands in rover_bringup
7. Fix the weird wheel drift
8. Write the xml for the control plugins
9. Write the task execution for waypoints

Lessons learned:
1. Don't name every ros2 package prepended with rover_ because it makes autocomplete hate you
2. I have no idea if it's best practice but I do really like separating the launch files into modules within
    their respective ROS2 

Okay actually pretty README starts now

# Cislune RE-RASSOR
![](Cislune_Logo.png)

This repository contains the codebase for Cislune's version of the Florida Space Institute's RE-RASSOR project. The codebase is custom, as well as some of the 3D parts. Full credits will be listed in the credits section.

Table of Contents:
- [Getting Started](#getting-started)
- [Installation and Setup](#installation-and-setup)
- [Rover Bringup](#rover-bringup)
- [Editing Guide](#editing-guide)
- [About the Project](#about-the-project)
- [Learning Outcomes](#learning-outcomes)
- [Credits](#credits)

## Getting Started

## Installation and Setup

## Rover Bringup

## Editing Guide

## About the Project

## Learning Outcomes

## Credits