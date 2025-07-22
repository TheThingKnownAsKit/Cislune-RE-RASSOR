# Cislune RE-RASSOR
![](Cislune_Logo.png)

This repository contains the codebase for Cislune's version of the Florida Space Institute's RE-RASSOR project. The codebase is custom, as well as some of the 3D parts. Full credits will be listed in the credits section.

This codebase was developed by a Cislune intern working over the summer of 2025. It was meant to be practice for programming the larger rover, Nostromo, as well as the rover potentially being useful in contracts the company had. A complete list of learning outcomes that will be applied to Nostromo and future projects is listed at the end of the document.

- Software Stack
    - ROS2 (Jazzy)
    - ROS2 Control
    - Gazebo (Harmonic)
    - Nav2
    - SLAM Toolbox
    - Docker (Ubuntu Noble image)
    - PlatformIO
    - Mosh
- Hardware
    - Jetson Orin Nano Developer Kit
    - Teensy 4.1
    - Intel Realsense D435i

[]()

- [Introduction](#introduction)
- [Installation and Setup](#installation-and-setup)
- [Rover Bringup](#rover-bringup)
- [Editing Guide](#editing-guide)
- [About the Project](#about-the-project)
- [Learning Outcomes](#learning-outcomes)
- [Credits](#credits)

## Introduction

**This code was only ever intended to run on Ubuntu based Linux distributions. There is no support for macOS or Windows.** However, because most of this will run in Docker, it is might be possible to translate the Linux setup commands into the corresponding commands for Windows or macOS. There was no way for me to test this, so I did not include it in the instructions.

This project was designed to run primarily in a Docker instance to help with consistency and portability. If you would like to learn more about docker, you can do so at [Docker Overview](https://docs.docker.com/get-started/docker-overview/). Specifically, this project uses Docker Engine with buildx for multi-architecture support. Installation details are listed in Installation and Setup. This docker setup is designed to work on both ARM64,x86-64, Nvidia GPU's, and not Nvidia GPU's.

Additionally, this project was designed with the assumption that the rover is using a Jetson Orin Nano as the onboard rover computer, a Teensy 4.1 as the microcontroller connected to the Jetson over USB, and a groundstation computer that remotely ssh's into the Jetson to run commands and view through Rviz2.

Most of the downloads and installations for necessary libraries will be handled inside the Dockerfile and compose files. There are a few things that need to be downloaded directly on the Jetson and on the groundstation computer. All of this setup has been configured in bash scripts included in the scripts/ folder of the repository, and the Installation and Setup section will instruct you on what scripts to run on what device. After all the installation and setup is done, you should only need to compose and connect the Docker container to develop and run the program. Bringup instructions, including all the different startup options, are explained in Rover Bringup.

This project was specifically designed to be modular. If you would like to edit this codebase for your own purposes and swap out some of the components, the guide to do so is in Editing Guide.

**If you are new to ROS2 projects and want to learn more**, I recommend two sources. Firstly, the beginning documentation for ROS2 is actually really good. It effectively teaches ROS2 concepts, though the quality does decrease as the complexity increases. You can find the ROS2 Jazzy documentation [here](https://docs.ros.org/en/jazzy/index.html). Secondly, I recommend using the official Nav2 documentation for everything else. It teaches concepts such as URDFs, transforms, and Gazebo. The official documentation for Gazebo is not very good and it is difficult to find updated tutorials, but Nav2 does a great job. You can find the official Nav2 documentation [here](https://docs.nav2.org/). [Articulated Robotics](https://articulatedrobotics.xyz/tutorials/) does a really good from scratch walkthrough as well, though it uses an outdated version of Gazebo that is not compatible with ROS2 Jazzy and up. It is difficult to find good tutorials for ROS2 Control that actually implement a Hardware Component, but you can find the official documentation [here](https://control.ros.org/master/doc/getting_started/getting_started.html). It does not have the best instructions so I recommend getting to this part last.

If you're curious about the background of the project, my learning outcomes as a student, or for the specific credits, those are all listed in the bottom sections.

## Installation and Setup

### Prerequisites
**Both the groundstation computer and the Jetson Orin Nano should be running Ubuntu 22.04 (Jammy Jellyfish) as their operating system.** While the Docker instance will be running Ubuntu 24.04 (Noble Numbat), there are compatibility issues between Nvidia's Jetpack SDK for Noble and Docker; the easiest solution is to downgrade the host computers.

Ensure that a recent version of [Git](https://git-scm.com/downloads/linux) and [VSCode](https://code.visualstudio.com/download) are downloaded.

Curl is also required for some of the scripts, ensure it is downloaded.

### Cloning the Repository

In the directory of your choice, either run the command `git clone https://github.com/TheThingKnownAsKit/Cislune-RE-RASSOR.git` to clone via https.

### Setup Scripts

For convenience, bash scripts have been written to do most of the setup for you. **All of these scripts are intended to be ran from the root of the repository,** so from the Cislune-RE-RASSOR folder level.

Before anything else, <u>make sure your system is up to date</u> by running the following commands. Ensure that all packages are upgraded, though if you know for certain that a package will not interact with this repository's tech stack at all, you can ignore it.
```Bash
sudo apt update
sudo apt upgrade
```

**Do not open VSCode before running the host system setup scripts.** This causes user permission issues with VSCode running Docker before the user is properly added to the usergroup. If you are running into permission issues when trying to start docker, first check to see if you are in the docker group by running `groups` in the host computer terminal. If you are and are still experiencing permission issues, close VSCode and run `killall code && newgrp docker` to end all VSCode services and refresh the docker group permissions. Reopen VSCode and it should work.

To setup the host system, you have three options:
1. `./scripts/development_setup.sh`
    
    The development script is intended to give you access to most of the repositories functionality on one non-Jetson computer; it has the intention of making development easier. You will have access to most functionality except actually moving the rover (Gazebo will work).

2. `./scripts/groundstation_setup.sh`

    This is the setup script you should run on your groundstation computer.

3. `./scripts/jetson_setup.sh`

    This is the setup script you should run on your Jetson Orin Nano.

To build and compose the Docker container, run the script `./scripts/docket_setup.sh` with the optional argument flags of `-r -c -v`.
- `-r` will build the image with no cache
- `-c` will recreate the containers even if no changes have been made since last compose
- `-v` will open a VSCode instance attached to the container for you, but you can also just do this through the GUI by navigating to Containers (note you need Container Tools extension installed for this) -> Right click on docker-rerassor_dev -> Attach VSCode

I recommend using the command `./scripts/docker_setup.sh -v` for convenience.

## Rover Bringup

Note that these instructions should be done within Docker instances or colcon will not be able to build and ROS2 will not be installed.

To mosh into the Jetson, use the command `mosh username@ip` from the terminal of the groundstation computer. If you don't know what your ip address is, do `ip a |grep net` and look for the address that ends in /24. TODO

ON THE GROUNDSTATION, from the repository root, run the following commands:
```Bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_bringup groundstation.launch.py
```

ON THE JETSON (via mosh), from the repository root, run the following commands:
```Bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_bringup rover.launch.py
```

If you want to launch the Gazebo simulator, run the following commands:
```Bash
colcon build --symlink-install
source install/setup.bash
ros2 launch rover_sim gazebo.sim.py
```

## Editing Guide

For the most part, the only thing that you would need to edit to make this work for your own rover is edit some of the config files and replace the URDF. There may be some functionality missing that you would have to add in yourself, but otherwise the libraries handle most of the code.

This repository is split into ROS2 packages that are subfolders of rover_ros2. Any folder that begins with rover_ is a ROS2 package with its own package.xml and CMakeLists.txt. 

## Learning Outcomes

Skills learned:
1. Advanced ROS2
2. ROS2 Control
3. Gazebo
4. Nav2
5. SLAM Toolbox
6. Micro ROS (not currently used in project)
7. URDF and Xacro

Lessons learned:
1. 

## Credits

The original concept for the RE-RASSOR was developed by the Florida Space Institute as a modular and accessible solution for lunar rovers. The main page for this project can be found [here](https://floridaspacegrant.org/program/re-rassor/), which will include links to the GitHub Repository and 3D models.

A custom chassis and camera mount were modeled by Cislune for this project, but the rest of the models are from the RE-RASSOR team.

Reference files for the codebase were taken from the ROS2 Control team (credits given at the beginning of those files), and Articulated Robotics was an extremely helpful resource in learning all of these concepts.