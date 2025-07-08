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