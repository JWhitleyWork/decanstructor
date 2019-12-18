# DeCANstructor #

The definitive CAN analysis tool for ROS. Allows rapid prototyping, reverse-engineering, and monitoring of CAN-based systems.

## Usage ##

By default, the node subscribes to the `can_in` topic with type can_msgs/Frame.

### Playback Mode ###

For playback mode, set the `~playback` parameter to true. If using the launch file, `use_sim_time` is automatically set to true. If not using the launch file, make sure to set this parameter yourself to get accurate change highlighting. A rosbag must be playing with the --clock option for playback mode to work properly. When playback mode is disabled, you can emit events with the "Publish Event" button. While in plaback mode, recorded events will flash an "Event Published" message in the bottom-right of the window.

## Screenshots ##

![Screenshot 1](/screenshots/decanstructor_2.png?raw=true "screenshot 1")
