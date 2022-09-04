
# potentialfield_local_planner

- potentialfield_local_planner is a robot local planning using Potential Field under ROS1.

------

## Built with

- ROS Noetic under Ubuntu 20.04 LTS

- ROS Melodic under Ubuntu 18.04 LTS

------


## Getting Started

### Installation

- Download potentialfield_local_planner github link.

``` bash
$ git clone -b diff https://github.com/qaz9517532846/potentialfield_local_planner.git
```

### Run

- potentialfield_local_planner add to move_base.launch file.

``` bash
  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_local_planner" value="potentialfield_local_planner/PotentialFieldLocalPlannerROS" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find zm_robot_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find zm_robot_navigation)/param/move_base_params.yaml" command="load" />
    <rosparam file="$(find potentialfield_local_planner)/param/potentialfield_local_planner_params.yaml" command="load" />
    <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
    <remap from="odom" to="$(arg odom_topic)"/>
  </node>
```

------

## Reference:

[1]. rto_core. https://github.com/dietriro/rto_core

[2].  Potential-Field-Local-Planner. https://github.com/UnalAsil/Potential-Field-Local-Planner

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2022 ZM Robotics Software Laboratory.
