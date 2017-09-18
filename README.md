# teraranger_description
# ROS package for TeraRanger array solutions by Terabee

This package is a collection of URDF files of teraranger products:
* [TeraRanger Tower](http://www.teraranger.com/teraranger-tower/)

## Building and running the package from source

To clone and build the package in your workspace follow these steps:

* If you have ssh key setup for your github account:

```
cd ~/ros_ws/src
git clone git@github.com:Terabee/teraranger_description.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

* If you prefer to use https use this set of commands:

```
cd ~/ros_ws/src
git clone https://github.com/Terabee/teraranger_description.git
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## Using teraranger_description

To add easily a tower to your robot description, first include the xacro (xml macro) file :
 ```
 <xacro:include filename="$(find teraranger_description)/urdf/teraranger_tower.urdf.xacro"/>
 ```
 Then you will be able to call the xacro from your main URDF file:
 ```
 <teraranger_tower multi_hub="false" hub_id='0' parent="base_link" x="0" y="0" z="0.5" roll="0" pitch="0" yaw="0" />
 ```
 
 If you want a custom setup, just include the base_hub and teraranger_one xacro files and build your own setup:
 ```
 <base_hub multi_hub="${multi_hub}" hub_id='${hub_id}' parent="${parent}" x="${x}" y="${y}" z="${z}" roll="${roll}" pitch="${pitch}" yaw="${yaw}" />
  <teraranger_one multi_hub="${multi_hub}" hub_id='${hub_id}' id="0" x="0.060" y="0.000" z="0.0" roll="0.0" pitch="0.0" yaw="0.000" gaussian_noise="0.06"/>
 ```
 
 
```
<base_hub multi_hub="${multi_hub}" hub_id='${hub_id}' parent="${parent}" x="${x}" y="${y}" z="${z}" roll="${roll}" pitch="${pitch}" yaw="${yaw}" />
 <teraranger_one multi_hub="${multi_hub}" hub_id='${hub_id}' id="0" x="0.060" y="0.000" z="0.0" roll="0.0" pitch="0.0" yaw="0.000" gaussian_noise="0.06"/>
 etc.
```
INFOS: By settings the multihub parameter to _true_ or _1_ you will enable auto-namespacing of the frame_id with the following convention:
* sensor frame = _hub_${hub_id}_base_range_${id}_
* hub frame = base_hub_${hub_id}

___To enable compatibility The ROS driver will append its namespace to the frame_if inside the RangeArray and Range message, thus it is strongly recommended to use hub_${hub_id} as node namespace for the driver.___


