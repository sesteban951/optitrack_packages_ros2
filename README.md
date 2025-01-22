# Optitrack Packages for ROS2
The packages have been tested on **Ubuntu 22.04**, **ROS2 Humble**, **NatNetSDK 4.0**, and **Motive 2.3**.
To get started you can skip to [Getting Started](#Getting-Started). This repo contains the following ROS2 packages for interfacing with optitrack Motive 3.0:
* [optitrack_wrapper_ros2](#optitrack_wrapper_ros2): a ROS2 package that wraps the optitrack NatNetSDK and converts SDK commands to ROS2 services, and frame data coming from the SDK to ROS2 messages that are published on a topic.
* [optitrack_wrapper_ros2_msgs](#optitrack_wrapper_ros2_msgs): a ROS2 package that contains the services and messages required for `optitrack_wrapper_ros2`.
* [optitrack_multiplexer_ros2](#optitrack_multiplexer_ros2): a ROS2 package that publishes on separate topics the rigid body, skeleton and unlabeled markers data.
* [optitrack_multiplexer_ros2_msgs](#optitrack_multiplexer_ros2_msgs): a ROS2 package that contains the messages required for `optitrack_multiplexer_ros2`.

At the end of this documentation you can find:
* [General Comments](#General-Comments): general comments about using the Motive software with 
* [Potential Improvements](#Potential-Improvements): improvements to the packages that would make for a safer code but are not necessarily for the functioning of the packages.

# Getting Started
## Create and build workspace
Create a ROS2 workspace and clone the repo inside the `src` folder of the workspace (or simply clone it inside an existing workspace), then build it: 
``` shell script
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:lis-epfl/optitrack_pkgs_ros2.git
cd ..
colcon build --symlink-install --packages-select optitrack_wrapper_ros2 optitrack_wrapper_ros2_msgs optitrack_multiplexer_ros2 optitrack_multiplexer_ros2_msgs
```

## Change the config parameters
Modify the config file `optitrack_wrapper_config.yaml` in `optitrack_wrapper_ros2/config` and change the values to the ones appearing in the data streaming pane in the Motive software (make sure to click the three dots then **show advanced** for Motive to dispay all the data). Also make sure to enable streaming the rigid bodies/skeletons/asset markers/labeled markers/unlabled markers in the data streaming pane.


Modify the config file `optitrack_multiplexer_config.yaml` in `optitrack_multiplexer_ros2/config` to specify the rigid bodies/skeletons you want to publish to a separate topic along other parameters.

## Launch the wrapper
In a terminal pane, source the environment and launch the wrapper:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch optitrack_wrapper_ros2 optitrack_wrapper.launch.py
```

## Launch the multiplexer
In an another terminal pane, source the environment and launch the multiplexer to publish your rigid bodies/skeletons/unlabeled markers on separate topics:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch optitrack_multiplexer_ros2 optitrack_multiplexer.launch.py
```

The total pipeline latency that is added by the wrapper + multiplexer is between 0.5 and 1 milliseconds.

## Launch both the wrapper and the multiplexer with the same launch file
To launch both the wrapper and the mulitplexer with the same launch file run:
``` shell script
cd ~/ros2_ws
. install/setup.bash
ros2 launch optitrack_multiplexer_ros2 wrapper_and_multiplexer.launch.py
```

# optitrack_wrapper_ros2
This package wraps the NatNetSDK and transforms commands sent from the SDK to the Motive software into ROS2 services. It also transforms the streamed data into a ROS2 message that is published. All the data is kept as is and no seleciton/multiplexing is done.

The reason we wrote this package, is that the NatNetSDK can be tedious to work with natively, so we presented a cleaner interface in ROS2 that allows for seemless interaction with the Motive software through NatNetSDK. Some of the issues we found with NatNetSDK:
* No use of namespaces
* Naming conventions are mixed
* If you add or modify the name of a rigid body in the Motive software, the boolean variable that indicates that the model list has changed does not change. We added an explicit mechanism to check for these cases and indicate that the model list has changed.
* The marker names in the `sRigidBodyDescription` struct (`szMarkerNames`) were always empty during testing.

So far only the command to get the data descriptions has been implemented as a ROS2 service, but one can add more as per his/her needs (use the implemented service as an example).

# optitrack_wrapper_ros2_msgs
Contains the services and messages used in `optitrack_wrapper_ros2`. Each data frame message includes the total latency up to the point of publishing it divided into the different steps of the pipeline.

# optitrack_multiplexer_node
This package is used to select the information from the data frame that is published by `optitrack_wrapper_ros2` on a ROS2 topic. In the config file you can specify:
* `rigid_body_names`: the names of the rigid bodies you want to stream seprated by a comma. Each rigid body data is streamed on a different topic. The names in Motive should not have spaces in them but rather separated with underscores or camelCase.
* `skeleton_names`: the names of the skeletons you want to stream. Each skeleton data is streamed on a different topic. The names in Motive should not have spaces in them.
* `publish_unlabeled_markers`: if true, it will publish all the unlabeled markers on a single topic i.e. a vector that contains the markers' position and IDs.

# optitrack_multiplexer_ros2_msgs
Contains the messages used in `optitrack_multiplexer_ros2`. Each message contains the total pipeline latency up to the point in time just before publishing the message. Unlike `optitrack_wrapper_ros2_msgs`, we only send to total pipeleine latency and not how it is divided along the steps of the pipeline. 

# General Comments
If you have **Motive 3.0** you get the option to select what information the Motive software broadcasts which reduces latency. This can be added as a service in `optitrack_wrapper_ros2`. 

The `mean_error` value of a rigid body is the difference between its model reconstructed markers and the actual measured markers. The residual for each marker is the average minimum distance between its triangulation and the projection rays of the camera (in mm/ray).

Leaving the `local_server` variable in `optitrack_wrapper_config.yaml` empty should work fine. However if you wanna put something more precise, you can run `ifconfig` on ubuntu to find your local adress (it is usually the inet that starts with 192.168...)

The `marker_set_data` field in the `FrameOfMocapData.msg` file in `optitrack_wrapper_ros2_msgs` includes all the assets that are being streamed. Each asset has its own markers that are model filled in case they are not visible by the cameras. It includes an “all” asset that includes the positions of all the markers of all the assets in order (not visible (so markers coordinates = 0), point cloud solved and model filled if occluded).


The `other_markers` field in the `FrameOfMocapData.msg` file in `optitrack_wrapper_ros2_msgs` is a subset of the “all” asset markers that includes only the markers that are visible and pointcloud solved (not model filled).

The rigid bodies includes all the selected assets that are being streamed. All assets have a pose and a mean error if they are tracked (indicated as well in the `tracking_valid` variable of the rigid body).

The labeled markers include only all the visible markers in the optitrack room (defined or undefined). If the marker belongs to a marker set, it is indicated with the model ID field, and the marker id indicates the index of the marker set’s markers vector that give us that marker’s position/name.

Using a single marker, there is no way in current optitrack (3.0) to label and track a single passive marker, so it has to be done algorithmically. The single unlabeled markers can change IDs any time the trajectory is broken. A given application needs to subscribe to the `unlabeled_markers` topic of the multiplexer and check which marker is the closest to which marker at the previous time/measurement.

The device and forceplate parts are implemented only in the optitrack wrapper but have not been tested.

The way we detect unlabeled markers is if the model id is either 0 or 1 which may not be the best way to do it. 

The mean error for skeletons does not work correctly (not sure if due to optitrack or implementation because it works correctly for rigid bodies using the same implementation).

# Potential Improvements
These are potential improvements of the current packages:
* Add support for names that include spaces in them in `optitrack_multiplexer_ros2`.
* Add `const` to arguments that are not allowed to change in class methods as well as to methods that are not allowed to change class attributes (all packages).
* Test the device and forceplate parts.
* More robust way (but more computationally expensive) to detect the unlabeled markers.
