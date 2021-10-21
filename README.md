# rqt_network

This is a utility to visualize the state of an entire ROS 2 network in a graphical way.
This utility will show all of the nodes in a graph, all of the topics, services, and actions that connect them, as well as some additional metadata about them.

 # Install
 
 ```bash
 git clone https://github.com/ros2/ros_network_viz.git
 colcon build --packages-up-to ros_network_vis
 . install/setup.bash
 ros2 run ros_network_viz ros_network_viz
 ```

# Features

* [ ] Show a legend/help for describing what everything does
* [x] Allow zooming in or out on the entire scene
* [ ] Have some kind of side panel to show warnings, errors, incompatible QoS, etc.

## Nodes

* [x] Show all nodes in a ROS 2 graph (including those with no topics or services)
* [x] Automatically update the graph as nodes come and go
* [x] Have an option to "pause" automatic graph updates
* [x] Hide hidden nodes by default (with option to turn on)
* [x] Hide rqt_network node by default (with option to turn on)
* [x] Show all parameters and initial parameter values on all nodes
* [x] Update parameters as they change on every node (via /parameter_events)
* [ ] Support parameter updates on nodes that don't have /parameter_events
* [x] Have an automatic layout of nodes that makes sense
* [x] Make the nodes draggable so the user can rearrange the layout
* [x] Show whether a node is a regular Node
* [x] Show whether a node is a LifecycleNode
* [x] Show what Lifecycle state a node is in
* [x] Update Lifecycle states for a node as they change (via /transition_event)
* [ ] Support lifecycle updates on nodes that don't have /transition_events
* [x] Show if a node is a ComponentNode
* [ ] Show the nodes that are inside of a ComponentNode
* [x] Have the ability to select one or more nodes and hide them

## Topics

* [x] Show all of the topics that a node publishes
* [x] Show all of the topics that a node subscribes to
* [x] Draw a line between the published and subscribed topics in the network
* [x] Hide default topics like /parameter_events, /rosout by default (with option to turn on)
* [x] Show the type for each topic
* [x] Show the Quality-of-Service settings for each topic
* [ ] Show the rate (hz) at which a topic is publishing data
* [ ] Show the bandwidth that a topic is using
* [x] Have the ability to hide one or more topics on a node
* [ ] Gracefully handle topics with unknown topic types
* [ ] Show the actual data flowing over a topic

## Services

* [x] Show all of the services that a node provides
* [x] Show all of the services that a node is a client for
* [x] Draw a line between the clients and services in the network
* [x] Hide default services like /describe_parameters, /get_parameter_types, /get_parameters, /list_parameters, /set_parameters, /set_parameters_atomically by default (with option to turn on)
* [x] Show the type for each service
* [ ] Show the Quality-of-Service settings for each service (does this even make sense for a service?)
* [x] Have the ability to hide one or more services on a node
* [ ] Gracefully handle services with unknown topic types

## Actions

* [x] Show all of the actions that a node provides
* [x] Show all of the actions that a node is a client for
* [x] Draw a line between the action clients and action services in the network
* [x] Show the type for each action
* [ ] Show the Quality-of-Service settings for each action (does this even make sense for an action?)
* [x] Have the ability to hide one or more actions on a node
* [ ] Gracefully handle actions with unknown topic types
