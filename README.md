# Triplet Graph
The triplet graph package offers an implementation of the triplet graph as described in Rokus Ottervanger's master thesis A Method for Graph Based, Task Focused Localization for Mobile Robots.

## Dependencies
Dependencies are:
* ROS Indigo, including
 * tf
 * RViz
* [geolib2](https://github.com/tue-robotics/geolib2)
* [tue_config](https://github.com/tue-robotics/tue_config)

## Getting started
To use the triplet graph software, place this package in a catkin workspace and compile using catkin_make. This will produce a library that can be linked to, but also two executables: one for mapping and one for localization, and an rviz-plugin for interaction with the graph.

### Configuration files
Before we are going to create a graph using one of the executables, `cd` to  `<your/catkin/workspace>/triplet_graph/config/`. You'll find yaml configuration files for localization and mapping. These are quite easy to understand as you find out when you inspect them. Each of the components 
* Corner detector,
* Odometry tracker,
* Association algorithm and
* Graph visualization
has its own configuration parameters, and there are some general parameters such as 
* the filename to find the existing graph or store a new or extended graph (in case of mapping), 
* the task's target node and 
* the initial pose of the robot.

Note that the initial pose of the robot is not actually a pose. Rather, it specifies where at least two nodes can be found with respect to the laser range finder sensor, and the uncertainty in these positions with respect to the robot (a gaussian over the absolute distance from the expected position). The association algorithm will use these points to estimate the positions of all other nodes in the graph and associate measured points with them. 

### Mapping
Now that we know what the configuration parameters are, we can run some software. The main localization algorithm can only be used with a given graph description of the environment (the filename of which is specified in the config file for localization). Such a graph can be made using the mapping algorithm and preferably a simulator. A robot with good odometry and sensor data may perform well enough. So let's create a graph!

Make sure all topics and tf frames are set correctly in the mapping.yaml config file. Now run your robot simulator and make sure that there are corners that can be detected from the produced sensor data. When that is done, execute
    rosrun triplet_graph mapping <path/to/mapping.yaml>
This will start the mapping algorithm, taking the corner points measured in the first measurement to relate points measured later to. The constructed graph can be visualised in RViz. To do this, add a MarkerArray to the visualization and select the mapping visualization topic. 

You can now drive the robot around, for example using tele-operation. When the robot sees new points together with enough old points, it will extend the graph with nodes representing these new points. Note that once a node is added to the graph, it will not be removed or merged with another node if it later appears to be the same point as measured before. This means that you have to be careful to not let the robot add false nodes. Also, lengths of edges that are added to the graph once, will not be updated again.

In the current version of the software, the initial pose is not stored. So before stopping the mapping routine, be sure to write down the positions of nodes relative to the robot's starting pose. These can then be used as the 'initial pose' in the configuration file of the localization routine (which is mandatory).

### Localization
Now that we have a graph, we can localize in it! To do that, make sure that the localization configuration file is in order: make sure the program will subscribe to the right topics, listen to the correct tf transformations, read the graph from the correct file and use the correct initial pose (the one you wrote down in the mapping step). Now the localization routine can be started, similar to the mapping program:
    rosrun triplet_graph localization <path/to/localization.yaml>

Again, in RViz, you can visualize what is happening using the MarkerArray visualization, but now subscribe to the localization visualization topic.

### RViz Tool
There is a simple RViz tool available for the triplet graph. This lets the user select nodes and set length uncertainty in the edges between them to a small value (this makes the structure of the selected nodes more rigid). To do this, make sure you only visualize the node markers (not the edges, points, or labels); select the nodes of a rigid object and press enter. The terminal in which you opened RViz will show you a little feedback. Note that is currently *not* possible to undo this action. 

## TODO:
The RViz tool currently also has functionality for *merging or deleting nodes*, but the graph does not handle these requests as deleting nodes is a rather tricky operation. The current implementation of the function to delete nodes results in segfaults. Would be nice if this could be fixed.

The *corner detector* needs some review. There is a bug in the detection of corners at jumps in the range data (it sometimes returns corners on the wrong side of the jump). But the entire algorithm does not handle real sensor data very well. A copletely different approach to detecting corners in the sensor data may show better results.