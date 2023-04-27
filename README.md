# Description

For 2023 the logic that will be used for the autonomous navigation mission of the URC is described by the following flow chart: 

![flow_chart](./diagrams/nav_flow_chart.jpeg "flow_chart")

The process described on this flowchart will be achieved using the following ROS nodes and topics:

![node_diagram](./diagrams/navigation_nodes.jpeg "node_diagram")

All nodes are located on the scripts folder and have the following correspondance

| name of the node on the diagram | name of the script        |
|---------------------------------|---------------------------|
| Pose_Publisher                  | odom_publisher.py         |
| navigation_controller           | NavigationController.py   |
| centr & Approach                | center_and_approach.py    |
| rotate_while_detecting_AR       | rotateWhileDetectingAr.py |
| follow_Gps2                     | follow_gps3.py            |

The nodes that are in the diagram and are not listed on this table are in a different package, please see the other quantum robotics repos for reference

# How to simulate navegation logic on q-mars LIGHT sim

1. Clone the [RoverSim repo](https://github.com/QuantumRoboticsURC/RoverSim) and follow the instructions stated on its readme file to run simulation alone
2. To ensure the simulation is working correctly try to control the robot frame by publishing cmd_vel commands or using [teleop_twist_keyboard package](http://wiki.ros.org/teleop_twist_keyboard)
3. If simulation is working, restart the simulation 
4. run the command `roslaunch qr_navigation autonomous_puzzlebot_sim.launch`
5. run navigation controller node using `rosrun qr_navigation NavigationController.py` 
6. Navigation controller will prompt you for goal coordinates, please consider that GNSS cordinates for x=0.0 and y=0.0 in Rviz are (19.594558,-99.228084) as stated on the file [./scripts/OdomPublisherPuzzlebotSim2.py](./scripts/OdomPublisherPuzzlebotSim2.py) and all other files using the function `xy2ll()` function of the [./scripts/gps_tranforms/alvinxy.py](./scripts/gps_tranforms/alvinxy.py) file. <br><br>
For testing you can use the GNSS coordinates found on [joses_tests.csv](./scripts/csv_files/joses_tests.csv)

1. then the Navigation controller will prompt you for the type of GNSS target, write "0" and press enter for gps_only, "1" and enter for aruco_post or "2" and enter for aruco gate.   