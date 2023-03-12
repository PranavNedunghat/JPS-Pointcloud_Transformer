# JPS-Pointcloud_Transformer
Contains the code for the 3D JPS algorithm and the code to transform pointcloud data into an occupancy grid map
Run the drone simulator and go to a location (I usually choose x = 5.0, y = 5.0, z = 1.0, because the point cloud information in the bag file has a pointcloud obstacle at the initial location of the robot, and my pointcloud Transformer node won't publish to the JPS node until the starting point is an unoccupied cell). Then run the bag file 2023-02-02-17-41-46.bag which contains the pointcloud information. Only then run the pointcloud.py, else you will get errors. This will launch a node that publishes the start, goal and occupancy grid map data to the JPS node in the drone's frame of reference. If the drone (start position) or the destination is occupied, it will not publish and you will need to move the drone to another (unoccupied) location. You don't need to worry about the destination because if the destination is occupied, the code will try to find the nearest unoccupied cell and select that as the destination. Additionally, the code is slow to update the drone's current position hence it is best to rerun the pointcloud.py file to see if it starts publishing. When both the drone's position and destination are unoccupied, the node will start publishing data to the JPS node. You can see if it is publishing as it will send a message to the terminal showing the destination coordinates in the drone's frame of reference. you can run the JPS node by using the command 'rosrun JPS Jump_Point_Search'. Additionally, I have created a launch file that launches both the pointcloud.py node and the JPS node. it is located in JPS/launch. Finally, if the JPS algorithm has found a suitable path, it will send a message to the terminal stating that it is publishing. Finally, You can go the rviz window and add a Marker, and subscribe to the /Visualize topic. You will see the obstacles in red, the path as a blue line, and the points in green. All of these are in the world frame.
pointcloud.py is located in JPS/scripts. Jump_Point_Search.cpp is located in JPS/src.
