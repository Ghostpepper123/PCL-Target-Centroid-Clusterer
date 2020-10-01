# PCL-Target-Centroid-Clusterer
This repository contains a C++ script named pcd_read.cpp that can take 1 (or 2) input model .pcd point cloud files, search for it in the scene point cloud, and return the centroid point of that model's location within the scene.  There is also a file called pcd_write.cpp that writes a point cloud to a .pcd file, right now it is configured to create a box as a target input model that can be searched for by the algorithm.  

I included two box1.pcd and box2.pcd target files, and a box_scene.pcd scene file.  
By default, the script will search for these two target files inside the box_scene.pcd file and draw a blue and green line to the centroid of each box from the origin. 

This technique can be used for finding the locations of known objects in scanned point clouds for UAVs utilizing LIDAR camera systems, or closer proximity applications such as giving the power of sight to a robotic arm that can grab objects.  

*In order to run the script, you must have Point Cloud Library (PCL) installed. Please refer to http://pointclouds.org/downloads/ to install it. 

If you have any questions about the program, please pull up an issue on here! I will be glad to help out.  

*Note, portions of this code have been extracted from tutorials like https://pcl.readthedocs.io/projects/tutorials/en/latest/correspondence_grouping.html#correspondence-grouping
This tutorial code uses a clustering algorithm to find the location of a target object inside a scene point cloud. 
I used this code as a starting point to then take the dimensions and calculate the centroid of the object that has been found, 
and I had to rearrange the code used in the tutorial to duplicate the process to find the centroid of 2 objects and draw a line to each one.  
