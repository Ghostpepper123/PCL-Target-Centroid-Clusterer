# PCL-Target-Centroid-Clusterer
This repository contains files of a C++ script that can take 1 (or 2) input model .pcd point cloud files, search for it in the scene point cloud, and return the centroid point of that model's location. 

The C++ script is pcd_read.cpp. I included a box1.pcd and box2.pcd target files, and a box_scene.pcd scene file.  
By default, the script will search for these two target files inside the box_scene.pcd file and draw a blue and green line to the centroid of each box from the origin. 

This technique can be used for finding the locations of known objects in scanned point clouds for essentially any application.  
*In order to run the script, you must have Point Cloud Library (PCL) installed. Please refer to http://pointclouds.org/downloads/ to install it. 

If you have any questions about the program, please pull up an issue on here! I will be glad to help out.  

*Note, portions of this code have been extracted from tutorials like http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
This tutorial code uses a clustering algorithm to find the location of a target object inside a scene point cloud. 
I used this code as a starting point to then take the dimensions and calculate the centroid of the object that has been found, 
and I had to rearrange the code used in the tutorial to duplicate the process to find the centroid of 2 objects and draw a line to each one.  
