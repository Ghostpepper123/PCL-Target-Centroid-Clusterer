#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>


typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_1 = "box1.pcd";
std::string model_filename_2 = "box2.pcd";
std::string scene_filename_ = "box_scene.pcd";
int modelNum = 0;

//Algorithm params
bool show_keypoints_ (false);//blue points
bool show_correspondences_ (true);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);//was true
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

float centerPointArray[3];

//1
//These are the global variables for the first box model (the tall one)
pcl::PointCloud<PointType>::Ptr rotated_model1(new pcl::PointCloud<PointType>());
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations1;

pcl::PointCloud<PointType>::Ptr model1(new pcl::PointCloud<PointType>());  
pcl::PointCloud<PointType>::Ptr model_keypoints1(new pcl::PointCloud<PointType>());

std::vector<pcl::Correspondences> clustered_corrs1;

//2
//These are the global variables for the first box model (the short one)
pcl::PointCloud<PointType>::Ptr rotated_model2(new pcl::PointCloud<PointType>());
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations2;

pcl::PointCloud<PointType>::Ptr model2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr model_keypoints2(new pcl::PointCloud<PointType>());

std::vector<pcl::Correspondences> clustered_corrs2;

//Scene
pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

//viewer
pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

void calcMinMax(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ){
  
  float minX, minY, minZ, maxX, maxY, maxZ;
  int totalSize = 0;
  
  //find min's and max's for each dimension of the cloud
  for (int i = 0; i < cloud->points.size(); ++i){
    if(i==0){
      
      minX = cloud->points[i].x;
      minY = cloud->points[i].y; 
      minZ = cloud->points[i].z;

      maxX = cloud->points[i].x;
      maxY = cloud->points[i].y; 
      maxZ = cloud->points[i].z;

    }
    //check for min's
    if(cloud->points[i].x < minX){
      //std::cout << "CHANGING minX" << std::endl;
      minX = cloud->points[i].x;
    } 
    if(cloud->points[i].y < minY){
      //std::cout << "CHANGING minY" << std::endl;
      minY = cloud->points[i].y;
    }
    if(cloud->points[i].z < minZ){
      //std::cout << "CHANGING minZ" << std::endl;
      minZ = cloud->points[i].z;
    }
    //check for maximums
    if(cloud->points[i].x > maxX){
      maxX = cloud->points[i].x;
    } 
    if(cloud->points[i].y > maxY){
      maxY = cloud->points[i].y;
    }
    if(cloud->points[i].z > maxZ){
      maxZ = cloud->points[i].z;
    }

    totalSize+=1;
  }//END FOR LOOP
  
  //after the for loop is ended, print out the number of points traversed and their min's and max's
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from the file with the above fields: "
            << std::endl;
  std::cout << " Min X: " << minX
            << " Min Y: " << minY
            << " Min Z: " << minZ
            << " Max X: " << maxX
            << " Max Y: " << maxY
            << " Max Z: " << maxZ
            << " total size: " << totalSize
            << std::endl;
  
  //print out dimensions of object
  std::cout << "OBJECT DIMENSIONS:" << std::endl;
  float xlength = maxX - minX;
  float ylength = maxY - minY;
  float zlength = maxZ - minZ;
  std::cout << " xlength: " << xlength << 
               " ylength: " << ylength << 
               " zlength: " << zlength << std::endl;
  centerPointArray[0] = minX + ( (maxX-minX) / 2);//without the / 2 this should be maxX
  centerPointArray[1] = minY + ( (maxY-minY) / 2);
  centerPointArray[2] = minZ + ( (maxZ-minZ) / 2);
  std::cout << "PRINTING CENTER POINT" << std::endl;
  std::cout << centerPointArray[0] << std::endl;
  std::cout << centerPointArray[1] << std::endl;
  std::cout << centerPointArray[2] << std::endl;

  //return centerPointArray;

}//end calcMinMax


//computeCloudResolution
double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


int correspondenceGrouping(pcl::PointCloud<PointType>::Ptr model, int modelNum){

  //these 3 objects are used with the model argument
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());

  if(modelNum == 1){

  std::cout << "LOADING MODEL FILE 1" << std::endl;
  if (pcl::io::loadPCDFile (model_filename_1, *model) == -1){ //load model_filename_1 into cloud model
    std::cout << "Couldn't read model cloud file 1!" << std::endl;
    return (-1);
  }

  }else if (modelNum == 2){

  std::cout << "LOADING MODEL FILE" << std::endl;
  if (pcl::io::loadPCDFile (model_filename_2, *model) == -1){ //load model_filename_2 into cloud model
    std::cout << "Couldn't read model cloud file 2!" << std::endl;
    return (-1);
  }

  }

  std::cout << "LOADING SCENE FILE" << std::endl;
  if (pcl::io::loadPCDFile (scene_filename_, *scene) == -1){ //load the scene file
    std::cout << "Couldn't read scene cloud file!" << std::endl;
    return (-1);
  }

  //compute cloud resolution (resolution invariance)
  //This block of code is disabled because use_cloud_resolution_ is false as far as I am using this script.  
  /*if (use_cloud_resolution_)
  {
    std::cout << "use_cloud_resoltion enabled, computing invariance" << std::endl;
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }*/

  //
  //  Compute Normals
  //
  
  std::cout << "COMPUTING NORMALS" << std::endl;
  std::cout << "defining norm_est" << std::endl;
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;

  std::cout << "norm_est.setKSearch(10)" << std::endl;
  norm_est.setKSearch (10);

  std::cout << "norm_est.setInputCloud (model)" << std::endl;
  norm_est.setInputCloud (model);

  std::cout << "norm_est.compute (*model_normals)" << std::endl;
  norm_est.compute (*model_normals);

  std::cout << "norm_est.setInputCloud (scene)" << std::endl;
  norm_est.setInputCloud (scene);

  std::cout << "norm_est.compute (*scene_normals)" << std::endl;
  norm_est.compute (*scene_normals);///////
  

  //
  //  Downsample Clouds to Extract keypoints
  //
  std::cout << "DOWNSAMPLING CLOUDS TO EXTRACT KEYPOINTS" << std::endl;
  pcl::UniformSampling<PointType> uniform_sampling;

  std::cout << "uniform_sampling.setInputCloud (model)" << std::endl;
  uniform_sampling.setInputCloud (model);

  std::cout << "uniform_sampling.setRadiusSearch (model_ss_)" << std::endl;
  uniform_sampling.setRadiusSearch (model_ss_);

  std::cout << "uniform_sampling.filter (*model_keypoints);" << std::endl;
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  
  std::cout << "building rototranslations" << std::endl;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::cout << "building clustered_corrs" << std::endl;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    std::cout << "use_hough_ is true, computing with hough. put more flags here incase this is true and slow!" << std::endl;
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
    //calcMinMax(model_scene_corrs);
  }
  /*else // Using GeometricConsistency
  {
    //I was using Hough when I was using this code, and it worked, so I temporarily disabled GeometricConsistency
    std::cout << "use_hough_ is not true, computing with the regular algorithm" << std::endl;
    std::cout << "building gc_clusterer" << std::endl;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;

    std::cout << "setGCSize(cg_size_)" << std::endl;
    gc_clusterer.setGCSize (cg_size_);

    std::cout << "setGCThreshold (cg_thresh_)" << std::endl;
    gc_clusterer.setGCThreshold (cg_thresh_);

    std::cout << "setInputCloud (model_keypoints)" << std::endl;
    gc_clusterer.setInputCloud (model_keypoints);

    std::cout << "setSceneCloud (scene_keypoints)" << std::endl;
    gc_clusterer.setSceneCloud (scene_keypoints);

    std::cout << "setModelSceneCorrespondences (model_scene_corrs)" << std::endl;
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    std::cout << "recognize (rototranslations, clustered_corrs)" << std::endl;
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }*/

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;

  /*
  //This for loop prints out the rotation matrix. I didn't find it usable so I commented it out
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }*/
  if(modelNum == 1){

    model1 = model;
    rototranslations1 = rototranslations;
    clustered_corrs1 = clustered_corrs;

  }else if (modelNum == 2){

    model2 = model;
    rototranslations2 = rototranslations;
    clustered_corrs2 = clustered_corrs;

  }
  
  //
  //  Visualization
  //
  /*
  std::cout << "STARTING VIEWER" << std::endl;
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

  std::cout << "ADDING scene_cloud TO VIEWER" << std::endl;
  //add scene cloud
  viewer.addPointCloud (scene, "scene_cloud");
  ///*****axes
  viewer.addCoordinateSystem (1.0);//add x,y,z axes

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    //add yellow model cloud
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    //pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());//red cloud
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    //this is the determining of the color (red) and actual adding of the point cloud 
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());
    
    if (show_correspondences_)
    {
      //std::stringstream ss_line;
      
      //pcl::ConstCloudIterator< PointT >::ConstCloudIterator	(	const PointCloud< PointT > & 	cloud	)	
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      //pcl::PointCloud<pcl::PointXYZRGB> cloudTarget;
      //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      ///pcl::ConstCloudIterator::Ptr cloud_iterator (new pcl::ConstCloudIterator(rotated_model));
      //std::cout << "rotatedmodelsize: "+rotated_model->points.size() << std::endl;
      //std::string addLineName;
      int start = 0;
      int max = rotated_model->points.size();
      int numPointsMatching = 0;
      int centerPointSlot = 0;
      int rotated_model_size = 0;
      for(int j = 0; j < rotated_model->points.size(); j++){
        rotated_model_size++;
      }
      std::cout << "rotatedmodelsize: " << rotated_model_size << std::endl;
      

      std::stringstream ss_line;
      calcMinMax(rotated_model);
      //add 2 slots to rotated_model array
      rotated_model->points.resize ((rotated_model_size+2) * 1);
      //define the first new slot to be at the center
      rotated_model->points[rotated_model_size+1].x = centerPointArray[0];
      rotated_model->points[rotated_model_size+1].y = centerPointArray[1];
      rotated_model->points[rotated_model_size+1].z = centerPointArray[2];
      //define the second new slot to be at the origin
      rotated_model->points[rotated_model_size+2].x = 0.0;
      rotated_model->points[rotated_model_size+2].y = 0.0;
      rotated_model->points[rotated_model_size+2].z = 0.0;
      //we aren't writing points to the .pcd file, just defining locations to draw a line to that are stored in the array
      PointType& origin_point = rotated_model->points[rotated_model_size+2];
      PointType& center_point = rotated_model->points[rotated_model_size+1];
      //calcMinMax(rotated_model);//********pcl::PointCloud<pcl::PointXYZRGB> cloudTarget;

      viewer.addLine<PointType, PointType> (origin_point, center_point, 0, 255, 255, ss_line.str ());

      //cloudTarget.points[cloudIndex].x = currentX
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);
        
        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }//for (std::size_t i = 0; i < rototranslations.size (); ++i)

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }*/
  return 0;
}////end correspondenceGrouping

//visualization function
//pcl::visualization::PCLVisualizer::Ptr
/*
int displayScene(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  std::cout << "INITIALIZING VIEWER" << std::endl;
  //pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  std::cout << "ADDING scene_cloud TO VIEWER" << std::endl;

  //add scene cloud
  //viewer.addPointCloud (scene, "scene_cloud");

  ///*****axes
  viewer.addCoordinateSystem (1.0);//add x,y,z axes

  pcl::PointCloud<PointType>::Ptr off_scene_model1 (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  /*
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  //viewer->addCoordinateSystem (1.0);//add x,y,z axes
  viewer->initCameraParameters ();
  
  //draw lines
  //draw vertical lines
  viewer->addLine<pcl::PointXYZ> (cloud->points[0], cloud->points[2499], "line1");
  viewer->addLine<pcl::PointXYZ> (cloud->points[2500], cloud->points[4999], "line2");
  

  while (!viewer.wasStopped ()){
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
}*/


int main (int argc, char** argv){

  /*
  pcl::PointCloud<PointType>::Ptr off_scene_model1 (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());
  
  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model1, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    //add yellow model cloud
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }*/

  //define PointCloud as cloud
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);



  //
  //  Visualization
  //

  //add scene cloud
  //viewer.addPointCloud (scene, "scene_cloud");
  viewer.addCoordinateSystem (1.0);//add x,y,z axes

  
  //std::cout << "CALLING correspondenceGrouping (1 time)" << std::endl;
  //std::cout << "model_filename_1: " << model_filename_1 << std::endl;
  modelNum = 1;
  correspondenceGrouping(model1, modelNum);
  
  //std::cout << "CALLING correspondenceGrouping (2 time)" << std::endl;
  modelNum = 2;
  correspondenceGrouping(model2, modelNum);
  //std::cout << "rototranslations1.size(): " << rototranslations1.size() << std::endl;
  for (std::size_t i = 0; i < rototranslations1.size (); ++i)
  {
    //std::cout << "STARTING rototranslations1 ITERATION" << std::endl;
    //iterate through all rototranslations for a certain model

    pcl::transformPointCloud (*model1, *rotated_model1, rototranslations1[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    //this is the determining of the color (red) and actual adding of the point cloud 
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler1 (rotated_model1, 255, 0, 0);

    calcMinMax(rotated_model1);
    //this line adds a point cloud to the viewer
    //viewer.addPointCloud (rotated_model1, rotated_model_color_handler1, ss_cloud.str ());
    //std::cout << "ss_cloud.str(): " << ss_cloud.str() << std::endl;
    if (show_correspondences_)
    {
      int centerPointSlot = 0;
      int rotated_model_size1 = 0;
      for(int j = 0; j < rotated_model1->points.size(); j++){
        rotated_model_size1++;
      }
      std::cout << "rotatedmodelsize1: " << rotated_model_size1 << std::endl;
      std::stringstream ss_line;
      ss_line << "line1";
      //add 2 slots to rotated_model array
      rotated_model1->points.resize ((rotated_model_size1+2) * 1);

      //define the first new slot to be at the center
      rotated_model1->points[rotated_model_size1+1].x = centerPointArray[0];
      rotated_model1->points[rotated_model_size1+1].y = centerPointArray[1];
      rotated_model1->points[rotated_model_size1+1].z = centerPointArray[2];

      //define the second new slot to be at the origin
      rotated_model1->points[rotated_model_size1+2].x = 0.0;
      rotated_model1->points[rotated_model_size1+2].y = 0.0;
      rotated_model1->points[rotated_model_size1+2].z = 0.0;

      //we aren't writing points to the .pcd file, just defining locations to draw a line to that are stored in the array
      PointType& origin_point1 = rotated_model1->points[rotated_model_size1+2];
      PointType& center_point1 = rotated_model1->points[rotated_model_size1+1];
      //calcMinMax(rotated_model);//********pcl::PointCloud<pcl::PointXYZRGB> cloudTarget;
      viewer.addLine<PointType, PointType> (origin_point1, center_point1, 0, 255, 255, ss_line.str ());

      //cloudTarget.points[cloudIndex].x = currentX
      /*
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);
        
        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }*/
      }
    }
  std::cout << "rototranslations2.size(): " << rototranslations2.size() << std::endl;
  for (std::size_t j = 0; j < rototranslations2.size (); ++j)
  {
    //iterate through all rototranslations for a certain model
    pcl::transformPointCloud (*model2, *rotated_model2, rototranslations2[j]);

    std::stringstream ss_cloud2;
    ss_cloud2 << "2instance" << j;

    //this is the determining of the color (red) and actual adding of the point cloud 
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler2 (rotated_model2, 0, 255, 0);

    calcMinMax(rotated_model2);
    //this commented out line adds the point cloud to the viewer
    ///viewer.addPointCloud (rotated_model2, rotated_model_color_handler2, ss_cloud2.str ());
    std::cout << "ss_cloud2.str(): " << ss_cloud2.str() << std::endl;

    int centerPointSlot = 0;
    int rotated_model_size2 = 0;
    for (int j = 0; j < rotated_model2->points.size(); j++){
      rotated_model_size2++;
    }
    std::cout << "rotatedmodelsize2: " << rotated_model_size2 << std::endl;
    std::stringstream ss_line2;
    ss_line2 << "line2";
    //add 2 slots to rotated_model array
    rotated_model2->points.resize((rotated_model_size2 + 2) * 1);

    //define the first new slot to be at the center
    rotated_model2->points[rotated_model_size2 + 1].x = centerPointArray[0];
    rotated_model2->points[rotated_model_size2 + 1].y = centerPointArray[1];
    rotated_model2->points[rotated_model_size2 + 1].z = centerPointArray[2];

    //define the second new slot to be at the origin
    rotated_model2->points[rotated_model_size2 + 2].x = 0.0;
    rotated_model2->points[rotated_model_size2 + 2].y = 0.0;
    rotated_model2->points[rotated_model_size2 + 2].z = 0.0;

    //we aren't writing points to the .pcd file, just defining locations to draw a line to that are stored in the array
    PointType &origin_point2 = rotated_model2->points[rotated_model_size2 + 2];
    PointType &center_point2 = rotated_model2->points[rotated_model_size2 + 1];
    viewer.addLine<PointType, PointType>(origin_point2, center_point2, 0, 255, 0, ss_line2.str());

  }//for (std::size_t i = 0; i < rototranslations.size (); ++i)

  ///these two lines of code add the two calculated model clouds to the viewer. 
  ///for calculating the center point of each model and drawing a line from the origin
  ///to each centroid, and these two lines aren't really needed
  //viewer.addPointCloud (rotated_model1, rotated_model_color_handler1, ss_cloud.str ());
  //viewer.addPointCloud (rotated_model2, rotated_model_color_handler2, ss_cloud2.str ());

  std::cout << "ADDING scene_cloud TO VIEWER" << std::endl;
  viewer.addPointCloud (scene, "scene_cloud");
  

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);

}//end main function























