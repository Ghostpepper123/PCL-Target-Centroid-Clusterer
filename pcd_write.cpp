#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <exception>

//#include <filesystem>

//namespace fs = std::filesystem;

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB> cloudTarget1;
  pcl::PointCloud<pcl::PointXYZRGB> cloudTarget2;
  pcl::PointCloud<pcl::PointXYZRGB> cloudScene;
  std::string targetFileName1 = "box1.pcd";
  std::string targetFileName2 = "box2.pcd";
  std::string sceneFileName = "box_scene.pcd";

  int firstBoxEndIndex = 0;
  int secondBoxEndIndex = 0;

  float lengthOfGroundY = 0.99;
  float lengthOfGroundX = 0.3;

  float firstBoxStart = 0.25;
  float firstBoxEnd = 0.36;//was 365

  float firstBoxStartZ = 0.25;//*****************
  float firstBoxEndZ = 0.01;//was 0.01

  float secondBoxStartY = 0.39;
  float secondBoxEndY = 0.5;
  float secondBoxStartX = firstBoxStart;
  float secondBoxEndX = firstBoxEnd;

  float secondBoxStartZ = 0.12;//****************
  float secondBoxEndZ = 0.01;//was 0.01

  float colorVar = 255.0;
  float colorVar2 = 255.0;

  //*********

  float groundWidth = 0.48;//originally 1.96
  float groundLength = 0.98;

  int targetSize = 200000;
  //the tall box has targetSize 1365
  //the short box has targetSize 682
  int sceneSize = 200000;//was 42500
  int pointsSavedToTarget = 0;
  int pointsSavedToScene = 0;

  ///WRITE BOX TARGET DATA
  // Fill in the cloud data
  cloudTarget1.width    = targetSize;//previous was 12500
  //the tall box has cloudTarget1.width 1365
  //the short box has cloudTarget1.width 682
  cloudTarget1.height   = 1;
  cloudTarget1.is_dense = false;
  cloudTarget1.points.resize (cloudTarget1.width * cloudTarget1.height);

  cloudTarget2.width    = targetSize;//previous was 12500
  //the tall box has cloudTarget1.width 1365
  //the short box has cloudTarget1.width 682
  cloudTarget2.height   = 1;
  cloudTarget2.is_dense = false;
  cloudTarget2.points.resize (cloudTarget2.width * cloudTarget2.height);  


  cloudScene.width    = sceneSize;//previous was 22500
  cloudScene.height   = 1;
  cloudScene.is_dense = false;
  cloudScene.points.resize (cloudScene.width * cloudScene.height);

  //write plane 1
  //cloud.points[0].x = 0.25;
  //cloud.points[0].y = 0.25;
  //cloud.points[0].z = 0;
  float currentX = firstBoxStart;
  float currentY = firstBoxStart;
  float currentZ = 0.0;

  int cloudIndex1 = 0;
  int cloudIndex2 = 0;
  int cloudIndexScene = 0;

  int totalCloudIndex = 0;
  
  //drawing the ground in the scene
  /*
  std::cout << "WRITING THE GROUND" << std::endl;
  currentZ = 0.0;// was 0.0100002. now it was 0.01
  std::cout << "currentZ: " << currentZ << std::endl;
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;
  for(float x = 0; x <= groundWidth; x+=0.01){

    for(float y = 0; y <= groundLength; y+=0.01){

      cloudScene.points[cloudIndexScene].x = x;
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = currentZ;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;      
      cloudIndexScene++;    


    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "ending ground draw: cloudIndexScene: " << cloudIndexScene << std::endl;
  */
  //plane 1
  std::cout << "WRITING PLANE 1" << std::endl;
  for(float z = firstBoxStartZ; z >= firstBoxEndZ; z-=0.01){//z starting was originally 0.5. now it was z>=0.02

    for(float y = firstBoxStart; y <= firstBoxEnd; y+=0.01){//y ending was originally 0.73
      if(cloudIndex1==0){
        std::cout << "cloudIndex1: " << cloudIndex1 << std::endl;
      }
      //write to first model
      cloudTarget1.points[cloudIndex1].x = currentX;//pcl::PointCloud<pcl::PointXYZRGB> cloudTarget1;
      cloudTarget1.points[cloudIndex1].y = y;
      cloudTarget1.points[cloudIndex1].z = z;  

      cloudTarget1.points[cloudIndex1].r = colorVar;
      cloudTarget1.points[cloudIndex1].g = colorVar;
      cloudTarget1.points[cloudIndex1].b = colorVar;
      cloudIndex1++;  
      //write to scene
      cloudScene.points[cloudIndexScene].x = currentX;//pcl::PointCloud<pcl::PointXYZRGB> cloudTarget1;
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;
      cloudIndexScene++;        


    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "start point plane 2 cloudIndex1: " << cloudIndex1 << std::endl;

  //plane 2
  std::cout << "WRITING PLANE 2" << std::endl;
  
  std::cout << "currentY: " << currentY << std::endl;
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;
  for(float z = firstBoxStartZ; z >= firstBoxEndZ; z-=0.01){

    for(float x = firstBoxStart; x <= firstBoxEnd; x+=0.01){
      //write model 1
      cloudTarget1.points[cloudIndex1].x = x;
      cloudTarget1.points[cloudIndex1].y = currentY;
      cloudTarget1.points[cloudIndex1].z = z;  

      cloudTarget1.points[cloudIndex1].r = colorVar;
      cloudTarget1.points[cloudIndex1].g = colorVar;
      cloudTarget1.points[cloudIndex1].b = colorVar;
      cloudIndex1++;    
      //write scene
      cloudScene.points[cloudIndexScene].x = x;
      cloudScene.points[cloudIndexScene].y = currentY;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;
      cloudIndexScene++;       


    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "start point plane 3 cloudIndex1: " << cloudIndex1 << std::endl;

  //plane 3
  std::cout << "WRITING PLANE 3" << std::endl;
  currentX = firstBoxEnd;//******double change this if necessary. was 0.37
  std::cout << "currentX: " << currentX << std::endl;
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;
  for(float z = firstBoxStartZ; z >= firstBoxEndZ; z-=0.01){

    for(float y = firstBoxStart; y <= firstBoxEnd; y+=0.01){
      //write model 1
      cloudTarget1.points[cloudIndex1].x = currentX;
      cloudTarget1.points[cloudIndex1].y = y;
      cloudTarget1.points[cloudIndex1].z = z;  

      cloudTarget1.points[cloudIndex1].r = colorVar;
      cloudTarget1.points[cloudIndex1].g = colorVar;
      cloudTarget1.points[cloudIndex1].b = colorVar;      
      cloudIndex1++;    
      //write scene
      cloudScene.points[cloudIndexScene].x = currentX;
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;      
      cloudIndexScene++;         


    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "start point plane 4 cloudIndex1: " << cloudIndex1 << std::endl;

  //plane 4
  std::cout << "WRITING PLANE 4" << std::endl;
  currentY = firstBoxEnd;//add "1" if doesn't work. was 0.37
  std::cout << "currentY: " << currentY << std::endl;
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;
  for(float z = firstBoxStartZ; z >= firstBoxEndZ; z-=0.01){

    for(float x = firstBoxStart; x <= firstBoxEnd; x+=0.01){
      //write model 1
      cloudTarget1.points[cloudIndex1].x = x;
      cloudTarget1.points[cloudIndex1].y = currentY;
      cloudTarget1.points[cloudIndex1].z = z;  

      cloudTarget1.points[cloudIndex1].r = colorVar;
      cloudTarget1.points[cloudIndex1].g = colorVar;
      cloudTarget1.points[cloudIndex1].b = colorVar;      
      cloudIndex1++;   
      //write scene 
      cloudScene.points[cloudIndexScene].x = x;
      cloudScene.points[cloudIndexScene].y = currentY;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;      
      cloudIndexScene++;

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;

  std::cout << "start point plane 5 cloudIndex1: " << cloudIndex1 << std::endl;

  //plane 5
  std::cout << "WRITING PLANE 5" << std::endl;
  currentZ = firstBoxStartZ;
  std::cout << "currentZ: " << currentZ << std::endl;
  //std::cout << "cloudIndex: " << cloudIndex << std::endl;
  for(float x = firstBoxStart; x <= firstBoxEnd; x+=0.01){

    for(float y = firstBoxStart; y <= firstBoxEnd; y+=0.01){

      cloudTarget1.points[cloudIndex1].x = x;  
      cloudTarget1.points[cloudIndex1].y = y;
      cloudTarget1.points[cloudIndex1].z = currentZ; 

      cloudTarget1.points[cloudIndex1].r = colorVar;
      cloudTarget1.points[cloudIndex1].g = colorVar;
      cloudTarget1.points[cloudIndex1].b = colorVar;       
      cloudIndex1++;  
      //write scene
      cloudScene.points[cloudIndexScene].x = x;  
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = currentZ; 

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;       
      cloudIndexScene++;  

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop

  std::cout << "final target cloudIndex1 of box 1: " << cloudIndex1 << std::endl;
  std::cout << "alfalfa" << std::endl;
  std::cout << " cloudIndexScene: " << cloudIndexScene << std::endl;
  firstBoxEndIndex = cloudIndex1;
  totalCloudIndex+=cloudIndex1;

  //cloudScene += cloudTarget1;
  cloudTarget1.width = cloudIndex1;//resize the first model cloud to cloudIndex1
  cloudTarget1.points.resize (cloudTarget1.width * cloudTarget1.height);

  //set scene equal to target, resize the scene to accomodate extra stuff

  cloudScene.width    = sceneSize;//previously was 22500
  cloudScene.height   = 1;
  cloudScene.is_dense = false;
  cloudScene.points.resize (cloudScene.width * cloudScene.height);
  
  //write plane 1 FOR SMALLER BOX
  //cloud.points[0].x = 0.25;
  //cloud.points[0].y = 0.25;
  //cloud.points[0].z = 0;
  //currentX = 0.39;//was 0.25
  currentY = 0.0;//was 0.75
  currentZ = 0.0;
  //int cloudIndex = 0;//cloudIndex should add up to 2500 for one plane
  cloudIndex2 = 0;
  //plane 1 // SECOND BOX
  std::cout << "WRITING PLANE 1" << std::endl;
  currentY = secondBoxStartY;
  for(float z = secondBoxStartZ; z >= secondBoxEndZ; z-=0.01){//z starting was originally 0.5
                                // >= 0.02
    for(float x = secondBoxStartX; x <= secondBoxEndX; x+=0.01){//this used to be 0.75 to <= 0.87

      cloudTarget2.points[cloudIndex2].x = x;
      cloudTarget2.points[cloudIndex2].y = currentY;
      cloudTarget2.points[cloudIndex2].z = z;  

      cloudTarget2.points[cloudIndex2].r = colorVar;
      cloudTarget2.points[cloudIndex2].g = colorVar;
      cloudTarget2.points[cloudIndex2].b = colorVar;
      cloudIndex2++;   
      //write scene 
      cloudScene.points[cloudIndexScene].x = x;
      cloudScene.points[cloudIndexScene].y = currentY;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;
      cloudIndexScene++;   

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "end point plane 1: cloudIndex2: " << cloudIndex2 << std::endl;

  //plane 2
  std::cout << "WRITING PLANE 2" << std::endl;
  currentX = secondBoxStartX;
  std::cout << "currentY: " << currentY << std::endl;
  //std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;
  for(float z = secondBoxStartZ; z >= secondBoxEndZ; z-=0.01){
                                // >= 0.02
    for(float y = secondBoxStartY; y <= secondBoxEndY; y+=0.01){

      cloudTarget2.points[cloudIndex2].x = currentX;
      cloudTarget2.points[cloudIndex2].y = y;  
      cloudTarget2.points[cloudIndex2].z = z;  

      cloudTarget2.points[cloudIndex2].r = colorVar;
      cloudTarget2.points[cloudIndex2].g = colorVar;
      cloudTarget2.points[cloudIndex2].b = colorVar;
      cloudIndex2++;  
      //write scene  
      cloudScene.points[cloudIndexScene].x = currentX;
      cloudScene.points[cloudIndexScene].y = y;  
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;
      cloudIndexScene++;          


    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;

  //plane 3
  std::cout << "WRITING PLANE 3" << std::endl;
  currentY = secondBoxEndY;
  std::cout << "currentX: " << currentX << std::endl;
  std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;
  for(float z = secondBoxStartZ; z >= secondBoxEndZ; z-=0.01){
                                // >= 0.02
    for(float x = secondBoxStartX; x <= secondBoxEndX; x+=0.01){

      cloudTarget2.points[cloudIndex2].x = x;
      cloudTarget2.points[cloudIndex2].y = currentY;
      cloudTarget2.points[cloudIndex2].z = z;  

      cloudTarget2.points[cloudIndex2].r = colorVar;
      cloudTarget2.points[cloudIndex2].g = colorVar;
      cloudTarget2.points[cloudIndex2].b = colorVar;      
      cloudIndex2++;    
      //write scene
      cloudScene.points[cloudIndexScene].x = x;
      cloudScene.points[cloudIndexScene].y = currentY;  
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;
      cloudIndexScene++;  

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;

  //plane 4
  
  std::cout << "WRITING PLANE 4" << std::endl;
  currentX = secondBoxEndX;//add "1" if doesn't work
  std::cout << "currentY: " << currentY << std::endl;
  std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;
  for(float z = secondBoxStartZ; z >= secondBoxEndZ; z-=0.01){
                                // >= 0.02
    for(float y = secondBoxStartY; y <= secondBoxEndY; y+=0.01){//was 0.25 to 0.37

      cloudTarget2.points[cloudIndex2].x = currentX;
      cloudTarget2.points[cloudIndex2].y = y;
      cloudTarget2.points[cloudIndex2].z = z;  

      cloudTarget2.points[cloudIndex2].r = colorVar;
      cloudTarget2.points[cloudIndex2].g = colorVar;
      cloudTarget2.points[cloudIndex2].b = colorVar;      
      cloudIndex2++;    
      //write scene
      cloudScene.points[cloudIndexScene].x = currentX;
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = z;  

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;      
      cloudIndexScene++; 

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;

  //plane 5
  
  std::cout << "WRITING PLANE 5" << std::endl;
  currentZ = secondBoxStartZ;
  std::cout << "currentZ: " << currentZ << std::endl;
  //std::cout << "cloudIndex2: " << cloudIndex2 << std::endl;
  for(float x = secondBoxStartX; x <= secondBoxEndX; x+=0.01){

    for(float y = secondBoxStartY; y <= secondBoxEndY; y+=0.01){

      cloudTarget2.points[cloudIndex2].x = x;  
      cloudTarget2.points[cloudIndex2].y = y;
      cloudTarget2.points[cloudIndex2].z = currentZ; 

      cloudTarget2.points[cloudIndex2].r = colorVar;
      cloudTarget2.points[cloudIndex2].g = colorVar;
      cloudTarget2.points[cloudIndex2].b = colorVar;       
      cloudIndex2++;    
      //write scene
      cloudScene.points[cloudIndexScene].x = x;  
      cloudScene.points[cloudIndexScene].y = y;
      cloudScene.points[cloudIndexScene].z = currentZ; 

      cloudScene.points[cloudIndexScene].r = colorVar;
      cloudScene.points[cloudIndexScene].g = colorVar;
      cloudScene.points[cloudIndexScene].b = colorVar;       
      cloudIndexScene++;

    }//end inner for loop
    //std::cout << "NEW ROW: " << cloudIndex << std::endl;

  }//end outer for loop
  std::cout << "final cloudIndex2 after writing box 2: " << cloudIndex2 << std::endl;
  std::cout << "cloudIndexScene: " << cloudIndexScene << std::endl;
  //cloudScene += cloudTarget2;
  cloudTarget2.width = cloudIndex2;//cloudIndex
  cloudTarget2.points.resize (cloudTarget2.width * cloudTarget2.height);

  //totalCloudIndex+=cloudIndex2;


  cloudScene.width = cloudIndexScene;//cloudIndex
  cloudScene.points.resize (cloudScene.width * cloudScene.height);
  std::cout << "alfalfa" << std::endl;
  std::cout << "cloudIndexScene: " << cloudIndexScene << std::endl;

  //******SAVE TO FILE*******
  pcl::io::savePCDFileASCII (targetFileName1, cloudTarget1);//write cloudTarget1 to targetFileName1 locally
  pcl::io::savePCDFileASCII ("/home/craig/PCL_READ/build/" + targetFileName1, cloudTarget1);

  pcl::io::savePCDFileASCII (targetFileName2, cloudTarget2);//write cloudTarget2 to targetFileName2 locally
  pcl::io::savePCDFileASCII ("/home/craig/PCL_READ/build/" + targetFileName2, cloudTarget2);  

  pcl::io::savePCDFileASCII (sceneFileName, cloudScene);//write cloudScene to sceneFileName
  pcl::io::savePCDFileASCII ("/home/craig/PCL_READ/build/" + sceneFileName, cloudScene);
  
  std::cerr << "Saved " << cloudTarget1.points.size() << " data points to " << targetFileName1 << std::endl;
  std::cerr << "Saved " << cloudTarget2.points.size() << " data points to " << targetFileName2 << std::endl;
  std::cerr << "Saved " << cloudScene.points.size() << " data points to " << sceneFileName << std::endl;
  /*
  fs::path sourceFile1 = targetFileName;
  fs::path sourceFile2 = sceneFileName;
  fs::path targetParent = "/home/craig/tdir1";//"/home/craig/PCL_READ/build";
  auto target1 = targetParent / sourceFile1.filename(); // sourceFile.filename() returns "sourceFile.ext".
  auto target2 = targetParent / sourceFile2.filename(); // sourceFile.filename() returns "sourceFile.ext".
  try // If you want to avoid exception handling, then use the error code overload of the following functions.
  {
      fs::create_directories(targetParent); // Recursively create target directory if not existing.
      fs::copy_file(sourceFile1, target1, fs::copy_options::overwrite_existing);

      fs::create_directories(targetParent); // Recursively create target directory if not existing.
      fs::copy_file(sourceFile2, target2, fs::copy_options::overwrite_existing);

  }
  catch (std::exception& e) // Not using fs::filesystem_error since std::bad_alloc can throw too.  
  {
      std::cout << e.what();
  }*/

  //for (size_t i = 0; i < cloudTarget.points.size (); ++i)
    //std::cerr << "    " << cloudTarget.points[i].x << " " << cloudTarget.points[i].y << " " << cloudTarget.points[i].z << std::endl;

  return (0);
}
