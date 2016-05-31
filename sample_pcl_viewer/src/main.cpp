// STD includes
#include <iostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Utilities

// Project includes
#include "vis.hpp"

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char** argv, std::string &cloudFilename)
{
  cloudFilename = "";
  
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
    cloudFilename = curParameter;    
  }  
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{    
  //----------------------------------------------------------------------------
  // Parse command line
  //----------------------------------------------------------------------------

  std::string cloudFilename;
  parseCommandLine(argc, argv, cloudFilename);
  
  //----------------------------------------------------------------------------
  // Load PCD file
  //----------------------------------------------------------------------------
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);  
  if (pcl::io::loadPCDFile (cloudFilename, *cloud) == -1)
  {
    std::cout << "Could not read pointcloud file '" + cloudFilename + "'" << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Calculate distance to origin for each point
  //----------------------------------------------------------------------------
  
  std::vector<float> distanceToOrigin (cloud->size());
  for (size_t pointId = 0; pointId < cloud->size(); pointId++)
  {
    distanceToOrigin[pointId] = cloud->points[pointId].getVector3fMap().norm();
  }

  //----------------------------------------------------------------------------
  // Print instructions
  //----------------------------------------------------------------------------
  
  std::cout << "This sample code shows how to visualize pointclouds using PCL library." << std::endl;
  std::cout << "There are 3 visualizations available:" << std::endl;
  std::cout << " 1. Pointcloud without color" << std::endl;
  std::cout << " 2. Pointcloud with color" << std::endl;
  std::cout << " 3. Points colored according to their Euclidean distance from the origin" << std::endl;
  std::cout << std::endl;
  std::cout << "|         Key         |             Function            |" << std::endl;
  std::cout << "|---------------------|---------------------------------|" << std::endl;
  std::cout << "| Keypad number keys  | change cloud visualization type |" << std::endl;
  std::cout << "| N                   | show normals                    |" << std::endl;
  std::cout << "| U                   | show colorbar                   |" << std::endl;
  std::cout << "| Keypad +/-          | change point size               |" << std::endl;
  std::cout << "| Q                   | quit                            |" << std::endl;
  
  //----------------------------------------------------------------------------
  // Display the pointcloud
  //----------------------------------------------------------------------------
  
  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition ( 0.0, 0.0,  0.0,   // camera position
                                 0.0, 0.0,  1.0,   // focal point
                                 0.0, -1.0, 0.0,   // normal
                                 0.0);             // viewport  
  visualizer.setBackgroundColor (utl::pclvis::bgColor.r, utl::pclvis::bgColor.g, utl::pclvis::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback, (void*)(&visState));
  
  visState.updateDisplay_ = true;
  
  while (!visualizer.wasStopped())
  {
    // Update display if needed
    if (visState.updateDisplay_)
    {     
      // First remove everything
      visualizer.removeAllPointClouds();
      visualizer.removeAllShapes();
      visualizer.removeAllCoordinateSystems();
      visState.updateDisplay_ = false;

      // Show pointcloud without color color
      if (visState.cloudDisplayState_ == VisState::CLOUD_NO_COLOR)
      {
        utl::pclvis::showPointCloud<pcl::PointXYZRGBNormal>(visualizer, cloud, "cloud", visState.pointSize_, utl::pclvis::Color(0.5, 0.5, 0.5));
        visualizer.addText("Cloud without color", 0, 100, 24, 1.0, 1.0, 1.0);
      }
      
      // Show pointcloud with color
      if (visState.cloudDisplayState_ == VisState::CLOUD_COLOR)
      {
        utl::pclvis::showPointCloudColor<pcl::PointXYZRGBNormal>(visualizer, cloud, "cloud", visState.pointSize_);
        visualizer.addText("Cloud with color", 0, 100, 24, 1.0, 1.0, 1.0);
      }
      
      // Color points according to their distance from origin
      if (visState.cloudDisplayState_ == VisState::CLOUD_DISTANCE_TO_ORIGIN)
      {
        utl::pclvis::showPointCloudWithData<pcl::PointXYZRGBNormal, float>(visualizer, cloud, distanceToOrigin, "cloud_curvature", visState.pointSize_);
        visualizer.addCoordinateSystem(0.2, "origin");
        visualizer.addText("Point distance to origin", 0, 100, 24, 1.0, 1.0, 1.0);
      }
      
      // Add normals
      if (visState.showNormals_)
        utl::pclvis::showNormalCloud<pcl::PointXYZRGBNormal>(visualizer, cloud, 10, 0.02, "cloud_normals", visState.pointSize_, utl::pclvis::Color(0.0, 1.0, 0.0));      
    }

    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
  }
  
  return 0;
}