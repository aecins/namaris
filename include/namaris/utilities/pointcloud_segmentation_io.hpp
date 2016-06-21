#ifndef POINTCLOUD_SEGMENTATION_IO_HPP
#define POINTCLOUD_SEGMENTATION_IO_HPP

// STD includes
#include <fstream>

// My includes
#include <namaris/utilities/map.hpp>

namespace utl
{
  namespace cloud
  {
    bool writeSegmentation (const std::string &filename, const utl::map::Map &segmentation)
    {
      //------------------------------------------------------------------------
      // Check that segmentation is not empty
      //------------------------------------------------------------------------
      
      if (segmentation.size() == 0)
      {
        std::cout << "[utl::cloud::writeSegmentation] Segmentation is empty, nothing to save." << std::endl;
//         return false;
      }
      
      //------------------------------------------------------------------------
      // Open file for writing
      //------------------------------------------------------------------------
      
      std::ofstream file(filename);
      if (!file.is_open())
      {
        std::cout << "[utl::cloud::writeSegmentation] Could not open file for writing ('" << filename << "')." << std::endl;
        return false;
      }
      
      //------------------------------------------------------------------------
      // Write header
      //------------------------------------------------------------------------
      
      file << segmentation.size() << " segments" << std::endl << std::endl;
      
      //------------------------------------------------------------------------
      // Write segment information
      //------------------------------------------------------------------------
      
      for (size_t segId = 0; segId < segmentation.size(); segId++)
      {
        file << "--------------------" << std::endl;
        file << segId << std::endl;
        file << "--------------------" << std::endl;
        
        for (size_t pointIdIt = 0; pointIdIt < segmentation[segId].size(); pointIdIt++)
          file << segmentation[segId][pointIdIt] << std::endl;
        
        file << std::endl;
      }
      
      return true;
    }
    
    bool readSegmentation (const std::string &filename, utl::map::Map &segmentation)
    {
      //------------------------------------------------------------------------
      // Open file for reading
      //------------------------------------------------------------------------
      
      std::ifstream file(filename);
      if (!file.is_open())
      {
        std::cout << "[utl::cloud::readSegmentation] Could not open file for reading ('" << filename << "')." << std::endl;
        return false;
      }

      segmentation.resize(0);
      
      //------------------------------------------------------------------------
      // Read header
      //------------------------------------------------------------------------
      
      int numSegments;
      std::string line;
      file >> numSegments;
      
      if (numSegments < 0)
      {
        std::cout << "[utl::cloud::readSegmentation] Number of segments is smaller than 0, segmentation file is corrupted." << std::endl;
        std::cout << "[utl::cloud::readSegmentation] In file: " << filename << std::endl;
        return false;
      }
      else if (numSegments == 0)
      {
        std::cout << "[utl::cloud::readSegmentation] File contains 0 segments." << std::endl;
        std::cout << "[utl::cloud::readSegmentation] In file: " << filename << std::endl;
        return true;        
      }
      
      file >> line;
      
      //------------------------------------------------------------------------
      // Initialize segmentation file
      //------------------------------------------------------------------------
      
      // Read segment point indices
      int numSegmentsRead = 0;
      bool done = false;
      file >> line;
      
      while (!done)
      {
        
        //----------------------------------------------------------------------
        // Read segment header
        
        int segIdRead;
        file >> segIdRead;
        
        if (segIdRead != numSegmentsRead)
        {
          std::cout << "[utl::cloud::readSegmentation] Unexpected segment id. Expected " << numSegmentsRead << ", got " << segIdRead << std::endl;
          std::cout << "[utl::cloud::readSegmentation] Segmentation file is corrupted." << std::endl;
          std::cout << "[utl::cloud::readSegmentation] In file: " << filename << std::endl;
          return false;
        }
        
        numSegmentsRead++;
        segmentation.push_back(std::vector<int> ());
        file >> line;
        
        //------------------------------------------------------------------------
        // Read point indices
      
        while (!file.eof())
        {
          file >> line;
          if (line == "--------------------" || line == "")
            break;
          else
          {
            int pointId = std::stoi(line);
            segmentation[segIdRead].push_back(pointId);
            line.clear();
          }
        }
        
        if (file.eof())
          done = true;
      }
      
      //------------------------------------------------------------------------      
      // Check that number of segments read is the same as number of segments in the header
      //------------------------------------------------------------------------
      
      if (numSegments != numSegmentsRead)
      {
        std::cout << "[utl::cloud::readSegmentation] Expected " << numSegments << " segments, was able to read " << numSegmentsRead << " segments." << std::endl;
        std::cout << "[utl::cloud::readSegmentation] Segmentation file is corrupted." << std::endl;
        std::cout << "[utl::cloud::readSegmentation] In file: " << filename << std::endl;
      }
      
      return true;
    }
  }
}

#endif  // POINTCLOUD_SEGMENTATION_IO_HPP