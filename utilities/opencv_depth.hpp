#ifndef OPENCV_DEPTH_UTILITIES_HPP
#define OPENCV_DEPTH_UTILITIES_HPP

#include <fstream>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace utl
{
  namespace ocv
  {
    /** \brief Write an OpenCV image to disk
      * \param[in] filename output filename
      * \param[in] image image
      * \note Adapted from HAL library of ARPG lab (https://github.com/arpg/HAL).
      */ 
    inline
    bool writeDepthImage(const std::string &filename, const cv::Mat &image)
    {
      std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
      if (file.is_open())
      {
        file << "P7" << std::endl;
        file << image.cols << " " << image.rows << std::endl;
        const size_t size = image.elemSize1() * image.rows * image.cols;
        file << 4294967295 << std::endl;
        file.write((const char*)image.data, size);
        file.close();
        
        return true;
      }
      else
      {
        std::cout << "Could not open file for writing (" << filename << ")" << std::endl;
        return false;
      }
    }
    
    /** \brief Read an image from disk and store it as OpenCV image
      * \param[in] filename output filename
      * \return OpenCV image
      * \note Adapted from HAL library of ARGG lab (https://github.com/arpg/HAL).
      */ 
    inline
    cv::Mat readDepthImage(const std::string &filename)
    {
      // magic number P7, portable depthmap, binary
      std::ifstream file( filename.c_str() );

      unsigned int        nImgWidth;
      unsigned int        nImgHeight;
      long unsigned int   nImgSize;

      cv::Mat imDepth;

      if( file.is_open() ) {
          std::string sType;
          file >> sType;
          file >> nImgWidth;
          file >> nImgHeight;
          file >> nImgSize;

          // the actual PGM/PPM expects this as the next field:
          //    nImgSize++;
          //    nImgSize = (log( nImgSize ) / log(2)) / 8.0;

          // but ours has the actual size (4 bytes of float * pixels):
          nImgSize = 4 * nImgWidth * nImgHeight;

          imDepth = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );

          file.seekg( file.tellg() + (std::ifstream::pos_type)1, std::ios::beg );
          file.read( (char*)imDepth.data, nImgSize );
          file.close();
      }
      return imDepth;
    }  
    
    /** \brief Rescale depth image pixel values. It is assumed that the input 
      * depth image is CV_16UC1 and pixel depth values are measured in mm.
      * \param[in] depth input depth image
      * \param[in] min_depth minimum depth value in mm
      * \param[in] max_depth maximum depth value in mm
      * \return scaled depth image
      */
    inline
    cv::Mat scaleDepth(const cv::Mat &depth, const float &min_depth = 400.0f, const float &max_depth = 4000.0f)
    {
      cv::Mat depth_scaled;    
      
      // Check that input image is correct type
      if (depth.depth() != CV_16U || depth.channels() != 1)
      {
        std::cout << "[utl::rescaleDepth] input depth must be of type CV_16UC1." << std::endl;
        return depth_scaled;
      }
      
      // Scale and return
      depth_scaled = (depth - min_depth) / (max_depth - min_depth) * pow(2.0, 16.0);
          
      return depth_scaled;
    }
    
    /** \brief Clean up depth image by removing all pixels for which the maximum
      * difference between the center pixel and it's neighbours is higher than a
      * threshold
      * \param[in] depth input depth image
      * \param[in] depth difference threshold
      * \return cleaned depth image
      */
    inline
    cv::Mat cleanDepth(const cv::Mat &depth, const float &depth_diff_thresh = 0.01)
    {
      cv::Mat depth_cleaned;
      depth.copyTo(depth_cleaned);
      int xRes = depth_cleaned.cols;
      int yRes = depth_cleaned.rows;
      
      // Filter depth
      for (int x = 0; x < xRes; x++)
      {
        for (int y = 0; y < yRes; y++)
        {
          // Get center pixel value
          float centerDepth = static_cast<float> (depth.at<ushort>(y,x));
          
          // Get neighbour coordinates
          std::vector<std::pair<int,int> > neighbours(8);
          neighbours[0] = std::pair<int,int> (y-1,  x-1 );
          neighbours[1] = std::pair<int,int> (y-1,  x   );
          neighbours[2] = std::pair<int,int> (y-1,  x+1 );
          neighbours[3] = std::pair<int,int> (y,    x-1);
          neighbours[4] = std::pair<int,int> (y,    x+1);
          neighbours[5] = std::pair<int,int> (y+1,  x-1);
          neighbours[6] = std::pair<int,int> (y+1,  x);
          neighbours[7] = std::pair<int,int> (y+1,  x+1);

          // Get maximum difference between center and neighbours
          for (size_t nbrId = 0; nbrId < neighbours.size(); nbrId++)
          {
            int nbrY = neighbours[nbrId].first;
            int nbrX = neighbours[nbrId].second;
            
            if (nbrY < 0 || nbrY > yRes-1 || nbrX < 0 || nbrX > xRes-1)
            {
              depth_cleaned.at<ushort> (y,x) = 0;
              continue;
            }
            
            float nbrDepth = static_cast<float> (depth.at<ushort>(nbrY,nbrX));
            if (std::abs(nbrDepth - centerDepth) > depth_diff_thresh)
            {
              depth_cleaned.at<ushort> (y,x) = 0;
              continue;
            }
          }
        }
      }
      
      return depth_cleaned;
    }

    /** \brief Clean up depth image by removing all pixels for which the maximum
      * difference between the center pixel and it's neighbours is higher than a
      * threshold
      * \param[in] depth input depth image
      * \param[in] depth difference threshold
      * \return cleaned depth image
      */
    inline
    void getDepthDiscontinuities(const cv::Mat &depth, cv::Mat &depth_disc, const float &depth_diff_thresh = 50.0f)
    {
      cv::Mat dDepth;
      cv::Mat kernel = - cv::Mat::ones(3, 3, CV_32F);
      kernel.at<float>(1,1) = 8;
      
      cv::filter2D(depth, dDepth, CV_32F, kernel, cv::Point(1,1), 0.0, cv::BORDER_CONSTANT);
      dDepth = cv::abs(dDepth);
      cv::threshold(dDepth, dDepth, depth_diff_thresh, 1.0, CV_THRESH_BINARY);
      
      dDepth.convertTo(depth_disc, CV_8U);
    }  
    
    /** \brief Clean up depth image by removing all pixels for which the maximum
      * difference between the center pixel and it's neighbours is higher than a
      * threshold
      * \param[in] depth input depth image
      * \param[in] depth difference threshold
      * \return cleaned depth image
      */
    inline
    void getDepthDiscontinuities_old(const cv::Mat &depth, cv::Mat &depth_disc, const float &depth_diff_thresh = 10.0f)
    {
      int xRes = depth.cols;
      int yRes = depth.rows;
      
      depth_disc = cv::Mat::zeros(yRes, xRes, CV_8U);
      
      // Filter depth
      for (int x = 0; x < xRes; x++)
      {
        for (int y = 0; y < yRes; y++)
        {
          // Get center pixel value
          float centerDepth = static_cast<float> (depth.at<ushort>(y,x));
          
          // Get neighbour coordinates
          std::vector<std::pair<int,int> > neighbours(8);
          neighbours[0] = std::pair<int,int> (y-1,  x-1 );
          neighbours[1] = std::pair<int,int> (y-1,  x   );
          neighbours[2] = std::pair<int,int> (y-1,  x+1 );
          neighbours[3] = std::pair<int,int> (y,    x-1);
          neighbours[4] = std::pair<int,int> (y,    x+1);
          neighbours[5] = std::pair<int,int> (y+1,  x-1);
          neighbours[6] = std::pair<int,int> (y+1,  x);
          neighbours[7] = std::pair<int,int> (y+1,  x+1);

          // Get maximum difference between center and neighbours
          for (size_t nbrId = 0; nbrId < neighbours.size(); nbrId++)
          {
            int nbrY = neighbours[nbrId].first;
            int nbrX = neighbours[nbrId].second;
            
            if (nbrY < 0 || nbrY > yRes-1 || nbrX < 0 || nbrX > xRes-1)
            {
              depth_disc.at<uchar> (y,x) = 1;
              continue;
            }
            
            float nbrDepth = static_cast<float> (depth.at<ushort>(nbrY,nbrX));
            if (std::abs(nbrDepth - centerDepth) > depth_diff_thresh)
            {
              depth_disc.at<uchar> (y,x) = 1;
              continue;
            }
          }
        }
      }    
    }
  }
}
  
#endif // OPENCV_DEPTH_UTILITIES_HPP