#ifndef KINECT_HPP
#define KINECT_HPP

// STD includes
#include <iostream>

// Eigen
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Utilities
#include <utilities/filesystem.hpp>
#include <utilities/std_vector.hpp>

namespace utl
{
  namespace kinect
  {
    /** \brief Image types from a Kinect camera */
    enum ImageType { RGB, DEPTH, IR };
    static const char * ImageTypeStrings[] = { "rgb", "depth", "ir" };

    /** \brief Generate a filename given for a frame
     *  \param[in]  frame_number  frame number of an image
     *  \param[in]  image_type    type of the image to be saved
     *  \return filename for the frame
     */
    inline
    std::string generateFilename(const int frame_id, const ImageType image_type)
    {
      std::string filename = "frame_" + std::to_string(frame_id) + "_";
      
      switch (image_type)
      {
        case RGB:
          filename += "rgb";
          break;
          
        case DEPTH:
          filename += "depth";
          break;
          
        case IR:
          filename += "ir";
          break;
          
        default:
          std::cout << "Unknown image type" << std::endl;
          filename += "WTF";
      }
      
      filename += ".png";
      
      return filename;
    }

    /** \brief Given a filename figure out if it is a valid frame and if so what
     * is it's image type and frame number
     *  \param[in]  filename      name of the file
     *  \param[out] frame_id      id of the frame to be saved
     *  \param[out] image_type    type of the image to be saved
     *  \return TRUE if a file for the specified frame exists
     */
    inline
    bool getImageInfo(const std::string &filename, int &frame_id, ImageType &image_type)
    {
      // Check if prefix is correct
      if (filename.substr(0, 6) != "frame_")
        return false;
      
      // Get image type
      int startPos = filename.find("_", 6, 1) + 1;
      int endPos   = filename.find(".png");
      std::string imageTypeString = filename.substr(startPos, endPos-startPos);
      
      if (imageTypeString == ImageTypeStrings[RGB])
        image_type = RGB;
      else if (imageTypeString == ImageTypeStrings[DEPTH])
        image_type = DEPTH;
      else if (imageTypeString == ImageTypeStrings[IR])
        image_type = IR;
      else
      {
        std::cout << "[getImageInfo] unknown image type postfix '" << imageTypeString << "'." << std::endl;
        return false;
      }
      
      // Get frame ID
      startPos = 6;
      endPos = filename.find("_", startPos, 1);
      frame_id = std::stoi(filename.substr(startPos, endPos-startPos));
      
      return true;
    }

    /** \brief Write a frame of specified type, frame id and size
     *  \param[in]  dirname     path to directory where frame should be saved
     *  \param[in] frame_id     frame id
     *  \param[in] image_type   frame image type
     *  \param[out] image       image
     *  \return TRUE if image was saved successfully
     */
    inline
    bool writeFrame ( const std::string &dirname,
                      const int frame_id,
                      const ImageType image_type,
                      cv::Mat &image
                    )
    {
      std::string filename = generateFilename(frame_id, image_type);
      filename = utl::fs::fullfile(dirname, filename);
      
      if (!cv::imwrite(filename, image))
      {
        std::cout << "[utl::kinect::writeFrame] couldn't save image to file '" << filename << "'." << std::endl;
        return false;
      }
      
      return true;
    }
    
    /** \brief Read a frame of specified type, frame id and size
     *  \param[in]  dirname     path to directory containing the file
     *  \param[in] frame_id     frame id
     *  \param[in] image_type   frame image type
     *  \param[out] image       image
     *  \return TRUE if image was loaded successfully
     */
    inline
    bool readFrame  ( const std::string &dirname,
                      const int frame_id,
                      const ImageType image_type,
                      cv::Mat &image
                    )
    {
      // Read image
      std::string filename = generateFilename(frame_id, image_type);
      image = cv::imread(utl::fs::fullfile(dirname, filename), CV_LOAD_IMAGE_ANYDEPTH+CV_LOAD_IMAGE_ANYCOLOR);
      
      if (image.empty())
        return false;
      
      return true;
    }
    
    /** \brief Convert a pose represented by an OpenCV rotation and translation
     * matrices to an Eigen 3D affine transform.
     *  \param[in] R  OpenCV matrix containing rotation matrix
     *  \param[in] t  OpenCV matrix containing translation vector
     *  \param[out] T  pose represented in an Eigen::Affine3f transform
     */
    inline
    void cvPose2eigPose ( const cv::Mat &R,  const cv::Mat &t, Eigen::Affine3f &T )
    {
      Eigen::Matrix3f R_eig;
      Eigen::Vector3f t_eig;
      cv::cv2eigen(t, t_eig);
      cv::cv2eigen(R, R_eig);
      T.translation() = t_eig;
      T.linear() = R_eig;
    }

    /** \brief Convert a pose represented by an Eigen 3D affine transform to 
     * OpenCV rotation and translation matrices.
     *  \param[in] T  pose represented in an Eigen::Affine3f transform
     *  \param[out] R  OpenCV matrix containing rotation matrix
     *  \param[out] t  OpenCV matrix containing translation vector
     */
    inline
    void eigPose2cvPose ( const Eigen::Affine3f T, cv::Mat &R, cv::Mat &t )
    {      
      cv::eigen2cv(T.rotation(), R);
      Eigen::Vector3f t_eig = T.translation();
      cv::eigen2cv(t_eig, t);
    }    
    
    /** \brief Convert a vector of OpenCV Point3f to a PCL pointcloud.
     *  \param[in]  cv_cloud opencv cloud
     *  \param[out] pcl_cloud PCL cloud
     */
    template <typename PointT>
    void cvCloud2pclCloud (const std::vector<cv::Point3f> &cv_cloud, pcl::PointCloud<PointT> &pcl_cloud)
    {
      pcl_cloud.resize(cv_cloud.size());
      for (size_t pointId = 0; pointId < cv_cloud.size(); pointId++)
        pcl_cloud.points[pointId] = pcl::PointXYZ(cv_cloud[pointId].x, cv_cloud[pointId].y, cv_cloud[pointId].z);
    }
    
    /** \brief Convert PCL cloud to a vector of OpenCV Point3f.
     *  \param[in]  pcl_cloud PCL cloud
     *  \param[out] cv_cloud OpenCV cloud
     */    
    template <typename PointT>
    void pclCloud2cvCloud (const pcl::PointCloud<PointT> &pcl_cloud, std::vector<cv::Point3f> &cv_cloud)
    {
      cv_cloud.resize(pcl_cloud.size());
      for (size_t pointId = 0; pointId < pcl_cloud.size(); pointId++)
        cv_cloud[pointId] = cv::Point3f(pcl_cloud.points[pointId].x, pcl_cloud.points[pointId].y, pcl_cloud.points[pointId].z);      
    }
    
    /** \brief Write stereo camera calibration parameters.
     *  \param[in] calib_filename   path to the calibration file
     *  \param[in] K_cam1           camera 1 matrix
     *  \param[in] K_cam2           camera 2 matrix
     *  \param[in] d_cam1           camera 1 distortion parameters
     *  \param[in] d_cam2           camera 2 distortion parameters
     *  \param[in] cam1_to_cam2_R   rotation from camera 1 pose to camera 2 pose
     *  \param[in] cam1_to_Cam2_t   translation from camera 1 pose to camera 2 pose
     *  \param[in] cam1_name        name of camera 1
     *  \param[in] cam1_name        name of camera 2
     *  \return TRUE if parameters were saved successfully
     */
    inline
    bool writeStereoCalibrationParameters ( const std::string &calib_filename,
                                            const cv::Mat &K_cam1, const cv::Mat &K_cam2,
                                            const cv::Mat &d_cam1, const cv::Mat &d_cam2,
                                            const cv::Size &size_cam1, const cv::Size &size_cam2,
                                            const cv::Mat &cam1_to_cam2_R, const cv::Mat &cam1_to_Cam2_t,
                                            const std::string &cam1_name = "cam1",
                                            const std::string &cam2_name = "cam2"
                                          )
    {
      cv::FileStorage fs(calib_filename, cv::FileStorage::WRITE);
      if( fs.isOpened() )
      {
        fs << "size_" + cam1_name << size_cam1  <<  "K_" + cam1_name  << K_cam1 << "d_" + cam1_name << d_cam1 <<
              "size_" + cam2_name << size_cam2  <<  "K_" + cam2_name  << K_cam2 << "d_" + cam2_name << d_cam2   <<
              cam1_name + "_to_" + cam2_name + "_R" <<  cam1_to_cam2_R  <<
              cam1_name + "_to_" + cam2_name + "_t" <<  cam1_to_Cam2_t;
      }
      else
      {
        std::cout << "[utl::kinect::writeStereoCalibrationParameters] Could not open calibration file to save parameters." << std::endl;
        std::cout << "[utl::kinect::writeStereoCalibrationParameters] Calibration file: '" << calib_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }

    /** \brief Read stereo camera calibration parameters.
     *  \param[in] calib_filename   path to the calibration file
     *  \param[in] K_cam1           camera 1 matrix
     *  \param[in] K_cam2           camera 2 matrix
     *  \param[in] d_cam1           camera 1 distortion parameters
     *  \param[in] d_cam2           camera 2 distortion parameters
     *  \param[in] cam1_to_cam2_R   rotation from camera 1 pose to camera 2 pose
     *  \param[in] cam1_to_Cam2_t   translation from camera 1 pose to camera 2 pose
     *  \param[in] cam1_name        name of camera 1
     *  \param[in] cam1_name        name of camera 2
     *  \return TRUE if parameters were loaded successfully
     */
    inline
    bool readStereoCalibrationParameters  ( const std::string &calib_filename,
                                            cv::Mat &K_cam1, cv::Mat &K_cam2,
                                            cv::Mat &d_cam1, cv::Mat &d_cam2,
                                            cv::Size &size_cam1, cv::Size &size_cam2,
                                            cv::Mat &cam1_to_cam2_R, cv::Mat &cam1_to_Cam2_t,
                                            const std::string &cam1_name = "cam1",
                                            const std::string &cam2_name = "cam2"
                                          )
    {
      cv::FileStorage fs(calib_filename, cv::FileStorage::READ);
      if( fs.isOpened() )
      {
        fs["K_" + cam1_name]    >> K_cam1;
        fs["d_" + cam1_name]    >> d_cam1;
        fs["size_" + cam1_name] >> size_cam1;
        fs["K_" + cam2_name]    >> K_cam2;
        fs["d_" + cam2_name]    >> d_cam2;
        fs["size_" + cam2_name] >> size_cam2;
        fs[cam1_name + "_to_" + cam2_name + "_R"] >>  cam1_to_cam2_R;
        fs[cam1_name + "_to_" + cam2_name + "_t"] >>  cam1_to_Cam2_t;
      }
      else
      {
        std::cout << "[utl::kinect::readStereoCalibrationParameters] Could not open calibration file to load parameters." << std::endl;
        std::cout << "[utl::kinect::readStereoCalibrationParameters] Calibration file: '" << calib_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }
    
    /** \brief Get the ideal coordinates for all pixels in an image.
     *  \param[in]  image_size size of an image
     *  \param[in]  K camera matrix
     *  \param[in]  d distortion coefficients
     *  \param[out] ideal_pixel_coordinates ideal coordinates of pixels stored in a 1xN CV_32FC2 matrix. First coordinate corresponds to x and second to y.
     */
    inline
    void getIdealPixelCoordinates  (const cv::Size& image_size, const cv::Mat& K,  const cv::Mat& d, cv::Mat &ideal_pixel_coordinates  )
    {
      cv::Mat pixel_coordinates (image_size.area(), 1, CV_32FC2);
      
      int pixId = 0;
      for (size_t x = 0; x < image_size.width; x++)
        for (size_t y = 0; y < image_size.height; y++)
        {
          pixel_coordinates.at<cv::Vec2f>(pixId)[0] = static_cast<float>(x);
          pixel_coordinates.at<cv::Vec2f>(pixId)[1] = static_cast<float>(y);
          pixId++;
        }
        
      cv::undistortPoints(pixel_coordinates, ideal_pixel_coordinates, K, d);
    }
    
    /** \brief Convert a depth image and corresponding ideal point coordinatesto
     * a PCL pointcloud.
     *  \param[in]  depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[in]  ideal_points a 1xN CV_32FC2 matrix where of ideal coordinates of depth image pixels. Pixels are ordered by scanning the rows.
     *  \param[out] cloud PCL pointcloud
     *  \note all depth values smaller than 0.2 metres and larger than 10 metres are set to NaN
     */
    template <typename PointT>
    void cvDepth2pclCloud(const cv::Mat& depth, const cv::Mat& ideal_pixel_points, pcl::PointCloud<PointT>& cloud)
    {
      if (depth.type() != CV_16UC1)
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] depth image must be of type CV_16UC1." << std::endl;
        abort();
      }

      if (ideal_pixel_points.type() != CV_32FC2)
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] ideal point matrix must be of type CV_32FC2." << std::endl;
        abort();
      }

      if (ideal_pixel_points.size().area() != depth.size().area())
      {
        std::cout << "[utl::kinect::cvDepth2pclCloud] number of points in depth image and ideal point matrix is different." << std::endl;
        abort();
      }
      
      // Prepare cloud      
      cloud.resize(depth.size().area());
      cloud.width = depth.cols;
      cloud.height = depth.rows;
      bool isDense = true;
      
      int pointId = 0;
      for (size_t x = 0; x < depth.cols; x++)
      {
        for (size_t y = 0; y < depth.rows; y++)
        {
          float z = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000;   // Convert milimetres to metres here
          if (z < 0.2 || z > 10.0)
          {
            cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
            isDense = false;
          }
          else
          {
            cloud.at(x,y).x = ideal_pixel_points.at<cv::Vec2f>(pointId)[0] * z;
            cloud.at(x,y).y = ideal_pixel_points.at<cv::Vec2f>(pointId)[1] * z;
            cloud.at(x,y).z = z;
          }
          pointId++;
        }
      }
      cloud.is_dense = isDense;
    }
    
    /** \brief Convert an OpenCV depth image to a pointcloud
      * \param[in] depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
      * \param[in] K depth camera calibration matrix
      * \param[out] cloud PCL pointcloud
      * \note For now the calibration matrix has to be 32F
      */
    template <typename PointT>
    void cvDepth32F2pclCloud(const cv::Mat& depth, const cv::Mat& K, pcl::PointCloud<PointT>& cloud)
    {
      // Check input
      if (K.depth() != CV_32F)
      {
        std::cout << "[cvDepth2pclCloud] calibration matrix must be CV_32F" << std::endl;
        exit(EXIT_FAILURE);
      }
      
      // Prepare cloud
      const int width   = depth.size().width;
      const int height  = depth.size().height;
      
      cloud.resize(width * height);
      cloud.width = width;
      cloud.height = height;
        
      const float inv_fx  = 1.0 / K.at<float>(0, 0);
      const float inv_fy  = 1.0 / K.at<float>(1, 1);
      const float ox      = K.at<float>(0, 2);
      const float oy      = K.at<float>(1, 2);
      
      bool isDense = true;
      
      for (size_t x = 0; x < width; x++)
      {
        for (size_t y = 0; y < height; y++)
        {
          float z = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000;   // Convert milimetres to metres here
          
          if (z < 0.2 || z > 5.0)
          {
            cloud.at(x,y).x = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).y = std::numeric_limits<float>::quiet_NaN();
            cloud.at(x,y).z = std::numeric_limits<float>::quiet_NaN();          
            isDense = false;
          }
          else
          {
            cloud.at(x,y).x = (x-ox)*z*inv_fx;
            cloud.at(x,y).y = (y-oy)*z*inv_fy;
            cloud.at(x,y).z = z;
          }
        } 
      }   
      
      cloud.is_dense = isDense;
    }
    
    /** \brief Write depth distortion parameters.
     *  \param[in]  dirname directory where depth distortion maps are saved
     *  \param[in]  dm_maps depth distortion maps
     *  \param[in]  dm_distances depth distortion map distances
     */
    inline
    bool writeDepthDistortionParameters ( const std::string &dm_filename, const std::vector<cv::Mat> &dm_maps, const std::vector<float> &dm_distances)
    {
      // Check distortion map validity
      if (dm_maps.size() == 0)
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] depth multiplier maps are empty. Nothing to save." << std::endl;
        return false;
      }
      
      if (dm_maps.size() != dm_distances.size())
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] number of depth multiplier maps and distances must be the same." << std::endl;
        return false;
      }
      
      cv::FileStorage fs(dm_filename, cv::FileStorage::WRITE);
      if( fs.isOpened() )
      {
        fs << "num_dm_maps" << static_cast<int>(dm_distances.size());
        for (size_t dmId = 0; dmId < dm_distances.size(); dmId++)
        {
          fs << "dm_" + std::to_string(dmId) + "_distance"    << dm_distances[dmId];
          fs << "dm_" + std::to_string(dmId) + "_multipliers" << dm_maps[dmId];
        }
      }
      else
      {
        std::cout << "[utl::kinect::writeDepthDistortionParameters] Could not open file to save parameters." << std::endl;
        std::cout << "[utl::kinect::writeDepthDistortionParameters] Calibration file: '" << dm_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;      
    }
    
    /** \brief Write depth distortion parameters.
     *  \param[in]  dirname directory where depth distortion maps are saved
     *  \param[in]  dm_maps depth distortion maps
     *  \param[in]  dm_distances depth distortion map distances
     */
    inline
    bool readDepthDistortionParameters ( const std::string &dm_dilename, std::vector<cv::Mat> &dm_maps, std::vector<float> &dm_distances)
    {
      cv::FileStorage fs(dm_dilename, cv::FileStorage::READ);
      if( fs.isOpened() )
      {
        int num_dm_maps;
        fs["num_dm_maps"]    >> num_dm_maps;
        
        dm_distances.resize(num_dm_maps);
        dm_maps.resize(num_dm_maps);
        
        for (size_t dmId = 0; dmId < num_dm_maps; dmId++)
        {
          fs["dm_" + std::to_string(dmId) + "_distance"]    >> dm_distances[dmId];
          fs["dm_" + std::to_string(dmId) + "_multipliers"] >> dm_maps[dmId];
        }
      }
      else
      {
        std::cout << "[utl::kinect::readDepthDistortionParameters] Could not open file to load parameters." << std::endl;
        std::cout << "[utl::kinect::readDepthDistortionParameters] Calibration file: '" << dm_dilename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }    
    
    /** \brief Correct depth distortion in a kinect depth image.
     *  \param[in]  depth OpenCV depth image (CV_16U where depth is expressed in milimetres)
     *  \param[out] depth_undistorted a 1xN CV_32FC2 matrix where of ideal coordinates of depth image pixels. Pixels are ordered by scanning the rows.
     *  \param[out] cloud PCL pointcloud
     */
    inline
    void correctDepthDistortion ( const cv::Mat& depth, cv::Mat& depth_corrected, const std::vector<cv::Mat> &dm_maps, const std::vector<float> &dm_distances)
    {
      // Check input
      if (depth.type() != CV_16U)
      {
        std::cout << "[utl::kinect::correctDepthDistortion] depth image must be of type CV_16U." << std::endl;
        abort;
      }
      
      if (dm_maps.size() == 0)
      {
        std::cout << "[utl::kinect::correctDepthDistortion] depth multiplier maps are empty returning depth unmodified." << std::endl;
        depth_corrected = depth.clone();
        return;
      }
      
      if (dm_maps.size() != dm_distances.size())
      {
        std::cout << "[utl::kinect::correctDepthDistortion] number of depth multiplier maps and distances must be the same." << std::endl;
        abort;
      }
      
      if (dm_maps[0].size() != depth.size())
      {
        std::cout << "[utl::kinect::correctDepthDistortion] size of depth map and depth multiplier maps must be the same." << std::endl;
        abort;
      }

      // Correct depth
      depth_corrected = cv::Mat(depth.size(), depth.type());
      
      for (size_t x = 0; x < depth.cols; x++)
      {
        for (size_t y = 0; y < depth.rows; y++)
        {
          float d_estimated, d_modifier;
          d_estimated = static_cast<float>(depth.at<unsigned short>(y,x)) / 1000.0f;
          d_modifier = 1.0f;
          
          // Update map
          std::pair<int,int> mapIndices = utl::stdvec::nearestValues(dm_distances, d_estimated);          
          
          if (mapIndices.first != -1 && mapIndices.second != -1)
          {
            float weight = (d_estimated - dm_distances[mapIndices.first]) / (dm_distances[mapIndices.second] - dm_distances[mapIndices.first]);
            d_modifier = dm_maps[mapIndices.first].at<float>(y,x) * (1-weight) + dm_maps[mapIndices.second].at<float>(y,x) * weight;
          }
          else if (mapIndices.first != -1 && mapIndices.second == -1)
          {
            d_modifier = dm_maps[mapIndices.first].at<float>(y,x);
          }
          else if (mapIndices.first == -1 && mapIndices.second != -1)
          {
            d_modifier = dm_maps[mapIndices.second].at<float>(y,x);
          }
          
          depth_corrected.at<unsigned short>(y,x) = static_cast<unsigned short>(d_estimated * d_modifier * 1000.0f);
        }
      }
    }
  }
}

#endif    // KINECT_HPP
