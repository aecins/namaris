#ifndef OPENCV_CALIBRATION_UTILITIES_HPP
#define OPENCV_CALIBRATION_UTILITIES_HPP

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/version.hpp>

namespace utl
{
  namespace ocvcalib
  {
    //--------------------------------------------------------------------------
    // OpenCV Eigen pose conversions
    //--------------------------------------------------------------------------
    
    /** \brief Convert a pose represented by an OpenCV rotation and translation
     * matrices to an Eigen 3D affine transform.
     *  \param[in] R  OpenCV matrix containing rotation matrix
     *  \param[in] t  OpenCV matrix containing translation vector
     *  \param[out] T  pose represented in an Eigen::Affine3f transform
     */
    template <class Scalar>
    inline
    void cvPose2eigPose ( const cv::Mat &R,  const cv::Mat &t, Eigen::Transform<Scalar, 3, Eigen::Affine> &T )
    {
      Eigen::Matrix< Scalar, 3, 3> R_eig;
      Eigen::Matrix< Scalar, 3, 1> t_eig;
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
    template <class Scalar>    
    inline
    void eigPose2cvPose ( const Eigen::Transform<Scalar, 3, Eigen::Affine> T, cv::Mat &R, cv::Mat &t )
    {      
      cv::eigen2cv(T.rotation(), R);
      Eigen::Matrix< Scalar, 3, 1> t_eig = T.translation();
      cv::eigen2cv(t_eig, t);
    }    

    //--------------------------------------------------------------------------
    // Calibration target
    //--------------------------------------------------------------------------
       
    /** \brief Structure for holding calibration board parameters
     */
    struct CalibTarget
    {
      cv::Size size_;        /**< number of squares */
      float squareSize_;     /**< width of the square in x direction */
    };
            
    /** \brief Generate 3D points corresponding to the corners of a calibration
     * target. Generated 3D points start at (0,0,0) and lie in the positive 
     * quadrant of the XY plane.
     *  \param[in]  target            calibration target
     *  \param[out] target_points_3D  3D coordinates of target corner points
     */
    void generateTargetPoints3D ( const utl::ocvcalib::CalibTarget &target, std::vector<cv::Point3f> &target_points_3D)
    {
      target_points_3D.resize(0);
      for( int i = 0; i < target.size_.height; i++ )
        for( int j = 0; j < target.size_.width; j++ )
          target_points_3D.push_back(cv::Point3f(float(j*target.squareSize_), float(i*target.squareSize_), 0));
    }

    /** \brief Generate 3D points corresponding to the bounding box of a
     * calibration target. Generated 3D points start at (0,0,0) and lie in the
     * positive  quadrant of the XY plane.
     *  \param[in]  target            calibration target
     *  \param[out] target_points_3D  3D coordinates of target bounding box
     */
    void generateTargetBoundingBoxPoints3D ( const utl::ocvcalib::CalibTarget &target, std::vector<cv::Point3f>& corners)
    {
      corners.resize(4);
      corners[0] = cv::Point3f(0.0f, 0.0f, 0.0f);
      corners[1] = cv::Point3f(float((target.size_.width-1)*target.squareSize_), 0.0f, 0.0f);
      corners[2] = cv::Point3f(float((target.size_.width-1)*target.squareSize_), float((target.size_.height-1)*target.squareSize_), 0.0f);
      corners[3] = cv::Point3f(0, float((target.size_.height-1)*target.squareSize_), 0.0f);
    }
    
    /** \brief Find coordinataes of calibration target corner points in an
     * image.
     *  \param[in]  image         image
     *  \param[in]  target        calibration target
     *  \param[out] target_points_2D          2D coordinates of detected target corner points
     *  \param[out] target_points_2D_refined  refined 2D coordinates of detected target corner points
     */
    bool detectTarget (const cv::Mat &image, const CalibTarget &target, std::vector<cv::Point2f> &target_points_2D, std::vector<cv::Point2f> &target_points_2D_refined)
    {
      // Resize image
      cv::Mat imageResized;
      float scale = 640.0f / static_cast<double>(image.cols);
    //   double scale = 1.0;
      cv::resize(image, imageResized, cv::Size(), scale, scale);
        
      // Detect chessboard corners
      bool found = cv::findChessboardCorners( imageResized, target.size_, target_points_2D,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
      
      if (!found)
        return false;
      
      // Scale detected corners
      for (auto cornerIt = target_points_2D.begin(); cornerIt != target_points_2D.end(); cornerIt++)
      {
        (*cornerIt).x /= scale;
        (*cornerIt).y /= scale;
      }
      
      // Refine detected corners
      target_points_2D_refined = target_points_2D;
      cv::cornerSubPix( image, target_points_2D_refined, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 0.01 ));
      
      return true;
    }
    
    //--------------------------------------------------------------------------
    // Calibration
    //--------------------------------------------------------------------------    
        
    /** \brief Intrinsic calibration of a camera.
     *  \param[in]  image_points      coordinates of the detected target corner points in the 2D images
     *  \param[in]  image_size        size of the image
     *  \param[in]  target            calibration target
     *  \param[out] K                 camera matrix
     *  \param[out] d                 distortion coefficients
     *  \param[out] target_poses      poses of the detected targets (transformation that bring target 3D points to their pose in camera coordinate system)
     *  \param[out] avg_reproj_error  average reprojection error
     *  \param[in]  calibration_flags calibration flags
     *  \return TRUE if calibration was successfull
     */
    bool  intrinsicCalibration  ( const std::vector<std::vector<cv::Point2f> > image_points, const cv::Size &image_size,
                                  const utl::ocvcalib::CalibTarget &target,
                                  cv::Mat& K, cv::Mat& d,
                                  std::vector<Eigen::Affine3f> target_poses,
                                  double &avg_reproj_error,
                                  const int calibration_flags = 0
                                )
    {
      // Generate chessboard points
      std::vector<std::vector<cv::Point3f> > objectPoints(1);
      utl::ocvcalib::generateTargetPoints3D(target, objectPoints[0]);
      objectPoints.resize(image_points.size(),objectPoints[0]);

      // Calibrate
      std::vector<cv::Mat> rvecs, tvecs;
      avg_reproj_error = cv::calibrateCamera(objectPoints, image_points, image_size, K,
                      d, rvecs, tvecs, calibration_flags);
      
      // Check that calibration results are in the valid range
      bool ok = cv::checkRange(K) && cv::checkRange(d);
      
      // Convert target poses to Eigen
      target_poses.resize(image_points.size());
      for (size_t imId = 0; imId < image_points.size(); imId++)
      {
        cv::Mat R;
        cv::Rodrigues(rvecs[imId], R);
        utl::ocvcalib::cvPose2eigPose(R, tvecs[imId], target_poses[imId]);
      }      

      return ok;
    }
    
    /** \brief Stereo calibration.
     *  \param[in]  image_points_cam1 target points detected in images from first camera
     *  \param[in]  image_points_cam2 target points detected in images from second camera
     *  \param[in]  image_size        size of the image
     *  \param[in]  target            calibration target
     *  \param[out] K_cam1            camera matrix for first camera
     *  \param[out] K_cam2            camera matrix for second camera
     *  \param[out] d_cam1            distortion coefficients for first camera
     *  \param[out] d_cam2            distortion coefficients for second camera
     *  \param[out] R                 3x3 rotation matrix from second camera to first camera
     *  \param[out] t                 3x1 translation vector from second camera to first camera
     *  \param[out] E
     *  \param[out] F
     *  \param[out] avg_reproj_error  average reprojection error
     *  \param[in]  calibration_flags calibration flags
     *  \return TRUE if calibration was successfull
     */
    bool stereoCalibration  ( const std::vector<std::vector<cv::Point2f> > &image_points_cam1,  const std::vector<std::vector<cv::Point2f> > &image_points_cam2, const cv::Size &image_size,
                              const utl::ocvcalib::CalibTarget &target,
                              cv::Mat& K_cam1, cv::Mat& d_cam1,
                              cv::Mat& K_cam2, cv::Mat& d_cam2,
                              cv::Mat &R, cv::Mat &T,
                              cv::Mat &E, cv::Mat &F,
                              double& avg_reproj_error,
                              const int calibration_flags = 0
                            )
    {
      // Generate chessboard 3D points
      std::vector<std::vector<cv::Point3f> > objectPoints(1);
      generateTargetPoints3D(target, objectPoints[0]);
      objectPoints.resize(image_points_cam1.size(),objectPoints[0]);
      
      // Calibrate
      avg_reproj_error = cv::stereoCalibrate  ( objectPoints, image_points_cam1, image_points_cam2,
                                                K_cam1, d_cam1,
                                                K_cam2, d_cam2,
                                                image_size,
                                                R, T, E, F,
#if CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR >= 1
                                                calibration_flags,
                                                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6)
#else
                                                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6),
                                                calibration_flags
#endif
                                              );
      
      return true;
    }    
    
    /** \brief Write single camera intrinsic calibration parameters to a file.
     *  \param[in] calib_filename   path to the calibration file
     *  \param[in] K                camera matrix
     *  \param[in] d                camera distortion parameters
     *  \param[in] cam_name         name of camera
     *  \return TRUE if parameters were saved successfully
     */
    inline
    bool writeIntrinsicCalibrationParameters  ( const std::string& calib_filename,
                                                const cv::Mat& K, const cv::Mat& d, const cv::Size &image_size,
                                                const std::string &cam_name = "cam"
                                              )
    {
      cv::FileStorage fs( calib_filename, cv::FileStorage::WRITE );
      if( fs.isOpened() )
      {
        fs << "size_" + cam_name << image_size  <<  "K_" + cam_name  << K << "d_" + cam_name << d;
      }
      else
      {
        std::cout << "[utl::kinect::writeIntrinsicCalibrationParameters] Could not open calibration file to save parameters." << std::endl;
        std::cout << "[utl::kinect::writeIntrinsicCalibrationParameters] Calibration file: '" << calib_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }

    /** \brief Read single camera intrinsic calibration parameters from a file.
     *  \param[in] calib_filename   path to the calibration file
     *  \param[in] K                camera matrix
     *  \param[in] d                camera distortion parameters
     *  \param[in] cam_name         name of camera
     *  \return TRUE if parameters were loaded successfully
     */
    inline
    bool readIntrinsicCalibrationParameters ( const std::string& calib_filename,
                                              cv::Mat& K, cv::Mat& d, cv::Size &image_size,
                                              const std::string &cam_name = "cam"
                                            )
    {
      cv::FileStorage fs(calib_filename, cv::FileStorage::READ);
      if( fs.isOpened() )
      {
        fs["K_" + cam_name]    >> K;
        fs["d_" + cam_name]    >> d;
        fs["size_" + cam_name] >> image_size;
      }
      else
      {
        std::cout << "[utl::kinect::readIntrinsicCalibrationParameters] Could not open calibration file to load parameters." << std::endl;
        std::cout << "[utl::kinect::readIntrinsicCalibrationParameters] Calibration file: '" << calib_filename << "'." << std::endl;
        return false;
      }
      
      fs.release();
      return true;
    }
      
    /** \brief Write stereo camera calibration parameters to a file.
     *  \param[in] calib_filename   path to the calibration file
     *  \param[in] K_cam1           camera 1 matrix
     *  \param[in] K_cam2           camera 2 matrix
     *  \param[in] d_cam1           camera 1 distortion parameters
     *  \param[in] d_cam2           camera 2 distortion parameters
     *  \param[in] size_cam1        image size for first camera
     *  \param[in] size_cam2        image size for second camera
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
                                            const cv::Mat &cam2_to_cam1_R, const cv::Mat &cam2_to_cam1_t,
                                            const std::string &cam1_name = "cam1",
                                            const std::string &cam2_name = "cam2"
                                          )
    {
      cv::FileStorage fs(calib_filename, cv::FileStorage::WRITE);
      if( fs.isOpened() )
      {
        fs << "size_" + cam1_name << size_cam1  <<  "K_" + cam1_name  << K_cam1 << "d_" + cam1_name << d_cam1 <<
              "size_" + cam2_name << size_cam2  <<  "K_" + cam2_name  << K_cam2 << "d_" + cam2_name << d_cam2   <<
              cam1_name + "_to_" + cam2_name + "_R" <<  cam2_to_cam1_R  <<
              cam1_name + "_to_" + cam2_name + "_t" <<  cam2_to_cam1_t;
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

    /** \brief Read stereo camera calibration parameters from a file.
     *  \param[in]  calib_filename   path to the calibration file
     *  \param[out] K_cam1           camera 1 matrix
     *  \param[out] K_cam2           camera 2 matrix
     *  \param[out] d_cam1           camera 1 distortion parameters
     *  \param[out] d_cam2           camera 2 distortion parameters
     *  \param[out] size_cam1        image size for first camera
     *  \param[out] size_cam2        image size for second camera
     *  \param[out] cam1_to_cam2_R   rotation from camera 1 pose to camera 2 pose
     *  \param[out] cam1_to_Cam2_t   translation from camera 1 pose to camera 2 pose
     *  \param[in]  cam1_name        name of camera 1
     *  \param[in]  cam2_name        name of camera 2
     *  \return TRUE if parameters were loaded successfully
     */
    inline
    bool readStereoCalibrationParameters  ( const std::string &calib_filename,
                                            cv::Mat &K_cam1, cv::Mat &K_cam2,
                                            cv::Mat &d_cam1, cv::Mat &d_cam2,
                                            cv::Size &size_cam1, cv::Size &size_cam2,
                                            cv::Mat &cam2_to_cam1_R, cv::Mat &cam2_to_cam1_t,
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
        fs[cam1_name + "_to_" + cam2_name + "_R"] >>  cam2_to_cam1_R;
        fs[cam1_name + "_to_" + cam2_name + "_t"] >>  cam2_to_cam1_t;
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

    //--------------------------------------------------------------------------
    // Undistortion
    //--------------------------------------------------------------------------    
    
    /** \brief Get the ideal coordinates for all pixels in an image.
     *  \param[in]  image_size size of an image
     *  \param[in]  K camera matrix
     *  \param[in]  d distortion coefficients
     *  \param[out] ideal_pixel_coordinates ideal coordinates of pixels stored in a 1xN CV_32FC2 matrix. First coordinate corresponds to x and second to y.
     */
    inline
    void getIdealPixelCoordinates  (const cv::Size& image_size, const cv::Mat& K, const cv::Mat& d, cv::Mat &ideal_pixel_coordinates  )
    {
      cv::Mat pixel_coordinates (image_size.area(), 1, CV_32FC2);
      
      int pixId = 0;
      for (size_t x = 0; x < static_cast<size_t>(image_size.width); x++)
        for (size_t y = 0; y < static_cast<size_t>(image_size.height); y++)
        {
          pixel_coordinates.at<cv::Vec2f>(pixId)[0] = static_cast<float>(x);
          pixel_coordinates.at<cv::Vec2f>(pixId)[1] = static_cast<float>(y);
          pixId++;
        }
        
      cv::undistortPoints(pixel_coordinates, ideal_pixel_coordinates, K, d);
    }
    
    /** \brief Undistort an image given ideal pixel coordinate maps produced by
     * cv::initUndistortRectifyMap(). Optionally a pixel mask can be provided 
     * that specifies the valid pixels in the distorted image. All the pixels in
     * the undistorted image that were interploated from at least one invalid 
     * original pixel will be set to zero.
     *  \param[in]  image         input image
     *  \param[in]  map_x         ideal x coordinates of the undistorted image
     *  \param[in]  map_y         ideal y coordinates of the undistorted image
     *  \param[in]  valid_pixels  a mask specifying valid pixels in the original image
     *  \return   undistorted image
     */    
    cv::Mat undistort (const cv::Mat &image, const cv::Mat &map_x, const cv::Mat &map_y, const cv::Mat &valid_pixels = cv::Mat())
    {
      //------------------------------------------------------------------------
      // Check input
      if (image.size() != map_x.size() || image.size() != map_y.size())
      {
        std::cout << "[utl::ocvcalib::undistort] pixel maps must be same size as image." << std::endl;
        std::abort();
      }
      
      if (valid_pixels.data)
      {
        if (valid_pixels.size() != image.size())
        {
          std::cout << "[utl::ocvcalib::undistort] discontinuity mask must be same size as input image." << std::endl;
          std::abort();
        }
        
        if (valid_pixels.type() != CV_8UC1)
        {
          std::cout << "[utl::ocvcalib::undistort] discontinuity mask must be of type CV_8UC1." << std::endl;
          std::abort();
        }
      }
      
      //------------------------------------------------------------------------
      // Undistort image
      
      cv::Mat image_undistorted;
      cv::remap (image, image_undistorted, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
      
      //------------------------------------------------------------------------
      // Apply discontinuity mask
      
      cv::Mat image_undistorted_final;
      
      if (valid_pixels.data)
      {
        // Undistort valid pixel mask
        cv::Mat valid_pixels_undistorted;
        cv::remap (valid_pixels, valid_pixels_undistorted, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT );
        
        // Set all pixels below 255 as invalid
        cv::threshold(255-valid_pixels_undistorted, valid_pixels_undistorted, 0, 255, CV_THRESH_BINARY_INV);
                
        // Mask invalid pixels
        image_undistorted_final = cv::Mat::zeros(image.size(), CV_16UC1);
        image_undistorted.copyTo(image_undistorted_final, valid_pixels_undistorted);
      }
      else
      {
        image_undistorted_final = image_undistorted;
      }
      
      return image_undistorted_final;
    }    
    
    //--------------------------------------------------------------------------
    // Miscellaneous
    //--------------------------------------------------------------------------    
    
    /** \brief Find the transformation that brings 3D target points to the pose
     * of the detected target.
     *  \param[in] target_points_2D   detected target points in 3D image
     *  \param[in] target_points_3D   3D target points in model coordinate system
     *  \param[in] K                  camera matrix
     *  \param[in] d                  camera distrortion coefficients
     *  \return pose of the target in camera coordinate system
     */    
    Eigen::Affine3f getTargetPose ( const std::vector<cv::Point2f> &target_points_2D,
                                    const std::vector<cv::Point3f> &target_points_3D,
                                    const cv::Mat &K, const cv::Mat &d
                                  )
    {
      cv::Mat R_cv, t_cv;
      cv::solvePnP(target_points_3D, target_points_2D, K, d, R_cv, t_cv);
      cv::Rodrigues(R_cv, R_cv);
      
      Eigen::Matrix3f R_tmp;
      Eigen::Vector3f t_tmp;
      cv::cv2eigen(R_cv, R_tmp);
      cv::cv2eigen(t_cv, t_tmp);
      
      Eigen::Affine3f T;
      T.linear() = R_tmp;
      T.translation() = t_tmp;
      
      return T;
    }
    
    /** \brief Given a 3D polygon find the corresponding 2D mask in the image
     * plane.
     *  \param[in]  polygon_points  3D points of the polygon in world coordinate system
     *  \param[in]  K               camera matrix
     *  \param[in]  d               camera distortion coefficients
     *  \param[in]  image_size      size of the image
     *  \param[in]  camera_pose     camera pose in world coordinate system
     */
    cv::Mat get3DpolygonImageMask ( const std::vector<cv::Point3f> &polygon_points,
                                    const cv::Mat &K, const cv::Mat &d, const cv::Size &image_size, const Eigen::Affine3f &camera_pose
                                  )
    {      
      std::vector<cv::Point2f> image_points;
      
      // Convert camera pose to OpenCV
      cv::Mat R, rvec, tvec;
      utl::ocvcalib::eigPose2cvPose(camera_pose.inverse(), R, tvec);
      cv::Rodrigues(R, rvec);
      
      // NOTE: lens distortion models used by OpenCV are only valid for points that 
      // are roughly within the view frustum of the camera. Thus if we want to find
      // the 2D coordinates of a world point that is too far from the camera frustum
      // we will get incorrect results. To combat this problem in this function a
      // a polygon defined in world coordinates is projected to the image plane 
      // coordinates in two steps.
      //  - first points are projected with zero distortion parameters.
      //  - if the resulting image cootdinates are too far away from the camera 
      //  camera frustum the no distortion result is accepted.
      //  - otherwise projection is run for the second time, now with the supplied
      // distortion parameters.
      cv::projectPoints(polygon_points, rvec, tvec, K, cv::Mat::zeros(1, 4, CV_64F), image_points);
        
      float dx = static_cast<float> (image_size.width) / 4;
      float dy = static_cast<float> (image_size.height) / 4;
      
      bool good = true;
      for (size_t pointId = 0; pointId < image_points.size(); pointId++)
      {
        if (  ! ( image_points[pointId].x < (static_cast<float>(image_size.width) + dx)   &&
                  image_points[pointId].x > -dx                                           &&
                  image_points[pointId].y < (static_cast<float>(image_size.height) + dy)  &&
                  image_points[pointId].y > -dy                                           )
          )
        {
          good = false;
          break;
        }
      }
      
      if (good)
        cv::projectPoints(polygon_points, rvec, tvec, K, d, image_points);
      
      // Create image mask
      cv::Mat mask = cv::Mat::zeros(image_size, CV_8U);
      std::vector<std::vector<cv::Point> > tmp (1);
      tmp[0].resize(image_points.size());
      for (size_t pointId = 0; pointId < image_points.size(); pointId++)
        tmp[0][pointId] = cv::Point(static_cast<int>(image_points[pointId].x), static_cast<int>(image_points[pointId].y));
      cv::fillPoly(mask, tmp, cv::Scalar(255));
      
      return mask;
    }    
    
    //--------------------------------------------------------------------------
    // Visualization
    //--------------------------------------------------------------------------    
    
    ////////////////////////////////////////////////////////////////////////////////
    void getCornerBoundingBox(const cv::Size &image_size, const std::vector<cv::Point2f> &corners, cv::Rect &roi, const float boundary_margin)
    {
      // Find the max coordinates of the corners
      float cornerMax_x = 0.0;
      float cornerMax_y = 0.0;
      float cornerMin_x = static_cast<float>(image_size.width);
      float cornerMin_y = static_cast<float>(image_size.height);
        
      for (size_t cornerId = 0; cornerId < corners.size(); cornerId++)
      {
        cornerMax_x = std::max(cornerMax_x, corners[cornerId].x);
        cornerMax_y = std::max(cornerMax_y, corners[cornerId].y);
        cornerMin_x = std::min(cornerMin_x, corners[cornerId].x);
        cornerMin_y = std::min(cornerMin_y, corners[cornerId].y);
      }
        
      // Find the cutout
      float dx = cornerMax_x - cornerMin_x;
      float dy = cornerMax_y - cornerMin_y;
      cornerMax_x += dx * boundary_margin;
      cornerMax_y += dy * boundary_margin;
      cornerMin_x -= dx * boundary_margin;
      cornerMin_y -= dy * boundary_margin;
      
      cornerMax_x = std::min(cornerMax_x, static_cast<float>(image_size.width));
      cornerMax_y = std::min(cornerMax_y, static_cast<float>(image_size.height));
      cornerMin_x = std::max(cornerMin_x, 0.0f);
      cornerMin_y = std::max(cornerMin_y, 0.0f);
      
      cornerMax_x = std::ceil (cornerMax_x);
      cornerMax_y = std::ceil (cornerMax_y);
      cornerMin_x = std::floor(cornerMin_x);
      cornerMin_y = std::floor(cornerMin_y);
          
      // Create ROI
      cv::Point2f topLeft   (cornerMin_x, cornerMin_y);
      cv::Point2f botRight  (cornerMax_x, cornerMax_y);  
      roi = cv::Rect(topLeft, botRight);
    }

    ////////////////////////////////////////////////////////////////////////////////
    void drawChessboardCorners  (const cv::Mat &image, const cv::Size &board_size, const std::vector<cv::Point2f> &corners, cv::Mat &corner_image)
    {
      int targetWidth = 800;
      
      // Get the bounding box for corners
      cv::Rect roi;
      getCornerBoundingBox(image.size(), corners, roi, 0.3);  
      
      // Cut out image
      image.copyTo(corner_image);
      image(roi).copyTo(corner_image);
      
      // Resize image
      float scaleFactor = targetWidth / static_cast<float>(roi.width);
      int targetHeight = static_cast<int>(static_cast<float>(roi.height) * scaleFactor);
      cv::resize(corner_image, corner_image, cv::Size(targetWidth, targetHeight));
      
      // Modify corners
      std::vector<cv::Point2f> cornersCropped(corners.size());
      for (size_t cornerId = 0; cornerId < corners.size(); cornerId++)
        cornersCropped[cornerId] = (corners[cornerId] - cv::Point2f(roi.x, roi.y)) * scaleFactor;
      
      // Draw corners
      cv::cvtColor(corner_image, corner_image, cv::COLOR_GRAY2BGR);
      cv::drawChessboardCorners( corner_image, board_size, cv::Mat(cornersCropped), true);
    }
  }
}


#endif    // OPENCV_CALIBRATION_UTILITIES_HPP
