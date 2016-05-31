#ifndef OPENCV_UTILITIES_HPP
#define OPENCV_UTILITIES_HPP

// STD
#include <iostream>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace utl
{
  namespace ocv
  {    
    /** \brief Create a lookup table for the blue to red colormap.
     *  \return a lookup table for the colormap
     */
    inline
    cv::Mat b2rColormapLUT ()
    {
      cv::Mat lut (256, 1, CV_8UC3);
      
      cv::Scalar blue   (255, 0, 0);
      cv::Scalar red    (0, 0, 255);
      cv::Scalar white  (255, 255, 255);
      
      for (size_t i = 0; i < 128; i++)
      {
        double weight = static_cast<float>(i) / 128;
        cv::Scalar curColor = white * weight + blue * (1 - weight);
        
        lut.at<cv::Vec3b>(i)[0] = static_cast<uchar>(curColor(0));
        lut.at<cv::Vec3b>(i)[1] = static_cast<uchar>(curColor(1));
        lut.at<cv::Vec3b>(i)[2] = static_cast<uchar>(curColor(2));
      }
      
      for (size_t i = 128; i < 256; i++)
      {
        double weight = static_cast<float>(i-128) / 128;
        cv::Scalar curColor = red * weight + white * (1 - weight);

        lut.at<cv::Vec3b>(i)[0] = static_cast<uchar>(curColor(0));
        lut.at<cv::Vec3b>(i)[1] = static_cast<uchar>(curColor(1));
        lut.at<cv::Vec3b>(i)[2] = static_cast<uchar>(curColor(2));
      }
      
      return lut;
    }

    /** \brief Apply colormap to a matrix. For now only color colormaps are supported.
     *  \param[in] src  source matrix. Must be either CV_8UC1 or CV_8UC3. If it is CV_8UC3 colormap is applied to grayscale image obtained from color image
     *  \param[in] dst  colormaped matrix CV_8UC1
     *  \param[in] lut  colormap represented as a 256x1 or 1x256 CV_8UC1 matrix
     */
    inline
    void applyColormap  (const cv::Mat &src, cv::Mat &dst, const cv::Mat &lut)
    {
      // Check input
      if (lut.type() != CV_8UC3)
      {
        std::cout << "[utl::ocv::applyColormap] colormap must by of type CV_8UC3." << std::endl;
        abort();
      }
      
      if (lut.size() != cv::Size(256, 1) && lut.size() != cv::Size(1, 256))
      {
        std::cout << "[utl::ocv::applyColormap] colormap must have size 1x256 or 256x1." << std::endl;
        abort();
      }
      
      if (src.type() != CV_8UC1 && src.type() != CV_8UC3)
      {
        std::cout << "[utl::ocv::applyColormap] source matrix must be of type CV_8U or CV_8UC3." << std::endl;
        abort();        
      }
      
      
      cv::Mat srcCopy = src.clone();
      
      if (src.type() == CV_8UC3)
      {
        cv::cvtColor(srcCopy.clone(), srcCopy, CV_BGR2GRAY);
      }
      
      // Apply colormap
      cv::cvtColor(srcCopy.clone(), srcCopy, CV_GRAY2BGR);      
      cv::LUT(srcCopy, lut, dst);
    }    
    
    /** \brief Generate an image with a visualization of a colormap
     *  \param[in] colormap a lookup table representing the colormap. Must have size 256x1 or 1x256 and be of type CV_8UC3
     *  \param[in] width    width of a bar representing each color
     *  \param[in] height   height of a bar representing each color
     */
    inline
    cv::Mat visualizeColormap  (const cv::Mat &colormap, const int width = 5, const int height = 100)
    {
      cv::Mat mat (height, width * 256, CV_8U);
  
      for (size_t i = 0; i < 256; i++)
      {
        cv::Mat roi(mat(cv::Rect(i *width, 0, width, height)));
        roi.setTo(i);
      }
  
      cv::Mat matColormapped;
      utl::ocv::applyColormap(mat, matColormapped, colormap);
      
      return matColormapped;
    }
    
    /** \brief Visualize matrix by converting it to CV_8U and rescaling values
     * such that minimum value corresponds to 0 and maximum value corresponds to
     * 255.
     *  \param[in] mat  input matrix
     *  \return matrix that can be visualized using cv::imshow()
     */    
    inline
    cv::Mat visualizeMatrix (const cv::Mat &im)
    {
      cv::Mat im_normalized;
      im.convertTo(im_normalized, CV_8U);
      cv::normalize(im_normalized.clone(), im_normalized, 0, 255, cv::NORM_MINMAX);
      return im_normalized;
    }

    /** \brief Rescale an image to a desired width. Both X and Y dimensions are
     * scaled by the same factor (up to discretization error).
     *  \param[in] image  image
     *  \param[in] new_width desired width of the image
     *  \return rescaled image
     */
    inline
    cv::Mat resize ( cv::Mat &image, const int new_width)
    {
      cv::Mat resized_image;
      if (image.cols > new_width)
      {
        float scaleFactor = static_cast<float>(new_width) / static_cast<float>(image.cols);
        int targetHeight = static_cast<int>(static_cast<float>(image.rows) * scaleFactor);
        cv::resize(image, resized_image, cv::Size(new_width, targetHeight));
      }
      else
      {
        resized_image = image.clone();
      }
      
      return resized_image;
    }
  }
}


#endif  // OPENCV_UTILITIES_HPP