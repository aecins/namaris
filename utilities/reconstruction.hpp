#ifndef RECONSTRUCTION_UTILITIES_HPP
#define RECONSTRUCTION_UTILITIES_HPP

#include <calibu/cam/Rectify.h>
#include <calibu/utils/Range.h>

namespace utl 
{
  namespace rec
  {
    //////////////////////////////////////////////////////////////////////////////
    // Create lookup table for rectification
    void CreateLookupTableMod(
        const calibu::CameraModelInterface& cam_from,
        const Eigen::Matrix3d& R_onKinv,
        calibu::LookupTable& lut
        )
    {
      const int w = cam_from.Width();
      const int h = cam_from.Height();

      for( int r = 0; r < h; ++r) {
        for( int c = 0; c < w; ++c) {
          // Remap
          const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
          Eigen::Vector2d p_warped = cam_from.Project(p_o);

          bool outside = (p_warped[0] > w - 1.0 || p_warped[0] < 0.0 || p_warped[1] > h - 1.0 || p_warped[1] < 0.0);
          
          // Clamp to valid image coords. This will cause out of image
          // data to be stretched from nearest valid coords with
          // no branching in rectify function.
          p_warped[0] = std::min(std::max(0.0, p_warped[0]), w - 1.0 );
          p_warped[1] = std::min(std::max(0.0, p_warped[1]), h - 1.0 );

          // Truncates the values for the left image
          int u  = (int) p_warped[0];
          int v  = (int) p_warped[1];
          float su = p_warped[0] - (double)u;
          float sv = p_warped[1] - (double)v;

          // Fix pixel access for last row/column to ensure all are in bounds
          if(u == (w-1)) {
            u -= 1;
            su = 1.0;
          }
          if(v == (w-1)) {
            v -= 1;
            sv = 1.0;
          }

          if (outside)
          {
            calibu::BilinearLutPoint p;
            p.idx0 = u + v*w;
            p.idx1 = u + v*w + w;
            p.w00  = 0;
            p.w01  = 0;
            p.w10  = 0;
            p.w11  = 0;
            lut.SetPoint( r, c, p );          
          }
          else
          {
            // Pre-compute the bilinear interpolation weights
            calibu::BilinearLutPoint p;
            p.idx0 = u + v*w;
            p.idx1 = u + v*w + w;
            p.w00  = (1-su)*(1-sv);
            p.w01  =    su *(1-sv);
            p.w10  = (1-su)*sv;
            p.w11  =     su*sv;
            lut.SetPoint( r, c, p );
          }
        }
      }
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Rectify an OpenCV image
    template <typename T>
    void Rectify(
        const calibu::LookupTable& lut,
        const cv::Mat inputImage,
        cv::Mat outputImage
        )
    {
      // Check input data
      assert(inputImage.type() == outputImage.type());
      assert(inputImage.channels() == 1);

      
      // Make the most of the continuous block of memory!
      const calibu::BilinearLutPoint* ptr   = &lut.m_vLutPixels[0];

      const int nHeight = lut.Height();
      const int nWidth  = lut.Width();

      // Make sure we have been given a correct lookup table.
      assert(inputImage.cols == nWidth && inputImage.rows == nHeight);
      
      const T* pInputImageData = reinterpret_cast<T*>(inputImage.data);
      T* pOutputRectImageData = reinterpret_cast<T*>(outputImage.data);
      
      for( int nRow = 0; nRow < nHeight; nRow++ ) {
        for( int nCol = 0; nCol < nWidth; nCol++ ) {
          *pOutputRectImageData++ =
            (T) ( ptr->w00 * pInputImageData[ ptr->idx0 ] +
                ptr->w01 * pInputImageData[ ptr->idx0 + 1 ] +
                ptr->w10 * pInputImageData[ ptr->idx1 ] +
                ptr->w11 * pInputImageData[ ptr->idx1 + 1 ] );
          ptr++;
        }
      }
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Rectify an OpenCV image preserving depth discontinuities. If any of the 
    // pixels in the original image used to contruct a pixel in the rectified
    // image are occlusion pixels, that rectified pixel is marked as occlusion.
    template <typename T>
    void RectifyDepth(
        const calibu::LookupTable& lut,
        const cv::Mat inputImage,
        const cv::Mat disc,
        cv::Mat outputImage
        )
    {
      // Check input data
      assert(inputImage.type() == outputImage.type());
      assert(inputImage.channels() == 1);

      
      // Make the most of the continuous block of memory!
      const calibu::BilinearLutPoint* ptr   = &lut.m_vLutPixels[0];

      const int nHeight = lut.Height();
      const int nWidth  = lut.Width();

      // Make sure we have been given a correct lookup table.
      assert(inputImage.cols == nWidth && inputImage.rows == nHeight);
      
      const T* pInputImageData      = reinterpret_cast<T*>(inputImage.data);
      T* pOutputRectImageData       = reinterpret_cast<T*>(outputImage.data);
      unsigned char* pDiscImageData = disc.data;
      
      for( int nRow = 0; nRow < nHeight; nRow++ ) {
        for( int nCol = 0; nCol < nWidth; nCol++ ) {
          if (pDiscImageData[ ptr->idx0 ] > 0 || pDiscImageData[ ptr->idx1 ] > 0)
            *pOutputRectImageData++ = (T) (0);
          else
            *pOutputRectImageData++ =
              (T) ( ptr->w00 * pInputImageData[ ptr->idx0 ] +
                  ptr->w01 * pInputImageData[ ptr->idx0 + 1 ] +
                  ptr->w10 * pInputImageData[ ptr->idx1 ] +
                  ptr->w11 * pInputImageData[ ptr->idx1 + 1 ] );
          ptr++;
        }
      }
    }

    bool saveOpenNICalibParamsSupplementary ( const std::string filename,
                                              const double depth_ir_dx,
                                              const double depth_ir_dy,
                                              const double depth_scaling
                                            )
    {
      cv::FileStorage fs( filename, cv::FileStorage::WRITE );

      if (fs.isOpened())
      {
        fs << "depth_ir_dx" << depth_ir_dx;
        fs << "depth_ir_dy" << depth_ir_dy;
        fs << "depth_scaling" << depth_scaling;
        
        fs.release();
        return true;
      }
      
      fs.release();
      return false;
    }

    bool loadOpenNICalibParamsSupplementary ( const std::string filename,
                                              double &depth_ir_dx,
                                              double &depth_ir_dy,
                                              double &depth_scaling
                                            )
    {
      cv::FileStorage fs;
      if (!fs.open(filename, cv::FileStorage::READ))
      {
        fs.release();
        return false;
      }
      
      fs["depth_ir_dx"] >> depth_ir_dx;
      fs["depth_ir_dy"] >> depth_ir_dy;
      fs["depth_scaling"] >> depth_scaling;
      
      fs.release();
      return true;
    }
  }
} 

#endif    // RECONSTRUCTION_UTILITIES_HPP