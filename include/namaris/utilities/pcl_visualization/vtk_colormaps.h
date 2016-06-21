#ifndef VTK_COLORMAPS_H
#define VTK_COLORMAPS_H

// VTK
#include <vtkSmartPointer.h>
#include <vtkLookupTable.h>

// Eigen
#include <Eigen/Dense>

// CPP tools
#include <utilities/pcl_visualization/color.h>
#include <utilities/pcl_visualization/vtk_colormaps_lut.hpp>


namespace utl
{
  namespace pclvis
  {
    class Colormap
    {    
      public:
        
        enum COLORMAP_TYPE {VTK_DEFAULT, JET, GRAYSCALE};
        
        // Constructors
        Colormap ()
          : colormapType_ (JET)
          , minVal_ (std::numeric_limits<double>::quiet_NaN())
          , maxVal_ (std::numeric_limits<double>::quiet_NaN())
        {};
        
        Colormap (COLORMAP_TYPE colormapType)
          : colormapType_ (colormapType)
          , minVal_ (std::numeric_limits<double>::quiet_NaN())
          , maxVal_ (std::numeric_limits<double>::quiet_NaN())
        {};
        
        Colormap (COLORMAP_TYPE colormapType, double minVal, double maxVal)
          : colormapType_ (colormapType)
          , minVal_ (minVal)
          , maxVal_ (maxVal)
        {};    
              
        // Set colormap type
        void setColormapType (COLORMAP_TYPE colormapType);
        
        // Manually set range limits
        void setRangeLimits (double minVal, double maxVal);
        
        // Reset range limits to undefined
        void resetRangeLimits();

        // Set range limits from data
        template <typename Scalar>
        void setRangeLimitsFromData (const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data);

        // Set range limits from data
        template <typename Scalar>
        void setRangeLimitsFromData (const std::vector<Scalar> &data);

        // Get vtk lookup table representing the colormap      
        vtkSmartPointer<vtkLookupTable> getColorLookupTable() const;
        
        // Map each element of a scalar vector to RGB values in [0,1] range given the colormap
        template <typename Scalar>
        Colors getColorsFromData (const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data);

        // Map each element of a scalar vector to RGB values in [0,1] range given the colormap
        template <typename Scalar>
        Colors getColorsFromData (const std::vector<Scalar> &data);
        
      private:
        
        static const int COLORMAP_NUM_DIVISIONS = 256;
        COLORMAP_TYPE colormapType_;
        double minVal_;
        double maxVal_;
        
        // Get specific colormaps
        vtkSmartPointer<vtkLookupTable> getLUT_Jet () const;
        vtkSmartPointer<vtkLookupTable> getLUT_VtkDefault () const;
        vtkSmartPointer<vtkLookupTable> getLUT_Grayscale () const;
        
    };
  }
}

#endif  // VTK_COLORMAPS_H