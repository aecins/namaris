#ifndef PCL_VISUALIZATION_UTILITIES_HPP
#define PCL_VISUALIZATION_UTILITIES_HPP

// PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkScalarBarActor.h>
#include <vtkActor2DCollection.h>
#include <vtkLine.h>

// Utilities
#include <namaris/utilities/std_vector.hpp>
#include <namaris/utilities/graph.hpp>
#include <namaris/utilities/map.hpp>
#include <namaris/utilities/geometry.hpp>
#include <namaris/utilities/pcl_visualization/color.hpp>
// #include <namaris/utilities/pcl_visualization/vtk_colormaps.hpp>

namespace utl
{
  namespace pclvis
  {
    //----------------------------------------------------------------------------
    // Colors
    //----------------------------------------------------------------------------  

    // Frequently used colors
    const Color red   (1.0, 0.0, 0.0);
    const Color green (0.0, 1.0, 0.0);
    const Color blue  (0.0, 0.0, 1.0);
    const Color white (1.0, 1.0, 1.0);
    const Color black (0.0, 0.0, 0.0);
    const Color bgColor (0.7, 0.7, 1.0);    // Background color for the PCL visaualizer
    
    /** \brief Convert a single point with RGB information to grayscale
     *  \param[in,out]  point point to be converted to grayscale
     *  \note conversion is done using the formula used in OpenCV (http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html)
     */
    inline
    std::vector<Color> colorizeData ( const std::vector<float> &data,
                                      const int colormal_type = 0
                                    )
    {
      // Get LUT
      vtkSmartPointer<vtkLookupTable> lut;
      pcl::visualization::getColormapLUT(static_cast<pcl::visualization::LookUpTableRepresentationProperties>(colormal_type), lut);
      
      // Map data to colors
      std::vector<Color> colors (data.size());
      for (size_t i = 0; i < data.size(); i++)
      {
        double rgb[3];
        lut->GetColor(static_cast<double>(data[i]), rgb);
        colors[i] = Color(rgb[0], rgb[1], rgb[2]);
      }
      
      return colors;
    }
    
    //----------------------------------------------------------------------------
    // Set rendering properties
    //----------------------------------------------------------------------------  
    
    /** \brief modify the rendering properties of a single normal pointcloud
     *  \param[in] visualizer   visualizer object
     *  \param[in] id           the point cloud object id prefix
     *  \param[in] point_size   size of the points used for visualization
     *  \param[in] color        color of the cloud
     *  \param[in] opacity      opacity of the cloud
     */
    inline
    void setPointCloudRenderProps  (  pcl::visualization::PCLVisualizer &visualizer,
                                      const std::string &id,
                                      const float point_size = -1.0,
                                      const Color &color = Color(),
                                      const float opacity = -1.0
                                    )
    {
      if (point_size > 0)
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
      if (color != Color())
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
      if (opacity > 0)
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
    }

    /** \brief modify the rendering properties of a cloud of normals
     *  \param[in] visualizer   visualizer object
     *  \param[in] id           the point cloud object id prefix
     *  \param[in] line_width   width of normal arrows
     *  \param[in] color        color of the cloud
     *  \param[in] opacity      opacity of the cloud
     */
    inline
    void setNormalCloudRenderProps  ( pcl::visualization::PCLVisualizer &visualizer,
                                      const std::string &id,
                                      const float line_width = -1.0,
                                      const Color &color = Color(),
                                      const float opacity = -1.0
                                    )
    {
      if (line_width > 0)
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id);
      if (color != Color())
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
      if (opacity > 0)
        visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
    }

    /** \brief modify the rendering properties of a line shape
     *  \param[in] visualizer   visualizer object
     *  \param[in] id           the point cloud object id prefix (default: cloud)
     *  \param[in] line_width   width of normal arrows
     *  \param[in] color        color of the cloud
     *  \param[in] opacity      opacity of the cloud
     */
    inline
    void setShapeRenderProps  ( pcl::visualization::PCLVisualizer &visualizer,
                                const std::string &id,
                                const Color &color = Color(),
                                const float opacity = -1.0
                            )
    {
      if (color != Color())
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
      if (opacity > 0)
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
    }    

    /** \brief modify the rendering properties of a line shape
     *  \param[in] visualizer   visualizer object
     *  \param[in] id           the point cloud object id prefix (default: cloud)
     *  \param[in] line_width   width of normal arrows
     *  \param[in] color        color of the cloud
     *  \param[in] opacity      opacity of the cloud
     */
    inline
    void setLineRenderProps ( pcl::visualization::PCLVisualizer &visualizer,
                              const std::string &id,
                              const float line_width = -1.0,
                              const Color &color = Color(),
                              const float opacity = -1.0
                            )
    {
      if (line_width > 0)
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id);
      if (color != Color())
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
      if (opacity > 0)
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
    }    
    
    /** \brief modify the rendering properties of a colormap
     *  \param[in] visualizer     visualizer object
     *  \param[in] id             the point cloud object id prefix (default: cloud)
     *  \param[in] colormal_type  colormap used \ref pcl::visualization::LookUpTableRepresentationProperties. If -1 colormap is unchanged
     *  \param[in] range_auto     if true, colormap limits are set automatically from the data
     *  \param[in] range_min      range minimum (if NaN colormap range is not updated)
     *  \param[in] range_max      range maximum (if NaN colormap range is not updated)
     *  \note this function sets colormap properties for clouds. It should be able to set colormap properties for shapes as well
     */
    inline
    void setColormapRenderProps ( pcl::visualization::PCLVisualizer &visualizer,
                                  const std::string &id,
                                  const int colormal_type = -1,
                                  const bool  range_auto = false,
                                  const float range_min = std::numeric_limits<float>::quiet_NaN(),
                                  const float range_max = std::numeric_limits<float>::quiet_NaN()
                                )
    {
      if (colormal_type != -1)
        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, colormal_type, id);
      if (range_auto)
        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO, id);
      else if (!std::isnan(range_min) and !std::isnan(range_max))
        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, range_min, range_max, id);
    }    
    
    //----------------------------------------------------------------------------
    // Individual pointcloud visualization
    //----------------------------------------------------------------------------  

    /** \brief Show a single pointcloud
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  cloud       pointcloud
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  point_size  size of the points used for visualization
     *  \param[in]  color       color of the cloud
     *  \param[in]  opacity     opacity of the cloud
     */
    template <typename PointT>
    inline
    void showPointCloud ( pcl::visualization::PCLVisualizer &visualizer,
                          const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const std::string &id = "cloud",
                          const float point_size = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                        )
    {
      visualizer.addPointCloud<PointT>(cloud, id);
      setPointCloudRenderProps(visualizer, id, point_size, color, opacity);
    }

    /** \brief Show a single pointcloud with color information
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  cloud       pointcloud
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  point_size  size of the points used for visualization
     *  \param[in]  opacity     opacity of the cloud
     */
    template <typename PointT>
    inline
    void showPointCloudColor  ( pcl::visualization::PCLVisualizer &visualizer,
                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const std::string &id = "cloud",
                                const float point_size = -1.0,
                                const float opacity = -1.0
                              )
    {
      pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (cloud);
      visualizer.addPointCloud<PointT>(cloud, color_handler, id);
      setPointCloudRenderProps(visualizer, id, point_size, Color(), opacity);
    }
    /** \brief visualize a pointcloud colored according to the scalar data vector
      * \param[in]  visualizer    visualizer object
      * \param[in]  cloud         pointcloud
      * \param[in]  data          data vector
      * \param[in]  id            the point cloud object id (default: cloud_seg)
      * \param[in]  point_size    size of the point (default 1.0)
      */    
    template <typename PointT, typename Scalar>
    inline
    void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
                                  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data,
                                  const std::string &id = "cloud",
                                  const float point_size = -1.0
                                )
    {
      pcl::visualization::PointCloudColorHandlerCustomData<PointT, float> color_handler (cloud, data);
      visualizer.addPointCloud<PointT> (cloud, color_handler, id);
      setPointCloudRenderProps(visualizer, id, point_size);
    }
    
    /** \brief visualize a pointcloud colored according to the scalar data vector
      * \param[in]  visualizer    visualizer object
      * \param[in]  cloud         pointcloud
      * \param[in]  data          data vector
      * \param[in]  id            the point cloud object id (default: cloud_seg)
      * \param[in]  point_size    size of the point (default 1.0)
      */    
    template <typename PointT, typename Scalar>
    inline
    void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
                                  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  const std::vector<Scalar> &data,
                                  const std::string &id = "cloud",
                                  const float point_size = -1.0
                                )
    {
      pcl::visualization::PointCloudColorHandlerCustomData<PointT, float> color_handler (cloud, data);      
      visualizer.addPointCloud<PointT> (cloud, color_handler, id);
      setPointCloudRenderProps(visualizer, id, point_size);
    }
    
    /** \brief Show pointcloud normals
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  cloud       pointcloud
     *  \param[in]  level       display only every level'th point (default: 100)
     *  \param[in]  scale       the normal arrow scale (default: 0.02m)
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  line_width  width of the normal lines
     *  \param[in]  color       color of the cloud
     *  \param[in]  opacity     opacity of the cloud
     */
    template <typename PointT>
    inline
    void showNormalCloud  ( pcl::visualization::PCLVisualizer &visualizer,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const int level = 100,
                            const float scale = 0.02f,
                            const std::string &id = "normals",
                            const float line_width = -1.0,
                            const Color &color = Color(),
                            const float opacity = -1.0
                          )
    {
      visualizer.addPointCloudNormals<PointT>(cloud, level, scale, id);
      setNormalCloudRenderProps(visualizer, id, line_width, color, opacity);
    }

    /** \brief Show pointcloud normals for a specific subset of points in the
     * cloud.
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  cloud       pointcloud
     *  \param[in]  indices     indices of points fow which normals will be displayed
     *  \param[in]  scale       the normal arrow scale (default: 0.02m)
     *  \param[in]  id          point cloud object id prefix (default: cloud)
     *  \param[in]  point_size  size of the points used for visualization
     *  \param[in]  color       color of the cloud
     *  \param[in]  opacity     opacity of the cloud
     */
    template <typename PointT>
    inline
    void showNormalCloud  ( pcl::visualization::PCLVisualizer &visualizer,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const std::vector<int> indices,
                            const float scale = 0.02f,
                            const std::string &id = "normals",
                            const float point_size = -1.0,
                            const Color &color = Color(),
                            const float opacity = -1.0
                          )
    {
      typename pcl::PointCloud<PointT>::Ptr cloudIndexed (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud<PointT>(*cloud, indices, *cloudIndexed);
      showNormalCloud<PointT>(visualizer, cloudIndexed, 1, scale, id, point_size, color, opacity);
    }
    
    //----------------------------------------------------------------------------
    // Multiple pointcloud visualization
    //----------------------------------------------------------------------------  
    
    /** \brief visualize multiple pointclouds
     *  \param[in] visualizer  visualizer object
     *  \param[in] clouds      a vector of clouds that need to bi visualized
     *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
     *  \param[in] point_size  size of the points used for visualization
     *  \param[in] opacity     opacity of the displayed points
     */
    template <typename PointT>
    inline
    void showPointClouds (  pcl::visualization::PCLVisualizer &visualizer,
                            const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                            const std::string &id_prefix = "segment_",
                            const float point_size = -1.0,
                            const float opacity = -1.0
                          )
    {
      for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
      {
        Color color = getGlasbeyColor (cloudId);
        std::string id = id_prefix + "_" + std::to_string(cloudId);
        showPointCloud<PointT>(visualizer, clouds[cloudId], id, point_size, color, opacity);
      }
    }
    
    /** \brief visualize multiple pointclouds
     *  \param[in] visualizer  visualizer object
     *  \param[in] clouds      a vector of clouds that need to be visualized
     *  \param[in] color       color of the pointclouds
     *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
     *  \param[in] point_size  size of the points used for visualization
     *  \param[in] opacity     opacity of the displayed points
     */
    template <typename PointT>
    inline
    void showPointClouds (  pcl::visualization::PCLVisualizer &visualizer,
                            const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                            const Color &color,
                            const std::string &id_prefix = "segment_",
                            const float point_size = -1.0,
                            const float opacity = -1.0
                          )
    {
      for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
      {
        std::string id = id_prefix + "_" + std::to_string(cloudId);
        showPointCloud<PointT>(visualizer, clouds[cloudId], id, point_size, color, opacity);
      }
    }
    
    /** \brief visualize multiple pointclouds
     *  \param[in] visualizer  visualizer object
     *  \param[in] clouds      a vector of clouds that need to bi visualized
     *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
     *  \param[in] point_size  size of the points used for visualization
     *  \param[in] opacity     opacity of the displayed normals
     */
    template <typename PointT>
    inline
    void showNormalClouds ( pcl::visualization::PCLVisualizer &visualizer,
                            const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                            const int level = 100,
                            const float scale = 0.02f,
                            const std::string &id_prefix = "segment_",
                            const float point_size = -1.0,
                            const Color &color = Color(),
                            const float opacity = -1.0
                          )
    {
      for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
      {
        std::string id = id_prefix + "_" + std::to_string(cloudId);
        showNormalCloud<PointT>(visualizer, clouds[cloudId], level, scale, id, point_size, color, opacity);
      }
    }    
    
    //----------------------------------------------------------------------------
    // Oversegmentation visualization
    //----------------------------------------------------------------------------  

    /** \brief Visualize multiple pointclouds that are a subset of some other cloud
     *  \param[in]  visualizer        visualizer object
     *  \param[in]  cloud             pointcloud
     *  \param[in]  segments          indices of points belonging to each segment
     *  \param[in]  id_prefix         the point cloud object id prefix (default: segment_)
     *  \param[in]  point_size        size of displayed points
     *  \param[in]  opacity           opacity of displayed points
     */
    template <typename PointT>
    inline
    void showSegmentation ( pcl::visualization::PCLVisualizer &visualizer,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const utl::map::Map &segments,
                            const std::string &id_prefix = "segment",
                            const float point_size = -1.0,
                            const float opacity = -1.0
                          )
    {
      // Extract individual clouds
      std::vector<typename pcl::PointCloud<PointT>::Ptr > clouds (segments.size());
      
      for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
      {
        typename pcl::PointCloud<PointT>::Ptr curSegmentCloud (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud, segments[cloudId], *curSegmentCloud);
        clouds[cloudId] = curSegmentCloud;
      }
      
      // Visualize
      showPointClouds<PointT>(visualizer, clouds, id_prefix, point_size, opacity);
    }  

    /** \brief Visualize multiple pointclouds that are a subset of some other colored cloud
     *  \param[in]  visualizer        visualizer object
     *  \param[in]  cloud             pointcloud
     *  \param[in]  segments          indices of points belonging to each segment
     *  \param[in]  id_prefix         the point cloud object id prefix (default: segment_)
     *  \param[in]  tint              tint of the color used to visualize each subcloud
     *  \param[in]  point_size        size of displayed points
     *  \param[in]  opacity           opacity of displayed points
     */
    template <typename PointT>
    inline
    void showSegmentationColored  ( pcl::visualization::PCLVisualizer &visualizer,
                                    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                    const utl::map::Map &segments,
                                    const std::string &id_prefix = "segment",
                                    const float tint = 1.0,
                                    const float point_size = -1.0,
                                    const float opacity = -1.0
                                  )
    {
      // Extract individual clouds
      std::vector<typename pcl::PointCloud<PointT>::Ptr > clouds (segments.size());
      
      for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
      {
        // Copy cloud
        typename pcl::PointCloud<PointT>::Ptr curSegmentCloud (new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud, segments[cloudId], *curSegmentCloud);
        
        // Color it
        utl::pclvis::rgb2gray<PointT>(*curSegmentCloud);
        utl::pclvis::tintPointCloud<PointT>(*curSegmentCloud, utl::pclvis::getGlasbeyColor(cloudId), 0.5);
        
        // Display
        utl::pclvis::showPointCloudColor<PointT>(visualizer, curSegmentCloud, id_prefix + "_" + std::to_string(cloudId), point_size, opacity);
      }      
    }
    
    //----------------------------------------------------------------------------
    // Foreground segmentation visualization
    //----------------------------------------------------------------------------
    
    inline
    void showFGsegmentationMesh (pcl::visualization::PCLVisualizer &visualizer, const std::vector<pcl::PolygonMesh> &mesh, std::vector<int> &segmentation)
    { 
      // Get segmentation mask
      std::vector<bool> segmentationMask (mesh.size(), false);
      for (size_t segId = 0; segId < segmentation.size(); segId++)
        segmentationMask[segmentation[segId]] = true;
      
      Color FGcolor(1.0, 1.0, 1.0);
      Color BGcolor(0.2, 0.2, 0.2);
      
      for (size_t segId = 0; segId < mesh.size(); segId++)
      {
        std::string segIdStr = "segment_" + std::to_string(segId);
        visualizer.addPolygonMesh(mesh[segId], segIdStr);
        if (segmentationMask[segId])
          setPointCloudRenderProps(visualizer, segIdStr, -1.0, FGcolor);
        else
          setPointCloudRenderProps(visualizer, segIdStr, -1.0, BGcolor);
      }                  
    }  

    /** \brief Visualize a foreground background segmentation of a pointcloud.
     *  \param[in] visualizer visualizer object
     *  \param[in] cloud input pointcloud
     *  \param[in] fg_indices indices of the foreground points
     *  \param[in] id_prefix         the point cloud object id prefix (default: segment)* 
     *  \param[in] point_size size of the points used for visualization
     */  
    template <typename PointT>
    inline
    void showFGSegmentation ( pcl::visualization::PCLVisualizer &visualizer,
                              const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                              const std::vector<int> &fg_indices,
                              const std::string &id_prefix = "segment",
                              const int point_size = -1.0f
                            )
    {
      // Show whole cloud
      utl::pclvis::Color curBgColor = utl::pclvis::bgColor;
      curBgColor.r *= 1.1;
      curBgColor.g *= 1.1;
      curBgColor.b *= 1.1;
      utl::pclvis::showPointCloud<PointT>(visualizer, cloud, id_prefix + "_cloud", point_size, curBgColor);
      
      // Show fg cloud
      typename pcl::PointCloud<PointT>::Ptr fgCloud (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, fg_indices, *fgCloud);
      utl::pclvis::showPointCloud<PointT>(visualizer, fgCloud, id_prefix + "_fg", point_size, utl::pclvis::Color(0.5, 0.5, 0.5));
    }    
//     template <typename PointT>
//     inline
//     void showFGsegmentation ( pcl::visualization::PCLVisualizer &visualizer,
//                               const pcl::PointCloud<PointT> &cloud,
//                               const std::vector<int> &fg_indices,
//                               const float point_size = -1.0f
//                             )
//     { 
//       // Get background points
//       std::vector<int> all_indices (cloud.size());
//       for (size_t pointId = 0; pointId < cloud.size(); pointId++)
//         all_indices[pointId] = pointId;    
//       std::vector<int> bg_indices = utl::stdvec::vectorDifference(all_indices, fg_indices);
//       
//       // Display
//       Color FGcolor(1.0, 1.0, 1.0);
//       Color BGcolor(0.2, 0.2, 0.2);
//       
//       showPointCloud<PointT>(visualizer, cloud, fg_indices, "foreground", point_size, FGcolor);
//       showPointCloud<PointT>(visualizer, cloud, bg_indices, "background", point_size, FGcolor);      
//     }
    
    /** \brief visualize a a freground background segmentation of a pointcloud where cloud is already colored
     *  \param[in] visualizer visualizer object
     *  \param[in] cloud input pointcloud
     *  \param[in] fg_indices indices of the foreground points
     *  \param[in] id_prefix         the point cloud object id prefix (default: segment)
     *  \param[in] point_size size of the points used for visualization
     *  \param[in] bg_color background color
     */
    template <typename PointT>
    inline
    void showFGSegmentationColor  ( pcl::visualization::PCLVisualizer &visualizer,
                                    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                    const std::vector<int> &fg_indices,
                                    const std::string &id_prefix = "segment",
                                    const int point_size = -1.0f,
                                    const Color &bg_color = Color (),
                                    const float opacity = -1.0f
                                  )
    {
      Color tint_color;
      if (bg_color == Color())
        tint_color = Color(1.0f, 1.0f, 1.0f);
      else
        tint_color = bg_color;
      
      // Get background indices
      std::vector<int> all_indices (cloud->size());
      for (size_t pointId = 0; pointId < cloud->size(); pointId++)
        all_indices[pointId] = pointId;    
      std::vector<int> bg_indices = utl::stdvec::vectorDifference(all_indices, fg_indices);
            
      // Show bg cloud
      typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud<PointT>(*cloud, bg_indices, *bgCloud);
      utl::pclvis::tintPointCloud<PointT>(*bgCloud, tint_color, 0.7);
      utl::pclvis::showPointCloudColor<PointT>(visualizer, bgCloud, id_prefix + "_cloud", point_size, opacity);
      
      // Show fg cloud
      typename pcl::PointCloud<PointT>::Ptr fgCloud (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, fg_indices, *fgCloud);
      utl::pclvis::showPointCloudColor<PointT>(visualizer, fgCloud, id_prefix + "_fg", point_size);
    }
    
//     template <typename PointT>
//     inline
//     void showFGsegmentationColored  ( pcl::visualization::PCLVisualizer &visualizer,
//                                       const pcl::PointCloud<PointT> &cloud,
//                                       const std::vector<int> &fg_indices,
//                                       const float point_size = -1.0f
//                                      )
//     { 
//       // Get background points
//       std::vector<int> all_indices (cloud.size());
//       for (size_t pointId = 0; pointId < cloud.size(); pointId++)
//         all_indices[pointId] = pointId;    
//       std::vector<int> bg_indices = utl::stdvec::vectorDifference(all_indices, fg_indices);
//       
//       // Get FG and BG clouds
//       typename pcl::PointCloud<PointT>::Ptr displayCloud (new pcl::PointCloud<PointT>);
//       pcl::copyPointCloud(cloud, *displayCloud);
//       
//       // Display
//       Color BGcolor(220, 220, 220);
// 
//       for (auto pointIt = bg_indices.begin(); pointIt != bg_indices.end(); pointIt++)
//       {
//         displayCloud->points[*pointIt].r = BGcolor.r;
//         displayCloud->points[*pointIt].g = BGcolor.g;
//         displayCloud->points[*pointIt].b = BGcolor.b;
//       }
//             
//       pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler(displayCloud);
//       visualizer.addPointCloud<PointT>(displayCloud, color_handler, "fg_segmentation");
//       setPointCloudRenderProps(visualizer, "fg_segmentation", point_size);
//     }  
    
    //----------------------------------------------------------------------------
    // Mesh visualization
    //----------------------------------------------------------------------------    
    
//     /** \brief visualize a set of meshes
//      *  \param[in] visualizer visualizer object
//      *  \param[in] meshes vector of meshes
//      *  \param[in] id_prefix the point cloud object id prefix (default: mesh_)
//      *  \param[in] color color of the meshes [0, 1]
//      */  
//     inline
//     void showMeshes(  pcl::visualization::PCLVisualizer &visualizer,
//                       const std::vector<pcl::PolygonMesh> &meshes,
//                       const std::string id_prefix = "mesh_",
//                       const Colour &color = Colour(1.0, 1.0, 1.0)
//                    )
//     {     
//       for (size_t meshId = 0; meshId < meshes.size(); meshId++)
//       {
//         std::string meshIdStr = "segment_" + std::to_string(meshId);
//         visualizer.addPolygonMesh(meshes[meshId], meshIdStr);
//         setPointCloudRenderProps(visualizer, meshIdStr, color);
//       }                  
//     }
//     
//     /** \brief visualize a set of meshes colored according to the scalar data vector
//      *  \param[in] visualizer visualizer object
//      *  \param[in] meshes vector of meshes
//      *  \param[in] data data vector
//      *  \param[in,out]  colormap  colormap object (if range limits are equal to NaN they are updated to min max of the data vector)
//      *  \param[in] id_prefix the point cloud object id prefix (default: mesh_)
//      */  
//     template <typename Scalar>
//     inline
//     void showMeshesWithData(  pcl::visualization::PCLVisualizer &visualizer,
//                               const std::vector<pcl::PolygonMesh> &meshes,
//                               const std::vector<Scalar> &data,
//                               Colormap &colormap,
//                               const std::string id_prefix = "mesh_"
//                           )
//     {     
//       Colours colors = colormap.getColoursFromData(data);
//       
//       for (size_t meshId = 0; meshId < meshes.size(); meshId++)
//       {
//         std::string meshIdStr = "segment_" + std::to_string(meshId);
//         visualizer.addPolygonMesh(meshes[meshId], meshIdStr);
//         visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[meshId].r, colors[meshId].g, colors[meshId].b, meshIdStr);
//       }
//     }
    
    //----------------------------------------------------------------------------
    // Graph visualization
    //----------------------------------------------------------------------------

    /** \brief visualize a graph defined on points in 3D space
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  points      points
     *  \param[in]  edges       edges in the graph
     *  \param[in]  id          id of the graph object (default: graph)
     *  \param[in]  line_width  width of the graph edge lines used for display (default 1.0)
     *  \param[in]  color       color of the graph edge lines
     *  \param[in]  opacity     opacity of the graph edge lines
     */
    template <typename PointT>
    inline
    void showPointGraph ( pcl::visualization::PCLVisualizer &visualizer,
                          const pcl::PointCloud<PointT> &points,
                          const utl::graph::Edges &edges,
                          const std::string &id = "graph",
                          const float line_width = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                       )
    {
      // Create the polydata where we will store all the geometric data
      vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
      
      // Create the points
      vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
      for (size_t pointId = 0; pointId < points.size(); pointId++)
      {
        double pt[3] = { points.points[pointId].x, points.points[pointId].y, points.points[pointId].z };
        points_vtk->InsertNextPoint(pt);
      }
      linesPolyData->SetPoints(points_vtk);   // Add the points to the polydata container
      
      // Create the lines
      vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
      for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
      {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, edges[edgeId].first);
        line->GetPointIds()->SetId(1, edges[edgeId].second);
        lines->InsertNextCell(line);
      }
      linesPolyData->SetLines(lines);         // Add the lines to the polydata container

      // Add polygon data to the visualizer
      visualizer.addModelFromPolyData (linesPolyData, id);
      utl::pclvis::setLineRenderProps(visualizer, id, line_width, color, opacity);
    }
    
    /** \brief visualize a graph defined on points in 3D space
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  points      points
     *  \param[in]  graph       graph adjacency list
     *  \param[in]  id          id of the graph object (default: graph)
     *  \param[in]  line_width  width of the graph edge lines used for display (default 1.0)
     *  \param[in]  color       color of the graph edge lines
     *  \param[in]  opacity     opacity of the graph edge lines
     */  
    template <typename PointT>
    inline
    void showPointGraph ( pcl::visualization::PCLVisualizer &visualizer,
                          const pcl::PointCloud<PointT> &points,
                          const utl::graph::Graph &graph,
                          const std::string &id = "graph",
                          const float line_width = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                       )
    {
      // Get graph edges and their weights
      utl::graph::Edges edges;
      edges = utl::graph::graph2Edges(graph);
      showPointGraph<PointT>(visualizer, points, edges, id, line_width, color, opacity);
    }
    
    /** \brief Visualize a weighted graph defined on points in 3D space
     *  \param[in]  visualizer    visualizer object
     *  \param[in]  points        points
     *  \param[in]  edges         graph edges
     *  \param[in]  edge_weights  edge weights
     *  \param[in]  id            id of the graph object (default: graph)
     *  \param[in]  line_width    width of the graph edge lines used for display (default 1.0)
     *  \param[in]  color         color of the graph edge lines
     *  \param[in]  opacity       opacity of the graph edge lines
     */  
    template <typename PointT>
    inline
    void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
                                  const pcl::PointCloud<PointT> &points,
                                  const utl::graph::Edges &edges,
                                  const utl::graph::EdgeWeights &edge_weights,
                                  const std::string &id = "graph",
                                  const float line_width = -1.0,
                                  const float opacity = -1.0
                                )
    {
      // Check that number of edges and edge weights is the same
      if (edges.size() != edge_weights.size())
      {
        std::cout << "[utl::pclvis::showPointGraphWeighted] Edges and edge weights are different size." << std::endl;
        return;
      }
      
      // Create the polydata where we will store all the geometric data
      vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
      
      // Create the points
      vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
      for (size_t pointId = 0; pointId < points.size(); pointId++)
      {
        double pt[3] = { points.points[pointId].x, points.points[pointId].y, points.points[pointId].z };
        points_vtk->InsertNextPoint(pt);
      }
      linesPolyData->SetPoints(points_vtk);   // Add the points to the polydata container
      
      // Create the lines
      vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
      for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
      {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, edges[edgeId].first);
        line->GetPointIds()->SetId(1, edges[edgeId].second);
        lines->InsertNextCell(line);
      }
      linesPolyData->SetLines(lines);         // Add the lines to the polydata container
      
      // Assign scalars to lines
      vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New ();
      for (size_t edgeId = 0; edgeId < edge_weights.size(); edgeId++)
        colors->InsertTuple1 (edgeId, edge_weights[edgeId]);
      linesPolyData->GetCellData()->SetScalars(colors);

      std::cout << linesPolyData->GetCellData()->GetScalars() << ", " << linesPolyData->GetPointData()->GetScalars() << std::endl;
      
      // Add polygon data to the visualizer
      visualizer.addModelFromPolyData (linesPolyData, id);
      utl::pclvis::setLineRenderProps(visualizer, id, line_width, Color(), opacity);
    }    
    
    /** \brief Visualize a weighted graph defined on points in 3D space
     *  \param[in]  visualizer    visualizer object
     *  \param[in]  points        points
     *  \param[in]  graph         adjacency between points
     *  \param[in]  graph_weights graph weights
     *  \param[in]  id            id of the graph object (default: graph)
     *  \param[in]  line_width    width of the graph edge lines used for display (default 1.0)
     *  \param[in]  color         color of the graph edge lines
     *  \param[in]  opacity       opacity of the graph edge lines
     */  
    template <typename PointT>
    inline
    void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
                                  const pcl::PointCloud<PointT> &points,
                                  const utl::graph::Graph &graph,
                                  const utl::graph::GraphWeights &graph_weights,
                                  const std::string &id = "graph",
                                  const float line_width = -1.0,
                                  const float opacity = -1.0
                       )
    {
      // Get graph edges and their weights
      utl::graph::Edges edges;
      std::vector<float> edgeWeights;
      utl::graph::graph2EdgesWeighted(graph, graph_weights, edges, edgeWeights);
      
      // Visualize
      showPointGraphWeighted<PointT>(visualizer, points, edges, edgeWeights, id, line_width, opacity);
    }      
    
    /** \brief DEPRECATED Visualize a weighted graph defined on points in 3D space
     *  \param[in] visualizer visualizer object
     *  \param[in] points points
     *  \param[in] edges graph edges
     *  \param[in] edge_weights edge weights
     *  \param[in] colormap colormap used to visualize weights
     *  \param[in] id_prefix prefix to be used for line objects (default: adj_line_)
     *  \param[in] line_width width of the lines used for display (default 1.0)
     */  
//     template <typename PointT>
//     inline
//     void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
//                                   const pcl::PointCloud<PointT> &points,
//                                   const std::vector<std::pair<int, int> > &edges,
//                                   const std::vector<float> &edge_weights,
//                                   Colormap &colormap,
//                                   const std::string &id_prefix = "edge",
//                                   const float line_width = -1.0
//                        )
//     {
//       // Get colors
//       Colors colors = colormap.getColorsFromData<float>(edge_weights);
// 
//       // Display graphs
//       for (size_t edgeId = 0; edgeId < edges.size(); edgeId++)
//       {
//         int sourceVtxId = edges[edgeId].first;
//         int targetVtxId = edges[edgeId].second;
//         std::string id_string = id_prefix + std::to_string(edgeId) + "_"  + std::to_string(edgeId);
//         visualizer.addLine(points.points[sourceVtxId], points.points[targetVtxId], id_string);
//         setLineRenderProps(visualizer, id_string, line_width, colors[edgeId]);
//       }
//     }
    
    /** \brief DEPRECATED Visualize a weighted graph defined on points in 3D space
     *  \param[in] visualizer visualizer object
     *  \param[in] points points
     *  \param[in] graph adjacency between points
     *  \param[in] graph_weights graph weights
     *  \param[in] id_prefix prefix to be used for line objects (default: adj_line_)
     *  \param[in] line_width width of the lines used for display (default 1.0)
     */  
//     template <typename PointT>
//     inline
//     void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
//                                   const pcl::PointCloud<PointT> &points,
//                                   const utl::graph::Graph &graph,
//                                   const utl::graph::GraphWeights &graph_weights,
//                                   Colormap &colormap,
//                                   const std::string &id_prefix = "edge",
//                                   const float line_width = -1.0
//                        )
//     {
//       // Get graph edges and their weights
//       std::vector<std::pair<int, int> > edges;
//       std::vector<float> edgeWeights;
//       utl::graph::graphWeighted2EdgePairs(graph, graph_weights, edges, edgeWeights);
//       
//       // Visualize
//       showPointGraphWeighted<PointT>(visualizer, points, edges, edgeWeights, colormap, id_prefix, line_width);
//     }  
    
    /** \brief visualize a 3d curve represented as an ordered set of points
     *  \param[in]  visualizer  visualizer object
     *  \param[in]  curve       curve
     *  \param[in]  id_prefix   prefix to be used for line objects (default: curve_)
     *  \param[in]  line_width  width of the lines used for display (default 1.0)
     *  \param[in]  color      color of the lines (default gray)
     */  
    template <typename PointT>
    inline
    void showCurve (pcl::visualization::PCLVisualizer &visualizer,
                    const pcl::PointCloud<PointT> &curve,
                    const std::string &id_prefix = "curve",
                    const float line_width = -1.0,
                    const Color &color = Color(0.4, 0.4, 0.4),
		    const float opacity = -1.0f
                  )
    {
      if (curve.size() < 2)
      {
        std::cout << "[utl::pclvis::showCurve] curve must contain at least two points!" << std::endl;
        return;
      }
      
      for (size_t linkId = 0; linkId < curve.size()-1; linkId++)
      {
        std::string id = id_prefix + "_" + std::to_string(linkId);
        visualizer.addLine(curve[linkId], curve[linkId+1], id);
        setLineRenderProps(visualizer, id, line_width, color, opacity);
      }
    }

    //----------------------------------------------------------------------------
    // Geometric primitives
    //----------------------------------------------------------------------------  

    /** \brief visualize a plane by plotting a square polygon 
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  pose            plane pose (XY plane X is normal)
     *  \param[in]  id              plane object id (default: plane)
     *  \param[in]  side_width      width of the square side
     *  \param[in]  color           color of the displayed plane
     *  \param[in]  opacity         opacity of the displayed plane
     */    
    inline
    void showPlane  ( pcl::visualization::PCLVisualizer &visualizer,
                      const Eigen::Affine3f &pose,
                      const std::string &id = "plane",
                      const float side_width = 0.05,
                      const Color color = Color(),
                      const float opacity = -1.0f
                    )
    {
      // First generate a square
      float d = side_width / 2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_polygon (new pcl::PointCloud<pcl::PointXYZ>);
      plane_polygon->resize(4);
      plane_polygon->points[0] = pcl::PointXYZ(-d, -d,  0);
      plane_polygon->points[1] = pcl::PointXYZ( d, -d,  0);
      plane_polygon->points[2] = pcl::PointXYZ( d,  d,  0);
      plane_polygon->points[3] = pcl::PointXYZ(-d,  d,  0);

      // Transform polygon according to the pose
      pcl::transformPointCloud<pcl::PointXYZ>(*plane_polygon, *plane_polygon, pose);
      
      // Display
      visualizer.addPolygon<pcl::PointXYZ>(plane_polygon, id);
      setShapeRenderProps(visualizer, id, color, opacity);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);     
    }
    
    /** \brief visualize a plane by plotting a square polygon 
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  plane_point     plane point
     *  \param[in]  plane_normal    plane normal
     *  \param[in]  id              plane object id (default: plane)
     *  \param[in]  side_width      width of the square side
     */    
    inline
    void showPlane  ( pcl::visualization::PCLVisualizer &visualizer,
                      const Eigen::Vector3f &plane_point,
                      const Eigen::Vector3f &plane_normal,
                      const std::string &id = "plane",
                      const float side_width = 0.05,
                      const Color color = Color(),
                      const float opacity = -1.0f
                    )
    {
      // First generate two vectors orthogonal to the normal
      Eigen::Vector3f normalOrth1 = Eigen::Vector3f(-plane_normal[1]/plane_normal[0], 1, 0);
      normalOrth1.normalize();
      Eigen::Vector3f normalOrth2 = normalOrth1.cross(plane_normal);

      // Then generate symmetry plane polygon
      pcl::PointCloud<pcl::PointXYZ>::Ptr plane_polygon (new pcl::PointCloud<pcl::PointXYZ>);
      plane_polygon->resize(4);
      plane_polygon->at(0).getVector3fMap() = plane_point + (  normalOrth1 + normalOrth2) * side_width/2;
      plane_polygon->at(1).getVector3fMap() = plane_point + (  normalOrth1 - normalOrth2) * side_width/2;
      plane_polygon->at(2).getVector3fMap() = plane_point + (- normalOrth1 - normalOrth2) * side_width/2;
      plane_polygon->at(3).getVector3fMap() = plane_point + (- normalOrth1 + normalOrth2) * side_width/2;
     
      visualizer.addPolygon<pcl::PointXYZ>(plane_polygon, 0.0, 1.0, 0.0, id);
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);

      if (color != Color())
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
      
      if (opacity != -1.0f)
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
      
//       // Show plane normal
//       pcl::PointXYZ p1, p2;
//       p1.getVector3fMap() = plane_point;
//       p2.getVector3fMap() = plane_point + plane_normal * 0.2;
//       visualizer.addLine(p1, p2);
    }

    /** \brief visualize a plane by plotting a square polygon 
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  plane_coefficients     coefficients of the equation of a plane (ax + by + cz + d = 0)
     *  \param[in]  plane_normal    plane normal
     *  \param[in]  id              plane object id (default: plane)
     *  \param[in]  side_width      width of the square side
     */    
    inline
    void showPlane  ( pcl::visualization::PCLVisualizer &visualizer,
                      const Eigen::Vector4f &plane_coefficients,
                      const std::string &id = "plane",
                      const float side_width = 0.05,
                      const Color color = Color(),
                      const float opacity = -1.0f
                    )
    {
      // Generate plane point and normal
      Eigen::Vector3f planePoint, planeNormal;
      geom::planeCoefficientsToPointNormal<float>(plane_coefficients, planePoint, planeNormal);
      
      // Show plane
      showPlane(visualizer, planePoint, planeNormal, id, side_width, color, opacity);
    }
    
    //----------------------------------------------------------------------------
    // Miscelanious
    //----------------------------------------------------------------------------    
    
    /** \brief Show correspondences between points by rendering a line between
     * each pair of corresponding points.
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  source_points The source points
     *  \param[in]  target_points The target points
     *  \param[in]  correspondences The mapping from source points to target points. Each element must be an index into target_points
     *  \param[in]  nth display only the Nth correspondence (e.g., skip the rest)
     *  \param[in]  id the polygon object id (default: "correspondences")
     *  \param[in]  line_width   width of the lines
     *  \param[in]  opacity opacity of the lines
     */
    template <typename PointT> void
    showCorrespondences ( pcl::visualization::PCLVisualizer &visualizer,
                          const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
                          const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
                          const pcl::Correspondences &correspondences,
                          const int nth = 1,
                          const std::string &id_prefix = "crsp_",
                          const float line_width = -1.0f,
                          const float opacity = -1.0f
                        )
    {
      // Show correspondences
      for (size_t crspId = 0; crspId < correspondences.size(); crspId+=nth)
      {
        std::string lineId = id_prefix + std::to_string(crspId);
        int queryId = correspondences[crspId].index_query;
        int matchId = correspondences[crspId].index_match;
        visualizer.addLine(source_points->at(queryId), target_points->at(matchId), lineId);
        utl::pclvis::Color clr = utl::pclvis::getGlasbeyColor(crspId);
        utl::pclvis::setLineRenderProps(visualizer, lineId, line_width, clr, opacity);
      }
    }
    
    /** \brief DEPRECATED Update the colorbar actor of PCLInteractorStyle
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  colormap        colormap object
     */    
//     inline
//     bool updateColorbar ( pcl::visualization::PCLVisualizer &visualizer,
//                           const Colormap &colormap
//                       )
//     {
//       // Create LUT
//       vtkSmartPointer<vtkLookupTable> colormapLUT = colormap.getColorLookupTable();
//       
//       // Update colorbar
//       vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle> interactorStyle = visualizer.getInteractorStyle();
//       vtkRenderer *curRenderer = interactorStyle->GetCurrentRenderer();
//       if (curRenderer != nullptr)
//       {
//         // Get 2D actors
//         vtkActor2DCollection *curActors = curRenderer->GetActors2D();
//         
//         // Loop over actors and find the colorbar actor
//         curActors->InitTraversal();
//         for (size_t i = 0; i < static_cast<size_t>(curActors->GetNumberOfItems()); i++)
//         {
//           vtkActor2D *curActor = curActors->GetNextActor2D();
//           if (!curActor->IsA("vtkScalarBarActor"))
//             continue;
//           
//           // Update the LUT table
//           vtkScalarBarActor *colorbarActor = vtkScalarBarActor::SafeDownCast(curActor);
//           colorbarActor->SetLookupTable(colormapLUT);
//           
//           return true;
//         }
//           
//         std::cout << "[pcl::updateColorbar] Could not find 'vtkLookupTable' actor in current renderer. This probably means that colorbar was not being displayed when this function was called" << std::endl;
//       }
//       else
//       {
//         std::cout << "[pcl::updateColorbar] CurrentRenderer does not exist yet" << std::endl;      
//       }
//       
//       return false;
//     }
    
    /** \brief show a visualization of camera view frustum
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  K               camera matrix
     *  \param[in]  height          camera height
     *  \param[in]  width           camera width
     *  \param[in]  pose            camera pose
     *  \param[in]  id              name id of the camera
     */
    inline
    void showCamera ( pcl::visualization::PCLVisualizer &visualizer,
                      const Eigen::Matrix3f &K,
                      const int height, const int width,
                      const Eigen::Affine3f &pose,
                      const std::string &id = "cam",
                      const Color &color = Color ()
                    )
    {
      float focal = (K(0,0) + K(1,1)) / 2;
      float height_f = static_cast<float>(height);
      float width_f = static_cast<float>(width);
      
      // create a 5-point visual for each camera
      pcl::PointXYZ p1, p2, p3, p4, p5;
      p1.x=0; p1.y=0; p1.z=0;
      float dist = 0.1;
      float minX, minY, maxX, maxY;
      maxX = dist*tan (std::atan (width_f / (2.0*focal)));
      minX = -maxX;
      maxY = dist*tan (std::atan (height_f / (2.0*focal)));
      minY = -maxY;
      p2.x=minX; p2.y=minY; p2.z=dist;
      p3.x=maxX; p3.y=minY; p3.z=dist;
      p4.x=maxX; p4.y=maxY; p4.z=dist;
      p5.x=minX; p5.y=maxY; p5.z=dist;
          
      p1=pcl::transformPoint (p1, pose);
      p2=pcl::transformPoint (p2, pose);
      p3=pcl::transformPoint (p3, pose);
      p4=pcl::transformPoint (p4, pose);
      p5=pcl::transformPoint (p5, pose);
      if (color == Color())
        visualizer.addText3D(id, p1, 0.01, 1.0, 1.0, 1.0, id);
      else
        visualizer.addText3D(id, p1, 0.01, color.r, color.g, color.b, id);

      visualizer.addLine (p1, p2, id + "_line1");
      visualizer.addLine (p1, p3, id + "_line2");
      visualizer.addLine (p1, p4, id + "_line3");
      visualizer.addLine (p1, p5, id + "_line4");
      visualizer.addLine (p2, p5, id + "_line5");
      visualizer.addLine (p5, p4, id + "_line6");
      visualizer.addLine (p4, p3, id + "_line7");
      visualizer.addLine (p3, p2, id + "_line8");
      
      setLineRenderProps(visualizer, id + "_line1", -1.0, color);
      setLineRenderProps(visualizer, id + "_line2", -1.0, color);
      setLineRenderProps(visualizer, id + "_line3", -1.0, color);
      setLineRenderProps(visualizer, id + "_line4", -1.0, color);
      setLineRenderProps(visualizer, id + "_line5", -1.0, color);
      setLineRenderProps(visualizer, id + "_line6", -1.0, color);
      setLineRenderProps(visualizer, id + "_line7", -1.0, color);
      setLineRenderProps(visualizer, id + "_line8", -1.0, color);
    }
    
    /** \brief show a visualization of camera view frustum
     *  \param[in]  visualizer      visualizer object
     *  \param[in]  K               camera matrix
     *  \param[in]  height          camera height
     *  \param[in]  width           camera width
     *  \param[in]  pose            camera pose
     *  \param[in]  id              name id of the camera
     */
    inline
    void removeCamera ( pcl::visualization::PCLVisualizer &visualizer, const std::string &id = "cam")
    {
      visualizer.removeText3D(id);
      visualizer.removeShape(id + "_line1");
      visualizer.removeShape(id + "_line2");
      visualizer.removeShape(id + "_line3");
      visualizer.removeShape(id + "_line4");
      visualizer.removeShape(id + "_line5");
      visualizer.removeShape(id + "_line6");
      visualizer.removeShape(id + "_line7");
      visualizer.removeShape(id + "_line8");
    }
  }
}
#endif // PCL_VISUALIZATION_UTILITIES_HPP