#ifndef OCTOMAP_PCL_CONVERSION_UTILITIES_HPP
#define OCTOMAP_PCL_CONVERSION_UTILITIES_HPP

// Octomap
#include <octomap/octomap.h>


namespace utl
{
  namespace oct
  {
    /** \brief Extract the pointcloud contatining centers of occupied voxels in 
     * an occupancy map.
     * \param[in] occupancy_map occupancy map
     * \param[out] cloud  pointcloud
     * \param[in] bbox_min minimum coordinate of a bounding box in the scene
     * \param[in] bbox_max maximum coordinate of a bounding box in the scene
     * \param[in] depth octree depth at which occupancy is evaluated
     */ 
    template<typename PointT>
    void getOccupiedCloud ( const octomap::OcTree &occupancy_map,
                            pcl::PointCloud<PointT> &cloud,
                            const Eigen::Vector3f  &bbox_min = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                            const Eigen::Vector3f  &bbox_max = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                            const unsigned int depth = 0
                          )
    {
      // Prepare output cloud
      cloud.resize(0);
      
      // Get bounding box
      octomap::point3d bbox_min_oct;
      if (bbox_min == bbox_min)
      {
        bbox_min_oct.x() = bbox_min[0]; bbox_min_oct.y() = bbox_min[1]; bbox_min_oct.z() = bbox_min[2];
      }
      else
      {
        double v[3];
        occupancy_map.getMetricMin(v[0], v[1], v[2]);
        bbox_min_oct = octomap::point3d(v[0], v[1], v[2]);
      }
      
      octomap::point3d bbox_max_oct;
      if (bbox_max == bbox_max)
      {
        bbox_max_oct.x() = bbox_max[0]; bbox_max_oct.y() = bbox_max[1]; bbox_max_oct.z() = bbox_max[2];
      }
      else
      {
        double v[3];
        occupancy_map.getMetricMax(v[0], v[1], v[2]);
        bbox_max_oct = octomap::point3d(v[0], v[1], v[2]);        
      }

      // Extract occupied voxel centers and convert to PCL pointcloud
      for(auto leafIt = occupancy_map.begin_leafs_bbx(bbox_min_oct, bbox_max_oct, depth), leafEnd=occupancy_map.end_leafs_bbx(); leafIt!= leafEnd; leafIt++)
      {    
        // If it is not occupied - skip it
        if (occupancy_map.isNodeOccupied(*leafIt))
        {
          PointT curPoint;
          curPoint.x = leafIt.getX();
          curPoint.y = leafIt.getY();
          curPoint.z = leafIt.getZ();      
          cloud.push_back(curPoint);
        }
      }
    }
    
    /** \brief Extract the pointcloud contatining centers of occluded voxels in
     * an occupancy map.
     * \param[in] occupancy_map occupancy map
     * \param[out] cloud  pointcloud
     * \param[in] bbox_min minimum coordinate of a bounding box in the scene
     * \param[in] bbox_max maximum coordinate of a bounding box in the scene
     * \param[in] depth octree depth at which occupancy is evaluated
     */ 
    template<typename PointT>
    void getOccludedCloud ( const octomap::OcTree &occupancy_map,
                            pcl::PointCloud<PointT> &cloud,
                            const Eigen::Vector3f  &bbox_min = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                            const Eigen::Vector3f  &bbox_max = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                            const unsigned int depth = 0
                          )
    {
      // Prepare output cloud
      cloud.resize(0);
      
      // Get bounding box
      octomap::point3d bbox_min_oct;
      if (bbox_min == bbox_min)
      {
        bbox_min_oct.x() = bbox_min[0]; bbox_min_oct.y() = bbox_min[1]; bbox_min_oct.z() = bbox_min[2];
      }
      else
      {
        double v[3];
        occupancy_map.getMetricMin(v[0], v[1], v[2]);
        bbox_min_oct = octomap::point3d(v[0], v[1], v[2]);
      }
      
      octomap::point3d bbox_max_oct;
      if (bbox_max == bbox_max)
      {
        bbox_max_oct.x() = bbox_max[0]; bbox_max_oct.y() = bbox_max[1]; bbox_max_oct.z() = bbox_max[2];
      }
      else
      {
        double v[3];
        occupancy_map.getMetricMax(v[0], v[1], v[2]);
        bbox_max_oct = octomap::point3d(v[0], v[1], v[2]);        
      }

      // Find unknown leafs in the occupancy map
      octomap::point3d_list occludedLeafs;
      occupancy_map.getUnknownLeafCenters(occludedLeafs, bbox_min_oct, bbox_max_oct, depth);
        
      // Convert to PCL pointcloud
      cloud.points.resize(occludedLeafs.size());
      int pointId = 0;
      for (auto leafIt = occludedLeafs.begin(); leafIt != occludedLeafs.end(); leafIt++, pointId++)
      {
        cloud.points[pointId].x = leafIt->x();
        cloud.points[pointId].y = leafIt->y();
        cloud.points[pointId].z = leafIt->z();
      }
    }
    
    /** \brief Extract the pointcloud contatining centers of voxels that are 
     * marked as either occluded or occupied in the occupancy map.
     * \param[in] occupancy_map occupancy map
     * \param[out] cloud  pointcloud
     * \param[in] bbox_min minimum coordinate of a bounding box in the scene
     * \param[in] bbox_max maximum coordinate of a bounding box in the scene
     * \param[in] depth octree depth at which occupancy is evaluated
     */ 
    template<typename PointT>
    void getOccludedOccupiedCloud ( const octomap::OcTree &occupancy_map,
                                    pcl::PointCloud<PointT> &cloud,
                                    const Eigen::Vector3f  &bbox_min = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                                    const Eigen::Vector3f  &bbox_max = Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN(),
                                    const unsigned int depth = 0
                                  )
    {
      // Extract occluded and occupied clouds
      pcl::PointCloud<PointT> occludedCloud;
      pcl::PointCloud<PointT> occupiedCloud;
      
      getOccludedCloud<PointT>(occupancy_map, occludedCloud, bbox_min, bbox_max, depth);
      getOccupiedCloud<PointT>(occupancy_map, occupiedCloud, bbox_min, bbox_max, depth);
      
      // Combine them into one cloud
      cloud.resize(0);
      cloud += occludedCloud;
      cloud += occupiedCloud;
    }

    /** \brief Convert Octomap cloud to a PCL cloud.
     * \param[in] octo_cloud Octomap cloud
     * \param[out] pcl_cloud  PCL cloud
     */ 
    template <typename PointT>
    void octoCloud2pclCloud (const octomap::Pointcloud &octo_cloud, pcl::PointCloud<PointT> &pcl_cloud)
    {
      pcl_cloud.resize(octo_cloud.size());
      pcl_cloud.is_dense = false;
      
      for (size_t i = 0; i < octo_cloud.size(); i++)
      {
        pcl_cloud.points[i].x = octo_cloud[i].x();
        pcl_cloud.points[i].y = octo_cloud[i].y();
        pcl_cloud.points[i].z = octo_cloud[i].z();
      }
    }
  }
}
  
#endif // OCTOMAP_PCL_CONVERSION_UTILITIES_HPP