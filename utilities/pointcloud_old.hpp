#ifndef POINTCLOUD_UTILITIES_HPP
#define POINTCLOUD_UTILITIES_HPP

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>

// CPP tools
#include <utilities/map.hpp>
#include <utilities/graph.hpp>
#include <utilities/geometry.hpp>

namespace utl
{
  namespace cloud
  {
    /** \brief Downsample a cloud using VoxelGrid filter
     *  \param[in]  cloud              input cloud
     *  \param[in]  voxel_size         size of the voxel
     *  \param[out] cloud_downsampled  downsampled cloud
     *  \param[out] downsample_map     map from downsampled points to original points
     *  \return voxelgrid structure used to downsample the cloud
     */
    template <typename PointT>
    pcl::VoxelGrid<PointT> downsampleCloud  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                              const float voxel_size,
                                              typename pcl::PointCloud<PointT>::Ptr &cloud_downsampled,
                                              utl::map::Map &downsample_map
                                            )
    {
      // Downsample the cloud
      pcl::VoxelGrid<PointT> vg;
      vg.setSaveLeafLayout(true);
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_size, voxel_size, voxel_size);
      vg.filter(*cloud_downsampled);
      
      // Generate a map from downsampled centroids to original points
      downsample_map.resize(cloud_downsampled->size());
      for (size_t pointIdHR = 0; pointIdHR < cloud->size(); pointIdHR++)
        downsample_map[vg.getCentroidIndex(cloud->at(pointIdHR))].push_back(pointIdHR);
      
      return vg;
    }

    /** \brief Downsample a cloud using VoxelGrid filter
     *  \param[in]  cloud              input cloud
     *  \param[in]  voxel_size         size of the voxel
     *  \param[out] cloud_downsampled  downsampled cloud
     *  \return voxelgrid structure used to downsample the cloud
     */
    template <typename PointT>
    pcl::VoxelGrid<PointT> downsampleCloud  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                              float voxel_size,
                                              typename pcl::PointCloud<PointT>::Ptr &cloud_downsampled
                                          )
    {
      std::vector<std::vector<int> > dummy;
      return downsampleCloud<PointT>(cloud, voxel_size, cloud_downsampled, dummy);
    }    
    
    /** \brief Methods for downsampling a normal */
    enum NormalDownsampleMethod
    { 
      AVERAGE,            /**< downsampled normal is an average of normals of the points belonging to the voxel renormalized to unit length */
      NEAREST_NEIGHBOR    /**< downsampled normal is chosen to be equal to the normal of the nearest neighbour to the voxel centroid */
    };
    
    /** \brief Downsample a cloud with normals using VoxelGrid filter.
     *  \param[in]  cloud              input cloud
     *  \param[in]  voxel_size         size of the voxel
     *  \param[out] cloud_downsampled  downsampled cloud
     *  \param[out] downsample_map     map from downsampled points to original points
     *  \param[in]  normal_downsample_method  method used for donwsampling normals (default: AVERAGE)
     *  \return voxelgrid structure used to downsample the cloud
     */
    template <typename PointT>
    pcl::VoxelGrid<PointT> downsampleCloudWithNormals ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                        const float voxel_size,
                                                        typename pcl::PointCloud<PointT>::Ptr &cloud_downsampled,
                                                        utl::map::Map &downsample_map,
                                                        const NormalDownsampleMethod normal_downsample_method = AVERAGE
                                                      )
    {
      // Downsample cloud
      pcl::VoxelGrid<PointT> vg = downsampleCloud<PointT>(cloud, voxel_size, cloud_downsampled, downsample_map);
      
      // Downsample normals
      if (normal_downsample_method == AVERAGE)
      {
        for (size_t pointId = 0; pointId < cloud_downsampled->size(); pointId++)
          cloud_downsampled->points[pointId].getNormalVector3fMap() /= cloud_downsampled->points[pointId].getNormalVector3fMap().norm();
      }
      else if (normal_downsample_method == NEAREST_NEIGHBOR)
      {
        // NOTE: brute search is faster since we are only need to search over a few points
        pcl::search::BruteForce<PointT> search;
        std::vector<int>            neighbors(1);
        std::vector<float>          distances(1);
        
        for (size_t pointId = 0; pointId < cloud_downsampled->size(); pointId++)
        {
          search.setInputCloud(cloud, boost::make_shared<std::vector<int> >(downsample_map[pointId]));
          search.nearestKSearch(cloud_downsampled->points[pointId], 1, neighbors, distances);
          cloud_downsampled->points[pointId].getNormalVector3fMap() = cloud->points[neighbors[0]].getNormalVector3fMap();
          cloud_downsampled->points[pointId].curvature = cloud->points[neighbors[0]].curvature;
        }
      }
      else
      {
        std::cout << "[utl::cloud::downsampleCloudWithNormals] unknown normal downsapling method" << std::endl;
        abort();
      }
      
      return vg;
    }
    
    /** \brief Downsample a cloud with normals using VoxelGrid filter.
     *  \param[in]  cloud              input cloud
     *  \param[in]  voxel_size         size of the voxel
     *  \param[out] cloud_downsampled  downsampled cloud
     *  \param[in]  normal_downsample_method  method used for donwsampling normals (default: AVERAGE)
     *  \return voxelgrid structure used to downsample the cloud
     */
    template <typename PointT>
    pcl::VoxelGrid<PointT> downsampleCloudWithNormals ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                        const float voxel_size,
                                                        typename pcl::PointCloud<PointT>::Ptr &cloud_downsampled,
                                                        const NormalDownsampleMethod normal_downsample_method = AVERAGE
                                                      )
    {
      std::vector<std::vector<int> > dummy;
      return downsampleCloudWithNormals<PointT>(cloud, voxel_size, cloud_downsampled, dummy, normal_downsample_method);
    }

    /** Generate graph structure representing local connectivity between points in
     * a pointcloud. Each point is connected to its k nearest neighbors.
     *  \param[in]  cloud             input cloud
     *  \param[in]  indices           indices of the points to be analyzed
     *  \param[in]  num_neighbours    maximum number of neighbours
     *  \param[out] g                 graph
     *  \return false if no edges were found, true otherwise
     */
    template <typename PointT>
    inline
    bool getCloudConnectivityNearestK ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                        const std::vector<int>                            &indices,
                                        graph::Graph                                      &g,
                                        const int                                         num_neighbours

                                   )
    {
      // Prepare graph structure
      g.clear();
      g.resize(cloud->size());
      
      // Prepare search tree
      pcl::search::KdTree<PointT> searchTree;
      searchTree.setInputCloud(cloud, boost::make_shared<std::vector<int> > (indices));

      // Loop over all points
      for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
      { 
        int pointId = indices[pointIdIt];
        
        // Find nearest neighbours
        std::vector<float>  distances(num_neighbours);
        std::vector<int>    neighbors(num_neighbours);
        searchTree.nearestKSearch(pointIdIt, num_neighbours, neighbors, distances);
            
        // Add corresponding edges to the graph
        for (size_t nbrId = 1; nbrId < neighbors.size(); nbrId++)
          graph::addEdge(pointId, neighbors[nbrId], g);
      }
          
      // If there are no edges - return false
      if (graph::getNumEdges(g) < 1)
      {
        std::cout << "[utl::cloud::getCloudConnectivityNearestK] no neighbouring points were found\n";
        return false;
      }
        
      // Otherwise return true
      return true;
    }
    
    
    /** Generate graph structure representing local connectivity between points in
      * a pointcloud. Each point is connected to its k nearest neighbors.
      *  \param[in]  cloud             input cloud
      *  \param[in]  num_neighbours    maximum number of neighbours
      *  \param[out] g                 graph
      *  \return false if no edges were found, true otherwise
      *  \note Note that a point may end up being connected to more than
      * num_neighbors points. Consider points A and B. B is within radius of A
      * but is not one of the num_neighbors closest points of A. On the other 
      * hand A is within num_neighbors closest points of B. This means that 
      * point A will be connected to num_neighbors of it's own neighbors and 
      * also to B.
      */
    template <typename PointT>
    inline
    bool getCloudConnectivityNearestK ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                        graph::Graph                                      &g,
                                        const int                                         num_neighbours

                                   )
    {
      // Create fake indices
      std::vector<int> fake_indices;
      fake_indices.resize(cloud->size());
      for (size_t pointId = 0; pointId < cloud->size(); pointId++)
        fake_indices[pointId] = pointId;
        
      // Build connectivity graph
      return getCloudConnectivityNearestK<PointT>(cloud, fake_indices, g, num_neighbours);
    }
    
    /** Generate graph structure representing local connectivity between points in
      * a pointcloud. Each point is connected to it's k nearest neighbors within a
      * radius r.
      * \param[in]  cloud             input cloud
      * \param[in]  indices           indices of the points to be analyzed
      * \param[in]  radius            radius within which neighbours are searched
      * \param[out] g                 graph
      * \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
      * \return false if no edges were found, true otherwise
      * \note Note that a point may end up being connected to more than
      * num_neighbors points. Consider points A and B. B is within radius of A
      * but is not one of the num_neighbors closest points of A. On the other 
      * hand A is within num_neighbors closest points of B. This means that 
      * point A will be connected to num_neighbors of it's own neighbors and 
      * also to B.
      */
    template <typename PointT>
    inline
    bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                      const std::vector<int>                            &indices,
                                      graph::Graph                                      &g,
                                      const double                                      radius,                                      
                                      const int                                         num_neighbours = 0
                                   )
    {
      // Prepare graph structure
      g.clear();
      g.resize(cloud->size());
      
      // Prepare search tree
      pcl::search::KdTree<PointT> searchTree;
      searchTree.setInputCloud(cloud, boost::make_shared<std::vector<int> > (indices));

      // Loop over all points
      for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
      { 
        int pointId = indices[pointIdIt];

        // Find nearest neighbours
        std::vector<float>  distances;
        std::vector<int>    neighbors;
        searchTree.radiusSearch(pointIdIt, radius, neighbors, distances, num_neighbours);
            
        // Add corresponding edges to the graph
        for (size_t nbrId = 1; nbrId < neighbors.size(); nbrId++)
          graph::addEdge(pointId, neighbors[nbrId], g);        
      }
          
      // If there are no edges - return false
      if (graph::getNumEdges(g) < 1)
      {
        std::cout << "[utl::cloud::getCloudConnectivityRadius] no neighbouring points were found." << std::endl;
        return false;
      }
        
      // Otherwise return true
      return true;
    }
    
    /** Generate graph structure representing local connectivity between points in
      * the pointcloud. Each point is connected to it's k nearest neighbors within a
      * radius r.
      * \param[in]  cloud             input cloud
     *  \param[in]  indices           indices of the points to be analyzed
      * \param[in]  radius            radius within which neighbours are searched
      * \param[out] g                 graph
      * \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
      * \return false if no edges were found, true otherwise
      */
    template <typename PointT>
    inline
    bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                      graph::Graph                                      &g,
                                      const double                                      radius,                                      
                                      const int                                         num_neighbours = 0
                                   )
    {
      // Create fake indices
      std::vector<int> fake_indices;
      fake_indices.resize(cloud->size());
      for (size_t pointId = 0; pointId < cloud->size(); pointId++)
        fake_indices[pointId] = pointId;
        
      // Build connectivity graph
      return getCloudConnectivityRadius<PointT>(cloud, fake_indices, g, radius, num_neighbours);
    }
    
    /** \brief Scale pointcloud clouds relative to it's mean point
     *  \param[in] cloud_in input pointcloud
     *  \param[in] scale_factor scale factor
     *  \param[in] cloud_out scaled pointcloud
     */
    template <typename T>
    inline
    void scalePointCloud( const typename pcl::PointCloud<T> &cloud_in,
                          float scale_factor,
                          typename pcl::PointCloud<T> &cloud_out)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid<T>(cloud_in, centroid);
      
      cloud_out.resize(cloud_in.size());
      
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        cloud_out.points[i].x = (cloud_in.points[i].x - centroid[0]) * scale_factor + centroid[0];
        cloud_out.points[i].y = (cloud_in.points[i].y - centroid[1]) * scale_factor + centroid[1];
        cloud_out.points[i].z = (cloud_in.points[i].z - centroid[2]) * scale_factor + centroid[2];
      }
    }
    
    /** \brief Fit a plane to a pointcloud.
      * \param[in] cloud input cloud
      * \param[out] plane_coefficients coefficients of a plane (ax + by + cz + d = 0)
      */
    template <typename PointT>
    inline
    void fitPlane ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud, Eigen::Vector4f &plane_coefficients)
    {
      //----------------------------------------------------------------------------
      // Check that we have a sufficient number of points
      if (cloud->size() < 3)
      {
        std::cout << "[utl::cloud::fitPlane3D] input cloud contains fewer that 3 points. Can not fit a plane." << std::endl;
        abort();
      }

      //----------------------------------------------------------------------------
      // Fit plane using PCA
      pcl::PCA<PointT> pcaSolver;
      pcaSolver.setInputCloud(cloud);

      // Extract plane point and normal
      Eigen::Vector3f point   = pcaSolver.getMean().head(3);
      Eigen::Vector3f normal  = pcaSolver.getEigenVectors().col(2);

      // Convert to plane coefficients
      utl::geom::pointNormalToPlaneCoefficients<float>(point, normal, plane_coefficients);
    }
    
    /** \brief Given two pointclouds find the closest point between them
     *  \param[in]  cloud1             first cloud
     *  \param[in]  cloud2             second cloud
     *  \param[out] search_tree1       KD tree for first cloud
     *  \param[out] search_tree2       KD tree for second cloud
     *  \param[out] closest_point_1    closest point in the first cloud
     *  \param[out] closest_point_2    closest point in the second cloud
     *  \return distance between two clouds
     */
    template <typename PointT>
    inline
    float cloudToCloudDistance  ( const pcl::PointCloud<PointT> &cloud1,
                                  const pcl::PointCloud<PointT> &cloud2,
                                  const pcl::search::KdTree<PointT> search_tree1,
                                  const pcl::search::KdTree<PointT> search_tree2,
                                  int closest_point_1,
                                  int closest_point_2
                                )
    {
      // Prepare variables
      closest_point_1 = -1;
      closest_point_2 = -1;
      float minDistance = std::numeric_limits<float>::max();
      
      // Prepare search
      std::vector<int> neighbours(1);
      std::vector<float> distances(1);
      
      if (cloud1.size() > cloud2.size())
      {        
        // Iterate over the points of the first cloud and find the closest distance to the second cloud        
        for (size_t pointId = 0; pointId < cloud2.size(); pointId++)
        {
          search_tree1.nearestKSearch(cloud2.points[pointId], 1, neighbours, distances);
          if (distances[0] < minDistance)
          {
            closest_point_1 = neighbours[0];
            closest_point_2 = pointId;
            minDistance = distances[0];
          }
        }        
      }
      else
      {
        // Iterate over the points of the first cloud and find the closest distance to the second cloud        
        for (size_t pointId = 0; pointId < cloud1.size(); pointId++)
        {
          search_tree2.nearestKSearch(cloud1.points[pointId], 1, neighbours, distances);
          if (distances[0] < minDistance)
          {
            closest_point_1 = pointId;
            closest_point_2 = neighbours[0];
            minDistance = distances[0];
          }
        }
      }

      return std::sqrt(minDistance);
    }
    
    /** \brief Given two pointclouds find the closest point between them
     *  \param[in]  cloud1             first cloud
     *  \param[in]  cloud2             second cloud
     *  \param[out] closest_point_1    closest point in the first cloud
     *  \param[out] closest_point_2    closest point in the second cloud
     *  \return distance between two clouds
     */
    template <typename PointT>
    float cloudToCloudDistance  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud1,
                                  const typename pcl::PointCloud<PointT>::ConstPtr &cloud2,
                                  int closest_point_1,
                                  int closest_point_2
                                )
    {
      // Prepare variables
      closest_point_1 = -1;
      closest_point_2 = -1;
      float minDistance = std::numeric_limits<float>::max();
      
      // Prepare search
      pcl::search::KdTree<PointT> search_tree;
      std::vector<int> neighbours(1);
      std::vector<float> distances(1);
      
      if (cloud1->size() > cloud2->size())
      {
        // First create a search tree for the second cloud
        search_tree.setInputCloud(cloud1);
        
        // Iterate over the points of the first cloud and find the closest distance to the second cloud        
        for (size_t pointId = 0; pointId < cloud2->size(); pointId++)
        {
          search_tree.nearestKSearch(cloud2->points[pointId], 1, neighbours, distances);
          if (distances[0] < minDistance)
          {
            closest_point_1 = neighbours[0];
            closest_point_2 = pointId;
            minDistance = distances[0];
          }
        }        
      }
      else
      {
        // First create a search tree for the second cloud
        search_tree.setInputCloud(cloud2);
        
        // Iterate over the points of the first cloud and find the closest distance to the second cloud        
        for (size_t pointId = 0; pointId < cloud1->size(); pointId++)
        {
          search_tree.nearestKSearch(cloud1->points[pointId], 1, neighbours, distances);
          if (distances[0] < minDistance)
          {
            closest_point_1 = pointId;
            closest_point_2 = neighbours[0];
            minDistance = distances[0];
          }
        }
      }

      return std::sqrt(minDistance);
    }    
  }
}

#endif  // POINTCLOUD_UTILITIES_HPP