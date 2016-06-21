#ifndef POINCLOUD_OCCLUSION_BOUNDARIES_HPP
#define POINCLOUD_OCCLUSION_BOUNDARIES_HPP

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

// Utilities
#include <namaris/utilities/math.hpp>
#include <namaris/utilities/geometry.hpp>
#include <namaris/utilities/std_vector.hpp>

namespace alg
{
  template <typename PointT>
  void pointcloudOcclusionBoundaries  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                        const std::vector<int>                  &indices,
                                        std::vector<int>                        &boundary_point_ids,
                                        const double                            radius,
                                        const float                             max_angle = pcl::deg2rad(135.0)
                                      )
  {
    boundary_point_ids.resize(0);
    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud, boost::make_shared<std::vector<int> > (indices));

    // Loop over points
    for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
    {
      int pointId = indices[pointIdIt];
      PointT curPoint = cloud->points[pointId];
      
      // Find neighbours in a radius
      std::vector<float>  distances;
      std::vector<int>    neighbours;
      searchTree.radiusSearch(pointIdIt, radius, neighbours, distances);
      neighbours.erase(neighbours.begin());     // Remove first element since it is the point itself
      
      // If current point has no neighbours mark it as occluded
      if (neighbours.empty())
      {
        boundary_point_ids.push_back(pointId);
        continue;
      }
      
      // Project them on the tangent plane
      Eigen::Vector3f planePoint  = curPoint.getVector3fMap();
      Eigen::Vector3f planeNormal = curPoint.getNormalVector3fMap();
      std::vector<Eigen::Vector3f> projectedNeighbours(neighbours.size());
      
      for (size_t neighbourId = 0; neighbourId < neighbours.size(); neighbourId++)
      {
        Eigen::Vector3f neighbourPoint          = cloud->points[neighbours[neighbourId]].getVector3fMap();
        Eigen::Vector3f neighbourPointProjected = utl::geom::projectPointToPlane<float>(neighbourPoint, planePoint, planeNormal);
        projectedNeighbours[neighbourId]        = neighbourPointProjected;
      }
      
      // Calculate signed angles between first vector and all other vectors
      Eigen::Vector3f referenceVector = projectedNeighbours[0] - planePoint;
      std::vector<float> angles (neighbours.size());
      for (size_t neighbourId = 0; neighbourId < projectedNeighbours.size(); neighbourId++)
      {
        Eigen::Vector3f currentVector = projectedNeighbours[neighbourId] - planePoint;
        float curAngle = utl::geom::vectorAngleSigned<float>(referenceVector, currentVector, planeNormal);
        angles[neighbourId] = curAngle;

      }

      // Calculate difference between consecutinve angles
      std::sort(angles.begin(), angles.end());
      std::vector<float> angleDifference(angles.size());
      for (size_t i = 1; i < angles.size(); i++)
      {
        angleDifference[i] = utl::geom::angleDifferenceCCw<float>(angles[i-1], angles[i]);
      }
      angleDifference[0] = utl::geom::angleDifferenceCCw<float>(angles[angles.size()-1], angles[0]);
      
      // If maximum difference is bigger than threshold mark point as boundary point
      if (utl::stdvec::vectorMax(angleDifference) > max_angle)
        boundary_point_ids.push_back(pointId);
    }  
  }
  
  template <typename PointT>
  void pointcloudOcclusionBoundaries  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                        std::vector<int>                        &boundary_point_ids,
                                        const double                            radius,
                                        const float                             max_angle = pcl::deg2rad(135.0)
                                      )
  {
    std::vector<int> fake_indices (cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      fake_indices[pointId] = pointId;
    
    pointcloudOcclusionBoundaries<PointT>(cloud, fake_indices, boundary_point_ids, radius, max_angle);
  }
}

#endif  // POINCLOUD_OCCLUSION_BOUNDARIES_HPP