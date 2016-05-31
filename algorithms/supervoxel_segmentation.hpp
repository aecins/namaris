#ifndef SUPERVOXEL_SEGMENTATION_HPP
#define SUPERVOXEL_SEGMENTATION_HPP

// PCL
#include <pcl/common/angles.h>
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>
#include <pcl/surface/gp3.h>

// CPP tools
#include <utilities/std_vector.hpp>
#include <utilities/pcl_typedefs.hpp>
#include <utilities/graph.hpp>
#include <utilities/map.hpp>
#include <utilities/math.hpp>

namespace alg
{
  /** \brief Oversegment input pointcloud
   *  \param[in]  cloud               input pointcloud
   *  \param[in]  voxel_resolution    spatial resolution of the voxelgrid filter applied before segmentation
   *  \param[in]  seed_resolution     distance between cluster seeds
   *  \param[in]  spatial_importance  weight of the Euclidean distance term used for calculating distances between points and cluster seeds
   *  \param[in]  normal_importance   weight of the normal difference term used for calculating distances between points and cluster seeds
   *  \param[out] point_to_svoxel_map a vector where each element represents a point and holds the supervoxel id it belongs to (0 is reserved for no label)
   *  \param[out] adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
   *  \param[out] centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
   *  \note: this method assumes that the input pointcloud has consistently 
   * oriented normals and attempts to ensure that supervoxel normals are also 
   * consistently oriented.
   */
  template <typename PointT>
  bool supervoxelSegmentation(  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const float voxel_resolution,
                                const float seed_resolution,
                                const float spatial_importance,
                                const float normal_importance,
                                utl::map::Map &svoxels,
                                utl::graph::Graph &adjacency,
                                pcl::PointCloud<PointN> &centroids
                             )
  {
    // Parameters
    int numRefinementIterations = 5;
    
    // Extract normal and point data into separate clouds
    pcl::PointCloud<Point>::Ptr cloudPoints  (new pcl::PointCloud<Point>);
    pcl::PointCloud<Normal>::Ptr cloudNormals (new pcl::PointCloud<Normal>);
    pcl::copyPointCloud(*cloud, *cloudPoints);
    pcl::copyPointCloud(*cloud, *cloudNormals);

    // Setup supervoxel clustering
    pcl::SupervoxelClustering<Point> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(false);
    super.setInputCloud(cloudPoints);
    super.setNormalCloud(cloudNormals);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    super.setColorImportance(0.0f);

    // Segment
    std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >  supervoxel_clusters;      
    std::multimap<uint32_t, uint32_t>                   label_adjacency;
    super.extract (supervoxel_clusters);                                          // Get supervoxel clusters
    super.refineSupervoxels(numRefinementIterations, supervoxel_clusters);                              // Refine clusters
    super.getSupervoxelAdjacency (label_adjacency);                               // Get supervoxel adjacency
    pcl::PointCloud<PointL>::Ptr labeledCloud = super.getLabeledCloud ();        // Get input cloud labels
    uint32_t maxSegmentId = super.getMaxLabel();
                 
    // Extract cluster assignment information    
    svoxels = utl::map::Map(maxSegmentId+1);
    for (size_t i = 0; i < labeledCloud->size(); i++)
    {
      uint32_t curLabel = labeledCloud->points[i].label;
      svoxels[curLabel].push_back(i);
    }

    // Extract adjacency information
    adjacency.resize(super.getMaxLabel()+1);
    for (auto edgeIt = label_adjacency.begin(); edgeIt != label_adjacency.end(); edgeIt++)
      utl::graph::addEdge(edgeIt->first, edgeIt->second, adjacency);

    // Extract centroids
    centroids.resize(super.getMaxLabel()+1);
    for (auto centroidIt = supervoxel_clusters.begin(); centroidIt != supervoxel_clusters.end(); centroidIt++)
      centroidIt->second->getCentroidPointNormal(centroids[centroidIt->first]);
    
    // Reorient centroid normals
    // NOTE: supervoxel clustering computes supervoxel normal by estimating a 
    // surface normal from all of the points in the supervoxel. As a result even
    // if the original point normals were consistently oriented, supervoxel 
    // normals are not guaranteed to be. To fix this we unsure that supervoxel
    // normal is consistent with the normals of the 5 points closest to its
    // centroid.    
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);
  
    for (size_t centroidId = 0; centroidId < centroids.size(); centroidId++)
    {
      // Find points nearest to the centroid center
      int numNeighbours = 5;
      std::vector<int> neighbours (numNeighbours);
      std::vector<float> distances (numNeighbours);
      
      PointT centroidPoint;
      centroidPoint.getVector3fMap() = centroids.at(centroidId).getVector3fMap();
      searchTree.nearestKSearch(centroidPoint, numNeighbours, neighbours, distances);
      
      int numFlip = 0;
      int numNoFlip = 0;
      
      for (size_t nbrIdIt = 0; nbrIdIt < static_cast<size_t>(numNeighbours); nbrIdIt++)
      {
        if (cloud->at(neighbours[nbrIdIt]).getNormalVector3fMap().dot(centroids.at(centroidId).getNormalVector3fMap()) < 0)
          numFlip++;
        else
          numNoFlip++;
        
        if (numFlip > numNoFlip)
          centroids.at(centroidId).getNormalVector3fMap() *= -1;
      }
    }    
    
    return true;
  }

  /** \brief Merge supervoxels given a merge graph indicating which supervoxels should be merged together
   *  \param[in]  merge_graph                  a graph indicacating which supervoxels must be merged together
   *  \param[in]  point_to_svoxel_map          a vector where each element represents a point and holds the supervoxel id it belongs to
   *  \param[in]  adjacency                    a vector where each element represents a cluster and holds the indices of its neighbouring clusters
   *  \param[in]  centroids                    a vector where each element represents a cluster and holds cluster centroid location and normal
   *  \param[out] point_to_svoxel_map_merged   a vector where each element represents a point and holds the supervoxel id it belongs to after merging
   *  \param[out] adjacency_merged             a vector where each element represents a cluster and holds the indices of its neighbouring clusters after merging
   *  \param[out] centroids_merged             a vector where each element represents a cluster and holds cluster centroid location and normal after merging
   */
  void mergeSupervoxels(  const utl::map::Map             &merge_map,
                          const utl::map::Map             &svoxels,
                          const utl::graph::Graph         &adjacency,
                          const pcl::PointCloud<PointN>   &centroids,
                          utl::map::Map                   &svoxels_merged,
                          utl::graph::Graph               &adjacency_merged,
                          pcl::PointCloud<PointN>         &centroids_merged
                        )
  {
//     //////////////////////////////////////////////////////////////////////////////
//     // Find a Next find all connected components in this graph and find a mapping from 
//     // original segment id to merged segment id
//     
//     utl::map::Map mergedSeg2Seg = utl::graph::getConnectedComponents(merge_graph);
//     
//     std::vector<size_t> seg2mergedSegMap (centroids.size());
//     for (size_t CCId = 0; CCId < mergedSeg2Seg.size(); CCId++)
//     {
//       for (size_t segIdIt = 0; segIdIt < mergedSeg2Seg[CCId].size(); segIdIt++)
//       {
//         int segId = mergedSeg2Seg[CCId][segIdIt];
//         seg2mergedSegMap[segId] = CCId;
//       }      
//     }

    //--------------------------------------------------------------------------
    // Find inverse merge map
    utl::map::Map merge_map_inverse = utl::map::invertMap(merge_map);
    
    //--------------------------------------------------------------------------
    // Point svoxels -> point merged svoxels
    
    svoxels_merged.resize(merge_map.size());
    for (size_t mergedSegId = 0; mergedSegId < svoxels_merged.size(); mergedSegId++)
      for (size_t segIdIt = 0; segIdIt < merge_map[mergedSegId].size(); segIdIt++)
        utl::stdvec::vectorAppend<int>(svoxels_merged[mergedSegId], svoxels[merge_map[mergedSegId][segIdIt]]);
    
    //--------------------------------------------------------------------------
    // Adjacency -> merged adjaceny
    
    adjacency_merged.resize(merge_map.size());
    
    for (size_t mergedSegId = 0; mergedSegId < merge_map.size(); mergedSegId++)
    {
      for (size_t origSegIdIt = 0; origSegIdIt < merge_map[mergedSegId].size(); origSegIdIt++)
      {
        size_t origSegId = merge_map[mergedSegId][origSegIdIt];
        
        for (size_t origSegNghbrIdIt = 0; origSegNghbrIdIt < adjacency[origSegId].size(); origSegNghbrIdIt++)
        {
          size_t origSegNghbrId = adjacency[origSegId][origSegNghbrIdIt];
          size_t mergedSegNghbrId = merge_map_inverse[origSegNghbrId][0];

          // Don't need to add link between vertex and itself
          if (mergedSegId != mergedSegNghbrId)
            utl::graph::addEdge(mergedSegId, mergedSegNghbrId, adjacency_merged);
        }
      }    
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Centroids -> merged centroids
    centroids_merged.resize(merge_map.size());
    for (size_t mergeSegId = 0; mergeSegId < merge_map.size(); mergeSegId++)
    {
      PointN centroid;
      centroid.getVector3fMap()       = Eigen::Vector3f(0.0, 0.0, 0.0);
      centroid.getNormalVector3fMap() = Eigen::Vector3f(0.0, 0.0, 0.0);
      centroid.curvature              = 0;
      
      int numOrigSegs = merge_map[mergeSegId].size();
      
      for (size_t origSegIdItr = 0; origSegIdItr < static_cast<size_t>(numOrigSegs); origSegIdItr++)
      {
        size_t origSegId = merge_map[mergeSegId][origSegIdItr];
        centroid.getVector3fMap()         += centroids[origSegId].getVector3fMap();
        centroid.getNormalVector3fMap()   += centroids[origSegId].getNormalVector3fMap();
        centroid.curvature                += centroids[origSegId].curvature;
      }
      
      centroid.getVector3fMap()       /= numOrigSegs;
      centroid.getNormalVector3fMap() /= centroid.getNormalVector3fMap().norm();  // Don't forget to normalize the normals!
      centroid.curvature              /= numOrigSegs;
      
      centroids_merged.points[mergeSegId] = centroid;
    }
  }

  /** \brief Merge supervoxels using the centroid normal criterion
   *  \param[in]  point_to_svoxel_map          a vector where each element represents a point and holds the supervoxel id it belongs to
   *  \param[in]  adjacency                    a vector where each element represents a cluster and holds the indices of its neighbouring clusters
   *  \param[in]  centroids                    a vector where each element represents a cluster and holds cluster centroid location and normal
   *  \param[out] point_to_svoxel_map_merged   a vector where each element represents a point and holds the supervoxel id it belongs to after merging
   *  \param[out] adjacency_merged             a vector where each element represents a cluster and holds the indices of its neighbouring clusters after merging
   *  \param[out] centroids_merged             a vector where each element represents a cluster and holds cluster centroid location and normal after merging
   *  \param[in]  normal_difference_thresh     angular difference between normals of two supervoxels
   */
  void mergeSupervoxelsCentroidNormal(  const utl::map::Map           &svoxels,
                                        const utl::graph::Graph       &adjacency,
                                        const pcl::PointCloud<PointN> &centroids,
                                        utl::map::Map                 &svoxels_merged,
                                        utl::graph::Graph             &adjacency_merged,
                                        pcl::PointCloud<PointN>       &centroids_merged,
                                        const float                   normal_difference_thresh
                        )
  {
    // First generate a graph structure where vertices correspond to supervoxels
    // and edges correspond to a merge between supervoxels
    utl::graph::Graph merge_graph(centroids.size());
    std::vector<std::pair<int, int> > adjacencyEdges = utl::graph::graph2EdgePairs(adjacency);
    
    for (auto edgeIt = adjacencyEdges.begin(); edgeIt != adjacencyEdges.end(); edgeIt++)
    {
      Eigen::Vector3f sourceNormal = centroids.points[edgeIt->first].getNormalVector3fMap();
      Eigen::Vector3f targetNormal = centroids.points[edgeIt->second].getNormalVector3fMap();      
      float normalDiffAngle   = acos(utl::math::clampValue(sourceNormal.dot(targetNormal), -1.0f, 1.0f));

      if (normalDiffAngle < normal_difference_thresh)
        utl::graph::addEdge(edgeIt->first, edgeIt->second, merge_graph);
    }

    // Convert it to a map from merged supervoxels to original supervoxels
    // and edges correspond to a merge between supervoxels
    utl::map::Map merge_map = utl::graph::getConnectedComponents(merge_graph);
        
    // Then merge supervoxles given the merge graph
    mergeSupervoxels(merge_map, svoxels, adjacency, centroids, svoxels_merged, adjacency_merged, centroids_merged);

  }

//   /** \brief Merge supervoxels greedily
//    *  \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
//    *  \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
//    *  \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
//    *  \param[in]  normal_difference_thresh    angular difference between normals of two supervoxels
//    */
//   void mergeSupervoxelsBoundaryCurvature( const std::vector<pcl::PointCloud<PointN> > &boundary_points,
//                                           const std::vector<std::pair<int, int> >     &boundary_point_labels,
//                                           const utl::Pts2Seg                          &point_to_svoxel_map,
//                                           const utl::graph::Graph                     &adjacency,
//                                           const pcl::PointCloud<PointN>               &centroids,
//                                           utl::Pts2Seg                                &point_to_svoxel_map_merged,
//                                           utl::graph::Graph                           &adjacency_merged,
//                                           pcl::PointCloud<PointN>                     &centroids_merged,
//                                           const float                                 min_average_curvature                                        
//                         )
//   {
//     //////////////////////////////////////////////////////////////////////////////
//     // First generate a graph structure where vertices correspond to supervoxels
//     // and edges correspond to merge between supervoxels
//     // NOTE: the criterion for merging two pathces should be improved (it should depend on the curvature at the edge)
//     utl::graph::Graph mergeGraph(centroids.size());
// 
//     for (size_t bdrId = 0; bdrId < boundary_point_labels.size(); bdrId++)
//     {
//       // Compute average curvature for a boundary
//       float averageCurvature = 0;
//       for (size_t pointId = 0; pointId < boundary_points[bdrId].size(); pointId++)
//         averageCurvature += boundary_points[bdrId].points[pointId].curvature;
//       averageCurvature /= boundary_points[bdrId].size();
//       
//       // If it is smaller than threshold - merge corresponding segments
//       if (averageCurvature < min_average_curvature)
//         utl::graph::addEdge(boundary_point_labels[bdrId].first, boundary_point_labels[bdrId].second, mergeGraph);
//     }
//     
//     // Then merge supervoxles given the merge graph
//     mergeSupervoxels(mergeGraph, point_to_svoxel_map, adjacency, centroids, point_to_svoxel_map_merged, adjacency_merged, centroids_merged);
//   }
// 
//   /** \brief Merge supervoxels greedily
//    *  \param[in]  clusters            a vector where each element represents a cluster and holds the indices of points in the input cloud belonging to the cluster
//    *  \param[in]  adjacency           a vector where each element represents a cluster and holds the indices of its neighbouring clusters
//    *  \param[in]  centroids           a vector where each element represents a cluster and holds cluster centroid location and normal
//    *  \param[in]  normal_difference_thresh    angular difference between normals of two supervoxels
//    */
//   void mergeSupervoxelsBoundaryCurvatureCentroidNormal (
//                                           const std::vector<pcl::PointCloud<PointN> > &boundary_points,
//                                           const std::vector<std::pair<int, int> >     &boundary_point_labels,
//                                           const utl::Pts2Seg                          &point_to_svoxel_map,
//                                           const utl::graph::Graph                     &adjacency,
//                                           const pcl::PointCloud<PointN>               &centroids,
//                                           utl::Pts2Seg                                &point_to_svoxel_map_merged,
//                                           utl::graph::Graph                           &adjacency_merged,
//                                           pcl::PointCloud<PointN>                     &centroids_merged,
//                                           const float                                 min_average_curvature,                                        
//                                           const float                                 normal_difference_thresh                                        
//                         )
//   {
//     //////////////////////////////////////////////////////////////////////////////
//     // First generate a graph structure where vertices correspond to supervoxels
//     // and edges correspond to merge between supervoxels
//     // NOTE: the criterion for merging two pathces should be improved (it should depend on the curvature at the edge)
//     utl::graph::Graph mergeGraph(centroids.size());
// 
//     for (size_t bdrId = 0; bdrId < boundary_point_labels.size(); bdrId++)
//     {
//       int sourceSegId = boundary_point_labels[bdrId].first;
//       int targetSegId = boundary_point_labels[bdrId].second;
//       
//       // Compute average curvature for a boundary
//       float averageCurvature = 0;
//       for (size_t pointId = 0; pointId < boundary_points[bdrId].size(); pointId++)
//         averageCurvature += boundary_points[bdrId].points[pointId].curvature;
//       averageCurvature /= boundary_points[bdrId].size();
//       
//       // Compute centroid normal difference between two suoervoxels
//       Eigen::Vector3f sourceNormal = centroids.points[sourceSegId].getNormalVector3fMap();
//       Eigen::Vector3f targetNormal = centroids.points[targetSegId].getNormalVector3fMap();      
//       float normalDiffAngle   = acos(sourceNormal.dot(targetNormal));
//       
//       // If it is smaller than threshold - merge corresponding segments
//       if ((averageCurvature < min_average_curvature) || (normalDiffAngle < normal_difference_thresh))
//         utl::graph::addEdge(sourceSegId, targetSegId, mergeGraph);
//       
//     }
//     
//     // Then merge supervoxles given the merge graph
//     mergeSupervoxels(mergeGraph, point_to_svoxel_map, adjacency, centroids, point_to_svoxel_map_merged, adjacency_merged, centroids_merged);
//   }
// 
  
  /** \brief Cleanup segmentation by swithcing labels for points who are surrounded by sufficiently many points with different label
   *  \param[in]  cloud                         pointcloud
   *  \param[in]  point_to_svoxel_map           a vector where each element represents a point and holds the supervoxel id it belongs to
   *  \param[in]  point_to_svoxel_map_cleanedup a vector where each element represents a point and holds the supervoxel id it belongs to after cleanup
   *  \param[out] normal_difference_thresh      angular difference between normals of two supervoxels
   *  \param[in]  num_neighbours                number of neighbors analyzed
   *  \param[in]  threshold_ratio               minimum number of neighbors with different label required to switch label
   *  \return number of points for which the label was changed
   */
  template <typename PointT>
  size_t cleanupSegmentation  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const utl::map::Map &svoxels,
                                utl::map::Map &svoxels_cleanedup,
                                const int num_neighbors = 8,
                                const int threshold_ratio = 5
                              )
  {    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud);
    
    int labelsChanged = 0;
    svoxels_cleanedup = utl::map::Map(svoxels.size());
    utl::map::Map svoxels_inverse = utl::map::invertMap(svoxels);
    if (!utl::map::isInjective(svoxels_inverse))
    {
      std::cout << "[alg::cleanupSegmentation] inverse of segmentation map is not onjective, something is wrong. Aborting..." << std::endl;
      abort();
    }
    
    // Loop over all points
    for (size_t pointId = 0; pointId < svoxels_inverse.size(); pointId++)
    {
      int curPointLabel = svoxels_inverse[pointId][0];
      
      // Find nearest neighbours
      std::vector<float>  distances(num_neighbors);
      std::vector<int>    neighbors(num_neighbors);
      searchTree.nearestKSearch(pointId, num_neighbors, neighbors, distances);
      
      // Get the labels of neighbours
      std::vector<int> neighborLabels = utl::map::remapVector(neighbors, svoxels_inverse);
      
      // Get most frequent label and cur label counts      
      int mostFreqLabel;
      size_t mostFreqLabelCount;
      utl::stdvec::vectorMode<int>(neighborLabels, mostFreqLabel, mostFreqLabelCount, utl::stdvec::DESCENDING);
      int origLabelCount = utl::stdvec::vectorCount<int>(neighborLabels, curPointLabel);
            
      // Change label
      if ((mostFreqLabel != curPointLabel) && (origLabelCount <  (num_neighbors - threshold_ratio)))
      {
        svoxels_cleanedup[mostFreqLabel].push_back(pointId);
        labelsChanged++;
      }
      else
        svoxels_cleanedup[curPointLabel].push_back(pointId);
    }
    
    return labelsChanged;
  }

  /** \brief Generate a mesh for an input pointcloud
   *  \param[in]  cloud               input pointcloud
   *  \param[out] triangles           output mesh
   */
  template <typename PointT>
  void meshSupervoxel ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                        pcl::PolygonMesh                                  &triangles
                      )
  {
    // Set verbosity level to ERROR only
    pcl::console::VERBOSITY_LEVEL curVerbosityLevel = pcl::console::getVerbosityLevel();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    
    // Create search tree*
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    
    // Use Greedy Projection Triangulation
    pcl::GreedyProjectionTriangulation<PointT> gpt;
    gpt.setSearchRadius(0.05);                           // Maximum distance between connected points
    gpt.setMu(2.0);                                      // Radius multiplier ???
    gpt.setMaximumNearestNeighbors(20);                   // Maximum number of neighbours
    gpt.setMaximumSurfaceAngle(pcl::deg2rad(45.0));      // Maximum normal angle difference between connected points
    gpt.setMinimumAngle(pcl::deg2rad(10.0));            // Minimum angle of each surface trinagle
    gpt.setMaximumAngle(pcl::deg2rad(120.0));           // Maximim angle of each triangle
    gpt.setNormalConsistency(true);                     // Set if the normals in the input cloud are consistent
    
    gpt.setInputCloud(cloud);
    gpt.setSearchMethod(tree);
    gpt.reconstruct(triangles);
    
    // Revert verbosity level
    pcl::console::setVerbosityLevel(curVerbosityLevel);
  }
}

#endif // SUPERVOXEL_SEGMENTATION_HPP