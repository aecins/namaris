#ifndef LCCP_SEGMETTION_HPP
#define LCCP_SEGMETTION_HPP

// PCL
#include <pcl/segmentation/lccp_segmentation.h>

// CPP tools
#include <utilities/std_vector.hpp>
#include <utilities/pcl_typedefs.hpp>
#include <utilities/graph.hpp>
#include <utilities/map.hpp>

namespace alg
{
  /** \brief Apply LCCP segmentation algorithm to a set of connected oriented supervoxels.
   *  \param[in]  centroids             pointcloud represening the supervoxel cetroid locations and orientations
   *  \param[in]  adjacency             adjacency between the supervoxels
   *  \param[in]  concavity_threshold   concavity threshold of the LCCP algorithm 
   *  \param[out] segments_to_patches   map from segment indices to supervoxels indices
   *  \param[in]  min_segment_size      segments containing fewer than this many patches are merged with the largest adjacent segment
   */
  void lccpSegmentation ( const pcl::PointCloud<PointN> &centroids,
                          const utl::graph::Graph       &adjacency,
                          const float                   concavity_threshold,
                          utl::map::Map                 &segments_to_svoxels,
                          const int                     &min_segment_size = 0
                        )
  { 
    //--------------------------------------------------------------------------
    // Convert segmentation information
    //--------------------------------------------------------------------------
    
    // Adjacency
    std::multimap<uint32_t, uint32_t>   label_adjacency;
    std::vector<std::pair<int,int> > adjacency_edges = utl::graph::graph2EdgePairs(adjacency);
    
    for (size_t edgeId = 0; edgeId < adjacency_edges.size(); edgeId++)
    {
      label_adjacency.insert(std::pair<uint32_t,uint32_t>(static_cast<uint32_t>(adjacency_edges[edgeId].first),   static_cast<uint32_t>(adjacency_edges[edgeId].second)));
      label_adjacency.insert(std::pair<uint32_t,uint32_t>(static_cast<uint32_t>(adjacency_edges[edgeId].second),  static_cast<uint32_t>(adjacency_edges[edgeId].first)));
    }

    // Centroids
    std::map <uint32_t, pcl::Supervoxel<Point>::Ptr >  supervoxel_clusters;
    for (size_t clusterId = 0; clusterId < centroids.size(); clusterId++)
    {
      pcl::Supervoxel<Point>::Ptr curSupervoxel (new pcl::Supervoxel<Point>);
      curSupervoxel->normal_.getNormalVector3fMap() = centroids.points[clusterId].getNormalVector3fMap();
      curSupervoxel->centroid_.getVector3fMap()     = centroids.points[clusterId].getVector3fMap();
      supervoxel_clusters.insert(std::pair<uint32_t, pcl::Supervoxel<Point>::Ptr>(static_cast<uint32_t>(clusterId), curSupervoxel));
    }
      
    //--------------------------------------------------------------------------
    // Segment
    //--------------------------------------------------------------------------
    
    std::map<uint32_t, std::set<uint32_t> > segmentToSupervoxelMap;
    
    pcl::LCCPSegmentation<Point> lccp;
    lccp.setConcavityToleranceThreshold(concavity_threshold);
    lccp.setInputSupervoxels(supervoxel_clusters, label_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();
    lccp.getSegmentToSupervoxelMap(segmentToSupervoxelMap);
    
//     // PCL 1.7.2
//     std::map<uint32_t, std::vector<uint32_t> > segmentSupervoxelMap;
//     lccp.segment(supervoxel_clusters, label_adjacency);        
//     lccp.removeSmallSegments(min_segment_size);
//     lccp.getSegmentSupervoxelMap(segmentSupervoxelMap);
    
    //--------------------------------------------------------------------------
    // Convert labelled cloud to segmentation map
    //--------------------------------------------------------------------------

    segments_to_svoxels.resize(0);
    for (auto segIt = segmentToSupervoxelMap.begin(); segIt != segmentToSupervoxelMap.end(); segIt++)
      if (segIt->second.size() > 1)
        segments_to_svoxels.push_back(std::vector<int>(segIt->second.begin(), segIt->second.end()));      
  }
  
  /** \brief Remap LCCP segmentation to full cloud segmentation
   *  \param[in]  svoxels                 map from supervoxels to points
   *  \param[in]  segment_to_svoxel_map   map from segments to supervoxels
   *  \param[out]  lccp_segmentation      final segmentation (map from segments to points)
   */  
  void remapSvoxelsToLCCPSegments ( const utl::map::Map &svoxels,
                                    const utl::map::Map &segment_to_svoxel_map,
                                    utl::map::Map       &lccp_segmentation
                                  )
  {
    lccp_segmentation.resize(segment_to_svoxel_map.size());
    
    for (size_t segId = 0; segId < segment_to_svoxel_map.size(); segId++)
    {
      for (size_t svoxelIdIt = 0; svoxelIdIt < segment_to_svoxel_map[segId].size(); svoxelIdIt++)
      {
        int svoxelId = segment_to_svoxel_map[segId][svoxelIdIt];
        utl::stdvec::vectorAppend<int>(lccp_segmentation[segId], svoxels[svoxelId]);
      }
    }
  }  
}

#endif // LCCP_SEGMETTION_HPP