#ifndef FELZENSWALB_SEGMENTATION_HPP
#define FELZENSWALB_SEGMENTATION_HPP

// STD includes
#include <math.h>

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Utilities
#include <namaris/utilities/graph.hpp>
#include <namaris/utilities/pointcloud.hpp>

// Algorithms
#include <namaris/algorithms/felzenswalb_segmentation/felzenswalb_segmentation.h>
#include <namaris/algorithms/felzenswalb_graph_segmentation/felzenswalb_graph_segmentation.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
alg::FelzenswalbSegmentation<PointT>::FelzenswalbSegmentation () :
  min_pts_per_segment_ (1),
  max_pts_per_segment_ (std::numeric_limits<int>::max ()),
  seg_threshold_ (10.0),
  num_neighbors_ (5),
  neighbor_radius_ (-1.0),
  recompute_graph_ (true)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
alg::FelzenswalbSegmentation<PointT>::~FelzenswalbSegmentation ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
alg::FelzenswalbSegmentation<PointT>::getMinSegmentSize () const
{
  return (min_pts_per_segment_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setMinSegmentSize (const int min_segment_size)
{
  min_pts_per_segment_ = min_segment_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
alg::FelzenswalbSegmentation<PointT>::getMaxSegmentSize () const
{
  return (max_pts_per_segment_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setMaxSegmentSize (const int max_segment_size)
{
  max_pts_per_segment_ = max_segment_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> unsigned int
alg::FelzenswalbSegmentation<PointT>::getNumberOfNeighbors () const
{
  return (num_neighbors_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setNumberOfNeighbors (const unsigned int num_neighbors)
{
  num_neighbors_ = num_neighbors;
  recompute_graph_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setEdgeWeightFunction (float (*edge_weight_function) (const PointT&, const PointT&))
{
  edge_weight_function_ = edge_weight_function;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::getPointGraph (std::vector<std::pair<int, int> > &edges, std::vector<float> &edge_weights) const
{
  if (edges_.size() == 0)
  {
    std::cout << "[alg::FelzenswalbSegmentation::getPointGraph] point graph is empty, you must segment first to generate the graph." << std::endl;
    return;
  }
  
  edges = edges_;
  edge_weights = edge_weights_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
alg::FelzenswalbSegmentation<PointT>::getNeighborRadius () const
{
  return (neighbor_radius_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setNeighborRadius (const float neighbor_radius)
{
  neighbor_radius_ = neighbor_radius;
  recompute_graph_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> float
alg::FelzenswalbSegmentation<PointT>::getThreshold () const
{
  return (seg_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::setThreshold (const float seg_threshold)
{
  seg_threshold_ = seg_threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::calculateEdgeWeights ()
{
  edge_weights_.resize(edges_.size());
  
  for (size_t edgeId = 0; edgeId < edges_.size(); edgeId++)
    edge_weights_[edgeId] = edge_weight_function_(input_->points[edges_[edgeId].first], input_->points[edges_[edgeId].second]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
alg::FelzenswalbSegmentation<PointT>::segment (std::vector<std::vector<int> > &segments)
{
  // First check that segmentation is possible
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    std::cout << "[alg::FelzenswalbSegmentation::segment] could not initialize segmentation." << std::endl;
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    std::cout << "[alg::FelzenswalbSegmentation::segment] could not initialize segmentation." << std::endl;
    deinitCompute ();
    return;
  }
  
  // Find graph connectivity
  if (recompute_graph_)
  {
    utl::graph::Graph edge_graph;
    utl::cloud::getCloudConnectivityRadius<PointT>  (input_, *indices_, edge_graph, neighbor_radius_, num_neighbors_);
    edges_ = utl::graph::graph2EdgePairs(edge_graph);
    recompute_graph_ = false;
  }
  
  // Estimate graph weights
  calculateEdgeWeights();
  
  // Segment
  alg::FelzenswalbGraphSegmentation seg;
  seg.setGraph(edges_, edge_weights_);
  seg.setThreshold(seg_threshold_);
  seg.setMinSegmentSize(min_pts_per_segment_);
  seg.setMaxSegmentSize(max_pts_per_segment_);
  seg.segment(segments);
  
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
alg::FelzenswalbSegmentation<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] input cloud is empty!" << std::endl;
    return (false);
  }

  // from here we check those parameters that are always valuable
  if (num_neighbors_ == 0)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] number of neighbors is 0!" << std::endl;
    return (false);
  }
  
  if (neighbor_radius_ <= 0)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] neighbor radius is less than or equal than zero!" << std::endl;
    return (false);
  }

  if (seg_threshold_ < 0)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] segmentation threshold is less than zero!" << std::endl;
    return (false);
  }
  
  if (!edge_weight_function_)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] Edge weight function is not defined!" << std::endl;
    return false;
  }
  
  
  if (min_pts_per_segment_ < 1)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] minimum segment size is less than one!" << std::endl;
    return (false);
  }
  
  if (max_pts_per_segment_ < 1)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] maximum segment size is less than one!" << std::endl;
    return (false);
  }

  if (max_pts_per_segment_ < min_pts_per_segment_)
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] maximum segment size is smaller than minimum segment size!" << std::endl;
    return (false);
  }
  
  if (indices_ && indices_->empty ())
  {
    std::cout << "[alg::FelzenswalbSegmentation::prepareForSegmentation] Input indices are empty!" << std::endl;
    return false;
  }
  
  return (true);
}

#endif    // FELZENSWALB_SEGMENTATION_HPP
