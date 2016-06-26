#ifndef FELZENSWALB_GRAPH_SEGMENTATION_HPP
#define FELZENSWALB_GRAPH_SEGMENTATION_HPP

// Utilities
#include <namaris/utilities/std_vector.hpp>

// Algorithms
#include <namaris/algorithms/felzenswalb_graph_segmentation/felzenswalb_graph_segmentation.h>

////////////////////////////////////////////////////////////////////////////////
alg::FelzenswalbGraphSegmentation::FelzenswalbGraphSegmentation()
  : num_edges_ (0)
  , num_vertices_ (0)
  , min_seg_size_ (1)
  , max_seg_size_ (std::numeric_limits<size_t>::max())
  , threshold_ (-1.0)
  , edges_ (NULL)
{ }

////////////////////////////////////////////////////////////////////////////////
alg::FelzenswalbGraphSegmentation::~FelzenswalbGraphSegmentation()
{
  delete [] edges_;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::FelzenswalbGraphSegmentation::setThreshold ( const float threshold )
{
  // Check input
  if (threshold < 0)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::setThreshold] threshold must be non-negative." << std::endl;
    return false;
  }
      
  threshold_ = threshold;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::FelzenswalbGraphSegmentation::setMinSegmentSize(const size_t min_seg_size)
{
  // Check input
  if (min_seg_size < 1)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::setMinSegmentSize] segments must contain at least one point." << std::endl;
    return false;
  }    
  
  min_seg_size_ = min_seg_size;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::FelzenswalbGraphSegmentation::setMaxSegmentSize(const size_t max_seg_size)
{
  // Check input
  if (max_seg_size < 1)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::setMaxSegmentSize] segments must contain at least one point." << std::endl;
    return false;
  }    
  
  max_seg_size_ = max_seg_size;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::FelzenswalbGraphSegmentation::setGraph ( const std::vector< std::pair< int, int > >& edges,
                                              const std::vector< float >& weights
                                            )
{
  if (edges.size() == 0)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::setGraph] input edges are empty!" << std::endl;
    return false;
  }
  
  // Check input
  if (edges.size() != weights.size())
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::setEdges] number of edges and number of weights must be the same." << std::endl;
    return false;
  }
  
  // Fill graph structure
  num_edges_ = edges.size();
  edges_ = new edge[num_edges_];
  for (size_t edgeId = 0; edgeId < num_edges_; edgeId++)
  {
    edges_[edgeId].a = edges[edgeId].first;
    edges_[edgeId].b = edges[edgeId].second;
    edges_[edgeId].w = weights[edgeId];
  }
  
  // Find number of vertices
  vertices_.clear();
  for (size_t edgeId = 0; edgeId < num_edges_; edgeId++)
  {
    vertices_.push_back(edges[edgeId].first);
    vertices_.push_back(edges[edgeId].second);
  }
  
  utl::stdvec::uniqueVector(vertices_);
  num_vertices_ = utl::stdvec::vectorMax(vertices_)+1;
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::FelzenswalbGraphSegmentation::segment(std::vector< std::vector< int > >& segments)
{
  // Check segmentation conditions
  if (edges_ == NULL)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::segment] edges are not set." << std::endl;
    return false;
  }
  
  if (threshold_ == -1.0)
  {
    std::cout << "[alg::FelzenswalbGraphSegmentation::segment] threhsold is not set." << std::endl;
    return false;
  }
  
  // Segment
  universe *segmentation = segment_graph(num_vertices_, num_edges_, edges_, threshold_);
  
  // Extract results
  segments.clear();
  
  std::vector<int> segIdMap;
  for (size_t vtxIdIt = 0; vtxIdIt < vertices_.size(); vtxIdIt++)
  {
    int vtxId = vertices_[vtxIdIt];
    
    // Get segment labels and check that it it's size is appropriate
    int segIdOriginal = segmentation->find(vtxId);
    if (segmentation->size(segIdOriginal) < min_seg_size_ || segmentation->size(segIdOriginal) > max_seg_size_)
      continue;
    
    // Remap original segment label to continuous integer range
    int segIdMapped;
    std::vector<int>::iterator segIdIt = std::find(segIdMap.begin(), segIdMap.end(), segIdOriginal);
    if (segIdIt == segIdMap.end())
    {
      segIdMapped = segIdMap.size();
      segIdMap.push_back(segIdOriginal);
      segments.push_back(std::vector<int> ());
    }
    else
    {
      segIdMapped = std::distance(segIdMap.begin(), segIdIt);
    }
    
    segments[segIdMapped].push_back(vtxId);
  }
  
  delete segmentation;
  return true;
}

#endif  // FELZENSWALB_GRAPH_SEGMENTATION_HPP