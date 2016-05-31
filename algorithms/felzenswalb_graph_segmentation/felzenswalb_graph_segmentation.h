#ifndef FELZENSWALB_GRAPH_SEGMENTATION_H
#define FELZENSWALB_GRAPH_SEGMENTATION_H

#include "segment-graph.hpp"

namespace alg
{
  /** \brief @b FelzenswalbGraphSegmentation performs segmentation of a weighted
   * graph using the approach described in
   * "Efficient Graph-Based Image Segmentation" by Felzenswalb et al.
   */  
  class FelzenswalbGraphSegmentation
  {
  public:
    
    /** \brief Empty constructor. */
    FelzenswalbGraphSegmentation ();
    
    /** \brief Desctructor. */
    ~FelzenswalbGraphSegmentation ();
    
    /** \brief Set the segmentation threshold controlling the over/under segmentation. */
    bool setThreshold ( const float threshold );
    
    /** \brief Set the size of the smallest valid segment. */
    bool setMinSegmentSize ( const size_t min_seg_size );

    /** \brief Set the size of the smallest valid segment. */
    bool setMaxSegmentSize ( const size_t max_seg_size );
    
    /** \brief Set the edges describing the graph strucutre.
     *  \param[in]   edges   vector of pairs of integers where integers correspond to indices of of the vertices connected by an edge
     *  \param[in]   weights vector of floats where each float corresponds to a weight of an edge.
     *  \note vertex ids must be positive
     *  \note edges and weights must be same size
     */
    bool setGraph (const std::vector<std::pair<int, int> > &edges, const std::vector<float> &weights);
    
    /** \brief Segment the graph.
     *  \param[out]   segments  a vector of segments where each segment is represented by the indices of points belonging to it
     */    
    bool segment (std::vector<std::vector<int> > &segments);
    
  private:
    
    // Member variables
    int num_edges_;
    int num_vertices_;
    size_t min_seg_size_;
    size_t max_seg_size_;
    float threshold_;
    edge *edges_;
    std::vector<int> vertices_;
  };
}

#endif // FELZENSWALB_GRAPH_SEGMENTATION_H
