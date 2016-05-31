#ifndef FELZENSWALB_SEGMENTATION_H
#define FELZENSWALB_SEGMENTATION_H

namespace alg
{
  /** \brief @b FelzenswalbSegmentation
   * Segments a pointcloud with normals using the graph segmentation approach
   * described in "Efficient Graph-Based Image Segmentation" by Felzenswalb et
   * al.
   * 
   * The algorithm works a follows. First a graph is defined on the pointcloud 
   * by connecting each point to k of it's nearest neighbors that are within a 
   * radius r from it. The weight of each edge is calculated based on the 
   * difference of normals between two neighboring points as well as the 
   * distance between them. The resulting weighted graph is then segmented using
   * the approach of Felzenswalb et al.
   */
  template <typename PointT>
  class FelzenswalbSegmentation : public pcl::PCLBase<PointT>
  {
    public:
      
      typedef boost::function<float (const PointT&, const PointT&)> edgeWeightFunction;

      using pcl::PCLBase <PointT>::input_;
      using pcl::PCLBase <PointT>::indices_;
      using pcl::PCLBase <PointT>::initCompute;
      using pcl::PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      FelzenswalbSegmentation ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */
      virtual
      ~FelzenswalbSegmentation ();

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      int
      getMinSegmentSize ()  const;

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMinSegmentSize (const int min_cluster_size);

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int
      getMaxSegmentSize () const;

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMaxSegmentSize (const int max_cluster_size);

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbors () const;

      /** \brief Set maximum number of neighbors for each point. */
      void
      setNumberOfNeighbors ( const unsigned int num_neighbors_);      

      /** \brief Returns the maximum distance between point and it's neighbor. */
      float
      getNeighborRadius () const;

      /** \brief Set the maximum distance between point and it's neighbor. */
      void
      setNeighborRadius ( const float num_neighbors_);
      
      /** \brief Returns the segmentation threshold. */
      float
      getThreshold () const;

      /** \brief Set the segmentation threshold. */
      void
      setThreshold ( const float seg_threshold);
            
      /** \brief Set condition function that needs to be satisfied for two neighbouring points to be in the same cluster */
      void
      setEdgeWeightFunction (float (*edge_weight_function) (const PointT&, const PointT&));
      
      /** \brief Returns the graph generated on the pointcloud.
       * \note this method should be run after segmentation process.
       */
      void
      getPointGraph (std::vector<std::pair<int, int> > &edges, std::vector<float> &edge_weights) const;

      /** \brief Segment the pointcloud.
        * \param[out] segments  a vector of segments where each segment is represented by the indices of points belonging to it
        */
      virtual void
      segment (std::vector<std::vector<int> > &segments);

    protected:

      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();
      
      /** \brief Calculate the weights of the edges connecting points
        */
      virtual void
      calculateEdgeWeights ();

    protected:

      /** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
      int min_pts_per_segment_;

      /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int max_pts_per_segment_;

      /** \brief Segmentation threshold. */
      float seg_threshold_;

      /** \brief Number of neighbors for each point. */
      unsigned int num_neighbors_;
      
      /** \brief Maximum distance between a point and it's neighbor. */
      float neighbor_radius_;
      
      /** \brief The condition function between two points that needs to to be satisfied for two neighboring points to belong to the same cluster.  */
      edgeWeightFunction edge_weight_function_;
            
      /** \brief Point graph edges. */
      std::vector<std::pair<int, int> > edges_;
      
      /** \brief Point graph edge weights. */
      std::vector<float> edge_weights_;
      
      /** \brief Variable indicating if point graph should be recomputed. */
      bool recompute_graph_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif
