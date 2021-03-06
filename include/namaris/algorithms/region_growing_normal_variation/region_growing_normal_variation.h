#ifndef REGION_GROWING_NORMAL_VARIATION_H
#define REGION_GROWING_NORMAL_VARIATION_H

// PCL includes
#include <pcl/common/angles.h>

// Algorithms includes
#include <namaris/algorithms/region_growing/region_growing.hpp>

namespace alg
{
  /** \brief @b RegionGrowingNormalVariation Performs region growing
   * segmentation on a pointcloud using a measure of normal angle change per
   * distance between adjacent points. This is essintially a @b RegionGrowing
   * class with a prespecified binary condition function.
   * 
   * Given a point in the cloud it's neighboring points are found. For all of 
   * them a two conditions are verified:
   *  1) unary condition determining if a neighboring point is a valid candidate
   *  (can bes specified by the user)
   *  2) binary condition between the original point and a neighboring point. 
   *  Points belong to the same cluster if the angle between their normals 
   *  normalized by the distance between them is smaller than a user specified
   *  threshold.
   * 
   * If the fraction of neighbors satisfying the unary constraint is larger than
   * a threshold and the fraction of neighbors satisfying the binary constraint 
   * is larger than (a different) threshold neighboring points satisfying both 
   * conditions are added to the cluster. The same procedure is run on the newly
   * added points. Once this process converges it is run again on the remaining
   * points in the cloud (i.e. points that were not yet assigned to any cluster)
   * .
   */
  template<typename PointT>
  class RegionGrowingNormalVariation : public alg::RegionGrowing<PointT>
  {   
    protected:
      using alg::RegionGrowing<PointT>::setBinaryConditionFunction;
      using alg::RegionGrowing<PointT>::input_;
      using alg::RegionGrowing<PointT>::indices_;
      
    public:
      /** \brief Constructor */
      RegionGrowingNormalVariation ();
      
      /** \brief Constructor */
      ~RegionGrowingNormalVariation ();
      
      /** \brief Set normal variation threshold (in radians per meter). */
      inline void
      setNormalVariationThreshold (const float threshold);

      /** \brief Set normal variation threshold (in radians per meter). */
      inline float
      getNormalVariationThreshold () const;

      /** \brief Set if input cloud has consistently oriented normals. */
      inline void
      setConsistentNormals (const bool normals_consistently_oriented_);
      
      /** \brief Check if input cloud is assumed to have consistently oriented normals. */
      inline bool
      getConsistentNormals () const;
      
    private:

      /** \brief Update the binary condition function to make sure it corresponds to the value of consistent normals flag. */
      void updateBinaryConditionFunction ();
      
      /** \brief Reorder indices based in ascending order of curvature. */
      void reorderIndicesCurvature ();
      
      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
       * the current settings. If it is possible then it returns true.
       */
      virtual bool
      prepareForSegmentation ();
      
      /** \brief Normal variation binary condition function for consistently oriented normals. */
      bool binaryConditionNonConsistent (const PointT& p1, const PointT& p2, float dist_squared) const;
            
      /** \brief Normal variation binary condition function for non-consistently oriented normals. */
      bool binaryConditionConsistent    (const PointT& p1, const PointT& p2, float dist_squared) const;

  private:
      
      // Normal variation threhsold
      float normal_variation_threshold_;
      
      // Flag indicating whether input cloud has consistently oriented normals
      bool normals_consistently_oriented_;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  // REGION_GROWING_NORMAL_VARIATION_H