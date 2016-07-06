#ifndef REGION_GROWING_SMOOTHNESS_HPP
#define REGION_GROWING_SMOOTHNESS_HPP

#include "region_growing_smoothness.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
alg::RegionGrowingSmoothness<PointT>::RegionGrowingSmoothness () :
  normal_angle_threshold_ (0.0f),
  normals_consistently_oriented_ (false)
{
  updateBinaryConditionFunction();  
}
      
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
alg::RegionGrowingSmoothness<PointT>::~RegionGrowingSmoothness ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
alg::RegionGrowingSmoothness<PointT>::setNormalAngleThreshold (const float threshold)
{
  normal_angle_threshold_ = threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
alg::RegionGrowingSmoothness<PointT>::getNormalAngleThreshold () const
{
  return normal_angle_threshold_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
alg::RegionGrowingSmoothness<PointT>::setConsistentNormals (const bool normals_consistently_oriented)
{
  normals_consistently_oriented_ = normals_consistently_oriented;
  updateBinaryConditionFunction();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline bool
alg::RegionGrowingSmoothness<PointT>::getConsistentNormals () const
{
  return normals_consistently_oriented_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
alg::RegionGrowingSmoothness<PointT>::updateBinaryConditionFunction ()
{
  if (normals_consistently_oriented_)
    setBinaryConditionFunction( boost::bind(&RegionGrowingSmoothness<PointT>::binaryConditionConsistent, this, _1, _2, _3) );
  else
    setBinaryConditionFunction( boost::bind(&RegionGrowingSmoothness<PointT>::binaryConditionNonConsistent, this, _1, _2, _3) );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
alg::RegionGrowingSmoothness<PointT>::reorderIndicesCurvature ()
{
  if (!indices_ || indices_->empty())
  {
    indices_->resize(input_->size());
    for (size_t pointId = 0; pointId < indices_->size(); pointId++)
      (*indices_)[pointId] = pointId;
  }
  
  // Extract curvature values into a vector
  std::vector<float> curvature (indices_->size());
  for (size_t pointIdIt = 0; pointIdIt < indices_->size(); pointIdIt++)
    curvature[pointIdIt] = input_->points[(*indices_)[pointIdIt]].curvature;
    
  // Reorder indices in increasing order of curvature
  std::vector<float> curvature_sorted;
  std::vector<size_t> order;
  utl::stdvec::sort(curvature, curvature_sorted, order, utl::stdvec::ASCENDING);
  *indices_ = utl::stdvec::reorder(*indices_, order);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
alg::RegionGrowingSmoothness<PointT>::binaryConditionConsistent (const PointT& p1, const PointT& p2, float dist_squared) const
{
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  Eigen::Vector3f n2 = p2.getNormalVector3fMap();
    
  float angle = std::acos (utl::math::clampValue (n1.dot (n2), 0.0f, 1.0f));
  
  return angle < normal_angle_threshold_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
alg::RegionGrowingSmoothness<PointT>::binaryConditionNonConsistent (const PointT& p1, const PointT& p2, float dist_squared) const
{
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  Eigen::Vector3f n2 = p2.getNormalVector3fMap();
    
  float angle = std::acos (utl::math::clampValue (std::abs (n1.dot (n2)), 0.0f, 1.0f));
  return angle < normal_angle_threshold_;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline bool
alg::RegionGrowingSmoothness<PointT>::prepareForSegmentation ()
{  
  // If user forgot to pass point cloud or if it is empty
  if ( normal_angle_threshold_ < 0.0f )
  {
    std::cout << "[alg::RegionGrowingSmoothness::prepareForSegmentation] normal variation threhsold must be non-negative!" << std::endl;
    return (false);
  }
  
  bool good = alg::RegionGrowing<PointT>::prepareForSegmentation();
  
  if (good)
  {
    this->reorderIndicesCurvature();
    return true;
  }
  else
  {
    return false;
  }
}

#endif  // REGION_GROWING_SMOOTHNESS_HPP