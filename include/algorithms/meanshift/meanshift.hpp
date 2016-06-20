#ifndef MEANSHIFT_HPP
#define MEANSHIFT_HPP

#include "meanshift.h"

/** \brief Custom comparator for sorting a vector of doubles and preserving
  * sort order. */
bool pairCompare(const std::pair<double, int>& firstPair, const std::pair<double, int>& secondPair)
{
  return firstPair.first > secondPair.first;
}

////////////////////////////////////////////////////////////////////////////////
alg::Meanshift::Meanshift()
  : bandwidth_ (0.02)
  , kernelType_ (GAUSSIAN)
  , maxModeInlierDistance_ (0.02)
  , dimensionality_ (0)
  , numPoints_ (0)
{ }

////////////////////////////////////////////////////////////////////////////////
void alg::Meanshift::setBandwidth(float bandwidth)
{
  bandwidth_ = bandwidth;
}

////////////////////////////////////////////////////////////////////////////////
float alg::Meanshift::getBandwidth() const
{
  return bandwidth_;
}

////////////////////////////////////////////////////////////////////////////////
void alg::Meanshift::setKernelType(alg::Meanshift::KERNEL_TYPE kernelType)
{
  kernelType_ = kernelType;
}

////////////////////////////////////////////////////////////////////////////////
float alg::Meanshift::getKernelType() const
{
  return kernelType_;
}

////////////////////////////////////////////////////////////////////////////////
void alg::Meanshift::setMaxModeInlierDistance(float maxModeInlierDistance)
{
  maxModeInlierDistance_ = maxModeInlierDistance;
}

////////////////////////////////////////////////////////////////////////////////
float alg::Meanshift::getMaxModeInlierDistance() const
{
  return maxModeInlierDistance_;
}
 
////////////////////////////////////////////////////////////////////////////////
bool alg::Meanshift::setInputPoints(const Eigen::MatrixXf& points)
{
  dimensionality_ = points.rows();
  numPoints_      = points.cols();

  if (dimensionality_ == 0 || numPoints_ == 0)
  {
    std::cout << "[utl::Meanshift::cluster] input data was not set!" << std::endl;
    return false;
  }

  points_ = boost::shared_ptr<CVectorPointSet<float> > (new CVectorPointSet<float> (dimensionality_, numPoints_, points.data()));
  geom_ = boost::make_shared<CEuclideanGeometry<float> >(dimensionality_);                                      // Define the type of geometry used (with dimensionality)
  return true;
}
  
////////////////////////////////////////////////////////////////////////////////
bool alg::Meanshift::cluster(Eigen::MatrixXf &unprunedModes)
{
  //----------------------------------------------------------------------------
  // Check that data was set
  if (dimensionality_ == 0 || numPoints_ == 0)
  {
    std::cout << "[utl::Meanshift::cluster] input data was not set!" << std::endl;
    return false;
  }
  
  //----------------------------------------------------------------------------
  // Setup meanshift object    
  if (kernelType_ == GAUSSIAN)
    ms_ = CMeanShift< float >::EGaussianKernel;
  else if (kernelType_ == EPANECHNIKOV)
    ms_ = CMeanShift< float >::EEpanechnikovKernel;
  else
  {
    std::cout << "[utl::Meanshift::cluster] unknown kernel type '" << kernelType_ << "'. This is a bug." << std::endl;
    return false;
  }
  
  ms_.setBandwidth(bandwidth_);

  //----------------------------------------------------------------------------
  // Prepare arrays
  unprunedModes_ = boost::shared_ptr<CVectorPointSet<float> > (new CVectorPointSet<float> (dimensionality_, numPoints_));
  
  //----------------------------------------------------------------------------
  // Cluster time!
  ms_.doMeanShift(*geom_, *points_, *unprunedModes_, probs_);                     // Perform meanshift. Result is a list of peaks where particles have convererged to
  
  //----------------------------------------------------------------------------
  // Convert to Eigen
  unprunedModes = Eigen::MatrixXf (dimensionality_, numPoints_);
  for (size_t pointId = 0; pointId < static_cast<size_t>(numPoints_); pointId++)
    for (size_t dId = 0; dId < static_cast<size_t>(dimensionality_); dId++)
      unprunedModes(dId,pointId) = *((*unprunedModes_)[pointId]+dId);
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool alg::Meanshift::pruneModes(Eigen::MatrixXf &prunedModes, std::vector<std::vector<int> > &prunedModeSupport)
{
  //----------------------------------------------------------------------------
  // Rearange modes in decreasing order of their probabilities
  
  // Get order
  std::vector<std::pair<double, int> > modeOrder (numPoints_);
  for (size_t pointId = 0; pointId < static_cast<size_t>(numPoints_); pointId++)
    modeOrder[pointId] = std::pair<double, int> (probs_[pointId], pointId);
  std::sort(modeOrder.begin(), modeOrder.end(), pairCompare);
      
  // Rearrange
  CVectorPointSet<float> unprunedModes_sorted (dimensionality_, numPoints_);
  for (size_t pointId = 0; pointId < static_cast<size_t>(numPoints_); pointId++)
    unprunedModes_sorted.addPoint((*unprunedModes_)[modeOrder[pointId].second]);        

  //----------------------------------------------------------------------------
  // Prune modes
  prunedModes_ = boost::make_shared<CVectorPointSet<float> >(dimensionality_, numPoints_);
  ms_.pruneModes(*geom_, unprunedModes_sorted, *prunedModes_, prunedModeSupport_, 1, maxModeInlierDistance_);
          
  //----------------------------------------------------------------------------
  // Convert pruned modes to eigen
  prunedModes = Eigen::MatrixXf (dimensionality_, prunedModes_->size());
  for (size_t pointId = 0; pointId < static_cast<size_t>(prunedModes_->size()); pointId++)
    for (size_t dId = 0; dId < static_cast<size_t>(dimensionality_); dId++)
      prunedModes(dId,pointId) = *((*prunedModes_)[pointId]+dId);
  
  //----------------------------------------------------------------------------
  // Rearrange pruned mode support
  for (size_t modeId = 0; modeId < prunedModeSupport_.size(); modeId++)    
    for (size_t pointId = 0; pointId < prunedModeSupport_[modeId].size(); pointId++)
      prunedModeSupport_[modeId][pointId] = modeOrder[prunedModeSupport_[modeId][pointId]].second;
  prunedModeSupport = prunedModeSupport_;
  
  return true;
}

#endif    // MEANSHIFT_HPP