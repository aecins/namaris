#ifndef POINTCLOUD_ALIGNMENT_HPP
#define POINTCLOUD_ALIGNMENT_HPP

// PCL includes
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

namespace alg
{
  /** \brief Align two clouds the ICP algorithm.
   *  Disstance metric:         point-to-plane
   *  Optimization method:      Levenberg-Marquardt
   *  Available correspondence rejectors:
   *    - max correspondence distance
   *    - max correspondence surface normal angle
   *    - one-to-one
   *  \param[in]  cloud_src           cloud that needs to be aligned
   *  \param[in]  cloud_tgt           cload that source will be aligned to
   *  \param[out] cloud_src_aligned   aligned source cloud
   *  \param[out] transform           transformation that takes source cloud points to aligned cloud points
   *  \param[in]  max_correspondence_distance             max correspondance distance threshold
   *  \param[in]  max_correspondece_surface_normal_angle  max correspondence normal angle threshold
   *  \param[in]  use_one_to_one      flag specifying if one-to-one rejection is used
   *  \param[in]  max_iterations      maximum number of ICP internal iterations
   *  \return distance between two clouds
   */
  template <typename PointT>
  inline
  bool alignClouds  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud_src,
                      const typename pcl::PointCloud<PointT>::ConstPtr &cloud_tgt,
                      typename pcl::PointCloud<PointT>::Ptr &cloud_src_aligned,
                      Eigen::Affine3f &transform,
                      pcl::CorrespondencesPtr &correspondences,
                      const float max_correspondence_distance = 0.0f,
                      const float max_correspondece_surface_normal_angle = 0.0f,
                      const bool use_one_to_one = true,
                      const int max_iterations = 100
                    )
  {
    //--------------------------------------------------------------------------
    // Typedefs
    //--------------------------------------------------------------------------
    
    typedef pcl::registration::CorrespondenceEstimation<PointT, PointT, float>                ClosestPoint;
    typedef pcl::registration::TransformationEstimationPointToPlane<PointT, PointT, float>    PointToPlaneLM;    // Point to plane using Levenberg-Marquardt
    typedef pcl::registration::CorrespondenceRejectorDistance                                 DistanceRejector;
    typedef pcl::registration::CorrespondenceRejectorOneToOne                                 OneToOneRejector;
    typedef pcl::registration::CorrespondenceRejectorSurfaceNormal                            SurfaceNormalRejector;
    
    //----------------------------------------------------------------------------
    // Prepare variables
    //----------------------------------------------------------------------------
    
    cloud_src_aligned.reset(new pcl::PointCloud<PointT>);
    
    //--------------------------------------------------------------------------
    // Run ICP
    //--------------------------------------------------------------------------
    
    pcl::IterativeClosestPoint<PointT, PointT, float> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    
    // Set correspondence estimation method
    boost::shared_ptr<ClosestPoint> correspondece_estimation (new ClosestPoint);
    icp.setCorrespondenceEstimation(correspondece_estimation);

    // Set error metric and minimization
    boost::shared_ptr<PointToPlaneLM> transformation_estimation (new PointToPlaneLM);
    icp.setTransformationEstimation (transformation_estimation);
    
    // Add max correspondence distance rejector
    if (max_correspondence_distance > 0.0f)
    {
      boost::shared_ptr<DistanceRejector> distance_rejector (new DistanceRejector);
      distance_rejector->setMaximumDistance(max_correspondence_distance);
      icp.addCorrespondenceRejector(distance_rejector);
    }

    // Add surface normal angle rejector
    if (max_correspondece_surface_normal_angle > 0.0f)
    {
      boost::shared_ptr<SurfaceNormalRejector> surface_normal_rejector (new SurfaceNormalRejector);
      surface_normal_rejector->setThreshold(std::cos(max_correspondece_surface_normal_angle));
      icp.addCorrespondenceRejector(surface_normal_rejector);
    }

    // Add ono-to-one rejector
    if (use_one_to_one)
    {
      boost::shared_ptr<OneToOneRejector> one_to_one_rejector (new OneToOneRejector);
      icp.addCorrespondenceRejector(one_to_one_rejector);
    }
    
    // Set convergence criteria
    icp.setMaximumIterations(max_iterations);
    icp.setTransformationEpsilon(1e-100);
    icp.setEuclideanFitnessEpsilon(1e-100);
    
    // Run ICP
    icp.align (*cloud_src_aligned);
    transform.matrix() = icp.getFinalTransformation();
    correspondences = icp.correspondences_;
    
    // Check if ICP has converged
    if (!icp.hasConverged())
      return false;
    
    return true;
  }
}

#endif  // POINTCLOUD_ALIGNMENT_HPP