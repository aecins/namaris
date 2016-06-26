#ifndef MEANSHIFT_H
#define MEANSHIFT_H

// Meanshift
#include <namaris/3rd_party/meanshift/MeanShift.h>
#include <namaris/3rd_party/meanshift/VectorPointSet.h>
#include <namaris/3rd_party/meanshift/EuclideanGeometry.h>


// #include "meanshiftlib/include/MeanShift.h"
// #include "meanshiftlib/include/VectorPointSet.h"
// #include "geometry/include/EuclideanGeometry.h"

// Eigen
#include <Eigen/Dense>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace alg
{
  /** \brief Class for clustering points in N-dimensional Euclidean spaces using
   * meanshift approach.
   * 
   * Points are assumed to be samples of an unknown probability density
   * distribution. Clustering is done by recovering the modes of the
   * distribution. The input to the algorithm is a set of N-dimensional points.
   * The output is a set of clusters (or modes) with their centeroid information
   * and a list of the original points belonging to the cluster.
   * NOTE: this is essentially a wrapper around a modified version of this code:
   * http://coewww.rutgers.edu/riul/research/code/manifoldMS/index.html
   */
  class Meanshift
  {
  public:
    
    /** \brief Types of kernels used for probability density estimation. */
    enum KERNEL_TYPE {GAUSSIAN, EPANECHNIKOV};
    
    /** \brief Empty constructor. */
    Meanshift ();
            
    /** \brief Set the bandwidth of the kernel used for density estimation.
     * Larger values make each point influence a larger area around it.
     *  \param[in] bandwidth   bandwidth of the kernel
     */
    void setBandwidth (float bandwidth);
    
    /** \brief Type of Kernel used for density estimation
     *  \param[in] bandwidth   bandwidth of the kernel
     */          
    void setKernelType      (KERNEL_TYPE kernelType);
    
    /** \brief Set the maximum allowed distance between a mode center and it's
     * inlier.
     *  \param[in]  maxModeInlierDistance   maximum distance between mode's
     * center and it's inlier
     */              
    void setMaxModeInlierDistance (float maxModeInlierDistance);
    
    /** \brief Set the input data points of the algorithm.
     *  \param[in]  points    a matrix containing the data points (columns are
     * points, rows are dimensions)
     */              
    bool setInputPoints (const Eigen::MatrixXf &points);

    /** \brief Get the bandwidth of the kernel. */
    float getBandwidth () const;
    
    /** \brief Get the type of kernel used. */
    float getKernelType ()  const;
    
    /** \brief Get the maximum distance between mode center and its inlier. */
    float getMaxModeInlierDistance () const;
    
    /** \brief Run meanshift procedure on the input points.
     *  \param[out] unprunedModes   a matrix representing the converged points.
     */
    bool cluster (Eigen::MatrixXf &unprunedModes);
    
    /** \brief Find dominant modes by prunning local modes that are too close to other modes
     *  \param[out] unprunedModes   a matrix representing the converged points.
     */    
    bool pruneModes (Eigen::MatrixXf &prunedModes, std::vector<std::vector<int> > &prunedModeSupport);    

  private:
    
    // Parameters
    float bandwidth_;
    KERNEL_TYPE kernelType_;
    float maxModeInlierDistance_;
    
    // Data and results
    int dimensionality_;
    int numPoints_;
    // NOTE: the reason why these are shared pointers is that if the underlying
    // class CVectorPointSet is can not be initialized twice (in constructor and
    // then during processing). It segfaults when class is destrcuted. One
    // option is to use regular pointers but then you have to keep track of
    // which pointers were allocated and bla-bla-bla so just use shared pointers
    // instead.
    boost::shared_ptr<CVectorPointSet<float> > points_;
    boost::shared_ptr<CVectorPointSet<float> > unprunedModes_;
    boost::shared_ptr<CVectorPointSet<float> > prunedModes_;
    std::vector<std::vector<int> > prunedModeSupport_;
    std::vector<double> probs_;
    
    // Meanshift object
    boost::shared_ptr<CEuclideanGeometry<float> > geom_;
    CMeanShift<float> ms_;    
  };
}

#endif // MEANSHIFT_H
