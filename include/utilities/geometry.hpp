#ifndef GEOMETRY_UTILITIES_HPP
#define GEOMETRY_UTILITIES_HPP

// STD includes
#include <iostream>

// CPP tools utilities
#include <utilities/math.hpp>

namespace utl
{
  namespace geom
  {
    /** \brief Extract plane unit normal and point on the plane closest to the origin
     * from coefficients of an equation of a plane (ax + by + cz + d = 0).
     *  \param[in]  plane_coefficients plane coefficients (ax + by + cz + d = 0)
     *  \param[out] plane_point plane point closest to the origin
     *  \param[out] plane_normal plane unit normal
     */
    template <class Scalar>
    inline
    void planeCoefficientsToPointNormal (const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients, Eigen::Matrix< Scalar, 3, 1> &plane_point, Eigen::Matrix< Scalar, 3, 1> &plane_normal)
    {
      plane_normal << plane_coefficients[0], plane_coefficients[1], plane_coefficients[2];
      Scalar norm = plane_normal.norm();
      plane_normal /= norm;
      plane_point  = plane_normal * ( - plane_coefficients[3] / norm );
    }

    /** \brief Given a point an a normal defining a plane extract coefficients 
     * of the equation of a plane (ax + by + cz + d = 0).
     *  \param[in]  plane_point plane point closest to the origin
     *  \param[in]  plane_normal plane unit normal
     *  \param[out] plane_coefficients plane coefficients (ax + by + cz + d = 0)
     */
    template <class Scalar>
    inline void
    pointNormalToPlaneCoefficients (const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal, Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
    {
      plane_coefficients.head(3) = plane_normal;
      plane_coefficients(3) = - plane_normal.dot(plane_point);
    }

    /** \brief Get the Euclidean distance between two N-dimensional points.
     *  \param[in] point1 first point
     *  \param[in] point2 second point
     *  \return distance between points
     */
    template <class Scalar>
    inline Scalar
    pointToPointDistance (const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> &point1, const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> &point2)
    {
      if (point1.size() != point2.size())
      {
        std::cout << "[utl::geom::pointToPointDistance] points have different dimensions. Returning NaN." << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
      }
      return (point1-point2).norm();
    }
    
    /** \brief Get distance between a point and a line.
     *  \param[in] point point
     *  \param[in] line_point1  first point of a line
     *  \param[in] line_point2  second point of a line
     *  \return distance between point and line
     */
    template <class Scalar>
    inline Scalar
    pointToLineDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2)
    {
      return ((point - line_point1).cross(point - line_point2)).norm() / (line_point2 - line_point1).norm();
    }
    
    /** \brief Get distance between a point and a plane.
     *  \param[in] point point
     *  \param[in] plane_point  a point on the plane
     *  \param[in] plane_normal plane normal
     *  \return distance between point and plane
     *  \note distance is signed and depends on the plane normal
     */
    template <class Scalar>
    inline Scalar
    pointToPlaneDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal)
    {
      Eigen::Matrix< Scalar, 3, 1> planeToPointVector = point - plane_point;
      return planeToPointVector.dot(plane_normal);
    }
    
    /** \brief Get distance between a point and a plane.
     *  \param[in] point point
     *  \param[in] plane_coefficients coefficients of the equation of the plane
     *  \return distance between point and plane
     *  \note distance is signed and depends on the plane normal
     */
    template <class Scalar>
    inline
    Scalar pointToPlaneDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
    {
      Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
      planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);
      return pointToPlaneDistance<Scalar>(point, plane_point, plane_normal);
    }

    /** \brief Get distance between two skew lines.
     *  \param[in] line1_point1 first point of the line 1
     *  \param[in] line1_point2 second point of the line 1
     *  \param[in] line2_point1 first point of the line 2
     *  \param[in] line2_point2 second point of the line 2
     *  \param[in] eps  tolerance for parallel lines check
     *  \return distance between two lines
     *  \note http://mathworld.wolfram.com/Line-LineDistance.html
     */
    template <class Scalar>
    inline
    Scalar lineToLineDistance ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                                const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                                const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                                const Eigen::Matrix< Scalar, 3, 1> &line2_point2,
                                const Scalar eps = 1e-12
                              )
    {
      // Get line direction vectors
      Eigen::Vector3f a = line1_point2 - line1_point1;
      Eigen::Vector3f b = line2_point2 - line2_point1;
      
      // If lines are parallel return the distance between point and line
      Scalar denom = a.cross(b).norm();
      if (denom < eps)
        return pointToLineDistance<Scalar>(line2_point1, line1_point1, line1_point2);
      
      Eigen::Vector3f c = line2_point1 - line1_point1;
      return std::abs(c.dot(a.cross(b))) / denom;
    }

    // NOTE Why would you have a special case for lines that go through origin?
    /** \brief Get distance between two lines. Both lines are assumed to go 
     * through the origin.
     *  \param[in] line1_direction line 1 direction vector (can be non-unit)
     *  \param[in] line2_direction line 2 direction vector (can be non-unit)
     *  \param[in] eps  tolerance for parallel lines check
     *  \return distance between two lines
     *  \note http://mathworld.wolfram.com/Line-LineDistance.html
     */
    template <class Scalar>
    inline
    Scalar lineToLineDistance ( const Eigen::Matrix< Scalar, 3, 1> &line1_direction,
                                const Eigen::Matrix< Scalar, 3, 1> &line2_direction,
                                const Scalar eps = 1e-12
                              )
    {
      // Get line direction vectors
      Eigen::Vector3f a = line1_direction;
      Eigen::Vector3f b = line2_direction;
      
      // If lines are parallel return the 
      Scalar denom = a.cross(b).norm();
      if (denom < eps)
        return static_cast<Scalar>(0.0f);
      
      Eigen::Vector3f c = line2_direction - line1_direction;
      return std::abs(c.dot(a.cross(b))) / denom;
    }

    /** \brief Get angle between two skew lines.
     *  \param[in] line1_point1 first point of the line 1
     *  \param[in] line1_point2 second point of the line 1
     *  \param[in] line2_point1 first point of the line 2
     *  \param[in] line2_point2 second point of the line 2
     *  \param[in] eps  tolerance for parallel lines check
     *  \return distance between two lines
     *  \note http://mathworld.wolfram.com/Line-LineDistance.html
     */
    template <class Scalar>
    inline
    Scalar lineLineAngle  ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point2,
                            const Scalar eps = 1e-12
                          )
    {
      // Get line direction vectors
      Eigen::Vector3f a = line1_point2 - line1_point1;
      Eigen::Vector3f b = line2_point2 - line2_point1;
      
      return std::acos(a.dot(b) / (a.norm() * b.norm() ));
    }
    
    /** \brief Get angle between a line and a plane.
     *  \param[in] line_direction direction vector of the line (can be non-unit)
     *  \param[in] plane_normal   unit normal of the plane
     */
    template <class Scalar>
    inline
    Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line_direction,
                            const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                          )
    {
      return std::asin (std::abs (line_direction.dot (plane_normal) / line_direction.norm ()));
    }    
    
    /** \brief Get angle between a line and a plane.
     *  \param[in] line_direction direction unit vector of the line
     *  \param[in] plane_normal   unit normal of the plane
     */
    template <class Scalar>
    inline
    Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line1_point,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point,
                            const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                          )
    {
      return linePlaneAngle<Scalar>(line1_point - line2_point, plane_normal);
    }
    
    /** \brief Project point on a line.
     *  \param[in] point point to be projected
     *  \param[in] line_point1  first points of a line
     *  \param[in] line_point2  second point of a line
     *  \return point projected onto a line
     */
    template <class Scalar>
    inline Eigen::Matrix< Scalar, 3, 1>
    projectPointToLine (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2)
    {
      Eigen::Matrix< Scalar, 3, 1> line_vector = line_point2 - line_point1;
      return line_point1 + (point - line_point1).dot(line_vector) * line_vector / line_vector.dot(line_vector);
    }    
    
    /** \brief Project point on a plane.
     *  \param[in] point point to be projected
     *  \param[in] plane_point  a point on the plane
     *  \param[in] plane_normal plane normal
     *  \return point projected onto a plane
     */
    template <class Scalar>
    inline Eigen::Matrix< Scalar, 3, 1>
    projectPointToPlane (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal)
    {
      return point - plane_normal * pointToPlaneDistance<Scalar>(point, plane_point, plane_normal);
    }
    
    /** \brief Project point on a plane.
     *  \param[in] point point to be projected
     *  \param[in] plane_coefficients coefficients of the equation of the plane
     *  \return point projected onto a plane
     */
    template <class Scalar>
    inline Eigen::Matrix< Scalar, 3, 1>
    projectPointToPlane (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
    {
      Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
      planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);
      return projectPointToPlane<Scalar>(point, plane_point, plane_normal);
    }
        
    /** \brief Compute an intersection point between a line and a plane.
     *  \param[in] line_point1 first point of the line
     *  \param[in] line_point2 second point of the line
     *  \param[in] plane plane coefficients (ax + by + cz + d = 0)
     *  \return point where line and plane intersect
     */
    template <class Scalar>
    inline Eigen::Matrix< Scalar, 3, 1>
    linePlaneIntersection ( const Eigen::Matrix< Scalar, 3, 1> &line_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line_point2,
                            const Eigen::Matrix< Scalar, 3, 1> &plane_point,
                            const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                          )
    {
      Eigen::Matrix< Scalar, 3, 1> line_direction = line_point2 - line_point1;
      Scalar d = plane_normal.dot(plane_point - line_point1) / line_direction.dot(plane_normal);
      return line_point1 + d * line_direction;
    }
        
    /** \brief Compute an intersection point between a line anf a plane
     *  \param[in] line_point1 first point of the line
     *  \param[in] line_point2 second point of the line
     *  \param[in] plane_coefficients plane coefficients (ax + by + cz + d = 0)
     *  \return point where line and plane intersect
     */
    template <class Scalar>
    inline Eigen::Matrix< Scalar, 3, 1>
    linePlaneIntersection(const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
    {
      Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
      planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);      
      return linePlaneIntersection(line_point1, line_point2, plane_point, plane_normal);
    }

    /** \brief Find the shortest line segment connecting two lines in 3d space.
     *  \param[in] line1_point1 first point of the line 1
     *  \param[in] line1_point2 second point of the line 1
     *  \param[in] line2_point1 first point of the line 2
     *  \param[in] line2_point2 second point of the line 2
     *  \param[out] seg_point1 first point of the shortest line segment
     *  \param[out] seg_point2 second points of the shortest line segment
     *  \param[in] eps maximum value of the determinant for the lines to be considered parallel
     *  \return true if lines are parallel
     *  \note http://geomalgorithms.com/a07-_distance.html
     */
    template <class Scalar>
    inline bool
    lineLineIntersection  ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point2,
                            Eigen::Matrix< Scalar, 3, 1> &seg_point1,
                            Eigen::Matrix< Scalar, 3, 1> &seg_point2,
                            Scalar eps = 1e-3
                          )
    {
      // Get direction vectors for both lines
      Eigen::Matrix< Scalar, 3, 1> l1 = line1_point2 - line1_point1;
      Eigen::Matrix< Scalar, 3, 1> l2 = line2_point2 - line2_point1;
      
      // Get temporary variables
      Scalar a = l1.dot(l1);
      Scalar b = l1.dot(l2);
      Scalar c = l2.dot(l2);
      Scalar d = l1.dot(line1_point1 - line2_point1);
      Scalar e = l2.dot(line1_point1 - line2_point1);
      
      // Calculate determinant and check that it is not too small
      Scalar D = a*c - b*b;
      
      if (D < eps)
        return true;
      
      // Calculate parameters
      Scalar alpha = (b*e - c*d) / D;
      Scalar beta  = (a*e - b*d) / D;
      
      // Get line segment points
      seg_point1 = line1_point1 + l1*alpha;
      seg_point2 = line2_point1 + l2*beta;

      return false;
    }    
    
    /** \brief Find a clockwise angle between two 3D vectors
     *  \param[in] v1 first vector
     *  \param[in] v2 second vector
     *  \param[in] normal plane normal
     *  \return clockwise angle between two vectors
     *  \note assumes right handed coordinate system
     */
    template <typename Scalar>
    inline Scalar
    vectorAngleCW (const Eigen::Matrix< Scalar, 3, 1> &v1, const Eigen::Matrix< Scalar, 3, 1> &v2, const Eigen::Matrix< Scalar, 3, 1> &normal)
    {
      Scalar cos = v1.dot(v2);
      Scalar sin = math::clampValue<Scalar>(normal.dot(v1.cross(v2)), -1.0, 1.0);
      return std::atan2(sin, cos);
    }
        
    /** \brief Get counter clockwise difference between two angles (in radians)
      * \param[in] angle_start    start angle
      * \param[in] angle_end      end angle 
      * \return angular distance from start angle to end angle
      */
    template <typename Scalar>
    inline Scalar
    angleDifferenceCCW (const Scalar start_angle, const Scalar end_angle)
    {
      return math::remainder(end_angle - start_angle, static_cast<Scalar>(2 * M_PI));
    }
    
    /** \brief Find a rotation matrix that aligns two vectors. Note that transformation matrix is not unique.
     *  \param target_normal target normal
     *  \param source_normal source normal
     *  \return 3x3 rotation matrix
     *  \note: http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
     */
    template <typename Scalar>
    inline
    Eigen::Matrix<Scalar,3,3> alignVectors(const Eigen::Matrix<Scalar,1,3> source_vector, const Eigen::Matrix<Scalar,1,3> target_vector)
    {
      Eigen::Matrix<Scalar,1,3> source_normal = source_vector / source_vector.norm();
      Eigen::Matrix<Scalar,1,3> target_normal = target_vector / target_vector.norm();
      
      if (source_normal == target_normal)
        return Eigen::Matrix<Scalar,3,3>::Identity();
      
      Eigen::Matrix<Scalar,1,3> k = -target_normal.cross(source_normal);           // Unit vector representing the axis of rotation between source and target
      Scalar sinTheta = k.norm();                                                 // Rotation angle sine
      Scalar cosTheta = target_normal.dot(source_normal);                         // Rotation angle cosince
      
      Eigen::Matrix<Scalar,3,3> K;
      K <<    0 , -k(2),  k(1),
            k(2),     0, -k(0),
          -k(1),  k(0),     0;
          
      Eigen::Matrix<Scalar,3,3> R;
      R = Eigen::Matrix<Scalar,3,3>::Identity() + K + (1 - cosTheta) * K * K / sinTheta / sinTheta;
      
      return R;    
    }
    
    /** \brief Linear spherical interpolation between two rotation matrices
     *  \param[in]  r_src source rotation source
     *  \param[in]  r_tgt target rotation matrix
     *  \param[in]  alpha interpolation factor (0 returns r_src and 1 returns r_tgt)
     *  \return interploated rotation
     */
    template <class Scalar>
    inline
    Eigen::Matrix< Scalar, 3, 3> interpolateRotation  ( const Eigen::Matrix< Scalar, 3, 3> &r_src,
                                                        const Eigen::Matrix< Scalar, 3, 3> &r_tgt,
                                                        const Scalar alpha
                                                      )
    {
      return Eigen::Quaternionf(r_src).slerp(alpha, Eigen::Quaternionf(r_tgt)).toRotationMatrix();
    }

    /** \brief Linear interploation between two 3D rigid transformations.
     *  \param[in]  t_src source transformation
     *  \param[in]  t_tgt target transformation
     *  \param[in]  alpha interpolation factor (0 returns t_src and 1 returns t_tgt)
     *  \return interploated transformation
     */
    template <class Scalar>
    inline
    Eigen::Transform<Scalar, 3, Eigen::Affine> interpolateTransformations ( const Eigen::Transform<Scalar, 3, Eigen::Affine> &t_src,
                                                                            const Eigen::Transform<Scalar, 3, Eigen::Affine> &t_tgt,
                                                                            const Scalar alpha
                                                                          )
    {
      Eigen::Transform<Scalar, 3, Eigen::Affine> t_itrp;
      
      // Interploate translation
      t_itrp.translation() = t_src.translation() + (t_tgt.translation()-t_src.translation()) * alpha;
      
      // Interploate rotation
      t_itrp.linear() = interpolateRotation<Scalar>(t_src.linear(), t_tgt.linear(), alpha);
      
      return t_itrp;
    }    
  }
}

#endif    // GEOMETRY_UTILITIES_HPP