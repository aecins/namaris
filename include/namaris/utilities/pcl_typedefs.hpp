#ifndef PCL_TYPEDEFS_HPP
#define PCL_TYPEDEFS_HPP

// PCL includes
#include <pcl/point_types.h>

// Point type definitions
// Naming convention:
//  T:    XYZ
//  N:    normal
//  C:    RGB
//  L:    label
typedef pcl::PointXYZ Point;
typedef pcl::Normal Normal;
typedef pcl::PointNormal PointN;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZRGBA PointCA;
typedef pcl::PointXYZRGBNormal PointNC;
typedef pcl::PointXYZL PointL;

// bool operator== (const PointN &lhs, const PointN &rhs)
// {
//   return (  lhs.x == rhs.x  && 
//             lhs.y == rhs.y  &&
//             lhs.z == rhs.z  &&
//             lhs.normal_x == rhs.normal_x  &&
//             lhs.normal_y == rhs.normal_y  &&
//             lhs.normal_z == rhs.normal_z
//           );
// }
// 
// bool operator!= (const PointN &lhs, const PointN &rhs)
// {
//   return !(lhs == rhs);
// }

#endif  // PCL_TYPEDEFS_HPP