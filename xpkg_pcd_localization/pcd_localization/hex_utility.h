/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef PCD_LOCALIZATION_HEX_UTILITY_H_
#define PCD_LOCALIZATION_HEX_UTILITY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hex {
namespace utility {

typedef struct {
  double time;
  Eigen::Vector3f translation;
  Eigen::Quaternionf orientation;
} HexTransStamped;

typedef struct {
  double time;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
} HexPointsStamped;

}  // namespace utility
}  // namespace hex

#endif  // PCD_LOCALIZATION_HEX_UTILITY_H_
