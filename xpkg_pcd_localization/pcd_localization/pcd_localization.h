/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef PCD_LOCALIZATION_PCD_LOCALIZATION_H_
#define PCD_LOCALIZATION_PCD_LOCALIZATION_H_

#include <pcd_localization/hex_utility.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

using hex::utility::HexPointsStamped;
using hex::utility::HexTransStamped;

namespace hex {
namespace localization {

enum class SystemState { kMove = 0, kStart, kFinish, kCharge };

class PcdLocalization {
 public:
  static PcdLocalization& GetPcdLocalization() {
    static PcdLocalization singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  PcdLocalization() = default;
  virtual ~PcdLocalization() = default;

  // Work Handle
  void UpdateTarget();
  const HexTransStamped& PointsAlignment(const Eigen::Affine3f&,
                                         const HexPointsStamped&);
  void ResultProcess(const HexTransStamped&, const Eigen::Affine3f&);

  // Help Handle
  const HexTransStamped& NdtAlignment(
      const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      double);
  // const HexTransStamped& DoubleIcpAlignment(
  //     const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
  //     const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr DownSample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsBox(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      const Eigen::Vector3f&);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsCylinder(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      double);
  Eigen::Affine3f HexTransToAffine(const HexTransStamped&);
  HexTransStamped AffineToHexTrans(const Eigen::Affine3f&, double);

  // Parameter
  std::string kmap_frame_;
  std::string kodom_frame_;
  double ktarget_voxel_size_;
  double ktarget_pcd_size_;
  double ktarget_update_distance_;
  double ksource_voxel_size_;
  double ksource_max_range_;
  double ksource_min_range_;
  double ksource_max_angle_;
  double ksource_min_angle_;
  double kndt_trans_epsilon_;
  double kndt_step_size_;
  double kndt_resolution_;
  int32_t kndt_max_iterations_;
  bool kmode_pure_lidar_;

  // Variable
  bool start_flag_;
  Eigen::Vector3f map_center_;
  Eigen::Affine3f last_trans_;
  Eigen::Affine3f delta_trans_;
  Eigen::Affine3f map_to_sensor_;
  Eigen::Affine3f map_to_odom_;
  Eigen::Affine3f rough_map_to_sensor_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_register_;
};

}  // namespace localization
}  // namespace hex

#endif  // PCD_LOCALIZATION_PCD_LOCALIZATION_H_
