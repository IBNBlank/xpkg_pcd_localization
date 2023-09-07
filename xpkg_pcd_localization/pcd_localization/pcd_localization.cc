/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include "pcd_localization/pcd_localization.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <vector>

#include "pcd_localization/data_interface.h"

namespace hex {
namespace localization {

bool PcdLocalization::Init() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();
  static bool first_init_flag = true;

  if (first_init_flag) {
    first_init_flag = false;

    // parameters
    kmap_frame_ = data_interface.GetMapFrame();
    kodom_frame_ = data_interface.GetOdomFrame();
    ktarget_voxel_size_ = data_interface.GetTargetVoxelSize();
    ktarget_pcd_size_ = data_interface.GetTargetPcdSize();
    ktarget_update_distance_ = data_interface.GetTargetUpdateDistance();
    ksource_voxel_size_ = data_interface.GetSourceVoxelSize();
    ksource_max_range_ = data_interface.GetSourceMaxRange();
    ksource_min_range_ = data_interface.GetSourceMinRange();
    ksource_max_angle_ = data_interface.GetSourceMaxAngle();
    ksource_min_angle_ = data_interface.GetSourceMinAngle();
    kndt_trans_epsilon_ = data_interface.GetNdtTransEpsilon();
    kndt_step_size_ = data_interface.GetNdtStepSize();
    kndt_resolution_ = data_interface.GetNdtResolution();
    kndt_max_iterations_ = data_interface.GetNdtMaxIterations();
    kmode_pure_lidar_ = data_interface.GetModePureLidar();

    // transform
    Eigen::Affine3f init_affine =
        HexTransToAffine(data_interface.GetInitTrans());
    map_center_ = init_affine.translation();
    last_trans_ = init_affine;
    map_to_sensor_ = init_affine;
    delta_trans_ = Eigen::Affine3f::Identity();
    map_to_odom_ = Eigen::Affine3f::Identity();

    // point cloud
    map_points_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(data_interface.GetMapPath().c_str(),
                                            *map_points_) == -1) {
      first_init_flag = true;
      data_interface.Log(LogLevel::kError, "### Load Map Failed ###");
      return false;
    }
    map_points_ = DownSample(map_points_, ktarget_voxel_size_);
    data_interface.PublishMapPoints(map_points_);
    data_interface.Log(LogLevel::kInfo, "### Load Map Finished ###");

    if (data_interface.GetModeAutoStart()) {
      start_flag_ = true;
      UpdateTarget();
    } else {
      start_flag_ = false;
    }
  } else {
    start_flag_ = false;

    data_interface.Log(LogLevel::kInfo, "### Wait For New Init Trans ###");
  }

  return true;
}

bool PcdLocalization::Work() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();
  // tf
  Eigen::Affine3f odom_to_sensor =
      HexTransToAffine(data_interface.ListenFrameToSensor(kodom_frame_));

  // get rough transformation
  if (data_interface.GetInitTransFlag()) {
    data_interface.ResetInitTransFlag();

    map_to_sensor_ = HexTransToAffine(data_interface.GetInitTrans());
    map_center_ = map_to_sensor_.translation();
    UpdateTarget();

    if (kmode_pure_lidar_) {
      last_trans_ = map_to_sensor_;
      delta_trans_ = Eigen::Affine3f::Identity();
    }

    start_flag_ = true;
  } else if (kmode_pure_lidar_) {
    map_to_sensor_ = last_trans_ * delta_trans_;
  } else {
    map_to_sensor_ = map_to_odom_ * odom_to_sensor;
  }

  if (start_flag_) {
    // update map
    if ((map_center_ - map_to_sensor_.translation()).norm() >
        ktarget_update_distance_) {
      // target
      map_center_ = map_to_sensor_.translation();
      UpdateTarget();
    }

    // pcd alignment
    if (data_interface.GetLidarCloudFlag()) {
      HexPointsStamped sensor_points = data_interface.GetLidarCloud();
      HexTransStamped fine_map_to_sensor =
          PointsAlignment(map_to_sensor_, sensor_points);

      if (fabs(fine_map_to_sensor.time - sensor_points.time) > 1.0) {
        data_interface.Log(LogLevel::kError, "### Diverge ###");
        return false;
      }
      ResultProcess(fine_map_to_sensor, odom_to_sensor);

      data_interface.ResetLidarCloudFlag();
    }
  }

  return true;
}

void PcdLocalization::UpdateTarget() {
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;
  ndt_new.setTransformationEpsilon(kndt_trans_epsilon_);
  ndt_new.setStepSize(kndt_step_size_);
  ndt_new.setResolution(kndt_resolution_);
  ndt_new.setMaximumIterations(kndt_max_iterations_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_points =
      CropPointsBox(map_points_, ktarget_pcd_size_, ktarget_pcd_size_,
                    ktarget_pcd_size_, map_center_);
  ndt_new.setInputTarget(target_points);
  // data_interface.PublishDebugPoints(target_points, kmap_frame_);

  ndt_register_ = ndt_new;
}

const HexTransStamped& PcdLocalization::PointsAlignment(
    const Eigen::Affine3f& rough_map_to_sensor,
    const HexPointsStamped& sensor_points_stamped) {
  static HexTransStamped fine_map_to_sensor;
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  // source
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_points = CropPointsCylinder(
      sensor_points_stamped.points, ksource_min_range_, ksource_max_range_,
      ksource_min_angle_, ksource_max_angle_);
  source_points = DownSample(source_points, ksource_voxel_size_);
  // data_interface.PublishDebugPoints(source_points, "base_link");

  // alignment
  fine_map_to_sensor = NdtAlignment(rough_map_to_sensor, source_points,
                                    sensor_points_stamped.time);

  // fine_map_to_sensor =
  //     DoubleIcpAlignment(rough_map_to_sensor, target_points, source_points,
  //     sensor_points_stamped.time);

  return fine_map_to_sensor;
}

void PcdLocalization::ResultProcess(const HexTransStamped& fine_map_to_sensor,
                                    const Eigen::Affine3f& odom_to_sensor) {
  static DataInterface& data_interface = DataInterface::GetDataInterface();
  map_to_sensor_ = HexTransToAffine(fine_map_to_sensor);

  if (kmode_pure_lidar_) {
    delta_trans_ = last_trans_.inverse() * map_to_sensor_;
    last_trans_ = map_to_sensor_;
  }

  map_to_odom_ = map_to_sensor_ * odom_to_sensor.inverse();
  // std::cout << "map to sensor: " << std::endl
  //           << map_to_sensor_.matrix() << std::endl;
  data_interface.BroadcastMapToOdom(map_to_odom_, fine_map_to_sensor.time);
  data_interface.PublishSensorTrans(fine_map_to_sensor);
}

const HexTransStamped& PcdLocalization::NdtAlignment(
    const Eigen::Affine3f& init_guess,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_points,
    double valid_time) {
  static HexTransStamped final_trans;

  // align
  // --- test --- //
  const auto align_start_time = std::chrono::system_clock::now();
  ndt_register_.setInputSource(source_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr align_pcd(
      new pcl::PointCloud<pcl::PointXYZ>());
  ndt_register_.align(*align_pcd, init_guess.matrix());
  const auto align_end_time = std::chrono::system_clock::now();
  std::cout << "align time: "
            << std::chrono::duration_cast<std::chrono::microseconds>(
                   align_end_time - align_start_time)
                       .count() /
                   1000.0
            << "ms" << std::endl;

  // check
  bool ndt_success = ndt_register_.hasConverged();
  Eigen::Affine3f ndt_result(Eigen::Affine3f::Identity());
  ndt_result.matrix() = ndt_register_.getFinalTransformation();
  if (ndt_success) {
    // final_trans = AffineToHexTrans(ndt_result, valid_time);
    final_trans = AffineToHexTrans(ndt_result, valid_time);
  } else {
    final_trans = AffineToHexTrans(init_guess, 0.0);
  }

  return final_trans;
}

// const HexTransStamped& PcdLocalization::DoubleIcpAlignment(
//     const Eigen::Affine3f& init_guess,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_points,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_points,
//     double valid_time) {
//   static HexTransStamped fine_result;
//   static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
//   icp_register; icp_register.setMaximumIterations(20);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr align_pcd(
//       new pcl::PointCloud<pcl::PointXYZ>());

//   // rough
//   pcl::PointCloud<pcl::PointXYZ>::Ptr rough_target_points =
//       DownSample(target_points, 2.0);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr rough_source_points =
//       DownSample(source_points, 0.5);
//   icp_register.setInputTarget(rough_target_points);
//   icp_register.setInputSource(rough_source_points);
//   icp_register.align(*align_pcd, init_guess.matrix());
//   bool rough_success = icp_register.hasConverged();
//   Eigen::Matrix4f rough_result = icp_register.getFinalTransformation();
//   if (!rough_success) {
//     fine_result = AffineToHexTrans(init_guess, 0.0);
//     return fine_result;
//   }

//   // fine
//   pcl::PointCloud<pcl::PointXYZ>::Ptr fine_target_points =
//       DownSample(target_points, 0.4);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr fine_source_points =
//       DownSample(source_points, 0.1);
//   icp_register.setInputTarget(fine_target_points);
//   icp_register.setInputSource(fine_source_points);
//   icp_register.align(*align_pcd, rough_result);
//   bool fine_success = icp_register.hasConverged();
//   Eigen::Affine3f align_result(Eigen::Affine3f::Identity());
//   align_result.matrix() =
//   icp_register.getFinalTransformation(); if (fine_success) {
//     fine_result = AffineToHexTrans(align_result, valid_time);
//   } else {
//     fine_result = AffineToHexTrans(align_result, 0.0);
//   }

//   return fine_result;
// }

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::DownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double voxel_size) {
  static pcl::VoxelGrid<pcl::PointXYZ> voxel_down_sampler;

  pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampled_pcd(
      new pcl::PointCloud<pcl::PointXYZ>());
  voxel_down_sampler.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_down_sampler.setInputCloud(raw_pcd);
  voxel_down_sampler.filter(*down_sampled_pcd);

  return down_sampled_pcd;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::CropPointsBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double offset_x,
    double offset_y, double offset_z, const Eigen::Vector3f& translation) {
  static pcl::CropBox<pcl::PointXYZ> crop_box_filter;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Vector4f min_point(translation.x() - offset_x,
                            translation.y() - offset_y,
                            translation.z() - offset_z, 1.0);
  Eigen::Vector4f max_point(translation.x() + offset_x,
                            translation.y() + offset_y,
                            translation.z() + offset_z, 1.0);
  crop_box_filter.setMin(min_point);
  crop_box_filter.setMax(max_point);
  crop_box_filter.setNegative(false);
  crop_box_filter.setInputCloud(raw_pcd);
  crop_box_filter.filter(*cropped_points);

  return cropped_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::CropPointsCylinder(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double range_min,
    double range_max, double angle_min, double angle_max) {
  static pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  cropped_points->clear();

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = raw_pcd->begin();
       it != raw_pcd->end(); it++) {
    const pcl::PointXYZ& point = *it;
    double range_square = point.x * point.x + point.y * point.y;
    double angle = atan2(point.y, point.x);
    if (range_square < range_max * range_max &&
        range_square > range_min * range_min && angle < angle_max &&
        angle > angle_min) {
      cropped_points->points.push_back(point);
    }
  }

  return cropped_points;
}

Eigen::Affine3f PcdLocalization::HexTransToAffine(
    const HexTransStamped& hex_trans) {
  Eigen::Affine3f affine(Eigen::Affine3f::Identity());
  affine.matrix().block<3, 3>(0, 0) = hex_trans.orientation.toRotationMatrix();
  affine.matrix().block<3, 1>(0, 3) = hex_trans.translation;

  return affine;
}

HexTransStamped PcdLocalization::AffineToHexTrans(const Eigen::Affine3f& affine,
                                                  double time) {
  HexTransStamped hex_trans;
  hex_trans.time = time;
  hex_trans.translation = affine.matrix().block<3, 1>(0, 3);
  hex_trans.orientation = Eigen::Quaternionf(affine.matrix().block<3, 3>(0, 0));

  return hex_trans;
}

}  // namespace localization
}  // namespace hex
