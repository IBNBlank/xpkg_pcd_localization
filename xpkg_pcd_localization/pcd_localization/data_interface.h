/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef PCD_LOCALIZATION_DATA_INTERFACE_H_
#define PCD_LOCALIZATION_DATA_INTERFACE_H_

#include <pcd_localization/hex_utility.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/PointCloud2.h"

using hex::utility::HexPointsStamped;
using hex::utility::HexTransStamped;

namespace hex {
namespace localization {

enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class DataInterface {
 public:
  static DataInterface& GetDataInterface() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(LogLevel, const char*, ...);
  inline void Work() { ros::spin(); }
  inline void Shutdown() { ros::shutdown(); }
  inline bool Ok() { return ros::ok(); }

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)());
  void Deinit();

  // Parameter Handle
  inline const std::string& GetMapPath() { return kmap_path_; }
  inline const std::string& GetMapFrame() { return kmap_frame_; }
  inline const std::string& GetOdomFrame() { return kodom_frame_; }
  inline const std::string& GetSensorFrame() { return ksensor_frame_; }
  inline double GetTargetVoxelSize() { return ktarget_voxel_size_; }
  inline double GetTargetPcdSize() { return ktarget_pcd_size_; }
  inline double GetTargetUpdateDistance() { return ktarget_update_distance_; }
  inline double GetSourceVoxelSize() { return ksource_voxel_size_; }
  inline double GetSourceMaxRange() { return ksource_max_range_; }
  inline double GetSourceMinRange() { return ksource_min_range_; }
  inline double GetSourceMaxAngle() { return ksource_max_angle_; }
  inline double GetSourceMinAngle() { return ksource_min_angle_; }
  inline double GetNdtTransEpsilon() { return kndt_trans_epsilon_; }
  inline double GetNdtStepSize() { return kndt_step_size_; }
  inline double GetNdtResolution() { return kndt_resolution_; }
  inline int32_t GetNdtMaxIterations() { return kndt_max_iterations_; }
  inline bool GetModeAutoStart() { return kmode_auto_start_; }
  inline bool GetModePureLidar() { return kmode_pure_lidar_; }

  // Publisher Handle
  void PublishSensorTrans(const HexTransStamped&);
  void PublishMapPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
  void PublishDebugPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                          const std::string&);

  // Subscriber Handle
  inline bool GetInitTransFlag() { return init_trans_flag_; }
  inline void ResetInitTransFlag() { init_trans_flag_ = false; }
  inline const HexTransStamped& GetInitTrans() { return init_trans_; }
  inline bool GetLidarCloudFlag() { return lidar_cloud_flag_; }
  inline void ResetLidarCloudFlag() { lidar_cloud_flag_ = false; }
  inline const HexPointsStamped& GetLidarCloud() { return lidar_cloud_; }

  // Other Handle
  void BroadcastMapToOdom(const Eigen::Affine3f&, double);
  const HexTransStamped& ListenFrameToSensor(const std::string&);

 protected:
  // Timer Handle
  inline void TimerCallback(const ros::TimerEvent&) { timer_handle_(); }

  // Subscriber Handle
  void InitTransHandle(const geometry_msgs::PoseWithCovarianceStamped&);
  void PointCloudHandle(const sensor_msgs::PointCloud2&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit();
  void VariableInit();
  void PublisherInit();
  void SubscriberInit();
  void TimerInit(double, void (*)());

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  ros::Timer timer_;
  void (*timer_handle_)();

  // Publisher Handle
  ros::Publisher sensor_trans_pub_;
  ros::Publisher map_points_pub_;
  ros::Publisher debug_points_pub_;

  // Subscriber Handle
  ros::Subscriber init_trans_sub_;
  ros::Subscriber lidar_points_sub_;

  // Parameters Handle
  std::string kmap_path_;
  std::string kmap_frame_;
  std::string kodom_frame_;
  std::string ksensor_frame_;
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
  std::vector<double> kinit_position_;
  std::vector<double> kinit_orientation_;
  bool kmode_auto_start_;
  bool kmode_pure_lidar_;

  // Variable Handle
  bool init_trans_flag_;
  bool lidar_cloud_flag_;
  HexTransStamped init_trans_;
  HexPointsStamped lidar_cloud_;
};

}  // namespace localization
}  // namespace hex

#endif  // PCD_LOCALIZATION_DATA_INTERFACE_H_
