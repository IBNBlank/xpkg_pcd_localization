/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include "pcd_localization/data_interface.h"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/TransformStamped.h"

namespace hex {
namespace localization {

void DataInterface::Log(LogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case LogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case LogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case LogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case LogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case LogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(LogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() { Shutdown(); }

void DataInterface::ParameterInit() {
  nh_local_ptr_->param<std::string>("map_path", kmap_path_, "");
  nh_local_ptr_->param<std::string>("map_frame", kmap_frame_, "map");
  nh_local_ptr_->param<std::string>("odom_frame", kodom_frame_, "odom");
  nh_local_ptr_->param<std::string>("sensor_frame", ksensor_frame_, "lidar");

  nh_local_ptr_->param<double>("target_voxel_size", ktarget_voxel_size_, 0.5);
  nh_local_ptr_->param<double>("target_pcd_size", ktarget_pcd_size_, 100.0);
  nh_local_ptr_->param<double>("target_update_distance",
                               ktarget_update_distance_, 45.0);

  nh_local_ptr_->param<double>("source_voxel_size", ksource_voxel_size_, 0.5);
  nh_local_ptr_->param<double>("source_max_range", ksource_max_range_, 50.0);
  nh_local_ptr_->param<double>("source_min_range", ksource_min_range_, 1.0);
  nh_local_ptr_->param<double>("source_max_angle", ksource_max_angle_, M_PI);
  nh_local_ptr_->param<double>("source_min_angle", ksource_min_angle_, -M_PI);

  nh_local_ptr_->param<double>("ndt_trans_epsilon", kndt_trans_epsilon_, 0.05);
  nh_local_ptr_->param<double>("ndt_step_size", kndt_step_size_, 0.1);
  nh_local_ptr_->param<double>("ndt_resolution", kndt_resolution_, 2.0);
  nh_local_ptr_->param<int32_t>("ndt_max_iterations", kndt_max_iterations_, 30);

  nh_local_ptr_->param<std::vector<double>>(
      "init_position", kinit_position_, std::vector<double>({0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "init_orientation", kinit_orientation_,
      std::vector<double>({1.0, 0.0, 0.0, 0.0}));

  nh_local_ptr_->param<bool>("mode_auto_start", kmode_auto_start_, true);
  nh_local_ptr_->param<bool>("mode_pure_lidar", kmode_pure_lidar_, false);
}

void DataInterface::VariableInit() {
  init_trans_flag_ = false;
  init_trans_.translation = Eigen::Vector3f(
      kinit_position_.at(0), kinit_position_.at(1), kinit_position_.at(2));
  init_trans_.orientation =
      Eigen::Quaternionf(kinit_orientation_.at(0), kinit_orientation_.at(1),
                         kinit_orientation_.at(2), kinit_orientation_.at(3));
  lidar_cloud_flag_ = false;
  lidar_cloud_.points =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

void DataInterface::PublisherInit() {
  sensor_trans_pub_ =
      nh_ptr_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "sensor_trans", 1);
  map_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("map_points", 1, true);
  debug_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("debug_points", 1);
}

void DataInterface::SubscriberInit() {
  init_trans_sub_ = nh_ptr_->subscribe("init_trans", 1,
                                       &DataInterface::InitTransHandle, this);
  lidar_points_sub_ = nh_ptr_->subscribe(
      "lidar_points", 1, &DataInterface::PointCloudHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  timer_handle_ = handle;
  timer_ = nh_ptr_->createTimer(ros::Duration(period * 0.001),
                                &DataInterface::TimerCallback, this);
}

void DataInterface::PublishSensorTrans(const HexTransStamped& sensor_trans) {
  geometry_msgs::PoseWithCovarianceStamped sensor_trans_msg;

  sensor_trans_msg.header.stamp = ros::Time::now();
  sensor_trans_msg.header.frame_id = kmap_frame_;
  sensor_trans_msg.pose.pose.position.x = sensor_trans.translation.x();
  sensor_trans_msg.pose.pose.position.y = sensor_trans.translation.y();
  sensor_trans_msg.pose.pose.position.z = sensor_trans.translation.z();
  sensor_trans_msg.pose.pose.orientation.w = sensor_trans.orientation.w();
  sensor_trans_msg.pose.pose.orientation.x = sensor_trans.orientation.x();
  sensor_trans_msg.pose.pose.orientation.y = sensor_trans.orientation.y();
  sensor_trans_msg.pose.pose.orientation.z = sensor_trans.orientation.z();

  sensor_trans_pub_.publish(sensor_trans_msg);
}

void DataInterface::PublishMapPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_points) {
  sensor_msgs::PointCloud2 map_points_msg;

  pcl::toROSMsg(*map_points, map_points_msg);
  map_points_msg.header.stamp = ros::Time::now();
  map_points_msg.header.frame_id = kmap_frame_;

  map_points_pub_.publish(map_points_msg);
}

void DataInterface::PublishDebugPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& debug_points,
    const std::string& frame) {
  sensor_msgs::PointCloud2 debug_points_msg;

  pcl::toROSMsg(*debug_points, debug_points_msg);
  debug_points_msg.header.stamp = ros::Time::now();
  debug_points_msg.header.frame_id = frame;

  debug_points_pub_.publish(debug_points_msg);
}

void DataInterface::BroadcastMapToOdom(const Eigen::Affine3f& trans,
                                       double time) {
  static tf2_ros::TransformBroadcaster tf_broadcaster;

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = kmap_frame_;
  transform.child_frame_id = kodom_frame_;
  transform.header.stamp = ros::Time(time);
  transform.transform.translation.x = trans.translation().x();
  transform.transform.translation.y = trans.translation().y();
  transform.transform.translation.z = trans.translation().z();
  Eigen::Quaternionf quat = Eigen::Quaternionf(trans.rotation());
  transform.transform.rotation.w = quat.w();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();

  tf_broadcaster.sendTransform(transform);
}

const HexTransStamped& DataInterface::ListenFrameToSensor(
    const std::string& frame) {
  static tf2_ros::Buffer tf2_buffer;
  static tf2_ros::TransformListener tf2_listener(tf2_buffer);
  static HexTransStamped sensor_trans{ros::Time::now().toSec(),
                                      Eigen::Vector3f(0, 0, 0),
                                      Eigen::Quaternionf(1, 0, 0, 0)};

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf2_buffer.lookupTransform(frame, ksensor_frame_, ros::Time(0));
    sensor_trans.time = transform.header.stamp.toSec();
    sensor_trans.translation = Eigen::Vector3f(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z);
    sensor_trans.orientation = Eigen::Quaternionf(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
  } catch (tf2::TransformException& ex) {
    Log(LogLevel::kError, "%s", ex.what());
    sensor_trans.time = transform.header.stamp.toSec();
    sensor_trans.translation = Eigen::Vector3f(0.0, 0.0, 0.0);
    sensor_trans.orientation = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
  }

  return sensor_trans;
}

void DataInterface::InitTransHandle(
    const geometry_msgs::PoseWithCovarianceStamped& msg) {
  if (ros::Time::now().toSec() - msg.header.stamp.toSec() < 0.2 &&
      !init_trans_flag_) {
    init_trans_.time = msg.header.stamp.toSec();
    init_trans_.translation =
        Eigen::Vector3f(msg.pose.pose.position.x, msg.pose.pose.position.y,
                        msg.pose.pose.position.z);
    init_trans_.orientation = Eigen::Quaternionf(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    init_trans_flag_ = true;
  }
}

void DataInterface::PointCloudHandle(const sensor_msgs::PointCloud2& msg) {
  if (ros::Time::now().toSec() - msg.header.stamp.toSec() < 0.2 &&
      !lidar_cloud_flag_) {
    lidar_cloud_.time = msg.header.stamp.toSec();
    lidar_cloud_.points->clear();
    pcl::fromROSMsg(msg, *lidar_cloud_.points);

    lidar_cloud_flag_ = true;
  }
}

}  // namespace localization
}  // namespace hex
