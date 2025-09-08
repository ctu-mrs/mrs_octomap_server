
#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_


#include <rclcpp/rclcpp.hpp>

#include <octomap/OcTreeNode.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/msg/Vector3.hpp>

#include <sensor_msgs/msg/PointCloud2.hpp>
#include <sensor_msgs/msg/LaserScan.hpp>
#include <sensor_msgs/msg/CameraInfo.hpp>

#include <std_srvs/srv/Empty.hpp>
#include <std_srvs/srv/Trigger.hpp>

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <octomap_msgs/msg/BoundingBoxQueryRequest.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>


#include <mrs_octomap_server/msg/PoseWithSize.hpp>


namespace mrs_octomap_server
{
class OctomapServer : public rclcpp::Node {

public:
    COMPOSITION_PUBLIC
  virtual void onInit(const rclcpp::NodeOptions& options);

  bool callbackLoadMap(std::shared_ptr<mrs_msgs::msg::String::Request> req, [[maybe_unused]] std::shared_ptr<mrs_msgs::msg::String::Response> resp);
  bool callbackSaveMap(std::shared_ptr<mrs_msgs::msg::String::Request> req, [[maybe_unused]] std::shared_ptr<mrs_msgs::msg::String::Response> resp);

  bool callbackResetMap(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> resp);

  void callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const SensorType_t sensor_type, const int sensor_id, const std::string topic,
                             const bool pcl_over_max_range = false);

  void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callbackCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg, const int sensor_id);
  bool loadFromFile(const std::string& filename);
  bool saveToFile(const std::string& filename);

private:
  std::atomic<bool> is_initialized_ = false;

  // | -------------------- topic subscribers ------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>            sh_height_;
  mrs_lib::SubscriberHandler<mrs_octomap_server::msg::PoseWithSize>    sh_clear_box_;

  std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_3dlaser_pc2_;
  std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_depth_cam_pc2_;
  std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>>  sh_depth_cam_info_;
  std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::LaserScan>>   sh_laser_scan_;

  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_full_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_binary_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_full_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_binary_;

  // | -------------------- service serviers -------------------- |

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ss_reset_map_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ss_save_map_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ss_load_map_;


  
  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_global_map_publisher_;
  double     _global_map_publisher_rate_;
  void       timerGlobalMapPublisher();

  rclcpp::TimerBase::SharedPtr timer_global_map_creator_;
  double     _global_map_creator_rate_;
  void       timerGlobalMapCreator();

  rclcpp::TimerBase::SharedPtr timer_local_map_publisher_;
  void       timerLocalMapPublisher();

  rclcpp::TimerBase::SharedPtr timer_local_map_resizer_;
  void       timerLocalMapResizer();

  rclcpp::TimerBase::SharedPtr timer_persistency_;
  void       timerPersistency();

  rclcpp::TimerBase::SharedPtr timer_altitude_alignment_;
  void       timerAltitudeAlignment();

  // | ----------------------- parameters ----------------------- |

  bool        _simulation_;
  std::string _uav_name_;

  bool _scope_timer_enabled_;

  double _robot_height_;

  bool        _persistency_enabled_;
  std::string _persistency_map_name_;
  double      _persistency_save_time_;

  bool   _persistency_align_altitude_enabled_;
  double _persistency_align_altitude_distance_;

  bool   _global_map_publish_full_;
  bool   _global_map_publish_binary_;
  bool   _global_map_enabled_;
  double _global_map_size_;

  bool _map_while_grounded_;

  bool _local_map_publish_full_;
  bool _local_map_publish_binary_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::shared_ptr<OcTree_t> octree_global_;
  std::shared_ptr<OcTree_t> octree_global_0_;
  std::shared_ptr<OcTree_t> octree_global_1_;
  int                       octree_global_idx_ = 0;
  std::mutex                mutex_octree_global_;

  std::shared_ptr<OcTree_t> octree_local_;
  std::shared_ptr<OcTree_t> octree_local_0_;
  std::shared_ptr<OcTree_t> octree_local_1_;
  int                       octree_local_idx_ = 0;
  std::mutex                mutex_octree_local_;

  std::atomic<bool> octrees_initialized_ = false;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  std::string _world_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  bool        _global_map_compress_;
  std::string _map_path_;

  float      _local_map_width_max_;
  float      _local_map_width_min_;
  float      _local_map_height_max_;
  float      _local_map_height_min_;
  float      local_map_width_;
  float      local_map_height_;
  std::mutex mutex_local_map_dimensions_;
  double     _local_map_publisher_rate_;

  double     local_map_duty_                 = 0;
  double     _local_map_duty_high_threshold_ = 0;
  double     _local_map_duty_low_threshold_  = 0;
  std::mutex mutex_local_map_duty_;

  bool   _unknown_rays_update_free_space_;
  bool   _unknown_rays_clear_occupied_;
  double _unknown_rays_distance_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y);

  bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTree_t>& octree);

  virtual void insertPointCloud(const geometry_msgs::msg::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud,
                                double free_ray_distance, bool unknown_clear_occupied = false);

  void initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params);
  void initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params);

  void timeoutGeneric(const std::string& topic, const rclcpp::Time& last_msg, [[maybe_unused]] const int n_pubs);
  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  int n_sensors_2d_lidar_;
  int n_sensors_3d_lidar_;
  int n_sensors_depth_cam_;

  std::vector<xyz_lut_t> sensor_2d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_3d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_depth_camera_xyz_lut_;

  std::vector<SensorParams2DLidar_t> sensor_params_2d_lidar_;

  std::vector<SensorParams3DLidar_t> sensor_params_3d_lidar_;

  std::vector<SensorParamsDepthCam_t> sensor_params_depth_cam_;

  std::mutex mutex_lut_;

  std::vector<bool> vec_camera_info_processed_;

  // sensor model
  double _probHit_;
  double _probMiss_;
  double _thresMin_;
  double _thresMax_;
};

}











#endif  // COMPOSITION__SERVER_COMPONENT_HPP_