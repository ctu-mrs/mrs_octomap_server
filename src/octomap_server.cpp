/* includes //{ */

#include <ros/init.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTreeNode.h>
#include <octomap/octomap_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Eigen>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Geometry>

#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>

#include <octomap_tools/octomap_methods.h>

#include <mrs_msgs/String.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>

#include <mrs_msgs/SetInt.h>

#include <filesystem>

#include <mrs_octomap_server/conversions.h>

#include <laser_geometry/laser_geometry.h>

#include <cmath>

//}

namespace ph = std::placeholders;

namespace mrs_octomap_server
{

/* defines //{ */

using vec3s_t = Eigen::Matrix<float, 3, -1>;
using vec3_t  = Eigen::Vector3f;

struct xyz_lut_t
{
  vec3s_t directions;  // a matrix of normalized direction column vectors
  vec3s_t offsets;     // a matrix of offset vectors
};

typedef struct
{
  double max_range;
  int    horizontal_rays;
} SensorParams2DLidar_t;

typedef struct
{
  double max_range;
  double vertical_fov;
  int    vertical_rays;
  int    horizontal_rays;
} SensorParams3DLidar_t;

typedef struct
{
  double max_range;
  double vertical_fov;
  double horizontal_fov;
  int    vertical_rays;
  int    horizontal_rays;
} SensorParamsDepthCam_t;

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::ColorOcTree;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::OcTree;
#endif

typedef enum
{

  LIDAR_3D,
  LIDAR_2D,
  LIDAR_1D,
  DEPTH_CAMERA,
  ULTRASOUND,

} SensorType_t;

//}

/* class OctomapServer //{ */

class OctomapServer : public nodelet::Nodelet {

public:
  virtual void onInit();

  bool callbackLoadMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);
  bool callbackSaveMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);

  bool callbackResetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  bool callbackSetLocalFractor([[maybe_unused]] mrs_msgs::SetInt::Request& req, [[maybe_unused]] mrs_msgs::SetInt::Response& resp);
  bool callbackSetGlobalFractor([[maybe_unused]] mrs_msgs::SetInt::Request& req, [[maybe_unused]] mrs_msgs::SetInt::Response& resp);

  void callback3dLidarCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp, const SensorType_t sensor_type, const int sensor_id);
  void callbackLaserScan(mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>& wrp);
  bool loadFromFile(const std::string& filename);
  bool saveToFile(const std::string& filename);

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  // | -------------------- topic subscribers ------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>            sh_height_;

  std::vector<mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>> sh_3dlaser_pc2_;
  std::vector<mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>> sh_depth_cam_pc2_;
  std::vector<mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>>   sh_laser_scan_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_map_global_full_;
  ros::Publisher pub_map_global_binary_;

  ros::Publisher pub_map_local_full_;
  ros::Publisher pub_map_local_binary_;

  // | -------------------- service serviers -------------------- |

  ros::ServiceServer ss_reset_map_;
  ros::ServiceServer ss_save_map_;
  ros::ServiceServer ss_load_map_;
  ros::ServiceServer ss_set_global_fractor_;
  ros::ServiceServer ss_set_local_fractor_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_global_map_publisher_;
  double     _global_map_publisher_rate_;
  void       timerGlobalMapPublisher([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_global_map_creator_;
  double     _global_map_creator_rate_;
  void       timerGlobalMapCreator([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_publisher_;
  void       timerLocalMapPublisher([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_resizer_;
  void       timerLocalMapResizer([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_persistency_;
  void       timerPersistency([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_altitude_alignment_;
  void       timerAltitudeAlignment([[maybe_unused]] const ros::TimerEvent& event);

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

  bool _global_map_publish_full_;
  bool _global_map_publish_binary_;
  bool _global_map_enabled_;

  bool _map_while_grounded_;

  bool _local_map_publish_full_;
  bool _local_map_publish_binary_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::shared_ptr<OcTree_t> octree_global_;
  std::mutex                mutex_octree_global_;

  std::shared_ptr<OcTree_t> octree_local_;
  std::shared_ptr<OcTree_t> octree_local_0_;
  std::shared_ptr<OcTree_t> octree_local_1_;
  int                       octree_local_idx_ = 0;
  std::mutex                mutex_octree_local_;

  std::atomic<bool> octrees_initialized_;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  std::string _world_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  bool        _global_map_compress_;
  std::string _map_path_;

  int        _local_map_width_;
  int        _local_map_height_;
  int        local_map_width_;
  int        local_map_height_;
  std::mutex mutex_local_map_dimensions_;
  double     _local_map_publisher_rate_;

  double     local_map_duty_ = 0;
  std::mutex mutex_local_map_duty_;

  bool   _unknown_rays_update_free_space_;
  bool   _unknown_rays_clear_occupied_;
  double _unknown_rays_distance_;

  int        local_resolution_fractor_;
  int        global_resolution_fractor_;
  std::mutex mutex_resolution_fractor_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, const int& from_fractor, std::shared_ptr<OcTree_t>& to, const int& to_fractor,
                      const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, const int& from_fractor, std::shared_ptr<OcTree_t>& to, const int& to_fractor);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y);

  bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTree_t>& octree);

  virtual void insertPointCloud(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud);

  void initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params);
  void initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params);

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

  // sensor model
  double _probHit_;
  double _probMiss_;
  double _thresMin_;
  double _thresMax_;
};

//}

/* onInit() //{ */

void OctomapServer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  /* params //{ */

  mrs_lib::ParamLoader param_loader(nh_, ros::this_node::getName());

  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("scope_timer/enabled", _scope_timer_enabled_);

  param_loader.loadParam("map_while_grounded", _map_while_grounded_);

  param_loader.loadParam("persistency/enabled", _persistency_enabled_);
  param_loader.loadParam("persistency/save_time", _persistency_save_time_);
  param_loader.loadParam("persistency/map_name", _persistency_map_name_);
  param_loader.loadParam("persistency/align_altitude/enabled", _persistency_align_altitude_enabled_);
  param_loader.loadParam("persistency/align_altitude/ground_detection_distance", _persistency_align_altitude_distance_);
  param_loader.loadParam("persistency/align_altitude/robot_height", _robot_height_);

  param_loader.loadParam("global_map/publisher_rate", _global_map_publisher_rate_);
  param_loader.loadParam("global_map/creation_rate", _global_map_creator_rate_);
  param_loader.loadParam("global_map/enabled", _global_map_enabled_);
  param_loader.loadParam("global_map/compress", _global_map_compress_);
  param_loader.loadParam("global_map/publish_full", _global_map_publish_full_);
  param_loader.loadParam("global_map/publish_binary", _global_map_publish_binary_);
  param_loader.loadParam("global_map/initial_fractor", global_resolution_fractor_);

  param_loader.loadParam("local_map/size/width", _local_map_width_);
  param_loader.loadParam("local_map/size/height", _local_map_height_);
  param_loader.loadParam("local_map/publisher_rate", _local_map_publisher_rate_);
  param_loader.loadParam("local_map/publish_full", _local_map_publish_full_);
  param_loader.loadParam("local_map/publish_binary", _local_map_publish_binary_);
  param_loader.loadParam("local_map/initial_fractor", local_resolution_fractor_);

  local_map_width_  = _local_map_width_;
  local_map_height_ = _local_map_height_;

  param_loader.loadParam("resolution", octree_resolution_);
  param_loader.loadParam("world_frame_id", _world_frame_);
  param_loader.loadParam("robot_frame_id", _robot_frame_);

  param_loader.loadParam("map_path", _map_path_);

  param_loader.loadParam("unknown_rays/update_free_space", _unknown_rays_update_free_space_);
  param_loader.loadParam("unknown_rays/clear_occupied", _unknown_rays_clear_occupied_);
  param_loader.loadParam("unknown_rays/ray_distance", _unknown_rays_distance_);

  param_loader.loadParam("sensor_params/2d_lidar/n_sensors", n_sensors_2d_lidar_);
  param_loader.loadParam("sensor_params/3d_lidar/n_sensors", n_sensors_3d_lidar_);
  param_loader.loadParam("sensor_params/depth_camera/n_sensors", n_sensors_depth_cam_);

  for (int i = 0; i < n_sensors_2d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/2d_lidar/sensor_" << i << "/max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/2d_lidar/sensor_" << i << "/horizontal_rays";

    SensorParams2DLidar_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);

    sensor_params_2d_lidar_.push_back(params);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/depth_camera/sensor_" << i << "/max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/depth_camera/sensor_" << i << "/horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params/depth_camera/sensor_" << i << "/vertical_rays";

    std::stringstream hfov_param_name;
    hfov_param_name << "sensor_params/depth_camera/sensor_" << i << "/horizontal_fov_angle";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params/depth_camera/sensor_" << i << "/vertical_fov_angle";

    SensorParamsDepthCam_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);
    param_loader.loadParam(vertical_rays_param_name.str(), params.vertical_rays);
    param_loader.loadParam(hfov_param_name.str(), params.horizontal_fov);
    param_loader.loadParam(vfov_param_name.str(), params.vertical_fov);

    sensor_params_depth_cam_.push_back(params);
  }

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/3d_lidar/sensor_" << i << "/max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_rays";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_fov_angle";

    SensorParams3DLidar_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);
    param_loader.loadParam(vertical_rays_param_name.str(), params.vertical_rays);
    param_loader.loadParam(vfov_param_name.str(), params.vertical_fov);

    sensor_params_3d_lidar_.push_back(params);
  }

  param_loader.loadParam("sensor_model/hit", _probHit_);
  param_loader.loadParam("sensor_model/miss", _probMiss_);
  param_loader.loadParam("sensor_model/min", _thresMin_);
  param_loader.loadParam("sensor_model/max", _thresMax_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
    ros::requestShutdown();
  }

  //}

  /* initialize sensor LUT model //{ */

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    xyz_lut_t lut_table;

    sensor_3d_lidar_xyz_lut_.push_back(lut_table);

    initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[i], sensor_params_3d_lidar_[i]);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    xyz_lut_t lut_table;

    sensor_depth_camera_xyz_lut_.push_back(lut_table);

    initializeDepthCamLUT(sensor_depth_camera_xyz_lut_[i], sensor_params_depth_cam_[i]);
  }

  //}

  /* initialize octomap object & params //{ */

  octree_global_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_->setProbHit(_probHit_);
  octree_global_->setProbMiss(_probMiss_);
  octree_global_->setClampingThresMin(_thresMin_);
  octree_global_->setClampingThresMax(_thresMax_);

  octree_local_0_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_0_->setProbHit(_probHit_);
  octree_local_0_->setProbMiss(_probMiss_);
  octree_local_0_->setClampingThresMin(_thresMin_);
  octree_local_0_->setClampingThresMax(_thresMax_);

  octree_local_1_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_1_->setProbHit(_probHit_);
  octree_local_1_->setProbMiss(_probMiss_);
  octree_local_1_->setClampingThresMin(_thresMin_);
  octree_local_1_->setClampingThresMax(_thresMax_);

  octree_local_ = octree_local_0_;

  if (_persistency_enabled_) {
    bool success = loadFromFile(_persistency_map_name_);

    if (success) {
      ROS_INFO("[OctomapServer]: loaded persistency map");
    } else {

      ROS_ERROR("[OctomapServer]: failed to load the persistency map, turning persistency off");

      _persistency_enabled_ = false;
    }
  }

  if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
    octrees_initialized_ = false;
  } else {
    octrees_initialized_ = true;
  }

  //}

  /* transformer //{ */

  transformer_ = std::make_unique<mrs_lib::Transformer>("OctomapServer");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->setLookupTimeout(ros::Duration(0.5));
  transformer_->retryLookupNewest(false);

  //}

  /* publishers //{ */

  pub_map_global_full_   = nh_.advertise<octomap_msgs::Octomap>("octomap_global_full_out", 1);
  pub_map_global_binary_ = nh_.advertise<octomap_msgs::Octomap>("octomap_global_binary_out", 1);

  pub_map_local_full_   = nh_.advertise<octomap_msgs::Octomap>("octomap_local_full_out", 1);
  pub_map_local_binary_ = nh_.advertise<octomap_msgs::Octomap>("octomap_local_binary_out", 1);

  //}

  /* subscribers //{ */

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapServer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_height_               = mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>(shopts, "height_in");

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream ss;
    ss << "lidar_3d_" << i << "_in";

    sh_3dlaser_pc2_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(
        shopts, ss.str(), std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, LIDAR_3D, i)));
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream ss;
    ss << "depth_camera_" << i << "_in";

    sh_depth_cam_pc2_.push_back(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(
        shopts, ss.str(), std::bind(&OctomapServer::callback3dLidarCloud2, this, std::placeholders::_1, DEPTH_CAMERA, i)));
  }

  /* sh_laser_scan_ = mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>(shopts, "laser_scan_in", &OctomapServer::callbackLaserScan, this); */

  //}

  /* service servers //{ */

  ss_reset_map_          = nh_.advertiseService("reset_map_in", &OctomapServer::callbackResetMap, this);
  ss_save_map_           = nh_.advertiseService("save_map_in", &OctomapServer::callbackSaveMap, this);
  ss_load_map_           = nh_.advertiseService("load_map_in", &OctomapServer::callbackLoadMap, this);
  ss_set_global_fractor_ = nh_.advertiseService("set_global_fractor_in", &OctomapServer::callbackSetGlobalFractor, this);
  ss_set_local_fractor_  = nh_.advertiseService("set_local_fractor_in", &OctomapServer::callbackSetLocalFractor, this);

  //}

  /* timers //{ */

  if (_global_map_enabled_) {
    timer_global_map_publisher_ = nh_.createTimer(ros::Rate(_global_map_publisher_rate_), &OctomapServer::timerGlobalMapPublisher, this);
    timer_global_map_creator_   = nh_.createTimer(ros::Rate(_global_map_creator_rate_), &OctomapServer::timerGlobalMapCreator, this);
  }

  timer_local_map_publisher_ = nh_.createTimer(ros::Rate(_local_map_publisher_rate_), &OctomapServer::timerLocalMapPublisher, this);

  timer_local_map_resizer_ = nh_.createTimer(ros::Rate(1.0), &OctomapServer::timerLocalMapResizer, this);

  if (_persistency_enabled_) {
    timer_persistency_ = nh_.createTimer(ros::Rate(1.0 / _persistency_save_time_), &OctomapServer::timerPersistency, this);
  }

  if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
    timer_altitude_alignment_ = nh_.createTimer(ros::Rate(1.0), &OctomapServer::timerAltitudeAlignment, this);
  }

  //}

  /* scope timer logger //{ */

  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  //}

  is_initialized_ = true;

  ROS_INFO("[%s]: Initialized", ros::this_node::getName().c_str());
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackLaserScan() //{ */

void OctomapServer::callbackLaserScan(mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>& wrp) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  if (!_map_while_grounded_) {

    if (!sh_control_manager_diag_.hasMsg()) {

      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, can not integrate data!");
      return;

    } else {

      ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

      if ((ros::Time::now() - last_time).toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, can not integrate data!");
        return;
      }

      // TODO is this the best option?
      if (!sh_control_manager_diag_.getMsg()->flying_normally) {
        ROS_INFO_THROTTLE(1.0, "[OctomapServer]: not flying normally, therefore, not integrating data");
        return;
      }
    }
  }

  sensor_msgs::LaserScanConstPtr scan = wrp.getMsg();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf;

  auto res = transformer_->getTransform(scan->header.frame_id, _world_frame_, scan->header.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: insertLaserScanCallback(): could not find tf from %s to %s", scan->header.frame_id.c_str(), _world_frame_.c_str());
    return;
  }

  pcl_ros::transformAsMatrix(res.value().transform, sensorToWorld);

  // laser scan to point cloud
  sensor_msgs::PointCloud2 ros_cloud;
  projector_.projectLaser(*scan, ros_cloud);
  pcl::fromROSMsg(ros_cloud, *pc);

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    sensor_msgs::LaserScan free_scan = *scan;

    double free_scan_distance = (scan->range_max - 1.0) < _unknown_rays_distance_ ? (scan->range_max - 1.0) : _unknown_rays_distance_;

    for (int i = 0; i < scan->ranges.size(); i++) {
      if (scan->ranges[i] > scan->range_max || scan->ranges[i] < scan->range_min) {
        free_scan.ranges[i] = scan->range_max - 1.0;  // valid under max range
      } else {
        free_scan.ranges[i] = scan->range_min - 1.0;  // definitely invalid
      }
    }

    sensor_msgs::PointCloud2 free_cloud;
    projector_.projectLaser(free_scan, free_cloud);

    pcl::fromROSMsg(free_cloud, *free_vectors_pc);
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  pc->header.frame_id              = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);
}

//}

/* callback3dLidarCloud2() //{ */

void OctomapServer::callback3dLidarCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp, const SensorType_t sensor_type, const int sensor_id) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  if (!_map_while_grounded_) {

    if (!sh_control_manager_diag_.hasMsg()) {

      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, can not integrate data!");
      return;

    } else {

      ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

      if ((ros::Time::now() - last_time).toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, can not integrate data!");
        return;
      }

      // TODO is this the best option?
      if (!sh_control_manager_diag_.getMsg()->flying_normally) {
        ROS_INFO_THROTTLE(1.0, "[OctomapServer]: not flying normally, therefore, not integrating data");
        return;
      }
    }
  }

  sensor_msgs::PointCloud2ConstPtr cloud = wrp.getMsg();

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr hit_pc          = boost::make_shared<PCLPointCloud>();

  pcl::fromROSMsg(*cloud, *pc);

  auto res = transformer_->getTransform(cloud->header.frame_id, _world_frame_, cloud->header.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: callback3dLidarCloud2(): could not find tf from %s to %s", cloud->header.frame_id.c_str(), _world_frame_.c_str());
    return;
  }

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf = res.value();
  pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);

  double max_range;

  switch (sensor_type) {

    case LIDAR_3D: {

      max_range = sensor_params_3d_lidar_[sensor_id].max_range;

      break;
    }

    case DEPTH_CAMERA: {

      max_range = sensor_params_depth_cam_[sensor_id].max_range;

      break;
    }

    default: {

      break;
    }
  }

  for (int i = 0; i < pc->size(); i++) {

    pcl::PointXYZ pt = pc->at(i);

    if ((std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) && ((pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2)) < pow(max_range, 2))) {

      hit_pc->push_back(pt);

    } else {

      if (_unknown_rays_update_free_space_) {

        vec3_t ray_vec;

        switch (sensor_type) {

          case LIDAR_3D: {

            ray_vec = sensor_3d_lidar_xyz_lut_[sensor_id].directions.col(i);

            break;
          }

          case DEPTH_CAMERA: {

            ray_vec = sensor_depth_camera_xyz_lut_[sensor_id].directions.col(i);

            break;
          }

          default: {

            break;
          }
        }

        if (ray_vec(2) > 0.0) {

          pcl::PointXYZ temp_pt;

          temp_pt.x = ray_vec(0) * float(max_range);
          temp_pt.y = ray_vec(1) * float(max_range);
          temp_pt.z = ray_vec(2) * float(max_range);

          free_vectors_pc->push_back(temp_pt);
        }
      }
    }
  }

  free_vectors_pc->header = pc->header;

  // Voxelize data
  /* if (hit_pc->size() > 0) { */

  /*   ROS_INFO("[OctomapServer]: size %d", hit_pc->size()); */

  /*   for (int i = 0; i < hit_pc->size(); i++) { */

  /*     const pcl::PointXYZ pt = hit_pc->at(i); */
  /*     const double        x  = pt.x; */
  /*     const double        y  = pt.y; */
  /*     const double        z  = pt.z; */

  /*     if (!std::isfinite(x)) { */
  /*       ROS_ERROR("NaN detected in variable \"x\"!!!"); */
  /*     } */
  /*   } */

  /*   pcl::VoxelGrid<PCLPoint> vg; */
  /*   vg.setInputCloud(hit_pc); */
  /*   vg.setLeafSize(0.2, 0.2, 0.2); */
  /*   PCLPointCloud::Ptr temp_pc; */
  /*   ROS_INFO("[OctomapServer]: voxel grid in"); */
  /*   vg.filter(*hit_pc); */
  /*   ROS_INFO("[OctomapServer]: voxel grid out"); */
  /* } */

  /* if (free_vectors_pc->size() > 0) { */
  /*   pcl::VoxelGrid<PCLPoint> vg; */
  /*   vg.setInputCloud(free_vectors_pc); */
  /*   vg.setLeafSize(1.0, 1.0, 1.0); */
  /*   vg.filter(*free_vectors_pc); */
  /* } */

  /* // filter lone pixels */
  /* if (hit_pc->size() > 0) { */
  /*   pcl::StatisticalOutlierRemoval<PCLPoint> sor(true); */
  /*   sor.setInputCloud(hit_pc); */
  /*   sor.setMeanK(50.0); */
  /*   sor.setStddevMulThresh(1.0); */
  /*   sor.filter(*hit_pc); */
  /* } */

  // transform to the map frame

  pcl::transformPointCloud(*hit_pc, *hit_pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  hit_pc->header.frame_id          = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, hit_pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef               = 0.5;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}  // namespace mrs_octomap_server

//}

// | -------------------- service callbacks ------------------- |

/* callbackLoadMap() //{ */

bool OctomapServer::callbackLoadMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[OctomapServer]: loading map");

  bool success = loadFromFile(req.value);

  if (success) {

    if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
      octrees_initialized_ = false;

      timer_altitude_alignment_.start();
    }

    res.success = true;
    res.message = "map loaded";

  } else {

    res.success = false;
    res.message = "map loading error";
  }

  return true;
}

//}

/* callbackSaveMap() //{ */

bool OctomapServer::callbackSaveMap([[maybe_unused]] mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  bool success = saveToFile(req.value);

  if (success) {

    res.message = "map saved";
    res.success = true;

  } else {

    res.message = "map saving failed";
    res.success = false;
  }

  return true;
}

//}

/* callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] std_srvs::Empty::Request& req, [[maybe_unused]] std_srvs::Empty::Response& resp) {

  {
    std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

    octree_global_->clear();
    octree_local_->clear();
  }

  octrees_initialized_ = true;

  ROS_INFO("[OctomapServer]: octomap cleared");

  return true;
}

//}

/* callbackSetLocalFractor() //{ */

bool OctomapServer::callbackSetGlobalFractor([[maybe_unused]] mrs_msgs::SetInt::Request& req, [[maybe_unused]] mrs_msgs::SetInt::Response& resp) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (req.value < 0) {

    ss << "fractor < 0 does not make sense!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[OctomapServer]: " << ss.str());

    resp.message = ss.str();
    resp.success = false;

    return true;
  }

  {
    std::scoped_lock lock(mutex_octree_global_, mutex_resolution_fractor_);

    int old_fractor = global_resolution_fractor_;

    global_resolution_fractor_ = req.value;

    if (old_fractor < global_resolution_fractor_) {

      for (OcTree_t::leaf_iterator it = octree_global_->begin_leafs(octree_global_->getTreeDepth() - req.value), end = octree_global_->end_leafs(); it != end;
           ++it) {

        auto orig_key = it.getKey();

        const unsigned int old_depth = it.getDepth();

        octomap::OcTreeNode* orig_node = it.getNode();

        octree_global_->eatChildren(orig_node);
      }
    }
  }

  resp.message = "fractor set";
  resp.success = true;

  return true;
}

//}

/* callbackSetLocalFractor() //{ */

bool OctomapServer::callbackSetLocalFractor([[maybe_unused]] mrs_msgs::SetInt::Request& req, [[maybe_unused]] mrs_msgs::SetInt::Response& resp) {

  if (!is_initialized_) {
    return false;
  }

  std::stringstream ss;

  if (req.value < 0) {

    ss << "fractor < 0 does not make sense!";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[OctomapServer]: " << ss.str());

    resp.message = ss.str();
    resp.success = false;

    return true;
  }

  {
    std::scoped_lock lock(mutex_octree_local_, mutex_resolution_fractor_);

    int old_fractor = local_resolution_fractor_;

    local_resolution_fractor_ = req.value;

    if (old_fractor < local_resolution_fractor_) {

      for (OcTree_t::leaf_iterator it = octree_local_->begin_leafs(octree_local_->getTreeDepth() - req.value), end = octree_local_->end_leafs(); it != end;
           ++it) {

        auto orig_key = it.getKey();

        const unsigned int old_depth = it.getDepth();

        octomap::OcTreeNode* orig_node = it.getNode();

        octree_local_->eatChildren(orig_node);
      }
    }
  }

  resp.message = "fractor set";
  resp.success = true;

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMapPublisher() //{ */

void OctomapServer::timerGlobalMapPublisher([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: full map publisher timer spinning");

  size_t octomap_size;

  {
    std::scoped_lock lock(mutex_octree_global_);

    octree_global_->size();
  }

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  /* if (_global_map_compress_) { */
  /*   octree_global_->prune(); */
  /* } */

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_global_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::globalMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::fullMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_global_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::globalMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::binaryMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerGlobalMapCreator() //{ */

void OctomapServer::timerGlobalMapCreator([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::timerGlobalMapCreator", scope_timer_logger_, _scope_timer_enabled_);

  ROS_INFO_ONCE("[OctomapServer]: global map creator timer spinning");

  // copy the local map into a buffer

  std::shared_ptr<OcTree_t> local_map_tmp_;
  {
    std::scoped_lock lock(mutex_octree_local_);

    local_map_tmp_ = std::make_shared<OcTree_t>(*octree_local_);
  }

  {
    std::scoped_lock lock(mutex_octree_global_);

    copyLocalMap(local_map_tmp_, local_resolution_fractor_, octree_global_, global_resolution_fractor_);
  }
}

//}

/* timerLocalMapPublisher() //{ */

void OctomapServer::timerLocalMapPublisher([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map publisher timer spinning");

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree_local_, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  if (_local_map_publish_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_local_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::fullMapToMsg(*octree_local_, map);
    }

    if (success) {
      pub_map_local_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to full representation");
    }
  }

  if (_local_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_local_);

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::binaryMapToMsg(*octree_local_, map);
    }

    if (success) {
      pub_map_local_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to binary representation");
    }
  }
}

//}

/* timerLocalMapResizer() //{ */

void OctomapServer::timerLocalMapResizer([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map resizer timer spinning");

  auto local_map_duty = mrs_lib::get_mutexed(mutex_local_map_duty_, local_map_duty_);

  ROS_INFO("[OctomapServer]: local map duty time: %.3f s", local_map_duty);

  {
    std::scoped_lock lock(mutex_local_map_dimensions_);

    /* if (local_map_duty > 0.9) { */
    /*   local_map_width_ -= int(ceil(10.0 * (local_map_duty - 0.9))); */
    /*   local_map_height_ -= int(ceil(10.0 * (local_map_duty - 0.9))); */
    /* } else if (local_map_duty < 0.8) { */
    /*   local_map_width_++; */
    /*   local_map_height_++; */
    /* } */

    /* if (local_map_width_ < 5) { */
    /*   local_map_width_ = 5; */
    /* } else if (local_map_width_ > _local_map_width_) { */
    /*   local_map_width_ = _local_map_width_; */
    /* } */

    /* if (local_map_height_ < 5) { */
    /*   local_map_height_ = 5; */
    /* } else if (local_map_height_ > _local_map_height_) { */
    /*   local_map_height_ = _local_map_height_; */
    /* } */

    local_map_duty = 0;
  }

  ROS_INFO("[OctomapServer]: local map size %d %d", local_map_width_, local_map_height_);

  mrs_lib::set_mutexed(mutex_local_map_duty_, local_map_duty, local_map_duty_);
}

//}

/* timerPersistency() //{ */

void OctomapServer::timerPersistency([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: persistency timer spinning");

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, won't save the map automatically!");
    return;

  } else {

    ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

    if ((ros::Time::now() - last_time).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, won't save the map automatically!");
      return;
    }
  }

  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  if (control_manager_diag->flying_normally) {

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: saving the map");

    bool success = saveToFile(_persistency_map_name_);

    if (success) {
      ROS_INFO("[OctomapServer]: persistent map saved");
    } else {
      ROS_ERROR("[OctomapServer]: failed to saved persistent map");
    }
  }
}

//}

/* timerAltitudeAlignment() //{ */

void OctomapServer::timerAltitudeAlignment([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: altitude alignment timer spinning");

  // | ---------- check for control manager diagnostics --------- |

  if (!sh_control_manager_diag_.hasMsg()) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: missing control manager diagnostics, won't save the map automatically!");
    return;

  } else {

    ros::Time last_time = sh_control_manager_diag_.lastMsgTime();

    if ((ros::Time::now() - last_time).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "[OctomapServer]: control manager diagnostics too old, won't save the map automatically!");
      return;
    }
  }

  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  // | -------------------- check for height -------------------- |

  bool got_height = false;

  if (sh_height_.hasMsg()) {

    ros::Time last_time = sh_height_.lastMsgTime();

    if ((ros::Time::now() - last_time).toSec() < 1.0) {
      got_height = true;
    }
  }

  // | -------------------- do the alignment -------------------- |

  bool align_using_height = false;

  if (control_manager_diag->motors) {

    if (!got_height) {

      ROS_INFO("[OctomapServer]: already in the air while missing height data, skipping alignment and clearing the map");

      {
        std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

        octree_global_->clear();
        octree_local_->clear();

        octrees_initialized_ = true;
      }

      timer_altitude_alignment_.stop();

      ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    } else {
      align_using_height = true;
    }

  } else {

    align_using_height = false;
  }

  // | ------ get the current UAV position in the map frame ----- |

  auto res = transformer_->getTransform(_robot_frame_, _world_frame_);

  double robot_x, robot_y, robot_z;

  if (res) {

    geometry_msgs::TransformStamped world_to_robot = res.value();

    robot_x = world_to_robot.transform.translation.x;
    robot_y = world_to_robot.transform.translation.y;
    robot_z = world_to_robot.transform.translation.z;

    ROS_INFO("[OctomapServer]: robot coordinates %.2f, %.2f, %.2f", robot_x, robot_y, robot_z);

  } else {

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: waiting for the tf from %s to %s", _world_frame_.c_str(), _robot_frame_.c_str());
    return;
  }

  auto ground_z = getGroundZ(octree_global_, robot_x, robot_y);

  if (!ground_z) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: could not calculate the Z of the ground below");

    {
      std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

      octree_global_->clear();
      octree_local_->clear();

      octrees_initialized_ = true;
    }

    timer_altitude_alignment_.stop();

    ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    return;
  }

  double ground_z_should_be = 0;

  if (align_using_height) {
    ground_z_should_be = robot_z - sh_height_.getMsg()->value;
  } else {
    ground_z_should_be = robot_z - _robot_height_ - 0.5 * octree_global_->getResolution();
  }

  double offset = ground_z_should_be - ground_z.value();

  ROS_INFO("[OctomapServer]: ground is at height %.2f m", ground_z.value());
  ROS_INFO("[OctomapServer]: ground should be at height %.2f m", ground_z_should_be);
  ROS_INFO("[OctomapServer]: shifting ground by %.2f m", offset);

  translateMap(octree_global_, 0, 0, offset);
  translateMap(octree_local_, 0, 0, offset);

  octrees_initialized_ = true;

  timer_altitude_alignment_.stop();
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::timerInsertPointCloud", scope_timer_logger_, _scope_timer_enabled_);

  ros::Time time_start = ros::Time::now();

  std::scoped_lock lock(mutex_octree_local_);

  auto [local_map_width, local_map_height] = mrs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

  auto resolution_fractor = mrs_lib::get_mutexed(mutex_resolution_fractor_, local_resolution_fractor_);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorOriginTf);

  const float free_space_ray_len = std::min(float(_unknown_rays_distance_), float(sqrt(2 * pow(local_map_width / 2.0, 2) + pow(local_map_height / 2.0, 2))));

  double coarse_res = octree_local_->getResolution() * pow(2.0, resolution_fractor);

  ROS_INFO_THROTTLE(1.0, "[OctomapServer]: current resolution = %.2f m", coarse_res);

  octomap::KeySet occupied_cells;
  octomap::KeySet free_cells;
  octomap::KeySet free_ends;

  // all measured points: make it free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::OcTreeKey key;
    if (octree_local_->coordToKeyChecked(measured_point, key)) {
      occupied_cells.insert(key);
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    octomap::OcTreeKey measured_key = octree_local_->coordToKey(measured_point);

    free_ends.insert(measured_key);
  }

  // FREE VECTORS
  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::KeyRay keyRay;

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    // check if the ray intersects a cell in the occupied list
    if (octree_local_->computeRayKeys(sensor_origin, measured_point, keyRay, resolution_fractor)) {

      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

        if (!_unknown_rays_clear_occupied_) {

          // check if the cell is occupied in the map
          auto node = octree_local_->search(*it2);

          if (node && octree_local_->isNodeOccupied(node)) {

            if (it2 == keyRay.begin()) {
              alterantive_ray_end = keyRay.begin();  // special case
            } else {
              alterantive_ray_end = it2 - 1;
            }

            break;
          }
        }
      }

      free_cells.insert(keyRay.begin(), alterantive_ray_end);
    }
  }

  // for FREE RAY ENDS
  for (octomap::KeySet::iterator it = free_ends.begin(), end = free_ends.end(); it != end; ++it) {

    octomap::point3d coords = octree_local_->keyToCoord(*it);

    octomap::KeyRay key_ray;
    if (octree_local_->computeRayKeys(sensor_origin, coords, key_ray, resolution_fractor)) {

      for (octomap::KeyRay::iterator it2 = key_ray.begin(), end = key_ray.end(); it2 != end; ++it2) {

        if (occupied_cells.count(*it2)) {

          octomap::KeyRay::iterator last_key = it2 != key_ray.begin() ? it2 - 1 : key_ray.begin();

          free_cells.insert(key_ray.begin(), last_key);
          break;

        } else {
          free_cells.insert(key_ray.begin(), key_ray.end());
        }
      }
    }
  }

  octomap::OcTreeNode* root = octree_local_->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = octree_local_->coordToKey(0, 0, 0, octree_local_->getTreeDepth());
    octree_local_->setNodeValue(key, 0.0);
  }

  // FREE CELLS
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {

    octree_local_->updateNodeDepth(*it, octree_local_->getProbMissLog(), int(octree_local_->getTreeDepth()) - resolution_fractor);
  }

  // OCCUPIED CELLS
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {

    octree_local_->updateNodeDepth(*it, octree_local_->getProbHitLog(), int(octree_local_->getTreeDepth()) - resolution_fractor);
  }

  // CROP THE MAP AROUND THE ROBOT
  {

    mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("OctomapServer::localMapCopy", scope_timer_logger_, _scope_timer_enabled_);

    auto [local_map_width, local_map_height] = mrs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

    float x        = sensor_origin.x();
    float y        = sensor_origin.y();
    float z        = sensor_origin.z();
    float width_2  = local_map_width / float(2.0);
    float height_2 = local_map_height / float(2.0);

    octomap::point3d roi_min(x - width_2, y - width_2, z - height_2);
    octomap::point3d roi_max(x + width_2, y + width_2, z + height_2);

    std::shared_ptr<OcTree_t> from;
    std::shared_ptr<OcTree_t> to;

    if (octree_local_idx_ == 0) {
      from              = octree_local_0_;
      octree_local_     = octree_local_1_;
      octree_local_idx_ = 1;
    } else {
      from              = octree_local_1_;
      octree_local_     = octree_local_0_;
      octree_local_idx_ = 0;
    }

    octree_local_->clear();

    copyInsideBBX2(from, local_resolution_fractor_, octree_local_, local_resolution_fractor_, roi_min, roi_max);
  }

  ros::Time time_end = ros::Time::now();

  {
    std::scoped_lock lock(mutex_local_map_duty_);

    local_map_duty_ += (time_end - time_start).toSec();
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params) {

  const int                                       rangeCount         = sensor_params.horizontal_rays;
  const int                                       verticalRangeCount = sensor_params.vertical_rays;
  std::vector<std::tuple<double, double, double>> coord_coeffs;
  const double                                    minAngle = 0.0;
  const double                                    maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = sensor_params.vertical_fov / 2.0;

  const double yDiff = maxAngle - minAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  double yAngle_step = yDiff / (rangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1)
    pAngle_step = pDiff / (verticalRangeCount - 1);
  else
    pAngle_step = 0;

  coord_coeffs.reserve(rangeCount * verticalRangeCount);

  for (int i = 0; i < rangeCount; i++) {
    for (int j = 0; j < verticalRangeCount; j++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + minAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);
      coord_coeffs.push_back({x_coeff, y_coeff, z_coeff});
    }
  }

  int it = 0;
  lut.directions.resize(3, rangeCount * verticalRangeCount);
  lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}  // namespace mrs_octomap_server

//}

/* initializeDepthCamLUT() //{ */

void OctomapServer::initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params) {

  const int horizontalRangeCount = sensor_params.horizontal_rays;
  const int verticalRangeCount   = sensor_params.vertical_rays;

  ROS_INFO("[OctomapServer]: initializing depth camera lut, res %d x %d = %d points", horizontalRangeCount, verticalRangeCount,
           horizontalRangeCount * verticalRangeCount);

  std::vector<std::tuple<double, double, double>> coord_coeffs;

  // yes it's flipped, pixel [0,0] is top-left
  const double horizontalMinAngle = sensor_params.horizontal_fov / 2.0;
  const double horizontalMaxAngle = -sensor_params.horizontal_fov / 2.0;

  const double verticalMinAngle = sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = -sensor_params.vertical_fov / 2.0;

  const double yDiff = horizontalMaxAngle - horizontalMinAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  Eigen::Quaterniond rot = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());

  double yAngle_step = yDiff / (horizontalRangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1) {
    pAngle_step = pDiff / (verticalRangeCount - 1);
  } else {
    pAngle_step = 0;
  }

  coord_coeffs.reserve(horizontalRangeCount * verticalRangeCount);

  for (int j = 0; j < verticalRangeCount; j++) {
    for (int i = 0; i < horizontalRangeCount; i++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + horizontalMinAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);

      Eigen::Vector3d p(x_coeff, y_coeff, z_coeff);

      p = rot * p;

      double r = (double)(i) / horizontalRangeCount;
      double g = (double)(j) / horizontalRangeCount;

      coord_coeffs.push_back({p.x(), p.y(), p.z()});
    }
  }

  int it = 0;
  lut.directions.resize(3, horizontalRangeCount * verticalRangeCount);
  lut.offsets.resize(3, horizontalRangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < horizontalRangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col + horizontalRangeCount * row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* loadFromFile() //{ */

bool OctomapServer::loadFromFile(const std::string& filename) {

  std::string file_path = _map_path_ + "/" + filename + ".ot";

  {
    std::scoped_lock lock(mutex_octree_global_);

    if (file_path.length() <= 3)
      return false;

    std::string suffix = file_path.substr(file_path.length() - 3, 3);

    if (suffix == ".bt") {
      if (!octree_global_->readBinary(file_path)) {
        return false;
      }
    } else if (suffix == ".ot") {

      auto tree = octomap::AbstractOcTree::read(file_path);
      if (!tree) {
        return false;
      }

      OcTree_t* octree = dynamic_cast<OcTree_t*>(tree);
      octree_global_   = std::shared_ptr<OcTree_t>(octree);

      if (!octree_global_) {
        ROS_ERROR("[OctomapServer]: could not read OcTree file");
        return false;
      }

    } else {
      return false;
    }

    octree_resolution_ = octree_global_->getResolution();
  }

  return true;
}

//}

/* saveToFile() //{ */

bool OctomapServer::saveToFile(const std::string& filename) {

  std::scoped_lock lock(mutex_octree_global_);

  std::string file_path        = _map_path_ + "/" + filename + ".ot";
  std::string tmp_file_path    = _map_path_ + "/tmp_" + filename + ".ot";
  std::string backup_file_path = _map_path_ + "/" + filename + "_backup.ot";

  try {
    std::filesystem::rename(file_path, backup_file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapServer]: failed to copy map to the backup path");
  }

  std::string suffix = file_path.substr(file_path.length() - 3, 3);

  if (!octree_global_->write(tmp_file_path)) {
    ROS_ERROR("[OctomapServer]: error writing to file '%s'", file_path.c_str());
    return false;
  }

  try {
    std::filesystem::rename(tmp_file_path, file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapServer]: failed to copy map to the backup path");
  }

  return true;
}

//}

/* copyInsideBBX2() //{ */

bool OctomapServer::copyInsideBBX2(std::shared_ptr<OcTree_t>& from, const int& from_fractor, std::shared_ptr<OcTree_t>& to, const int& to_fractor,
                                   const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->setNodeValue(key, 1.0);
  }

  // iterate over leafs of the original "from" tree (up to the desired fractor depth)
  for (OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max, from->getTreeDepth() - from_fractor), end = from->end_leafs_bbx(); it != end;
       ++it) {

    octomap::OcTreeNode* orig_node = it.getNode();

    from->eatChildren(orig_node);

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(orig_node->getValue());
  }

  // update the region the the fractor of the global map
  if (to_fractor > from_fractor) {

    // iterate over leafs of the original "from" tree (up to the desired fractor depth)
    for (OcTree_t::leaf_bbx_iterator it = to->begin_leafs_bbx(p_min, p_max, to->getTreeDepth() - to_fractor), end = to->end_leafs_bbx(); it != end; ++it) {

      octomap::OcTreeNode* orig_node = it.getNode();

      to->eatChildren(orig_node);
    }
  }

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->deleteNode(key, to->getTreeDepth());
  }

  return true;
}

//}

/* copyLocalMap() //{ */

bool OctomapServer::copyLocalMap(std::shared_ptr<OcTree_t>& from, const int& from_fractor, std::shared_ptr<OcTree_t>& to, const int& to_fractor) {

  octomap::OcTreeKey minKey, maxKey;

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, 1.0);
  }

  // iterate over leafs of the original "from" tree (up to the desired fractor depth)
  for (OcTree_t::leaf_iterator it = from->begin_leafs(from->getTreeDepth() - from_fractor), end = from->end_leafs(); it != end; ++it) {

    octomap::OcTreeNode* orig_node = it.getNode();

    from->eatChildren(orig_node);

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(orig_node->getValue());
  }

  // update the region the the fractor of the global map
  if (to_fractor > from_fractor) {

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    from->getMetricMin(min_x, min_y, min_z);
    from->getMetricMax(max_x, max_y, max_z);

    octomap::point3d p_min(min_x, min_y, min_z);
    octomap::point3d p_max(max_x, max_y, max_z);

    // iterate over leafs of the original "from" tree (up to the desired fractor depth)
    for (OcTree_t::leaf_bbx_iterator it = to->begin_leafs_bbx(p_min, p_max, to->getTreeDepth() - to_fractor), end = to->end_leafs_bbx(); it != end; ++it) {

      octomap::OcTreeNode* orig_node = it.getNode();

      to->eatChildren(orig_node);
    }
  }

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->deleteNode(key, to->getTreeDepth());
  }

  return true;
}

//}

/* touchNode() //{ */

octomap::OcTreeNode* OctomapServer::touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0) {

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode* OctomapServer::touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                                    unsigned int depth, unsigned int max_depth = 0) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {

    octree->eatChildren(node);

    return node;
  }
}

//}

/* expandNodeRecursive() //{ */

void OctomapServer::expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

  if (node_depth < octree->getTreeDepth()) {

    octree->expandNode(node);

    for (int i = 0; i < 8; i++) {
      auto child = octree->getNodeChild(node, i);

      expandNodeRecursive(octree, child, node_depth + 1);
    }

  } else {
    return;
  }
}

//}

/* getGroundZ() //{ */

std::optional<double> OctomapServer::getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y) {

  octomap::point3d p_min(float(x - _persistency_align_altitude_distance_), float(y - _persistency_align_altitude_distance_), -10000);
  octomap::point3d p_max(float(x + _persistency_align_altitude_distance_), float(y + _persistency_align_altitude_distance_), 10000);

  for (OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  std::vector<octomap::point3d> occupied_points;

  for (OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    if (octree->isNodeOccupied(*it)) {

      occupied_points.push_back(it.getCoordinate());
    }
  }

  if (occupied_points.size() < 3) {

    ROS_ERROR("[OctomapServer]: low number of points for ground z calculation");
    return {};

  } else {

    double max_z = std::numeric_limits<double>::lowest();

    for (int i = 0; i < occupied_points.size(); i++) {
      if (occupied_points[i].z() > max_z) {
        max_z = occupied_points[i].z() - (octree_resolution_ / 2.0);
      }
    }

    /* for (int i = 0; i < occupied_points.size(); i++) { */
    /*   z += occupied_points[i].z(); */
    /* } */
    /* z /= occupied_points.size(); */

    return {max_z};
  }
}

//}

/* translateMap() //{ */

bool OctomapServer::translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z) {

  ROS_INFO("[OctomapServer]: translating map by %.2f, %.2f, %.2f", x, y, z);

  octree->expand();

  // allocate the new future octree
  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(octree_resolution_);
  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMin());
  octree_new->setClampingThresMax(octree->getClampingThresMax());

  for (OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    auto coords = it.getCoordinate();

    coords.x() += float(x);
    coords.y() += float(y);
    coords.z() += float(z);

    auto value = it->getValue();
    auto key   = it.getKey();

    auto new_key = octree_new->coordToKey(coords);

    octree_new->setNodeValue(new_key, value);
  }

  octree_new->prune();

  octree = octree_new;

  ROS_INFO("[OctomapServer]: map translated");

  return true;
}

//}

}  // namespace mrs_octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_server::OctomapServer, nodelet::Nodelet)
