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

#include <mrs_msgs/String.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>

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

//}

/* class OctomapServer //{ */

class OctomapServer : public nodelet::Nodelet {

public:
#ifdef COLOR_OCTOMAP_SERVER
  using PCLPoint      = pcl::PointXYZRGB;
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;
  using OcTreeT       = octomap::ColorOcTree;
#else
  using PCLPoint      = pcl::PointXYZ;
  using PCLPointCloud = pcl::PointCloud<PCLPoint>;
  using OcTreeT       = octomap::OcTree;
#endif

  virtual void onInit();

  bool callbackLoadMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);
  bool callbackSaveMap(mrs_msgs::String::Request& req, [[maybe_unused]] mrs_msgs::String::Response& resp);

  bool callbackResetMap(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  void callbackPointCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp);
  void callbackLaserScan(mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>& wrp);
  bool loadFromFile(const std::string& filename);
  bool saveToFile(const std::string& filename);

private:
  ros::NodeHandle nh_;
  bool            is_initialized_;

  // | -------------------- topic subscribers ------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::Float64Stamped>            sh_height_;
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>            sh_point_cloud_2_;
  mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>              sh_laser_scan_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_map_global_full_;
  ros::Publisher pub_map_global_binary_;

  ros::Publisher pub_map_local_full_;
  ros::Publisher pub_map_local_binary_;

  // | -------------------- service serviers -------------------- |

  ros::ServiceServer ss_reset_map_;
  ros::ServiceServer ss_save_map_;
  ros::ServiceServer ss_load_map_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_global_map_;
  double     _global_map_rate_;
  void       timerGlobalMap([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_local_map_;
  void       timerLocalMap([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_persistency_;
  void       timerPersistency([[maybe_unused]] const ros::TimerEvent& event);

  ros::Timer timer_altitude_alignment_;
  void       timerAltitudeAlignment([[maybe_unused]] const ros::TimerEvent& event);

  // | ----------------------- parameters ----------------------- |

  bool        _simulation_;
  std::string _uav_name_;

  double _robot_height_;

  bool        _persistency_enabled_;
  std::string _persistency_map_name_;
  double      _persistency_save_time_;

  bool   _persistency_align_altitude_enabled_;
  double _persistency_align_altitude_distance_;

  bool _global_map_publish_full_;
  bool _global_map_publish_binary_;

  double _local_map_size_;
  bool   _local_map_enabled_;
  bool   _local_map_publish_full_;
  bool   _local_map_publish_binary_;

  mrs_lib::Transformer transformer_;

  std::shared_ptr<OcTreeT> octree_;
  std::mutex               mutex_octree_;
  std::atomic<bool>        octree_initialized_;

  std::shared_ptr<OcTreeT> octree_local_;
  std::mutex               mutex_octree_local_;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  double     avg_time_local_map_processing_ = 0;
  std::mutex mutex_avg_time_local_map_processing_;

  octomap::KeyRay    m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  double      m_maxRange;
  std::string _world_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  unsigned    m_treeDepth;
  unsigned    m_maxTreeDepth;
  bool        _global_map_compress_;
  std::string _map_path_;

  double _local_map_horizontal_distance_;
  double _local_map_vertical_distance_;
  double _local_map_rate_;
  double _local_map_max_computation_duty_cycle_;

  double local_map_horizontal_offset_ = 0;
  double local_map_vertical_offset_   = 0;

  bool   _unknown_rays_update_free_space_;
  double _unknown_rays_distance_;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  void expandNodeRecursive(std::shared_ptr<OcTreeT>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTreeT>& octree, const double& x, const double& y);

  bool translateMap(std::shared_ptr<OcTreeT>& octree, const double& x, const double& y, const double& z);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTreeT>& octree);

  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
    for (unsigned i = 0; i < 3; ++i)
      min[i] = std::min(in[i], min[i]);
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
    for (unsigned i = 0; i < 3; ++i)
      max[i] = std::max(in[i], max[i]);
  };

  virtual void insertPointCloud(const geometry_msgs::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud, const PCLPointCloud::ConstPtr& free_cloud);

  void initializeLidarLUT(const size_t w, const size_t h);

  xyz_lut_t m_sensor_3d_xyz_lut;
  bool      m_sensor_3d_params_enabled;
  float     m_sensor_3d_vfov;
  int       m_sensor_3d_vrays;
  int       m_sensor_3d_hrays;

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

  param_loader.loadParam("persistency/enabled", _persistency_enabled_);
  param_loader.loadParam("persistency/save_time", _persistency_save_time_);
  param_loader.loadParam("persistency/map_name", _persistency_map_name_);
  param_loader.loadParam("persistency/align_altitude/enabled", _persistency_align_altitude_enabled_);
  param_loader.loadParam("persistency/align_altitude/ground_detection_distance", _persistency_align_altitude_distance_);
  param_loader.loadParam("persistency/align_altitude/robot_height", _robot_height_);

  param_loader.loadParam("global_map/rate", _global_map_rate_);
  param_loader.loadParam("global_map/compress", _global_map_compress_);
  param_loader.loadParam("global_map/publish_full", _global_map_publish_full_);
  param_loader.loadParam("global_map/publish_binary", _global_map_publish_binary_);

  param_loader.loadParam("local_map/enabled", _local_map_enabled_);
  param_loader.loadParam("local_map/horizontal_distance", _local_map_horizontal_distance_);
  param_loader.loadParam("local_map/vertical_distance", _local_map_vertical_distance_);
  param_loader.loadParam("local_map/rate", _local_map_rate_);
  param_loader.loadParam("local_map/max_computation_duty_cycle", _local_map_max_computation_duty_cycle_);
  param_loader.loadParam("local_map/publish_full", _local_map_publish_full_);
  param_loader.loadParam("local_map/publish_binary", _local_map_publish_binary_);

  param_loader.loadParam("resolution", octree_resolution_);
  param_loader.loadParam("world_frame_id", _world_frame_);
  param_loader.loadParam("robot_frame_id", _robot_frame_);

  param_loader.loadParam("map_path", _map_path_);

  param_loader.loadParam("unknown_rays/update_free_space", _unknown_rays_update_free_space_);
  param_loader.loadParam("unknown_rays/ray_distance", _unknown_rays_distance_);

  param_loader.loadParam("sensor_params_3d/enabled", m_sensor_3d_params_enabled);
  param_loader.loadParam("sensor_params_3d/vertical_fov_angle", m_sensor_3d_vfov);
  param_loader.loadParam("sensor_params_3d/vertical_rays", m_sensor_3d_vrays);
  param_loader.loadParam("sensor_params_3d/horizontal_rays", m_sensor_3d_hrays);

  param_loader.loadParam("sensor_model/hit", _probHit_);
  param_loader.loadParam("sensor_model/miss", _probMiss_);
  param_loader.loadParam("sensor_model/min", _thresMin_);
  param_loader.loadParam("sensor_model/max", _thresMax_);
  param_loader.loadParam("sensor_model/max_range", m_maxRange);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: Could not load all non-optional parameters. Shutting down.", ros::this_node::getName().c_str());
    ros::requestShutdown();
  }

  //}

  /* initialize sensor LUT model //{ */

  if (m_sensor_3d_params_enabled) {

    initializeLidarLUT(m_sensor_3d_hrays, m_sensor_3d_vrays);
  }

  //}

  /* initialize octomap object & params //{ */

  octree_ = std::make_shared<OcTreeT>(octree_resolution_);
  octree_->setProbHit(_probHit_);
  octree_->setProbMiss(_probMiss_);
  octree_->setClampingThresMin(_thresMin_);
  octree_->setClampingThresMax(_thresMax_);

  octree_local_ = std::make_shared<OcTreeT>(octree_resolution_);
  octree_local_->setProbHit(_probHit_);
  octree_local_->setProbMiss(_probMiss_);
  octree_local_->setClampingThresMin(_thresMin_);
  octree_local_->setClampingThresMax(_thresMax_);

  m_treeDepth    = octree_->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

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
    octree_initialized_ = false;
  } else {
    octree_initialized_ = true;
  }

  //}

  /* transformer //{ */

  transformer_ = mrs_lib::Transformer("OctomapServer", _uav_name_);

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
  sh_point_cloud_2_        = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "point_cloud_in", &OctomapServer::callbackPointCloud2, this);
  sh_laser_scan_           = mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>(shopts, "laser_scan_in", &OctomapServer::callbackLaserScan, this);

  //}

  /* service servers //{ */

  ss_reset_map_ = nh_.advertiseService("reset_map_in", &OctomapServer::callbackResetMap, this);
  ss_save_map_  = nh_.advertiseService("save_map_in", &OctomapServer::callbackSaveMap, this);
  ss_load_map_  = nh_.advertiseService("load_map_in", &OctomapServer::callbackLoadMap, this);

  //}

  /* timers //{ */

  timer_global_map_ = nh_.createTimer(ros::Rate(_global_map_rate_), &OctomapServer::timerGlobalMap, this);

  if (_local_map_enabled_) {
    timer_local_map_ = nh_.createTimer(ros::Rate(_local_map_rate_), &OctomapServer::timerLocalMap, this);
  }

  if (_persistency_enabled_) {
    timer_persistency_ = nh_.createTimer(ros::Rate(1.0 / _persistency_save_time_), &OctomapServer::timerPersistency, this);
  }

  if (_persistency_enabled_ && _persistency_align_altitude_enabled_) {
    timer_altitude_alignment_ = nh_.createTimer(ros::Rate(1.0), &OctomapServer::timerAltitudeAlignment, this);
  }

  //}

  avg_time_local_map_processing_ = (1.0 / _local_map_rate_) * _local_map_max_computation_duty_cycle_;

  is_initialized_ = true;

  ROS_INFO("[%s]: Initialized", ros::this_node::getName().c_str());
}

//}

// | --------------------- topic callbacks -------------------- |

/* insertLaserScanCallback() //{ */

void OctomapServer::callbackLaserScan(mrs_lib::SubscribeHandler<sensor_msgs::LaserScan>& wrp) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  sensor_msgs::LaserScanConstPtr scan = wrp.getMsg();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf;

  auto res = transformer_.getTransform(scan->header.frame_id, _world_frame_, scan->header.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: insertLaserScanCallback(): could not find tf from %s to %s", scan->header.frame_id.c_str(), _world_frame_.c_str());
    return;
  }

  pcl_ros::transformAsMatrix(res.value().getTransform().transform, sensorToWorld);

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

/* insertCloudCallback() //{ */

void OctomapServer::callbackPointCloud2(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& wrp) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  sensor_msgs::PointCloud2ConstPtr cloud = wrp.getMsg();

  ros::Time time_start = ros::Time::now();

  PCLPointCloud::Ptr pc              = boost::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = boost::make_shared<PCLPointCloud>();
  pcl::fromROSMsg(*cloud, *pc);

  auto res = transformer_.getTransform(cloud->header.frame_id, _world_frame_, cloud->header.stamp);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: insertCloudScanCallback(): could not find tf from %s to %s", cloud->header.frame_id.c_str(),
                      _world_frame_.c_str());
    return;
  }

  Eigen::Matrix4f                 sensorToWorld;
  geometry_msgs::TransformStamped sensorToWorldTf = res.value().getTransform();
  pcl_ros::transformAsMatrix(sensorToWorldTf.transform, sensorToWorld);

  // compute free rays, if required
  if (_unknown_rays_update_free_space_) {

    /* Eigen::Affine3d s2w = tf2::transformToEigen(sensorToWorldTf); */

    /* const auto tf_rot = s2w.rotation(); */
    /* // origin of all rays of the lidar sensor */
    /* const vec3_t origin_pt = s2w.translation(); */

    // go through all points in the cloud and update voxels in the helper voxelmap that the rays
    // from the sensor origin to the point go through according to how long part of the ray
    // intersects the voxel
    for (int i = 0; i < pc->size(); i++) {

      pcl::PointXYZ pt = pc->at(i);

      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {

        const vec3_t ray_vec = m_sensor_3d_xyz_lut.directions.col(i);

        if (ray_vec(2) > 0.0) {
          pt.x = ray_vec(0) * float(_unknown_rays_distance_);
          pt.y = ray_vec(1) * float(_unknown_rays_distance_);
          pt.z = ray_vec(2) * float(_unknown_rays_distance_);

          free_vectors_pc->push_back(pt);
        }
      }
    }
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  pc->header.frame_id              = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, pc, free_vectors_pc);

  const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorToWorldTf.transform.translation);

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef               = 0.95;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    ROS_INFO_THROTTLE(5.0, "[OctomapServer]: avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}

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
      octree_initialized_ = false;

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
    std::scoped_lock lock(mutex_octree_);

    octree_->clear();

    octree_initialized_ = true;
  }

  ROS_INFO("[OctomapServer]: octomap cleared");

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMap() //{ */

void OctomapServer::timerGlobalMap([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: full map timer spinning");

  std::scoped_lock lock(mutex_octree_);

  size_t octomap_size = octree_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  if (_global_map_compress_) {
    octree_->prune();
  }

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_, map)) {
      pub_map_global_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_, map)) {
      pub_map_global_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerLocalMap() //{ */

void OctomapServer::timerLocalMap([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapServer]: local map timer spinning");

  std::scoped_lock lock(mutex_octree_local_);

  double duty_factor = avg_time_local_map_processing_ / (_local_map_max_computation_duty_cycle_ * (1.0 / _local_map_rate_));

  if (duty_factor >= 1.0) {

    local_map_horizontal_offset_ -= 0.5;
    local_map_vertical_offset_ -= 0.25;

  } else if (duty_factor <= 0.5) {

    local_map_horizontal_offset_ += 0.5;
    local_map_vertical_offset_ += 0.25;

    if (local_map_vertical_offset_ >= 0) {
      local_map_horizontal_offset_ = 0;
    }

    if (local_map_vertical_offset_ >= 0) {
      local_map_vertical_offset_ = 0;
    }

  }

  double horizontal_distance = _local_map_horizontal_distance_ + local_map_horizontal_offset_;
  double vertical_distance   = _local_map_vertical_distance_ + local_map_vertical_offset_;

  if (horizontal_distance < 10) {
    vertical_distance = 10;
    ROS_ERROR_THROTTLE(1.0, "[OctomapServer]: saturating local map size to 10, your computer is probably not very powerfull");
  }

  if (vertical_distance < 5) {
    vertical_distance = 5;
    ROS_ERROR_THROTTLE(1.0, "[OctomapServer]: saturating local map vertical size to 5, your computer is probably not very powerfull");
  }

  bool success = createLocalMap(_robot_frame_, horizontal_distance, vertical_distance, octree_local_);

  if (!success) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: failed to create the local map");
    return;
  }

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    ROS_WARN("[%s]: Nothing to publish, octree is empty", ros::this_node::getName().c_str());
    return;
  }

  if (pub_map_global_full_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::fullMapToMsg(*octree_local_, map)) {
      pub_map_local_full_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = ros::Time::now();  // TODO

    if (octomap_msgs::binaryMapToMsg(*octree_local_, map)) {
      pub_map_local_binary_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to binary representation");
    }
  }
}

//}

/* timerPersistency() //{ */

void OctomapServer::timerPersistency([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (!octree_initialized_) {
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
        std::scoped_lock lock(mutex_octree_);

        octree_->clear();
      }

      octree_initialized_ = true;

      timer_altitude_alignment_.stop();

      ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    } else {
      align_using_height = true;
    }

  } else {

    align_using_height = false;
  }

  // | ------ get the current UAV position in the map frame ----- |

  auto res = transformer_.getTransform(_robot_frame_, _world_frame_);

  double robot_x, robot_y, robot_z;

  if (res) {

    geometry_msgs::TransformStamped world_to_robot = res.value().getTransform();

    robot_x = world_to_robot.transform.translation.x;
    robot_y = world_to_robot.transform.translation.y;
    robot_z = world_to_robot.transform.translation.z;

    ROS_INFO("[OctomapServer]: robot coordinates %.2f, %.2f, %.2f", robot_x, robot_y, robot_z);

  } else {

    ROS_INFO_THROTTLE(1.0, "[OctomapServer]: waiting for the tf from %s to %s", _world_frame_.c_str(), _robot_frame_.c_str());
    return;
  }

  auto ground_z = getGroundZ(octree_, robot_x, robot_y);

  if (!ground_z) {

    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: could not calculate the Z of the ground below");

    {
      std::scoped_lock lock(mutex_octree_);

      octree_->clear();
    }

    octree_initialized_ = true;

    timer_altitude_alignment_.stop();

    ROS_INFO("[OctomapServer]: stopping the altitude alignment timer");

    return;
  }

  double ground_z_should_be = 0;

  if (align_using_height) {
    ground_z_should_be = robot_z - sh_height_.getMsg()->value;
  } else {
    ground_z_should_be = robot_z - _robot_height_ - 0.5 * octree_->getResolution();
  }

  double offset = ground_z_should_be - ground_z.value();

  ROS_INFO("[OctomapServer]: ground is at height %.2f m", ground_z.value());
  ROS_INFO("[OctomapServer]: ground should be at height %.2f m", ground_z_should_be);
  ROS_INFO("[OctomapServer]: shifting ground by %.2f m", offset);

  translateMap(octree_, 0, 0, offset);

  ROS_INFO("[OctomapServer]: octomap translated");

  octree_initialized_ = true;

  timer_altitude_alignment_.stop();
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud) {

  std::scoped_lock lock(mutex_octree_);

  const octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

  if (!octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !octree_->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
  }

  const float free_space_ray_len = 15.0;

  // instead of direct scan insertion, compute probabilistic update
  octomap::KeySet free_cells, occupied_cells;
  const bool      free_space_bounded = free_space_ray_len > 0.0f;

  // all points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    octomap::KeyRay  keyRay;
    const float      point_distance = float((measured_point - sensorOrigin).norm());

    octomap::OcTreeKey key;
    if (octree_->coordToKeyChecked(measured_point, key)) {

      occupied_cells.insert(key);

      /* updateMinKey(key, m_updateBBXMin); */
      /* updateMaxKey(key, m_updateBBXMax); */
    }

    // move end point to distance min(free space ray len, current distance)
    measured_point = sensorOrigin + (measured_point - sensorOrigin).normalize() * std::min(free_space_ray_len, point_distance);

    // free cells
    if (octree_->computeRayKeys(sensorOrigin, measured_point, keyRay)) {
      free_cells.insert(keyRay.begin(), keyRay.end());
    }
  }

  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    octomap::KeyRay  keyRay;

    // check if the ray intersects a cell in the occupied list
    if (octree_->computeRayKeys(sensorOrigin, measured_point, keyRay)) {

      bool                      ray_is_cool         = true;
      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

        // check if the cell was spotted as occupied by a valid ray
        if (occupied_cells.find(*it2) != occupied_cells.end()) {
          ray_is_cool = false;
          break;
        }

        // check if the cell is occupied in a map
        auto node = octree_->search(*it2);
        if (node && octree_->isNodeOccupied(node)) {

          if (it2 == keyRay.begin()) {
            alterantive_ray_end = keyRay.begin();  // special case
          } else {
            alterantive_ray_end = it2 - 1;
          }

          break;
        }
      }

      if (ray_is_cool) {
        free_cells.insert(keyRay.begin(), alterantive_ray_end);
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    /* if (occupied_cells.find(*it) == occupied_cells.end()) { */
    octree_->updateNode(*it, false);
    /* } */
  }

  // now mark all occupied cells:
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initializeLidarLUT(const size_t w, const size_t h) {

  const int                                       rangeCount         = w;
  const int                                       verticalRangeCount = h;
  std::vector<std::tuple<double, double, double>> coord_coeffs;
  const double                                    minAngle = 0.0;
  const double                                    maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -m_sensor_3d_vfov / 2.0;
  const double verticalMaxAngle = m_sensor_3d_vfov / 2.0;

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
  m_sensor_3d_xyz_lut.directions.resize(3, rangeCount * verticalRangeCount);
  m_sensor_3d_xyz_lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      m_sensor_3d_xyz_lut.directions.col(it) = vec3_t(x_coeff, y_coeff, z_coeff);
      m_sensor_3d_xyz_lut.offsets.col(it)    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* loadFromFile() //{ */

bool OctomapServer::loadFromFile(const std::string& filename) {

  std::string file_path = _map_path_ + "/" + filename + ".ot";

  {
    std::scoped_lock lock(mutex_octree_);

    if (file_path.length() <= 3)
      return false;

    std::string suffix = file_path.substr(file_path.length() - 3, 3);

    if (suffix == ".bt") {
      if (!octree_->readBinary(file_path)) {
        return false;
      }
    } else if (suffix == ".ot") {

      auto tree = octomap::AbstractOcTree::read(file_path);
      if (!tree) {
        return false;
      }

      OcTreeT* octree = dynamic_cast<OcTreeT*>(tree);
      octree_         = std::shared_ptr<OcTreeT>(octree);

      if (!octree_) {
        ROS_ERROR("[OctomapServer]: could not read OcTree file");
        return false;
      }

    } else {
      return false;
    }

    m_treeDepth        = octree_->getTreeDepth();
    m_maxTreeDepth     = m_treeDepth;
    octree_resolution_ = octree_->getResolution();

    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    octree_->getMetricMin(minX, minY, minZ);
    octree_->getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] = octree_->coordToKey(minX);
    m_updateBBXMin[1] = octree_->coordToKey(minY);
    m_updateBBXMin[2] = octree_->coordToKey(minZ);

    m_updateBBXMax[0] = octree_->coordToKey(maxX);
    m_updateBBXMax[1] = octree_->coordToKey(maxY);
    m_updateBBXMax[2] = octree_->coordToKey(maxZ);
  }

  return true;
}

//}

/* saveToFile() //{ */

bool OctomapServer::saveToFile(const std::string& filename) {

  std::scoped_lock lock(mutex_octree_);

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

  if (!octree_->write(tmp_file_path)) {
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

/* copyInsideBBX() //{ */

bool OctomapServer::copyInsideBBX(std::shared_ptr<OcTreeT>& from, std::shared_ptr<OcTreeT>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  mrs_lib::ScopeTimer scope_timer("copyInsideBBX", ros::Duration(1.0));

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  for (OcTreeT::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = from->search(k);

    expandNodeRecursive(from, node, it.getDepth());
  }

  for (OcTreeT::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    /* auto coords = it.getCoordinate(); */
    /* octomap::OcTreeKey k = to->coordToKey(coords); */

    to->setNodeValue(it.getKey(), it->getValue());
  }

  /* from->prune(); */
  /* to->prune(); */

  return true;
}

//}

/* expandNodeRecursive() //{ */

void OctomapServer::expandNodeRecursive(std::shared_ptr<OcTreeT>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

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

std::optional<double> OctomapServer::getGroundZ(std::shared_ptr<OcTreeT>& octree, const double& x, const double& y) {

  octomap::point3d p_min(float(x - _persistency_align_altitude_distance_), float(y - _persistency_align_altitude_distance_), -10000);
  octomap::point3d p_max(float(x + _persistency_align_altitude_distance_), float(y + _persistency_align_altitude_distance_), 10000);

  for (OcTreeT::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  std::vector<octomap::point3d> occupied_points;

  for (OcTreeT::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

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

bool OctomapServer::translateMap(std::shared_ptr<OcTreeT>& octree, const double& x, const double& y, const double& z) {

  octree->expand();

  // allocate the new future octree
  std::shared_ptr<OcTreeT> octree_new = std::make_shared<OcTreeT>(octree_resolution_);
  octree_new->setProbHit(_probHit_);
  octree_new->setProbMiss(_probMiss_);
  octree_new->setClampingThresMin(_thresMin_);
  octree_new->setClampingThresMax(_thresMax_);

  for (OcTreeT::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    auto coords = it.getCoordinate();

    coords.x() += float(x);
    coords.y() += float(y);
    coords.z() += float(z);

    auto value = it->getValue();
    auto key   = it.getKey();

    auto new_key = octree_new->coordToKey(coords);

    octree_new->setNodeValue(new_key, value);
  }

  octree_new->expand();

  octree = octree_new;

  return true;
}

//}

/* createLocalMap() //{ */

bool OctomapServer::createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance,
                                   std::shared_ptr<OcTreeT>& octree) {

  ros::Time time_start = ros::Time::now();

  auto res = transformer_.getTransform(frame_id, _world_frame_);

  if (!res) {
    ROS_WARN_THROTTLE(1.0, "[OctomapServer]: createLocalMap(): could not find tf from %s to %s", frame_id.c_str(), _world_frame_.c_str());
    return false;
  }

  geometry_msgs::TransformStamped world_to_robot = res.value().getTransform();

  double robot_x = world_to_robot.transform.translation.x;
  double robot_y = world_to_robot.transform.translation.y;
  double robot_z = world_to_robot.transform.translation.z;

  bool success = true;

  // copy the maps
  {
    std::scoped_lock lock(mutex_octree_);

    octree->clear();

    const octomap::point3d p_min =
        octomap::point3d(float(robot_x - horizontal_distance), float(robot_y - horizontal_distance), float(robot_z - vertical_distance));
    const octomap::point3d p_max =
        octomap::point3d(float(robot_x + horizontal_distance), float(robot_y + horizontal_distance), float(robot_z + vertical_distance));

    success = copyInsideBBX(octree_, octree, p_min, p_max);
  }

  {
    std::scoped_lock lock(mutex_avg_time_local_map_processing_);

    ros::Time time_end = ros::Time::now();

    double exec_duration = (time_end - time_start).toSec();

    double coef                    = 0.00;
    avg_time_local_map_processing_ = coef * avg_time_local_map_processing_ + (1.0 - coef) * exec_duration;

    if (avg_time_local_map_processing_ > ((1.0 / _local_map_rate_) * _local_map_max_computation_duty_cycle_)) {
      ROS_ERROR_THROTTLE(5.0, "[OctomapServer]: avg local map creation time = %.3f sec", avg_time_local_map_processing_);
    } else {
      ROS_WARN_THROTTLE(5.0, "[OctomapServer]: avg local map creation time = %.3f sec", avg_time_local_map_processing_);
    }
  }

  return success;
}

//}

}  // namespace mrs_octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_server::OctomapServer, nodelet::Nodelet)
