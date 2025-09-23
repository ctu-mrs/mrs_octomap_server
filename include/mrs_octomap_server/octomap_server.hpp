#ifndef COMPOSITION__OCTOMAP_SERVER_COMPONENT_HPP_
#define COMPOSITION__OCTOMAP_SERVER_COMPONENT_HPP_


// Octomap
#include <octomap/OcTreeNode.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

// Messages ROS2
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

//remplce laser_geometry
#include <laser_geometry/laser_geometry.hpp>  

// Messages de capteurs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Services ROS2
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// pcl_ros n'existe pas en ROS2, utilisez directement PCL et tf2 pour les transformations
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// Messages Octomap
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/conversions.h>

// MRS Lib
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/publisher_handler.h>

// Messages personnalisés
// #include <mrs_octomap_server/msg/PoseWithSize.hpp>
#include <mrs_modules_msgs/msg/pose_with_size.hpp>


#include <mrs_octomap_server/conversions.h>

// Messages MRS
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/srv/string.hpp>

// Eigen
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/memory.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.hpp>


#include <filesystem>


#include <cmath>

namespace mrs_octomap_server
{

    #if USE_ROS_TIMER == 1
    typedef mrs_lib::ROSTimer TimerType;
    #else
    typedef mrs_lib::ThreadTimer TimerType;
    #endif

    using vec3s_t = Eigen::Matrix<float, 3, -1>;
    using vec3_t  = Eigen::Vector3f;
    using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;

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
    double free_ray_distance;
    double vertical_fov;
    int    vertical_rays;
    int    horizontal_rays;
    bool   update_free_space;
    bool   clear_occupied;
    double free_ray_distance_unknown;
    } SensorParams3DLidar_t;

    typedef struct
    {
    double max_range;
    double free_ray_distance;
    double vertical_fov;
    double horizontal_fov;
    int    vertical_rays;
    int    horizontal_rays;
    bool   update_free_space;
    bool   clear_occupied;
    double free_ray_distance_unknown;
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

    const std::string _sensor_names_[] = {"LIDAR_3D", "LIDAR_2D", "LIDAR_1D", "DEPTH_CAMERA", "ULTRASOUND"};


    class OctomapServer : public rclcpp::Node
    {
        public:
        explicit OctomapServer(const rclcpp::NodeOptions & options);
        virtual void onInit();

        private:

        std::atomic<bool> is_initialized_ = false;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Clock::SharedPtr clock_;

        // | -------------------- callbacks ------------------- |

        //rclcpp::CallbackGroup::SharedPtr cbgrp_main_;
        //rclcpp::CallbackGroup::SharedPtr cbgrp_sensors_;
        //rclcpp::CallbackGroup::SharedPtr cbgrp_status_;

        bool callbackLoadMap(const std::shared_ptr<mrs_msgs::srv::String::Request> req, std::shared_ptr<mrs_msgs::srv::String::Response> resp);
        bool callbackSaveMap(const std::shared_ptr<mrs_msgs::srv::String::Request> req, std::shared_ptr<mrs_msgs::srv::String::Response> resp);

        bool callbackResetMap([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req, [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> resp);

        void callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const SensorType_t sensor_type, const int sensor_id, const std::string topic,
                                    const bool pcl_over_max_range);

        void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void callbackCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, const int sensor_id);
        bool loadFromFile(const std::string& filename);
        bool saveToFile(const std::string& filename);   

        // | -------------------- topic subscribers ------------------- |

        mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
        mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>            sh_height_;
        mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::PoseWithSize>    sh_clear_box_;

        std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_3dlaser_pc2_;
        std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_depth_cam_pc2_;
        std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>>  sh_depth_cam_info_;
        std::vector<mrs_lib::SubscriberHandler<sensor_msgs::msg::LaserScan>>   sh_laser_scan_;

        // | ----------------------- publishers ----------------------- |

        mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_global_full_;
        mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_global_binary_;
        mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_local_full_;
        mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_local_binary_;


        // rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_full_;
        // rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_global_binary_;
        // rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_full_;
        // rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_map_local_binary_;

        // | -------------------- service servers -------------------- |

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr ss_reset_map_;
        rclcpp::Service<mrs_msgs::srv::String>::SharedPtr ss_save_map_;
        rclcpp::Service<mrs_msgs::srv::String>::SharedPtr ss_load_map_;


        
        // | ------------------------- timers ------------------------- |
        rclcpp ::TimerBase::SharedPtr timer_init_;

        std::shared_ptr<TimerType> timer_global_map_publisher_;
        double     _global_map_publisher_rate_;
        void       timerGlobalMapPublisher();

        std::shared_ptr<TimerType> timer_global_map_creator_;
        double     _global_map_creator_rate_;
        void       timerGlobalMapCreator();

        std::shared_ptr<TimerType> timer_local_map_publisher_;
        void       timerLocalMapPublisher();

        std::shared_ptr<TimerType> timer_local_map_resizer_;
        void       timerLocalMapResizer();

        std::shared_ptr<TimerType> timer_persistency_;
        void       timerPersistency();

        std::shared_ptr<TimerType> timer_altitude_alignment_;
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

}  // namespace mrs_octomap_server

#endif  // COMPOSITION__OCTOMAP_SERVER_COMPONENT_HPP_
