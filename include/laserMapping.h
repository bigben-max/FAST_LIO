#pragma once

#include <math.h>
#include <omp.h>
#include <unistd.h>
#include <csignal>
#include <fstream>
#include <mutex>
#include <thread>
//
#include <Eigen/Core>

//
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

//
#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "preprocess.h"
#include "so3_math.h"

namespace odometry {

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

PointCloudXYZI::Ptr feats_down_body;
PointCloudXYZI::Ptr feats_down_world;

PointCloudXYZI::Ptr laserCloudOri;
PointCloudXYZI::Ptr corr_normvect;
int effct_feat_num = 0;
std::vector<PointVector> Nearest_Points;
KD_TREE ikdtree;

class laserMapping {
 public:
  laserMapping() = default;
  ~laserMapping() {}

  void init();
  void initRos(ros::NodeHandle &nh);
  void work();

  static void SigHandle(int sig) {
    ROS_WARN("catch sig %d", sig);
    // sig_buffer.notify_all();
    ros::shutdown();
  }

  void lasermap_fov_segment();
  void load_imu();
  void load_pcl();
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  bool sync_packages(MeasureGroup &meas);
  void map_incremental();
  void publish_frame_world(const ros::Publisher &pubLaserCloudFull);
  void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect);
  void publish_map(const ros::Publisher &pubLaserCloudMap);
  void publish_path(const ros::Publisher pubPath);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  static void h_share_model(state_ikfom &s,
                            esekfom::dyn_share_datastruct<double> &ekfom_data);

  void dump_lio_state_to_log(FILE *fp);
  void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po,
                              state_ikfom &s) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }
  void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }
  template <typename T>
  void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
  }
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
  }

  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                   state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
  }
  void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    for (int i = 0; i < points_history.size(); i++)
      _featsArray->push_back(points_history[i]);
  }

  template <typename T>
  void set_posestamp(T &out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
  }

 private:
  /*** Time Log Variables ***/
  double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0,
         kdtree_delete_time = 0.0;
  double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN],
      s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN],
      s_plot10[MAXN], s_plot11[MAXN];
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;
  int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0,
      kdtree_delete_counter = 0;
  bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false,
       path_en = true;
  /**************************/

  float DET_RANGE = 300.0f;
  const float MOV_THRESHOLD = 1.5f;

  std::mutex mtx_buffer;
  condition_variable sig_buffer;

  std::string root_dir = ROOT_DIR;
  std::string map_file_path, lid_topic, imu_topic;

  double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  double filter_size_corner_min = 0, filter_size_surf_min = 0,
         filter_size_map_min = 0, fov_deg = 0;
  double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0,
         lidar_end_time = 0, first_lidar_time = 0.0;

  int time_log_counter = 0, scan_count = 0, publish_count = 0;
  int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
      laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
  bool lidar_pushed, flg_first_scan = true, flg_EKF_inited;
  bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

  std::vector<std::vector<int>> pointSearchInd_surf;
  std::vector<BoxPointType> cub_needrm;

  std::vector<double> extrinT;
  std::vector<double> extrinR;
  std::deque<double> time_buffer;
  std::deque<PointCloudXYZI::Ptr> lidar_buffer;
  std::deque<ImuMeasurement> imu_buffer;

  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;
  double timediff_lidar_wrt_imu = 0.0;
  bool timediff_set_flg = false;
  double lidar_mean_scantime = 0.0;
  int scan_num = 0;
  int process_increments = 0;

  PointCloudXYZI::Ptr featsFromMap;
  PointCloudXYZI::Ptr feats_undistort;

  PointCloudXYZI::Ptr _featsArray;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;

  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_save;

  V3F XAxisPoint_body{LIDAR_SP_LEN, 0.0, 0.0};
  V3F XAxisPoint_world{LIDAR_SP_LEN, 0.0, 0.0};
  V3D euler_cur;
  V3D position_last{Zero3d};
  V3D Lidar_T_wrt_IMU{Zero3d};
  M3D Lidar_R_wrt_IMU{Eye3d};

  /*** EKF inputs and output ***/
  MeasureGroup Measures;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  state_ikfom state_point;
  vect3 pos_lid;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  std::shared_ptr<Preprocess> p_pre;
  std::shared_ptr<ImuProcess> p_imu;

  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Publisher pubLaserCloudFull_;
  ros::Publisher pubLaserCloudFull_body_;
  ros::Publisher pubLaserCloudEffect_;
  ros::Publisher pubLaserCloudMap_;
  ros::Publisher pubOdomAftMapped_;
  ros::Publisher pubPath_;
};

}  // namespace odometry