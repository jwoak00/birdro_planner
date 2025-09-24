#ifndef BIRDRO_PLANNER_HPP
#define BIRDRO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>
#include <vector>
// Removed validation status dependency; using runtime speed/accel guard instead

using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::PlannerWaypoints;
using autoware::freespace_planning_algorithms::PlannerWaypoint;


using namespace autoware_planning_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;

namespace birdro_planner
{
class BirdroPlanner : public rclcpp::Node
{
public:
  explicit BirdroPlanner(const rclcpp::NodeOptions & options);

private:
  void init_ros();
  void init_algo_parameter();
  PlannerCommonParam get_planner_common_param();

  // Sub
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr main_trajectory_pub_; // optional mirror publisher
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occlusion_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_; // NEW
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void on_goal_pose(const PoseStamped::ConstSharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_timer();
  void reset();
  void updateTargetIndex();
  bool isPlanRequired();
  void planTrajectory();
  bool isStopped();
  void publishOcclusionMarker(const Pose & goal_pose);
  void publishGoalMarker(); // NEW
  void deleteOcclusionMarker();
  TransformStamped getTransform(const std::string & from, const std::string & to);
  Pose transformPoseToFrame(
    const PoseStamped & pose_stamped, const std::string & target_frame);
  double computeTrajectoryLength(const Trajectory & traj) const;
  void publishTrajectory(const Trajectory & traj);
  void cacheStopHoldTrajectory();


  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  VehicleShape vehicle_shape_;

  LaneletRoute::ConstSharedPtr route_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Odometry::ConstSharedPtr odom_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_ = 0;
  size_t target_index_ = 0;
  bool is_completed_ = false;
  bool reset_in_progress_ = false;
  bool waiting_for_new_goal_stop_ = false;
  bool has_pending_goal_pose_ = false;
  bool has_stop_hold_base_traj_ = false;
  bool is_new_parking_cycle_ = true;
  std::string last_replan_reason_;

  // Parameters (runtime configurable)
  bool use_occlusion_marker_ = false;
  // removed NDZ forbidden zone marker; keep goal marker only
  bool visualize_goal_marker_ = true;  // NEW
  bool require_stop_before_planning_ = true; // allow first plan while moving if false
  bool mirror_to_main_traj_{false};

  // Hardcoded / default-initialized parameters
  std::string algorithm_name_ = "astar";  // NEW: 선택 알고리즘 (astar | rrtstar)
  double waypoints_velocity_ = 5.0 / 3.6; // m/s
  double update_rate_ = 10.0;
  double th_arrived_distance_m_ = 1.0;
  double th_stopped_time_sec_ = 1.0;
  double th_stopped_velocity_mps_ = 0.01;
  double th_course_out_distance_m_ = 1.0;
  double vehicle_shape_margin_m_ = 1.0;
  bool replan_when_obstacle_found_ = true;
  bool replan_when_course_out_ = true;
  int obstacle_threshold_ = 100; // NEW: start/goal 차단 셀 판정 임계값

  // (Removed obstacle debouncing state for simplification)

  PoseStamped current_pose_;
  PoseStamped goal_pose_;
  PoseStamped pending_goal_pose_;
  Trajectory stop_hold_base_traj_;

  // === 새로 추가: 장애물 재계획 디바운싱/쿨다운 상태 ===
  rclcpp::Time last_replan_time_;          // 생성자에서 now()로 설정
  Pose last_replan_pose_;
  size_t obstacle_detect_consecutive_count_ = 0;
  size_t obstacle_detect_confirm_count_ = 6;
  double min_obstacle_replan_interval_sec_ = 3.0;
  double min_replan_progress_distance_m_ = 2.0;

  // --- Runtime Speed Guard (replaces validation guard) ---
  bool sg_enable_{true};
  double sg_max_speed_mps_{5.0 / 3.6};
  double sg_max_accel_mps2_{2.5};
  double sg_max_lateral_accel_mps2_{4.0};
  int sg_trigger_consecutive_{3};
  double sg_stop_duration_sec_{1.0};
  int sg_mode_{0}; // 0=stop, 1=hold_last_valid (reuse concept)

  int sg_current_violation_count_{0};
  rclcpp::Time sg_last_eval_time_{};
  double sg_last_speed_mps_{0.0};
  rclcpp::Time sg_safety_stop_until_{};
  bool sg_in_safety_stop_{false};
  Trajectory sg_last_valid_traj_;
  void init_speed_guard_params();
  void evaluateSpeedGuard();
  void speedGuardEnterSafetyStop(const char * reason);
  // --- Runtime Speed Guard end ---

  // --- Speed Guard freeze (mode 0 전용) ---
  PoseStamped sg_freeze_pose_;
  bool sg_freeze_active_{false};

  // --- Velocity ramp parameters ---
  bool ramp_enable_{true};
  double ramp_max_accel_mps2_{1.0};          // 가속 한계
  double ramp_max_decel_mps2_{1.0};          // 감속 한계
  bool ramp_reverse_gate_enable_{true};      // 시작이 반대 방향이면 0속도 구간으로 게이트
  double ramp_reverse_gate_speed_thresh_mps_{0.3};  // 현재 속도가 이보다 크면 게이트 발동
  double ramp_reverse_gate_stop_distance_m_{0.5};   // 시작부 0속도 구간 길이

  // Helper
  void applySpeedRampProfile(Trajectory & traj, double v0_abs);

  // --- Course-out behavior tuning ---
  double course_out_publish_margin_m_{0.2};
  double min_course_out_replan_interval_sec_{1.0};
  double course_out_publish_epsilon_m_{0.02};
};
}  // namespace birdro_planner
#endif
