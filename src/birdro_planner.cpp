#include "birdro_planner.hpp"
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/exceptions.h>
namespace birdro_planner
{

namespace
{
  PoseArray trajectory2PoseArray(const Trajectory & trajectory)
  {
    PoseArray pose_array;
    pose_array.header = trajectory.header;

    for (const auto & point : trajectory.points) {
      pose_array.poses.push_back(point.pose);
    }

    return pose_array;
  }

  std::vector<size_t> getReversingIndices(const Trajectory & trajectory)
  {
    std::vector<size_t> indices;

    // 빈/짧은 궤적 가드
    if (trajectory.points.size() < 2) {
      return indices;
    }

    for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
      if (
        trajectory.points.at(i).longitudinal_velocity_mps *
          trajectory.points.at(i + 1).longitudinal_velocity_mps < 0) {
        indices.push_back(i);
      }
    }

    return indices;
  }

  size_t getNextTargetIndex(
    const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
    const size_t current_target_index)
  {
    if (!reversing_indices.empty()) {
      for (const auto reversing_index : reversing_indices) {
        if (reversing_index > current_target_index) {
          return reversing_index;
        }
      }
    }

    return trajectory_size - 1;
  }

  Trajectory getPartialTrajectory(
    const Trajectory & trajectory, const size_t start_index, const size_t end_index)
  {
    Trajectory partial_trajectory;
    partial_trajectory.header = trajectory.header;
    // 발행 시점에서 now()를 찍도록 여기서는 덮어쓰지 않음
    // partial_trajectory.header.stamp = rclcpp::Clock().now();

    // 필요한 크기만 예약
    partial_trajectory.points.reserve(end_index - start_index + 1);
    for (size_t i = start_index; i <= end_index; ++i) {
      partial_trajectory.points.push_back(trajectory.points.at(i));
    }

    // Modify velocity at start/end point
    if (partial_trajectory.points.size() >= 2) {
      partial_trajectory.points.front().longitudinal_velocity_mps =
        partial_trajectory.points.at(1).longitudinal_velocity_mps;
    }
    if (!partial_trajectory.points.empty()) {
      partial_trajectory.points.back().longitudinal_velocity_mps = 0;
    }

    return partial_trajectory;
  }

  double calcDistance2d(const Trajectory & trajectory, const Pose & pose)
  {
    if (trajectory.points.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto idx = autoware::motion_utils::findNearestIndex(trajectory.points, pose.position);
    return autoware::universe_utils::calcDistance2d(trajectory.points.at(idx), pose);
  }

  Pose transformPose(const Pose & pose, const TransformStamped & transform)
  {
    // 변환 정보가 비어 있으면 원본 포즈를 그대로 사용한다.
    if (transform.header.frame_id.empty() && transform.child_frame_id.empty()) {
      return pose;
    }

    PoseStamped transformed_pose;
    PoseStamped orig_pose;
    orig_pose.pose = pose;

    // 입력 포즈의 프레임/시간을 명시적으로 설정
    // lookupTransform(target, source, ...) → child_frame_id == source
    orig_pose.header.frame_id =
      transform.child_frame_id.empty() ? transform.header.frame_id : transform.child_frame_id;
    orig_pose.header.stamp = transform.header.stamp;

    try {
      tf2::doTransform(orig_pose, transformed_pose, transform);
    } catch (const tf2::TransformException &) {
      return pose;
    }

    return transformed_pose.pose;
  }

  Trajectory createTrajectory(
    const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
    const double & velocity)
  {
    Trajectory trajectory;
    trajectory.header = planner_waypoints.header;

    for (const auto & awp : planner_waypoints.waypoints) {
      TrajectoryPoint point;

      point.pose = awp.pose.pose;

      point.pose.position.z = current_pose.pose.position.z;  // height = const
      point.longitudinal_velocity_mps = velocity;      // velocity = const

      // switch sign by forward/backward
      point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;

      trajectory.points.push_back(point);
    }

    return trajectory;
  }

  Trajectory createStopTrajectory(const PoseStamped & current_pose)
  {
    PlannerWaypoints waypoints;
    PlannerWaypoint waypoint;

    // 현재 포즈의 헤더를 그대로 사용 (ROS time 일관성)
    waypoints.header = current_pose.header;
    waypoint.pose.header = waypoints.header;
    waypoint.pose.pose = current_pose.pose;
    waypoint.is_back = false;
    waypoints.waypoints.push_back(waypoint);

    return createTrajectory(current_pose, waypoints, 0.0);
  }

  Trajectory createStopTrajectory(const Trajectory & trajectory)
  {
    Trajectory stop_trajectory = trajectory;
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      stop_trajectory.points.at(i).longitudinal_velocity_mps = 0.0;
    }
    return stop_trajectory;
  }

}  // namespace

BirdroPlanner::BirdroPlanner(const rclcpp::NodeOptions & options)
: Node("birdro_planner_node", options)
{
  // Declare user configurable parameters early (so init functions can use them)
  algorithm_name_ = declare_parameter<std::string>("algorithm_name", algorithm_name_);
  waypoints_velocity_ = declare_parameter<double>("cruise_velocity", waypoints_velocity_);
  update_rate_ = declare_parameter<double>("update_rate", update_rate_);
  th_arrived_distance_m_ = declare_parameter<double>("th_arrived_distance_m", th_arrived_distance_m_);
  th_stopped_time_sec_ = declare_parameter<double>("th_stopped_time_sec", th_stopped_time_sec_);
  th_stopped_velocity_mps_ = declare_parameter<double>("th_stopped_velocity_mps", th_stopped_velocity_mps_);
  th_course_out_distance_m_ = declare_parameter<double>("th_course_out_distance_m", th_course_out_distance_m_);
  vehicle_shape_margin_m_ = declare_parameter<double>("vehicle_shape_margin_m", vehicle_shape_margin_m_);
  replan_when_obstacle_found_ = declare_parameter<bool>("replan_when_obstacle_found", replan_when_obstacle_found_);
  replan_when_course_out_ = declare_parameter<bool>("replan_when_course_out", replan_when_course_out_);
  use_occlusion_marker_ = declare_parameter<bool>("use_occlusion_marker", use_occlusion_marker_);
  visualize_goal_marker_ = declare_parameter<bool>("visualize_goal_marker", visualize_goal_marker_);
  // Course-out behavior parameters
  course_out_publish_margin_m_ = declare_parameter<double>(
    "course_out.publish_margin_m", course_out_publish_margin_m_);
  min_course_out_replan_interval_sec_ = declare_parameter<double>(
    "course_out.replan_min_interval_sec", min_course_out_replan_interval_sec_);
  course_out_publish_epsilon_m_ = declare_parameter<double>(
    "course_out.publish_epsilon_m", course_out_publish_epsilon_m_);
  // --- Speed Guard 파라미터 초기화 ---
  init_speed_guard_params();

  // --- Velocity ramp parameters ---
  ramp_enable_ = declare_parameter<bool>("ramp.enable", ramp_enable_);
  ramp_max_accel_mps2_ = declare_parameter<double>("ramp.max_accel_mps2", ramp_max_accel_mps2_);
  ramp_max_decel_mps2_ = declare_parameter<double>("ramp.max_decel_mps2", ramp_max_decel_mps2_);
  ramp_reverse_gate_enable_ =
    declare_parameter<bool>("ramp.reverse_gate.enable", ramp_reverse_gate_enable_);
  ramp_reverse_gate_speed_thresh_mps_ = declare_parameter<double>(
    "ramp.reverse_gate.speed_thresh_mps", ramp_reverse_gate_speed_thresh_mps_);
  ramp_reverse_gate_stop_distance_m_ = declare_parameter<double>(
    "ramp.reverse_gate.stop_distance_m", ramp_reverse_gate_stop_distance_m_);

  // size_t -> int64_t 로 선언 후 캐스팅 (구형/신규 파라미터 이름 모두 지원)
  const int64_t obstacle_detect_confirm_param = declare_parameter<int64_t>(
    "obstacle_detect_confirm_count",
    static_cast<int64_t>(obstacle_detect_confirm_count_));
  const int64_t obstacle_replan_confirm_param = declare_parameter<int64_t>(
    "obstacle_replan_confirm_count", obstacle_detect_confirm_param);
  obstacle_detect_confirm_count_ = static_cast<size_t>(obstacle_replan_confirm_param);

  min_obstacle_replan_interval_sec_ =
    declare_parameter<double>("obstacle_replan_min_interval_sec", min_obstacle_replan_interval_sec_);

  min_replan_progress_distance_m_ =
    declare_parameter<double>("min_replan_progress_distance_m", min_replan_progress_distance_m_);

  require_stop_before_planning_ =
    declare_parameter<bool>("require_stop_before_planning", require_stop_before_planning_);

  init_ros(); // depends on update_rate_
  init_algo_parameter(); // depends on algorithm_name_
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  last_replan_time_ = now();

  RCLCPP_INFO(get_logger(), "BirdroPlanner initialized. Algorithm=%s, update_rate=%.2f Hz", algorithm_name_.c_str(), update_rate_);
}

void BirdroPlanner::init_speed_guard_params()
{
  sg_enable_ = declare_parameter<bool>("speed_guard.enable", sg_enable_);
  sg_max_speed_mps_ = declare_parameter<double>("speed_guard.max_speed_mps", sg_max_speed_mps_);
  sg_max_accel_mps2_ = declare_parameter<double>("speed_guard.max_accel_mps2", sg_max_accel_mps2_);
  sg_max_lateral_accel_mps2_ =
    declare_parameter<double>("speed_guard.max_lateral_accel_mps2", sg_max_lateral_accel_mps2_);
  sg_trigger_consecutive_ =
    declare_parameter<int>("speed_guard.trigger_consecutive", sg_trigger_consecutive_);
  sg_stop_duration_sec_ =
    declare_parameter<double>("speed_guard.stop_duration_sec", sg_stop_duration_sec_);
  sg_mode_ = declare_parameter<int>("speed_guard.mode", sg_mode_);
}

void BirdroPlanner::init_ros()
{
  using std::placeholders::_1;
  rclcpp::QoS qos{1};
  route_sub_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&BirdroPlanner::onRoute, this, _1));
  occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", rclcpp::QoS{1}, std::bind(&BirdroPlanner::onOccupancyGrid, this, _1));
  RCLCPP_INFO(
    get_logger(), "Subscribed occupancy grid topic: %s",
    occupancy_grid_sub_->get_topic_name());
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 100, std::bind(&BirdroPlanner::onOdometry, this, _1));
  sub_goal_pose_ = create_subscription<PoseStamped>(
    "/planning/mission_planning/checkpoint", 100, std::bind(&BirdroPlanner::on_goal_pose, this, _1));
  initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1, std::bind(&BirdroPlanner::onInitialPose, this, _1));

  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
  // Optionally mirror to main trajectory for downstream nodes expecting lane driving path
  mirror_to_main_traj_ = declare_parameter<bool>("mirror_to_main_trajectory", mirror_to_main_traj_);
  if (mirror_to_main_traj_) {
    main_trajectory_pub_ = create_publisher<Trajectory>(
      "/planning/scenario_planning/lane_driving/trajectory", qos);
  }
  parking_state_pub_ =
    create_publisher<std_msgs::msg::Bool>("/planning/scenario_planning/parking/is_completed", qos);
  debug_pose_array_pub_ = create_publisher<PoseArray>("~/debug/pose_array", qos);
  debug_partial_pose_array_pub_ = create_publisher<PoseArray>("~/debug/partial_pose_array", qos);
  occlusion_marker_pub_ =
    create_publisher<visualization_msgs::msg::Marker>("~/debug/occlusion_zone", qos);
  goal_marker_pub_ =
    create_publisher<visualization_msgs::msg::Marker>("~/debug/goal_marker", qos);
  // (Validation status subscription removed; using runtime speed guard only)

  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&BirdroPlanner::on_timer, this));
}

void BirdroPlanner::publishTrajectory(const Trajectory & traj)
{
  trajectory_pub_->publish(traj);
  if (mirror_to_main_traj_ && main_trajectory_pub_) {
    main_trajectory_pub_->publish(traj);
  }
}

void BirdroPlanner::cacheStopHoldTrajectory()
{
  has_stop_hold_base_traj_ = false;

  if (!partial_trajectory_.points.empty()) {
    stop_hold_base_traj_ = partial_trajectory_;
    has_stop_hold_base_traj_ = true;
    return;
  }

  if (!trajectory_.points.empty()) {
    stop_hold_base_traj_ = trajectory_;
    has_stop_hold_base_traj_ = true;
    return;
  }

  if (current_pose_.header.frame_id != "") {
    stop_hold_base_traj_ = createStopTrajectory(current_pose_);
    has_stop_hold_base_traj_ = !stop_hold_base_traj_.points.empty();
  }
}

void BirdroPlanner::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  route_ = msg;
  goal_pose_.header = msg->header;
  goal_pose_.pose = msg->goal_pose;
  is_new_parking_cycle_ = true;
  reset();
}
void BirdroPlanner::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  (void)msg; // 메시지 내용은 사용하지 않으므로 경고 방지

  RCLCPP_INFO(get_logger(), "New initial pose received. Clearing current goal and trajectory.");
  deleteOcclusionMarker();
  // 현재 목표 지점을 비웁니다.
  goal_pose_ = PoseStamped();

  // 플래너의 모든 상태를 초기화합니다.
  reset();
}
void BirdroPlanner::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

// Scenario subscription removed (single fixed map source as per requirements)

void BirdroPlanner::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer (최소 1개 유지)
  while (odom_buffer_.size() > 1) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);
    if (time_diff.seconds() < th_stopped_time_sec_) {
      break;
    }
    odom_buffer_.pop_front();
  }
}

void BirdroPlanner::on_goal_pose(const PoseStamped::ConstSharedPtr msg)
{
  deleteOcclusionMarker();
  pending_goal_pose_ = *msg;
  goal_pose_ = *msg;
  has_pending_goal_pose_ = true;
  is_new_parking_cycle_ = true;

  const bool need_stop_for_new_goal = require_stop_before_planning_ && !isStopped();
  waiting_for_new_goal_stop_ = need_stop_for_new_goal;
  reset_in_progress_ = require_stop_before_planning_;
  last_replan_reason_ = "new_goal";

  // --- 새 goal 수신 시 SpeedGuard freeze (mode0) 해제 ---
  if (sg_mode_ == 0 && sg_freeze_active_) {
    RCLCPP_INFO(get_logger(), "[SpeedGuard] Freeze released by new goal");
    sg_freeze_active_ = false;
  }

  cacheStopHoldTrajectory();
  reset();
  if (visualize_goal_marker_) publishGoalMarker();
  if (use_occlusion_marker_) publishOcclusionMarker(goal_pose_.pose);
  RCLCPP_INFO(get_logger(), "New goal received (%.2f, %.2f)", goal_pose_.pose.position.x, goal_pose_.pose.position.y);
}

void BirdroPlanner::publishOcclusionMarker(const Pose & goal_pose)
{
  if (!use_occlusion_marker_) return;
  visualization_msgs::msg::Marker occlusion_marker;
  occlusion_marker.header.frame_id = goal_pose_.header.frame_id.empty() ? "map" : goal_pose_.header.frame_id;
  occlusion_marker.header.stamp = this->get_clock()->now();
  occlusion_marker.ns = "occlusion_zone";
  occlusion_marker.id = 0;
  occlusion_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  occlusion_marker.action = visualization_msgs::msg::Marker::ADD;
  occlusion_marker.pose = goal_pose;
  double radius = 0.4; // 0.4m 반경
  occlusion_marker.scale.x = radius * 2.0;
  occlusion_marker.scale.y = radius * 2.0;
  occlusion_marker.scale.z = 0.1;
  occlusion_marker.color.r = 0.5f;
  occlusion_marker.color.g = 0.5f;
  occlusion_marker.color.b = 0.5f;
  occlusion_marker.color.a = 0.5f;
  occlusion_marker.lifetime = rclcpp::Duration(0, 0);
  occlusion_marker_pub_->publish(occlusion_marker); // PUBLISH 추가
}
// NEW: goal marker
void BirdroPlanner::publishGoalMarker()
{
  if (!goal_marker_pub_) return;
  visualization_msgs::msg::Marker m;
  m.header = goal_pose_.header;
  if (m.header.frame_id.empty()) {
    m.header.frame_id = "map";
  }
  // 항상 현재 시간으로 스탬프
  m.header.stamp = this->now();
  m.ns = "goal";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose = goal_pose_.pose;
  m.scale.x = 0.6;
  m.scale.y = 0.6;
  m.scale.z = 0.6;
  m.color.r = 0.0f;
  m.color.g = 1.0f;
  m.color.b = 0.0f;
  m.color.a = 0.85f;
  m.lifetime = rclcpp::Duration(0,0);
  goal_marker_pub_->publish(m);
}
// Removed: publishForbiddenZone (visualization is handled by external overlay node)

void BirdroPlanner::deleteOcclusionMarker()
{
  RCLCPP_INFO(get_logger(), "Making occlusion zone marker invisible.");
  visualization_msgs::msg::Marker invisible_marker;

  invisible_marker.header.stamp = this->get_clock()->now();
  // frame_id는 필수로 채워줘야 합니다. 이전에 발행된 마커와 동일한 프레임을 사용합니다.
  invisible_marker.header.frame_id = "map"; // 또는 goal_pose_.header.frame_id

  // === 수정의 핵심 부분 ===
  invisible_marker.ns = "occlusion_zone";
  invisible_marker.id = 0;
  invisible_marker.action = visualization_msgs::msg::Marker::ADD; // DELETE 대신 ADD

  // color.a (alpha, 투명도)를 0.0으로 설정하여 완전히 투명하게 만듭니다.
  invisible_marker.color.a = 0.0;

  // scale을 0으로 만드는 방법도 있지만, 일부 RViz 버전에서 문제를 일으킬 수 있으므로
  // 투명도를 조절하는 것이 가장 안정적입니다.
  invisible_marker.scale.x = 0.1; // 크기는 아주 작게만 설정해도 됩니다.
  invisible_marker.scale.y = 0.1;
  invisible_marker.scale.z = 0.1;

  occlusion_marker_pub_->publish(invisible_marker);
}

bool BirdroPlanner::isPlanRequired()
{
  // --- 추가: SpeedGuard freeze 상태에서는 재계획 금지 (mode 0) ---
  if (sg_mode_ == 0 && sg_freeze_active_) {
    RCLCPP_INFO(get_logger(), "[isPlanRequired] SpeedGuard freeze active -> no replan");
    return false;
  }

  if (trajectory_.points.empty()) {
    last_replan_reason_ = "initial_or_empty";
    RCLCPP_INFO(get_logger(), "[isPlanRequired] Trajectory empty -> replan");
    return true;
  }

  const auto now_time = now();

  auto distance_from_last_replan = [&]() {
    const auto & p = current_pose_.pose.position;
    const auto & q = last_replan_pose_.position;
    const double dx = p.x - q.x;
    const double dy = p.y - q.y;
    return std::sqrt(dx * dx + dy * dy);
  };

  if (replan_when_obstacle_found_) {
    if (!occupancy_grid_) {
      RCLCPP_WARN(get_logger(), "[isPlanRequired] Occupancy grid not ready yet (skip obstacle check)");
      return false;
    }
    algo_->setMap(*occupancy_grid_);

    // Choose a safe forward trajectory for obstacle check
    Trajectory forward_trajectory;
    if (!partial_trajectory_.points.empty()) {
      // Use partial trajectory segment from ego to the end
      const auto ego_in_partial_frame =
        transformPoseToFrame(current_pose_, partial_trajectory_.header.frame_id);
      const size_t nearest_index_partial =
        autoware::motion_utils::findNearestIndex(partial_trajectory_.points, ego_in_partial_frame.position);
      const size_t end_index_partial = partial_trajectory_.points.size() - 1;
      forward_trajectory = getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);
    } else if (!trajectory_.points.empty()) {
      // Fall back to remaining full trajectory from ego to the end
      const auto ego_in_traj_frame = transformPoseToFrame(current_pose_, trajectory_.header.frame_id);
      const size_t nearest_index_full =
        autoware::motion_utils::findNearestIndex(trajectory_.points, ego_in_traj_frame.position);
      const size_t end_index_full = trajectory_.points.size() - 1;
      forward_trajectory = getPartialTrajectory(trajectory_, nearest_index_full, end_index_full);
    } else {
      // No trajectory to check yet
      RCLCPP_INFO(get_logger(), "[isPlanRequired] No trajectory available for obstacle check yet");
      return true; // trigger planning if we reached here with empty trajectory
    }

    if (forward_trajectory.points.empty()) {
      RCLCPP_INFO(get_logger(), "[isPlanRequired] Forward trajectory empty -> skip obstacle check");
      return false;
    }

    const bool is_obstacle_found =
      algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));

    if (is_obstacle_found) {
      ++obstacle_detect_consecutive_count_;
    } else {
      obstacle_detect_consecutive_count_ = 0;
    }

    if (is_obstacle_found) {
      // 아직 확정 임계치 미도달
      if (obstacle_detect_consecutive_count_ < obstacle_detect_confirm_count_) {
        RCLCPP_INFO(get_logger(),
          "[isPlanRequired] Obstacle detected but need %zu/%zu consecutive frames",
          obstacle_detect_consecutive_count_, obstacle_detect_confirm_count_);
        return false;
      }

      const double dt = (now_time - last_replan_time_).seconds();
      if (dt < min_obstacle_replan_interval_sec_) {
        RCLCPP_INFO(get_logger(),
          "[isPlanRequired] Obstacle detected but in cooldown (%.2f < %.2f s)",
          dt, min_obstacle_replan_interval_sec_);
        return false;
      }

      const double progress_dist = distance_from_last_replan();
      if (progress_dist < min_replan_progress_distance_m_) {
        RCLCPP_INFO(get_logger(),
          "[isPlanRequired] Obstacle detected but insufficient progress since last replan (%.2f < %.2f m)",
          progress_dist, min_replan_progress_distance_m_);
        return false;
      }

      last_replan_reason_ = "obstacle";
      RCLCPP_WARN(get_logger(),
        "[isPlanRequired] Obstacle confirmed (consecutive=%zu) -> replan",
        obstacle_detect_consecutive_count_);
      return true;
    }
  }

  if (replan_when_course_out_) {
    // Compute distance both for the full trajectory and the actually published partial trajectory
    // If frame differs, transform ego pose to trajectory frame when TF is available
    auto ego_pose_for_dist = current_pose_.pose;
    double dist_full = std::numeric_limits<double>::infinity();
    double dist_partial = std::numeric_limits<double>::infinity();

    // Try TF transform to trajectory frame (full)
    if (!trajectory_.header.frame_id.empty() &&
        current_pose_.header.frame_id != trajectory_.header.frame_id) {
      const auto tf = getTransform(trajectory_.header.frame_id, current_pose_.header.frame_id);
      if (!tf.header.frame_id.empty()) {
        ego_pose_for_dist = transformPose(current_pose_.pose, tf);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[isPlanRequired] Failed to transform ego pose to trajectory frame (%s -> %s).",
          current_pose_.header.frame_id.c_str(), trajectory_.header.frame_id.c_str());
        ego_pose_for_dist = current_pose_.pose; // fallback without transform
      }
    }

    if (!trajectory_.points.empty()) {
      dist_full = calcDistance2d(trajectory_, ego_pose_for_dist);
    }
    if (!partial_trajectory_.points.empty()) {
      // Ensure pose is in partial trajectory frame as well (it should match full, but be safe)
      auto ego_pose_for_partial = current_pose_.pose;
      if (!partial_trajectory_.header.frame_id.empty() &&
          current_pose_.header.frame_id != partial_trajectory_.header.frame_id) {
        const auto tfp = getTransform(partial_trajectory_.header.frame_id, current_pose_.header.frame_id);
        if (!tfp.header.frame_id.empty()) {
          ego_pose_for_partial = transformPose(current_pose_.pose, tfp);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[isPlanRequired] Missing transform to partial trajectory frame (%s -> %s).",
            current_pose_.header.frame_id.c_str(), partial_trajectory_.header.frame_id.c_str());
        }
      }
      dist_partial = calcDistance2d(partial_trajectory_, ego_pose_for_partial);
    }

    const double dist_eval = std::min(dist_full, dist_partial);
    if (dist_eval > th_course_out_distance_m_) {
      // simple cooldown to avoid oscillation
      const double dt = (now_time - last_replan_time_).seconds();
      if (dt < min_course_out_replan_interval_sec_) {
        RCLCPP_INFO(get_logger(),
          "[isPlanRequired] Course-out detected but in cooldown (%.2f < %.2f s)",
          dt, min_course_out_replan_interval_sec_);
        return false;
      }
      last_replan_reason_ = "course_out";
      RCLCPP_WARN(get_logger(),
        "[isPlanRequired] Vehicle deviated from path (full=%.2f, partial=%.2f, th=%.2f) -> replan",
        std::isfinite(dist_full) ? dist_full : -1.0,
        std::isfinite(dist_partial) ? dist_partial : -1.0,
        th_course_out_distance_m_);
      return true;
    }
  }

  return false;
}

void BirdroPlanner::updateTargetIndex()
{
  const auto is_near_target =
    autoware::universe_utils::calcDistance2d(
      trajectory_.points.at(target_index_), current_pose_) < th_arrived_distance_m_;

  if (is_near_target && isStopped()) {
    const auto new_target_index =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);

    if (new_target_index == target_index_) {
      // Finished publishing all partial trajectories
      is_completed_ = true;
      if (!goal_pose_.header.frame_id.empty() && use_occlusion_marker_) {
        RCLCPP_INFO(get_logger(), "Goal reached. Publishing occlusion zone marker.");
        publishOcclusionMarker(goal_pose_.pose);
      } else {
        RCLCPP_INFO(get_logger(), "Goal reached.");
      }
      goal_pose_ = PoseStamped();  // Clear goal pose
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
      std_msgs::msg::Bool is_completed_msg;
      is_completed_msg.data = is_completed_;
      parking_state_pub_->publish(is_completed_msg);
    } else {
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void BirdroPlanner::on_timer()
{
  // --- Runtime Speed Guard ---
  evaluateSpeedGuard();
  if (sg_enable_) {
    if (sg_in_safety_stop_) {
      if (now() < sg_safety_stop_until_) {
        if (sg_mode_ == 0) {
          auto stop_traj = createStopTrajectory(sg_freeze_active_ ? sg_freeze_pose_ : current_pose_);
          stop_traj.header.stamp = now();
          publishTrajectory(stop_traj);
          debug_partial_pose_array_pub_->publish(PoseArray());
          return;
        } else if (sg_mode_ == 1) {
          if (!sg_last_valid_traj_.points.empty()) {
            auto hold = sg_last_valid_traj_;
            hold.header.stamp = now();
            publishTrajectory(hold);
            debug_partial_pose_array_pub_->publish(trajectory2PoseArray(hold));
            return;
          } else {
            auto stop = createStopTrajectory(current_pose_);
            stop.header.stamp = now();
            publishTrajectory(stop);
            return;
          }
        }
      } else {
        sg_in_safety_stop_ = false;
        RCLCPP_INFO(get_logger(), "[SpeedGuard] Safety stop released");
        // mode 0: 해제 후에도 freeze 유지 → 여기서는 아무 것도 하지 않음
      }
    }
  }

  // --- mode 0 freeze 상태이면 계속 정지 유지 & 재계획 차단 ---
  if (sg_mode_ == 0 && sg_freeze_active_) {
    auto stop_traj = createStopTrajectory(sg_freeze_pose_);
    stop_traj.header.stamp = now();
    // mirror 퍼블리시 일관성 유지
    publishTrajectory(stop_traj);
    debug_partial_pose_array_pub_->publish(PoseArray());
    // goal 이 새로 들어오기 전까지 아무 것도 하지 않음
    return;
  }

  // Check required inputs are ready (single grid)
  if (!occupancy_grid_ || goal_pose_.header.frame_id.empty() || !odom_) {
    // is_completed_가 true가 된 후 goal_pose_가 비워지므로,
    // 이 상태에서는 타이머를 계속 실행하도록 아래 로직으로 이동
    if (!is_completed_) {
      return;
    }
  }
  if (is_completed_) {
    // 주차가 완료되었으면 정지 트랙토리만 발행하고 마커 관리 로직으로 넘어감
    if (odom_) {
      // frame_id는 기존 경로 프레임을 유지하고 타임스탬프만 최신화
      partial_trajectory_.header.stamp = odom_->header.stamp;
    }
    const auto stop_trajectory = createStopTrajectory(partial_trajectory_);
    publishTrajectory(stop_trajectory);
  } else {
    // 주행 중일 때의 로직 (기존 로직과 거의 동일)
    if (current_pose_.header.frame_id == "") {
      return;
    }

    const bool is_reset_required = !reset_in_progress_ && isPlanRequired();
    if (is_reset_required) {
      if (!is_new_parking_cycle_) {
        const auto stop_trajectory = partial_trajectory_.points.empty()
                                      ? createStopTrajectory(current_pose_)
                                      : createStopTrajectory(partial_trajectory_);
        publishTrajectory(stop_trajectory);
        debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
        debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
      }
      cacheStopHoldTrajectory();
      reset();
      // 기존: 항상 stop 대기 -> 변경: 파라미터에 따라 즉시 계획
      reset_in_progress_ = require_stop_before_planning_;
      RCLCPP_INFO(get_logger(), "Initiating replanning sequence. Reason=%s", last_replan_reason_.c_str());

      if (!require_stop_before_planning_) {
        planTrajectory();
        reset_in_progress_ = false;
      }
    }

    if (reset_in_progress_) {
      const bool stopped = isStopped();
      if (stopped) {
        if (has_pending_goal_pose_) {
          goal_pose_ = pending_goal_pose_;
          has_pending_goal_pose_ = false;
        }
        waiting_for_new_goal_stop_ = false;
        planTrajectory();
        RCLCPP_INFO(get_logger(), "New trajectory planned (points=%zu)", trajectory_.points.size());
        reset_in_progress_ = false;
      } else {
        if (waiting_for_new_goal_stop_) {
          Trajectory base_for_stop;
          if (has_stop_hold_base_traj_ && !stop_hold_base_traj_.points.empty()) {
            base_for_stop = stop_hold_base_traj_;
          } else if (!partial_trajectory_.points.empty()) {
            base_for_stop = partial_trajectory_;
          }

          auto stop_trajectory = base_for_stop.points.empty()
                                  ? createStopTrajectory(current_pose_)
                                  : createStopTrajectory(base_for_stop);
          stop_trajectory.header.stamp = now();
          publishTrajectory(stop_trajectory);
          debug_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
          debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_trajectory));
          partial_trajectory_ = stop_trajectory;
          if (!has_stop_hold_base_traj_ && !base_for_stop.points.empty()) {
            stop_hold_base_traj_ = base_for_stop;
            has_stop_hold_base_traj_ = true;
          }
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Waiting for stop before planning (require_stop_before_planning=true).");
      }
    }

    if (trajectory_.points.size() <= 1) {
      is_new_parking_cycle_ = false;
      return;
    }

    // Clamp start index to nearest to ego before extracting partial (extra safety in dynamic scenes)
    if (!trajectory_.points.empty()) {
      const auto ego_in_traj_frame = transformPoseToFrame(current_pose_, trajectory_.header.frame_id);
      const size_t nearest_idx =
        autoware::motion_utils::findNearestIndex(trajectory_.points, ego_in_traj_frame.position);
      if (nearest_idx > prev_target_index_) {
        prev_target_index_ = nearest_idx;
        if (target_index_ < prev_target_index_) {
          target_index_ = getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);
        }
      }
    }

    updateTargetIndex();
    partial_trajectory_ = getPartialTrajectory(trajectory_, prev_target_index_, target_index_);
    // 발행 직전 타임스탬프 갱신
    partial_trajectory_.header.stamp = now();

    // Safety guard: avoid publishing a trajectory segment too far from ego
    if (replan_when_course_out_) {
      // compute distance to partial trajectory in its frame
      auto ego_pose_for_partial = current_pose_.pose;
      if (!partial_trajectory_.header.frame_id.empty() &&
          current_pose_.header.frame_id != partial_trajectory_.header.frame_id) {
        const auto tfp = getTransform(partial_trajectory_.header.frame_id, current_pose_.header.frame_id);
        if (!tfp.header.frame_id.empty()) {
          ego_pose_for_partial = transformPose(current_pose_.pose, tfp);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[on_timer] Missing transform to partial trajectory frame (%s -> %s).",
            current_pose_.header.frame_id.c_str(), partial_trajectory_.header.frame_id.c_str());
        }
      }
      if (!partial_trajectory_.points.empty()) {
        const double d = calcDistance2d(partial_trajectory_, ego_pose_for_partial);
        const double hard_th = th_course_out_distance_m_ + course_out_publish_margin_m_;
        if (d > hard_th + course_out_publish_epsilon_m_) {
          last_replan_reason_ = "course_out";
          RCLCPP_WARN(get_logger(),
            "[on_timer] Partial trajectory far from ego (%.2f > %.2f=th+margin+eps). Publish STOP and trigger replan.",
            d, hard_th + course_out_publish_epsilon_m_);
          // Publish stop to avoid validator error, then request replanning on next cycle
          auto stop_traj = createStopTrajectory(current_pose_);
          stop_traj.header.stamp = now();
          publishTrajectory(stop_traj);
          debug_partial_pose_array_pub_->publish(trajectory2PoseArray(stop_traj));
          // Force immediate plan if allowed
          const double dt = (now() - last_replan_time_).seconds();
          if (!reset_in_progress_ && dt > min_course_out_replan_interval_sec_) {
            cacheStopHoldTrajectory();
            reset();
            reset_in_progress_ = require_stop_before_planning_;
            if (!require_stop_before_planning_) {
              planTrajectory();
              reset_in_progress_ = false;
            }
          }
          is_new_parking_cycle_ = false;
          return;
        } else if (d > th_course_out_distance_m_) {
          // Slightly over threshold but within margin: allow publish to avoid oscillation
          RCLCPP_INFO(get_logger(),
            "[on_timer] Partial trajectory slightly far (%.2f > %.2f, margin=%.2f). Allow publish.",
            d, th_course_out_distance_m_, course_out_publish_margin_m_);
        }
      }
    }

  publishTrajectory(partial_trajectory_);
    debug_pose_array_pub_->publish(trajectory2PoseArray(trajectory_));
    debug_partial_pose_array_pub_->publish(trajectory2PoseArray(partial_trajectory_));
  }


  is_new_parking_cycle_ = false;
}

void BirdroPlanner::planTrajectory()
{
  if (!occupancy_grid_) {
    RCLCPP_WARN(get_logger(), "planTrajectory called but occupancy grid is null");
    return;
  }

  // Provide robot shape and map for the planner
  // NDZ overlay is handled by external overlay node; consume grid as-is
  algo_->setMap(*occupancy_grid_);

  // Calculate poses in costmap frame
  const auto current_to_costmap_tf =
    getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id);
  if (current_to_costmap_tf.header.frame_id.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to lookup transform. from=%s, to=%s",
      occupancy_grid_->header.frame_id.c_str(),
      current_pose_.header.frame_id.c_str());
    return;
  }

  const auto goal_to_costmap_tf =
    getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id);
  if (goal_to_costmap_tf.header.frame_id.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to lookup transform. from=%s, to=%s",
      occupancy_grid_->header.frame_id.c_str(),
      goal_pose_.header.frame_id.c_str());
    return;
  }

  const auto current_pose_in_costmap_frame =
    transformPose(current_pose_.pose, current_to_costmap_tf);

  const auto goal_pose_in_costmap_frame = transformPose(goal_pose_.pose, goal_to_costmap_tf);

  RCLCPP_INFO(get_logger(), "Planning from (%.2f, %.2f) to (%.2f, %.2f) in frame '%s'",
              current_pose_in_costmap_frame.position.x, current_pose_in_costmap_frame.position.y,
              goal_pose_in_costmap_frame.position.x, goal_pose_in_costmap_frame.position.y,
              occupancy_grid_->header.frame_id.c_str());

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  std::string error_msg;
  bool result = false;
  try {
    result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  } catch (const std::exception & e) {
    error_msg = e.what();
  }
  const rclcpp::Time end = get_clock()->now();

  const double planning_time = (end - start).seconds();
  RCLCPP_INFO(get_logger(), "Freespace planning finished in %.3f s (result=%s)", planning_time, result?"success":"fail");

  if (result) {
    RCLCPP_INFO(get_logger(), "Goal reachable. Building trajectory...");
    trajectory_ = createTrajectory(current_pose_, algo_->getWaypoints(), waypoints_velocity_);
    reversing_indices_ = getReversingIndices(trajectory_);
    // Anchor start index to the nearest point to ego to avoid far-start publish
    const auto ego_in_traj_frame = transformPoseToFrame(current_pose_, trajectory_.header.frame_id);
    const size_t nearest_index_full =
      autoware::motion_utils::findNearestIndex(trajectory_.points, ego_in_traj_frame.position);
    prev_target_index_ = nearest_index_full;
    target_index_ = getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

    // 속도 램프/방향 전환 0속도 관통 적용
    const double v0_abs = odom_ ? std::abs(odom_->twist.twist.linear.x) : 0.0;
    applySpeedRampProfile(trajectory_, v0_abs);

    const double traj_length = computeTrajectoryLength(trajectory_);
    RCLCPP_INFO(get_logger(), "Trajectory generated: points=%zu, reversing_points=%zu, length=%.2f m",
                trajectory_.points.size(), reversing_indices_.size(), traj_length);
    last_replan_time_ = now();
    last_replan_pose_ = current_pose_.pose;

  
    obstacle_detect_consecutive_count_ = 0;
    has_stop_hold_base_traj_ = false;
  } else {
    RCLCPP_WARN(get_logger(), "Failed to plan to goal: %s", error_msg.c_str());
    reset();
  }
}

void BirdroPlanner::reset()
{
  trajectory_ = Trajectory();
  partial_trajectory_ = Trajectory();
  is_completed_ = false;
  obstacle_detect_consecutive_count_ = 0;
  std_msgs::msg::Bool is_completed_msg;
  is_completed_msg.data = is_completed_;
  parking_state_pub_->publish(is_completed_msg);

  // Publish empty PoseArray to clear visualization in RViz
  debug_pose_array_pub_->publish(PoseArray());
  debug_partial_pose_array_pub_->publish(PoseArray());
  //is_occlusion_marker_published_ = false;
}

TransformStamped BirdroPlanner::getTransform(const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

Pose BirdroPlanner::transformPoseToFrame(
  const PoseStamped & pose_stamped, const std::string & target_frame)
{
  if (target_frame.empty() || pose_stamped.header.frame_id == target_frame) {
    return pose_stamped.pose;
  }

  if (pose_stamped.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[transformPoseToFrame] Source pose frame is empty while converting to %s",
      target_frame.c_str());
    return pose_stamped.pose;
  }

  const auto tf = getTransform(target_frame, pose_stamped.header.frame_id);
  if (tf.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[transformPoseToFrame] Missing transform (%s -> %s)",
      pose_stamped.header.frame_id.c_str(), target_frame.c_str());
    return pose_stamped.pose;
  }

  return transformPose(pose_stamped.pose, tf);
}

void BirdroPlanner::init_algo_parameter()
{
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  vehicle_shape_.length = vehicle_info.vehicle_length_m;
  vehicle_shape_.width = vehicle_info.vehicle_width_m;
  vehicle_shape_.base_length = vehicle_info.wheel_base_m;
  vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
  vehicle_shape_.base2back = vehicle_info.rear_overhang_m;

  // Extend robot shape
  autoware::freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  extended_vehicle_shape.length += vehicle_shape_margin_m_;
  extended_vehicle_shape.width += vehicle_shape_margin_m_;
  extended_vehicle_shape.base2back += vehicle_shape_margin_m_ / 2;

  const auto planner_common_param = get_planner_common_param();

  // initialize specified algorithm
  if (algorithm_name_ == "astar") {
    algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else if (algorithm_name_ == "rrtstar") {
    algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algorithm_name_ + " exists.");
  }
  RCLCPP_INFO(get_logger(), "Initialized planning algorithm: %s", algorithm_name_.c_str());
}

PlannerCommonParam BirdroPlanner::get_planner_common_param()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit", 30000.0);

  p.theta_size = declare_parameter<int>("theta_size", 120);
  p.angle_goal_range = declare_parameter<double>("angle_goal_range", 6.0);
  p.curve_weight = declare_parameter<double>("curve_weight", 0.5);
  p.reverse_weight = declare_parameter<double>("reverse_weight", 1.0);
  p.direction_change_weight = declare_parameter<double>("direction_change_weight", 1.5);
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range", 0.5);
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range", 1.0);
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio", 0.5);
  p.turning_steps = declare_parameter<int>("turning_steps", 1);

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold", 100);

  return p;
}

bool BirdroPlanner::isStopped()
{
  for (const auto & odom : odom_buffer_) {
    if (std::abs(odom->twist.twist.linear.x) > th_stopped_velocity_mps_) {
      return false;
    }
  }
  return true;
}

double BirdroPlanner::computeTrajectoryLength(const Trajectory & traj) const
{
  if (traj.points.size() < 2) return 0.0;
  double length = 0.0;
  for (size_t i = 1; i < traj.points.size(); ++i) {
    length += autoware::universe_utils::calcDistance2d(traj.points[i - 1], traj.points[i]);
  }
  return length;
}

// --- Speed Guard helper implementations ---
void BirdroPlanner::speedGuardEnterSafetyStop(const char * reason)
{
  sg_safety_stop_until_ = now() + rclcpp::Duration::from_seconds(sg_stop_duration_sec_);
  sg_in_safety_stop_ = true;
  RCLCPP_WARN(get_logger(), "[SpeedGuard] Enter safety stop (%.2fs) reason=%s mode=%d",
    sg_stop_duration_sec_, reason, sg_mode_);

  if (sg_mode_ == 0) {
    // mode 0: 제자리에 정지
    // Freeze: 현재 위치 저장 + 기존 경로 폐기 → 재사용 방지
    sg_freeze_pose_ = current_pose_;
    sg_freeze_active_ = true;

    trajectory_ = Trajectory();
    partial_trajectory_ = Trajectory();
    reversing_indices_.clear();
    prev_target_index_ = 0;
    target_index_ = 0;
  } else {
    // mode 1: 마지막 정상 궤적 유지
    if (!trajectory_.points.empty()) {
      sg_last_valid_traj_ = trajectory_;
    }
  }
}

void BirdroPlanner::evaluateSpeedGuard()
{
  if (!sg_enable_ || !odom_) return;
  const double v = odom_->twist.twist.linear.x;
  double accel = 0.0;
  if (sg_last_eval_time_.nanoseconds() != 0) {
    const double dt = (now() - sg_last_eval_time_).seconds();
    if (dt > 1e-3) accel = (v - sg_last_speed_mps_) / dt;
  }
  const double yaw_rate = odom_->twist.twist.angular.z;
  const double lateral_acc = std::abs(v * yaw_rate);
  bool violation = false;
  if (std::abs(v) > sg_max_speed_mps_) violation = true;
  if (accel > sg_max_accel_mps2_) violation = true;
  if (lateral_acc > sg_max_lateral_accel_mps2_) violation = true;
  if (violation) ++sg_current_violation_count_; else sg_current_violation_count_ = 0;
  if (violation) {
    RCLCPP_INFO(get_logger(), "[SpeedGuard] v=%.2f accel=%.2f lat=%.2f viol=%d/%d",
      v, accel, lateral_acc, sg_current_violation_count_, sg_trigger_consecutive_);
  }
  if (sg_current_violation_count_ >= sg_trigger_consecutive_) {
    speedGuardEnterSafetyStop("speed_threshold");
    sg_current_violation_count_ = 0;
  }
  sg_last_speed_mps_ = v;
  sg_last_eval_time_ = now();
}

void BirdroPlanner::applySpeedRampProfile(Trajectory & traj, double v0_abs)
{
  if (!ramp_enable_ || traj.points.size() < 2) return;

  const size_t N = traj.points.size();
  // 1) 누적 거리 계산
  std::vector<double> s(N, 0.0);
  for (size_t i = 1; i < N; ++i) {
    s[i] = s[i - 1] + autoware::universe_utils::calcDistance2d(traj.points[i - 1], traj.points[i]);
  }

  // 2) 부호, 목표 크기(기존 크루즈) 추출
  std::vector<int> sign(N, 1);
  std::vector<double> v_des(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    const double v_raw = traj.points[i].longitudinal_velocity_mps;
    sign[i] = (v_raw >= 0.0) ? 1 : -1;
    v_des[i] = std::abs(v_raw);
  }

  // 3) 방향 전환 지점(부호 flip)에서 0 속도 강제
  //    reversing_indices_는 이미 최신화되어 있음
  for (const auto idx : reversing_indices_) {
    if (idx < N) v_des[idx] = 0.0;
  }

  // 4) 시작부가 현재 진행 방향과 반대이면, 일정 거리(게이트)까지 0 속도 강제
  if (ramp_reverse_gate_enable_ && v0_abs > ramp_reverse_gate_speed_thresh_mps_) {
    // 시작부 의도된 부호(첫 포인트의 sign)와 현재 이동 부호(속도 부호)를 비교
    // 주행 중 부호는 알 수 없으므로 v0_abs만 사용하고, "첫 목표 부호가 음수"면 게이트 실행
    if (sign.front() < 0) {
      for (size_t i = 0; i < N; ++i) {
        if (s[i] <= ramp_reverse_gate_stop_distance_m_) {
          v_des[i] = 0.0;
        } else {
          break;
        }
      }
    }
  }

  // 5) 전방 패스 (가속 제한, v^2 = v0^2 + 2 a s)
  std::vector<double> v_fwd(N, 0.0);
  v_fwd[0] = std::min(v_des[0], v0_abs);
  for (size_t i = 1; i < N; ++i) {
    const double ds = std::max(0.0, s[i] - s[i - 1]);
    const double v_lim = std::sqrt(std::max(0.0, v_fwd[i - 1] * v_fwd[i - 1] + 2.0 * ramp_max_accel_mps2_ * ds));
    v_fwd[i] = std::min(v_des[i], v_lim);
  }

  // 6) 후방 패스 (감속 제한, 끝점 0속도)
  std::vector<double> v_bwd = v_fwd;
  v_bwd[N - 1] = 0.0;
  for (size_t i = N - 1; i >= 1; --i) {
    const double ds = std::max(0.0, s[i] - s[i - 1]);
    const double v_prev_lim = std::sqrt(std::max(0.0, v_bwd[i] * v_bwd[i] + 2.0 * ramp_max_decel_mps2_ * ds));
    v_bwd[i - 1] = std::min(v_bwd[i - 1], v_prev_lim);
    if (i == 1) break; // size_t 언더플로우 방지
  }

  // 7) 최종 속도 적용 (기존 부호 유지)
  for (size_t i = 0; i < N; ++i) {
    traj.points[i].longitudinal_velocity_mps = sign[i] * v_bwd[i];
  }
}


}  // namespace birdro_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(birdro_planner::BirdroPlanner)
