# Birdro Planner

자유공간(freespace) 경로 계획을 수행하는 ROS 2 컴포넌트 노드입니다. A* 또는 RRT* 기반 알고리즘을 사용하여 목표 지점까지의 궤적을 생성하고, 후진 전환 구간별 부분 궤적을 발행합니다. 런타임 안전 속도 가드(SpeedGuard), 속도 램프 프로파일, 장애물/이탈 기반 재계획, 목표/가림 구역 마커 시각화를 지원합니다.

- 소스: [src/birdro_planner.cpp](/home/ok/skyautonet_birdro/ros/src/skyautonet/autonomous_system/planning/birdro_planner/src/birdro_planner.cpp)
- 노드 클래스: `birdro_planner::BirdroPlanner` (Composable Component)
- 기본 노드명: `birdro_planner_node`

## 빌드

- ROS 2 Humble + colcon 환경

```bash
colcon build --packages-select birdro_planner
source install/setup.bash
```

## 실행

- 컴포넌트 컨테이너로 로드

```bash
# 다중 스레드 컨테이너 실행
ros2 run rclcpp_components component_container_mt --ros-args \
  -p algorithm_name:=astar \
  -p update_rate:=10.0
```

```bash
# 실행 중 컨테이너에 컴포넌트 로드
ros2 component load /ComponentManager birdro_planner birdro_planner::BirdroPlanner
```

- 매개변수 YAML 사용

```bash
ros2 run rclcpp_components component_container_mt --ros-args \
  --params-file /path/to/birdro_planner.param.yaml
```

## 토픽 인터페이스

구독(Subscriptions)
- ~/input/route (autoware_lanelet2_msgs/LaneletRoute) – 목표 포즈 포함 경로 입력
- ~/input/occupancy_grid (nav_msgs/OccupancyGrid) – 비용맵 입력
- ~/input/odometry (nav_msgs/Odometry) – 자차 상태
- /planning/mission_planning/checkpoint (geometry_msgs/PoseStamped) – 목표 포즈
- /initialpose (geometry_msgs/PoseWithCovarianceStamped) – 초기 위치(수신 시 목표/상태 리셋)

발행(Publications)
- ~/output/trajectory (autoware Trajectory) – 계획 궤적
- /planning/scenario_planning/lane_driving/trajectory (autoware Trajectory) – mirror 퍼블리시(옵션)
- /planning/scenario_planning/parking/is_completed (std_msgs/Bool) – 완료 상태
- ~/debug/pose_array (geometry_msgs/PoseArray) – 전체 궤적 디버그
- ~/debug/partial_pose_array (geometry_msgs/PoseArray) – 부분 궤적 디버그
- ~/debug/occlusion_zone (visualization_msgs/Marker) – 가림(occlusion) 영역 마커
- ~/debug/goal_marker (visualization_msgs/Marker) – 목표 지점 마커

## 주요 동작

- 부분 궤적 발행: 후진 전환 인덱스 단위로 궤적을 나누어 발행합니다. 각 세그먼트의 시작점 속도는 다음 점 속도로, 끝점 속도는 0으로 보정합니다.
- 목표 도달: 목표 도달 및 정지 시 완료 플래그를 발행하고 가림 마커를 표시합니다.
- 재계획 트리거
  - 장애물 기반: 전방 부분 궤적 상 장애물 검출 연속 프레임 임계치, 최소 간격, 진행 거리 조건을 만족 시 재계획합니다.
  - 이탈(course-out) 기반: 궤적(전체/부분)에서 이탈 거리 임계 초과 시 재계획합니다. 하드 마진 초과 시 STOP 궤적을 우선 발행하여 안전하게 전환합니다.
- SpeedGuard: 속도/가속/횡가속을 모니터링하여 위반 연속 시 안전 정지를 수행합니다.
  - mode 0: 현 위치에서 정지(freeze). 새 goal 수신 전까지 재계획/발행 차단.
  - mode 1: 마지막 정상 궤적을 유지(hold)하거나 STOP 발행.
- Velocity Ramp: 최대 가감속 제약으로 속도 프로파일을 재구성하고, 방향 전환 지점에서 0속도를 강제합니다. 높은 속도로 진행 중 반대 방향 시작 시 게이트 구간(stop 거리)까지 0속도를 부여합니다.
- TF/프레임: 비용맵 프레임으로 현재/목표 포즈를 변환해 계획합니다. 발행/거리 계산 시 프레임이 다르면 TF 변환을 사용합니다.

자세한 로직은 [src/birdro_planner.cpp](/home/ok/skyautonet_birdro/ros/src/skyautonet/autonomous_system/planning/birdro_planner/src/birdro_planner.cpp)를 참고하세요.

## 파라미터

핵심
- algorithm_name: "astar" | "rrtstar"
- cruise_velocity: 기본 주행 속도 [m/s]
- update_rate: 타이머 주기 [Hz]
- vehicle_shape_margin_m: 로봇 형상 여유(m)

완료/정지 판정
- th_arrived_distance_m: 목표 도달 거리 임계
- th_stopped_time_sec: 정지 판정 시간 윈도우
- th_stopped_velocity_mps: 정지 판정 속도 임계

재계획
- replan_when_obstacle_found: 장애물 기반 재계획 사용
- obstacle_detect_confirm_count: 장애물 연속 검출 필요 프레임 수
- obstacle_replan_min_interval_sec: 장애물 기반 재계획 최소 간격
- min_replan_progress_distance_m: 직전 재계획 대비 최소 진행 거리
- replan_when_course_out: 이탈 기반 재계획 사용
- th_course_out_distance_m: 이탈 거리 임계
- course_out.publish_margin_m: 발행 허용 마진
- course_out.replan_min_interval_sec: 이탈 재계획 최소 간격
- course_out.publish_epsilon_m: 발행 히스테리시스

계획 전 정지
- require_stop_before_planning: true면 계획 전 완전 정지 필요

미러/시각화
- mirror_to_main_trajectory: 메인 궤적 토픽으로 미러 발행
- use_occlusion_marker: 목표 도달 시 가림 마커 표시
- visualize_goal_marker: 목표 마커 표시

SpeedGuard
- speed_guard.enable
- speed_guard.mode: 0=freeze, 1=hold
- speed_guard.max_speed_mps
- speed_guard.max_accel_mps2
- speed_guard.max_lateral_accel_mps2
- speed_guard.trigger_consecutive: 연속 위반 임계카운트
- speed_guard.stop_duration_sec: 안전정지 유지 시간

Velocity Ramp
- ramp.enable
- ramp.max_accel_mps2
- ramp.max_decel_mps2
- ramp.reverse_gate.enable
- ramp.reverse_gate.speed_thresh_mps
- ramp.reverse_gate.stop_distance_m

플래너 공통 파라미터(기본값 포함)
- time_limit: 30000.0
- theta_size: 120
- angle_goal_range: 6.0
- curve_weight: 0.5
- reverse_weight: 1.0
- direction_change_weight: 1.5
- lateral_goal_range: 0.5
- longitudinal_goal_range: 1.0
- max_turning_ratio: 0.5
- turning_steps: 1
- obstacle_threshold: 100

## 예시 파라미터(YAML)

```yaml
birdro_planner:
  ros__parameters:
    algorithm_name: astar
    update_rate: 10.0
    cruise_velocity: 1.0
    vehicle_shape_margin_m: 0.2

    # stop/arrive
    th_arrived_distance_m: 0.5
    th_stopped_time_sec: 1.0
    th_stopped_velocity_mps: 0.05

    # replanning
    replan_when_obstacle_found: true
    obstacle_detect_confirm_count: 3
    obstacle_replan_min_interval_sec: 1.0
    min_replan_progress_distance_m: 0.5

    replan_when_course_out: true
    th_course_out_distance_m: 0.8
    course_out.publish_margin_m: 0.5
    course_out.replan_min_interval_sec: 1.0
    course_out.publish_epsilon_m: 0.2

    require_stop_before_planning: false

    # speed guard
    speed_guard.enable: true
    speed_guard.mode: 0
    speed_guard.max_speed_mps: 2.0
    speed_guard.max_accel_mps2: 1.5
    speed_guard.max_lateral_accel_mps2: 1.0
    speed_guard.trigger_consecutive: 3
    speed_guard.stop_duration_sec: 1.0

    # velocity ramp
    ramp.enable: true
    ramp.max_accel_mps2: 1.0
    ramp.max_decel_mps2: 1.0
    ramp.reverse_gate.enable: true
    ramp.reverse_gate.speed_thresh_mps: 0.3
    ramp.reverse_gate.stop_distance_m: 1.0

    # visualize/mirror
    mirror_to_main_trajectory: false
    use_occlusion_marker: true
    visualize_goal_marker: true

    # planner common
    time_limit: 30000.0
    theta_size: 120
    angle_goal_range: 6.0
    curve_weight: 0.5
    reverse_weight: 1.0
    direction_change_weight: 1.5
    lateral_goal_range: 0.5
    longitudinal_goal_range: 1.0
    max_turning_ratio: 0.5
    turning_steps: 1
    obstacle_threshold: 100
```

## 프레임/TF 요구사항

- 비용맵 프레임 ← 현재 포즈 프레임, 목표 포즈 프레임 변환이 가능해야 합니다.
- TF 조회 실패 시 계획/발행이 제한될 수 있습니다.

## 트러블슈팅

- OccupancyGrid 준비 전: 장애물 검사/계획이 지연될 수 있음.
- TF 변환 실패: 프레임명 확인 및 TF 브로드캐스터 상태 점검.
- SpeedGuard freeze(mode 0): 새 goal 수신 전까지 STOP 유지 및 재계획 차단됨.
- 초기포즈 수신: 현재 목표/궤적이 리셋됩니다.

---
참고: 구현 상세 및 최신 파라미터는 소스 [src/birdro_planner.cpp](/home/ok/skyautonet_birdro/ros/src/skyautonet/autonomous_system/planning/birdro_planner/src/birdro_planner.cpp)를 확인하세요.
