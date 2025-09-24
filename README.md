# birdro_planner

birdro_planner는 Autoware freespace planner(A* / RRT*)를 사용해 주차·저속 환경에서 점-목표(goal)에 대한 궤적을 생성하는 ROS 2 패키지입니다. 추가로, OSM에 정의한 No Driving Zone(NDZ) 폴리곤을 점유 그리드에 오버레이하는 보조 노드(ndz_costmap_overlay)를 포함합니다.

- 플래너 노드: [`src/birdro_planner.cpp`](src/birdro_planner.cpp), [`include/birdro_planner.hpp`](include/birdro_planner.hpp)
- NDZ 오버레이 노드: [`src/ndz_costmap_overlay.cpp`](src/ndz_costmap_overlay.cpp)
- 런치 파일: [`launch/birdro_planner.launch.xml`](launch/birdro_planner.launch.xml)

## 주요 기능

- OccupancyGrid 기반 freespace 경로 계획 (A* 또는 RRT*)
- NDZ(진입 금지 구역) 폴리곤 오버레이로 경로 생성 시 해당 영역 회피
- 속도/가속/횡가속 실시간 가드(SpeedGuard)
  - 모드 0: 안전정지 + 위치 얼림(freeze)
  - 모드 1: 마지막 정상 궤적 유지
- 방향 전환 구간 0속도 강제 및 가속/감속 램프 적용
- 장애물/코스이탈에 따른 재계획 디바운싱과 쿨다운

## 빌드

- Linux + ROS 2 + Autoware Universe 환경을 가정합니다.
- 의존: autoware.universe(freespace_planning_algorithms 등), tf2, visualization_msgs, tinyxml2, lanelet2, geography_utils, tier4_map_msgs 등.

```bash
# 워크스페이스 루트(예: /home/ok/skyautonet_birdro/ros)에서
colcon build --packages-select birdro_planner
source install/setup.bash
```

## 실행

런치 파일(플래너 + NDZ 오버레이 동시 실행)을 권장합니다.

```bash
ros2 launch birdro_planner birdro_planner.launch.xml
```

- 플래너는 NDZ가 반영된 그리드로 자동 리맵됩니다.
- Autoware freespace planner 파라미터/차량 파라미터 파일 경로는 런치 인자를 통해 설정됩니다.

단일 노드로 실행하려면:

```bash
# NDZ 오버레이
ros2 run birdro_planner ndz_costmap_overlay --ros-args \
  -p input_topic:=/planning/scenario_planning/parking/costmap_generator/occupancy_grid \
  -p output_topic:=/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz \
  -p map_topic:=/perception/occupancy_grid_map/map \
  -p ndz.enable:=true \
  -p ndz.osm_path:=/path/to/lanelet2_map.osm \
  -p ndz.way_id:=-10000

# 플래너
ros2 run birdro_planner birdro_planner_node --ros-args \
  --remap ~/input/occupancy_grid:=/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz \
  --remap ~/input/odometry:=/localization/kinematic_state \
  --remap ~/input/vector_map:=/map/vector_map \
  --remap ~/output/trajectory:=/planning/scenario_planning/parking/trajectory
```

참고: 제공 런치 파일은 위 리맵/파라미터를 이미 포함합니다. 파일 참조: [`launch/birdro_planner.launch.xml`](launch/birdro_planner.launch.xml)

## 토픽 I/O

### birdro_planner_node
- Subscriptions
  - `~/input/route` (autoware_planning_msgs/LaneletRoute, transient_local)
  - `~/input/occupancy_grid` (nav_msgs/OccupancyGrid)
  - `~/input/odometry` (nav_msgs/Odometry)
  - `/planning/mission_planning/checkpoint` (geometry_msgs/PoseStamped, 목표)
  - `/initialpose` (geometry_msgs/PoseWithCovarianceStamped)
- Publications
  - `~/output/trajectory` (autoware_planning_msgs/Trajectory)
  - 선택: `/planning/scenario_planning/lane_driving/trajectory` (mirror, `mirror_to_main_trajectory:=true`일 때)
  - `/planning/scenario_planning/parking/is_completed` (std_msgs/Bool)
  - `~/debug/pose_array`, `~/debug/partial_pose_array` (geometry_msgs/PoseArray)
  - `~/debug/goal_marker`, `~/debug/occlusion_zone` (visualization_msgs/Marker)

### ndz_costmap_overlay
- Subscriptions
  - `input_topic` (nav_msgs/OccupancyGrid, 동적/로컬 코스트맵)
  - `map_topic` (nav_msgs/OccupancyGrid, 정적 맵, 선택)
  - `/map/map_projector_info` (tier4_map_msgs/MapProjectorInfo, transient_local)
- Publications
  - `output_topic` (nav_msgs/OccupancyGrid, NDZ 반영)
  - `~/ndz_polygon` (visualization_msgs/Marker)

## 주요 파라미터

### 플래너 노드(birdro_planner_node)
- `algorithm_name` ["astar" | "rrtstar"] (기본 "astar")
- `cruise_velocity` [m/s] (기본 5.0/3.6)
- `update_rate` [Hz] (기본 10.0)
- 도착/정지 판정: `th_arrived_distance_m`, `th_stopped_time_sec`, `th_stopped_velocity_mps`
- 코스 이탈/재계획: `th_course_out_distance_m`, `course_out.publish_margin_m`, `course_out.replan_min_interval_sec`, `course_out.publish_epsilon_m`
- 장애물 재계획: `replan_when_obstacle_found`, `obstacle_detect_confirm_count`, `obstacle_replan_min_interval_sec`, `min_replan_progress_distance_m`
- 정지 후 계획 여부: `require_stop_before_planning` (true면 정지 후 계획)
- 차체 여유: `vehicle_shape_margin_m`
- 목표 마커: `visualize_goal_marker`
- 미러 퍼블리시: `mirror_to_main_trajectory` (lane_driving/trajectory로 사본 발행)
- 속도 램프:
  - `ramp.enable`, `ramp.max_accel_mps2`, `ramp.max_decel_mps2`
  - `ramp.reverse_gate.enable`, `ramp.reverse_gate.speed_thresh_mps`, `ramp.reverse_gate.stop_distance_m`
- SpeedGuard:
  - `speed_guard.enable`, `speed_guard.max_speed_mps`, `speed_guard.max_accel_mps2`, `speed_guard.max_lateral_accel_mps2`
  - `speed_guard.trigger_consecutive`, `speed_guard.stop_duration_sec`, `speed_guard.mode` (0=정지/프리즈, 1=마지막 정상 궤적 유지)

상세 구현은 [`src/birdro_planner.cpp`](src/birdro_planner.cpp), [`include/birdro_planner.hpp`](include/birdro_planner.hpp) 참고.

### NDZ 오버레이(ndz_costmap_overlay)
- I/O
  - `input_topic` (기본 `/planning/scenario_planning/parking/costmap_generator/occupancy_grid`)
  - `output_topic` (기본 `/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz`)
  - `map_topic` (기본 `/perception/occupancy_grid_map/map`)
  - `free_padding_margin_m` [m] (그리드 경계 외부로 free padding 확장)
- NDZ 정의
  - `ndz.enable` (기본 true)
  - `ndz.osm_path` (lanelet2 OSM 파일 경로)
  - `ndz.way_id` (폴리곤을 구성하는 OSM way ID)
  - `ndz.ref_lat`, `ndz.ref_lon` (나이브 투영시 기준점)
  - `ndz.offset_x_m`, `ndz.offset_y_m` (오프셋)

동작 요약: OSM에서 way의 노드들을 읽어 폴리곤 생성 → `/map/map_projector_info`로부터 projector를 수신해 동일 투영계로 좌표 변환 → 현재 OccupancyGrid 메타(해상도/원점/회전)에 맞춰 마스크 생성 → NDZ 내부 셀을 점유(100)로 승격 → 원본(-1) 셀은 Free(0)로 변환하여 goal 유효성 문제 방지 → 동적/정적 그리드를 max 합성 → 필요 시 free padding으로 경계 확장. 구현: [`src/ndz_costmap_overlay.cpp`](src/ndz_costmap_overlay.cpp)

## NDZ OSM 작성 가이드

- NDZ 폴리곤을 구성할 OSM way를 선택/생성하고 해당 `way_id`를 파라미터로 지정합니다.
- 맵 projector가 `LOCAL`인 경우, 각 노드에 `local_x`/`local_y` 태그가 있으면 이를 우선 사용합니다. 태그가 없으면 ref_lat/lon 기반 나이브 투영으로 폴리곤을 구성합니다.
- projector 타입/원점은 `/map/map_projector_info`에서 수신합니다. 메시지 정의는 Autoware Universe의 tier4_map_msgs 참고.

RViz에서 `~/ndz_polygon` 마커로 폴리곤을 확인할 수 있습니다.

## 예시 파라미터/런치

런치 파일: [`launch/birdro_planner.launch.xml`](launch/birdro_planner.launch.xml)

핵심 부분:
```xml
<node pkg="birdro_planner" exec="ndz_costmap_overlay" name="ndz_costmap_overlay">
  <param name="input_topic" value="/planning/scenario_planning/parking/costmap_generator/occupancy_grid"/>
  <param name="output_topic" value="/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz"/>
  <param name="map_topic" value="/perception/occupancy_grid_map/map"/>
  <param name="ndz.enable" value="true"/>
  <param name="ndz.osm_path" value="/path/to/lanelet2_map.osm"/>
  <param name="ndz.way_id" value="-10000"/>
</node>

<node pkg="birdro_planner" exec="birdro_planner_node" name="birdro_planner">
  <remap from="~/input/occupancy_grid" to="/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz"/>
  <remap from="~/input/odometry" to="/localization/kinematic_state"/>
  <remap from="~/input/vector_map" to="/map/vector_map"/>
  <remap from="~/output/trajectory" to="/planning/scenario_planning/parking/trajectory"/>
  <param name="visualize_goal_marker" value="true"/>
  <param name="use_occlusion_marker" value="false"/>
  <param name="require_stop_before_planning" value="true"/>
  <!-- SpeedGuard/Ramp 파라미터 등 추가 가능 -->
</node>
```

## 동작 팁

- TF 변환 실패 시 플래너가 계획을 중단합니다. OccupancyGrid/odometry/goal의 frame_id와 TF를 확인하세요.
- NDZ가 의도와 다르게 보이면 projector 정보(`/map/map_projector_info`)와 OSM local_x/local_y 태그를 점검하세요.
- SpeedGuard 위반 시 안전정지/프리즈가 발생합니다. 파라미터로 임계값 및 동작 모드를 조정하세요.
- Unknown(-1) 셀은 Free(0)로 변환됩니다. 맵 생성 파이프라인에서 Unknown을 사용하는 경우 이 특성을 고려하세요.

## 라이선스
본 패키지의 라이선스는 상위 리포지터리 정책을 따릅니다.
