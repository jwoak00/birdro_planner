#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tinyxml2.h>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <memory>

// Autoware map projector interfaces
#include <tier4_map_msgs/msg/map_projector_info.hpp>
#include <geography_utils/lanelet2_projector.hpp>
#include <lanelet2_io/Projection.h>

using tier4_map_msgs::msg::MapProjectorInfo;

namespace
{
double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

struct GridRotation
{
  double cos_yaw{1.0};
  double sin_yaw{0.0};
};

GridRotation makeRotation(const geometry_msgs::msg::Quaternion & q)
{
  const double yaw = quaternionToYaw(q);
  return GridRotation{std::cos(yaw), std::sin(yaw)};
}

bool isSameMeta(const nav_msgs::msg::MapMetaData & a, const nav_msgs::msg::MapMetaData & b)
{
  return a.width == b.width && a.height == b.height && a.resolution == b.resolution &&
         a.origin.position.x == b.origin.position.x && a.origin.position.y == b.origin.position.y &&
         a.origin.orientation.x == b.origin.orientation.x &&
         a.origin.orientation.y == b.origin.orientation.y &&
         a.origin.orientation.z == b.origin.orientation.z &&
         a.origin.orientation.w == b.origin.orientation.w;
}

bool worldToGridIndex(
  const nav_msgs::msg::MapMetaData & meta, const GridRotation & rot, const double wx, const double wy,
  size_t & ix, size_t & iy)
{
  const double dx = wx - meta.origin.position.x;
  const double dy = wy - meta.origin.position.y;
  const double local_x = rot.cos_yaw * dx + rot.sin_yaw * dy;
  const double local_y = -rot.sin_yaw * dx + rot.cos_yaw * dy;

  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }

  const double gx = local_x / meta.resolution;
  const double gy = local_y / meta.resolution;

  if (gx < 0.0 || gy < 0.0 || gx >= static_cast<double>(meta.width) ||
      gy >= static_cast<double>(meta.height)) {
    return false;
  }

  ix = static_cast<size_t>(gx);
  iy = static_cast<size_t>(gy);
  return true;
}
}  // namespace

class NDZCostmapOverlay : public rclcpp::Node {
public:
  NDZCostmapOverlay() : Node("ndz_costmap_overlay") {
    // Parameters
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/planning/scenario_planning/parking/costmap_generator/occupancy_grid");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz");
    map_topic_ = declare_parameter<std::string>(
      "map_topic", "/perception/occupancy_grid_map/map");
    free_padding_margin_m_ = declare_parameter<double>("free_padding_margin_m", 5.0);

    // NDZ params (same as planner)
    ndz_enable_ = declare_parameter<bool>("ndz.enable", true);
    ndz_osm_path_ = declare_parameter<std::string>("ndz.osm_path", "");
    ndz_way_id_ = static_cast<long>(declare_parameter<int64_t>("ndz.way_id", -10000));
    ndz_ref_lat_ = declare_parameter<double>("ndz.ref_lat", 0.0);
    ndz_ref_lon_ = declare_parameter<double>("ndz.ref_lon", 0.0);
    ndz_offset_x_m_ = declare_parameter<double>("ndz.offset_x_m", 0.0);
    ndz_offset_y_m_ = declare_parameter<double>("ndz.offset_y_m", 0.0);

    // pubs/subs
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, rclcpp::QoS(1),
      std::bind(&NDZCostmapOverlay::onGrid, this, std::placeholders::_1));
    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/ndz_polygon", rclcpp::QoS(1).transient_local());

    if (!map_topic_.empty()) {
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&NDZCostmapOverlay::onMap, this, std::placeholders::_1));
    }

    // Subscribe projector info (transient local so late joiners get it)
    map_proj_sub_ = create_subscription<MapProjectorInfo>(
      "/map/map_projector_info", rclcpp::QoS(1).transient_local(),
      std::bind(&NDZCostmapOverlay::onProjectorInfo, this, std::placeholders::_1));

    if (ndz_enable_) loadNoDrivingZoneFromOSM();
    RCLCPP_INFO(
      get_logger(), "NDZCostmapOverlay started. in=%s map=%s out=%s",
      input_topic_.c_str(), map_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onProjectorInfo(const MapProjectorInfo::ConstSharedPtr msg) {
    projector_info_ = *msg;
    have_projector_info_ = true;
    projector_ = geography_utils::get_lanelet2_projector(projector_info_);
  RCLCPP_INFO(get_logger(), "Received map projector info (type=%s). Reloading NDZ polygon with projector...",
        projector_info_.projector_type.c_str());
    // Reload polygon with the projector to ensure perfect alignment
    loadNoDrivingZoneFromOSM();
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    last_dynamic_grid_ = *msg;
    have_dynamic_grid_ = true;

    nav_msgs::msg::OccupancyGrid out;
    if (!composeOutput(out)) {
      return;
    }
    pub_->publish(out);
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    map_grid_ = *msg;
    have_map_ = true;

    nav_msgs::msg::OccupancyGrid out;
    if (!composeOutput(out)) {
      return;
    }
    pub_->publish(out);
  }

  void publishPolygonMarker() {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "ndz";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.2;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.8f;
    for (const auto & p : ndz_polygon_map_frame_) m.points.push_back(p);
    if (!ndz_polygon_map_frame_.empty()) m.points.push_back(ndz_polygon_map_frame_.front());
    m.lifetime = rclcpp::Duration(0,0);
    marker_pub_->publish(m);
  }

  void loadNoDrivingZoneFromOSM() {
    ndz_polygon_map_frame_.clear();
    if (ndz_osm_path_.empty()) {
      RCLCPP_WARN(get_logger(), "NDZ: osm_path is empty");
      return;
    }
    tinyxml2::XMLDocument doc;
    const auto rc = doc.LoadFile(ndz_osm_path_.c_str());
    if (rc != tinyxml2::XML_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "NDZ: failed to load OSM: %s", ndz_osm_path_.c_str());
      return;
    }
    auto osm = doc.FirstChildElement("osm");
    if (!osm) { RCLCPP_ERROR(get_logger(), "NDZ: invalid OSM (no <osm>)"); return; }
    struct NodeInfo { double lat{0.0}; double lon{0.0}; double local_x{0.0}; double local_y{0.0}; bool has_local{false}; };
    std::unordered_map<long, NodeInfo> nodes; // id -> node info
    for (auto n = osm->FirstChildElement("node"); n; n = n->NextSiblingElement("node")) {
      long id=0; double lat=0.0, lon=0.0; double local_x=0.0, local_y=0.0; bool has_local=false;
      n->QueryInt64Attribute("id", &id);
      n->QueryDoubleAttribute("lat", &lat);
      n->QueryDoubleAttribute("lon", &lon);
      // Read optional <tag k="local_x" v="..."/> and <tag k="local_y" v="..."/>
      for (auto tag = n->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
        const char * k = tag->Attribute("k");
        const char * v = tag->Attribute("v");
        if (!k || !v) continue;
        if (std::strcmp(k, "local_x") == 0) { local_x = std::atof(v); has_local = true; }
        else if (std::strcmp(k, "local_y") == 0) { local_y = std::atof(v); has_local = true; }
      }
      nodes[id] = NodeInfo{lat, lon, local_x, local_y, has_local};
    }
    std::vector<long> refs;
    for (auto w = osm->FirstChildElement("way"); w; w = w->NextSiblingElement("way")) {
      long wid=0; w->QueryInt64Attribute("id", &wid);
      if (wid != ndz_way_id_) continue;
      for (auto nd = w->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
        long ref=0; nd->QueryInt64Attribute("ref", &ref); refs.push_back(ref);
      }
      break;
    }
    if (refs.size() < 3) {
      RCLCPP_WARN(get_logger(), "NDZ: way %ld not found or too few points", ndz_way_id_);
      return;
    }
    // Fallback ref lat/lon for naive projection
    double ref_lat = ndz_ref_lat_;
    double ref_lon = ndz_ref_lon_;
    if (ref_lat == 0.0 && ref_lon == 0.0) {
      const auto it = nodes.find(refs.front());
      if (it != nodes.end()) { ref_lat = it->second.lat; ref_lon = it->second.lon; }
    }
    const double deg2rad = M_PI / 180.0;
    const double R = 6378137.0;
    const double cos_lat = std::cos(ref_lat * deg2rad);
    std::vector<geometry_msgs::msg::Point> poly;
    const bool use_local = have_projector_info_ && projector_info_.projector_type == MapProjectorInfo::LOCAL;
    for (auto rid : refs) {
      const auto it = nodes.find(rid);
      if (it == nodes.end()) continue;
      geometry_msgs::msg::Point p;
      if (have_projector_info_ && projector_) {
        if (use_local && it->second.has_local) {
          p.x = it->second.local_x + ndz_offset_x_m_;
          p.y = it->second.local_y + ndz_offset_y_m_;
          p.z = 0.0;
        } else if (use_local && !it->second.has_local) {
          // LOCAL projector without local tags in OSM; fallback to naive projection
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "NDZ: LOCAL projector but node %ld lacks local_x/local_y; using naive projection.", rid);
          p.x = (it->second.lon - ref_lon) * deg2rad * R * cos_lat + ndz_offset_x_m_;
          p.y = (it->second.lat - ref_lat) * deg2rad * R + ndz_offset_y_m_;
          p.z = 0.0;
        } else {
          // Project lat/lon using the same projector as lanelet2_map_loader
          lanelet::GPSPoint gps;
          gps.lat = it->second.lat;
          gps.lon = it->second.lon;
          gps.ele = 0.0;
          const auto p3 = projector_->forward(gps);
          p.x = p3.x() + ndz_offset_x_m_;
          p.y = p3.y() + ndz_offset_y_m_;
          p.z = 0.0;
        }
      } else {
        // Fallback: naive equirectangular projection
        p.x = (it->second.lon - ref_lon) * deg2rad * R * cos_lat + ndz_offset_x_m_;
        p.y = (it->second.lat - ref_lat) * deg2rad * R + ndz_offset_y_m_;
        p.z = 0.0;
      }
      poly.push_back(p);
    }
    if (poly.size() >= 3) {
      ndz_polygon_map_frame_ = poly;
      RCLCPP_INFO(get_logger(), "NDZ: polygon loaded (%zu vertices)", poly.size());
      // Invalidate mask to rebuild with updated polygon on the next grid
      last_grid_meta_ = nav_msgs::msg::MapMetaData();
      publishPolygonMarker();
    }
  }

  static bool pointInPolygon(double x, double y, const std::vector<geometry_msgs::msg::Point> & poly) {
    if (poly.size() < 3) return false;
    bool inside = false;
    size_t j = poly.size() - 1;
    for (size_t i = 0; i < poly.size(); ++i) {
      const auto & pi = poly[i];
      const auto & pj = poly[j];
      const bool inter = ((pi.y > y) != (pj.y > y)) &&
                         (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x);
      if (inter) {
        inside = !inside;
      }
      j = i;
    }
    return inside;
  }

  void buildMask(const nav_msgs::msg::MapMetaData & meta) {
    last_grid_meta_ = meta;
    ndz_mask_.assign(meta.width * meta.height, 0);
    // Account for grid orientation (yaw)
    const auto & o = meta.origin;
    const auto rot = makeRotation(o.orientation);
    for (size_t y = 0; y < meta.height; ++y) {
      for (size_t x = 0; x < meta.width; ++x) {
        const double cell_x = (static_cast<double>(x) + 0.5) * meta.resolution;
        const double cell_y = (static_cast<double>(y) + 0.5) * meta.resolution;
        // Rotate cell center by yaw and translate by origin position
        const double wx = o.position.x + (rot.cos_yaw * cell_x - rot.sin_yaw * cell_y);
        const double wy = o.position.y + (rot.sin_yaw * cell_x + rot.cos_yaw * cell_y);
        if (pointInPolygon(wx, wy, ndz_polygon_map_frame_)) {
          ndz_mask_[y * meta.width + x] = 1;
        }
      }
    }
    // RCLCPP_INFO(get_logger(), "NDZ: mask built for grid %ux%u (res=%.2f)", meta.width, meta.height, meta.resolution);
  }

  void applyNDZ(nav_msgs::msg::OccupancyGrid & grid) {
    if (!ndz_enable_ || ndz_polygon_map_frame_.size() < 3) {
      return;
    }
    if (!isSameMeta(grid.info, last_grid_meta_)) {
      buildMask(grid.info);
    }
    if (ndz_mask_.empty()) {
      return;
    }
    const size_t limit = std::min(grid.data.size(), ndz_mask_.size());
    for (size_t i = 0; i < limit; ++i) {
      if (ndz_mask_[i]) {
        grid.data[i] = std::max<int8_t>(grid.data[i], 100);
      }
    }
  }

  void fuseOccupancy(
    nav_msgs::msg::OccupancyGrid & target, const nav_msgs::msg::OccupancyGrid & source) const
  {
    if (source.data.empty()) {
      return;
    }

    const auto source_rot = makeRotation(source.info.origin.orientation);
    const auto target_rot = makeRotation(target.info.origin.orientation);

    for (size_t y = 0; y < source.info.height; ++y) {
      for (size_t x = 0; x < source.info.width; ++x) {
        const auto idx = y * source.info.width + x;
        const int8_t val = source.data.at(idx);
        if (val < 0) {
          continue;
        }

        const double cell_x = (static_cast<double>(x) + 0.5) * source.info.resolution;
        const double cell_y = (static_cast<double>(y) + 0.5) * source.info.resolution;
        const double wx = source.info.origin.position.x +
                          (source_rot.cos_yaw * cell_x - source_rot.sin_yaw * cell_y);
        const double wy = source.info.origin.position.y +
                          (source_rot.sin_yaw * cell_x + source_rot.cos_yaw * cell_y);

        size_t ix = 0;
        size_t iy = 0;
        if (!worldToGridIndex(target.info, target_rot, wx, wy, ix, iy)) {
          continue;
        }

        const size_t target_idx = iy * target.info.width + ix;
        if (target_idx >= target.data.size()) {
          continue;
        }
        target.data[target_idx] = std::max<int8_t>(target.data[target_idx], val);
      }
    }
  }

bool composeOutput(nav_msgs::msg::OccupancyGrid & out) {
  if (!have_map_ && !have_dynamic_grid_) {
    return false;
  }

    if (have_map_) {
      out = map_grid_;
    } else {
      out = last_dynamic_grid_;
    }

    applyNDZ(out);

  if (have_map_ && have_dynamic_grid_) {
    fuseOccupancy(out, last_dynamic_grid_);
  }

  // Unknown(-1) 셀은 Free(0)로 변환해 탐색 알고리즘이 goal을 유효하다고 판단하도록 한다
  for (auto & cell : out.data) {
    if (cell < 0) {
      cell = 0;
    }
  }

  padFreeMargin(out, free_padding_margin_m_);

  out.header.stamp = now();
  return true;
}

  void padFreeMargin(nav_msgs::msg::OccupancyGrid & grid, const double margin_m) const
  {
    if (margin_m <= 0.0) {
      return;
    }
    const double resolution = grid.info.resolution;
    if (resolution <= 0.0) {
      return;
    }

    const size_t padding_cells = static_cast<size_t>(std::ceil(margin_m / resolution));
    if (padding_cells == 0) {
      return;
    }

    const size_t old_width = grid.info.width;
    const size_t old_height = grid.info.height;
    const size_t new_width = old_width + 2 * padding_cells;
    const size_t new_height = old_height + 2 * padding_cells;

    std::vector<int8_t> padded(new_width * new_height, 0);
    for (size_t y = 0; y < old_height; ++y) {
      for (size_t x = 0; x < old_width; ++x) {
        const size_t from_index = y * old_width + x;
        const size_t to_index = (y + padding_cells) * new_width + (x + padding_cells);
        padded[to_index] = grid.data[from_index];
      }
    }

    const auto rot = makeRotation(grid.info.origin.orientation);
    const double shift = static_cast<double>(padding_cells) * resolution;
    grid.info.origin.position.x += (-rot.cos_yaw + rot.sin_yaw) * shift;
    grid.info.origin.position.y += (-rot.sin_yaw - rot.cos_yaw) * shift;

    grid.info.width = static_cast<uint32_t>(new_width);
    grid.info.height = static_cast<uint32_t>(new_height);
    grid.data = std::move(padded);
  }

private:
  // params
  std::string input_topic_;
  std::string output_topic_;
  std::string map_topic_;
  double free_padding_margin_m_{5.0};
  bool ndz_enable_{true};
  std::string ndz_osm_path_{};
  long ndz_way_id_{-10000};
  double ndz_ref_lat_{0.0};
  double ndz_ref_lon_{0.0};
  double ndz_offset_x_m_{0.0};
  double ndz_offset_y_m_{0.0};
  // state
  std::vector<geometry_msgs::msg::Point> ndz_polygon_map_frame_;
  std::vector<uint8_t> ndz_mask_;
  nav_msgs::msg::MapMetaData last_grid_meta_{};
  nav_msgs::msg::OccupancyGrid map_grid_{};
  nav_msgs::msg::OccupancyGrid last_dynamic_grid_{};
  bool have_map_{false};
  bool have_dynamic_grid_{false};
  // projector state
  MapProjectorInfo projector_info_{};
  bool have_projector_info_{false};
  std::unique_ptr<lanelet::Projector> projector_{};
  // io
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<MapProjectorInfo>::SharedPtr map_proj_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NDZCostmapOverlay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
