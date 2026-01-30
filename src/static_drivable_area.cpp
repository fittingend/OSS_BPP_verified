#include "static_drivable_area.hpp"

#include "data_manager.hpp"
#include "path_utils.hpp"
#include "trajectory.hpp"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
namespace
{
const std::vector<autoware::common_types::DrivableLanes> * g_drivable_lanes = nullptr;

bool isAlmostSame(
  const autoware::common_types::PointXYZ & a, const autoware::common_types::PointXYZ & b)
{
  constexpr double eps = 1.0e-4;
  return std::fabs(a.x - b.x) < eps && std::fabs(a.y - b.y) < eps && std::fabs(a.z - b.z) < eps;
}

void appendLineString(
  const lanelet::ConstLineString3d & line, bool skip_first,
  std::vector<autoware::common_types::PointXYZ> & out, double lateral_offset_sign,
  double lateral_offset)
{
  if (line.empty()) {
    return;
  }

  for (size_t i = skip_first ? 1 : 0; i < line.size(); ++i) {
    autoware::common_types::PointXYZ point;
    point.x = line[i].x();
    point.y = line[i].y() + lateral_offset * lateral_offset_sign;
    point.z = line[i].z();

    if (!out.empty() && isAlmostSame(out.back(), point)) {
      continue;
    }
    out.push_back(point);
  }
}
}  // namespace

namespace
{
using autoware::common_types::DrivableLanes;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::PointXYZ;
using autoware::common_types::Pose;
using autoware::common_types::Position;

bool checkHasSameLane(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelet & target_lane);

PointXYZ makePointXYZ(const lanelet::ConstPoint3d & p)
{
  PointXYZ out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

double distance2d(const PointXYZ & a, const PointXYZ & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double distance2d(
  const autoware::common_types::Point & a, const PointXYZ & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double distance2d(
  const autoware::common_types::Point & a, const autoware::common_types::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double distance3d(const PointXYZ & a, const PointXYZ & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

PointXYZ lerpPoint(const PointXYZ & a, const PointXYZ & b, double ratio)
{
  PointXYZ out;
  out.x = a.x + (b.x - a.x) * ratio;
  out.y = a.y + (b.y - a.y) * ratio;
  out.z = a.z + (b.z - a.z) * ratio;
  return out;
}

PointXYZ toPointXYZ(const autoware::common_types::Point & point)
{
  PointXYZ out;
  out.x = point.x;
  out.y = point.y;
  out.z = point.z;
  return out;
}

PointXYZ toPointXYZ(const autoware::common_types::Position & position)
{
  PointXYZ out;
  out.x = position.x;
  out.y = position.y;
  out.z = position.z;
  return out;
}

double normalizeYaw(double yaw)
{
  constexpr double pi = M_PI;
  while (yaw > pi) yaw -= 2.0 * pi;
  while (yaw < -pi) yaw += 2.0 * pi;
  return yaw;
}

double getYawFromPose(const Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double calcSquaredDistance2d(const PointXYZ & a, const autoware::common_types::Position & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double distance2d(
  const autoware::common_types::Position & a, const PointXYZ & b)
{
  return distance2d(toPointXYZ(a), b);
}

double calcSquaredDistance2d(
  const autoware::common_types::Point & a, const autoware::common_types::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

std::vector<PointXYZ> extractCenterlinePoints(const PathWithLaneId & path)
{
  std::vector<PointXYZ> points;
  points.reserve(path.points.size());
  for (const auto & point : path.points) {
    points.push_back(toPointXYZ(point.point.pose.position));
  }
  return points;
}

std::vector<double> computeArcLengths(const std::vector<PointXYZ> & points)
{
  std::vector<double> arc;
  arc.reserve(points.size());
  double accum = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i == 0) {
      arc.push_back(0.0);
      continue;
    }
    accum += distance2d(points[i - 1], points[i]);
    arc.push_back(accum);
  }
  return arc;
}

double projectPointOntoPolylineS(
  const std::vector<PointXYZ> & polyline, const std::vector<double> & arc, const PointXYZ & point)
{
  if (polyline.size() < 2 || polyline.size() != arc.size()) {
    return 0.0;
  }

  double best_s = 0.0;
  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto & p0 = polyline[i];
    const auto & p1 = polyline[i + 1];
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double seg_len2 = vx * vx + vy * vy;
    if (seg_len2 < 1.0e-9) {
      continue;
    }
    const double wx = point.x - p0.x;
    const double wy = point.y - p0.y;
    double t = (vx * wx + vy * wy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);
    const double proj_x = p0.x + t * vx;
    const double proj_y = p0.y + t * vy;
    const double dx = point.x - proj_x;
    const double dy = point.y - proj_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      const double seg_s = arc[i + 1] - arc[i];
      best_s = arc[i] + t * seg_s;
    }
  }
  return best_s;
}

double projectPointOntoLineStringS(
  const lanelet::ConstLineString3d & line, const autoware::common_types::Position & position)
{
  if (line.size() < 2) {
    return 0.0;
  }

  double accum = 0.0;
  double best_s = 0.0;
  double best_dist2 = std::numeric_limits<double>::max();
  const double px = position.x;
  const double py = position.y;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto p0 = line[i].basicPoint2d();
    const auto p1 = line[i + 1].basicPoint2d();
    const double vx = p1.x() - p0.x();
    const double vy = p1.y() - p0.y();
    const double seg_len2 = vx * vx + vy * vy;
    if (seg_len2 < 1.0e-9) {
      continue;
    }
    const double wx = px - p0.x();
    const double wy = py - p0.y();
    double t = (vx * wx + vy * wy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);
    const double proj_x = p0.x() + t * vx;
    const double proj_y = p0.y() + t * vy;
    const double dx = px - proj_x;
    const double dy = py - proj_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      const double seg_len = std::sqrt(seg_len2);
      best_s = accum + t * seg_len;
    }
    accum += std::sqrt(seg_len2);
  }

  return best_s;
}

size_t findNearestLaneletIndex(
  const std::vector<DrivableLanes> & lanes, const Pose & pose)
{
  if (lanes.empty()) {
    return 0;
  }
  size_t best_index = 0;
  double best_dist2 = std::numeric_limits<double>::max();
  const auto target = toPointXYZ(pose.position);

  for (size_t i = 0; i < lanes.size(); ++i) {
    auto centerline = lanes[i].left_lane.centerline();
    if (centerline.empty()) {
      centerline = lanes[i].right_lane.centerline();
    }
    if (centerline.empty()) {
      continue;
    }
    for (const auto & pt : centerline) {
      const double dx = pt.x() - target.x;
      const double dy = pt.y() - target.y;
      const double dist2 = dx * dx + dy * dy;
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best_index = i;
      }
    }
  }
  return best_index;
}

double calcLateralOffset(
  const PointXYZ & p0, const PointXYZ & p1, const autoware::common_types::Position & target)
{
  const double vx = p1.x - p0.x;
  const double vy = p1.y - p0.y;
  const double wx = target.x - p0.x;
  const double wy = target.y - p0.y;
  const double len = std::sqrt(vx * vx + vy * vy);
  if (len < 1e-6) {
    return 0.0;
  }
  return ((vx * wy) - (vy * wx)) / len;
}

double calcLongitudinalOffsetToSegment(
  const std::vector<PointXYZ> & bound, size_t seg_idx,
  const autoware::common_types::Position & target)
{
  if (bound.size() < 2) {
    return 0.0;
  }
  if (seg_idx >= bound.size() - 1) {
    seg_idx = bound.size() - 2;
  }

  const auto & p0 = bound[seg_idx];
  const auto & p1 = bound[seg_idx + 1];
  const double vx = p1.x - p0.x;
  const double vy = p1.y - p0.y;
  const double len2 = vx * vx + vy * vy;
  if (len2 < 1e-9) {
    return 0.0;
  }
  const double wx = target.x - p0.x;
  const double wy = target.y - p0.y;
  const double ratio = (wx * vx + wy * vy) / len2;
  return ratio * std::sqrt(len2);
}

double calcLongitudinalOffsetToSegment(
  const std::vector<PointXYZ> & bound, size_t seg_idx,
  const autoware::common_types::Point & target)
{
  autoware::common_types::Position pos;
  pos.x = target.x;
  pos.y = target.y;
  pos.z = target.z;
  return calcLongitudinalOffsetToSegment(bound, seg_idx, pos);
}

PointXYZ calcLongitudinalOffsetPoint(
  const std::vector<PointXYZ> & bound, size_t seg_idx, double target_length)
{
  if (bound.empty()) {
    return PointXYZ{};
  }
  if (bound.size() == 1) {
    return bound.front();
  }
  if (seg_idx >= bound.size() - 1) {
    seg_idx = bound.size() - 2;
  }

  if (target_length >= 0.0) {
    double remaining = target_length;
    size_t idx = seg_idx;
    while (idx < bound.size() - 1) {
      const auto & p0 = bound[idx];
      const auto & p1 = bound[idx + 1];
      const double seg_len = distance2d(p0, p1);
      if (remaining <= seg_len) {
        const double ratio = (seg_len < 1e-9) ? 0.0 : remaining / seg_len;
        return lerpPoint(p0, p1, ratio);
      }
      remaining -= seg_len;
      ++idx;
    }
    return bound.back();
  }

  double remaining = -target_length;
  size_t idx = seg_idx;
  while (idx > 0) {
    const auto & p0 = bound[idx];
    const auto & p1 = bound[idx - 1];
    const double seg_len = distance2d(p0, p1);
    if (remaining <= seg_len) {
      const double ratio = (seg_len < 1e-9) ? 0.0 : remaining / seg_len;
      return lerpPoint(p0, p1, ratio);
    }
    remaining -= seg_len;
    --idx;
  }
  return bound.front();
}

PointXYZ calcLongitudinalOffsetStartPoint(
  const std::vector<PointXYZ> & bound, const Pose & pose, const size_t nearest_seg_idx,
  const double offset)
{
  const double base =
    calcLongitudinalOffsetToSegment(bound, nearest_seg_idx, pose.position);
  return calcLongitudinalOffsetPoint(bound, nearest_seg_idx, base + offset);
}

PointXYZ calcLongitudinalOffsetGoalPoint(
  const std::vector<PointXYZ> & bound, const Pose & pose, const size_t nearest_seg_idx,
  const double offset)
{
  const double base =
    calcLongitudinalOffsetToSegment(bound, nearest_seg_idx, pose.position);
  return calcLongitudinalOffsetPoint(bound, nearest_seg_idx + 1, base + offset);
}

PointXYZ projectOntoLaneBoundary(
  const lanelet::ConstLineString3d & line, const autoware::common_types::Position & position)
{
  PointXYZ projected = toPointXYZ(position);
  if (line.size() < 2) {
    return projected;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto & p0 = line[i];
    const auto & p1 = line[i + 1];
    const double vx = p1.x() - p0.x();
    const double vy = p1.y() - p0.y();
    const double vz = p1.z() - p0.z();
    const double len2 = vx * vx + vy * vy + vz * vz;
    if (len2 < 1.0e-9) {
      continue;
    }
    const double wx = position.x - p0.x();
    const double wy = position.y - p0.y();
    const double ratio = std::clamp((wx * vx + wy * vy) / len2, 0.0, 1.0);
    const double proj_x = p0.x() + ratio * vx;
    const double proj_y = p0.y() + ratio * vy;
    const double proj_z = p0.z() + ratio * vz;
    const double dx = proj_x - position.x;
    const double dy = proj_y - position.y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      projected.x = proj_x;
      projected.y = proj_y;
      projected.z = proj_z;
    }
  }
  return projected;
}

PointXYZ weightedAverageBoundaryPoints(
  const std::vector<PointXYZ> & candidates, const PointXYZ & reference)
{
  if (candidates.empty()) {
    return reference;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  constexpr double kMinDistance = 0.1;
  double total_weight = 0.0;
  PointXYZ accumulated{0.0, 0.0, 0.0};
  for (const auto & pt : candidates) {
    const double dist = std::hypot(pt.x - reference.x, pt.y - reference.y);
    const double weight = 1.0 / std::max(dist, kMinDistance);
    accumulated.x += pt.x * weight;
    accumulated.y += pt.y * weight;
    accumulated.z += pt.z * weight;
    total_weight += weight;
  }

  if (total_weight < 1.0e-6) {
    return candidates.front();
  }
  accumulated.x /= total_weight;
  accumulated.y /= total_weight;
  accumulated.z /= total_weight;
  return accumulated;
}

size_t findNearestSegmentIndexFromLateralDistance(
  const std::vector<PointXYZ> & points, const Pose & pose, const double yaw_threshold)
{
  if (points.size() < 2) {
    return 0;
  }

  const double base_yaw = getYawFromPose(pose);
  std::optional<size_t> closest_idx;
  double min_lateral = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points[i];
    const auto & p1 = points[i + 1];
    const double seg_yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    const double yaw_diff = normalizeYaw(seg_yaw - base_yaw);
    if (std::abs(yaw_diff) > yaw_threshold) {
      continue;
    }

    const double lon = calcLongitudinalOffsetToSegment(points, i, pose.position);
    const double seg_len = distance2d(p0, p1);
    double lat = 0.0;
    if (lon < 0.0) {
      lat = distance2d(pose.position, p0);
    } else if (lon > seg_len) {
      lat = distance2d(pose.position, p1);
    } else {
      lat = std::abs(calcLateralOffset(p0, p1, pose.position));
    }

    if (lat < min_lateral) {
      min_lateral = lat;
      closest_idx = i;
    }
  }

  if (closest_idx) {
    return *closest_idx;
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const double dist = std::sqrt(calcSquaredDistance2d(points[i], pose.position));
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

size_t findNearestSegmentIndexFromLateralDistance(
  const std::vector<PointXYZ> & points, const autoware::common_types::Point & target_point)
{
  if (points.size() < 2) {
    return 0;
  }

  std::optional<size_t> closest_idx;
  double min_lateral = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & p0 = points[i];
    const auto & p1 = points[i + 1];
    const double lon = calcLongitudinalOffsetToSegment(points, i, target_point);
    const double seg_len = distance2d(p0, p1);
    double lat = 0.0;
    if (lon < 0.0) {
      lat = distance2d(target_point, p0);
    } else if (lon > seg_len) {
      lat = distance2d(target_point, p1);
    } else {
      autoware::common_types::Position pos;
      pos.x = target_point.x;
      pos.y = target_point.y;
      pos.z = target_point.z;
      lat = std::abs(calcLateralOffset(p0, p1, pos));
    }

    if (lat < min_lateral) {
      min_lateral = lat;
      closest_idx = i;
    }
  }

  if (closest_idx) {
    return *closest_idx;
  }
  return 0;
}

std::vector<lanelet::ConstPoint3d> extractBoundFromPolygon(
  const lanelet::ConstPolygon3d & polygon, const size_t start_idx, const size_t end_idx,
  const bool clockwise)
{
  std::vector<lanelet::ConstPoint3d> ret;
  if (polygon.empty()) {
    return ret;
  }

  auto advance = [&](size_t idx) -> size_t {
    if (clockwise) {
      return (idx + 1) % polygon.size();
    }
    return (idx == 0) ? polygon.size() - 1 : idx - 1;
  };

  size_t idx = start_idx;
  ret.push_back(polygon[idx]);
  while (idx != end_idx) {
    idx = advance(idx);
    ret.push_back(polygon[idx]);
  }
  return ret;
}


std::vector<lanelet::ConstPoint3d> buildLaneletBound(
  const std::vector<DrivableLanes> & lanes, const bool is_left)
{
  constexpr double eps = 1.0e-4;
  std::vector<lanelet::ConstPoint3d> bound;
  bound.reserve(lanes.size() * 8);
  for (const auto & lane : lanes) {
    const auto line = is_left ? lane.left_lane.leftBound3d() : lane.right_lane.rightBound3d();
    for (const auto & pt : line) {
      if (bound.empty()) {
        bound.push_back(pt);
        continue;
      }
      const auto prev = bound.back();
      const double dx = prev.x() - pt.x();
      const double dy = prev.y() - pt.y();
      const double dz = prev.z() - pt.z();
      if (std::sqrt(dx * dx + dy * dy + dz * dz) > eps) {
        bound.push_back(pt);
      }
    }
  }
  return bound;
}

std::vector<lanelet::ConstPoint3d> convertToBoundPoints(
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left)
{
  constexpr double overlap_threshold = 0.01;
  std::vector<lanelet::ConstPoint3d> points;
  for (const auto & drivable_lane : drivable_lanes) {
    const auto bound =
      is_left ? drivable_lane.left_lane.leftBound3d() : drivable_lane.right_lane.rightBound3d();
    for (const auto & point : bound) {
      if (
        points.empty() ||
        overlap_threshold < (points.back().basicPoint2d() - point.basicPoint2d()).norm()) {
        points.push_back(point);
      }
    }
  }
  return points;
}

std::vector<PointXYZ> convertBound(
  const std::vector<lanelet::ConstPoint3d> & bound)
{
  std::vector<PointXYZ> converted;
  converted.reserve(bound.size());
  for (const auto & pt : bound) {
    converted.push_back(makePointXYZ(pt));
  }
  return converted;
}

std::optional<lanelet::ConstPolygon3d> getPolygonByPoint(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const lanelet::ConstPoint3d & point, const std::string & subtype)
{
  if (!route_handler) {
    return std::nullopt;
  }
  const auto & map = route_handler->getLaneletMap();
  if (!map) {
    return std::nullopt;
  }
  for (const auto & polygon : map->polygonLayer) {
    const auto type = polygon.attributeOr("subtype", "");
    if (type != subtype) {
      continue;
    }
    for (const auto & poly_point : polygon) {
      if (poly_point.id() == point.id()) {
        return polygon;
      }
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstPolygon3d> getPolygonById(
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::string & id_str)
{
  if (!route_handler || id_str.empty()) {
    return std::nullopt;
  }
  char *end = nullptr;
  const long poly_id_long = std::strtol(id_str.c_str(), &end, 10);
  if (end == id_str.c_str()) {
    return std::nullopt;
  }
  const auto polygon_id = static_cast<lanelet::Id>(poly_id_long);
  const auto & map = route_handler->getLaneletMap();
  if (!map) {
    return std::nullopt;
  }
  const auto it = map->polygonLayer.find(polygon_id);
  if (it == map->polygonLayer.end()) {
    return std::nullopt;
  }
  return *it;
}

std::optional<size_t> findPointIndexInPolygon(
  const lanelet::ConstPolygon3d & polygon, const lanelet::ConstPoint3d & point)
{
  for (size_t i = 0; i < polygon.size(); ++i) {
    if (polygon[i].id() == point.id()) {
      return i;
    }
  }
  return std::nullopt;
}

bool isPolygonClockwise(const lanelet::ConstPolygon3d & polygon)
{
  if (polygon.empty()) {
    return true;
  }
  double area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & p0 = polygon[i].basicPoint2d();
    const auto & p1 = polygon[(i + 1) % polygon.size()].basicPoint2d();
    area += (p0.x() * p1.y()) - (p1.x() * p0.y());
  }
  return area < 0.0;
}

std::vector<double> calcArcLengths(const std::vector<PointXYZ> & points)
{
  std::vector<double> arc;
  arc.resize(points.size());
  if (points.empty()) {
    return arc;
  }
  arc[0] = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    arc[i] = arc[i - 1] + distance2d(points[i - 1], points[i]);
  }
  return arc;
}

PointXYZ pointAlongArc(
  const std::vector<PointXYZ> & points, const std::vector<double> & arc, const double target_s)
{
  if (points.empty()) {
    return PointXYZ{};
  }
  if (arc.empty() || target_s <= 0.0) {
    return points.front();
  }
  const double total = arc.back();
  if (target_s >= total) {
    return points.back();
  }

  auto it = std::lower_bound(arc.begin(), arc.end(), target_s);
  if (it == arc.begin()) {
    return points.front();
  }
  const size_t idx = static_cast<size_t>(std::distance(arc.begin(), it));
  if (*it - target_s < 1e-9) {
    return points[idx];
  }
  const size_t prev_idx = idx - 1;
  const double seg_len = arc[idx] - arc[prev_idx];
  const double ratio = (seg_len < 1e-9) ? 0.0 : (target_s - arc[prev_idx]) / seg_len;
  return lerpPoint(points[prev_idx], points[idx], ratio);
}

struct ProjectionResult
{
  size_t segment_index{0};
  double ratio{0.0};
  double arc_length{0.0};
  PointXYZ point{};
};

ProjectionResult projectOntoBound(
  const std::vector<PointXYZ> & bound, const std::vector<double> & arc,
  const Position & target)
{
  ProjectionResult result;
  if (bound.empty()) {
    return result;
  }
  if (bound.size() == 1) {
    result.point = bound.front();
    result.arc_length = 0.0;
    return result;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < bound.size(); ++i) {
    const auto & p0 = bound[i];
    const auto & p1 = bound[i + 1];
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double len2 = vx * vx + vy * vy;
    double ratio = 0.0;
    if (len2 > 1e-9) {
      const double wx = target.x - p0.x;
      const double wy = target.y - p0.y;
      ratio = (wx * vx + wy * vy) / len2;
      ratio = std::clamp(ratio, 0.0, 1.0);
    }

    const PointXYZ candidate = lerpPoint(p0, p1, ratio);
    const double dx = candidate.x - target.x;
    const double dy = candidate.y - target.y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      result.segment_index = i;
      result.ratio = ratio;
      result.point = candidate;
    }
  }
  const double base_s = arc.at(result.segment_index);
  const double next_s = arc.at(std::min(result.segment_index + 1, arc.size() - 1));
  result.arc_length = base_s + (next_s - base_s) * result.ratio;
  return result;
}

std::vector<PointXYZ> deduplicateBound(
  const std::vector<PointXYZ> & bound, const double threshold)
{
  if (bound.size() < 2) {
    return bound;
  }
  std::vector<PointXYZ> filtered;
  filtered.reserve(bound.size());
  filtered.push_back(bound.front());
  for (size_t i = 1; i < bound.size(); ++i) {
    if (distance2d(filtered.back(), bound[i]) > threshold) {
      filtered.push_back(bound[i]);
    }
  }
  return filtered;
}

std::vector<PointXYZ> removeSharpPoints(std::vector<PointXYZ> bound)
{
  if (bound.size() < 3) {
    return bound;
  }
  constexpr double epsilon = 1e-3;
  size_t idx = 1;
  while (idx + 1 < bound.size()) {
    const auto & p1 = bound[idx - 1];
    const auto & p2 = bound[idx];
    const auto & p3 = bound[idx + 1];

    const double v1x = p2.x - p1.x;
    const double v1y = p2.y - p1.y;
    const double v2x = p3.x - p2.x;
    const double v2y = p3.y - p2.y;

    const double len1 = std::hypot(v1x, v1y);
    const double len2 = std::hypot(v2x, v2y);

    if (len1 < epsilon || len2 < epsilon) {
      bound.erase(bound.begin() + static_cast<std::ptrdiff_t>(idx));
      if (idx > 1) {
        --idx;
      }
      continue;
    }

    const double product = v1x * v2x + v1y * v2y;
    if (product / (len1 * len2) > std::cos(M_PI_4) + epsilon) {
      bound.erase(bound.begin() + static_cast<std::ptrdiff_t>(idx));
      if (idx > 1) {
        --idx;
      }
      continue;
    }
    ++idx;
  }
  return bound;
}

std::vector<PointXYZ> trimBound(
  const std::vector<PointXYZ> & bound, const std::vector<double> & arc,
  const double start_s, const double goal_s)
{
  if (bound.empty() || arc.empty()) {
    return {};
  }

  const double total = arc.back();
  const double s_min = std::clamp(start_s, 0.0, total);
  const double s_max = std::clamp(goal_s, s_min, total);
  std::vector<PointXYZ> trimmed;
  trimmed.reserve(bound.size());
  trimmed.push_back(pointAlongArc(bound, arc, s_min));
  for (size_t i = 0; i < bound.size(); ++i) {
    const double s_val = arc[i];
    if (s_val > s_min && s_val < s_max) {
      if (distance2d(trimmed.back(), bound[i]) > 1e-4) {
        trimmed.push_back(bound[i]);
      }
    }
  }
  const auto end_point = pointAlongArc(bound, arc, s_max);
  if (distance2d(trimmed.back(), end_point) > 1e-4) {
    trimmed.push_back(end_point);
  }
  return trimmed;
}

struct BoundaryPolylineData
{
  std::vector<PointXYZ> points;
  std::vector<double> arc;
  std::vector<double> lane_start_s;
};

BoundaryPolylineData buildBoundaryPolylineData(
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left)
{
  BoundaryPolylineData data;
  data.points.reserve(drivable_lanes.size() * 8);
  data.arc.reserve(drivable_lanes.size() * 8);
  data.lane_start_s.reserve(drivable_lanes.size());

  double accumulated = 0.0;
  constexpr double duplicate_threshold = 1.0e-6;

  for (const auto & lanes : drivable_lanes) {
    data.lane_start_s.push_back(accumulated);
    const auto line =
      is_left ? lanes.left_lane.leftBound3d() : lanes.right_lane.rightBound3d();
    for (size_t i = 0; i < line.size(); ++i) {
      PointXYZ point = makePointXYZ(line[i]);
      if (data.points.empty()) {
        data.points.push_back(point);
        data.arc.push_back(accumulated);
        continue;
      }

      const auto & prev = data.points.back();
      const double dist = distance2d(prev, point);
      if (dist < duplicate_threshold) {
        continue;
      }
      accumulated += dist;
      data.points.push_back(point);
      data.arc.push_back(accumulated);
    }
  }

  return data;
}

std::vector<double> computeCenterlineArcLengths(const PathWithLaneId & path)
{
  const auto centerline_points = extractCenterlinePoints(path);
  return computeArcLengths(centerline_points);
}

std::vector<PointXYZ> buildTrimmedBoundary(
  const std::vector<PointXYZ> & boundary_polyline, const std::vector<double> & boundary_arc,
  const std::vector<PointXYZ> & centerline_points, const std::vector<double> & s_center,
  const double boundary_s_min, const double boundary_s_max,
  const std::vector<double> * target_s_values)
{
  if (
    boundary_polyline.size() < 2 || boundary_arc.size() != boundary_polyline.size() ||
    centerline_points.empty() || centerline_points.size() != s_center.size())
  {
    return {};
  }

  const double clamped_min =
    std::clamp(boundary_s_min, boundary_arc.front(), boundary_arc.back());
  const double clamped_max =
    std::clamp(boundary_s_max, boundary_arc.front(), boundary_arc.back());
  if (clamped_max - clamped_min < 1.0e-6) {
    return {pointAlongArc(boundary_polyline, boundary_arc, clamped_min)};
  }

  struct BoundSample
  {
    PointXYZ point;
    double s;
  };

  std::vector<BoundSample> samples;
  samples.reserve(boundary_polyline.size() + (target_s_values ? target_s_values->size() : 0));

  auto addSample = [&](const PointXYZ & point, double s) {
    samples.push_back(BoundSample{point, s});
  };

  addSample(pointAlongArc(boundary_polyline, boundary_arc, clamped_min), clamped_min);
  for (size_t i = 0; i < boundary_polyline.size(); ++i) {
    const double s = boundary_arc[i];
    if (s > clamped_min && s < clamped_max) {
      addSample(boundary_polyline[i], s);
    }
  }
  addSample(pointAlongArc(boundary_polyline, boundary_arc, clamped_max), clamped_max);

  if (target_s_values && !target_s_values->empty()) {
    for (const double target_s : *target_s_values) {
      if (target_s >= s_center.front() && target_s <= s_center.back()) {
        const double ratio =
          (target_s - s_center.front()) / std::max(s_center.back() - s_center.front(), 1.0e-6);
        const double boundary_s =
          clamped_min + ratio * std::max(clamped_max - clamped_min, 1.0e-6);
        const double clamped =
          std::clamp(boundary_s, boundary_arc.front(), boundary_arc.back());
        addSample(pointAlongArc(boundary_polyline, boundary_arc, clamped), clamped);
      }
    }
  }

  std::sort(
    samples.begin(), samples.end(),
    [](const BoundSample & a, const BoundSample & b) { return a.s < b.s; });

  constexpr double kMinBoundaryPointDist = 1.0e-4;
  constexpr double kMinBoundaryPointDist2 = kMinBoundaryPointDist * kMinBoundaryPointDist;
  std::vector<PointXYZ> trimmed;
  trimmed.reserve(samples.size());
  PointXYZ last_point{};
  bool has_last_point = false;
  for (const auto & sample : samples) {
    if (has_last_point) {
      const double dx = sample.point.x - last_point.x;
      const double dy = sample.point.y - last_point.y;
      const double dz = sample.point.z - last_point.z;
      if ((dx * dx + dy * dy + dz * dz) < kMinBoundaryPointDist2) {
        continue;
      }
    }
    trimmed.push_back(sample.point);
    last_point = sample.point;
    has_last_point = true;
  }
  return trimmed;
}

void removeNearDuplicatePoints(std::vector<PointXYZ> & bound)
{
  if (bound.empty()) {
    return;
  }
  constexpr double kMinBoundaryPointDist = 0.01;
  constexpr double kMinBoundaryPointDist2 = kMinBoundaryPointDist * kMinBoundaryPointDist;
  std::vector<PointXYZ> filtered;
  filtered.reserve(bound.size());
  filtered.push_back(bound.front());
  for (size_t i = 1; i < bound.size(); ++i) {
    const auto & prev = filtered.back();
    const auto & curr = bound[i];
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 > kMinBoundaryPointDist2) {
      filtered.push_back(curr);
    }
  }
  bound.swap(filtered);
}

void smoothBoundaryZ(std::vector<PointXYZ> & bound)
{
  if (bound.size() < 3) {
    return;
  }
  const size_t window = 2;
  std::vector<double> original_z(bound.size());
  for (size_t i = 0; i < bound.size(); ++i) {
    original_z[i] = bound[i].z;
  }
  for (size_t i = 0; i < bound.size(); ++i) {
    const size_t start = (i > window) ? i - window : 0;
    const size_t end = std::min(bound.size() - 1, i + window);
    double sum = 0.0;
    size_t count = 0;
    for (size_t j = start; j <= end; ++j) {
      sum += original_z[j];
      ++count;
    }
    bound[i].z = sum / static_cast<double>(count);
  }
}
}  // namespace

std::vector<lanelet::ConstPoint3d> getBoundWithHatchedRoadMarkings(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler)
{
  if (!route_handler) {
    return original_bound;
  }

  std::vector<lanelet::ConstPoint3d> expanded_bound;
  expanded_bound.reserve(original_bound.size());

  std::optional<lanelet::ConstPolygon3d> current_polygon;
  std::vector<size_t> current_polygon_indices;

  const auto get_index = [&](const lanelet::ConstPolygon3d & polygon, const lanelet::ConstPoint3d & point)
                          -> std::optional<size_t> {
    return findPointIndexInPolygon(polygon, point);
  };

  for (size_t idx = 0; idx < original_bound.size(); ++idx) {
    const auto & bound_point = original_bound[idx];
    const auto polygon = getPolygonByPoint(route_handler, bound_point, "hatched_road_markings");
    bool close_polygon = false;

    if (!current_polygon) {
      if (!polygon) {
        expanded_bound.push_back(bound_point);
      } else {
        current_polygon = polygon;
        current_polygon_indices.clear();
        if (auto poly_idx = get_index(*current_polygon, bound_point)) {
          current_polygon_indices.push_back(*poly_idx);
        }
      }
    } else {
      if (!polygon || polygon->id() != current_polygon->id()) {
        close_polygon = true;
      } else if (auto poly_idx = get_index(*current_polygon, bound_point)) {
        current_polygon_indices.push_back(*poly_idx);
      }
    }

    if (idx == original_bound.size() - 1 && current_polygon) {
      close_polygon = true;
    }

    if (close_polygon && current_polygon) {
      if (current_polygon_indices.size() == 1) {
        expanded_bound.push_back((*current_polygon)[current_polygon_indices.front()]);
      } else if (current_polygon_indices.size() >= 2) {
        const bool is_clockwise = isPolygonClockwise(*current_polygon);
        const bool clockwise_iter = is_clockwise;
        const auto poly_points = extractBoundFromPolygon(
          *current_polygon, current_polygon_indices.front(), current_polygon_indices.back(),
          clockwise_iter);
        expanded_bound.insert(expanded_bound.end(), poly_points.begin(), poly_points.end());
      }

      current_polygon.reset();
      current_polygon_indices.clear();

      if (polygon) {
        current_polygon = polygon;
        if (auto poly_idx = get_index(*current_polygon, bound_point)) {
          current_polygon_indices.push_back(*poly_idx);
        }
      }
    }
  }

  return expanded_bound.empty() ? original_bound : expanded_bound;
}

std::vector<lanelet::ConstPoint3d> getBoundWithIntersectionAreas(
  const std::vector<lanelet::ConstPoint3d> & original_bound,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler,
  const std::vector<DrivableLanes> & drivable_lanes, bool is_left)
{
  if (!route_handler) {
    return original_bound;
  }

  std::vector<lanelet::ConstPoint3d> expanded_bound = original_bound;
  constexpr double distance_threshold = 1e-4;

  for (const auto & drivable_lane : drivable_lanes) {
    const auto edge_lanelet = is_left ? drivable_lane.left_lane : drivable_lane.right_lane;
    const std::string id = edge_lanelet.attributeOr("intersection_area", "");
    if (id.empty() || id == "else") {
      continue;
    }
    const auto polygon_opt = getPolygonById(route_handler, id);
    if (!polygon_opt) {
      continue;
    }
    const auto polygon = *polygon_opt;

    const auto find_shared_index = [&](bool search_from_start) -> std::optional<size_t> {
      if (search_from_start) {
        for (size_t i = 0; i < expanded_bound.size(); ++i) {
          if (findPointIndexInPolygon(polygon, expanded_bound[i])) {
            return i;
          }
        }
      } else {
        for (size_t i = expanded_bound.size(); i > 0; --i) {
          if (findPointIndexInPolygon(polygon, expanded_bound[i - 1])) {
            return i - 1;
          }
        }
      }
      return std::nullopt;
    };

    const auto start_idx = find_shared_index(true);
    const auto end_idx = find_shared_index(false);
    if (!start_idx || !end_idx || *end_idx <= *start_idx) {
      continue;
    }

    const auto start_poly_idx =
      findPointIndexInPolygon(polygon, expanded_bound[*start_idx]);
    const auto end_poly_idx =
      findPointIndexInPolygon(polygon, expanded_bound[*end_idx]);
    if (!start_poly_idx || !end_poly_idx) {
      continue;
    }

    const bool is_clockwise = isPolygonClockwise(polygon);
    const bool clockwise_iteration = is_clockwise ? is_left : !is_left;
    auto polygon_segment = extractBoundFromPolygon(
      polygon, *start_poly_idx, *end_poly_idx, clockwise_iteration);

    std::vector<lanelet::ConstPoint3d> new_bound;
    new_bound.reserve(expanded_bound.size() + polygon_segment.size());
    new_bound.insert(new_bound.end(), expanded_bound.begin(), expanded_bound.begin() + *start_idx);

    for (const auto & pt : polygon_segment) {
      if (
        new_bound.empty() ||
        (pt.basicPoint2d() - new_bound.back().basicPoint2d()).norm() > distance_threshold) {
        new_bound.push_back(pt);
      }
    }

    new_bound.insert(
      new_bound.end(), expanded_bound.begin() + *end_idx, expanded_bound.end());

    expanded_bound = std::move(new_bound);
  }

  return expanded_bound;
}

std::vector<PointXYZ> postProcess(
  const std::vector<PointXYZ> & original_bound, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::vector<DrivableLanes> & drivable_lanes, const bool is_left,
  const bool is_driving_forward)
{
  if (original_bound.size() < 2 || !planner_data || !planner_data->self_odometry) {
    return original_bound;
  }

  auto bound = original_bound;
  const auto route_handler = planner_data->route_handler;

  auto collectLanelets = [&]() {
    lanelet::ConstLanelets lanelets;
    for (const auto & lanes : drivable_lanes) {
      lanelets.push_back(lanes.left_lane);
      lanelets.push_back(lanes.right_lane);
      for (const auto & middle : lanes.middle_lanes) {
        lanelets.push_back(middle);
      }
    }
    return lanelets;
  };
  const auto base_lanelets = collectLanelets();

  const auto appendLaneBoundary = [&](const lanelet::ConstLanelet & lane) {
    const auto line = is_left ? lane.leftBound3d() : lane.rightBound3d();
    for (const auto & pt : line) {
      if (
        bound.empty() ||
        distance2d(bound.back(), makePointXYZ(pt)) >
          1.0e-3) {
        bound.push_back(makePointXYZ(pt));
      }
    }
  };

  if (!is_driving_forward) {
    std::reverse(bound.begin(), bound.end());
  }

  const Pose current_pose = planner_data->self_odometry->pose.pose;
  const size_t ego_seg_idx = planner_data->findEgoSegmentIndex(path.points, current_pose);
  autoware::common_types::Point current_point{};
  current_point.x = current_pose.position.x;
  current_point.y = current_pose.position.y;
  current_point.z = current_pose.position.z;
  const auto cropped_points = autoware::motion_utils::cropPoints(
    path.points, current_point, ego_seg_idx,
    planner_data->parameters.forward_path_length,
    planner_data->parameters.backward_path_length + planner_data->parameters.input_path_interval);

  const Pose front_pose =
    cropped_points.empty() ? current_pose : cropped_points.front().point.pose;
  size_t front_start_idx =
    findNearestSegmentIndexFromLateralDistance(bound, front_pose, M_PI_2);
  auto start_point =
    calcLongitudinalOffsetStartPoint(bound, front_pose, front_start_idx, -0.5);
  Pose start_pose = front_pose;
  start_pose.position.x = start_point.x;
  start_pose.position.y = start_point.y;
  start_pose.position.z = start_point.z;
  const size_t start_idx =
    findNearestSegmentIndexFromLateralDistance(bound, start_pose, M_PI_2);

  const Pose goal_pose =
    path.points.empty() ? current_pose : path.points.back().point.pose;
  const size_t goal_start_idx =
    findNearestSegmentIndexFromLateralDistance(bound, goal_pose, M_PI_2);
  auto goal_point = calcLongitudinalOffsetGoalPoint(
    bound, goal_pose, goal_start_idx, planner_data->parameters.vehicle_length);
  Pose goal_projected = goal_pose;
  goal_projected.position.x = goal_point.x;
  goal_projected.position.y = goal_point.y;
  goal_projected.position.z = goal_point.z;
  const size_t goal_idx = std::max(
    goal_start_idx, findNearestSegmentIndexFromLateralDistance(bound, goal_projected, M_PI_2));

  if (goal_idx < start_idx) {
    return removeSharpPoints(bound);
  }

  std::vector<PointXYZ> processed;
  processed.reserve(goal_idx - start_idx + 3);
  constexpr double overlap_threshold = 1e-3;
  processed.push_back(start_point);

  for (size_t i = start_idx + 1; i <= goal_idx && i < bound.size(); ++i) {
    if (distance2d(processed.back(), bound[i]) > overlap_threshold) {
      processed.push_back(bound[i]);
    }
  }

  if (distance2d(processed.back(), goal_point) > overlap_threshold) {
    processed.push_back(goal_point);
  }

  processed = deduplicateBound(processed, overlap_threshold);
  processed = removeSharpPoints(std::move(processed));
  return processed;
}

std::vector<PointXYZ> calcBound(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const std::vector<DrivableLanes> & drivable_lanes,
  const bool enable_expanding_hatched_road_markings,
  const bool enable_expanding_intersection_areas,
  const bool enable_expanding_freespace_areas, const bool is_left,
  const bool is_driving_forward)
{
  (void)enable_expanding_freespace_areas;
  if (!planner_data) {
    return {};
  }

  auto bound_points = convertToBoundPoints(drivable_lanes, is_left);
  if (enable_expanding_hatched_road_markings) {
    bound_points = getBoundWithHatchedRoadMarkings(bound_points, planner_data->route_handler);
  }
  if (enable_expanding_intersection_areas) {
    bound_points = getBoundWithIntersectionAreas(bound_points, planner_data->route_handler, drivable_lanes, is_left);
  }

  auto bound_xyz = convertBound(bound_points);
  bound_xyz = removeSharpPoints(std::move(bound_xyz));

  return postProcess(bound_xyz, path, planner_data, drivable_lanes, is_left, is_driving_forward);
}

bool checkHasSameLane(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelet & target_lane)
{
  for (const auto & lane : lanelets) {
    if (lane.id() == target_lane.id()) {
      return true;
    }
  }
  return false;
}

void setDrivableAreaLanes(
  const std::vector<autoware::common_types::DrivableLanes> * lanes)
{
  g_drivable_lanes = lanes;
}

void clearDrivableAreaLanes()
{
  g_drivable_lanes = nullptr;
}

void generateDrivableArea(
  autoware::common_types::PathWithLaneId & path, double /*longitudinal_offset*/,
  double lateral_offset, bool /*use_lane_id*/)
{
  if (!g_drivable_lanes || g_drivable_lanes->empty()) {
    return;
  }

  std::vector<autoware::common_types::PointXYZ> left_bound;
  std::vector<autoware::common_types::PointXYZ> right_bound;
  left_bound.reserve(64);
  right_bound.reserve(64);

  std::unordered_map<lanelet::Id, const autoware::common_types::DrivableLanes *> lane_map;
  lane_map.reserve(g_drivable_lanes->size() * 2);
  for (const auto & lanes : *g_drivable_lanes) {
    lane_map[lanes.left_lane.id()] = &lanes;
    lane_map[lanes.right_lane.id()] = &lanes;
    for (const auto & middle : lanes.middle_lanes) {
      lane_map[middle.id()] = &lanes;
    }
  }

  std::unordered_set<lanelet::Id> visited;
  visited.reserve(lane_map.size());

  auto appendLane = [&](const autoware::common_types::DrivableLanes & lanes) {
    appendLineString(lanes.left_lane.leftBound3d(), !left_bound.empty(), left_bound, +1.0, lateral_offset);
    appendLineString(lanes.right_lane.rightBound3d(), !right_bound.empty(), right_bound, -1.0, lateral_offset);
  };

  for (const auto & point : path.points) {
    for (const auto lane_id : point.lane_ids) {
      if (lane_id == 0) {
        continue;
      }
      if (!visited.insert(static_cast<lanelet::Id>(lane_id)).second) {
        continue;
      }
      const auto it = lane_map.find(static_cast<lanelet::Id>(lane_id));
      if (it == lane_map.end()) {
        continue;
      }
      appendLane(*it->second);
    }
  }

  if (left_bound.empty() && !g_drivable_lanes->empty()) {
    for (const auto & lanes : *g_drivable_lanes) {
      appendLane(lanes);
    }
  }

  auto trimToPathStart = [&](std::vector<autoware::common_types::PointXYZ> & bound) {
    if (bound.empty() || path.points.empty()) {
      return;
    }
    const auto & start = path.points.front().point.pose.position;
    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < bound.size(); ++i) {
      const double dx = bound[i].x - start.x;
      const double dy = bound[i].y - start.y;
      const double dz = bound[i].z - start.z;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }
    if (best_index > 0) {
      bound.erase(bound.begin(), bound.begin() + static_cast<std::ptrdiff_t>(best_index));
    }
  };

  trimToPathStart(left_bound);
  trimToPathStart(right_bound);

  path.left_bound = std::move(left_bound);
  path.right_bound = std::move(right_bound);
}

void generateDrivableArea(
  autoware::common_types::PathWithLaneId & path,
  const std::vector<autoware::common_types::DrivableLanes> & drivable_lanes,
  const std::shared_ptr<const PlannerData> & planner_data, const bool is_driving_forward)
{
  if (path.points.empty() || drivable_lanes.empty()) {
    path.left_bound.clear();
    path.right_bound.clear();
    return;
  }

  std::unordered_map<lanelet::Id, lanelet::ConstLanelet> lanelet_map;
  lanelet_map.reserve(drivable_lanes.size() * 4);
  for (const auto & lanes : drivable_lanes) {
    lanelet_map[lanes.left_lane.id()] = lanes.left_lane;
    lanelet_map[lanes.right_lane.id()] = lanes.right_lane;
    for (const auto & middle : lanes.middle_lanes) {
      lanelet_map[middle.id()] = middle;
    }
  }

  auto project_for_point =
    [&](const autoware::common_types::PathPointWithLaneId & path_point, const bool use_left) {
      std::vector<PointXYZ> candidates;
      candidates.reserve(path_point.lane_ids.size());
      for (const auto lane_id_raw : path_point.lane_ids) {
        const auto lane_id = static_cast<lanelet::Id>(lane_id_raw);
        const auto it = lanelet_map.find(lane_id);
        if (it == lanelet_map.end()) {
          continue;
        }
        const auto & line = use_left ? it->second.leftBound3d() : it->second.rightBound3d();
        candidates.push_back(projectOntoLaneBoundary(line, path_point.point.pose.position));
      }
      if (candidates.empty() && !drivable_lanes.empty()) {
        const auto & fallback_lane = drivable_lanes.front();
        const auto & line =
          use_left ? fallback_lane.left_lane.leftBound3d() : fallback_lane.right_lane.rightBound3d();
        candidates.push_back(projectOntoLaneBoundary(line, path_point.point.pose.position));
      }
      const auto query = toPointXYZ(path_point.point.pose.position);
      if (candidates.empty()) {
        return query;
      }
      return weightedAverageBoundaryPoints(candidates, query);
    };

  path.left_bound.clear();
  path.right_bound.clear();
  path.left_bound.reserve(path.points.size());
  path.right_bound.reserve(path.points.size());

  for (const auto & point : path.points) {
    path.left_bound.push_back(project_for_point(point, true));
    path.right_bound.push_back(project_for_point(point, false));
  }
}

}  // namespace autoware::behavior_path_planner::utils
