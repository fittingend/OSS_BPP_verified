// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "path_utils.hpp"

#include "utils.hpp"
#include "trajectory.hpp"
#include "bpp_path_utils.hpp"
#include "resample.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cstdint>

namespace autoware::behavior_path_planner::utils
{

using autoware::common_types::DrivableAreaInfo;
using autoware::common_types::DrivableLanes;
using autoware::common_types::Orientation;
using autoware::common_types::PathPoint;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Pose;
using autoware::common_types::Position;
using autoware::route_handler::RouteHandler;
using autoware::common_types::BehaviorModuleOutput;
using rosless::utils::getArcCoordinatesFromSequence;

std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, size_t start, size_t end, double offset);

namespace
{

double calcDistance2d(const Position & a, const Position & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistance2d(const PathPoint & a, const PathPoint & b)
{
  return calcDistance2d(a.pose.position, b.pose.position);
}

double calcDistanceToSegment2d(
  const lanelet::ConstPoint3d & start, const lanelet::ConstPoint3d & end,
  const Position & target)
{
  const double sx = start.x();
  const double sy = start.y();
  const double ex = end.x();
  const double ey = end.y();
  const double dx = ex - sx;
  const double dy = ey - sy;
  const double len_sq = dx * dx + dy * dy;
  double ratio = 0.0;
  if (len_sq > 1e-6) {
    ratio = ((target.x - sx) * dx + (target.y - sy) * dy) / len_sq;
    ratio = std::clamp(ratio, 0.0, 1.0);
  }
  const double proj_x = sx + ratio * dx;
  const double proj_y = sy + ratio * dy;
  const double px = target.x - proj_x;
  const double py = target.y - proj_y;
  return std::sqrt(px * px + py * py);
}

double calcDistanceToLaneletCenterline(
  const lanelet::ConstLanelet & lanelet, const Position & target)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < centerline.size(); ++i) {
    const auto & prev = centerline[i - 1];
    const auto & curr = centerline[i];
    const double dist = calcDistanceToSegment2d(
      prev, curr, target);
    best = std::min(best, dist);
  }
  return best;
}

std::vector<int64_t> collectNearbyLaneIds(
  const PathPoint & point, const lanelet::ConstLanelets & lanelets, double threshold)
{
  std::vector<std::pair<double, int64_t>> nearby;
  const Position pos = point.pose.position;
  for (const auto & lanelet : lanelets) {
    const double dist = calcDistanceToLaneletCenterline(lanelet, pos);
    if (dist <= threshold) {
      nearby.emplace_back(dist, lanelet.id());
    }
  }
  std::sort(nearby.begin(), nearby.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.first < rhs.first;
  });
  std::vector<int64_t> ids;
  ids.reserve(nearby.size());
  for (const auto & entry : nearby) {
    ids.push_back(entry.second);
  }
  return ids;
}

double calcAzimuthAngle(const Position & from, const Position & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

Orientation createQuaternionFromYaw(const double yaw)
{
  Orientation q{};
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

double getYaw(const Orientation & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<PathPointWithLaneId> removeOverlapPoints(
  const std::vector<PathPointWithLaneId> & points, const double threshold = 0.1)
{
  if (points.size() < 2) {
    return points;
  }

  std::vector<PathPointWithLaneId> filtered;
  filtered.reserve(points.size());
  filtered.push_back(points.front());

  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev = filtered.back().point.pose.position;
    const auto & curr = points[i].point.pose.position;
    const double dist = calcDistance2d(prev, curr);
    if (dist > threshold) {
      filtered.push_back(points[i]);
    }
  }

  return filtered;
}

std::optional<double> calcDistanceToForwardStopPoint(const PathWithLaneId & path)
{
  if (path.points.size() < 2) {
    return std::nullopt;
  }

  constexpr double stop_vel_threshold = 1e-3;
  std::optional<size_t> stop_index;

  for (size_t i = 0; i < path.points.size(); ++i) {
    if (path.points[i].point.longitudinal_velocity_mps <= stop_vel_threshold) {
      stop_index = i;
      break;
    }
  }

  if (!stop_index) {
    return std::nullopt;
  }

  const auto s_vec = calcPathArcLengthArray(path, 0, *stop_index + 1, 0.0);
  return s_vec.back();
}

double lerp(const double a, const double b, const double t)
{
  return a + (b - a) * t;
}

Orientation lerpQuaternion(const Orientation & q1, const Orientation & q2, const double t)
{
  Orientation out{};
  out.x = lerp(q1.x, q2.x, t);
  out.y = lerp(q1.y, q2.y, t);
  out.z = lerp(q1.z, q2.z, t);
  out.w = lerp(q1.w, q2.w, t);
  const double norm = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z + out.w * out.w);
  if (norm > 1e-6) {
    out.x /= norm;
    out.y /= norm;
    out.z /= norm;
    out.w /= norm;
  }
  return out;
}

PathPoint interpolatePathPoint(const PathPoint & from, const PathPoint & to, const double t)
{
  PathPoint out{};
  out.pose.position.x = lerp(from.pose.position.x, to.pose.position.x, t);
  out.pose.position.y = lerp(from.pose.position.y, to.pose.position.y, t);
  out.pose.position.z = lerp(from.pose.position.z, to.pose.position.z, t);
  out.pose.orientation = lerpQuaternion(from.pose.orientation, to.pose.orientation, t);
  out.longitudinal_velocity_mps = lerp(from.longitudinal_velocity_mps, to.longitudinal_velocity_mps, t);
  return out;
}

PathPointWithLaneId interpolatePointByArcLength(
  const PathWithLaneId & path, const std::vector<double> & s_vec, const double target_s)
{
  if (path.points.empty()) {
    return PathPointWithLaneId{};
  }

  auto upper = std::upper_bound(s_vec.begin(), s_vec.end(), target_s);
  if (upper == s_vec.begin()) {
    return path.points.front();
  }
  if (upper == s_vec.end()) {
    return path.points.back();
  }

  const size_t idx1 = static_cast<size_t>(std::distance(s_vec.begin(), upper));
  const size_t idx0 = idx1 - 1;
  const double s0 = s_vec.at(idx0);
  const double s1 = s_vec.at(idx1);
  const double t = (std::abs(s1 - s0) < 1e-6) ? 0.0 : (target_s - s0) / (s1 - s0);

  PathPointWithLaneId point;
  point.point = interpolatePathPoint(path.points[idx0].point, path.points[idx1].point, t);
  point.lane_ids = mergeLaneIdsPreservingOrder(path.points[idx0].lane_ids, path.points[idx1].lane_ids);
  return point;
}

Pose projectToCenterline(const lanelet::ConstLanelet & lanelet, const Pose & pose)
{
  Pose projected = pose;
  if (lanelet.centerline().size() < 2) {
    return projected;
  }

  const auto & centerline = lanelet.centerline();
  const Position p = pose.position;

  double best_dist2 = std::numeric_limits<double>::max();
  Position best_position = p;
  double best_yaw = 0.0;

  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const auto & p0 = centerline[i];
    const auto & p1 = centerline[i + 1];

    const double x0 = p0.x();
    const double y0 = p0.y();
    const double x1 = p1.x();
    const double y1 = p1.y();

    const double vx = x1 - x0;
    const double vy = y1 - y0;
    const double len2 = vx * vx + vy * vy;
    if (len2 < 1e-8) {
      continue;
    }

    const double wx = p.x - x0;
    const double wy = p.y - y0;
    double t = (wx * vx + wy * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);

    const double proj_x = x0 + t * vx;
    const double proj_y = y0 + t * vy;
    const double dx = p.x - proj_x;
    const double dy = p.y - proj_y;
    const double dist2 = dx * dx + dy * dy;

    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_position.x = proj_x;
      best_position.y = proj_y;
      best_position.z = p0.z();
      best_yaw = std::atan2(vy, vx);
    }
  }

  projected.position = best_position;
  projected.orientation = createQuaternionFromYaw(best_yaw);
  return projected;
}

struct SegmentProjection
{
  size_t segment_index{0};
  double ratio{0.0};
};

SegmentProjection projectPoseOntoPath(const PathWithLaneId & path, const Pose & pose)
{
  SegmentProjection projection;
  if (path.points.size() < 2) {
    return projection;
  }

  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const auto & p0 = path.points[i].point.pose.position;
    const auto & p1 = path.points[i + 1].point.pose.position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 < 1e-6) {
      continue;
    }
    const double wx = pose.position.x - p0.x;
    const double wy = pose.position.y - p0.y;
    double ratio = (wx * vx + wy * vy) / len2;
    ratio = std::clamp(ratio, 0.0, 1.0);
    const double proj_x = p0.x + ratio * vx;
    const double proj_y = p0.y + ratio * vy;
    const double dx = pose.position.x - proj_x;
    const double dy = pose.position.y - proj_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      projection.segment_index = i;
      projection.ratio = ratio;
    }
  }
  return projection;
}

void snapPointToPose(PathPointWithLaneId & point, const Pose & pose)
{
  point.point.pose = pose;
  point.point.longitudinal_velocity_mps = 0.0;
  point.point.lateral_velocity_mps = 0.0;
  point.point.heading_rate_rps = 0.0;
}

bool clampPathEndAtGoal(PathWithLaneId & path, const Pose & goal_pose)
{
  if (path.points.empty()) {
    return false;
  }
  if (path.points.size() == 1) {
    snapPointToPose(path.points.front(), goal_pose);
    return true;
  }

  const auto projection = projectPoseOntoPath(path, goal_pose);
  const size_t idx = std::min(projection.segment_index, path.points.size() - 1);
  std::vector<PathPointWithLaneId> clipped;
  clipped.reserve(idx + 2);
  clipped.insert(clipped.end(), path.points.begin(), path.points.begin() + idx + 1);

  if (idx + 1 < path.points.size()) {
    PathPointWithLaneId interpolated;
    interpolated.point = interpolatePathPoint(path.points[idx].point, path.points[idx + 1].point, projection.ratio);
    interpolated.lane_ids = mergeLaneIdsPreservingOrder(
      path.points[idx].lane_ids, path.points[idx + 1].lane_ids);
    clipped.push_back(interpolated);
  }

  path.points = std::move(clipped);
  snapPointToPose(path.points.back(), goal_pose);
  return true;
}

bool snapLastPointToGoal(PathWithLaneId & path, const Pose & goal_pose)
{
  if (path.points.empty()) {
    return false;
  }
  snapPointToPose(path.points.back(), goal_pose);
  return true;
}

lanelet::ConstLanelets collectLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & handler)
{
  lanelet::ConstLanelets lanelets;
  if (!handler) {
    return lanelets;
  }

  std::unordered_set<int64_t> unique_ids;
  for (const auto & point : path.points) {
    for (const auto id : point.lane_ids) {
      unique_ids.insert(id);
    }
  }

  for (const auto id : unique_ids) {
    try {
      lanelets.push_back(handler->getLaneletsFromId(id));
    } catch (...) {
      // ignore missing ids
    }
  }
  return lanelets;
}

void assignLaneIdsFromRoute(
  PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler)
{
  if (!route_handler || !route_handler->isHandlerReady() || path.points.empty()) {
    return;
  }
  const auto & lanelets = route_handler->getRouteLanelets();
  if (lanelets.empty()) {
    return;
  }
  std::vector<int64_t> last_valid_ids;
  constexpr double kNearbyThreshold = 1.5;
  for (auto & point : path.points) {
    auto ids = point.lane_ids;
    const auto nearby = collectNearbyLaneIds(point.point, lanelets, kNearbyThreshold);
    if (!nearby.empty()) {
      ids = mergeLaneIdsPreservingOrder(nearby, ids);
    }
    if (ids.empty()) {
      lanelet::ConstLanelet closest;
      if (route_handler->getClosestLaneletWithinRoute(point.point.pose, &closest)) {
        ids.push_back(closest.id());
      } else if (!last_valid_ids.empty()) {
        ids = last_valid_ids;
      }
    }
    if (ids.empty() && !last_valid_ids.empty()) {
      ids = last_valid_ids;
    }
    if (!ids.empty()) {
      last_valid_ids = ids;
    }
    point.lane_ids = ids;
  }
}

std::vector<DrivableLanes> generateRawDrivableLanes(const lanelet::ConstLanelets & lanelets)
{
  std::vector<DrivableLanes> drivable_lanes;
  drivable_lanes.reserve(lanelets.size());
  for (const auto & ll : lanelets) {
    DrivableLanes lanes;
    lanes.left_lane = ll;
    lanes.right_lane = ll;
    lanes.middle_lanes.push_back(ll);
    drivable_lanes.push_back(lanes);
  }
  return drivable_lanes;
}

namespace
{
bool hasSameLane(const lanelet::ConstLanelet & target, const DrivableLanes & lanes)
{
  if (target.id() == lanes.left_lane.id() || target.id() == lanes.right_lane.id()) {
    return true;
  }
  return std::any_of(
    lanes.middle_lanes.begin(), lanes.middle_lanes.end(),
    [&target](const auto & middle) { return middle.id() == target.id(); });
}

lanelet::ConstLanelets collectLanelets(const DrivableLanes & lanes)
{
  lanelet::ConstLanelets out = lanes.middle_lanes;
  out.push_back(lanes.right_lane);
  out.push_back(lanes.left_lane);
  return out;
}

auto gatherLaneIds(const std::vector<DrivableLanes> & lanes)
{
  std::unordered_set<lanelet::Id> ids;
  ids.reserve(lanes.size() * 4);
  for (const auto & l : lanes) {
    ids.insert(l.left_lane.id());
    ids.insert(l.right_lane.id());
    for (const auto & m : l.middle_lanes) {
      ids.insert(m.id());
    }
  }
  return ids;
}
}  // namespace

std::vector<DrivableLanes> combineDrivableLanesImpl(
  const std::vector<DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<DrivableLanes> & new_drivable_lanes_vec)
{
  if (new_drivable_lanes_vec.empty()) {
    return original_drivable_lanes_vec;
  }
  if (original_drivable_lanes_vec.empty()) {
    return new_drivable_lanes_vec;
  }

  auto updated_drivable_lanes_vec = original_drivable_lanes_vec;
  for (const auto & new_lanes : new_drivable_lanes_vec) {
    bool merged = false;
    for (auto & existing : updated_drivable_lanes_vec) {
      if (hasSameLane(new_lanes.left_lane, existing)) {
        existing.left_lane = new_lanes.left_lane;
        merged = true;
      }
      if (hasSameLane(new_lanes.right_lane, existing)) {
        existing.right_lane = new_lanes.right_lane;
        merged = true;
      }
      for (const auto & middle : new_lanes.middle_lanes) {
        if (hasSameLane(middle, existing)) {
          const bool already_has = std::any_of(
            existing.middle_lanes.begin(), existing.middle_lanes.end(),
            [&middle](const auto & existing_middle) { return existing_middle.id() == middle.id(); });
          if (!already_has) {
            existing.middle_lanes.push_back(middle);
          }
          merged = true;
        }
      }
    }
    if (!merged) {
      updated_drivable_lanes_vec.push_back(new_lanes);
    }
  }
  return updated_drivable_lanes_vec;
}

std::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes)
{
  if (lanes.size() <= 2) {
    return std::nullopt;
  }
  std::unordered_map<lanelet::Id, size_t> first_seen;
  auto register_lane = [&](const lanelet::ConstLanelet & lane, size_t idx) -> std::optional<size_t> {
    const auto it = first_seen.find(lane.id());
    if (it != first_seen.end()) {
      if (idx >= it->second + 2) {
        return idx;
      }
      return std::nullopt;
    }
    first_seen.emplace(lane.id(), idx);
    return std::nullopt;
  };

  for (size_t i = 0; i < lanes.size(); ++i) {
    if (auto overlap = register_lane(lanes[i].left_lane, i)) {
      return overlap;
    }
    if (auto overlap = register_lane(lanes[i].right_lane, i)) {
      return overlap;
    }
    for (const auto & middle : lanes[i].middle_lanes) {
      if (auto overlap = register_lane(middle, i)) {
        return overlap;
      }
    }
  }
  return std::nullopt;
}

std::vector<DrivableLanes> cutOverlappedLanesImpl(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes)
{
  const auto overlapped_lanelet_idx = getOverlappedLaneletId(lanes);
  if (!overlapped_lanelet_idx) {
    return lanes;
  }

  std::vector<DrivableLanes> shorten_lanes{lanes.begin(), lanes.begin() + *overlapped_lanelet_idx};
  const auto allowed_ids = gatherLaneIds(shorten_lanes);

  const auto original_points = path.points;
  path.points.clear();
  for (const auto & point : original_points) {
    const bool within_lane = std::any_of(
      point.lane_ids.begin(), point.lane_ids.end(),
      [&allowed_ids](const auto id) { return allowed_ids.count(id) > 0; });
    if (within_lane) {
      path.points.push_back(point);
    }
  }

  return shorten_lanes;
}

std::vector<DrivableLanes> generateDrivableLanes(
  PathWithLaneId & path, const lanelet::ConstLanelets & lanelets)
{
  auto lanes = generateRawDrivableLanes(lanelets);
  if (lanes.empty()) {
    return lanes;
  }

  std::vector<DrivableLanes> merged;
  merged.reserve(lanes.size());
  for (const auto & lane_group : lanes) {
    if (merged.empty()) {
      merged.push_back(lane_group);
      continue;
    }
    merged = combineDrivableLanes(merged, std::vector<DrivableLanes>{lane_group});
  }

  if (merged.empty()) {
    merged = lanes;
  }

  merged = cutOverlappedLanes(path, merged);
  return merged;
}

std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & lanelets)
{
  return generateRawDrivableLanes(lanelets);
}

}  // namespace

std::vector<DrivableLanes> combineDrivableLanes(
  const std::vector<DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<DrivableLanes> & new_drivable_lanes_vec)
{
  return combineDrivableLanesImpl(original_drivable_lanes_vec, new_drivable_lanes_vec);
}

std::vector<DrivableLanes> cutOverlappedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes)
{
  return cutOverlappedLanesImpl(path, lanes);
}

std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, const size_t start, const size_t end, const double offset)
{
  const auto bounded_start = std::min(start, path.points.size());
  const auto bounded_end = std::min(end, path.points.size());
  std::vector<double> out;
  if (bounded_start >= bounded_end) {
    return out;
  }

  out.reserve(bounded_end - bounded_start);
  double sum = offset;
  out.push_back(sum);

  for (size_t i = bounded_start + 1; i < bounded_end; ++i) {
    sum += calcDistance2d(path.points[i].point, path.points[i - 1].point);
    out.push_back(sum);
  }
  return out;
}

size_t findNearestPointIndexForward(
  const std::vector<PathPointWithLaneId> & points, const Position & pos, const size_t start_idx)
{
  if (points.empty()) {
    return 0;
  }
  const size_t begin_idx = std::min(start_idx, points.size() - 1);
  size_t nearest_idx = begin_idx;
  double best_dist2 = std::numeric_limits<double>::max();
  for (size_t i = begin_idx; i < points.size(); ++i) {
    const double dx = points[i].point.pose.position.x - pos.x;
    const double dy = points[i].point.pose.position.y - pos.y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

size_t findNearestPointIndex(const std::vector<PathPointWithLaneId> & points, const Position & pos)
{
  return findNearestPointIndexForward(points, pos, 0);
}

void snapPathEndToGoal(
  PathWithLaneId & path, const Pose & goal_pose, const std::optional<double> & goal_arc_length)
{
  if (path.points.empty()) {
    return;
  }

  size_t goal_idx = 0;
  if (goal_arc_length) {
    const auto s_vec = calcPathArcLengthArray(path, 0, path.points.size(), 0.0);
    if (!s_vec.empty()) {
      const double target_s = std::clamp(*goal_arc_length, 0.0, s_vec.back());
      auto it = std::lower_bound(s_vec.begin(), s_vec.end(), target_s);
      if (it == s_vec.end()) {
        goal_idx = path.points.size() - 1;
      } else {
        const size_t idx = static_cast<size_t>(std::distance(s_vec.begin(), it));
        if (idx > 0 && (target_s - s_vec[idx - 1]) < (s_vec[idx] - target_s)) {
          goal_idx = idx - 1;
        } else {
          goal_idx = idx;
        }
      }
    }
  } else {
    goal_idx = findNearestPointIndex(path.points, goal_pose.position);
  }

  if (goal_idx + 1 < path.points.size()) {
    path.points.erase(
      path.points.begin() + static_cast<std::ptrdiff_t>(goal_idx + 1), path.points.end());
  }

  auto & last_point = path.points.back().point;
  last_point.pose = goal_pose;
  last_point.longitudinal_velocity_mps = 0.0;
  last_point.lateral_velocity_mps = 0.0;
  last_point.heading_rate_rps = 0.0;
}

PathWithLaneId resamplePathWithSpline(
  const PathWithLaneId & path, const double interval, const bool keep_input_points,
  const std::pair<double, double> target_section)
{
  if (path.points.size() < 2) {
    return path;
  }

  const auto find_almost_same_values =
    [](const std::vector<double> & values, const double value) -> std::optional<std::vector<size_t>> {
      constexpr double epsilon = 0.2;
      std::vector<size_t> indices;
      for (size_t i = 0; i < values.size(); ++i) {
        if (std::abs(values[i] - value) < epsilon) {
          indices.push_back(i);
        }
      }
      if (indices.empty()) {
        return std::nullopt;
      }
      return indices;
    };

  const auto s_vec = calcPathArcLengthArray(path, 0, path.points.size(), 0.0);

  std::vector<double> s_in;
  std::unordered_set<int64_t> unique_lane_ids;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double s = s_vec.at(i);
    for (const auto lane_id : path.points.at(i).lane_ids) {
      if (!keep_input_points && unique_lane_ids.count(lane_id) != 0) {
        continue;
      }
      unique_lane_ids.insert(lane_id);
      if (!find_almost_same_values(s_in, s)) {
        s_in.push_back(s);
      }
    }
  }

  std::vector<double> s_out = s_in;
  const double start_s = std::max(target_section.first, 0.0);
  const double end_s = std::min(target_section.second, s_vec.back());
  for (double s = start_s; s < end_s; s += interval) {
    if (!find_almost_same_values(s_out, s)) {
      s_out.push_back(s);
    }
  }
  if (!find_almost_same_values(s_out, end_s)) {
    s_out.push_back(end_s);
  }

  const auto closest_stop_dist = calcDistanceToForwardStopPoint(path);
  if (closest_stop_dist) {
    const auto duplicate_indices = find_almost_same_values(s_out, *closest_stop_dist);
    if (duplicate_indices) {
      s_out.at(duplicate_indices->front()) = *closest_stop_dist;
      for (size_t i = duplicate_indices->size(); i > 1; --i) {
        s_out.erase(s_out.begin() + duplicate_indices->at(i - 1));
      }
    } else {
      s_out.push_back(*closest_stop_dist);
    }
  }

  if (s_out.size() < 2) {
    return path;
  }

  std::sort(s_out.begin(), s_out.end());

  return rosless::utils::resamplePath(path, s_out);
}

size_t getIdxByArclength(
  const PathWithLaneId & path, const size_t target_idx, const double signed_arc)
{
  if (path.points.empty()) {
    throw std::runtime_error("[getIdxByArclength] path points must be > 0");
  }

  if (signed_arc >= 0.0) {
    double sum_length = 0.0;
    for (size_t i = target_idx; i + 1 < path.points.size(); ++i) {
      sum_length += calcDistance2d(path.points[i].point, path.points[i + 1].point);
      if (sum_length > signed_arc) {
        return i + 1;
      }
    }
    return path.points.size() - 1;
  }

  double sum_length = 0.0;
  for (size_t i = target_idx; i > 0; --i) {
    sum_length -= calcDistance2d(path.points[i].point, path.points[i - 1].point);
    if (sum_length < signed_arc) {
      return i - 1;
    }
  }
  return 0;
}

void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const double forward, const double backward)
{
  if (path.points.size() < 3) {
    return;
  }

  const auto start_idx = getIdxByArclength(path, target_idx, -backward);
  const auto end_idx = getIdxByArclength(path, target_idx, forward);
  const std::vector<PathPointWithLaneId> clipped_points(
    path.points.begin() + static_cast<std::ptrdiff_t>(start_idx),
    path.points.begin() + static_cast<std::ptrdiff_t>(end_idx) + 1);
  path.points = clipped_points;
}

double projectPoseToPathArcLength(
  const PathWithLaneId & path, const std::vector<double> & s_vec, const Pose & pose)
{
  if (path.points.empty() || s_vec.empty()) {
    return 0.0;
  }
  if (path.points.size() < 2) {
    return s_vec.front();
  }
  const Position target = pose.position;
  double best_dist2 = std::numeric_limits<double>::infinity();
  double best_s = s_vec.front();
  for (size_t idx = 0; idx + 1 < path.points.size(); ++idx) {
    const auto & p0 = path.points[idx].point.pose.position;
    const auto & p1 = path.points[idx + 1].point.pose.position;
    const double seg_dx = p1.x - p0.x;
    const double seg_dy = p1.y - p0.y;
    const double seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy;
    double t = 0.0;
    if (seg_len2 > 1e-6) {
      const double wx = target.x - p0.x;
      const double wy = target.y - p0.y;
      t = (wx * seg_dx + wy * seg_dy) / seg_len2;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double proj_x = p0.x + seg_dx * t;
    const double proj_y = p0.y + seg_dy * t;
    const double dx = target.x - proj_x;
    const double dy = target.y - proj_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      const double segment_s = s_vec[idx];
      const double next_s = s_vec[idx + 1];
      best_s = segment_s + t * (next_s - segment_s);
    }
  }
  return best_s;
}

std::pair<double, double> computeClipRangeS(
  const PathWithLaneId & path, const std::vector<double> & s_vec, const Pose & ego_pose,
  const BehaviorPathPlannerParameters & parameter)
{
  if (path.points.empty() || s_vec.empty()) {
    return {0.0, 0.0};
  }

  const double ego_s = projectPoseToPathArcLength(path, s_vec, ego_pose);
  const double max_s = s_vec.back();
  const double start_s = std::clamp(ego_s - parameter.backward_path_length, 0.0, max_s);
  const double end_s = std::clamp(ego_s + parameter.forward_path_length, 0.0, max_s);
  return {start_s, end_s};
}

std::vector<double> makeResampleSGrid(const double clip_start_s, const double clip_end_s, const double interval)
{
  std::vector<double> grid;
  if (clip_end_s <= clip_start_s) {
    grid.push_back(clip_start_s);
    return grid;
  }
  if (interval <= 0.0) {
    grid.push_back(clip_start_s);
    grid.push_back(clip_end_s);
    return grid;
  }
  const double length = clip_end_s - clip_start_s;
  const std::size_t count = static_cast<std::size_t>(std::floor(length / interval)) + 1;
  if (count == 0) {
    grid.push_back(clip_start_s);
    grid.push_back(clip_end_s);
    return grid;
  }
  grid.reserve(count);
  for (std::size_t idx = 0; idx < count; ++idx) {
    double target = clip_start_s + static_cast<double>(idx) * interval;
    if (idx == count - 1) {
      target = clip_end_s;
    } else if (target > clip_end_s) {
      target = clip_end_s;
    }
    grid.push_back(target);
  }
  return grid;
}

PathWithLaneId samplePathByArcLengths(
  const PathWithLaneId & path, const std::vector<double> & s_vec,
  const std::vector<double> & target_s)
{
  PathWithLaneId resampled;
  resampled.header = path.header;
  if (path.points.empty() || s_vec.empty() || target_s.empty()) {
    return resampled;
  }
  resampled.points.reserve(target_s.size());
  for (const double s_point : target_s) {
    resampled.points.push_back(interpolatePointByArcLength(path, s_vec, s_point));
  }
  return resampled;
}

PathWithLaneId postProcessReferencePath(
  const PathWithLaneId & path, const Pose & ego_pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & parameter,
  const std::shared_ptr<RouteHandler> & route_handler,
  BehaviorModuleOutput::ReferencePathDiagnostics * diagnostics)
{
  if (path.points.empty()) {
    return path;
  }
  (void)goal_pose;

  PathWithLaneId processed = path;
  const auto arc_vec = calcPathArcLengthArray(processed, 0, processed.points.size(), 0.0);
  if (arc_vec.empty()) {
    return processed;
  }

  const double goal_s = arc_vec.back();
  const auto lane_segments = createLaneSegments(processed, arc_vec);
  const auto [clip_start_s, clip_end_s] = computeClipRangeS(processed, arc_vec, ego_pose, parameter);
  const double clipped_length = std::max(0.0, clip_end_s - clip_start_s);
  const auto resample_s = makeResampleSGrid(clip_start_s, clip_end_s, parameter.output_path_interval);
  PathWithLaneId output = samplePathByArcLengths(processed, arc_vec, resample_s);
  const auto output_s = calcPathArcLengthArray(output, 0, output.points.size(), 0.0);
  assignLaneIdsFromSegments(output, output_s, lane_segments);
  output.points = removeOverlapPoints(output.points);
  const bool goal_snapped = false;
  assignLaneIdsFromRoute(output, route_handler);

  if (diagnostics) {
    diagnostics->clip_start_s = clip_start_s;
    diagnostics->clip_end_s = clip_end_s;
    diagnostics->clipped_length = clipped_length;
    diagnostics->expected_point_count = resample_s.size();
    diagnostics->actual_point_count = output.points.size();
    diagnostics->goal_s = goal_s;
    diagnostics->goal_snapped = goal_snapped;
    diagnostics->output_interval = parameter.output_path_interval;
  }

  return output;
}

std::vector<double> spline_two_points(
  const std::vector<double> & base_s, const std::vector<double> & base_x, const double begin_diff,
  const double end_diff, const std::vector<double> & new_s)
{
  if (base_s.size() != 2 || base_x.size() != 2) {
    return {};
  }

  const double h = base_s[1] - base_s[0];
  const double c = begin_diff;
  const double d = base_x[0];
  const double a = (end_diff * h - 2.0 * base_x[1] + c * h + 2.0 * d) / std::pow(h, 3);
  const double b = (3.0 * base_x[1] - end_diff * h - 2.0 * c * h - 3.0 * d) / std::pow(h, 2);

  std::vector<double> result;
  result.reserve(new_s.size());
  for (const double s : new_s) {
    const double ds = s - base_s[0];
    result.push_back(d + (c + (b + a * ds) * ds) * ds);
  }
  return result;
}

std::vector<Pose> interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval)
{
  std::vector<Pose> interpolated_poses;
  const double distance = calcDistance2d(start_pose.position, end_pose.position);

  std::vector<double> new_s;
  constexpr double eps = 0.3;
  for (double s = eps; s < distance - eps; s += resample_interval) {
    new_s.push_back(s);
  }

  const std::vector<double> base_s{0.0, distance};
  const std::vector<double> base_x{start_pose.position.x, end_pose.position.x};
  const std::vector<double> base_y{start_pose.position.y, end_pose.position.y};

  const auto interpolated_x =
    spline_two_points(base_s, base_x, std::cos(getYaw(start_pose.orientation)),
      std::cos(getYaw(end_pose.orientation)), new_s);
  const auto interpolated_y =
    spline_two_points(base_s, base_y, std::sin(getYaw(start_pose.orientation)),
      std::sin(getYaw(end_pose.orientation)), new_s);

  interpolated_poses.reserve(interpolated_x.size());
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    Pose pose = start_pose;
    pose.position.x = interpolated_x[i];
    pose.position.y = interpolated_y[i];
    pose.position.z = end_pose.position.z;
    interpolated_poses.push_back(pose);
  }

  for (size_t i = 0; i < interpolated_poses.size(); ++i) {
    const Position & from = interpolated_poses[i].position;
    const Position & to =
      (i + 1 < interpolated_poses.size()) ? interpolated_poses[i + 1].position : end_pose.position;
    interpolated_poses[i].orientation = createQuaternionFromYaw(calcAzimuthAngle(from, to));
  }

  return interpolated_poses;
}

PathWithLaneId combinePath(const PathWithLaneId & first, const PathWithLaneId & second)
{
  if (first.points.empty()) {
    return second;
  }
  if (second.points.empty()) {
    return first;
  }

  PathWithLaneId path;
  path.header = first.header;
  path.points.insert(path.points.end(), first.points.begin(), first.points.end());
  path.points.insert(path.points.end(), std::next(second.points.begin()), second.points.end());
  path.points = removeOverlapPoints(path.points);
  return path;
}

std::vector<LaneSegment> createLaneSegments(
  const PathWithLaneId & path, const std::vector<double> & s_vec)
{
  std::vector<LaneSegment> segments;
  if (path.points.empty() || s_vec.size() != path.points.size()) {
    return segments;
  }
  std::vector<int64_t> current_ids = path.points.front().lane_ids;
  double start_s = s_vec.front();
  double end_s = start_s;
  for (size_t idx = 1; idx < path.points.size(); ++idx) {
    const auto & point_lane_ids = path.points[idx].lane_ids;
    const auto & ids = point_lane_ids.empty() ? current_ids : point_lane_ids;
    const double s = s_vec[idx];
    if (ids != current_ids) {
      segments.push_back({start_s, end_s, current_ids});
      current_ids = ids;
      start_s = s;
    }
    end_s = s;
  }
  segments.push_back({start_s, end_s, current_ids});
  return segments;
}

void assignLaneIdsFromSegments(
  PathWithLaneId & path, const std::vector<double> & s_vec,
  const std::vector<LaneSegment> & segments)
{
  if (segments.empty() || path.points.empty() || s_vec.size() != path.points.size()) {
    return;
  }
  size_t seg_idx = 0;
  for (size_t idx = 0; idx < path.points.size(); ++idx) {
    const double s = s_vec[idx];
    while (seg_idx + 1 < segments.size() && s > segments[seg_idx].s_end) {
      ++seg_idx;
    }
    if (seg_idx >= segments.size()) {
      seg_idx = segments.size() - 1;
    }
    const auto & lane_ids = segments[seg_idx].lane_ids;
    path.points[idx].lane_ids = lane_ids;
  }
}

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  BehaviorModuleOutput output;
  if (!planner_data || !planner_data->route_handler || !planner_data->self_odometry) {
    return output;
  }

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto params = planner_data->parameters;
  const auto goal_pose = route_handler->getGoalPose();

  PathWithLaneId reference_path;
  reference_path.header = route_handler->getRouteHeader();

  constexpr double extra_margin = 10.0;
  const double backward_length = params.backward_path_length + extra_margin;
  const auto lane_sequence =
    route_handler->getLaneletSequence(
      current_lane, backward_length, params.forward_path_length, true);
  if (lane_sequence.empty()) {
    output.path = reference_path;
    output.reference_path = reference_path;
    return output;
  }

  const Pose no_shift_pose = projectToCenterline(current_lane, current_pose);
  reference_path = getCenterLinePath(
    *route_handler, lane_sequence, no_shift_pose, backward_length, params.forward_path_length,
    params);
  if (reference_path.points.empty()) {
    output.path = reference_path;
    output.reference_path = reference_path;
    return output;
  }

  const size_t ego_seg_idx =
    planner_data->findEgoSegmentIndex(reference_path.points, no_shift_pose);
  autoware::common_types::Point crop_point{};
  crop_point.x = no_shift_pose.position.x;
  crop_point.y = no_shift_pose.position.y;
  crop_point.z = no_shift_pose.position.z;
  reference_path.points = autoware::motion_utils::cropPoints(
    reference_path.points, crop_point, ego_seg_idx, params.forward_path_length,
    params.backward_path_length + params.input_path_interval);
  if (reference_path.points.size() < 2) {
    output.path = reference_path;
    output.reference_path = reference_path;
    return output;
  }

  auto drivable_lanes = generateDrivableLanes(lane_sequence);
  const auto & route_lanelets = route_handler->getRouteLanelets();
  if (!route_lanelets.empty()) {
    auto route_lanes = generateDrivableLanes(route_lanelets);
    drivable_lanes = combineDrivableLanes(drivable_lanes, route_lanes);
  }

  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;
  return output;
}

}  // namespace autoware::behavior_path_planner::utils
