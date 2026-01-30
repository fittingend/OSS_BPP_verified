#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_utils
{
namespace
{
using autoware::common_types::Orientation;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Position;
using autoware::common_types::Pose;

double calcSquaredDistance2d(const Position & a, const Position & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double calcSquaredDistance2d(const PathPointWithLaneId & point, const Position & pos)
{
  return calcSquaredDistance2d(point.point.pose.position, pos);
}

double normalizeYaw(const double yaw)
{
  double normalized = yaw;
  constexpr double pi = M_PI;
  while (normalized > pi) {
    normalized -= 2.0 * pi;
  }
  while (normalized < -pi) {
    normalized += 2.0 * pi;
  }
  return normalized;
}

double yawFromOrientation(const Orientation & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double calcYawDeviation(const Pose & point_pose, const Pose & target_pose)
{
  const double point_yaw = yawFromOrientation(point_pose.orientation);
  const double target_yaw = yawFromOrientation(target_pose.orientation);
  return normalizeYaw(point_yaw - target_yaw);
}

std::vector<double> calcArcLengths(const std::vector<PathPointWithLaneId> & points)
{
  std::vector<double> arc(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1].point.pose.position;
    const auto & curr = points[i].point.pose.position;
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    arc[i] = arc[i - 1] + std::hypot(dx, dy);
  }
  return arc;
}
}  // namespace

void setZeroVelocityAfterStopDistance(PathWithLaneId & path, const double stop_distance)
{
  if (path.points.empty()) {
    return;
  }

  constexpr double stop_vel_threshold = 1e-3;
  size_t stop_index = path.points.size() - 1;
  bool stop_found = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    if (path.points[i].point.longitudinal_velocity_mps <= stop_vel_threshold) {
      stop_index = i;
      stop_found = true;
      break;
    }
  }

  if (path.points.size() == 1) {
    path.points.front().point.longitudinal_velocity_mps = 0.0;
    return;
  }

  const auto arc_lengths = calcArcLengths(path.points);
  const double stop_s = arc_lengths.at(stop_index);
  const double target_s = stop_s + std::max(stop_distance, 0.0);

  size_t zero_start_idx = stop_index;
  for (size_t i = stop_index; i < arc_lengths.size(); ++i) {
    if (arc_lengths[i] >= target_s) {
      zero_start_idx = i;
      break;
    }
  }

  for (size_t i = zero_start_idx; i < path.points.size(); ++i) {
    path.points[i].point.longitudinal_velocity_mps = 0.0;
  }
  if (!stop_found) {
    path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  for (auto & point_with_lane : path.points) {
    point_with_lane.point.is_final = false;
  }
}

namespace
{
double calcDistance2d(const PathPointWithLaneId & a, const PathPointWithLaneId & b)
{
  const auto & p0 = a.point.pose.position;
  const auto & p1 = b.point.pose.position;
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  return std::hypot(dx, dy);
}

std::vector<PathPointWithLaneId> cropForwardPoints(
  const std::vector<PathPointWithLaneId> & points, const Point & target_pos,
  size_t target_seg_idx, const double forward_length)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1 || forward_length <= 0.0) {
    return {points.front()};
  }
  if (target_seg_idx >= points.size() - 1) {
    target_seg_idx = points.size() - 2;
  }

  double sum_length =
    -calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
    sum_length += calcDistance2d(points[i], points[i - 1]);
    if (forward_length < sum_length) {
      const size_t end_idx = i;
      return std::vector<PathPointWithLaneId>(points.begin(), points.begin() + end_idx);
    }
  }
  return points;
}

std::vector<PathPointWithLaneId> cropBackwardPoints(
  const std::vector<PathPointWithLaneId> & points, const Point & target_pos,
  size_t target_seg_idx, const double backward_length)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1 || backward_length <= 0.0) {
    return points;
  }
  if (target_seg_idx >= points.size() - 1) {
    target_seg_idx = points.size() - 2;
  }

  double sum_length =
    -calcLongitudinalOffsetToSegment(points, target_seg_idx, target_pos);
  for (int i = static_cast<int>(target_seg_idx); i > 0; --i) {
    sum_length -= calcDistance2d(points[i], points[i - 1]);
    if (sum_length < -backward_length) {
      const size_t begin_idx = static_cast<size_t>(i);
      return std::vector<PathPointWithLaneId>(points.begin() + begin_idx, points.end());
    }
  }
  return points;
}
}  // namespace

double calcLongitudinalOffsetToSegment(
  const std::vector<PathPointWithLaneId> & points, const size_t seg_idx, const Point & target_pos)
{
  if (points.size() < 2 || seg_idx >= points.size() - 1) {
    return 0.0;
  }

  const auto & p0 = points.at(seg_idx).point.pose.position;
  const auto & p1 = points.at(seg_idx + 1).point.pose.position;
  const double vx = p1.x - p0.x;
  const double vy = p1.y - p0.y;
  const double len2 = vx * vx + vy * vy;
  if (len2 < 1e-8) {
    return 0.0;
  }

  const double wx = target_pos.x - p0.x;
  const double wy = target_pos.y - p0.y;
  const double ratio = (wx * vx + wy * vy) / len2;
  return ratio * std::sqrt(len2);
}

std::vector<PathPointWithLaneId> cropPoints(
  const std::vector<PathPointWithLaneId> & points, const Point & target_pos,
  const size_t target_seg_idx, const double forward_length, const double backward_length)
{
  if (points.size() < 2) {
    return points;
  }

  auto cropped_forward = cropForwardPoints(points, target_pos, target_seg_idx, forward_length);
  if (cropped_forward.size() < 2) {
    return points;
  }

  const size_t adjusted_idx =
    std::min(target_seg_idx, cropped_forward.size() > 1 ? cropped_forward.size() - 2 : size_t{0});
  auto cropped =
    cropBackwardPoints(cropped_forward, target_pos, adjusted_idx, backward_length);
  if (cropped.size() < 2) {
    return points;
  }
  return cropped;
}

size_t findFirstNearestIndexWithSoftConstraints(
  const std::vector<PathPointWithLaneId> & points, const Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  if (points.empty()) {
    return 0;
  }

  const auto search = [&](const bool use_dist, const bool use_yaw) -> std::optional<size_t> {
    const double dist_sq_threshold = dist_threshold * dist_threshold;
    double min_sq_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool found = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const double sq_dist = calcSquaredDistance2d(points[i], pose.position);
      if (use_dist && sq_dist > dist_sq_threshold) {
        if (found) {
          break;
        }
        continue;
      }

      const double yaw_dev = std::abs(calcYawDeviation(points[i].point.pose, pose));
      if (use_yaw && yaw_dev > yaw_threshold) {
        if (found) {
          break;
        }
        continue;
      }

      if (sq_dist >= min_sq_dist) {
        continue;
      }

      min_sq_dist = sq_dist;
      min_idx = i;
      found = true;
    }
    if (found) {
      return min_idx;
    }
    return std::nullopt;
  };

  if (const auto idx = search(true, true)) {
    return *idx;
  }
  if (const auto idx = search(true, false)) {
    return *idx;
  }

  double min_sq_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const double sq_dist = calcSquaredDistance2d(points[i], pose.position);
    if (sq_dist < min_sq_dist) {
      min_sq_dist = sq_dist;
      min_idx = i;
    }
  }
  return min_idx;
}

size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const std::vector<PathPointWithLaneId> & points, const Pose & pose,
  const double dist_threshold, const double yaw_threshold)
{
  if (points.size() < 2) {
    return 0;
  }

  const size_t nearest_idx =
    findFirstNearestIndexWithSoftConstraints(points, pose, dist_threshold, yaw_threshold);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx >= points.size() - 1) {
    return points.size() - 2;
  }

  autoware::common_types::Point target_pos;
  target_pos.x = pose.position.x;
  target_pos.y = pose.position.y;
  target_pos.z = pose.position.z;
  const double signed_length =
    calcLongitudinalOffsetToSegment(points, nearest_idx, target_pos);
  if (signed_length <= 0.0) {
    return nearest_idx - 1;
  }
  return nearest_idx;
}

}  // namespace autoware::motion_utils
