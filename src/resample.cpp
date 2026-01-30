#include "resample.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rosless::utils
{
namespace
{
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Pose;
using autoware::common_types::Position;

Point toPoint(const Position & pos)
{
  Point p;
  p.x = pos.x;
  p.y = pos.y;
  p.z = pos.z;
  return p;
}

Position toPosition(const Point & point)
{
  Position pos;
  pos.x = point.x;
  pos.y = point.y;
  pos.z = point.z;
  return pos;
}

double distance2d(const Point & a, const Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

std::vector<double> calcArcLengths(const std::vector<Point> & points)
{
  std::vector<double> s(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    s[i] = s[i - 1] + distance2d(points[i - 1], points[i]);
  }
  return s;
}

std::vector<double> calcArcLengths(const PathWithLaneId & path)
{
  std::vector<Point> pts;
  pts.reserve(path.points.size());
  for (const auto & p : path.points) {
    pts.push_back(toPoint(p.point.pose.position));
  }
  return calcArcLengths(pts);
}

size_t findSegmentIndex(const std::vector<double> & arclength, const double target)
{
  if (target <= arclength.front()) {
    return 0;
  }
  if (target >= arclength.back()) {
    return arclength.size() - 2;
  }
  const auto it = std::upper_bound(arclength.begin(), arclength.end(), target);
  const auto idx = std::distance(arclength.begin(), it);
  return (idx == 0) ? 0 : static_cast<size_t>(idx - 1);
}

Point interpolatePoint(
  const Point & a, const Point & b, const double ratio)
{
  Point p;
  p.x = a.x + (b.x - a.x) * ratio;
  p.y = a.y + (b.y - a.y) * ratio;
  p.z = a.z + (b.z - a.z) * ratio;
  return p;
}

double interpolateScalar(const double a, const double b, const double ratio)
{
  return a + (b - a) * ratio;
}

std::vector<double> buildResampledArclength(double length, double interval)
{
  if (interval <= 0.0) {
    throw std::invalid_argument("resample interval must be positive");
  }
  std::vector<double> target;
  for (double s = 0.0; s < length; s += interval) {
    target.push_back(s);
  }
  if (target.empty() || std::abs(target.back() - length) > 1e-4) {
    target.push_back(length);
  }
  return target;
}

void insertYaw(std::vector<PathPointWithLaneId> & points)
{
  if (points.empty()) {
    return;
  }
  if (points.size() == 1) {
    points.front().point.pose.orientation = autoware::common_types::Orientation{};
    return;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < points.size()) {
      const auto & from = points[i].point.pose.position;
      const auto & to = points[i + 1].point.pose.position;
      yaw = std::atan2(to.y - from.y, to.x - from.x);
    } else {
      const auto & from = points[i - 1].point.pose.position;
      const auto & to = points[i].point.pose.position;
      yaw = std::atan2(to.y - from.y, to.x - from.x);
    }
    autoware::common_types::Orientation q;
    const double half_yaw = yaw * 0.5;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(half_yaw);
    q.w = std::cos(half_yaw);
    points[i].point.pose.orientation = q;
  }
}

}  // namespace

std::vector<Pose> resamplePoseVector(
  const std::vector<Pose> & poses,
  const std::vector<double> & resampled_arclength)
{
  if (poses.size() < 2 || resampled_arclength.empty()) {
    return poses;
  }

  std::vector<Point> positions;
  positions.reserve(poses.size());
  for (const auto & pose : poses) {
    positions.push_back(toPoint(pose.position));
  }
  const auto arclength = calcArcLengths(positions);

  std::vector<Pose> output;
  output.reserve(resampled_arclength.size());

  for (const auto target_s : resampled_arclength) {
    const size_t idx = findSegmentIndex(arclength, target_s);
    const double s0 = arclength[idx];
    const double s1 = arclength[idx + 1];
    const double ratio = (s1 - s0) < 1e-6 ? 0.0 : (target_s - s0) / (s1 - s0);
    Pose pose;
    pose.position = toPosition(interpolatePoint(positions[idx], positions[idx + 1], ratio));
    output.push_back(pose);
  }

  if (output.size() >= 2) {
    for (size_t i = 0; i < output.size(); ++i) {
      double yaw = 0.0;
      if (i + 1 < output.size()) {
        yaw = std::atan2(
          output[i + 1].position.y - output[i].position.y,
          output[i + 1].position.x - output[i].position.x);
      } else {
        yaw = std::atan2(
          output[i].position.y - output[i - 1].position.y,
          output[i].position.x - output[i - 1].position.x);
      }
      autoware::common_types::Orientation q;
      const double half = yaw * 0.5;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(half);
      q.w = std::cos(half);
      output[i].orientation = q;
    }
  }

  return output;
}

std::vector<Pose> resamplePoseVector(const std::vector<Pose> & poses, double resample_interval)
{
  if (poses.size() < 2) {
    return poses;
  }
  std::vector<Point> positions;
  positions.reserve(poses.size());
  for (const auto & pose : poses) {
    positions.push_back(toPoint(pose.position));
  }
  const auto arclength = calcArcLengths(positions);
  const auto targets = buildResampledArclength(arclength.back(), resample_interval);
  return resamplePoseVector(poses, targets);
}

PathWithLaneId resamplePath(
  const PathWithLaneId & input_path,
  const std::vector<double> & resampled_arclength)
{
  if (input_path.points.size() < 2 || resampled_arclength.empty()) {
    return input_path;
  }

  const auto arclength = calcArcLengths(input_path);

  PathWithLaneId resampled;
  resampled.header = input_path.header;
  resampled.left_bound = input_path.left_bound;
  resampled.right_bound = input_path.right_bound;
  resampled.points.reserve(resampled_arclength.size());

  for (const auto target_s : resampled_arclength) {
    const size_t idx = findSegmentIndex(arclength, target_s);
    const auto & prev_point = input_path.points[idx];
    const auto & next_point = input_path.points[idx + 1];
    const double s0 = arclength[idx];
    const double s1 = arclength[idx + 1];
    const double ratio = (s1 - s0) < 1e-6 ? 0.0 : (target_s - s0) / (s1 - s0);

    PathPointWithLaneId point{};
    point.point.pose.position = toPosition(interpolatePoint(
      toPoint(prev_point.point.pose.position), toPoint(next_point.point.pose.position), ratio));

    point.point.longitudinal_velocity_mps = interpolateScalar(
      prev_point.point.longitudinal_velocity_mps, next_point.point.longitudinal_velocity_mps, ratio);
    point.point.lateral_velocity_mps = interpolateScalar(
      prev_point.point.lateral_velocity_mps, next_point.point.lateral_velocity_mps, ratio);
    point.point.heading_rate_rps = interpolateScalar(
      prev_point.point.heading_rate_rps, next_point.point.heading_rate_rps, ratio);
    point.point.is_final = prev_point.point.is_final;

    point.lane_ids = prev_point.lane_ids;
    resampled.points.push_back(point);
  }

  insertYaw(resampled.points);
  return resampled;
}

PathWithLaneId resamplePath(const PathWithLaneId & input_path, double resample_interval)
{
  if (input_path.points.size() < 2) {
    return input_path;
  }
  const auto arclength = calcArcLengths(input_path);
  const auto targets = buildResampledArclength(arclength.back(), resample_interval);
  return resamplePath(input_path, targets);
}

}  // namespace rosless::utils
