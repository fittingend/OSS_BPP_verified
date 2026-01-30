#include "bpp_path_utils.hpp"

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <limits>

namespace rosless::utils
{
namespace
{
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Pose;
using autoware::route_handler::RouteHandler;

lanelet::BasicPoint2d to2d(const lanelet::BasicPoint3d & pt)
{
  return lanelet::BasicPoint2d(pt.x(), pt.y());
}

std::vector<lanelet::BasicPoint2d> buildCenterline(const lanelet::ConstLanelets & lanelet_sequence)
{
  std::vector<lanelet::BasicPoint2d> centerline_points;
  centerline_points.reserve(64);
  for (const auto & ll : lanelet_sequence) {
    const auto & cl = ll.centerline2d();
    if (centerline_points.empty()) {
      for (const auto & point : cl) {
        centerline_points.emplace_back(point.x(), point.y());
      }
    } else {
      for (size_t i = 1; i < cl.size(); ++i) {
        centerline_points.emplace_back(cl[i].x(), cl[i].y());
      }
    }
  }
  return centerline_points;
}

}  // namespace

ArcCoordinates getArcCoordinatesFromSequence(
  const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose)
{
  ArcCoordinates result{};
  const auto centerline_points = buildCenterline(lanelet_sequence);
  if (centerline_points.size() < 2) {
    return result;
  }

  const double px = pose.position.x;
  const double py = pose.position.y;

  double min_dist2 = std::numeric_limits<double>::max();
  double s_at_min = 0.0;
  double s_accum = 0.0;

  for (size_t i = 0; i + 1 < centerline_points.size(); ++i) {
    const auto & p0 = centerline_points[i];
    const auto & p1 = centerline_points[i + 1];

    const double vx = p1.x() - p0.x();
    const double vy = p1.y() - p0.y();
    const double seg_len2 = vx * vx + vy * vy;
    if (seg_len2 < 1e-6) {
      continue;
    }

    const double wx = px - p0.x();
    const double wy = py - p0.y();
    double t = (wx * vx + wy * vy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);

    const double proj_x = p0.x() + t * vx;
    const double proj_y = p0.y() + t * vy;
    const double dx = px - proj_x;
    const double dy = py - proj_y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      const double seg_len = std::sqrt(seg_len2);
      s_at_min = s_accum + seg_len * t;
    }

    s_accum += std::sqrt(seg_len2);
  }

  result.length = s_at_min;
  result.distance = std::sqrt(min_dist2);
  return result;
}

double getLaneletSequenceLength2d(const lanelet::ConstLanelets & lanelet_sequence)
{
  double total_length = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    const auto & centerline = lanelet.centerline2d();
    for (size_t i = 0; i + 1 < centerline.size(); ++i) {
      const double dx = centerline[i + 1].x() - centerline[i].x();
      const double dy = centerline[i + 1].y() - centerline[i].y();
      total_length += std::hypot(dx, dy);
    }
  }
  return total_length;
}

bool laneletSequenceContainsLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lanelet)
{
  return std::any_of(
    lanelet_sequence.begin(), lanelet_sequence.end(),
    [&](const lanelet::ConstLanelet & lanelet) { return lanelet.id() == target_lanelet.id(); });
}

lanelet::ConstLanelets getLaneletsFromPath(
  const PathWithLaneId & path, const std::shared_ptr<RouteHandler> & route_handler)
{
  std::vector<int64_t> lane_ids;
  lane_ids.reserve(path.points.size());
  for (const auto & point : path.points) {
    for (const auto & id : point.lane_ids) {
      if (
        std::find(lane_ids.begin(), lane_ids.end(), id) ==
        lane_ids.end()) {
        lane_ids.push_back(id);
      }
    }
  }

  lanelet::ConstLanelets lanelets;
  lanelets.reserve(lane_ids.size());
  for (const auto & id : lane_ids) {
    lanelets.push_back(route_handler->getLaneletsFromId(id));
  }
  return lanelets;
}

}  // namespace rosless::utils
