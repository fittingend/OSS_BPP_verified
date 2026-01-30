#include "route_handler.hpp"
#include "bpp_path_utils.hpp"
#include "path_utils.hpp"
#include "common_types.hpp"

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::route_handler
{
namespace
{
using autoware::common_types::LaneletPrimitive;
using autoware::common_types::LaneletRoute;
using autoware::common_types::LaneletSegment;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Position;
using autoware::common_types::Pose;
using autoware::common_types::Quaternion;
using autoware::common_types::UUID;

// ======================
//  Geometry helpers
// ======================
Quaternion createQuaternionFromYaw(double yaw)
{
  Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalizeRadian(double a)
{
  constexpr double pi = 3.14159265358979323846;
  while (a > pi) {
    a -= 2.0 * pi;
  }
  while (a < -pi) {
    a += 2.0 * pi;
  }
  return a;
}

double yawFromOrientation(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
double calcDistance2d(
  const Point & p1, const Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistance3d(const Point & p1, const Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

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

double calcDistance3d(const Position & p1, const Position & p2)
{
  return calcDistance3d(toPoint(p1), toPoint(p2));
}

double calcDistance3d(const Position & p1, const Point & p2)
{
  return calcDistance3d(toPoint(p1), p2);
}

double calcDistance3d(const Point & p1, const Position & p2)
{
  return calcDistance3d(p1, toPoint(p2));
}

double calcDistance2d(const lanelet::ConstPoint3d & lanelet_point, const Point & point)
{
  const double dx = lanelet_point.x() - point.x;
  const double dy = lanelet_point.y() - point.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistancePointToSegment2d(
  const Point & point, const lanelet::ConstPoint3d & segment_start,
  const lanelet::ConstPoint3d & segment_end)
{
  const double sx = segment_start.x();
  const double sy = segment_start.y();
  const double ex = segment_end.x();
  const double ey = segment_end.y();

  const double seg_dx = ex - sx;
  const double seg_dy = ey - sy;
  const double point_dx = point.x - sx;
  const double point_dy = point.y - sy;

  const double seg_length_sq = seg_dx * seg_dx + seg_dy * seg_dy;
  double ratio = 0.0;
  if (seg_length_sq > std::numeric_limits<double>::epsilon()) {
    ratio = (point_dx * seg_dx + point_dy * seg_dy) / seg_length_sq;
    ratio = std::clamp(ratio, 0.0, 1.0);
  }

  const double closest_x = sx + ratio * seg_dx;
  const double closest_y = sy + ratio * seg_dy;
  const double dx = point.x - closest_x;
  const double dy = point.y - closest_y;

  return std::sqrt(dx * dx + dy * dy);
}

double calcDistanceToLaneletCenterline2d(
  const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.empty()) {
    return std::numeric_limits<double>::max();
  }

  if (centerline.size() == 1) {
    return calcDistance2d(centerline.front(), point);
  }

  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const double dist = calcDistancePointToSegment2d(point, centerline[i], centerline[i + 1]);
    if (dist < min_distance) {
      min_distance = dist;
    }
  }

  return min_distance;
}

double calcLaneletYawAtPoint(const lanelet::ConstLanelet & lanelet, const Point & point);

std::optional<lanelet::ConstLanelet> findClosestLanelet(
  const lanelet::LaneletMapPtr & map, const Pose & pose)
{
  if (!map) {
    return std::nullopt;
  }
  const Point search_point{pose.position.x, pose.position.y, pose.position.z};
  const double pose_yaw = yawFromOrientation(pose.orientation);
  const double distance_threshold = 10.0;

  double best_yaw_diff = std::numeric_limits<double>::max();
  double best_distance = std::numeric_limits<double>::max();
  std::optional<lanelet::ConstLanelet> best;
  for (const auto & lanelet : map->laneletLayer) {
    const double distance = calcDistanceToLaneletCenterline2d(lanelet, search_point);
    const double lane_yaw = calcLaneletYawAtPoint(lanelet, search_point);
    const double yaw_diff = std::abs(normalizeRadian(lane_yaw - pose_yaw));
    const bool within_distance = distance <= distance_threshold;
    if (within_distance) {
      if (yaw_diff < best_yaw_diff || (yaw_diff == best_yaw_diff && distance < best_distance)) {
        best_yaw_diff = yaw_diff;
        best_distance = distance;
        best = lanelet;
      }
    } else if (!best) {
      if (distance < best_distance) {
        best_distance = distance;
        best = lanelet;
      }
    }
  }
  if (!best) {
    for (const auto & lanelet : map->laneletLayer) {
      const double distance = calcDistanceToLaneletCenterline2d(lanelet, search_point);
      if (distance < best_distance) {
        best_distance = distance;
        best = lanelet;
      }
    }
  }
  return best;
}

double getCenterlineLength2d(const lanelet::ConstLanelet & lanelet)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  auto prev_it = centerline.begin();
  auto it = std::next(prev_it);
  for (; it != centerline.end(); ++it, ++prev_it) {
    const double dx = it->x() - prev_it->x();
    const double dy = it->y() - prev_it->y();
    length += std::hypot(dx, dy);
  }
  return length;
}

bool isSameUuid(const UUID & lhs, const UUID & rhs)
{
  return std::equal(std::begin(lhs.bytes), std::end(lhs.bytes), std::begin(rhs.bytes));
}

Point toPoint(const lanelet::ConstPoint3d & pt)
{
  Point p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return p;
}

double calcLaneletYawAtPoint(const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto & centerline = lanelet.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }

  size_t best_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const double dist = calcDistancePointToSegment2d(point, centerline[i], centerline[i + 1]);
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }

  const auto & start = centerline[best_idx];
  const auto & end = centerline[std::min(best_idx + 1, centerline.size() - 1)];
  return std::atan2(end.y() - start.y(), end.x() - start.x());
}

ReferencePoint makeReferencePoint(const lanelet::ConstPoint3d & pt)
{
  ReferencePoint ref;
  ref.is_waypoint = false;
  ref.point = toPoint(pt);
  return ref;
}

ReferencePoint makeReferencePoint(const Point & pt)
{
  ReferencePoint ref;
  ref.is_waypoint = true;
  ref.point = pt;
  return ref;
}

template <typename T>
bool isIndexWithinVector(const std::vector<T> & vec, const int index)
{
  return 0 <= index && index < static_cast<int>(vec.size());
}

template <typename T>
void removeIndicesFromVector(std::vector<T> & vec, std::vector<size_t> indices)
{
  std::sort(indices.begin(), indices.end(), std::greater<size_t>());
  for (const auto idx : indices) {
    if (idx < vec.size()) {
      vec.erase(vec.begin() + idx);
    }
  }
}

bool isClose(const Point & p1, const Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

PiecewiseReferencePoints convertWaypointsToReferencePoints(
  const std::vector<Point> & piecewise_waypoints)
{
  PiecewiseReferencePoints refs;
  for (const auto & waypoint : piecewise_waypoints) {
    refs.push_back(makeReferencePoint(waypoint));
  }
  return refs;
}

lanelet::ArcCoordinates calcArcCoordinates(
  const lanelet::ConstLanelet & lanelet, const Point & point)
{
  const auto center2d = lanelet::utils::to2D(lanelet.centerline());
  lanelet::BasicPoint2d target{point.x, point.y};
  return lanelet::geometry::toArcCoordinates(center2d, target);
}

template <typename LaneletT>
bool exists(const std::vector<LaneletT> & lanelets, const lanelet::ConstLanelet & target)
{
  return std::any_of(
    lanelets.begin(), lanelets.end(),
    [&](const auto & llt) { return llt.id() == target.id(); });
}

bool exists(const std::vector<LaneletPrimitive> & primitives, const lanelet::Id id)
{
  return std::any_of(
    primitives.begin(), primitives.end(),
    [&](const auto & primitive) { return primitive.id == id; });
}

lanelet::ConstLanelets getAllNeighbors(
  const std::shared_ptr<lanelet::routing::RoutingGraph> & routing_graph,
  const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets neighbors;
  if (!routing_graph) {
    return neighbors;
  }

  std::vector<lanelet::ConstLanelet> stack;
  std::unordered_set<lanelet::Id> visited;
  stack.push_back(lanelet);

  while (!stack.empty()) {
    const auto current = stack.back();
    stack.pop_back();
    if (!visited.insert(current.id()).second) {
      continue;
    }
    neighbors.push_back(current);

    const auto right_relations = routing_graph->rightRelations(current);
    for (const auto & relation : right_relations) {
      if (relation.relationType == lanelet::routing::RelationType::Right ||
          relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        stack.push_back(relation.lanelet);
      }
    }

    const auto left_relations = routing_graph->leftRelations(current);
    for (const auto & relation : left_relations) {
      if (relation.relationType == lanelet::routing::RelationType::Left ||
          relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        stack.push_back(relation.lanelet);
      }
    }
  }

  return neighbors;
}

}  // namespace


Header RouteHandler::getRouteHeader() const
{
  if (!route_ptr_) {
    //RCLCPP_WARN(logger_, "[Route Handler] getRouteHeader: Route has not been set yet");
    return Header();
  }
  return route_ptr_->header;
}

RouteHandler::RouteHandler() = default;

RouteHandler::RouteHandler(
  const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph)
: lanelet_map_ptr_(lanelet_map), routing_graph_ptr_(routing_graph)
{
  traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,
    lanelet::Participants::Vehicle
  );

  if (!routing_graph_ptr_) {
  routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(
    *lanelet_map_ptr_, *traffic_rules_ptr_);
  }
  
}

void RouteHandler::setMap(
  const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph)
{
  lanelet_map_ptr_ = lanelet_map;
  routing_graph_ptr_ = routing_graph;

  if (lanelet_map_ptr_) {
    traffic_rules_ptr_ = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);

    if (!routing_graph_ptr_ && traffic_rules_ptr_) {
      routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(
        *lanelet_map_ptr_, *traffic_rules_ptr_);
    }
  }

  is_handler_ready_ = false;
  setLaneletsFromRouteMsg();
}
void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!lanelet_map_ptr_) {
    std::cerr << "[RouteHandler] Cannot set route: lanelet map is not set.\n";
    is_handler_ready_ = false;
    return;
  }

  if (isRouteLooped(route_msg.segments)) {
    std::cerr << "[RouteHandler] Loop detected within route. Previous route remains active.\n";
    return;
  }

  if (!route_ptr_ || !isSameUuid(route_ptr_->uuid, route_msg.uuid)) {
    original_start_pose_ = route_msg.start_pose;
    original_goal_pose_ = route_msg.goal_pose;
  }

  route_ptr_ = std::make_shared<LaneletRoute>(route_msg);
  setLaneletsFromRouteMsg();
}

bool RouteHandler::isRouteLooped(const RouteSections & route_sections) const
{
  std::unordered_set<lanelet::Id> visited_lanelets;
  for (const auto & section : route_sections) {
    for (const auto & primitive : section.primitives) {
      const auto lanelet_id = static_cast<lanelet::Id>(primitive.id);
      if (!visited_lanelets.insert(lanelet_id).second) {
        return true;
      }
    }
  }
  return false;
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!route_ptr_ || !lanelet_map_ptr_) {
    return;
  }

  route_lanelets_.clear();
  preferred_lanelets_.clear();

  std::size_t primitive_size = 0;
  for (const auto & route_section : route_ptr_->segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);

  for (const auto & route_section : route_ptr_->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = static_cast<lanelet::Id>(primitive.id);
      if (!lanelet_map_ptr_->laneletLayer.exists(id)) {
        std::cerr << "[RouteHandler] lanelet id " << id << " not found in map.\n";
        continue;
      }
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }

  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_ptr_->segments.empty()) {
    goal_lanelets_.reserve(route_ptr_->segments.back().primitives.size());
    for (const auto & primitive : route_ptr_->segments.back().primitives) {
      const auto id = static_cast<lanelet::Id>(primitive.id);
      if (lanelet_map_ptr_->laneletLayer.exists(id)) {
        goal_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
      }
    }
    start_lanelets_.reserve(route_ptr_->segments.front().primitives.size());
    for (const auto & primitive : route_ptr_->segments.front().primitives) {
      const auto id = static_cast<lanelet::Id>(primitive.id);
      if (lanelet_map_ptr_->laneletLayer.exists(id)) {
        start_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
      }
    }
  }
  is_handler_ready_ = true;
}

void RouteHandler::setRouteLanelets(const lanelet::ConstLanelets & path_lanelets)
{
  if (!lanelet_map_ptr_ || !routing_graph_ptr_) {
    return;
  }

  if (!path_lanelets.empty()) {
    const auto & first_lanelet = path_lanelets.front();
    start_lanelets_ = getAllNeighbors(routing_graph_ptr_, first_lanelet);
    const auto & last_lanelet = path_lanelets.back();
    goal_lanelets_ = getAllNeighbors(routing_graph_ptr_, last_lanelet);
  } else {
    start_lanelets_.clear();
    goal_lanelets_.clear();
  }

  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());
    const auto right_relations = routing_graph_ptr_->rightRelations(lane);
    for (const auto & right_relation : right_relations) {
      if (right_relation.relationType == lanelet::routing::RelationType::Right) {
        route_lanelets_id.insert(right_relation.lanelet.id());
      } else if (right_relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        candidate_lanes_id.insert(right_relation.lanelet.id());
      }
    }
    const auto left_relations = routing_graph_ptr_->leftRelations(lane);
    for (const auto & left_relation : left_relations) {
      if (left_relation.relationType == lanelet::routing::RelationType::Left) {
        route_lanelets_id.insert(left_relation.lanelet.id());
      } else if (left_relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        candidate_lanes_id.insert(left_relation.lanelet.id());
      }
    }
  }

  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);
    auto previous_lanelets = routing_graph_ptr_->previous(lanelet);
    bool is_connected_to_main_lanes_prev = false;
    bool is_connected_to_candidate_prev = true;
    if (exists(start_lanelets_, lanelet)) {
      is_connected_to_candidate_prev = false;
    }
    while (!previous_lanelets.empty() && is_connected_to_candidate_prev &&
           !is_connected_to_main_lanes_prev) {
      is_connected_to_candidate_prev = false;

      for (const auto & prev_lanelet : previous_lanelets) {
        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }
        if (exists(start_lanelets_, prev_lanelet)) {
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_prev = true;
          previous_lanelets = routing_graph_ptr_->previous(prev_lanelet);
          break;
        }
      }
    }

    auto following_lanelets = routing_graph_ptr_->following(lanelet);
    bool is_connected_to_main_lanes_next = false;
    bool is_connected_to_candidate_next = true;
    if (exists(goal_lanelets_, lanelet)) {
      is_connected_to_candidate_next = false;
    }
    while (!following_lanelets.empty() && is_connected_to_candidate_next &&
           !is_connected_to_main_lanes_next) {
      is_connected_to_candidate_next = false;
      for (const auto & next_lanelet : following_lanelets) {
        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }
        if (exists(goal_lanelets_, next_lanelet)) {
          break;
        }
        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_next = true;
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  route_lanelets_.clear();
  route_lanelets_.reserve(route_lanelets_id.size());
  for (const auto & id : route_lanelets_id) {
    route_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  is_handler_ready_ = true;
}

bool RouteHandler::overrideRouteWithShortestPath(const Pose & start_pose, const Pose & goal_pose)
{
  if (!lanelet_map_ptr_ || !routing_graph_ptr_) {
    return false;
  }
  const auto start_lanelet = findClosestLanelet(lanelet_map_ptr_, start_pose);
  const auto goal_lanelet = findClosestLanelet(lanelet_map_ptr_, goal_pose);
  if (!start_lanelet || !goal_lanelet) {
    return false;
  }

  const auto route_opt = routing_graph_ptr_->getRoute(*start_lanelet, *goal_lanelet);
  if (!route_opt) {
    return false;
  }

  const auto & shortest_path = route_opt->shortestPath();
  if (shortest_path.empty()) {
    return false;
  }

  route_lanelets_.assign(shortest_path.begin(), shortest_path.end());
  preferred_lanelets_ = route_lanelets_;
  start_lanelets_.clear();
  goal_lanelets_.clear();
  start_lanelets_.push_back(route_lanelets_.front());
  goal_lanelets_.push_back(route_lanelets_.back());
  return true;
}

void RouteHandler::clearRoute()
{
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();
  route_ptr_.reset();
  is_handler_ready_ = false;
}

Pose RouteHandler::getGoalPose() const
{
  if (route_ptr_) return route_ptr_->goal_pose;
  return Pose{};
}

Pose RouteHandler::getStartPose() const
{
  if (route_ptr_) return route_ptr_->start_pose;
  return Pose{};
}

Pose RouteHandler::getOriginalGoalPose() const { return original_goal_pose_; }
Pose RouteHandler::getOriginalStartPose() const { return original_start_pose_; }


lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  using lanelet::utils::to2D;
  double accumulated = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    const auto & centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }
    auto prev_pt = centerline.front();
    for (const auto & pt : centerline) {
      const double distance =
        lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated + distance > s) {
        const double ratio = (s - accumulated) / distance;
        const auto interpolated = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d{
          lanelet::InvalId, interpolated.x(), interpolated.y(), interpolated.z()};
      }
      accumulated += distance;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d{};
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered;
  filtered.left_bound = input_path.left_bound;
  filtered.right_bound = input_path.right_bound;
  filtered.header = input_path.header;

  for (const auto & pt : input_path.points) {
    if (filtered.points.empty()) {
      filtered.points.push_back(pt);
      continue;
    }
    constexpr double min_dist = 1.0e-3;
    auto & back = filtered.points.back();
    if (calcDistance3d(back.point.pose.position, pt.point.pose.position) < min_dist) {
      back.lane_ids.insert(back.lane_ids.end(), pt.lane_ids.begin(), pt.lane_ids.end());
      back.point.longitudinal_velocity_mps =
        std::min(back.point.longitudinal_velocity_mps, pt.point.longitudinal_velocity_mps);
    } else {
      filtered.points.push_back(pt);
    }
  }
  return filtered;
}

std::vector<Waypoints> RouteHandler::calcWaypointsVector(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  std::vector<Waypoints> waypoints_vec;
  if (!lanelet_map_ptr_) {
    return waypoints_vec;
  }

  for (const auto & lanelet : lanelet_sequence) {
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }
    const auto attr = lanelet.attribute("waypoints").asId();
    if (!attr) {
      continue;
    }

    PiecewiseWaypoints piecewise_waypoints;
    piecewise_waypoints.lanelet_id = lanelet.id();
    const auto & ls = lanelet_map_ptr_->lineStringLayer.get(attr.value());
    for (const auto & waypoint : ls) {
      piecewise_waypoints.piecewise_waypoints.push_back(toPoint(waypoint));
    }
    if (piecewise_waypoints.piecewise_waypoints.empty()) {
      continue;
    }

    if (
      !waypoints_vec.empty() &&
      isClose(
        waypoints_vec.back().back().piecewise_waypoints.back(),
        piecewise_waypoints.piecewise_waypoints.front(), 1.0)) {
      waypoints_vec.back().push_back(piecewise_waypoints);
    } else {
      Waypoints new_waypoints;
      new_waypoints.push_back(piecewise_waypoints);
      waypoints_vec.push_back(new_waypoints);
    }
  }

  return waypoints_vec;
}

void RouteHandler::removeOverlappedCenterlineWithWaypoints(
  std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
  const std::vector<Point> & piecewise_waypoints,
  const lanelet::ConstLanelets & lanelet_sequence,
  const size_t piecewise_waypoints_lanelet_sequence_index,
  const bool is_removing_direction_forward) const
{
  const double margin_ratio = 10.0;
  const auto & target_lanelet = lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index);
  const auto front_arc = calcArcCoordinates(target_lanelet, piecewise_waypoints.front());
  const auto back_arc = calcArcCoordinates(target_lanelet, piecewise_waypoints.back());
  const double lanelet_length =
    lanelet::geometry::length(target_lanelet.centerline().basicLineString());

  const double front_threshold =
    -lanelet_length + front_arc.length - std::abs(front_arc.distance) * margin_ratio;
  const double back_threshold =
    back_arc.length + std::abs(back_arc.distance) * margin_ratio;

  double offset_arc_length = 0.0;
  int target_index = static_cast<int>(piecewise_waypoints_lanelet_sequence_index);
  while (isIndexWithinVector(lanelet_sequence, target_index)) {
    auto & target_points = piecewise_ref_points_vec.at(target_index);
    const auto & lanelet = lanelet_sequence.at(target_index);
    const double lanelet_arc_length =
      lanelet::geometry::length(lanelet.centerline().basicLineString());

    std::vector<size_t> overlapped_indices;
    bool finished = false;
    for (size_t i = 0; i < target_points.size(); ++i) {
      const size_t idx =
        is_removing_direction_forward ? i : target_points.size() - 1 - i;
      const auto & ref_point = target_points.at(idx);
      if (ref_point.is_waypoint) {
        if (target_index == static_cast<int>(piecewise_waypoints_lanelet_sequence_index)) {
          overlapped_indices.clear();
        }
        continue;
      }

      const double ref_arc =
        (is_removing_direction_forward ? 0.0 : -lanelet_arc_length) +
        calcArcCoordinates(lanelet, ref_point.point).length;

      if (is_removing_direction_forward) {
        if (back_threshold < offset_arc_length + ref_arc) {
          finished = true;
          break;
        }
      } else if (offset_arc_length + ref_arc < front_threshold) {
        finished = true;
        break;
      }

      overlapped_indices.push_back(idx);
    }

    removeIndicesFromVector(target_points, overlapped_indices);
    if (finished) {
      break;
    }

    target_index += is_removing_direction_forward ? 1 : -1;
    offset_arc_length = (is_removing_direction_forward ? 1.0 : -1.0) * lanelet_arc_length;
  }
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  return getCenterLinePath(
    lanelet_sequence, 0.0, std::numeric_limits<double>::max(), true);
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  using lanelet::utils::to2D;
  PathWithLaneId reference_path{};
  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  std::vector<PiecewiseReferencePoints> piecewise_ref_points_vec;
  piecewise_ref_points_vec.reserve(lanelet_sequence.size());

  double s = 0.0;
  for (const auto & lanelet : lanelet_sequence) {
    piecewise_ref_points_vec.emplace_back();
    const auto & centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }

    for (size_t i = 0; i < centerline.size(); ++i) {
      const auto & pt = centerline[i];
      const auto next_pt = (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(p));
      }
      if (s >= s_start && s <= s_end) {
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(pt));
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        piecewise_ref_points_vec.back().push_back(makeReferencePoint(p));
      }
      s += distance;
    }
  }

  const auto waypoints_vec = calcWaypointsVector(lanelet_sequence);
  for (const auto & waypoints : waypoints_vec) {
    for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
      const auto & piecewise_waypoints = it->piecewise_waypoints;
      const auto lanelet_id = it->lanelet_id;
      const auto lanelet_it = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [&](const auto & lanelet) { return lanelet.id() == lanelet_id; });
      if (lanelet_it == lanelet_sequence.end()) {
        continue;
      }
      const size_t lanelet_index = std::distance(lanelet_sequence.begin(), lanelet_it);
      const auto ref_points_by_waypoints = convertWaypointsToReferencePoints(piecewise_waypoints);

      const bool is_first = it == waypoints.begin();
      const bool is_last = it == waypoints.end() - 1;
      if (is_first || is_last) {
        const auto original_points = piecewise_ref_points_vec.at(lanelet_index);
        auto & current_points = piecewise_ref_points_vec.at(lanelet_index);
        current_points = ref_points_by_waypoints;
        if (is_first) {
          current_points.insert(current_points.begin(), original_points.begin(), original_points.end());
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence, lanelet_index, false);
        }
        if (is_last) {
          current_points.insert(current_points.end(), original_points.begin(), original_points.end());
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence, lanelet_index, true);
        }
      } else {
        piecewise_ref_points_vec.at(lanelet_index) = ref_points_by_waypoints;
      }
    }
  }

  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    const float speed_limit =
      traffic_rules_ptr_
        ? static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value())
        : 0.0F;
    const auto & ref_points = piecewise_ref_points_vec.at(lanelet_idx);
    for (size_t pt_idx = 0; pt_idx < ref_points.size(); ++pt_idx) {
      const auto & ref_point = ref_points[pt_idx];
      PathPointWithLaneId path_point{};
      path_point.point.pose.position = toPosition(ref_point.point);
      path_point.point.longitudinal_velocity_mps = speed_limit;
      path_point.lane_ids.push_back(lanelet.id());
      if (pt_idx + 1 == ref_points.size() && lanelet_idx + 1 < lanelet_sequence.size()) {
        path_point.lane_ids.push_back(lanelet_sequence.at(lanelet_idx + 1).id());
      }
      reference_path.points.push_back(path_point);
    }
  }

  reference_path = removeOverlappingPoints(reference_path);

  if (reference_path.points.size() == 1 && lanelet_map_ptr_) {
    const auto lane_id = reference_path.points.front().lane_ids.front();
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    const auto point = reference_path.points.front().point.pose.position;
    const auto yaw = calcLaneletYawAtPoint(lanelet, toPoint(point));
    PathPointWithLaneId extra{};
    extra.lane_ids.push_back(lane_id);
    constexpr double ds = 0.1;
    extra.point.pose.position.x = point.x + ds * std::cos(yaw);
    extra.point.pose.position.y = point.y + ds * std::sin(yaw);
    extra.point.pose.position.z = point.z;
    reference_path.points.push_back(extra);
  }

  for (size_t i = 0; i < reference_path.points.size(); ++i) {
    double yaw = 0.0;
    if (i + 1 < reference_path.points.size()) {
      yaw = std::atan2(
        reference_path.points[i + 1].point.pose.position.y -
          reference_path.points[i].point.pose.position.y,
        reference_path.points[i + 1].point.pose.position.x -
          reference_path.points[i].point.pose.position.x);
    } else if (i > 0) {
      yaw = std::atan2(
        reference_path.points[i].point.pose.position.y -
          reference_path.points[i - 1].point.pose.position.y,
        reference_path.points[i].point.pose.position.x -
          reference_path.points[i - 1].point.pose.position.x);
    }
    reference_path.points[i].point.pose.orientation = createQuaternionFromYaw(yaw);
  }

  return reference_path;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  if (!closest_lanelet || !lanelet_map_ptr_) {
    return false;
  }
  if (route_lanelets_.empty()) {
    return false;
  }

  const Point search_point{
    search_pose.position.x, search_pose.position.y, search_pose.position.z};

  std::unordered_set<lanelet::Id> visited_lanelets;
  double min_distance = std::numeric_limits<double>::max();
  lanelet::ConstLanelet best_lanelet;
  bool found_lanelet = false;

  for (const auto & lanelet : route_lanelets_) {
    const auto lanelet_id = lanelet.id();
    if (!visited_lanelets.insert(lanelet_id).second) {
      continue;
    }
    const double distance = calcDistanceToLaneletCenterline2d(lanelet, search_point);
    if (distance < min_distance) {
      min_distance = distance;
      best_lanelet = lanelet;
      found_lanelet = true;
    }
  }

  if (!found_lanelet) {
    return false;
  }

  *closest_lanelet = best_lanelet;
  return true;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  return lanelet_map_ptr_->laneletLayer.get(id);
}

bool RouteHandler::isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == "road_shoulder";
}

bool RouteHandler::isRouteLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return isLaneletPartOfRoute(lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      if (only_route_lanes) {
        break;
      }
      const auto next_lanes = getNextLanelets(current_lanelet);
      if (next_lanes.empty()) {
        break;
      }
      next_lanelet = next_lanes.front();
    }
    if (lanelet.id() == next_lanelet.id()) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += getCenterlineLength2d(next_lanelet);
  }

  return lanelet_sequence_forward;
}
lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet,
  const double backward_distance,
  const double forward_distance,
  const bool only_route_lanes) const
{
  Pose current_pose{};
  current_pose.orientation.w = 1.0;
  if (!lanelet.centerline().empty()) {
    current_pose.position = toPosition(toPoint(lanelet.centerline().front()));
  }

  lanelet::ConstLanelets lanelet_sequence;

  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  // 전방 방향 lanelet 시퀀스
  const lanelet::ConstLanelets lanelet_sequence_forward = std::invoke([&]() {
    if (only_route_lanes) {
      return getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
    } else if (isShoulderLanelet(lanelet)) {
      return getShoulderLaneletSequenceAfter(lanelet, forward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // 후방 방향 lanelet 시퀀스
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate =
      rosless::utils::getArcCoordinatesFromSequence({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      if (only_route_lanes) {
        return getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
      } else if (isShoulderLanelet(lanelet)) {
        return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
      }
    }
    return lanelet::ConstLanelets{};
  });

  // loop check (중복 lanelet 제거)
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      // 뒤/앞 시퀀스가 같은 lanelet 에서 만나는 경우, 중복 없이 forward 쪽으로만 반환
      return lanelet_sequence_forward;
    }
  }

  // backward 시퀀스 + 현재 lanelet + forward 시퀀스 이어 붙이기
  lanelet_sequence.insert(
    lanelet_sequence.end(),
    lanelet_sequence_backward.begin(),
    lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);

  lanelet_sequence.insert(
    lanelet_sequence.end(),
    lanelet_sequence_forward.begin(),
    lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet,
  const double min_length,
  const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0.0;
  lanelet::ConstLanelets previous_lanelets;

  auto checkForLoop =
    [&lanelet](const lanelet::ConstLanelets & lanelets_to_check, const bool is_route_lanelets) {
      if (is_route_lanelets) {
        return std::none_of(
          lanelets_to_check.begin(), lanelets_to_check.end(),
          [lanelet](auto & prev_llt) { return lanelet.id() != prev_llt.id(); });
      }
      return std::any_of(
        lanelets_to_check.begin(), lanelets_to_check.end(),
        [lanelet](auto & prev_llt) { return lanelet.id() == prev_llt.id(); });
    };

  auto isNewLanelet = [&lanelet,
                       &lanelet_sequence_backward](const lanelet::ConstLanelet & lanelet_to_check) {
    if (lanelet.id() == lanelet_to_check.id()) return false;
    return std::none_of(
      lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
      [&lanelet_to_check](auto & backward) { return (backward.id() == lanelet_to_check.id()); });
  };

  while (length < min_length) {
    previous_lanelets.clear();
    bool is_route_lanelets = true;
    if (!getPreviousLaneletsWithinRoute(current_lanelet, &previous_lanelets)) {
      if (only_route_lanes) break;
      previous_lanelets = getPreviousLanelets(current_lanelet);
      if (previous_lanelets.empty()) break;
      is_route_lanelets = false;
    }

    if (checkForLoop(previous_lanelets, is_route_lanelets)) break;

    for (const auto & prev_lanelet : previous_lanelets) {
      if (!isNewLanelet(prev_lanelet) || exists(goal_lanelets_, prev_lanelet)) continue;
      lanelet_sequence_backward.push_back(prev_lanelet);
      length += getCenterlineLength2d(prev_lanelet);
      current_lanelet = prev_lanelet;
      break;
    }
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet,
  const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!isShoulderLanelet(lanelet) || !lanelet_map_ptr_) {
    return lanelet_sequence_forward;
  }

  double length = 0.0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());

  while (length < min_length) {
    const auto next_lanelet_opt = getFollowingShoulderLanelet(current_lanelet);
    if (!next_lanelet_opt) {
      break;
    }
    const auto & next_lanelet = *next_lanelet_opt;
    if (!visited_lanelets.insert(next_lanelet.id()).second) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length += getCenterlineLength2d(next_lanelet);
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet,
  const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!isShoulderLanelet(lanelet) || !lanelet_map_ptr_) {
    return lanelet_sequence_backward;
  }

  double length = 0.0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::unordered_set<lanelet::Id> visited_lanelets;
  visited_lanelets.insert(lanelet.id());

  while (length < min_length) {
    const auto prev_lanelet_opt = getPreviousShoulderLanelet(current_lanelet);
    if (!prev_lanelet_opt) {
      break;
    }
    const auto & prev_lanelet = *prev_lanelet_opt;
    if (!visited_lanelets.insert(prev_lanelet.id()).second) {
      break;
    }
    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), prev_lanelet);
    current_lanelet = prev_lanelet;
    length += getCenterlineLength2d(prev_lanelet);
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  if (!routing_graph_ptr_) {
    return {};
  }
  return routing_graph_ptr_->following(lanelet);
}

lanelet::ConstLanelets RouteHandler::getPreviousLanelets(const lanelet::ConstLanelet & lanelet) const
{
  if (!routing_graph_ptr_) {
    return {};
  }
  return routing_graph_ptr_->previous(lanelet);
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet,
  lanelet::ConstLanelet * next_lanelet) const
{
  if (!next_lanelet || !routing_graph_ptr_ || !route_ptr_) {
    return false;
  }

  lanelet::ConstLanelets next_lanelets{};
  if (getNextLaneletsWithinRoute(lanelet, &next_lanelets)) {
    *next_lanelet = next_lanelets.front();
    return true;
  }
  return false;
}

bool RouteHandler::getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet,
  lanelet::ConstLanelets * next_lanelets) const
{
  if (!next_lanelets || !routing_graph_ptr_ || !route_ptr_) {
    return false;
  }
  if (route_ptr_->segments.empty()) {
    return false;
  }
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }

  const auto start_lane_id = route_ptr_->segments.front().preferred_primitive.id;

  const auto following_lanelets = routing_graph_ptr_->following(lanelet);
  next_lanelets->clear();
  for (const auto & llt : following_lanelets) {
    if (start_lane_id != llt.id() && exists(route_lanelets_, llt)) {
      next_lanelets->push_back(llt);
    }
  }
  return !(next_lanelets->empty());
}

bool RouteHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet,
  lanelet::ConstLanelets * previous_lanelets) const
{
  if (!previous_lanelets || !routing_graph_ptr_) {
    return false;
  }

  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  const auto candidate_lanelets = routing_graph_ptr_->previous(lanelet);
  previous_lanelets->clear();
  for (const auto & llt : candidate_lanelets) {
    if (exists(route_lanelets_, llt)) {
      previous_lanelets->push_back(llt);
    }
  }
  return !(previous_lanelets->empty());
}

std::optional<lanelet::ConstLanelet> RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!lanelet_map_ptr_) {
    return std::nullopt;
  }

  for (const auto & candidate : lanelet_map_ptr_->laneletLayer) {
    if (!isShoulderLanelet(candidate)) {
      continue;
    }
    if (candidate.id() == lanelet.id()) {
      continue;
    }
    if (lanelet::geometry::follows(lanelet, candidate)) {
      return candidate;
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  if (!lanelet_map_ptr_) {
    return std::nullopt;
  }

  for (const auto & candidate : lanelet_map_ptr_->laneletLayer) {
    if (!isShoulderLanelet(candidate)) {
      continue;
    }
    if (candidate.id() == lanelet.id()) {
      continue;
    }
    if (lanelet::geometry::follows(candidate, lanelet)) {
      return candidate;
    }
  }

  return std::nullopt;
}

bool RouteHandler::isLaneletPartOfRoute(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::contains(route_lanelets_, lanelet);
}

}
