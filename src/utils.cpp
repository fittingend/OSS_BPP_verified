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

#include "utils.hpp"
#include "common_types.hpp"
#include "bpp_path_utils.hpp"
#include "resample.hpp"

#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils
{

using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::Point;
using autoware::common_types::Pose;
using autoware::common_types::Position;
using autoware::common_types::Vector3;
using rosless::utils::ArcCoordinates;
using rosless::utils::getArcCoordinatesFromSequence;
using rosless::utils::getLaneletSequenceLength2d;
using rosless::utils::getLaneletsFromPath;
using rosless::utils::laneletSequenceContainsLanelet;

using Point2d      = boost::geometry::model::d2::point_xy<double>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using Polygon2d    = boost::geometry::model::polygon<Point2d>;

using autoware::common_types::ObjectClassification;
using autoware::common_types::Shape;


double l2Norm(const Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

//리팩터링 버전 - 필수적인 함수!
PathWithLaneId generateCenterLinePath(const PlannerData & planner_data)
{
  PathWithLaneId centerline_path;

  const auto & p = planner_data.parameters;
  const auto & route_handler = *planner_data.route_handler;
  const auto & pose = planner_data.self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler.getClosestLaneletWithinRoute(pose, &current_lane)) {
    return centerline_path;
  }

  lanelet::ConstLanelets lanelet_sequence = route_handler.getLaneletSequence(
    current_lane, p.backward_path_length, p.forward_path_length, false);

  centerline_path = getCenterLinePath(
    route_handler, lanelet_sequence, pose,
    p.backward_path_length, p.forward_path_length, p);

  centerline_path.header = route_handler.getRouteHeader();
  return centerline_path;
}

PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter)
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = getArcCoordinatesFromSequence(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  const auto lane_length = getLaneletSequenceLength2d(lanelet_sequence);
  s_forward = std::clamp(s_forward, 0.0, lane_length);

  lanelet::ConstLanelet goal_lanelet;
  if (route_handler.getClosestLaneletWithinRoute(route_handler.getGoalPose(), &goal_lanelet)) {
    if (laneletSequenceContainsLanelet(lanelet_sequence, goal_lanelet)) {
      const auto goal_arc_coordinates =
        getArcCoordinatesFromSequence(lanelet_sequence, route_handler.getGoalPose());
      s_forward = std::clamp(s_forward, 0.0, goal_arc_coordinates.length);
    }
  }

  const auto raw_path_with_lane_id =
    route_handler.getCenterLinePath(lanelet_sequence, s_backward, s_forward, true);

  if (parameter.input_path_interval > 0.0) {
    return rosless::utils::resamplePath(raw_path_with_lane_id, parameter.input_path_interval);
  }
  return raw_path_with_lane_id;
}

}  // namespace autoware::behavior_path_planner::utils
