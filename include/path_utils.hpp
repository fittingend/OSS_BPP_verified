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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_

//#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "common_types.hpp"
#include "data_manager.hpp"
#include "parameters.hpp"
#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils
{

using autoware::common_types::Pose;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::BehaviorModuleOutput;
using lanelet::ConstLanelet;

struct LaneSegment
{
  double s_start{0.0};
  double s_end{0.0};
  std::vector<int64_t> lane_ids{};
};

std::vector<LaneSegment> createLaneSegments(
  const PathWithLaneId & path, const std::vector<double> & s_vec);

void assignLaneIdsFromSegments(
  PathWithLaneId & path, const std::vector<double> & s_vec,
  const std::vector<LaneSegment> & segments);



/**
 * @brief path 상 각 점까지의 누적 arc length 계산
 * @param path       대상 경로
 * @param start      시작 index (기본 0)
 * @param end        끝 index (기본: path.size()-1 까지)
 * @param offset     시작 offset (m)
 * @return           각 점까지의 누적 거리 배열
 */
std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, const size_t start = 0,
  const size_t end = std::numeric_limits<size_t>::max(), const double offset = 0.0);

inline std::vector<int64_t> mergeLaneIdsPreservingOrder(
  const std::vector<int64_t> & base, const std::vector<int64_t> & extra)
{
  std::vector<int64_t> merged = base;
  for (const auto id : extra) {
    if (std::find(merged.begin(), merged.end(), id) == merged.end()) {
      merged.push_back(id);
    }
  }
  return merged;
}

/**
 * @brief resample path by spline with constant interval distance
 * @param [in] path original path to be resampled
 * @param [in] interval constant interval distance
 * @param [in] keep_input_points original points are kept in the resampled points
 * @param [in] target_section target section defined by arclength if you want to resample a part of
 * the path
 * @return resampled path
 */
PathWithLaneId resamplePathWithSpline(
  const PathWithLaneId & path, const double interval, const bool keep_input_points = false,
  const std::pair<double, double> target_section = {0.0, std::numeric_limits<double>::max()});

void snapPathEndToGoal(
  PathWithLaneId & path, const Pose & goal_pose, const std::optional<double> & goal_arc_length = std::nullopt);

/**
 * @brief Gets index based on target index and arc length.
 * @param [in] path Path.
 * @param [in] target_idx Target index.
 * @param [in] signed_arc Signed arc length. (Positive is forward)
 * @return Index
 */
size_t getIdxByArclength(
  const PathWithLaneId & path, const size_t target_idx, const double signed_arc);

/**
 * @brief Clips path according to the target index and length.
 * @param [in] path Path.
 * @param [in] target_idx Target index.
 * @param [in] forward Forward distance from target index.
 * @param [in] backward Backward distance from target index.
 * @return Index
 */
void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const double forward, const double backward);

/**
 * @brief Post-process a reference path to align with Autoware behavior: resample, clip near goal, snap endpoint.
 */
PathWithLaneId postProcessReferencePath(
  const PathWithLaneId & path, const Pose & ego_pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & parameter,
  const std::shared_ptr<RouteHandler> & route_handler,
  autoware::common_types::BehaviorModuleOutput::ReferencePathDiagnostics * diagnostics =
    nullptr);

// PathWithLaneId convertWayPointsToPathWithLaneId(
//   const autoware::freespace_planning_algorithms::PlannerWaypoints & waypoints,
//   const double velocity, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Get indices where the driving direction would reverse.
 * @param [in] path Original path.
 * @return Indices.
 */
// std::vector<size_t> getReversingIndices(const PathWithLaneId & path);

/**
 * @brief Divide path by given indices.
 * @param [in] path Original path.
 * @param [in] indices Where to split the path.
 * @return Divided paths.
 */
// std::vector<PathWithLaneId> dividePath(
//   const PathWithLaneId & path, const std::vector<size_t> & indices);

/**
 * @brief Corrects the velocity implemented in the trajectory by judging the vehicle driving
 * direction.
 * @param [in] divided_paths Multiple path with lane ID.
 * @return
 */
// void correctDividedPathVelocity(std::vector<PathWithLaneId> & divided_paths);

// only two points is supported
std::vector<double> spline_two_points(
  const std::vector<double> & base_s, const std::vector<double> & base_x, const double begin_diff,
  const double end_diff, const std::vector<double> & new_s);

/**
 * @brief Interpolates pose between 2 pose.
 * @param [in] start_pose Start pose.
 * @param [in] end_pose End pose.
 * @param [in] resample_interval Resampling interval.
 * @return Array of pose
 */
std::vector<Pose> interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval);

/**
 * @brief Get ego pose which is not shifted.
 * @param [in] ego_pose Ego pose.
 * @param [in] prev_path Previous path with lane ID.
 * @return Unshifted pose.
 */
// geometry_msgs::msg::Pose getUnshiftedEgoPose(
//   const geometry_msgs::msg::Pose & ego_pose, const ShiftedPath & prev_path);

// PathWithLaneId calcCenterLinePath(
//   const std::shared_ptr<const PlannerData> & planner_data, const Pose & ref_pose,
//   const double longest_dist_to_shift_line,
//   const std::optional<PathWithLaneId> & prev_module_path = std::nullopt);

/**
 * @brief Combines 2 path which do not overlap.
 * @param [in] path1 First path.
 * @param [in] path2 Second path.
 * @return Combined path.
 */
PathWithLaneId combinePath(const PathWithLaneId & path1, const PathWithLaneId & path2);

BehaviorModuleOutput getReferencePath(
  const ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data);

std::vector<autoware::common_types::DrivableLanes> combineDrivableLanes(
  const std::vector<autoware::common_types::DrivableLanes> & original_drivable_lanes_vec,
  const std::vector<autoware::common_types::DrivableLanes> & new_drivable_lanes_vec);

std::vector<autoware::common_types::DrivableLanes> cutOverlappedLanes(
  PathWithLaneId & path, const std::vector<autoware::common_types::DrivableLanes> & lanes);

// BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data);

}  // namespace autoware::behavior_path_planner::utils

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_
