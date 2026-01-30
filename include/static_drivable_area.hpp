#pragma once

#include "common_types.hpp"

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
struct PlannerData;
}  // namespace autoware::behavior_path_planner

namespace autoware::behavior_path_planner::utils
{

void setDrivableAreaLanes(
  const std::vector<autoware::common_types::DrivableLanes> * lanes);

void clearDrivableAreaLanes();

void generateDrivableArea(
  autoware::common_types::PathWithLaneId & path, double longitudinal_offset,
  double lateral_offset, bool use_lane_id);

void generateDrivableArea(
  autoware::common_types::PathWithLaneId & path,
  const std::vector<autoware::common_types::DrivableLanes> & lanes,
  const std::shared_ptr<const autoware::behavior_path_planner::PlannerData> & planner_data,
  bool is_driving_forward = true);

}  // namespace autoware::behavior_path_planner::utils
