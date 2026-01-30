#include "planner_manager.hpp"

#include "data_manager.hpp"
#include "static_drivable_area.hpp"
#include "path_utils.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <cmath>

namespace autoware::behavior_path_planner
{

namespace
{
double distanceSquaredXY(
  const autoware::common_types::PointXYZ & point,
  const autoware::common_types::Position & position)
{
  const double dx = point.x - position.x;
  const double dy = point.y - position.y;
  return dx * dx + dy * dy;
}

void ensureBoundOrder(autoware::common_types::PathWithLaneId & path)
{
  if (path.points.size() < 2) {
    return;
  }
  const auto & start = path.points.front().point.pose.position;
  const auto & goal = path.points.back().point.pose.position;
  auto adjust = [&](std::vector<autoware::common_types::PointXYZ> & bound) {
    if (bound.size() < 2) {
      return;
    }
    const double first_start = distanceSquaredXY(bound.front(), start);
    const double last_start = distanceSquaredXY(bound.back(), start);
    if (first_start > last_start) {
      std::reverse(bound.begin(), bound.end());
    }
  };
  adjust(path.left_bound);
  adjust(path.right_bound);
}

bool findClosestRouteLanelet(
  const std::shared_ptr<PlannerData> & data, lanelet::ConstLanelet * lanelet_out)
{
  if (!data || !data->route_handler || !data->self_odometry || !lanelet_out) {
    return false;
  }

  return data->route_handler->getClosestLaneletWithinRoute(
    data->self_odometry->pose.pose, lanelet_out);
}
}  // namespace

PlannerManager::PlannerManager()
{
  current_route_lanelet_ = std::make_shared<std::optional<lanelet::ConstLanelet>>(std::nullopt);
  processing_time_.emplace("total_time", 0.0);
}

void PlannerManager::resetProcessingTime()
{
  for (auto & item : processing_time_) {
    item.second = 0.0;
  }
}

BehaviorModuleOutput PlannerManager::run(const std::shared_ptr<PlannerData> & data)
{
  BehaviorModuleOutput output;
  resetProcessingTime();

  if (!data) {
    return output;
  }

  lanelet::ConstLanelet closest_lanelet;
  if (!findClosestRouteLanelet(data, &closest_lanelet)) {
    return output;
  }

  *current_route_lanelet_ = closest_lanelet;
  output = getReferencePath(data);
  generateCombinedDrivableArea(output, data);
  return output;
}

void PlannerManager::generateCombinedDrivableArea(
  BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const
{
  if (!data) {
    return;
  }

  const auto & drivable_lanes = output.drivable_area_info.drivable_lanes;
  if (drivable_lanes.empty()) {
    return;
  }

  autoware::behavior_path_planner::utils::generateDrivableArea(
    output.path, drivable_lanes, data, true);
  autoware::behavior_path_planner::utils::generateDrivableArea(
    output.reference_path, drivable_lanes, data, true);

  ensureBoundOrder(output.path);
  ensureBoundOrder(output.reference_path);
}

void PlannerManager::updateCurrentRouteLanelet(
  const std::shared_ptr<PlannerData> & data, bool)
{
  lanelet::ConstLanelet closest_lanelet;
  if (findClosestRouteLanelet(data, &closest_lanelet)) {
    *current_route_lanelet_ = closest_lanelet;
  }
}

BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  BehaviorModuleOutput output;
  if (!data || !current_route_lanelet_ || !current_route_lanelet_->has_value()) {
    return output;
  }

  output = autoware::behavior_path_planner::utils::getReferencePath(
    current_route_lanelet_->value(), data);
  publishDebugRootReferencePath(output);
  return output;
}

void PlannerManager::publishDebugRootReferencePath(
  const BehaviorModuleOutput &) const
{
  // No-op in this stand-alone implementation. Hook for future logging if needed.
}

void PlannerManager::resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data)
{
  lanelet::ConstLanelet closest_lanelet;
  if (findClosestRouteLanelet(data, &closest_lanelet)) {
    *current_route_lanelet_ = closest_lanelet;
  } else if (current_route_lanelet_) {
    current_route_lanelet_->reset();
  }
}

}  // namespace autoware::behavior_path_planner
