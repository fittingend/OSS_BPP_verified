#pragma once

#include "common_types.hpp"

#include <limits>
#include <vector>

namespace autoware::motion_utils
{

/**
 * @brief Set velocities to zero after the stop point and mark the final path point.
 * @param path Target path that will be modified in place.
 * @param stop_distance Distance after the stop point where zero velocity should start.
 */
void setZeroVelocityAfterStopDistance(
  autoware::common_types::PathWithLaneId & path, double stop_distance);

double calcLongitudinalOffsetToSegment(
  const std::vector<autoware::common_types::PathPointWithLaneId> & points,
  size_t seg_idx, const autoware::common_types::Point & target_pos);

std::vector<autoware::common_types::PathPointWithLaneId> cropPoints(
  const std::vector<autoware::common_types::PathPointWithLaneId> & points,
  const autoware::common_types::Point & target_pos, size_t target_seg_idx,
  double forward_length, double backward_length);

size_t findFirstNearestIndexWithSoftConstraints(
  const std::vector<autoware::common_types::PathPointWithLaneId> & points,
  const autoware::common_types::Pose & pose,
  double dist_threshold = std::numeric_limits<double>::max(),
  double yaw_threshold = std::numeric_limits<double>::max());

size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const std::vector<autoware::common_types::PathPointWithLaneId> & points,
  const autoware::common_types::Pose & pose,
  double dist_threshold = std::numeric_limits<double>::max(),
  double yaw_threshold = std::numeric_limits<double>::max());

}  // namespace autoware::motion_utils
