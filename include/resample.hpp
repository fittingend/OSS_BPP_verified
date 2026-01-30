#pragma once

#include "common_types.hpp"

#include <vector>

namespace rosless::utils
{

std::vector<autoware::common_types::Pose> resamplePoseVector(
  const std::vector<autoware::common_types::Pose> & poses,
  const std::vector<double> & resampled_arclength);

std::vector<autoware::common_types::Pose> resamplePoseVector(
  const std::vector<autoware::common_types::Pose> & poses, double resample_interval);

autoware::common_types::PathWithLaneId resamplePath(
  const autoware::common_types::PathWithLaneId & input_path,
  const std::vector<double> & resampled_arclength);

autoware::common_types::PathWithLaneId resamplePath(
  const autoware::common_types::PathWithLaneId & input_path, double resample_interval);

}  // namespace rosless::utils
