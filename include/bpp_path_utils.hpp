#pragma once

#include "common_types.hpp"
#include "route_handler.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <vector>

namespace rosless::utils
{
struct ArcCoordinates
{
  double length{0.0};
  double distance{0.0};
};

ArcCoordinates getArcCoordinatesFromSequence(
  const lanelet::ConstLanelets & lanelet_sequence,
  const autoware::common_types::Pose & pose);

double getLaneletSequenceLength2d(const lanelet::ConstLanelets & lanelet_sequence);

bool laneletSequenceContainsLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lanelet);

lanelet::ConstLanelets getLaneletsFromPath(
  const autoware::common_types::PathWithLaneId & path,
  const std::shared_ptr<autoware::route_handler::RouteHandler> & route_handler);

}  // namespace rosless::utils
