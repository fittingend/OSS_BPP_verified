// route_handler.hpp (수정 버전)

#ifndef AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_
#define AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_

// ★ ROS 헤더 대신 우리 타입 헤더 사용

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include "common_types.hpp"

#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::route_handler
{
using autoware::common_types::Header;
using autoware::common_types::Point;
using autoware::common_types::Position;
using autoware::common_types::Orientation;
using autoware::common_types::Vector3;
using autoware::common_types::TimeStamp;
using autoware::common_types::PathPoint;
using autoware::common_types::Pose;
using autoware::common_types::PoseStamped;
using autoware::common_types::UUID;
using autoware::common_types::LaneletRoute;
using autoware::common_types::LaneletSegment;
using autoware::common_types::LaneletPrimitive;
using autoware::common_types::Quaternion;
using autoware::common_types::PathPointWithLaneId;
using autoware::common_types::PathWithLaneId;

using RouteSections = std::vector<LaneletSegment>;
using lanelet::LaneletMapPtr;
using lanelet::routing::RoutingGraphPtr;
using lanelet::traffic_rules::TrafficRulesPtr;
using lanelet::ConstLanelet;

struct ReferencePoint
{
  bool is_waypoint{false};
  Point point{};
};
using PiecewiseReferencePoints = std::vector<ReferencePoint>;

struct PiecewiseWaypoints
{
  lanelet::Id lanelet_id{lanelet::InvalId};
  std::vector<Point> piecewise_waypoints;
};
using Waypoints = std::vector<PiecewiseWaypoints>;
//using lanelet::utils::to2D();

class RouteHandler
{
public:
  RouteHandler();
  explicit RouteHandler(const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph);

  // 기존 setMap(const LaneletMapBin&) 대신 “이미 만들어진 lanelet map/graph” 을 받도록
  void setMap(const LaneletMapPtr & lanelet_map, const RoutingGraphPtr & routing_graph);

  void setRoute(const LaneletRoute & route_msg);
  void setRouteLanelets(const lanelet::ConstLanelets & path_lanelets);
  void clearRoute();

  bool isMapReady() const { return static_cast<bool>(lanelet_map_ptr_ && routing_graph_ptr_); }
  bool isHandlerReady() const { return is_handler_ready_; }

  // 원래 TIER IV RouteHandler 가 제공하던 API 중,
  // mission planner / behavior path planner 에서 실제로 쓰는 것만 우선 추려서 선언
  const LaneletMapPtr & getLaneletMap() const { return lanelet_map_ptr_; }
  const RoutingGraphPtr & getRoutingGraph() const { return routing_graph_ptr_; }

  const LaneletRoute & getRoute() const { return *route_ptr_; }
  const lanelet::ConstLanelets & getRouteLanelets() const { return route_lanelets_; }
  RouteSections getRouteSections() const;
  bool overrideRouteWithShortestPath(const Pose & start_pose, const Pose & goal_pose);

  bool planPathLaneletsBetweenCheckpoints(
    const Pose & start_checkpoint, const Pose & goal_checkpoint,
    lanelet::ConstLanelets * path_lanelets,
    const bool consider_no_drivable_lanes = false) const;

  std::vector<LaneletSegment> createMapSegments(
    const lanelet::ConstLanelets & path_lanelets) const;

  lanelet::ConstLanelet getLaneletsFromId(const lanelet::Id id) const;

  // goal 관련 주요 API (behavior path planner 가 많이 씀)
  Pose getGoalPose() const;
  Pose getStartPose() const;
  Pose getOriginalGoalPose() const;
  Pose getOriginalStartPose() const;

  lanelet::Id getGoalLaneId() const;
  bool getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const;

  // reference path 생성 (behavior path planner 에서 사용)
  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence) const;

  PathWithLaneId getCenterLinePath(
    const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end, bool use_exact) const;
  Header getRouteHeader() const;
  // … 필요하면 나중에 다른 메서드도 여기 계속 추가 (원본 RouteHandler 참고)
  lanelet::ConstLanelets getLaneletSequence(
    const lanelet::ConstLanelet & lanelet,
    const double backward_distance,
    const double forward_distance,
    const bool only_route_lanes) const;
  bool getClosestLaneletWithinRoute(const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const;

private:
  LaneletMapPtr lanelet_map_ptr_{nullptr};
  RoutingGraphPtr routing_graph_ptr_{nullptr};
  TrafficRulesPtr traffic_rules_ptr_{nullptr};

  std::shared_ptr<LaneletRoute> route_ptr_{nullptr};
  bool is_handler_ready_{false};

  Pose original_start_pose_{};
  Pose original_goal_pose_{};

  //Logger logger_{rosless_log::get_logger("route_handler")};

  // 내부 헬퍼 함수들 (원래 cpp 파일에 있던 것들 그대로 사용 가능)
  std::vector<lanelet::ConstLanelet> getMainLanelets(
    const lanelet::ConstLanelets & path_lanelets) const;
  lanelet::ConstLanelets getNeighborsWithinRoute(const lanelet::ConstLanelet & lanelet) const;
  bool hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const;
  bool isLaneletPartOfRoute(const lanelet::ConstLanelet & lanelet) const;
  bool isRouteLanelet(const lanelet::ConstLanelet & lanelet) const;
  bool isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const;

  lanelet::ConstLanelets getLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet, double min_length, bool only_route_lanes) const;
  lanelet::ConstLanelets getLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet, double min_length, bool only_route_lanes) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceAfter(
    const lanelet::ConstLanelet & lanelet, double min_length) const;
  lanelet::ConstLanelets getShoulderLaneletSequenceUpTo(
    const lanelet::ConstLanelet & lanelet, double min_length) const;
  bool getNextLaneletWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const;
  bool getNextLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * next_lanelets) const;
  bool getPreviousLaneletsWithinRoute(
    const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * previous_lanelets) const;
  lanelet::ConstLanelets getNextLanelets(const lanelet::ConstLanelet & lanelet) const;
  lanelet::ConstLanelets getPreviousLanelets(const lanelet::ConstLanelet & lanelet) const;
  std::optional<lanelet::ConstLanelet> getFollowingShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;
  std::optional<lanelet::ConstLanelet> getPreviousShoulderLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  bool isRouteLooped(const RouteSections & route_sections) const;
  void setLaneletsFromRouteMsg();
  std::vector<Waypoints> calcWaypointsVector(const lanelet::ConstLanelets & lanelet_sequence) const;
  void removeOverlappedCenterlineWithWaypoints(
    std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
    const std::vector<Point> & piecewise_waypoints,
    const lanelet::ConstLanelets & lanelet_sequence,
    size_t piecewise_waypoints_lanelet_sequence_index,
    bool is_removing_direction_forward) const;

  // etc...

  lanelet::ConstLanelets route_lanelets_;
  lanelet::ConstLanelets preferred_lanelets_;
  lanelet::ConstLanelets start_lanelets_;
  lanelet::ConstLanelets goal_lanelets_;
};

}  // namespace autoware::route_handler

#endif  // AUTOWARE__ROUTE_HANDLER__ROUTE_HANDLER_HPP_
