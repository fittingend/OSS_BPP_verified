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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_

#include "parameters.hpp"
#include "common_types.hpp"

//#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
//#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "trajectory.hpp"

#include "route_handler.hpp"
// #include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
// #include <autoware_lanelet2_extension/utility/utilities.hpp>
// #include <rclcpp/clock.hpp>
// #include <rclcpp/time.hpp>

// #include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
// #include <autoware_perception_msgs/msg/predicted_objects.hpp>
// #include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
// #include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
// #include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
// #include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
// #include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tier4_planning_msgs/msg/detail/velocity_limit__struct.hpp>
// #include <tier4_planning_msgs/msg/lateral_offset.hpp>
// #include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>



namespace autoware::behavior_path_planner
{

// --------------------------------------------------------------------------
//  단순 struct 로 변환된 data 타입들 (ROS 메시지 대체)
// --------------------------------------------------------------------------
using autoware::route_handler::RouteHandler;
using autoware::common_types::Header;
using autoware::common_types::Pose;
using autoware::common_types::Odometry;
using autoware::common_types::PathWithLaneId;
using autoware::common_types::PredictedObjects;
using autoware::common_types::OperationModeState;
using autoware::common_types::VelocityLimit;
using autoware::common_types::PoseWithUuidStamped;
using autoware::common_types::AccelWithCovarianceStamped;
using autoware::common_types::UUID;

//using autoware_adapi_v1_msgs::msg::OperationModeState;
//using autoware_perception_msgs::msg::PredictedObject;
//using autoware_perception_msgs::msg::PredictedObjects;
//using autoware_planning_msgs::msg::PoseWithUuidStamped;
//using geometry_msgs::msg::AccelWithCovarianceStamped;
//using geometry_msgs::msg::PoseStamped;
//using nav_msgs::msg::Odometry; 
//using tier4_planning_msgs::msg::PathWithLaneId;
//using PlanResult = PathWithLaneId::SharedPtr;
//using tier4_planning_msgs::msg::VelocityLimit;
//using unique_identifier_msgs::msg::UUID;

// --------------------------------------------------------------------------
//  사용하지 않는 데이터 타입들 
// --------------------------------------------------------------------------

//using autoware_perception_msgs::msg::TrafficLightGroup;
//using autoware_vehicle_msgs::msg::HazardLightsCommand;
//using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
//using nav_msgs::msg::OccupancyGrid;
//using tier4_planning_msgs::msg::LateralOffset; 
//sing lanelet::TrafficLight;


// BPP 파라미터 “설정값” (원래는 ROS param 이던 것들)
struct BehaviorPathPlannerConfig
{
  double traffic_light_signal_timeout{1.0};
  double planning_hz{10.0};

  double backward_path_length{5.0};
  double forward_path_length{300.0};

  double min_acc{-3.0};
  double max_acc{2.0};
  double max_vel{20.0};

  double backward_length_buffer_for_end_of_pull_over{5.0};
  double backward_length_buffer_for_end_of_pull_out{5.0};

  double minimum_pull_over_length{16.0};
  double refine_goal_search_radius_range{7.5};

  double turn_signal_intersection_search_distance{30.0};
  double turn_signal_intersection_angle_threshold_deg{15.0};
  double turn_signal_minimum_search_distance{10.0};
  double turn_signal_search_time{3.0};
  double turn_signal_shift_length_threshold{0.3};
  double turn_signal_remaining_shift_length_threshold{0.1};
  bool   turn_signal_on_swerving{true};

  bool   enable_akima_spline_first{false};
  bool   enable_cog_on_centerline{false};
  double input_path_interval{2.0};
  double output_path_interval{2.0};
  double ego_nearest_dist_threshold{1.0};
  double ego_nearest_yaw_threshold{0.785398}; // 45deg
};


//Behavior Path Planner가 매 사이클마다 참고하는 모든 입력 센서/지도/파라미터를 한 군데 모아둔 컨텍스트 구조체
struct PlannerData
{
  Odometry::ConstSharedPtr self_odometry{};
  AccelWithCovarianceStamped::ConstSharedPtr self_acceleration{};
  PredictedObjects::ConstSharedPtr dynamic_object{};

  //(dummy/null 가능)
  // OccupancyGrid::ConstSharedPtr occupancy_grid{};
  // OccupancyGrid::ConstSharedPtr costmap{};
  // LateralOffset::ConstSharedPtr lateral_offset{};

  //시뮬/테스트에선 고정 AUTONOMOUS 로 보고 생략 가능
  OperationModeState::ConstSharedPtr operation_mode{};

  //필수는 아님
  PathWithLaneId::SharedPtr prev_output_path{std::make_shared<PathWithLaneId>()};
  std::optional<PoseWithUuidStamped> prev_modified_goal{};
  std::optional<UUID> prev_route_id{};

  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};

  //std::map<int64_t, TrafficSignalStamped> traffic_light_id_map;

  //이 구조체가 있어야 planner가 길이, 가속도, 각종 threshold 를 알 수 있음
  BehaviorPathPlannerParameters parameters{};

  //최소 버전에서는 dummy init만 해서 실제 기능은 끄는 방향 가능
  // autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters
  //   drivable_area_expansion_parameters{};

  VelocityLimit::ConstSharedPtr external_limit_max_velocity{};

  mutable std::vector<Pose> drivable_area_expansion_prev_path_poses{};
  mutable std::vector<double> drivable_area_expansion_prev_curvatures{};
  // mutable TurnSignalDecider turn_signal_decider;

  // ----------------------------------------------------
  //  init_parameters: 순수 C++ 버전
  // ----------------------------------------------------
  void init_parameters(const VehicleInfo & vehicle_info,
                       const BehaviorPathPlannerConfig & cfg)
  {
    // parameters.traffic_light_signal_timeout =
    //   node.declare_parameter<double>("traffic_light_signal_timeout");

    // 1) 차량 제원 복사
    parameters.vehicle_info = vehicle_info;
    parameters.vehicle_width = vehicle_info.vehicle_width_m;
    parameters.vehicle_length = vehicle_info.vehicle_length_m;
    parameters.wheel_tread = vehicle_info.wheel_tread_m;
    parameters.wheel_base = vehicle_info.wheel_base_m;
    parameters.front_overhang = vehicle_info.front_overhang_m;
    parameters.rear_overhang = vehicle_info.rear_overhang_m;
    parameters.left_over_hang = vehicle_info.left_overhang_m;
    parameters.right_over_hang = vehicle_info.right_overhang_m;
    parameters.base_link2front = vehicle_info.max_longitudinal_offset_m;
    parameters.base_link2rear = parameters.rear_overhang;

    // NOTE: backward_path_length is used not only calculating path length but also calculating the
    // size of a drivable area.
    //       The drivable area has to cover not the base link but the vehicle itself. Therefore
    //       rear_overhang must be added to backward_path_length. In addition, because of the
    //       calculation of the drivable area in the autoware_path_optimizer package, the drivable
    //       area has to be a little longer than the backward_path_length parameter by adding
    //       min_backward_offset.
    
    // 2) backward_path_length 는 “차량 뒤 오버행 + margin”을 더해줌 (원래 주석 그대로)
    constexpr double min_backward_offset = 1.0;
    const double backward_offset = vehicle_info.rear_overhang_m + min_backward_offset;

    parameters.backward_path_length = cfg.backward_path_length + backward_offset;
    parameters.forward_path_length  = cfg.forward_path_length;

    // 3) 가속도 / 속도 파라미터
    parameters.min_acc = cfg.min_acc;
    parameters.max_acc = cfg.max_acc;
    parameters.max_vel = cfg.max_vel;

    parameters.backward_length_buffer_for_end_of_pull_over = 
      cfg.backward_length_buffer_for_end_of_pull_over;
    parameters.backward_length_buffer_for_end_of_pull_out  = 
      cfg.backward_length_buffer_for_end_of_pull_out;

    parameters.minimum_pull_over_length        = cfg.minimum_pull_over_length;
    parameters.refine_goal_search_radius_range = cfg.refine_goal_search_radius_range;

    // 4) turn signal 관련 (지금은 안 써도 되지만, struct 일관성 위해 채워만 둠)
    parameters.turn_signal_intersection_search_distance =
      cfg.turn_signal_intersection_search_distance;
    parameters.turn_signal_intersection_angle_threshold_deg =
      cfg.turn_signal_intersection_angle_threshold_deg;
    parameters.turn_signal_minimum_search_distance =
      cfg.turn_signal_minimum_search_distance;
    parameters.turn_signal_search_time =
      cfg.turn_signal_search_time;
    parameters.turn_signal_shift_length_threshold =
      cfg.turn_signal_shift_length_threshold;
    parameters.turn_signal_remaining_shift_length_threshold =
      cfg.turn_signal_remaining_shift_length_threshold;
    parameters.turn_signal_on_swerving =
      cfg.turn_signal_on_swerving;

    // 5) path 샘플링 / ego 근접 판단
    parameters.enable_akima_spline_first = cfg.enable_akima_spline_first;
    parameters.enable_cog_on_centerline  = cfg.enable_cog_on_centerline;
    parameters.input_path_interval       = cfg.input_path_interval;
    parameters.output_path_interval      = cfg.output_path_interval;
    parameters.ego_nearest_dist_threshold = cfg.ego_nearest_dist_threshold;
    parameters.ego_nearest_yaw_threshold  = cfg.ego_nearest_yaw_threshold;

    // 6) 신호등 타임아웃 (지금은 안 써도 되지만, 나중을 위해 유지)
    parameters.traffic_light_signal_timeout = cfg.traffic_light_signal_timeout;
  }

  // std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
  //   const PathWithLaneId & path, const size_t shift_start_idx, const size_t shift_end_idx,
  //   const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
  //   const bool is_driving_forward, const bool egos_lane_is_shifted,
  //   const bool override_ego_stopped_check = false, const bool is_pull_out = false,
  //   const bool is_lane_change = false, const bool is_pull_over = false) const
  // {
  //   if (shift_start_idx + 1 > path.points.size()) {
  //     RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
  //     return std::make_pair(TurnSignalInfo{}, true);
  //   }

  //   if (shift_end_idx + 1 > path.points.size()) {
  //     RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
  //     return std::make_pair(TurnSignalInfo{}, true);
  //   }

  //   std::vector<double> lengths(path.points.size(), 0.0);
  //   ShiftedPath shifted_path{path, lengths};
  //   ShiftLine shift_line;

  //   {
  //     const auto start_pose = path.points.at(shift_start_idx).point.pose;
  //     const auto start_shift_length =
  //       lanelet::utils::getArcCoordinates(current_lanelets, start_pose).distance;
  //     const auto end_pose = path.points.at(shift_end_idx).point.pose;
  //     const auto end_shift_length =
  //       lanelet::utils::getArcCoordinates(current_lanelets, end_pose).distance;
  //     shifted_path.shift_length.at(shift_start_idx) = start_shift_length;
  //     shifted_path.shift_length.at(shift_end_idx) = end_shift_length;

  //     shift_line.start = start_pose;
  //     shift_line.end = end_pose;
  //     shift_line.start_shift_length = start_shift_length;
  //     shift_line.end_shift_length = end_shift_length;
  //     shift_line.start_idx = shift_start_idx;
  //     shift_line.end_idx = shift_end_idx;
  //   }

  //   return turn_signal_decider.getBehaviorTurnSignalInfo(
  //     shifted_path, shift_line, current_lanelets, route_handler, parameters, self_odometry,
  //     current_shift_length, is_driving_forward, egos_lane_is_shifted, override_ego_stopped_check,
  //     is_pull_out, is_lane_change, is_pull_over);
  // }

  // std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
  //   const ShiftedPath & path, const ShiftLine & shift_line,
  //   const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
  //   const bool is_driving_forward, const bool egos_lane_is_shifted,
  //   const bool override_ego_stopped_check = false, const bool is_pull_out = false) const
  // {
  //   return turn_signal_decider.getBehaviorTurnSignalInfo(
  //     path, shift_line, current_lanelets, route_handler, parameters, self_odometry,
  //     current_shift_length, is_driving_forward, egos_lane_is_shifted, override_ego_stopped_check,
  //     is_pull_out);
  // }

  // TurnIndicatorsCommand getTurnSignal(
  //   const PathWithLaneId & path, const TurnSignalInfo & turn_signal_info,
  //   TurnSignalDebugData & debug_data)
  // {
  //   const auto & current_pose = self_odometry->pose.pose;
  //   const auto & current_vel = self_odometry->twist.twist.linear.x;
  //   return turn_signal_decider.getTurnSignal(
  //     route_handler, path, turn_signal_info, current_pose, current_vel, parameters, debug_data);
  // }

  // std::optional<TrafficSignalStamped> getTrafficSignal(const int64_t id) const
  // {
  //   if (traffic_light_id_map.count(id) == 0) {
  //     return std::nullopt;
  //   }

  //   const auto elapsed_time =
  //     (rclcpp::Clock{RCL_ROS_TIME}.now() - traffic_light_id_map.at(id).stamp).seconds();
  //   if (elapsed_time > parameters.traffic_light_signal_timeout) {
  //     return std::nullopt;
  //   }

  //   return traffic_light_id_map.at(id);
  // }

  //가장 단순한 방식: “거리 기반 nearest index”
  template <class T>
  size_t findEgoIndex(const std::vector<T> & points, const Pose & ego_pose) const
  {
    if constexpr (std::is_same_v<T, autoware::common_types::PathPointWithLaneId>) {
      return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        points, ego_pose, parameters.ego_nearest_dist_threshold,
        parameters.ego_nearest_yaw_threshold);
    } else {
      double min_dist = std::numeric_limits<double>::max();
      size_t min_idx = 0;
      for (size_t i = 0; i < points.size(); ++i) {
        const auto & p = points[i].point.pose.position;
        const double dx = p.x - ego_pose.position.x;
        const double dy = p.y - ego_pose.position.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
          min_dist = dist;
          min_idx = i;
        }
      }
      return min_idx;
    }
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points, const Pose & ego_pose) const
  {
    if constexpr (std::is_same_v<T, autoware::common_types::PathPointWithLaneId>) {
      return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        points, ego_pose, parameters.ego_nearest_dist_threshold,
        parameters.ego_nearest_yaw_threshold);
    } else {
      const size_t idx = findEgoIndex(points, ego_pose);
      return (idx == 0) ? 0 : idx - 1;
    }
  }
  
  // template <class T>
  // size_t findEgoIndex(const std::vector<T> & points) const
  // {
  //   return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
  //     points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
  //     parameters.ego_nearest_yaw_threshold);
  // }

  // template <class T>
  // size_t findEgoSegmentIndex(const std::vector<T> & points) const
  // {
  //   return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
  //     points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
  //     parameters.ego_nearest_yaw_threshold);
  // }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
