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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_

//#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

//용도: 여러 behavior 모듈(LaneChange, Avoidance, PullOver …)을 켜고 끄는 플래그
//테스트 시나리오에서는 해당사항 없어서 삭제
// struct ModuleConfigParameters
// {
//   bool enable_module{false};
//   bool enable_rtc{false};
//   bool enable_simultaneous_execution_as_approved_module{false};
//   bool enable_simultaneous_execution_as_candidate_module{false};
// };


#include <cstdint>

namespace autoware::behavior_path_planner
{

struct VehicleInfo
{
  double vehicle_width_m{1.8};
  double vehicle_length_m{4.5};
  double wheel_tread_m{1.5};
  double wheel_base_m{2.7};
  double front_overhang_m{0.9};
  double rear_overhang_m{0.9};
  double left_overhang_m{0.2};
  double right_overhang_m{0.2};
  double max_longitudinal_offset_m{4.5};
};

struct BehaviorPathPlannerParameters
{
  // --- 차량 제원 ---
  VehicleInfo vehicle_info{};
  double wheel_base{0.0};
  double front_overhang{0.0};
  double rear_overhang{0.0};
  double vehicle_width{0.0};
  double vehicle_length{0.0};
  double wheel_tread{0.0};
  double left_over_hang{0.0};
  double right_over_hang{0.0};
  double base_link2front{0.0};
  double base_link2rear{0.0};

  // --- 경로 길이 ---
  double backward_path_length{20.0};
  double forward_path_length{400.0};

  // --- 속도/가속도 제한 ---
  double min_acc{-3.0};
  double max_acc{2.0};
  double max_vel{20.0};

  // [m] 풀오버 끝에서 여유 거리 버퍼
  double backward_length_buffer_for_end_of_pull_over{5.0};
  double backward_length_buffer_for_end_of_pull_out{5.0};

  // --- goal 근처 path 보정 ---
  double minimum_pull_over_length{16.0};
  double refine_goal_search_radius_range{7.5};
  double minimum_pull_out_length{16.0};
  double refine_start_search_radius_range{7.5};

  //신호등관련
  double turn_signal_intersection_search_distance{30.0};
  double turn_signal_intersection_angle_threshold_deg{15.0};
  double turn_signal_minimum_search_distance{10.0};
  double turn_signal_search_time{3.0};
  double turn_signal_shift_length_threshold{0.3};
  double turn_signal_remaining_shift_length_threshold{0.1};
  bool   turn_signal_on_swerving{true};
  double traffic_light_signal_timeout{1.0};

  // --- path 보간/샘플링 ---
  bool   enable_akima_spline_first{false};
  bool   enable_cog_on_centerline{false};
  double input_path_interval{2.0};
  double output_path_interval{2.0};

  // --- ego와 path 매칭 허용 오차 ---
  double ego_nearest_dist_threshold{3.0};
  double ego_nearest_yaw_threshold{1.046};  // 60도
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_
