#pragma once

#include <boost/scope_exit.hpp>
#include <lanelet2_core/LaneletMap.h>
#include "common_types.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

// ===== Autoware 공통 타입들: 기존 헤더에서 가져온다고 가정 =====

struct PlannerData;
using autoware::common_types::BehaviorModuleOutput;

class SceneModuleInterface;
class SceneModuleManagerInterface;
class SceneModuleVisitor;

enum class ModuleStatus;
enum class SlotStatus;

struct SlotOutput;

struct SceneModuleUpdateInfo
{
  enum class Action { NONE, ADD, DELETE, MOVE };

  SceneModuleUpdateInfo(
    std::string module_name, ModuleStatus module_status, Action action, std::string description)
  : module_name(std::move(module_name)),
    status(module_status),
    action(action),
    description(std::move(description))
  {
  }

  std::string module_name;
  ModuleStatus status;
  Action action;
  std::string description;
};

struct DebugInfo
{
  std::vector<SceneModuleUpdateInfo> scene_status;
  std::vector<SlotStatus> slot_status;
};

using SceneModulePtr = std::shared_ptr<SceneModuleInterface>;
using SceneModuleManagerPtr = std::shared_ptr<SceneModuleManagerInterface>;

// 기존 StopWatch 대체 (ros utils 없이)
template<class T>
class StopWatchStandalone
{
public:
  void tic(const std::string & key)
  {
    times_[key] = std::chrono::steady_clock::now();
  }

  double toc(const std::string & key, bool reset = false)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto it = times_.find(key);
    if (it == times_.end()) return 0.0;
    const auto dt =
      std::chrono::duration_cast<T>(now - it->second).count();
    if (reset) times_.erase(it);
    return static_cast<double>(dt);
  }

private:
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> times_;
};

// ================= SubPlannerManager (ROS 의존 없음) =================
/*
class SubPlannerManager
{
public:
  SubPlannerManager(
    const std::shared_ptr<std::optional<lanelet::ConstLanelet>> & current_route_lanelet,
    std::unordered_map<std::string, double> & processing_time,
    DebugInfo & debug_info);

  void addSceneModuleManager(const SceneModuleManagerPtr & manager);
  const std::vector<SceneModuleManagerPtr> & getSceneModuleManager() const;

  std::vector<SceneModulePtr> approved_modules() const;
  std::vector<SceneModulePtr> candidate_modules() const;

  template<class Pred>
  bool isAnyApprovedPred(Pred pred) const
  {
    return std::any_of(approved_module_ptrs_.begin(), approved_module_ptrs_.end(), pred);
  }

  template<class Pred>
  bool isAnyCandidatePred(Pred pred) const
  {
    return std::any_of(candidate_module_ptrs_.begin(), candidate_module_ptrs_.end(), pred);
  }

  SlotOutput propagateFull(
    const std::shared_ptr<PlannerData> & data,
    const SlotOutput & previous_slot_output);

  SlotOutput propagateWithExclusiveCandidate(
    const std::shared_ptr<PlannerData> & data,
    const SlotOutput & previous_slot_output);

  void propagateWithFailedApproved();

  SlotOutput propagateWithWaitingApproved(
    const std::shared_ptr<PlannerData> & data,
    const SlotOutput & previous_slot_output);

private:
  friend class PlannerManager;

  std::vector<SceneModulePtr> getRequestModules(
    const BehaviorModuleOutput & previous_module_output,
    const std::vector<SceneModulePtr> & deleted_modules) const;

  SceneModulePtr selectHighestPriorityModule(
    std::vector<SceneModulePtr> & request_modules) const;

  void clearApprovedModules();

  void updateCandidateModules(
    const std::vector<SceneModulePtr> & request_modules,
    const SceneModulePtr & highest_priority_module);

  std::pair<SceneModulePtr, BehaviorModuleOutput> runRequestModules(
    const std::vector<SceneModulePtr> & request_modules,
    const std::shared_ptr<PlannerData> & data,
    const BehaviorModuleOutput & previous_module_output);

  BehaviorModuleOutput run(
    const SceneModulePtr & module_ptr,
    const std::shared_ptr<PlannerData> & planner_data,
    const BehaviorModuleOutput & previous_module_output) const;

  SlotOutput runApprovedModules(
    const std::shared_ptr<PlannerData> & data,
    const BehaviorModuleOutput & upstream_slot_output);

  bool isAnyCandidateExclusive() const;

  void addApprovedModule(const SceneModulePtr & module_ptr);
  void clearCandidateModules();
  void deleteExpiredModules(const SceneModulePtr & module_ptr);
  void resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data) const;
  const SceneModuleManagerPtr & getManager(const SceneModulePtr & module_ptr) const;
  void sortByPriority(std::vector<SceneModulePtr> & modules) const;

private:
  std::vector<SceneModuleManagerPtr> manager_ptrs_;
  std::vector<SceneModulePtr> candidate_module_ptrs_;
  std::vector<SceneModulePtr> approved_module_ptrs_;

  std::shared_ptr<std::optional<lanelet::ConstLanelet>> current_route_lanelet_;
  std::unordered_map<std::string, double> & processing_time_;
  DebugInfo & debug_info_;
};
*/
// ================= PlannerManager (핵심) =================

class PlannerManager {
public:
  PlannerManager();

  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);

  void generateCombinedDrivableArea(
    BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & data) const;

  void updateCurrentRouteLanelet(
    const std::shared_ptr<PlannerData> & data, bool is_any_approved_module_running);

  BehaviorModuleOutput getReferencePath(
    const std::shared_ptr<PlannerData> & data) const;

  void publishDebugRootReferencePath(
    const BehaviorModuleOutput & reference_path) const;

  void resetCurrentRouteLanelet(const std::shared_ptr<PlannerData> & data);

private:
  std::shared_ptr<std::optional<lanelet::ConstLanelet>> current_route_lanelet_;
  std::unordered_map<std::string, double> processing_time_;
  DebugInfo debug_info_;

  void resetProcessingTime();
};


}  // namespace autoware::behavior_path_planner
