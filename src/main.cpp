#include "common_types.hpp"
#include "route_handler.hpp"
#include "data_manager.hpp"
#include "planner_manager.hpp"
#include "path_utils.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{
using autoware::behavior_path_planner::BehaviorPathPlannerConfig;
using autoware::behavior_path_planner::BehaviorModuleOutput;
using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::PlannerManager;
using autoware::behavior_path_planner::VehicleInfo;
using autoware::behavior_path_planner::PathWithLaneId;
using autoware::common_types::KinematicState;
using autoware::common_types::TimeStamp;
using autoware::common_types::LaneletPrimitive;
using autoware::common_types::LaneletRoute;
using autoware::common_types::LaneletSegment;
using autoware::common_types::Odometry;
using autoware::common_types::PointXYZ;
using autoware::common_types::Pose;
using autoware::common_types::UUID;
using autoware::route_handler::RouteHandler;
using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;

const std::string kOutputTopic = "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id";

double extractYawFromOrientation(const autoware::common_types::Orientation & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct MissionData
{
  Pose start_pose;
  Pose goal_pose;
  std::vector<std::int64_t> lanelet_ids;
  UUID uuid{};
  bool allow_modification{false};
};

struct MissionUpdate
{
  MissionData mission;
  std::int64_t timestamp_ns{0};
};

struct LocalizationFrame
{
  KinematicState state;
  std::int64_t timestamp_ns{0};
};

struct MapInfo
{
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
  std::size_t lanelet_count{0};
};

MapInfo computeMapInfo(const lanelet::LaneletMapPtr & map)
{
  MapInfo info;
  if (!map) {
    return info;
  }
  for (const auto & lanelet : map->laneletLayer) {
    ++info.lanelet_count;
    for (const auto & pt : lanelet.centerline()) {
      const auto & p = pt.basicPoint();
      info.min_x = std::min(info.min_x, p.x());
      info.max_x = std::max(info.max_x, p.x());
      info.min_y = std::min(info.min_y, p.y());
      info.max_y = std::max(info.max_y, p.y());
    }
  }
  return info;
}

std::shared_ptr<RouteHandler> createRouteHandler(const lanelet::LaneletMapPtr & map)
{
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  auto routing_graph_unique = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  lanelet::routing::RoutingGraphPtr routing_graph(routing_graph_unique.release());
  return std::make_shared<RouteHandler>(map, routing_graph);
}

void applyVelocityProfile(PathWithLaneId & path, const KinematicState * ego_state)
{
  if (path.points.empty()) {
    return;
  }
  if (path.points.size() == 1) {
    path.points.front().point.longitudinal_velocity_mps = 0.0;
    return;
  }

  const double lane_speed = path.points.front().point.longitudinal_velocity_mps;
  const double ego_speed =
    ego_state ? std::abs(ego_state->twist.linear_x) : lane_speed;
  const double cruise_speed = std::max(lane_speed, ego_speed);

  for (std::size_t i = 0; i + 1 < path.points.size(); ++i) {
    path.points[i].point.longitudinal_velocity_mps = cruise_speed;
  }
  path.points.back().point.longitudinal_velocity_mps = 0.0;
}

double calcDistance2d(const lanelet::BasicPoint2d & a, const lanelet::BasicPoint2d & b)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  return std::sqrt(dx * dx + dy * dy);
}

double calcDistanceToSegment(
  const lanelet::BasicPoint2d & point, const lanelet::BasicPoint2d & start,
  const lanelet::BasicPoint2d & end)
{
  const double seg_dx = end.x() - start.x();
  const double seg_dy = end.y() - start.y();
  const double length_sq = seg_dx * seg_dx + seg_dy * seg_dy;
  if (length_sq < std::numeric_limits<double>::epsilon()) {
    return calcDistance2d(point, start);
  }
  const double t = ((point.x() - start.x()) * seg_dx + (point.y() - start.y()) * seg_dy) / length_sq;
  const double clamped = std::clamp(t, 0.0, 1.0);
  const lanelet::BasicPoint2d proj(start.x() + clamped * seg_dx, start.y() + clamped * seg_dy);
  return calcDistance2d(point, proj);
}

double calcDistanceToLaneletCenterline(const lanelet::ConstLanelet & lanelet, const Pose & pose)
{
  const auto & centerline = lanelet.centerline2d();
  if (centerline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const lanelet::BasicPoint2d pose_point(pose.position.x, pose.position.y);
  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t idx = 1; idx < centerline.size(); ++idx) {
    const auto & prev = centerline[idx - 1];
    const auto & curr = centerline[idx];
    const auto prev_point = lanelet::BasicPoint2d(prev.x(), prev.y());
    const auto curr_point = lanelet::BasicPoint2d(curr.x(), curr.y());
    const double dist = calcDistanceToSegment(pose_point, prev_point, curr_point);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

double calcPoseDistance(const Pose & lhs, const Pose & rhs)
{
  const double dx = lhs.position.x - rhs.position.x;
  const double dy = lhs.position.y - rhs.position.y;
  const double dz = lhs.position.z - rhs.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

json extractMessage(const json & entry)
{
  if (entry.contains("message") && entry["message"].is_object()) {
    return entry["message"];
  }
  return entry;
}

std::vector<std::int64_t> laneIdsFromJson(const json & point_json)
{
  std::vector<std::int64_t> ids;
  if (!point_json.contains("lane_ids")) {
    return ids;
  }
  for (const auto & id_entry : point_json["lane_ids"]) {
    if (id_entry.is_number_integer()) {
      ids.push_back(id_entry.get<std::int64_t>());
    }
  }
  return ids;
}

struct FrameComparison
{
  std::size_t answer_point_count{0};
  double max_point_diff{0.0};
  double end_point_diff{0.0};
  bool lane_id_mismatch{false};
};

struct AnswerFrame
{
  json message;
  std::optional<std::int64_t> timestamp_ns;
};

TimeStamp currentTimeStamp()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto secs = time_point_cast<seconds>(now);
  const auto nsec = duration_cast<nanoseconds>(now - secs);
  TimeStamp stamp;
  stamp.sec = secs.time_since_epoch().count();
  stamp.nsec = static_cast<std::int64_t>(nsec.count());
  return stamp;
}

std::optional<std::int64_t> extractTimestampFromMessage(const json & message)
{
  const auto header_it = message.find("header");
  if (header_it == message.end()) {
    return std::nullopt;
  }
  const auto & header = *header_it;
  const auto stamp_it = header.find("stamp");
  if (stamp_it == header.end()) {
    return std::nullopt;
  }
  const auto & stamp = *stamp_it;
  const auto sec = stamp.value("sec", 0LL);
  const auto nsec = stamp.value("nanosec", 0LL);
  return sec * 1'000'000'000LL + nsec;
}

std::optional<TimeStamp> extractHeaderStamp(const json & message)
{
  const auto header_it = message.find("header");
  if (header_it == message.end()) {
    return std::nullopt;
  }
  const auto & header = *header_it;
  const auto stamp_it = header.find("stamp");
  if (stamp_it == header.end()) {
    return std::nullopt;
  }
  const auto & stamp = *stamp_it;
  TimeStamp out{};
  out.sec = stamp.value("sec", 0LL);
  out.nsec = stamp.value("nanosec", 0LL);
  return out;
}

std::optional<std::int64_t> extractTimestampFromDocument(const json & document)
{
  if (document.contains("timestamp") && document["timestamp"].is_number_integer()) {
    return document["timestamp"].get<std::int64_t>();
  }
  if (document.contains("message") && document["message"].is_object()) {
    return extractTimestampFromMessage(document["message"]);
  }
  return extractTimestampFromMessage(document);
}

double computeMessagePathLength(const json & message)
{
  const auto & points = message.value("points", json::array());
  double length = 0.0;
  for (size_t idx = 1; idx < points.size(); ++idx) {
    const auto & prev = points[idx - 1]["point"]["pose"]["position"];
    const auto & curr = points[idx]["point"]["pose"]["position"];
    const double dx = curr.value("x", 0.0) - prev.value("x", 0.0);
    const double dy = curr.value("y", 0.0) - prev.value("y", 0.0);
    const double dz = curr.value("z", 0.0) - prev.value("z", 0.0);
    length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return length;
}

FrameComparison comparePathToAnswer(const PathWithLaneId & path, const json * answer_msg)
{
  FrameComparison result{};
  if (!answer_msg) {
    return result;
  }
  const auto & answer_points = answer_msg->value("points", json::array());
  result.answer_point_count = answer_points.size();
  const std::size_t pair_count = std::min(path.points.size(), answer_points.size());
  for (std::size_t idx = 0; idx < pair_count; ++idx) {
    const auto & our_point = path.points[idx];
    const auto & ans_point = answer_points[idx];
    const auto our_pose = our_point.point.pose.position;
    const auto ans_pose = ans_point["point"]["pose"]["position"];
    const double dx = our_pose.x - ans_pose.value("x", 0.0);
    const double dy = our_pose.y - ans_pose.value("y", 0.0);
    const double dz = our_pose.z - ans_pose.value("z", 0.0);
    const double diff = std::sqrt(dx * dx + dy * dy + dz * dz);
    result.max_point_diff = std::max(result.max_point_diff, diff);
    const auto ans_lane_ids = laneIdsFromJson(ans_point);
    if (our_point.lane_ids.size() != ans_lane_ids.size()) {
      result.lane_id_mismatch = true;
    } else {
      for (std::size_t id_idx = 0; id_idx < our_point.lane_ids.size(); ++id_idx) {
        if (our_point.lane_ids[id_idx] != ans_lane_ids[id_idx]) {
          result.lane_id_mismatch = true;
          break;
        }
      }
    }
  }
  if (!path.points.empty() && !answer_points.empty()) {
    const auto & last_point = path.points.back().point.pose.position;
    const auto & ans_last = answer_points.back()["point"]["pose"]["position"];
    const double dx = last_point.x - ans_last.value("x", 0.0);
    const double dy = last_point.y - ans_last.value("y", 0.0);
    const double dz = last_point.z - ans_last.value("z", 0.0);
    result.end_point_diff = std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return result;
}

void logRouteLanelets(std::ostream & os, const std::vector<lanelet::ConstLanelet> & lanelets)
{
  os << "    Route lanelets (" << lanelets.size() << " entries): ";
  for (std::size_t idx = 0; idx < lanelets.size(); ++idx) {
    if (idx > 0) {
      os << ", ";
    }
    os << lanelets[idx].id();
    if (idx >= 9) {
      os << (lanelets.size() > 10 ? ", ..." : "");
      break;
    }
  }
  os << '\n';
}

std::vector<AnswerFrame> loadAnswerMessages(const std::filesystem::path & path)
{
  std::vector<AnswerFrame> messages;
  std::ifstream stream(path);
  if (!stream.is_open()) {
    std::cerr << "[Answer] Unable to open answer path: " << path << "\n";
    return messages;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    try {
      const auto document = json::parse(line);
      const AnswerFrame frame{
        extractMessage(document),
        extractTimestampFromDocument(document),
      };
      messages.push_back(frame);
    } catch (const std::exception & e) {
      std::cerr << "[Answer] Failed to parse JSONL line: " << e.what() << "\n";
    }
  }
  return messages;
}

void logFrameDiagnostics(
  std::ostream & os, std::size_t frame_idx,
  const LocalizationFrame * localization, const MissionData & mission,
  const std::shared_ptr<RouteHandler> & route_handler, const PathWithLaneId & path,
  const json * answer_msg)
{
  os << "==== Frame " << frame_idx << " diagnostics ====\n";
  os << std::fixed << std::setprecision(3);

  if (localization) {
    const auto & pose = localization->state.pose;
    os << "  Ego pose: (" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")\n";
    os << "  Ego linear velocity: " << localization->state.twist.linear_x << " m/s\n";
  } else {
    os << "  Ego pose: <none>\n";
  }

  if (!mission.lanelet_ids.empty()) {
    os << "  Mission goal: (" << mission.goal_pose.position.x << ", " << mission.goal_pose.position.y << ", "
       << mission.goal_pose.position.z << ")\n";
    os << "  Mission lanelet IDs: [";
    for (std::size_t i = 0; i < mission.lanelet_ids.size(); ++i) {
      if (i > 0) {
        os << ", ";
      }
      os << mission.lanelet_ids[i];
    }
    os << "]\n";
  } else {
    os << "  Mission: <no lanelets>\n";
  }

  if (route_handler && route_handler->isHandlerReady()) {
    const auto lanelets = route_handler->getRouteLanelets();
    logRouteLanelets(os, lanelets);
    if (localization) {
      lanelet::ConstLanelet nearest;
      if (route_handler->getClosestLaneletWithinRoute(localization->state.pose, &nearest)) {
        const double dist = calcDistanceToLaneletCenterline(nearest, localization->state.pose);
        os << "    Nearest lanelet id: " << nearest.id() << ", distance to pose: " << dist << " m\n";
      }
    }
    const auto goal_pose = route_handler->getGoalPose();
    os << "    Route handler goal: (" << goal_pose.position.x << ", " << goal_pose.position.y << ")\n";
  } else {
    os << "  Route handler not ready\n";
  }

  const FrameComparison answer_view = comparePathToAnswer(path, answer_msg);
  os << "  Path points: " << path.points.size();
  if (!path.points.empty()) {
    const auto & start = path.points.front().point.pose.position;
    const auto & end = path.points.back().point.pose.position;
    os << " (start: [" << start.x << ", " << start.y << "], end: [" << end.x << ", " << end.y << "])\n";
    if (!mission.lanelet_ids.empty()) {
      const Pose path_goal_pose = path.points.back().point.pose;
      const double goal_distance = calcPoseDistance(path_goal_pose, mission.goal_pose);
      os << "    Goal difference (mission vs path end): " << goal_distance << " m\n";
    }
    auto printLaneSet = [&](const std::vector<std::int64_t> & ids, const std::string & label) {
      if (ids.empty()) {
        return;
      }
      os << "    " << label << ": ";
      const size_t limit = std::min<size_t>(ids.size(), 5);
      for (size_t idx = 0; idx < limit; ++idx) {
        if (idx > 0) {
          os << ", ";
        }
        os << ids[idx];
      }
      if (ids.size() > limit) {
        os << ", ...";
      }
      os << '\n';
    };
    printLaneSet(path.points.front().lane_ids, "Front lane IDs");
    printLaneSet(path.points.back().lane_ids, "End lane IDs");
    os << "    Sample point lane IDs:\n";
    const size_t preview = std::min<size_t>(3, path.points.size());
    for (size_t idx = 0; idx < preview; ++idx) {
      printLaneSet(path.points[idx].lane_ids, "point[" + std::to_string(idx) + "] lane IDs");
    }
    for (size_t idx = 0; idx < preview && idx < path.points.size(); ++idx) {
      const size_t rev_idx = path.points.size() - 1 - idx;
      if (rev_idx <= idx) {
        break;
      }
      printLaneSet(path.points[rev_idx].lane_ids, "point[" + std::to_string(rev_idx) + "] lane IDs");
    }
  } else {
    os << '\n';
  }
  os << "    Lane ID mismatch with answer: " << (answer_view.lane_id_mismatch ? "yes" : "no") << '\n';
  os << "    Max point diff vs answer: " << answer_view.max_point_diff << " m\n";
  os << "    End point diff vs answer: " << answer_view.end_point_diff << " m\n";
  if (answer_msg) {
    os << "  Answer point count: " << answer_view.answer_point_count << '\n';
    const auto & ans_points = answer_msg->value("points", json::array());
    if (!ans_points.empty()) {
      const auto & ans_start = ans_points.front()["point"]["pose"]["position"];
      const auto & ans_end = ans_points.back()["point"]["pose"]["position"];
      os << "    Answer start: [" << ans_start.value("x", 0.0) << ", " << ans_start.value("y", 0.0) << "]\n";
      os << "    Answer end: [" << ans_end.value("x", 0.0) << ", " << ans_end.value("y", 0.0) << "]\n";
    }
  }
  os << "==== End Frame " << frame_idx << " ====\n\n";
  os << std::defaultfloat;
}

Pose parsePose(const json & pose_json)
{
  Pose pose;
  if (const auto pos_it = pose_json.find("position"); pos_it != pose_json.end() && pos_it->is_object()) {
    pose.position.x = pos_it->value("x", pose.position.x);
    pose.position.y = pos_it->value("y", pose.position.y);
    pose.position.z = pos_it->value("z", pose.position.z);
  }
  if (const auto ori_it = pose_json.find("orientation"); ori_it != pose_json.end() && ori_it->is_object()) {
    pose.orientation.x = ori_it->value("x", pose.orientation.x);
    pose.orientation.y = ori_it->value("y", pose.orientation.y);
    pose.orientation.z = ori_it->value("z", pose.orientation.z);
    pose.orientation.w = ori_it->value("w", pose.orientation.w);
  }
  return pose;
}

std::optional<MissionUpdate> parseMissionLine(const std::string & line)
{
  if (line.empty()) {
    return std::nullopt;
  }
  try {
    const auto document = json::parse(line);
    MissionUpdate update;
    update.timestamp_ns = document.value("timestamp", 0LL);
    const auto & message = document.at("message");
    update.mission.start_pose = parsePose(message.at("start_pose"));
    update.mission.goal_pose = parsePose(message.at("goal_pose"));

    if (message.contains("segments")) {
      for (const auto & segment : message.at("segments")) {
        if (segment.contains("preferred_primitive") && segment["preferred_primitive"].contains("id")) {
          update.mission.lanelet_ids.push_back(
            segment["preferred_primitive"]["id"].get<std::int64_t>());
        }
      }
    }

    update.mission.allow_modification = message.value("allow_modification", false);

    if (const auto uuid_it = message.find("uuid"); uuid_it != message.end()) {
      if (const auto inner = uuid_it->find("uuid"); inner != uuid_it->end() && inner->is_array()) {
        const auto & array = *inner;
        const auto limit = std::min<std::size_t>(array.size(), sizeof(update.mission.uuid.bytes));
        for (std::size_t idx = 0; idx < limit; ++idx) {
          update.mission.uuid.bytes[idx] = static_cast<std::uint8_t>(array[idx].get<int>());
        }
      }
    }

    if (update.mission.lanelet_ids.empty()) {
      std::cerr << "[Mission] No preferred lane IDs found in mission entry.\n";
      return std::nullopt;
    }

    return update;
  } catch (const std::exception & e) {
    std::cerr << "[Mission] Failed to parse mission line: " << e.what() << "\n";
    return std::nullopt;
  }
}

std::optional<LocalizationFrame> parseLocalizationLine(const std::string & line)
{
  if (line.empty()) {
    return std::nullopt;
  }
  try {
    const auto document = json::parse(line);
    LocalizationFrame frame;
    frame.timestamp_ns = document.value("timestamp", 0LL);
    const auto & message = document.at("message");

    if (message.contains("header")) {
      const auto & header = message.at("header");
      frame.state.header.frame_id = header.value("frame_id", frame.state.header.frame_id);
      if (header.contains("stamp")) {
        const auto & stamp = header.at("stamp");
        frame.state.header.stamp.sec = stamp.value("sec", frame.state.header.stamp.sec);
        frame.state.header.stamp.nsec = stamp.value("nanosec", frame.state.header.stamp.nsec);
      }
    }

    if (message.contains("pose") && message["pose"].contains("pose")) {
      frame.state.pose = parsePose(message["pose"]["pose"]);
    }

    if (message.contains("twist") && message["twist"].contains("twist")) {
      const auto & twist = message["twist"]["twist"];
      if (twist.contains("linear")) {
        const auto & linear = twist["linear"];
        frame.state.twist.linear_x = linear.value("x", frame.state.twist.linear_x);
        frame.state.twist.linear_y = linear.value("y", frame.state.twist.linear_y);
        frame.state.twist.linear_z = linear.value("z", frame.state.twist.linear_z);
      }
      if (twist.contains("angular")) {
        const auto & angular = twist["angular"];
        frame.state.twist.angular_x = angular.value("x", frame.state.twist.angular_x);
        frame.state.twist.angular_y = angular.value("y", frame.state.twist.angular_y);
        frame.state.twist.angular_z = angular.value("z", frame.state.twist.angular_z);
      }
    }

    return frame;
  } catch (const std::exception & e) {
    std::cerr << "[Localization] Failed to parse localization line: " << e.what() << "\n";
    return std::nullopt;
  }
}

std::vector<LocalizationFrame> loadLocalizationFrames(const std::filesystem::path & path)
{
  std::vector<LocalizationFrame> frames;
  std::ifstream stream(path);
  if (!stream.is_open()) {
    std::cerr << "[Localization] Unable to open: " << path << "\n";
    return frames;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    if (const auto parsed = parseLocalizationLine(line)) {
      frames.push_back(*parsed);
    }
  }
  return frames;
}

std::vector<MissionUpdate> loadMissionUpdates(const std::filesystem::path & path)
{
  std::vector<MissionUpdate> updates;
  std::ifstream stream(path);
  if (!stream.is_open()) {
    std::cerr << "[Mission] Unable to open: " << path << "\n";
    return updates;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    if (const auto parsed = parseMissionLine(line)) {
      updates.push_back(*parsed);
    }
  }
  return updates;
}

std::size_t countJsonlFrames(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return 0;
  }
  std::size_t count = 0;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    ++count;
  }
  return count;
}

LaneletRoute buildRouteMessage(const MissionData & mission)
{
  LaneletRoute route_msg;
  route_msg.header.frame_id = "map";
  route_msg.start_pose = mission.start_pose;
  route_msg.goal_pose = mission.goal_pose;
  route_msg.allow_modification = mission.allow_modification;
  route_msg.uuid = mission.uuid;
  route_msg.segments.reserve(mission.lanelet_ids.size());

  for (const auto id : mission.lanelet_ids) {
    LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    LaneletPrimitive primitive = segment.preferred_primitive;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
    route_msg.segments.push_back(segment);
  }

  return route_msg;
}

bool areUuidsEqual(const UUID & lhs, const UUID & rhs)
{
  for (std::size_t idx = 0; idx < sizeof(lhs.bytes); ++idx) {
    if (lhs.bytes[idx] != rhs.bytes[idx]) {
      return false;
    }
  }
  return true;
}

bool applyMissionRoute(
  const MissionData & mission, const std::shared_ptr<RouteHandler> & route_handler,
  const std::shared_ptr<PlannerData> & planner_data, PlannerManager & planner_manager)
{
  if (!mission.lanelet_ids.empty()) {
    // Autoware ref: behavior_path_planner_node.cpp::run() resets scene modules when a new uuid route arrives.
    const auto route_msg = buildRouteMessage(mission);
    route_handler->setRoute(route_msg);
    planner_data->route_handler = route_handler;

    const bool had_prev_route = planner_data->prev_route_id.has_value();
    const bool has_same_route = had_prev_route && areUuidsEqual(*planner_data->prev_route_id, route_msg.uuid);
    if (!had_prev_route || !has_same_route) {
      planner_manager.resetCurrentRouteLanelet(planner_data);
    }
    planner_data->prev_route_id = route_msg.uuid;

    if (!route_handler->isHandlerReady()) {
      std::cerr << "[Mission] Route handler failed to set the route.\n";
      return false;
    }
    return true;
  }
  std::cerr << "[Mission] Mission does not provide lanelet IDs.\n";
  return false;
}

void writePathJson(const PathWithLaneId & path, std::ostream & os, std::int64_t timestamp_ns)
{
  auto convertBoundOrdered = [](const std::vector<PointXYZ> & bound) {
    ordered_json array = ordered_json::array();
    for (const auto & pt : bound) {
      ordered_json entry = ordered_json::object();
      entry["x"] = pt.x;
      entry["y"] = pt.y;
      entry["z"] = pt.z;
      array.push_back(entry);
    }
    return array;
  };

  ordered_json document = ordered_json::object();
  document["topic"] = kOutputTopic;
  document["timestamp"] = timestamp_ns;
  ordered_json message = ordered_json::object();
  ordered_json header = ordered_json::object();
  ordered_json stamp = ordered_json::object();
  stamp["sec"] = path.header.stamp.sec;
  stamp["nanosec"] = path.header.stamp.nsec;
  header["stamp"] = stamp;
  header["frame_id"] = path.header.frame_id;
  message["header"] = header;
  ordered_json points = ordered_json::array();
  for (const auto & point : path.points) {
    ordered_json point_json = ordered_json::object();
    ordered_json point_content = ordered_json::object();
    ordered_json pose_content = ordered_json::object();
    ordered_json position = ordered_json::object();
    position["x"] = point.point.pose.position.x;
    position["y"] = point.point.pose.position.y;
    position["z"] = point.point.pose.position.z;
    ordered_json orientation = ordered_json::object();
    orientation["x"] = point.point.pose.orientation.x;
    orientation["y"] = point.point.pose.orientation.y;
    orientation["z"] = point.point.pose.orientation.z;
    orientation["w"] = point.point.pose.orientation.w;
    pose_content["position"] = position;
    pose_content["orientation"] = orientation;
    point_content["pose"] = pose_content;
    point_content["longitudinal_velocity_mps"] = point.point.longitudinal_velocity_mps;
    point_content["lateral_velocity_mps"] = point.point.lateral_velocity_mps;
    point_content["heading_rate_rps"] = point.point.heading_rate_rps;
    point_content["is_final"] = point.point.is_final;
    point_json["point"] = point_content;
    point_json["lane_ids"] = point.lane_ids;
    points.push_back(point_json);
  }
  message["points"] = points;
  message["left_bound"] = convertBoundOrdered(path.left_bound);
  message["right_bound"] = convertBoundOrdered(path.right_bound);
  document["message"] = message;

  auto validate_order = [](const ordered_json & doc) {
    auto it = doc.begin();
    if (it == doc.end() || it.key() != "topic") {
      return false;
    }
    ++it;
    if (it == doc.end() || it.key() != "timestamp") {
      return false;
    }
    ++it;
    if (it == doc.end() || it.key() != "message") {
      return false;
    }
    return true;
  };
  if (!validate_order(document)) {
    std::cerr << "[Output] JSON key order violation at ts=" << timestamp_ns << "\n";
  }

  os << document.dump() << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  namespace fs = std::filesystem;
  constexpr const char * kScenarioPrefix = "--scenario_";
  const fs::path default_map_path = fs::path("../sample-map-planning") / "lanelet2_map.osm";
  const fs::path reference_root = fs::path("../reference_IO");
  std::string scenario_flag;
  std::size_t pad_final_frames = 0;file:///home/sujin/Documents/01_GIT/OSS_BPP/reference_IO/scenario_1/planning__scenario_planning__lane_driving__behavior_planning__path_with_lane_id.jsonl
  
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind(kScenarioPrefix, 0) == 0) {
      scenario_flag = arg;
      continue;
    }
    if (arg == "--pad-final-frames") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --pad-final-frames\n";
        return 1;
      }
      pad_final_frames = static_cast<std::size_t>(std::strtoull(argv[++i], nullptr, 10));
      continue;
    }
    std::cerr << "Unknown argument: " << arg << "\n";
    return 1;
  }
  if (scenario_flag.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " --scenario_<n> [--pad-final-frames <uint>]\n";
    std::cerr << "Supported scenarios: ";
    std::cerr << "--scenario_1, --scenario_2, --scenario_3\n";
    return 1;
  }
  if (scenario_flag.rfind(kScenarioPrefix, 0) != 0) {
    std::cerr << "Expected scenario flag starting with " << kScenarioPrefix << "\n";
    return 1;
  }
  const auto scenario_name = scenario_flag.substr(std::strlen(kScenarioPrefix));
  if (scenario_name.empty()) {
    std::cerr << "Missing scenario identifier after " << kScenarioPrefix << "\n";
    return 1;
  }
  const fs::path scenario_dir = reference_root / ("scenario_" + scenario_name);
  const fs::path localization_file = scenario_dir / "localization__kinematic_state.jsonl";
  const fs::path mission_file = scenario_dir / "planning__mission_planning__route.jsonl";
  const fs::path output_path =
    scenario_dir / "planning__scenario_planning__lane_driving__behavior_planning__path_with_lane_id.jsonl";

  const fs::path map_path = default_map_path;

  if (!fs::exists(map_path)) {
    std::cerr << "[Lanelet2] Map file not found: " << map_path << "\n";
    return 1;
  }
  if (!fs::exists(scenario_dir) || !fs::is_directory(scenario_dir)) {
    std::cerr << "[Input] Scenario directory missing: " << scenario_dir << "\n";
    return 1;
  }

  const auto localization_frames = loadLocalizationFrames(localization_file);
  const auto mission_updates = loadMissionUpdates(mission_file);

  std::ofstream output_stream(output_path, std::ios::trunc);
  if (!output_stream.is_open()) {
    std::cerr << "[Output] Cannot open output file: " << output_path << "\n";
    return 1;
  }

  const fs::path diag_path = scenario_dir / "frame_diagnostics.txt";
  std::ofstream diag_stream(diag_path, std::ios::trunc);
  if (!diag_stream.is_open()) {
    std::cerr << "[Diag] Unable to write diagnostics to: " << diag_path << "\n";
  }
  const fs::path shift_diag_path = fs::current_path() / "debug_frame_shift.txt";
  std::ofstream shift_stream(shift_diag_path, std::ios::trunc);
  if (!shift_stream.is_open()) {
    std::cerr << "[ShiftDiag] Unable to write frame shift diagnostics to: " << shift_diag_path << "\n";
  }
  constexpr std::int64_t kOutputIntervalNs = 100'000'000LL;  // 10 Hz
  std::vector<std::size_t> output_frame_indices;
  output_frame_indices.reserve(localization_frames.size());
  if (!localization_frames.empty()) {
    const auto start_ts = localization_frames.front().timestamp_ns;
    const auto end_ts = localization_frames.back().timestamp_ns;
    const auto grid_start_ts = (start_ts / kOutputIntervalNs) * kOutputIntervalNs;
    const auto grid_end_ts =
      ((end_ts + kOutputIntervalNs - 1) / kOutputIntervalNs) * kOutputIntervalNs;
    std::size_t src_idx = 0;
    for (std::int64_t target_ts = grid_start_ts; target_ts <= grid_end_ts;
         target_ts += kOutputIntervalNs) {
      while (src_idx + 1 < localization_frames.size() &&
             localization_frames[src_idx + 1].timestamp_ns <= target_ts) {
        ++src_idx;
      }
      std::size_t chosen_idx = src_idx;
      if (src_idx + 1 < localization_frames.size()) {
        const auto curr_ts = localization_frames[src_idx].timestamp_ns;
        const auto next_ts = localization_frames[src_idx + 1].timestamp_ns;
        const auto curr_diff = std::llabs(curr_ts - target_ts);
        const auto next_diff = std::llabs(next_ts - target_ts);
        if (next_diff < curr_diff) {
          chosen_idx = src_idx + 1;
        }
      }
      if (output_frame_indices.empty() || output_frame_indices.back() != chosen_idx) {
        output_frame_indices.push_back(chosen_idx);
      }
    }
    for (std::size_t pad = 0; pad < pad_final_frames; ++pad) {
      output_frame_indices.push_back(output_frame_indices.back());
    }
  }
  const auto target_frame_count = output_frame_indices.size();

  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(
    lanelet::Origin({35.23808753540768, 139.9009591876285}));
  auto lanelet_map_unique = lanelet::load(map_path.string(), projector, &errors);
  if (!lanelet_map_unique) {
    std::cerr << "[Lanelet2] Failed to load map: " << map_path << "\n";
    return 1;
  }
  lanelet::LaneletMapPtr lanelet_map(lanelet_map_unique.release());
  const auto map_info = computeMapInfo(lanelet_map);
  std::cerr << "[Lanelet2] Map loaded with " << map_info.lanelet_count << " lanelets.\n";

  const auto route_handler = createRouteHandler(lanelet_map);
  auto planner_data = std::make_shared<PlannerData>();
  VehicleInfo vehicle_info{};
  BehaviorPathPlannerConfig config{};
  planner_data->init_parameters(vehicle_info, config);
  planner_data->route_handler = route_handler;
  auto ego_odometry = std::make_shared<Odometry>();
  planner_data->self_odometry = ego_odometry;

  PlannerManager planner_manager;

  MissionData active_mission{};
  bool has_active_mission = false;
  std::size_t next_mission_idx = 0;
  auto applyMissionForFrame = [&](std::size_t frame_idx) {
    if (next_mission_idx < mission_updates.size() && next_mission_idx == frame_idx) {
      active_mission = mission_updates[next_mission_idx].mission;
      const bool ok = applyMissionRoute(active_mission, route_handler, planner_data, planner_manager);
      has_active_mission = ok;
      ++next_mission_idx;
    }
  };

  LocalizationFrame last_localization{};
  bool has_localization = false;
  std::optional<std::int64_t> prev_localization_ts{};

  for (std::size_t out_idx = 0; out_idx < target_frame_count; ++out_idx) {
    const std::size_t frame_idx = output_frame_indices[out_idx];
    last_localization = localization_frames[frame_idx];
    if (prev_localization_ts && last_localization.timestamp_ns &&
        *prev_localization_ts == last_localization.timestamp_ns) {
      std::cerr << "[Sync] Localization timestamp repeated at frame " << frame_idx << '\n';
    }
    prev_localization_ts = last_localization.timestamp_ns;
    has_localization = true;

    applyMissionForFrame(frame_idx);

    if (has_localization) {
      auto odo = std::make_shared<Odometry>();
      odo->pose.pose = last_localization.state.pose;
      odo->pose.covariance.fill(0.0);
      odo->twist.linear.x = last_localization.state.twist.linear_x;
      odo->twist.linear.y = last_localization.state.twist.linear_y;
      odo->twist.linear.z = last_localization.state.twist.linear_z;
      odo->twist.angular.x = last_localization.state.twist.angular_x;
      odo->twist.angular.y = last_localization.state.twist.angular_y;
      odo->twist.angular.z = last_localization.state.twist.angular_z;
      odo->twist.covariance.fill(0.0);
      planner_data->self_odometry = std::const_pointer_cast<const Odometry>(odo);
    }
    const KinematicState * ego_state_ptr = has_localization ? &last_localization.state : nullptr;
    PathWithLaneId path{};
    const json * answer_msg = nullptr;
    BehaviorModuleOutput output{};

    if (has_active_mission) {
      output = planner_manager.run(planner_data);
      path = output.path;
      if (path.points.empty()) {
        std::cerr << "[BPP] Output path empty at frame " << frame_idx << "\n";
      } else {
        applyVelocityProfile(path, ego_state_ptr);
      }
    } else {
      std::cerr << "[BPP] No mission route before frame " << frame_idx << ", emitting empty path.\n";
    }

    if (!path.points.empty()) {
      path = autoware::behavior_path_planner::utils::resamplePathWithSpline(
        path, planner_data->parameters.output_path_interval, false);
    }

    const auto path_s = autoware::behavior_path_planner::utils::calcPathArcLengthArray(
      path, 0, path.points.size(), 0.0);
    const auto lane_segments =
      autoware::behavior_path_planner::utils::createLaneSegments(path, path_s);
    const auto reference_stats = output.reference_path_stats;
    path.header.frame_id = "map";
    TimeStamp stamp = currentTimeStamp();
    if (has_localization) {
      stamp.sec = last_localization.state.header.stamp.sec;
      stamp.nsec = last_localization.state.header.stamp.nsec;
    }
    const std::int64_t frame_timestamp_ns = stamp.sec * 1'000'000'000LL + stamp.nsec;
    path.header.stamp = stamp;
    writePathJson(path, output_stream, frame_timestamp_ns);

    if (diag_stream.is_open()) {
      if (reference_stats && out_idx >= 55 && out_idx <= 85) {
        diag_stream << std::fixed << std::setprecision(3);
        diag_stream << "  [clip/resample] start=" << reference_stats->clip_start_s
                    << ", end=" << reference_stats->clip_end_s
                    << ", length=" << reference_stats->clipped_length
                    << ", interval=" << reference_stats->output_interval
                    << ", expected=" << reference_stats->expected_point_count
                    << ", actual=" << reference_stats->actual_point_count << '\n';
        diag_stream << "    goal_s=" << reference_stats->goal_s
                    << ", goal_snapped=" << (reference_stats->goal_snapped ? "yes" : "no") << '\n';
        if (has_localization && !path.points.empty()) {
          const double ego_start_dist =
            calcPoseDistance(path.points.front().point.pose, last_localization.state.pose);
          diag_stream << "    Ego->path start distance: " << ego_start_dist << " m\n";
        }
        if (!path.points.empty()) {
          const auto & start_pose = path.points.front().point.pose;
          const auto & end_pose = path.points.back().point.pose;
          diag_stream << "    Path start: [" << start_pose.position.x << ", " << start_pose.position.y
                      << "], end: [" << end_pose.position.x << ", " << end_pose.position.y << "]\n";
        }
        if (!active_mission.lanelet_ids.empty() && !path.points.empty()) {
          const double goal_dist = calcPoseDistance(path.points.back().point.pose, active_mission.goal_pose);
          diag_stream << "    Mission goal distance: " << goal_dist << " m\n";
        }
        diag_stream << std::defaultfloat;
      }
      if (out_idx == 0 || (out_idx >= 60 && out_idx <= 80)) {
        const size_t segment_count = lane_segments.size();
        const size_t lane_id_changes = segment_count > 0 ? segment_count - 1 : 0;
        const size_t empty_lane_points =
          std::count_if(path.points.begin(), path.points.end(), [](const auto & point) {
            return point.lane_ids.empty();
          });
        diag_stream << "  [lane segments] count=" << segment_count
                    << ", changes=" << lane_id_changes
                    << ", empty lane_id points=" << empty_lane_points << '\n';
      }
      logFrameDiagnostics(
        diag_stream, out_idx,
        has_localization ? &last_localization : nullptr,
        active_mission, route_handler, path,
        answer_msg);
    }
    const bool debug_frame =
      (out_idx <= 10) || (out_idx >= 55 && out_idx <= 85);
    if (shift_stream.is_open() && debug_frame) {
      shift_stream << "==== Frame " << out_idx << " shift diagnostics ====\n";
      shift_stream << std::fixed << std::setprecision(3);
      if (has_localization) {
        const auto & pose = last_localization.state.pose;
        const double yaw = extractYawFromOrientation(pose.orientation);
        shift_stream << "[DBG][frame=" << frame_idx << "] loc_pose=(" << pose.position.x << ", "
                     << pose.position.y << ", yaw=" << yaw << ") loc_ts=" << last_localization.timestamp_ns
                     << "\n";
      } else {
        shift_stream << "[DBG][frame=" << frame_idx << "] loc_pose=<none>\n";
      }
      if (planner_data->self_odometry) {
        const auto & planner_pose = planner_data->self_odometry->pose.pose;
        const double yaw = extractYawFromOrientation(planner_pose.orientation);
        shift_stream << "[DBG][frame=" << frame_idx << "] planner_pose=(" << planner_pose.position.x << ", "
                     << planner_pose.position.y << ", yaw=" << yaw << ")\n";
      }
      const double path_length = path_s.empty() ? 0.0 : path_s.back();
      const auto & ref_path = output.reference_path;
      if (!ref_path.points.empty()) {
        const auto & ref_start = ref_path.points.front().point.pose.position;
        const auto & ref_end = ref_path.points.back().point.pose.position;
        shift_stream << "[DBG][frame=" << frame_idx << "] ref_path_first=(" << ref_start.x << ", " << ref_start.y
                     << ") ref_path_last=(" << ref_end.x << ", " << ref_end.y << ") ref_size=" << ref_path.points.size()
                     << "\n";
      }
      if (reference_stats) {
        shift_stream << "  Clip range: [" << reference_stats->clip_start_s << ", "
                     << reference_stats->clip_end_s << "], length=" << reference_stats->clipped_length << "\n";
        if (!path.points.empty() && !path_s.empty()) {
          const auto interpolate_position = [&](double target_s) {
            autoware::common_types::Position pos{};
            auto upper = std::upper_bound(path_s.begin(), path_s.end(), target_s);
            if (upper == path_s.begin()) {
              return path.points.front().point.pose.position;
            }
            if (upper == path_s.end()) {
              return path.points.back().point.pose.position;
            }
            const size_t idx1 = static_cast<size_t>(std::distance(path_s.begin(), upper));
            const size_t idx0 = idx1 - 1;
            const double s0 = path_s[idx0];
            const double s1 = path_s[idx1];
            const double t = (std::abs(s1 - s0) < 1e-6) ? 0.0 : (target_s - s0) / (s1 - s0);
            const auto & p0 = path.points[idx0].point.pose.position;
            const auto & p1 = path.points[idx1].point.pose.position;
            pos.x = p0.x + (p1.x - p0.x) * t;
            pos.y = p0.y + (p1.y - p0.y) * t;
            pos.z = p0.z + (p1.z - p0.z) * t;
            return pos;
          };
          const auto clip_start_pt = interpolate_position(reference_stats->clip_start_s);
          const auto & resample_first = path.points.front().point.pose.position;
          shift_stream << "[DBG][frame=" << frame_idx << "] clipped_first=(" << clip_start_pt.x << ", "
                       << clip_start_pt.y << ") clipped_size=" << reference_stats->actual_point_count << "\n";
          shift_stream << "[DBG][frame=" << frame_idx << "] out_first=(" << resample_first.x << ", " << resample_first.y
                       << ") out_size=" << path.points.size() << "\n";
        }
      }
      shift_stream << "[DBG][frame=" << frame_idx << "] ref_length=" << path_length << "\n";
      shift_stream << "[DBG][frame=" << frame_idx << "] Expected points: "
                   << (reference_stats ? reference_stats->expected_point_count : 0)
                   << ", actual: " << path.points.size() << "\n";
      const auto answer_point_count =
        answer_msg ? answer_msg->value("points", json::array()).size() : 0;
      const auto answerPoseAtIndex = [&](size_t idx) -> std::optional<Pose> {
        if (!answer_msg) {
          return std::nullopt;
        }
        const auto & points = answer_msg->value("points", json::array());
        if (idx >= points.size()) {
          return std::nullopt;
        }
        return parsePose(points[idx]["point"]["pose"]);
      };
      auto computeDiff = [&](size_t our_idx, size_t ans_idx)
        -> std::optional<std::pair<double, double>> {
        if (our_idx >= path.points.size()) {
          return std::nullopt;
        }
        const auto ans_pose = answerPoseAtIndex(ans_idx);
        if (!ans_pose) {
          return std::nullopt;
        }
        const auto & ours = path.points[our_idx].point.pose.position;
        return std::make_pair(ours.x - ans_pose->position.x, ours.y - ans_pose->position.y);
      };
      std::vector<std::pair<double, double>> offsets;
      const auto addOffset = [&](size_t our_idx) {
        if (answer_point_count == 0) {
          return;
        }
        const size_t ans_idx = std::min(our_idx, answer_point_count - 1);
        if (const auto diff = computeDiff(our_idx, ans_idx)) {
          offsets.push_back(*diff);
          shift_stream << "[DBG][frame=" << frame_idx << "] Offset idx=" << our_idx << " dx=" << diff->first
                       << " dy=" << diff->second << "\n";
        }
      };
      if (!path.points.empty()) {
        addOffset(0);
        addOffset(path.points.size() / 2);
        addOffset(path.points.size() - 1);
      }
      if (offsets.size() >= 2) {
        const auto & base = offsets.front();
        bool all_same = true;
        for (const auto & offset : offsets) {
          if (std::abs(base.first - offset.first) > 0.1 || std::abs(base.second - offset.second) > 0.1) {
            all_same = false;
            break;
          }
        }
        if (all_same) {
          shift_stream << "  DETECTED_GLOBAL_SHIFT dx=" << base.first << ", dy=" << base.second << "\n";
        }
      }
      shift_stream << "==== End Frame " << frame_idx << " ====\n\n";
      shift_stream << std::defaultfloat;
      shift_stream.flush();
    }
    planner_data->prev_output_path = std::make_shared<PathWithLaneId>(path);
  }

  std::cerr << "[BPP] Generated " << target_frame_count << " frames.\n";
  return 0;
}
