#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lanelet2_core/LaneletMap.h>

namespace autoware::common_types
{
// --------------------------------------------------------------------------
// 1) Time / Header
// --------------------------------------------------------------------------
struct TimeStamp
{
  std::int64_t sec{0};
  std::int64_t nsec{0};
};

struct Duration
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

struct Header
{
  std::uint32_t seq{0};
  TimeStamp stamp{};
  std::string frame_id;
};

// --------------------------------------------------------------------------
// 2) 기본 공간/자세 타입
// --------------------------------------------------------------------------
struct UUID
{
  std::uint8_t bytes[16]{};
};

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Position
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Autoware에서 Quaternion 과 Orientation 을 사실상 동일하게 쓰므로 alias 처리
using Quaternion = Orientation;

struct Pose
{
  Position position;
  Orientation orientation;
};

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// --------------------------------------------------------------------------
// 3) Lanelet / Route 타입
// --------------------------------------------------------------------------

// lane 하나의 고유 ID, type(lane) 만 담는 구조
struct LaneletPrimitive
{
  std::int64_t id{};          // lanelet::Id
  std::string primitive_type; // 보통 "lane" 하나만 써도 충분
};

// LaneletSegment: "차선 변경 가능한 구간 하나" = 여러 개의 lanelet 묶음
struct LaneletSegment
{
  LaneletPrimitive preferred_primitive;          // 실제 주행 lane 예) preferred = lane 10
  std::vector<LaneletPrimitive> primitives;      // 예) primitives = {10,11} (여차하면 11로 lane change 가능)
};

// 경로 전체
struct LaneletRoute
{
  Header header;
  Pose start_pose;
  Pose goal_pose;
  std::vector<LaneletSegment> segments;
  UUID uuid{};                  // Request-Response 매칭용 ID
  bool allow_modification{true};
};

// --------------------------------------------------------------------------
// 4) 벡터/속도 관련
// --------------------------------------------------------------------------
struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct Polygon
{
  std::vector<Point> point;   // Autoware 옛 버전 메시지 스타일
};

struct Shape
{
  enum Type { BOUNDING_BOX, CYLINDER, POLYGON } type{BOUNDING_BOX};
  Polygon footprint;
  Vector3 dimensions;
};

// --------------------------------------------------------------------------
// 5) Path 계열 타입
// --------------------------------------------------------------------------
struct PathPoint
{
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double heading_rate_rps{0.0};
  bool is_final{false};
};

struct PathPointWithLaneId
{
  PathPoint point;
  std::vector<std::int64_t> lane_ids;
};

struct PathWithLaneId
{
  using SharedPtr = std::shared_ptr<PathWithLaneId>;

  Header header;
  std::vector<PathPointWithLaneId> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

struct Path
{
  Header header;
  std::vector<PathPoint> points;
  std::vector<PointXYZ> left_bound;
  std::vector<PointXYZ> right_bound;
};

struct TrajectoryPoint
{
  Duration time_from_start{};
  Pose pose;
  double longitudinal_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double acceleration_mps2{0.0};
  double heading_rate_rps{0.0};
  double front_wheel_angle_rad{0.0};
  double rear_wheel_angle_rad{0.0};
};

struct Trajectory
{
  Header header;
  std::vector<TrajectoryPoint> points;
};

// --------------------------------------------------------------------------
// 6) Covariance 포함 타입
// --------------------------------------------------------------------------
struct PoseWithCovariance
{
  Pose pose;
  std::array<double, 36> covariance{}; // 6x6 matrix
};

struct TwistWithCovariance
{
  Vector3 linear;                 // m/s
  Vector3 angular;                // rad/s
  std::array<double, 36> covariance{}; // 6x6 matrix
};

// --------------------------------------------------------------------------
// 7) Odometry / Accel 계열
// --------------------------------------------------------------------------
struct Odometry
{
  using ConstSharedPtr = std::shared_ptr<const Odometry>;

  Header header;
  std::string child_frame_id;
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};

struct KinematicState
{
  Header header;
  Pose pose;

  struct TwistData
  {
    double linear_x{0.0};
    double linear_y{0.0};
    double linear_z{0.0};
    double angular_x{0.0};
    double angular_y{0.0};
    double angular_z{0.0};
  } twist;
};

struct Accel
{
  Vector3 linear;   // m/s²
  Vector3 angular;  // rad/s²
};

struct AccelWithCovariance
{
  Accel accel;
  std::array<double, 36> covariance{}; // 6x6 matrix
};

struct AccelWithCovarianceStamped
{
  using ConstSharedPtr = std::shared_ptr<const AccelWithCovarianceStamped>;

  Header header;
  AccelWithCovariance accel;
};

// --------------------------------------------------------------------------
// 8) Objects / Prediction
// --------------------------------------------------------------------------
struct PredictedObjectKinematics
{
  PoseWithCovariance initial_pose_with_covariance;
  TwistWithCovariance initial_twist_with_covariance;
  AccelWithCovariance initial_accel_with_covariance;
  std::vector<Pose> predicted_paths;  // 미래 예측 궤적
};

struct ObjectClassification
{
  // 클래스 라벨 상수
  static constexpr std::uint8_t UNKNOWN         = 0;
  static constexpr std::uint8_t CAR             = 1;
  static constexpr std::uint8_t TRUCK           = 2;
  static constexpr std::uint8_t BUS             = 3;
  static constexpr std::uint8_t TRAILER         = 4;
  static constexpr std::uint8_t MOTORCYCLE      = 5;
  static constexpr std::uint8_t BICYCLE         = 6;
  static constexpr std::uint8_t PEDESTRIAN      = 7;
  static constexpr std::uint8_t ANIMAL          = 8;
  static constexpr std::uint8_t HAZARD          = 9;  // 위험 객체
  static constexpr std::uint8_t OVER_DRIVABLE   = 10; // 밟고 지나갈 수 있는 객체
  static constexpr std::uint8_t UNDER_DRIVABLE  = 11; // 아래로 통과 가능(고가도로 등)

  std::uint8_t label{UNKNOWN};
  float probability{1.0F};
};

struct PredictedObject
{
  UUID object_id{};
  float existence_probability{1.0F};
  ObjectClassification classification;
  PredictedObjectKinematics kinematics;
  Shape shape;
};

struct PredictedObjects
{
  using ConstSharedPtr = std::shared_ptr<const PredictedObjects>;

  Header header{};
  std::vector<PredictedObject> objects;
};

struct PredictedPath
{
  std::vector<Pose> path;   // 미래 예측 궤적
  float confidence{1.0F};
};

// --------------------------------------------------------------------------
// 9) OperationMode / VelocityLimit
// --------------------------------------------------------------------------
struct OperationModeState
{
  using ConstSharedPtr = std::shared_ptr<const OperationModeState>;

  enum class Mode : std::uint8_t
  {
    UNKNOWN = 0,
    MANUAL,
    AUTONOMOUS,
  };

  TimeStamp stamp{};
  Mode mode{Mode::UNKNOWN};
};

struct VelocityLimit
{
  using ConstSharedPtr = std::shared_ptr<const VelocityLimit>;

  TimeStamp stamp{};
  double max_velocity{0.0};
};

// --------------------------------------------------------------------------
// 10) PoseStamped / PoseWithUuid
// --------------------------------------------------------------------------
struct PoseWithUuidStamped
{
  TimeStamp stamp{};
  Pose pose{};
  UUID uuid{};
};

struct PoseStamped
{
  Header header;
  Pose pose;
};

// --------------------------------------------------------------------------
// 11) Drivable Area / BehaviorModuleOutput (단순화 버전)
// --------------------------------------------------------------------------
struct DrivableLanes
{
  lanelet::ConstLanelet right_lane;
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelets middle_lanes;
};

// 테스트 시나리오에는 필요 없는 필드는 모두 제거한 단순 버전
struct DrivableAreaInfo
{
  std::vector<DrivableLanes> drivable_lanes{};
};

struct BehaviorModuleOutput
{
  BehaviorModuleOutput() = default;

  // path planned by module
  PathWithLaneId path{};

  // reference path planned by module
  PathWithLaneId reference_path{};

  struct ReferencePathDiagnostics
  {
    double clip_start_s{0.0};
    double clip_end_s{0.0};
    double clipped_length{0.0};
    std::size_t expected_point_count{0};
    std::size_t actual_point_count{0};
    double goal_s{0.0};
    bool goal_snapped{false};
    double output_interval{0.0};
  };

  std::optional<ReferencePathDiagnostics> reference_path_stats{};

  std::optional<PoseWithUuidStamped> modified_goal{};

  // drivable area info to create drivable area
  DrivableAreaInfo drivable_area_info;
};

// --------------------------------------------------------------------------
// 12) Utility 타입
// --------------------------------------------------------------------------
struct PoseWithDetail
{
  Pose pose;
  std::string detail;
  explicit PoseWithDetail(const Pose & p, const std::string & d = "") : pose(p), detail(d) {}
};

using PoseWithDetailOpt = std::optional<PoseWithDetail>;

}  // namespace autoware::common_types
