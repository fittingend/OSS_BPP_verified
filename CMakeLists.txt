cmake_minimum_required(VERSION 3.14)
project(OSS_Planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

# ------------------------------------------------------------
# 0. 공통 타입(common_types) 인터페이스 라이브러리
# ------------------------------------------------------------
add_library(common_types INTERFACE)
target_include_directories(common_types INTERFACE
    ${CMAKE_SOURCE_DIR}/include
)

# ------------------------------------------------------------
# 1. 외부 라이브러리: Rosless-Lanelet2
#    lanelet2_core, lanelet2_io, lanelet2_routing, ...
# ------------------------------------------------------------
add_subdirectory(libs/Rosless-Lanelet2)


add_executable(bpp_runner
  src/main.cpp
  src/planner_manager.cpp
  src/path_utils.cpp
  src/utils.cpp
  src/static_drivable_area.cpp
  src/route_handler.cpp
  src/bpp_path_utils.cpp
  src/resample.cpp
  src/trajectory.cpp
)

target_include_directories(bpp_runner PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(bpp_runner PRIVATE
  common_types
  Boost::system
  Boost::filesystem
  Eigen3::Eigen
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  nlohmann_json::nlohmann_json
)
