add_library(rko_slam.align_sessions STATIC)
add_library(rko_slam::align_sessions ALIAS rko_slam.align_sessions)

target_sources(
  rko_slam.align_sessions
  PRIVATE align_sessions.cpp
  PUBLIC FILE_SET
         HEADERS
         FILES
         align_sessions.hpp
         BASE_DIRS
         ../..)

target_link_libraries(
  rko_slam.align_sessions
  PUBLIC rko_slam::core rko_slam::closures
  PRIVATE yaml-cpp::yaml-cpp)
set_target_properties(rko_slam.align_sessions PROPERTIES POSITION_INDEPENDENT_CODE ON)
