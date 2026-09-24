cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_NAME CALLRobotBFaceToFaceMover)
set(PROJECT_DESCRIPTION "Isolated face-to-face CALL trajectory mover for Kinova B")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(
  etc/CALLRobotBFaceToFaceMover.in.yaml
  "${CMAKE_CURRENT_BINARY_DIR}/etc/CALLRobotBFaceToFaceMover.yaml"
  @ONLY)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CALLRobotBFaceToFaceMover.yaml"
  DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
