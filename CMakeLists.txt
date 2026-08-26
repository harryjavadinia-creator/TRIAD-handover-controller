cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_NAME HandoverInterceptionController)
set(PROJECT_DESCRIPTION "HandoverInterceptionController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")

# The handover-object URDF description is bundled with this controller.
# Install it under the controller's own namespaced runtime directory so the
# path referenced from the generated YAML always points at an installed
# location, not at this source checkout.
set(CALL_OBJECT_DESCRIPTION_INSTALL_PREFIX "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${PROJECT_NAME}/call_object_description")
install(FILES call_object_description/README.md DESTINATION "${CALL_OBJECT_DESCRIPTION_INSTALL_PREFIX}")
install(FILES call_object_description/urdf/call_object.urdf DESTINATION "${CALL_OBJECT_DESCRIPTION_INSTALL_PREFIX}/urdf")

configure_file(etc/HandoverInterceptionController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HandoverInterceptionController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HandoverInterceptionController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
