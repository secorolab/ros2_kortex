cmake_minimum_required(VERSION 3.14)
include(FetchContent)
FetchContent_Declare(
  kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip
  URL_HASH MD5=64bd86e7ab8bda90ef1fc7d6a356e080
)
FetchContent_MakeAvailable(kinova_binary_api)

project(kortex_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# this is needed by the kortex_api to tell it we are compiling for linux
add_definitions(-D_OS_UNIX=1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kortex_api REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kinova_mediator REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/build)

include_directories(
    ${orocos_kdl_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/lib/release
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include/client
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include/client_stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include/google
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/include/messages
)


#Current: only support Linux x86_64
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(API_URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip)
else()
  # TODO(future) to support ARM or other builds logic could go here to fetch the precompiled libKortexApiCpp.a
  # see notes in kortex_api CMakeList.txt
  message(WARNING "Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
  message(FATAL_ERROR "Unsupported System: currently support is for Linux x68_64.")
endif()

# CMake does not allow IMPORTED libraries to be installed
# The package kortex_api will download and setup the include directories
add_library(KortexApiCpp STATIC IMPORTED)
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION ${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  INTERFACE_LINK_LIBRARIES KortexApiCpp
)
target_link_libraries(KortexApiCpp INTERFACE pthread)
add_dependencies(KortexApiCpp kortex_api)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/kortex_math_util.cpp
)
target_link_libraries(${PROJECT_NAME} KortexApiCpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
target_link_libraries(
    ${PROJECT_NAME}
    ${kdl_parser_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${urdf_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/kinova_api/lib/release/libKortexApiCpp.a
    ${CMAKE_CURRENT_SOURCE_DIR}/../../kinova_mediator/build/libkinova_mediator.so
    )
# kortex_api is the headers for the Kortex API
ament_target_dependencies(
  ${PROJECT_NAME}
  SYSTEM kortex_api
  hardware_interface
  kdl_parser
  orocos_kdl
  urdf
  ament_index_cpp
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  kortex_api
  pluginlib
  rclcpp
)
ament_package()
