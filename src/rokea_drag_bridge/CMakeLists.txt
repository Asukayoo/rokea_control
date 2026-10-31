cmake_minimum_required(VERSION 3.12)
project(rokea_drag_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# xCoreSDK-CPP: default to vendored third_party under rokea_control.
set(XCORE_SDK_ROOT "" CACHE PATH "Path to xCoreSDK-CPP repository root")
if(NOT XCORE_SDK_ROOT)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/xCoreSDK-CPP/include/rokae/robot.h")
    set(XCORE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/xCoreSDK-CPP")
  endif()
endif()
if(NOT XCORE_SDK_ROOT OR NOT EXISTS "${XCORE_SDK_ROOT}/include/rokae/robot.h")
  message(FATAL_ERROR
    "xCoreSDK-CPP not found. Set -DXCORE_SDK_ROOT=/path/to/xCoreSDK-CPP "
    "(expected include/rokae/robot.h).")
endif()

set(_XCORE_INCLUDE "${XCORE_SDK_ROOT}/include")
set(_XCORE_LIB_DIR "${XCORE_SDK_ROOT}/lib/Linux/${CMAKE_SYSTEM_PROCESSOR}")
set(_XCORE_STATIC "${_XCORE_LIB_DIR}/libxCoreSDK.a")
set(_XCORE_EIGEN "${XCORE_SDK_ROOT}/external")
if(NOT EXISTS "${_XCORE_STATIC}")
  message(FATAL_ERROR "Missing ${_XCORE_STATIC}")
endif()
if(NOT EXISTS "${_XCORE_EIGEN}/Eigen/Core")
  message(FATAL_ERROR "Missing Eigen under ${_XCORE_EIGEN}")
endif()

find_package(Threads REQUIRED)

add_executable(rokea_drag_bridge_node src/rokea_drag_bridge_node.cpp)
target_include_directories(rokea_drag_bridge_node PRIVATE
  "${_XCORE_INCLUDE}"
  "${_XCORE_EIGEN}"
)
target_compile_definitions(rokea_drag_bridge_node PRIVATE EIGEN_DONT_ALIGN_STATICALLY)

ament_target_dependencies(rokea_drag_bridge_node rclcpp sensor_msgs std_msgs)
# Plain signature (must match ament_target_dependencies).
target_link_libraries(rokea_drag_bridge_node
  "${_XCORE_STATIC}"
  Threads::Threads
  ${CMAKE_DL_LIBS}
)
if(EXISTS "${_XCORE_LIB_DIR}/libxMateModel.a")
  target_link_libraries(rokea_drag_bridge_node "${_XCORE_LIB_DIR}/libxMateModel.a")
endif()

install(TARGETS rokea_drag_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch dds DESTINATION share/${PROJECT_NAME})

ament_package()
