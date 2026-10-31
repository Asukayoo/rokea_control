cmake_minimum_required(VERSION 3.12)
project(ar3_right_arm_status LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指向已克隆并解压预编译库的 xCoreSDK-CPP 根目录
#   git clone https://github.com/RokaeRobot/xCoreSDK-CPP.git
#   从 Release 下载 linux 包，在 SDK 根目录解压到 lib/
set(XCORE_SDK_ROOT "" CACHE PATH "Path to xCoreSDK-CPP repository root")

if(NOT XCORE_SDK_ROOT)
  message(FATAL_ERROR
    "Set -DXCORE_SDK_ROOT=/path/to/xCoreSDK-CPP\n"
    "Clone https://github.com/RokaeRobot/xCoreSDK-CPP and extract prebuilt libs into lib/ "
    "(see that repo's lib/README.md).")
endif()

set(_XCORE_INCLUDE "${XCORE_SDK_ROOT}/include")
set(_XCORE_LIB_DIR "${XCORE_SDK_ROOT}/lib/Linux/${CMAKE_SYSTEM_PROCESSOR}")
set(_XCORE_STATIC "${_XCORE_LIB_DIR}/libxCoreSDK.a")

if(NOT EXISTS "${_XCORE_INCLUDE}/rokae/robot.h")
  message(FATAL_ERROR "Missing headers under ${_XCORE_INCLUDE}/rokae/")
endif()
if(NOT EXISTS "${_XCORE_STATIC}")
  message(FATAL_ERROR
    "Missing ${_XCORE_STATIC}\n"
    "Download the matching package from "
    "https://github.com/RokaeRobot/xCoreSDK-CPP/releases and extract into lib/")
endif()

find_package(Threads REQUIRED)

# SDK 自带 header-only Eigen（与官方 example 一致）
set(_XCORE_EIGEN "${XCORE_SDK_ROOT}/external")
if(NOT EXISTS "${_XCORE_EIGEN}/Eigen/Core")
  message(FATAL_ERROR "Missing Eigen under ${_XCORE_EIGEN}")
endif()

function(add_xcore_exe name src)
  add_executable(${name} ${src})
  target_include_directories(${name} PRIVATE
    "${_XCORE_INCLUDE}"
    "${_XCORE_EIGEN}"
  )
  target_compile_definitions(${name} PRIVATE EIGEN_DONT_ALIGN_STATICALLY)
  target_link_libraries(${name} PRIVATE
    "${_XCORE_STATIC}"
    Threads::Threads
    ${CMAKE_DL_LIBS}
  )
  if(EXISTS "${_XCORE_LIB_DIR}/libxMateModel.a")
    target_link_libraries(${name} PRIVATE "${_XCORE_LIB_DIR}/libxMateModel.a")
  endif()
endfunction()

add_xcore_exe(ar3_right_arm_status main.cpp)
add_xcore_exe(power_probe power_probe.cpp)
add_xcore_exe(move_absj_home move_absj_home.cpp)
add_xcore_exe(drag_joint_torque_100ms drag_joint_torque_100ms.cpp)
add_xcore_exe(dual_arm_drag_100ms dual_arm_drag_100ms.cpp)
add_xcore_exe(dual_arm_rt_1khz dual_arm_rt_1khz.cpp)

# 空环不依赖 SDK
add_executable(empty_1khz empty_1khz.cpp)
target_link_libraries(empty_1khz PRIVATE Threads::Threads)

install(TARGETS ar3_right_arm_status power_probe move_absj_home drag_joint_torque_100ms
                dual_arm_drag_100ms dual_arm_rt_1khz empty_1khz
        RUNTIME DESTINATION bin)
