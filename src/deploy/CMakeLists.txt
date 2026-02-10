cmake_minimum_required(VERSION 3.0.2)
project(deploy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs
  DEPENDS libtorch onnxruntime
)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../library/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

set(ONNXRUNTIME_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../library/onnxruntime)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_c_api.h
  PATHS ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH
)
# 动态检测可用的 ONNX Runtime 库文件
file(GLOB ONNXRUNTIME_LIB_FILES "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so*")

if(ONNXRUNTIME_LIB_FILES)
  # 按文件名长度排序，优先选择版本号最全的文件（如 libonnxruntime.so.1.14）
  list(SORT ONNXRUNTIME_LIB_FILES)
  list(REVERSE ONNXRUNTIME_LIB_FILES)
  
  # 选择第一个可用的库文件
  set(ONNXRUNTIME_LIBRARY ${ONNXRUNTIME_LIB_FILES})
  list(GET ONNXRUNTIME_LIB_FILES 0 ONNXRUNTIME_LIBRARY)
  
  message(STATUS "Found ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")
  message(STATUS "All available ONNX Runtime libraries: ${ONNXRUNTIME_LIB_FILES}")
else()
  message(FATAL_ERROR "No ONNX Runtime library found in ${ONNXRUNTIME_ROOT}/lib")
endif()

include_directories(
  include
  ${TORCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# 设置模型文件路径
set(POLICY_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../policy/")
add_definitions(-DPOLICY_MODEL_PATH="${POLICY_MODEL_PATH}")

add_executable(rl_deploy src/deploy.cpp)

target_link_libraries(rl_deploy
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ONNXRUNTIME_LIBRARY}
  Threads::Threads
)

add_dependencies(rl_deploy ${catkin_EXPORTED_TARGETS})

set_target_properties(rl_deploy PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)