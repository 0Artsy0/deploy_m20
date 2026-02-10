cmake_minimum_required(VERSION 3.0.2)
project(sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
  set(OpenGL_GL_PREFERENCE "GLVND")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  sensor_msgs
  geometry_msgs
)

## 查找系统依赖
find_package(OpenGL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW REQUIRED glfw3)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../library/mujoco")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")

## 动态检测可用的 MuJoCo 库文件
file(GLOB MUJOCO_LIB_FILES "${MUJOCO_DIR}/lib/libmujoco.so*")

if(MUJOCO_LIB_FILES)
  # 按文件名长度排序，优先选择版本号最全的文件
  list(SORT MUJOCO_LIB_FILES)
  list(REVERSE MUJOCO_LIB_FILES)
  
  # 选择第一个可用的库文件
  list(GET MUJOCO_LIB_FILES 0 MUJOCO_LIBRARY)
  
  message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARY}")
  message(STATUS "All available MuJoCo libraries: ${MUJOCO_LIB_FILES}")
else()
  message(FATAL_ERROR "No MuJoCo library found in ${MUJOCO_DIR}/lib")
endif()

###################################
## catkin特定配置 ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs robot_msgs sensor_msgs geometry_msgs
  DEPENDS OpenGL GLFW
)

###########
## 构建 ##
###########

## 指定头文件的额外位置
include_directories(
  include
  ${MUJOCO_INCLUDE_DIR}
  ${MUJOCO_DIR}/simulate
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

## 声明MuJoCo仿真可执行文件
add_executable(sim2sim_mujoco
  src/mujoco_sim.cpp
  include/mujoco/glfw_adapter.cc
  include/mujoco/platform_ui_adapter.cc
  include/mujoco/simulate.cc
  include/mujoco/glfw_dispatch.cc
)

## 声明Gazebo仿真可执行文件
add_executable(sim2sim_gazebo
  src/gazebo_sim.cpp
)

## 重命名可执行文件（可选，确保输出名称正确）
set_target_properties(sim2sim_mujoco PROPERTIES OUTPUT_NAME sim2sim_mujoco)
set_target_properties(sim2sim_gazebo PROPERTIES OUTPUT_NAME sim2sim_gazebo)

## 添加cmake目标依赖
add_dependencies(sim2sim_mujoco ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sim2sim_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 指定要链接的库
target_link_libraries(sim2sim_mujoco
  ${MUJOCO_LIBRARY}
  ${GLFW_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenGL_LIBRARIES}
  pthread
)

target_link_libraries(sim2sim_gazebo
  ${catkin_LIBRARIES}
)

# 设置机器人描述文件路径
set(ROBOT_DESCRIPTION_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../robot_description")
add_definitions(-DROBOT_DESCRIPTION_PATH="${ROBOT_DESCRIPTION_PATH}")