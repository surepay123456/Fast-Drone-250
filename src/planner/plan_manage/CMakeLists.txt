# cmake_minimum_required(VERSION 2.8.3)
# project(ego_planner)

# set(CMAKE_BUILD_TYPE "Release")
# # ADD_COMPILE_OPTIONS(-std=c++11 )
# # ADD_COMPILE_OPTIONS(-std=c++14 )
# # set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# # 强制使用 C++14 标准
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

# set(CMAKE_C_STANDARD 99)
# set(CMAKE_C_STANDARD_REQUIRED TRUE)
# # acados include and lib path
# set(acados_include /home/xep/acados/include)
# set(acados_lib /home/xep/acados/lib)
# set(c_generated_code_path ${PROJECT_SOURCE_DIR}/include/tube_mpc/c_generated_code)
# # 设置链接目录 将libpath加入到链接目录中
# link_directories(${acados_lib})

# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   geometry_msgs
#   quadrotor_msgs
#   plan_env
#   path_searching
#   bspline_opt
#   traj_utils
#   message_generation
#   cv_bridge
#   quadrotor_common
# )

# # catkin_package(CATKIN_DEPENDS message_runtime)
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ego_planner
#  CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
# #  DEPENDS system_lib
# )

# include_directories(
#   include
#   # SYSTEM
#   ${catkin_INCLUDE_DIRS} 
#   ${EIGEN3_INCLUDE_DIR}
#   ${PCL_INCLUDE_DIRS}
#   include/tube_mpc
#   # acados include
#   ${acados_include}/blasfeo/include/
#   ${acados_include}/hpipm/include/
#   ${acados_include}/
#   ${acados_include}/acados/
#   ${c_generated_code_path}/
#   ${c_generated_code_path}/augmented_yaw_model_constraints/
#   ${c_generated_code_path}/augmented_yaw_model_model/
# )


# add_executable(ego_planner_node
#   src/ego_planner_node.cpp 
#   src/ego_replan_fsm.cpp
#   src/planner_manager.cpp
#   )
# target_link_libraries(ego_planner_node 
#   ${catkin_LIBRARIES}
#   )
# #add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# #  这里加入wrapper的库 给traj_sever
# add_executable(traj_server src/traj_server.cpp)
# target_link_libraries(traj_server ${catkin_LIBRARIES})

# add_executable(mpc_traj_server_node src/mpc_traj_server_node.cpp src/mpc_traj_server.cpp)
# target_link_libraries(mpc_traj_server_node ${catkin_LIBRARIES})
# #add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ########################################
# #
# # file()
# file(GLOB AUGMENTED_YAW_MODEL_MODEL_SRCS
#     ${c_generated_code_path}/augmented_yaw_model_model/*.c
# )
# file(GLOB AUGMENTED_YAW_MODEL_CONSTRAINTS_SRCS
#     ${c_generated_code_path}/augmented_yaw_model_constraints/*.c
# )
# # 模型ode源文件
# set(MODEL_SRCS 
#     ${AUGMENTED_YAW_MODEL_MODEL_SRCS}
# )
# # 求解OCP源文件
# set(OCP_SRCS
#     ${AUGMENTED_YAW_MODEL_CONSTRAINTS_SRCS}
#     ${c_generated_code_path}/acados_solver_augmented_yaw_model.c
# )

# # 将上述的生成源文件打包成一个库
# add_library(acados_ocp_solver_lib SHARED ${MODEL_SRCS} ${OCP_SRCS})
# target_link_libraries(acados_ocp_solver_lib acados hpipm blasfeo m)

# add_library(acados_wrapper_lib SHARED  src/acados_mpc_wrapper.cpp)
# target_link_libraries(acados_wrapper_lib acados_ocp_solver_lib)

# add_executable(acados_mpc_test src/main_acados_test.cpp)
# target_link_libraries(acados_mpc_test acados_wrapper_lib ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

# 强制使用 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

# acados include and lib path
set(acados_include /home/xep/acados/include)
set(acados_lib /home/xep/acados/lib)
set(c_generated_code_path ${PROJECT_SOURCE_DIR}/include/tube_mpc/c_generated_code)

# 设置链接目录 将libpath加入到链接目录中
link_directories(${acados_lib})

# 查找依赖项
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  quadrotor_common
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_planner
  CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include/tube_mpc
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/
  ${acados_include}/acados/
  ${c_generated_code_path}/
  ${c_generated_code_path}/augmented_yaw_model_constraints/
  ${c_generated_code_path}/augmented_yaw_model_model/
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
)


add_executable(mpc_traj_server_node src/mpc_traj_server_node.cpp src/mpc_traj_server.cpp)
target_link_libraries(mpc_traj_server_node ${catkin_LIBRARIES})

file(GLOB AUGMENTED_YAW_MODEL_MODEL_SRCS
    ${c_generated_code_path}/augmented_yaw_model_model/*.c
)
file(GLOB AUGMENTED_YAW_MODEL_CONSTRAINTS_SRCS
    ${c_generated_code_path}/augmented_yaw_model_constraints/*.c
)
set(MODEL_SRCS ${AUGMENTED_YAW_MODEL_MODEL_SRCS})
set(OCP_SRCS ${AUGMENTED_YAW_MODEL_CONSTRAINTS_SRCS} ${c_generated_code_path}/acados_solver_augmented_yaw_model.c)

add_library(acados_ocp_solver_lib SHARED ${MODEL_SRCS} ${OCP_SRCS})
target_link_libraries(acados_ocp_solver_lib acados hpipm blasfeo m)

# add_library(acados_wrapper_lib SHARED src/acados_mpc_wrapper.cpp)
# target_link_libraries(acados_wrapper_lib acados_ocp_solver_lib)

# add_executable(acados_mpc_test src/main_acados_test.cpp)
# target_link_libraries(acados_mpc_test acados_wrapper_lib ${catkin_LIBRARIES})

add_executable(acados_simple_test src/acados_simple_test.cpp)
target_link_libraries(acados_simple_test acados_ocp_solver_lib ${catkin_LIBRARIES})

add_library(acados_simple_wrapper SHARED src/acados_simple_wrapper.cpp)
target_link_libraries(acados_simple_wrapper acados_ocp_solver_lib)
add_executable(main_simple_wrapper_test src/main_simple_wrapper_test.cpp)
target_link_libraries(main_simple_wrapper_test acados_simple_wrapper ${catkin_LIBRARIES})


add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server acados_simple_wrapper ${catkin_LIBRARIES})

