cmake_minimum_required(VERSION 3.5)
project(limo_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/inverse_kinematics.py
  scripts/forward_kinematics.py
  scripts/pure_pursuit.py
  scripts/stanley.py
  scripts/mpc.py
  scripts/pid.py
  scripts/gps_emulator.py
  scripts/ekf.py
  scripts/mpc_evaluator.py
  # scripts/parameter_sweep.py
  # config/mpc_sweep_config.yaml
  scripts/comparative_analyzer.py
  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  config
  launch
  path
  DESTINATION share/${PROJECT_NAME}
  # FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  )
# Install Python packages to the same location as executables
# install(DIRECTORY
#   launch
#   path
#   scripts/PathPlanning
#   scripts/utils
#   DESTINATION lib/${PROJECT_NAME}  # Same as executables
#   USE_SOURCE_PERMISSIONS
#   FILES_MATCHING
#   PATTERN "*.py"
#   PATTERN "__pycache__" EXCLUDE
# )

ament_package()
