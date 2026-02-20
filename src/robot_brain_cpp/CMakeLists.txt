# CMake = the engine  
# CMakeLists.txt  = the blueprint
# CMake reads the blueprint and produces the actual build instructions.

# CMake is a build-system generator.A meta-build tool that writes the actual build files the compiler understands.

#What is CMakeLists.txt ?
# instead of writing platform‑specific build scripts, we write one portable CMakeLists.txt, 
#and CMake produces the correct build instructions for each environment.
#This is why ROS2 uses it everywhere — it guarantees portability and reproducibility across all robots, laptops, and CI systems.

#When writing C++ code (like brain_node.cpp), the computer cannot run it directly because it’s just a text file. 
#It needs to be converted into a "binary" or "executable", a program the computer can actually run.

#ROS2 uses a tool called Colcon to build the C++ code, but Colcon doesn't know what to do without instructions.
# It looks at the CMakeLists.txt to find out:
#What is the project name? (e.g., robot_brain_cpp)
#What libraries are needed? (e.g., "the ROS2 library rclcpp and the message library sensor_msgs to understand the IMU data.")
#Which is the file to be turned into a program? (e.g., "src/brain_node.cpp to be turned into a program called brain_node.")
#Where should the finished program be put? (e.g., "Install it in the install folder so we can run it later.")

cmake_minimum_required(VERSION 3.8)
project(robot_brain_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")     #If the compiler is Clang
  add_compile_options(-Wall -Wextra -Wpedantic)                             #then enable strict warnings
endif()

# find_dependencies : CMake searches: our workspace, our ROS installation,System paths.
# This is why sourcing our workspace (source install/setup.bash) is essential — it updates the search path

# find dependencies
find_package(ament_cmake REQUIRED)  
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the executable
add_executable(brain_node src/brain_node.cpp)

# Link the libraries
ament_target_dependencies(brain_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install the executable so 'ros2 run' can find it
install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()  #exports our dependencies, generates packages metadata, MAKES PACKAGE DISCOVERABLE by ROS2 tools for find_package, integrates with colcon

# CMake ensures:
# same flags,same dependencies, same build steps, same install paths.
# This makes builds reproducible across: laptops, CI, Docker, robots

#------------------------------------

# ROS2 is built on top of:
# C++
# ament (ROS2’s build system)
# colcon (workspace orchestrator)

# All of these expect the package to declare:

# what it builds
# what it depends on
# how it should be installed
# how it integrates with the ROS2 ecosystem

# CMake is the glue that makes this possible.
#---------------------------
# In ROS2, CMake does four critical jobs:
# 1️ Find dependencies
# find_package(rclcpp REQUIRED)  
# This tells ROS2: “I need rclcpp. Fail if it’s missing.”

# 2️ Define build targets
# add_executable(brain_node src/brain_node.cpp)  
# This declares the binary you want to build.

# 3️ Link libraries
# ament_target_dependencies(brain_node rclcpp std_msgs sensor_msgs)  
# This ensures your node links against the correct ROS2 libraries.

# 4️ Install the node
# So that ros2 run robot_brain_cpp brain_node works.