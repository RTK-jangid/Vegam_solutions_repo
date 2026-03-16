cmake_minimum_required(VERSION 3.8)
project(vegam_solution_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Include the header directories so the compiler can find your .hpp files
include_directories(include)

# 3. Create the executable and include ALL .cpp files
add_executable(bot_controller
  src/main.cpp
  src/container_bot.cpp
  src/detection.cpp
)

# 4. Link the dependencies to your executable
ament_target_dependencies(bot_controller 
  rclcpp 
  geometry_msgs 
  sensor_msgs
)

# 5. Install the executable
install(TARGETS
  bot_controller
  DESTINATION lib/${PROJECT_NAME})

# 6. Install the 'config' folder so ROS can find your .yml parameters at runtime
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
