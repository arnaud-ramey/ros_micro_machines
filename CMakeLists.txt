### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(ros_micro_machines)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
SET(DEPS  roscpp  roslib  std_msgs)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
find_package( OpenCV REQUIRED )
# nothing to do in this package
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# nothing to do in this package
### 5 - Invoke message/service/action generation (generate_messages())
# nothing to do in this package
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS   include
  CATKIN_DEPENDS ${DEPS}
  DEPENDS
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
SET(PKG_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_micro_machines)
include_directories(include  ${catkin_INCLUDE_DIRS})
add_executable       (game.exe src/game.cpp)
target_link_libraries(game.exe  ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable       (wheel.exe src/wheel.cpp)
target_link_libraries(wheel.exe  ${catkin_LIBRARIES} ${OpenCV_LIBS})
### 8 - Tests to build (catkin_add_gtest())
# nothing to do in this package
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
