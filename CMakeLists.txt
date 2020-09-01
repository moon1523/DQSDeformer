# Set  the minimum  required version  of cmake  for a  project.
cmake_minimum_required(VERSION 2.6)

# C++11
SET(CMAKE_CXX_FLAGS "-std=c++0x")

#----------------------------------------------------------------------------
# Find Geant4 package, activating all available UI and Vis drivers by default
# You can set WITH_GEANT4_UIVIS to OFF via the command line or ccmake/cmake-gui
# to build a batch mode only executable
#
option(WITH_GEANT4_UIVIS "Build example with Geant4 UI and Vis drivers" ON)
if(WITH_GEANT4_UIVIS)
  find_package(Geant4 REQUIRED ui_all vis_all)
else()
  find_package(Geant4 REQUIRED)
endif()

# locate source files
include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/Dual-Quaternion-Skinning-Sample-Codes)
file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.hh)
file(GLOB dqs ${PROJECT_SOURCE_DIR}/Dual-Quaternion-Skinning-Sample-Codes/*)

# Add an executable to the project using the specified source files.

add_executable(DQSdeformer DQSdeformer.cc ${sources} ${headers} ${dqs})
target_link_libraries(DQSdeformer ${Geant4_LIBRARIES})
