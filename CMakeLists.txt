cmake_minimum_required(VERSION 3.9.0)

cmake_policy(SET CMP0048 NEW)

project(Azure-Kinect-Samples LANGUAGES C CXX
    VERSION 1.4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4abt REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
set(VTK_DIR "/home/hurel/codes/openCV/VTK/build")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

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

# These specific settings tell the loader to search the directory of the
# executable for shared objects. This is done on Linux to emulate the default
# behavior of the Windows loader, which searches for DLLs in the path of the
# executable.
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
    set(CMAKE_BUILD_RPATH "\$ORIGIN")
endif()

# If using clang or GCC, set default visibilty to hidden
if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_CXX_VISIBILITY_PRESET hidden)
    set(CMAKE_C_VISIBILITY_PRESET hidden)
    set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
endif()

# If using clang or GCC only linked shared libraries if needed
if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed,-rpath-link=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed,-rpath-link=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
endif()

add_subdirectory(extern)
add_subdirectory(helper)

include_directories(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/Dual-Quaternion-Skinning-Sample-Codes
    /home/hurel/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes)
file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.hh)
file(GLOB dqs ${PROJECT_SOURCE_DIR}/Dual-Quaternion-Skinning-Sample-Codes/*)

add_executable(MotionCapt MotionCapt.cc ${sources} ${headers} ${dqs})

# Dependencies of this library
target_link_libraries(MotionCapt PRIVATE
    k4a
    k4arecord
    k4abt
    window_controller_3d::window_controller_3d
    glfw::glfw
    ${Geant4_LIBRARIES}
    ${OpenCV_LIBS} 
    ${VTK_LIBRARIES}
    )



