cmake_minimum_required(VERSION 3.9.0)

project(DQS_deformer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

#openMP
FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    set (LIBIGL_INCLUDE_DIR "/home/sungho/code/BBW/libigl/include")
endif()

#k4a
FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4abt REQUIRED)

add_subdirectory(extern)
add_subdirectory(helper)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_TETGEN       "Use TetGen"           ON)
find_package(LIBIGL REQUIRED QUIET)

#file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.h)
include_directories(${PROJECT_SOURCE_DIR}/include
		/home/sungho/code/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes)
add_executable(MotionCapt MotionCapt.cc ${sources} ${headers})# ${dqs})

# Dependencies of this library
target_link_libraries(MotionCapt PRIVATE
    k4a
    k4arecord
    k4abt
    window_controller_3d::window_controller_3d
    igl::core
    igl::opengl_glfw
    igl::tetgen
    )



