################################################################################
# user build settings

SET(TRACE FALSE)
SET(HAVE_G2O TRUE) 
SET(USE_ROS FALSE) # Set FALSE if you want to build this package without Catkin
SET(DEBUG_OUTPUT FALSE) # Only relevant if build without ROS

################################################################################

SET(PROJECT_NAME vilo)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

SET(CMAKE_VERBOSE_MAKEFILE OFF)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Set definitions
IF(TRACE)
  ADD_DEFINITIONS(-DVILO_TRACE)
ENDIF()
IF(HAVE_G2O)
  ADD_DEFINITIONS(-DUSE_BUNDLE_ADJUSTMENT)
ENDIF()
IF(USE_ROS)
  ADD_DEFINITIONS(-DVILO_USE_ROS)
ENDIF()
IF(DEBUG_OUTPUT)
  ADD_DEFINITIONS(-DVILO_DEBUG_OUTPUT)
ENDIF()

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
# Ubuntu's prebuilt PCL uses the 16-byte SSE Eigen ABI. AVX/-march=native
# changes aligned_allocator's malloc/free convention across the library boundary.
ADD_DEFINITIONS(-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)
IF(DEFINED ENV{ARM_ARCHITECTURE})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
ENDIF()
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Add plain cmake packages 
# Respect -DOpenCV_DIR and normal CMake discovery. The hint preserves the
# original workstation installation without forcing that path on new clones.
FIND_PACKAGE(OpenCV 3 REQUIRED CONFIG
  HINTS "/usr/local/opencv3/usr/local/share/OpenCV")
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
FIND_PACKAGE(Pangolin REQUIRED)


# Include dirs
INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/thirdparty/g2o            # g2o
  ${PROJECT_SOURCE_DIR}/thirdparty/fast/include   # fast
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus         # sophus
  ${PROJECT_SOURCE_DIR}/include			  # vilo
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  /usr/include/suitesparse # for cholmod
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Pangolin_LIBRARIES}
)



# Set sourcefiles
LIST(APPEND SOURCEFILES
  ${PROJECT_SOURCE_DIR}/src/frame_handler_mono.cpp
  ${PROJECT_SOURCE_DIR}/src/frame_handler_base.cpp
  ${PROJECT_SOURCE_DIR}/src/frame.cpp
  ${PROJECT_SOURCE_DIR}/src/point.cpp
  ${PROJECT_SOURCE_DIR}/src/map.cpp
  ${PROJECT_SOURCE_DIR}/src/pose_optimizer.cpp
  ${PROJECT_SOURCE_DIR}/src/initialization.cpp
  ${PROJECT_SOURCE_DIR}/src/matcher.cpp
  ${PROJECT_SOURCE_DIR}/src/reprojector.cpp
  ${PROJECT_SOURCE_DIR}/src/feature_alignment.cpp
  ${PROJECT_SOURCE_DIR}/src/feature_detection.cpp
  ${PROJECT_SOURCE_DIR}/src/depth_filter.cpp
  ${PROJECT_SOURCE_DIR}/src/config.cpp
  ${PROJECT_SOURCE_DIR}/src/viewer.cpp
  ${PROJECT_SOURCE_DIR}/src/photomatric_calibration.cpp
  ${PROJECT_SOURCE_DIR}/src/coarse_tracker.cpp
  ${PROJECT_SOURCE_DIR}/src/camera.cpp
  ${PROJECT_SOURCE_DIR}/src/file_reader.cpp
  ${PROJECT_SOURCE_DIR}/src/ba_types.cpp
  ${PROJECT_SOURCE_DIR}/src/bundle_adjustment.cpp
  ${PROJECT_SOURCE_DIR}/src/imu.cpp
  ${PROJECT_SOURCE_DIR}/src/lidar.cpp

  ${PROJECT_SOURCE_DIR}/src/vikit/robust_cost.cpp
  ${PROJECT_SOURCE_DIR}/src/vikit/math_utils.cpp
  ${PROJECT_SOURCE_DIR}/src/vikit/vision.cpp
  ${PROJECT_SOURCE_DIR}/src/vikit/homography.cpp
  ${PROJECT_SOURCE_DIR}/src/vikit/performance_monitor.cpp
  ${PROJECT_SOURCE_DIR}/src/vikit/so3_functions.cpp)

#set(VILO_LIBS ${PROJECT_SOURCE_DIR}/build/lib/libvilo.so)
set(G2O_LIBS ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so)
set(FAST_LIBS ${PROJECT_SOURCE_DIR}/thirdparty/fast/build/libfast.so)
set(SOPHUS_LIBS ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/build/libSophus.so)


# Create vilo library
ADD_LIBRARY(vilo SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(vilo 
${LINK_LIBS}
${G2O_LIBS} cholmod cxsparse
${FAST_LIBS}
${SOPHUS_LIBS}
)

################################################################################
# TESTS
ADD_EXECUTABLE(test_vilo example/test_vilo.cpp)
TARGET_LINK_LIBRARIES(test_vilo vilo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    sensor_msgs
    geometry_msgs
    nav_msgs
    rosgraph_msgs
    tf2_ros
    tf2_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package(CATKIN_DEPENDS roscpp rosbag sensor_msgs geometry_msgs nav_msgs rosgraph_msgs tf2_ros tf2_msgs)

add_executable(
    test_vilo_bag
    example/test_vilo_bag.cpp
)

target_link_libraries(
    test_vilo_bag
    vilo
    ${catkin_LIBRARIES}
)

enable_testing()
add_executable(map_consistency_test tests/map_consistency_test.cpp)
target_link_libraries(map_consistency_test vilo ${catkin_LIBRARIES})
add_test(NAME map_consistency COMMAND map_consistency_test)
add_executable(lidar_memory_test tests/lidar_memory_test.cpp)
target_link_libraries(lidar_memory_test vilo ${catkin_LIBRARIES})
# Keep the original standalone bin/ commands and catkin's lib/vilo commands in
# sync, so rosrun cannot accidentally select an older pre-ROS bag executable.
foreach(entry test_vilo_bag lidar_memory_test)
  add_custom_command(TARGET ${entry} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:${entry}> ${PROJECT_SOURCE_DIR}/bin/${entry})
endforeach()

install(TARGETS vilo test_vilo test_vilo_bag
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch rviz example DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.rviz" PATTERN "*.yaml")
