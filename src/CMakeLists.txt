#Start WOLF build
MESSAGE("Starting WOLF CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#CMAKE modules

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
MESSAGE(STATUS ${CMAKE_MODULE_PATH})

# Some wolf compilation options

IF((CMAKE_BUILD_TYPE MATCHES DEBUG) OR (CMAKE_BUILD_TYPE MATCHES debug) OR (CMAKE_BUILD_TYPE MATCHES Debug))
  set(_WOLF_DEBUG true)
ENDIF()

option(_WOLF_TRACE "Enable wolf tracing macro" ON)

option(BUILD_EXAMPLES "Build examples" OFF)
set(BUILD_TESTS true)

# Does this has any other interest
# but for the examples ?
# yes, for the tests !
IF(BUILD_EXAMPLES OR BUILD_TESTS)
  set(_WOLF_ROOT_DIR ${CMAKE_SOURCE_DIR})
ENDIF(BUILD_EXAMPLES OR BUILD_TESTS)

#find dependencies.

FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)

FIND_PACKAGE(Threads REQUIRED)

FIND_PACKAGE(Ceres QUIET) #Ceres is not required
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: Ceres related sources will be built.")
ENDIF(Ceres_FOUND)

FIND_PACKAGE(faramotics QUIET) #faramotics is not required
IF(faramotics_FOUND)
	FIND_PACKAGE(GLUT REQUIRED)
	FIND_PACKAGE(pose_state_time REQUIRED)
    MESSAGE("Faramotics Library FOUND: Faramotics related sources will be built.")
ENDIF(faramotics_FOUND)

FIND_PACKAGE(laser_scan_utils QUIET) #laser_scan_utils is not required
IF(laser_scan_utils_FOUND)
    MESSAGE("laser_scan_utils Library FOUND: laser_scan_utils related sources will be built.")
ENDIF(laser_scan_utils_FOUND)

FIND_PACKAGE(raw_gps_utils QUIET) #raw_gps_utils is not required
IF(raw_gps_utils_FOUND)
    MESSAGE("raw_gps_utils Library FOUND: raw_gps_utils related sources will be built.")
ENDIF(raw_gps_utils_FOUND)

# Vision Utils
FIND_PACKAGE(vision_utils QUIET)
IF (vision_utils_FOUND)
	MESSAGE("vision_utils Library FOUND: vision related sources will be built.")
	SET(PRINT_INFO_VU false)
	FIND_PACKAGE(OpenCV QUIET)
ENDIF(vision_utils_FOUND)

# OpenCV
FIND_PACKAGE(OpenCV QUIET)
IF (OPENCV_FOUND)
	MESSAGE("opencv Library FOUND: opencv related sources will be built.")
ENDIF(OPENCV_FOUND)

# Cereal
FIND_PACKAGE(cereal QUIET)
IF(cereal_FOUND)
    MESSAGE("cereal Library FOUND: cereal related sources will be built.")
ENDIF(cereal_FOUND)

# Apriltag
# TODO: write proper files to be able to use find_package with apriltag library
FIND_PATH(APRILTAG_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
FIND_LIBRARY(APRILTAG_LIBRARY NAMES apriltag PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}" "apriltag" ${APRILTAG_LIBRARY_PATH})

IF(APRILTAG_LIBRARY)
    SET(Apriltag_FOUND TRUE)
    MESSAGE("apriltag Library FOUND in ${APRILTAG_LIBRARY}: apriltag related sources will be built.")
ENDIF(APRILTAG_LIBRARY)

# YAML with yaml-cpp
INCLUDE (${PROJECT_SOURCE_DIR}/cmake_modules/FindYamlCpp.cmake)
IF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library FOUND: yaml-cpp related sources will be built.")
ELSEIF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library NOT FOUND!")
ENDIF(YAMLCPP_FOUND)

#GLOG
INCLUDE (${PROJECT_SOURCE_DIR}/cmake_modules/FindGlog.cmake)
IF(GLOG_FOUND)
    MESSAGE("glog Library FOUND: glog related sources will be built.")
    MESSAGE(STATUS ${GLOG_INCLUDE_DIR})
    MESSAGE(STATUS ${GLOG_LIBRARY})
ELSEIF(GLOG_FOUND)
    MESSAGE("glog Library NOT FOUND!")
ENDIF(GLOG_FOUND)

# SuiteSparse doesn't have find*.cmake:
FIND_PATH(
    Suitesparse_INCLUDE_DIRS
    NAMES SuiteSparse_config.h
    PATHS /usr/include/suitesparse /usr/local/include/suitesparse)
MESSAGE("Found suitesparse_INCLUDE_DIRS:" ${Suitesparse_INCLUDE_DIRS})

IF(Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND TRUE)
   MESSAGE("Suitesparse FOUND: wolf_solver will be built.")
ELSE (Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND FALSE)
   MESSAGE(FATAL_ERROR "Suitesparse NOT FOUND")
ENDIF (Suitesparse_INCLUDE_DIRS)

# Define the directory where will be the configured config.h
SET(WOLF_CONFIG_DIR ${PROJECT_BINARY_DIR}/conf/internal)

# Create the specified output directory if it does not exist.
IF(NOT EXISTS "${WOLF_CONFIG_DIR}")
  message(STATUS "Creating config output directory: ${WOLF_CONFIG_DIR}")
  file(MAKE_DIRECTORY "${WOLF_CONFIG_DIR}")
ENDIF()
IF(EXISTS "${WOLF_CONFIG_DIR}" AND NOT IS_DIRECTORY "${WOLF_CONFIG_DIR}")
  message(FATAL_ERROR "Bug: Specified CONFIG_DIR: "
    "${WOLF_CONFIG_DIR} exists, but is not a directory.")
ENDIF()
# Configure config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/internal/config.h.in "${WOLF_CONFIG_DIR}/config.h")

#TEMPORAL INCLUDE UNTIL WE FIGURE OUT WHAT TO DO WITH FILES in src/temp

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(../dummyInclude)

# Include config.h directory at first.
include_directories("${PROJECT_BINARY_DIR}/conf")

INCLUDE_DIRECTORIES(.)

# include spdlog (logging library)
FIND_PATH(SPDLOG_INCLUDE_DIR spdlog.h /usr/local/include/spdlog /usr/include/spdlog)
IF (SPDLOG_INCLUDE_DIR)
  INCLUDE_DIRECTORIES(${SPDLOG_INCLUDE_DIR})
  MESSAGE(STATUS "Found spdlog: ${SPDLOG_INCLUDE_DIR}")
ELSE (SPDLOG_INCLUDE_DIR)
 MESSAGE(FATAL_ERROR "Could not find spdlog")
ENDIF (SPDLOG_INCLUDE_DIR)  
  
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})

IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)

IF(faramotics_FOUND)
    INCLUDE_DIRECTORIES(${faramotics_INCLUDE_DIRS})
ENDIF(faramotics_FOUND)

IF(laser_scan_utils_FOUND)
    INCLUDE_DIRECTORIES(${laser_scan_utils_INCLUDE_DIRS})
ENDIF(laser_scan_utils_FOUND)

IF(raw_gps_utils_FOUND)
    INCLUDE_DIRECTORIES(${raw_gps_utils_INCLUDE_DIRS})
ENDIF(raw_gps_utils_FOUND)

IF(vision_utils_FOUND)
	INCLUDE_DIRECTORIES(${vision_utils_INCLUDE_DIR})
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(vision_utils_FOUND)

IF(OPENCV_FOUND)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
ENDIF(OPENCV_FOUND)

# cereal
IF(cereal_FOUND)
    INCLUDE_DIRECTORIES(${cereal_INCLUDE_DIRS})
ENDIF(cereal_FOUND)

IF(Suitesparse_FOUND)
    INCLUDE_DIRECTORIES(${Suitesparse_INCLUDE_DIRS})
ENDIF(Suitesparse_FOUND)

IF(YAMLCPP_FOUND)
    INCLUDE_DIRECTORIES(${YAMLCPP_INCLUDE_DIR})
ENDIF(YAMLCPP_FOUND)

IF(GLOG_FOUND)
    INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIR})
ENDIF(GLOG_FOUND)

IF(APRILTAG_INCLUDE_DIR)
    INCLUDE_DIRECTORIES(${APRILTAG_INCLUDE_DIR})
ENDIF(APRILTAG_INCLUDE_DIR)

#headers
SET(HDRS_BASE
capture/capture_motion.h
    eigen_assert.h
    eigen_predicates.h
landmark/landmark_match.h
    make_unique.h
    pinhole_tools.h
processor/processor_capture_holder.h
processor/processor_tracker_landmark.h
    )
SET(HDRS
capture/capture_motion.h
capture/capture_GPS_fix.h
capture/capture_IMU.h
capture/capture_odom_2D.h
capture/capture_odom_3D.h
factor/factor_block_absolute.h
factor/factor_container.h
factor/factor_corner_2D.h
factor/factor_AHP.h
factor/factor_epipolar.h
factor/factor_IMU.h
factor/factor_fix_bias.h
factor/factor_GPS_2D.h
factor/factor_GPS_pseudorange_3D.h
factor/factor_GPS_pseudorange_2D.h
factor/factor_odom_2D.h
factor/factor_odom_2D_analytic.h
factor/factor_odom_3D.h
factor/factor_point_2D.h
factor/factor_point_to_line_2D.h
factor/factor_pose_2D.h
factor/factor_pose_3D.h
factor/factor_quaternion_absolute.h
factor/factor_relative_2D_analytic.h
    temp/diff_drive_tools.h
    temp/diff_drive_tools.hpp
feature/feature_corner_2D.h
feature/feature_GPS_fix.h
feature/feature_GPS_pseudorange.h
feature/feature_IMU.h
feature/feature_odom_2D.h
feature/feature_polyline_2D.h
    IMU_tools.h
landmark/landmark_corner_2D.h
landmark/landmark_container.h
landmark/landmark_line_2D.h
landmark/landmark_polyline_2D.h
    local_parametrization_polyline_extreme.h
processor/processor_frame_nearest_neighbor_filter.h
processor/processor_IMU.h
    test/processor_IMU_UnitTester.h
processor/processor_odom_2D.h
processor/processor_odom_3D.h
processor/processor_tracker_feature_dummy.h
processor/processor_tracker_landmark_dummy.h
sensor/sensor_camera.h
sensor/sensor_GPS.h
sensor/sensor_GPS_fix.h
sensor/sensor_IMU.h
sensor/sensor_odom_2D.h
sensor/sensor_odom_3D.h
    )
  SET(HDRS_CAPTURE
capture/capture_GPS_fix.h
capture/capture_IMU.h
capture/capture_odom_2D.h
capture/capture_odom_3D.h
capture/capture_velocity.h
capture/capture_wheel_joint_position.h
    )
  SET(HDRS_CONSTRAINT
factor/factor_autodiff_trifocal.h
factor/factor_autodiff_distance_3D.h
factor/factor_AHP.h
factor/factor_block_absolute.h
factor/factor_container.h
factor/factor_corner_2D.h
factor/factor_diff_drive.h
factor/factor_epipolar.h
factor/factor_IMU.h
factor/factor_fix_bias.h
factor/factor_GPS_2D.h
factor/factor_GPS_pseudorange_3D.h
factor/factor_GPS_pseudorange_2D.h
factor/factor_odom_2D.h
factor/factor_odom_2D_analytic.h
factor/factor_odom_3D.h
factor/factor_point_2D.h
factor/factor_point_to_line_2D.h
factor/factor_pose_2D.h
factor/factor_pose_3D.h
factor/factor_quaternion_absolute.h
factor/factor_relative_2D_analytic.h
    )
  SET(HDRS_FEATURE
feature/feature_corner_2D.h
feature/feature_diff_drive.h
feature/feature_GPS_fix.h
feature/feature_GPS_pseudorange.h
feature/feature_IMU.h
feature/feature_odom_2D.h
feature/feature_polyline_2D.h
    )
  SET(HDRS_LANDMARK
landmark/landmark_match.h
landmark/landmark_corner_2D.h
landmark/landmark_container.h
landmark/landmark_line_2D.h
landmark/landmark_polyline_2D.h
    )
  SET(HDRS_PROCESSOR
processor/processor_capture_holder.h
processor/processor_diff_drive.h
processor/processor_frame_nearest_neighbor_filter.h
processor/processor_IMU.h
processor/processor_odom_2D.h
processor/processor_odom_3D.h
processor/processor_tracker_feature_dummy.h
processor/processor_tracker_landmark.h
processor/processor_tracker_landmark_dummy.h
    )
  SET(HDRS_SENSOR
sensor/sensor_camera.h
sensor/sensor_diff_drive.h
sensor/sensor_GPS.h
sensor/sensor_GPS_fix.h
sensor/sensor_IMU.h
sensor/sensor_odom_2D.h
sensor/sensor_odom_3D.h
    )
# [Add generic derived header before this line]

SET(HDRS_DTASSC
    data_association/matrix.h
    data_association/association_solver.h
    data_association/association_node.h
    data_association/association_tree.h
    data_association/association_nnls.h
    )

SET(HDRS_CORE 
core/capture_base.h
core/capture_buffer.h
core/capture_pose.h
core/capture_void.h
core/factor_analytic.h
core/factor_autodiff.h
core/factor_base.h
core/factory.h
core/feature_base.h
core/feature_match.h
core/feature_pose.h
core/frame_base.h
core/hardware_base.h
core/landmark_base.h
core/local_parametrization_angle.h
core/local_parametrization_base.h
core/local_parametrization_homogeneous.h
core/local_parametrization_quaternion.h
core/logging.h
core/map_base.h
core/motion_buffer.h
core/node_base.h
core/problem.h
core/processor_base.h
core/factory.h
core/processor_loopclosure_base.h
core/processor_motion.h
core/processor_tracker_feature.h
core/processor_tracker.h
core/rotations.h
core/sensor_base.h
core/factory.h
core/singleton.h
core/state_angle.h
core/state_block.h
core/state_homogeneous_3D.h
core/state_quaternion.h
core/three_D_tools.h
core/time_stamp.h
core/track_matrix.h
core/trajectory_base.h
core/wolf.h
  )
SET(SRCS_CORE 
core/capture_base.cpp
core/capture_pose.cpp
core/capture_void.cpp
core/factor_analytic.cpp
core/factor_base.cpp
core/feature_base.cpp
core/feature_pose.cpp
core/frame_base.cpp
core/hardware_base.cpp
core/landmark_base.cpp
core/local_parametrization_base.cpp
core/local_parametrization_homogeneous.cpp
core/local_parametrization_quaternion.cpp
core/map_base.cpp
core/motion_buffer.cpp
core/node_base.cpp
core/problem.cpp
core/processor_base.cpp
core/processor_loopclosure_base.cpp
core/processor_motion.cpp
core/processor_tracker.cpp
core/processor_tracker_feature.cpp
core/sensor_base.cpp
core/state_block.cpp
core/time_stamp.cpp
core/track_matrix.cpp
core/trajectory_base.cpp
  )
#sources
SET(SRCS_BASE
capture/capture_motion.cpp
processor/processor_capture_holder.cpp
processor/processor_tracker_landmark.cpp
    )

SET(SRCS
  local_parametrization_polyline_extreme.cpp
  test/processor_IMU_UnitTester.cpp
  )
  SET(SRCS_CAPTURE
capture/capture_GPS_fix.cpp
capture/capture_IMU.cpp
capture/capture_odom_2D.cpp
capture/capture_odom_3D.cpp
capture/capture_velocity.cpp
capture/capture_wheel_joint_position.cpp
    )
  SET(SRCS_FEATURE
feature/feature_corner_2D.cpp
feature/feature_diff_drive.cpp
feature/feature_GPS_fix.cpp
feature/feature_GPS_pseudorange.cpp
feature/feature_IMU.cpp
feature/feature_odom_2D.cpp
feature/feature_polyline_2D.cpp
    )
  SET(SRCS_LANDMARK
landmark/landmark_corner_2D.cpp
landmark/landmark_container.cpp
landmark/landmark_line_2D.cpp
landmark/landmark_polyline_2D.cpp
    )
  SET(SRCS_PROCESSOR
processor/processor_frame_nearest_neighbor_filter.cpp
processor/processor_diff_drive.cpp
processor/processor_IMU.cpp
processor/processor_odom_2D.cpp
processor/processor_odom_3D.cpp
processor/processor_tracker_feature_dummy.cpp
processor/processor_tracker_landmark_dummy.cpp
    )
  SET(SRCS_SENSOR
sensor/sensor_camera.cpp
sensor/sensor_diff_drive.cpp
sensor/sensor_GPS.cpp
sensor/sensor_GPS_fix.cpp
sensor/sensor_IMU.cpp
sensor/sensor_odom_2D.cpp
sensor/sensor_odom_3D.cpp
    )
SET(SRCS_DTASSC
    data_association/association_solver.cpp
    data_association/association_node.cpp
    data_association/association_tree.cpp
    data_association/association_nnls.cpp
    )

# Add the solver sub-directory
add_subdirectory(solver)

#optional HDRS and SRCS
IF (Ceres_FOUND)
    SET(HDRS_WRAPPER
        ceres_wrapper/sparse_utils.h
        #ceres_wrapper/solver_manager.h
        ceres_wrapper/ceres_manager.h
        #ceres_wrapper/qr_manager.h
        ceres_wrapper/cost_function_wrapper.h
        ceres_wrapper/create_numeric_diff_cost_function.h
        ceres_wrapper/local_parametrization_wrapper.h 
        )
    SET(SRCS_WRAPPER
        #ceres_wrapper/solver_manager.cpp
        ceres_wrapper/ceres_manager.cpp
        #ceres_wrapper/qr_manager.cpp
        ceres_wrapper/local_parametrization_wrapper.cpp 
        )
ELSE(Ceres_FOUND)
    SET(HDRS_WRAPPER)
    SET(SRCS_WRAPPER)
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    SET(HDRS ${HDRS}
capture/capture_laser_2D.h
sensor/sensor_laser_2D.h
processor/processor_tracker_feature_corner.h
processor/processor_tracker_landmark_corner.h
processor/processor_tracker_landmark_polyline.h
        )
    SET(SRCS ${SRCS}
capture/capture_laser_2D.cpp
sensor/sensor_laser_2D.cpp
processor/processor_tracker_feature_corner.cpp
processor/processor_tracker_landmark_corner.cpp
processor/processor_tracker_landmark_polyline.cpp
        )
ENDIF(laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    SET(HDRS ${HDRS}
capture/capture_GPS.h
processor/processor_GPS.h
        )
    SET(SRCS ${SRCS}
capture/capture_GPS.cpp
processor/processor_GPS.cpp
        )
ENDIF(raw_gps_utils_FOUND)

# Vision
IF (vision_utils_FOUND)
    SET(HDRS ${HDRS}
capture/capture_image.h
feature/feature_point_image.h
landmark/landmark_AHP.h
processor/processor_params_image.h
processor/processor_tracker_feature_image.h
processor/processor_tracker_landmark_image.h
        )
    SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
processor/processor_tracker_feature_trifocal.h
    )
  SET(HDRS_LANDMARK ${HDRS_LANDMARK}
landmark/landmark_point_3D.h
    )
    SET(SRCS ${SRCS}
capture/capture_image.cpp
feature/feature_point_image.cpp
landmark/landmark_AHP.cpp
processor/processor_tracker_feature_image.cpp
processor/processor_tracker_landmark_image.cpp
        )
    SET(SRCS_LANDMARK ${SRCS_LANDMARK}
landmark/landmark_point_3D.cpp
    )
    SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
processor/processor_tracker_feature_trifocal.cpp
    )
ENDIF(vision_utils_FOUND)

IF (OPENCV_FOUND AND Apriltag_FOUND)
    SET(HDRS ${HDRS}
        landmark_apriltag.h
        )
    SET(SRCS ${SRCS}
        landmark_apriltag.cpp
        )
ENDIF(OPENCV_FOUND AND Apriltag_FOUND)

# Add the capture sub-directory
# ADD_SUBDIRECTORY(captures)

# Add the factors sub-directory
# ADD_SUBDIRECTORY(factors)

# Add the features sub-directory
# ADD_SUBDIRECTORY(features)

# Add the landmarks sub-directory
# ADD_SUBDIRECTORY(landmarks)

# Add the processors sub-directory
# ADD_SUBDIRECTORY(processors)

# Add the sensors sub-directory
# ADD_SUBDIRECTORY(sensors)

# Add the hello_wolf sub-directory
ADD_SUBDIRECTORY(hello_wolf)

IF (cereal_FOUND)
  ADD_SUBDIRECTORY(serialization/cereal)
ENDIF(cereal_FOUND)

IF (Suitesparse_FOUND)
    ADD_SUBDIRECTORY(solver_suitesparse)
ENDIF(Suitesparse_FOUND)

# LEAVE YAML FILES ALWAYS IN THE LAST POSITION !!
IF(YAMLCPP_FOUND)
    # headers
    SET(HDRS ${HDRS}
        yaml/yaml_conversion.h
        )

    # sources
    SET(SRCS ${SRCS}
        yaml/processor_odom_3D_yaml.cpp
        yaml/processor_IMU_yaml.cpp
        yaml/sensor_camera_yaml.cpp
        yaml/sensor_odom_3D_yaml.cpp
        yaml/sensor_IMU_yaml.cpp
        )
    IF(laser_scan_utils_FOUND)
        SET(SRCS ${SRCS}
            yaml/sensor_laser_2D_yaml.cpp
            )
    ENDIF(laser_scan_utils_FOUND)
    IF(vision_utils_FOUND)
        SET(SRCS ${SRCS}
        	yaml/processor_image_yaml.cpp
        	yaml/processor_tracker_feature_trifocal_yaml.cpp
            )
            IF(Apriltag_FOUND)
                SET(SRCS ${SRCS}
                    yaml/processor_tracker_landmark_apriltag_yaml.cpp
                )
            ENDIF(Apriltag_FOUND)
    ENDIF(vision_utils_FOUND)
ENDIF(YAMLCPP_FOUND)

# create the shared library
add_library(${PROJECT_NAME}_core SHARED
                 ${SRCS_CORE}
                 ${SRCS_BASE} 
                )
ADD_LIBRARY(${PROJECT_NAME} 
            SHARED 
            ${SRCS_BASE} 
            ${SRCS}
            ${SRCS_CAPTURE}
            ${SRCS_CONSTRAINT}
            ${SRCS_FEATURE}
            ${SRCS_LANDMARK}
            ${SRCS_PROCESSOR}
            ${SRCS_SENSOR}
            #${SRCS_DTASSC} 
            ${SRCS_SOLVER}
            ${SRCS_WRAPPER}
            )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PROJECT_NAME}_core)
#Link the created libraries
#=============================================================
IF (Ceres_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF(Ceres_FOUND)

IF (laser_scan_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${laser_scan_utils_LIBRARY})
ENDIF (laser_scan_utils_FOUND)

IF (raw_gps_utils_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${raw_gps_utils_LIBRARY})
ENDIF (raw_gps_utils_FOUND)

IF (OPENCV_FOUND)
   	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
	IF (vision_utils_FOUND)
    	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${vision_utils_LIBRARY})
	ENDIF (vision_utils_FOUND)
ENDIF (OPENCV_FOUND)

IF (OPENCV_FOUND)
   	TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
ENDIF (OPENCV_FOUND)

IF (YAMLCPP_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YAMLCPP_LIBRARY})
ENDIF (YAMLCPP_FOUND)

IF (GLOG_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GLOG_LIBRARY})
ENDIF (GLOG_FOUND)

#check if this is done correctly
IF (OPENCV_FOUND AND Apriltag_FOUND)
    LINK_LIBRARIES(apriltag m)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${APRILTAG_LIBRARY}  ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS} m)
ENDIF(OPENCV_FOUND AND Apriltag_FOUND)

#install library
install(TARGETS ${PROJECT_NAME}_core DESTINATION lib/iri-algorithms EXPORT ${PROJECT_NAME}_core-targets)
install(EXPORT ${PROJECT_NAME}_core-targets DESTINATION lib/iri-algorithms)
#=============================================================
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)

install(EXPORT ${PROJECT_NAME}Targets DESTINATION lib/cmake/${PROJECT_NAME})

#install headers
INSTALL(FILES ${HDRS_BASE}
    DESTINATION include/iri-algorithms/wolf)
INSTALL(FILES ${HDRS}
    DESTINATION include/iri-algorithms/wolf)
INSTALL(FILES ${HDRS_CORE}
    DESTINATION include/iri-algorithms/wolf/core)
#INSTALL(FILES ${HDRS_DTASSC}
#    DESTINATION include/iri-algorithms/wolf/data_association)
INSTALL(FILES ${HDRS_CAPTURE}
    DESTINATION include/iri-algorithms/wolf/capture)
INSTALL(FILES ${HDRS_CONSTRAINT}
    DESTINATION include/iri-algorithms/wolf/factor)
INSTALL(FILES ${HDRS_FEATURE}
    DESTINATION include/iri-algorithms/wolf/feature)
INSTALL(FILES ${HDRS_SENSOR}
    DESTINATION include/iri-algorithms/wolf/sensor)
INSTALL(FILES ${HDRS_PROCESSOR}
    DESTINATION include/iri-algorithms/wolf/processor)
INSTALL(FILES ${HDRS_LANDMARK}
    DESTINATION include/iri-algorithms/wolf/landmark)
INSTALL(FILES ${HDRS_WRAPPER}
    DESTINATION include/iri-algorithms/wolf/ceres_wrapper)
#INSTALL(FILES ${HDRS_SOLVER_SUITESPARSE}
#    DESTINATION include/iri-algorithms/wolf/solver_suitesparse)
INSTALL(FILES ${HDRS_SOLVER}
    DESTINATION include/iri-algorithms/wolf/solver)
INSTALL(FILES ${HDRS_SERIALIZATION}
    DESTINATION include/iri-algorithms/wolf/serialization)
INSTALL(FILES "${CMAKE_SOURCE_DIR}/cmake_modules/Findwolf.cmake"
    DESTINATION "lib/cmake/${PROJECT_NAME}")

#install Find*.cmake
configure_file("${CMAKE_SOURCE_DIR}/cmake_modules/wolfConfig.cmake"
               "${CMAKE_BINARY_DIR}/wolfConfig.cmake" @ONLY)

INSTALL(FILES "${WOLF_CONFIG_DIR}/config.h"
DESTINATION include/iri-algorithms/wolf/internal)

INSTALL(FILES "${CMAKE_BINARY_DIR}/wolfConfig.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")

INSTALL(DIRECTORY ${SPDLOG_INCLUDE_DIRS} DESTINATION "include/iri-algorithms/")

export(PACKAGE ${PROJECT_NAME})

#############
## Testing ##
#############
IF (GLOG_FOUND)
    IF(BUILD_TESTS)
        MESSAGE("Building tests.")
        add_subdirectory(test)
    ENDIF(BUILD_TESTS)
ENDIF (GLOG_FOUND)

IF(BUILD_EXAMPLES)
  #Build examples
  MESSAGE("Building examples.")
  ADD_SUBDIRECTORY(examples)
ENDIF(BUILD_EXAMPLES)

