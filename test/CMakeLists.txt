# Retrieve googletest from github & compile
add_subdirectory(gtest)


## Added these two include_dirs: ######################
#
#CMAKE modules
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
#MESSAGE(STATUS ${CMAKE_MODULE_PATH})
#
# Include Eigen
#FIND_PACKAGE(Eigen 3 REQUIRED)
#INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
#
# Include Ceres
#FIND_PACKAGE(Ceres QUIET) #Ceres is not required
#IF(Ceres_FOUND)
#    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
#ENDIF(Ceres_FOUND)
#
## and now gtest_motion_2d works ######################


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

############# USE THIS TEST AS AN EXAMPLE #################
#                                                         #
# Create a specific test executable for gtest_example     #
wolf_add_gtest(gtest_example gtest_example.cpp)           #
target_link_libraries(gtest_example ${PROJECT_NAME})      #
#                                                         #
###########################################################


################# ADD YOUR TESTS BELOW ####################
#                                                         #
#           ==== IN ALPHABETICAL ORDER! ====              #
#                                                         #

# ------- First Core classes ----------

# CaptureBase class test
wolf_add_gtest(gtest_capture_base gtest_capture_base.cpp)
target_link_libraries(gtest_capture_base ${PROJECT_NAME})

# CaptureBase class test
#wolf_add_gtest(gtest_factor_sparse gtest_factor_sparse.cpp)
#target_link_libraries(gtest_factor_sparse ${PROJECT_NAME})

# FactorAutodiff class test
wolf_add_gtest(gtest_factor_autodiff gtest_factor_autodiff.cpp)
target_link_libraries(gtest_factor_autodiff ${PROJECT_NAME})

# FeatureBase classes test
wolf_add_gtest(gtest_eigen_predicates gtest_eigen_predicates.cpp)
target_link_libraries(gtest_eigen_predicates ${PROJECT_NAME})

# FeatureBase classes test
wolf_add_gtest(gtest_feature_base gtest_feature_base.cpp)
target_link_libraries(gtest_feature_base ${PROJECT_NAME})

# FrameBase classes test
wolf_add_gtest(gtest_frame_base gtest_frame_base.cpp)
target_link_libraries(gtest_frame_base ${PROJECT_NAME})

# IMU tools test
wolf_add_gtest(gtest_IMU_tools gtest_IMU_tools.cpp)
# target_link_libraries(gtest_IMU_tools ${PROJECT_NAME}) // WOLF library not needed

# LocalParametrizationXxx classes test
wolf_add_gtest(gtest_local_param gtest_local_param.cpp)
target_link_libraries(gtest_local_param ${PROJECT_NAME})

# MotionBuffer class test
wolf_add_gtest(gtest_motion_buffer gtest_motion_buffer.cpp)
target_link_libraries(gtest_motion_buffer ${PROJECT_NAME})

# PackKFBuffer
wolf_add_gtest(gtest_pack_KF_buffer gtest_pack_KF_buffer.cpp)
target_link_libraries(gtest_pack_KF_buffer ${PROJECT_NAME})

# Problem class test
wolf_add_gtest(gtest_problem gtest_problem.cpp)
target_link_libraries(gtest_problem ${PROJECT_NAME})

# ProcessorBase class test
wolf_add_gtest(gtest_processor_base gtest_processor_base.cpp)
target_link_libraries(gtest_processor_base ${PROJECT_NAME})

# ProcessorMotion class test
wolf_add_gtest(gtest_processor_motion gtest_processor_motion.cpp)
target_link_libraries(gtest_processor_motion ${PROJECT_NAME})
  
# Rotation test
wolf_add_gtest(gtest_rotation gtest_rotation.cpp)

# SE3 test
wolf_add_gtest(gtest_SE3 gtest_SE3.cpp)

# SensorBase test
wolf_add_gtest(gtest_sensor_base gtest_sensor_base.cpp)
target_link_libraries(gtest_sensor_base ${PROJECT_NAME})

# shared_from_this test
wolf_add_gtest(gtest_shared_from_this gtest_shared_from_this.cpp)
target_link_libraries(gtest_shared_from_this ${PROJECT_NAME})

# SolverManager test
wolf_add_gtest(gtest_solver_manager gtest_solver_manager.cpp)
target_link_libraries(gtest_solver_manager ${PROJECT_NAME})

# TimeStamp class test
wolf_add_gtest(gtest_time_stamp gtest_time_stamp.cpp)
target_link_libraries(gtest_time_stamp ${PROJECT_NAME})

# TrackMatrix class test
wolf_add_gtest(gtest_track_matrix gtest_track_matrix.cpp)
target_link_libraries(gtest_track_matrix ${PROJECT_NAME})

# TrajectoryBase class test
wolf_add_gtest(gtest_trajectory gtest_trajectory.cpp)
target_link_libraries(gtest_trajectory ${PROJECT_NAME})

# ------- Now Derived classes ----------

IF (Ceres_FOUND)
# CeresManager test
wolf_add_gtest(gtest_ceres_manager gtest_ceres_manager.cpp)
target_link_libraries(gtest_ceres_manager ${PROJECT_NAME})
ENDIF(Ceres_FOUND)

# FactorAbs(P/O/V) classes test
wolf_add_gtest(gtest_factor_absolute gtest_factor_absolute.cpp)
target_link_libraries(gtest_factor_absolute ${PROJECT_NAME})

IF (OPENCV_FOUND AND APRILTAG_LIBRARY)
	# FactorAutodiffApriltag test
	wolf_add_gtest(gtest_factor_autodiff_apriltag gtest_factor_autodiff_apriltag.cpp)
	target_link_libraries(gtest_factor_autodiff_apriltag ${PROJECT_NAME})
ENDIF(OPENCV_FOUND AND APRILTAG_LIBRARY)

# FactorAutodiffDistance3D test
wolf_add_gtest(gtest_factor_autodiff_distance_3D gtest_factor_autodiff_distance_3D.cpp)
target_link_libraries(gtest_factor_autodiff_distance_3D ${PROJECT_NAME})

IF(vision_utils_FOUND)
# FactorAutodiffTrifocal test
wolf_add_gtest(gtest_factor_autodiff_trifocal gtest_factor_autodiff_trifocal.cpp)
target_link_libraries(gtest_factor_autodiff_trifocal ${PROJECT_NAME})
ENDIF(vision_utils_FOUND)

# FactorOdom3D class test
wolf_add_gtest(gtest_factor_odom_3D gtest_factor_odom_3D.cpp)
target_link_libraries(gtest_factor_odom_3D ${PROJECT_NAME})

# FactorPose2D class test
wolf_add_gtest(gtest_factor_pose_2D gtest_factor_pose_2D.cpp)
target_link_libraries(gtest_factor_pose_2D ${PROJECT_NAME})

# FactorPose3D class test
wolf_add_gtest(gtest_factor_pose_3D gtest_factor_pose_3D.cpp)
target_link_libraries(gtest_factor_pose_3D ${PROJECT_NAME})

IF (OPENCV_FOUND AND APRILTAG_LIBRARY)
	# FeatureApriltag test
	wolf_add_gtest(gtest_feature_apriltag gtest_feature_apriltag.cpp)
	target_link_libraries(gtest_feature_apriltag ${PROJECT_NAME})
ENDIF(OPENCV_FOUND AND APRILTAG_LIBRARY)

# FeatureIMU test
wolf_add_gtest(gtest_feature_IMU gtest_feature_IMU.cpp)
target_link_libraries(gtest_feature_IMU ${PROJECT_NAME})

# IMUTest (Bias + estimation)
wolf_add_gtest(gtest_IMU gtest_IMU.cpp)
target_link_libraries(gtest_IMU ${PROJECT_NAME})

IF (OPENCV_FOUND AND APRILTAG_LIBRARY)
	# gtest_landmark_apriltag test
	wolf_add_gtest(gtest_landmark_apriltag gtest_landmark_apriltag.cpp)
	target_link_libraries(gtest_landmark_apriltag ${PROJECT_NAME})
ENDIF(OPENCV_FOUND AND APRILTAG_LIBRARY)

# MakePosDef function test
wolf_add_gtest(gtest_make_posdef gtest_make_posdef.cpp)
target_link_libraries(gtest_make_posdef ${PROJECT_NAME})

# Parameter prior test
wolf_add_gtest(gtest_param_prior gtest_param_prior.cpp)
target_link_libraries(gtest_param_prior ${PROJECT_NAME})

# Pinhole test
wolf_add_gtest(gtest_pinhole gtest_pinhole.cpp)
target_link_libraries(gtest_pinhole ${PROJECT_NAME})

# ProcessorFrameNearestNeighborFilter class test
wolf_add_gtest(gtest_processor_frame_nearest_neighbor_filter_2D gtest_processor_frame_nearest_neighbor_filter_2D.cpp)
target_link_libraries(gtest_processor_frame_nearest_neighbor_filter_2D ${PROJECT_NAME})

# ProcessorIMU class test
wolf_add_gtest(gtest_processor_IMU gtest_processor_IMU.cpp)
target_link_libraries(gtest_processor_IMU ${PROJECT_NAME})

# ProcessorIMUJacobians test
wolf_add_gtest(gtest_processor_IMU_jacobians gtest_processor_IMU_jacobians.cpp)
target_link_libraries(gtest_processor_IMU_jacobians ${PROJECT_NAME})

IF(vision_utils_FOUND)
# ProcessorFeatureTrifocal test
wolf_add_gtest(gtest_processor_tracker_feature_trifocal gtest_processor_tracker_feature_trifocal.cpp)
target_link_libraries(gtest_processor_tracker_feature_trifocal ${PROJECT_NAME})
ENDIF(vision_utils_FOUND)

# ProcessorMotion in 2D
wolf_add_gtest(gtest_odom_2D gtest_odom_2D.cpp)
target_link_libraries(gtest_odom_2D ${PROJECT_NAME})

# ProcessorOdom3D class test
wolf_add_gtest(gtest_odom_3D gtest_odom_3D.cpp)
target_link_libraries(gtest_odom_3D ${PROJECT_NAME})

# SensorBase test
wolf_add_gtest(gtest_sensor_camera gtest_sensor_camera.cpp)
target_link_libraries(gtest_sensor_camera ${PROJECT_NAME})


IF (OPENCV_FOUND AND APRILTAG_LIBRARY)
	# ProcessorTrackerLandmarkApriltag test
	wolf_add_gtest(gtest_processor_tracker_landmark_apriltag gtest_processor_tracker_landmark_apriltag.cpp)
	target_link_libraries(gtest_processor_tracker_landmark_apriltag ${PROJECT_NAME})
ENDIF(OPENCV_FOUND AND APRILTAG_LIBRARY)



# ------- Now Core classes Serialization ----------

add_subdirectory(serialization)

