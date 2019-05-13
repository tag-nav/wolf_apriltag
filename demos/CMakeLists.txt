# Dynamically remove objects from list
# add_executable(test_list_remove test_list_remove.cpp)

# Matrix product test
#ADD_EXECUTABLE(test_matrix_prod test_matrix_prod.cpp)

#ADD_EXECUTABLE(test_eigen_template test_eigen_template.cpp)

ADD_EXECUTABLE(test_fcn_ptr test_fcn_ptr.cpp)

ADD_EXECUTABLE(test_kf_callback test_kf_callback.cpp)
TARGET_LINK_LIBRARIES(test_kf_callback ${PROJECT_NAME})

ADD_EXECUTABLE(test_wolf_logging test_wolf_logging.cpp)
TARGET_LINK_LIBRARIES(test_wolf_logging ${PROJECT_NAME})

IF(Ceres_FOUND)
    # test_processor_odom_3D
    ADD_EXECUTABLE(test_processor_odom_3D test_processor_odom_3D.cpp)
    TARGET_LINK_LIBRARIES(test_processor_odom_3D ${PROJECT_NAME})

#    ADD_EXECUTABLE(test_motion_2d test_motion_2d.cpp)
#    TARGET_LINK_LIBRARIES(test_motion_2d ${PROJECT_NAME})
ENDIF(Ceres_FOUND)




# State blocks with local parametrizations test
#ADD_EXECUTABLE(test_state_quaternion test_state_quaternion.cpp)
#TARGET_LINK_LIBRARIES(test_state_quaternion ${PROJECT_NAME})

# Testing Eigen Permutations
#ADD_EXECUTABLE(test_permutations solver/test_permutations.cpp)
#TARGET_LINK_LIBRARIES(test_permutations ${PROJECT_NAME})

# Enable Yaml config files
IF(YAMLCPP_FOUND)
#    ADD_EXECUTABLE(test_yaml test_yaml.cpp)
#    TARGET_LINK_LIBRARIES(test_yaml ${PROJECT_NAME})

#    ADD_EXECUTABLE(test_yaml_conversions test_yaml_conversions.cpp)
#    TARGET_LINK_LIBRARIES(test_yaml_conversions ${PROJECT_NAME})

    # SensorFactory classes test
#    ADD_EXECUTABLE(test_wolf_factories test_wolf_factories.cpp)
#    TARGET_LINK_LIBRARIES(test_wolf_factories ${PROJECT_NAME})

    # Map load and save test
#    ADD_EXECUTABLE(test_map_yaml test_map_yaml.cpp)
#    TARGET_LINK_LIBRARIES(test_map_yaml ${PROJECT_NAME})
ENDIF(YAMLCPP_FOUND)

IF(Suitesparse_FOUND)
IF (0) # These cannot compile on MacOSX -- we'll fix it some day.
    # Testing a ccolamd test
    ADD_EXECUTABLE(test_ccolamd solver/test_ccolamd.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd ${PROJECT_NAME})

    # Testing a blocks ccolamd test
    ADD_EXECUTABLE(test_ccolamd_blocks solver/test_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_ccolamd_blocks ${PROJECT_NAME})

    # Testing an incremental blocks ccolamd test
    ADD_EXECUTABLE(test_incremental_ccolamd_blocks solver/test_incremental_ccolamd_blocks.cpp)
    TARGET_LINK_LIBRARIES(test_incremental_ccolamd_blocks ${PROJECT_NAME})

    # Testing an incremental QR with block ccolamd test
    ADD_EXECUTABLE(test_iQR solver/test_iQR.cpp)
    TARGET_LINK_LIBRARIES(test_iQR ${PROJECT_NAME})

    # Testing QR solver test tending to wolf
    ADD_EXECUTABLE(test_iQR_wolf solver/test_iQR_wolf.cpp)
    TARGET_LINK_LIBRARIES(test_iQR_wolf ${PROJECT_NAME})

    # Testing SPQR simple test
    #ADD_EXECUTABLE(test_SPQR solver/test_SPQR.cpp)
    #TARGET_LINK_LIBRARIES(test_SPQR ${PROJECT_NAME})
ENDIF(0)

ENDIF(Suitesparse_FOUND)

# Building and populating the wolf tree
# ADD_EXECUTABLE(test_wolf_tree test_wolf_tree.cpp)
# TARGET_LINK_LIBRARIES(test_wolf_tree ${PROJECT_NAME})

# Building and populating the wolf tree from .graph file (TORO)
#ADD_EXECUTABLE(test_wolf_imported_graph test_wolf_imported_graph.cpp)
#TARGET_LINK_LIBRARIES(test_wolf_imported_graph ${PROJECT_NAME})

# Comparing performance of wolf auto_diff and ceres auto_diff
#ADD_EXECUTABLE(test_wolf_autodiffwrapper test_wolf_autodiffwrapper.cpp)
#TARGET_LINK_LIBRARIES(test_wolf_autodiffwrapper ${PROJECT_NAME})

# Prunning wolf tree from .graph file (TORO)
#ADD_EXECUTABLE(test_wolf_prunning test_wolf_prunning.cpp)
#TARGET_LINK_LIBRARIES(test_wolf_prunning ${PROJECT_NAME})

# Sparsification from wolf tree from .graph file (TORO)
ADD_EXECUTABLE(test_sparsification test_sparsification.cpp)
TARGET_LINK_LIBRARIES(test_sparsification ${PROJECT_NAME})

# Comparing analytic and autodiff odometry factors
#ADD_EXECUTABLE(test_analytic_odom_factor test_analytic_odom_factor.cpp)
#TARGET_LINK_LIBRARIES(test_analytic_odom_factor ${PROJECT_NAME})

# Vision
IF(vision_utils_FOUND)

    IF(Ceres_FOUND)
        # Testing many things for the 3D image odometry
        ADD_EXECUTABLE(test_image test_image.cpp)
        TARGET_LINK_LIBRARIES(test_image ${PROJECT_NAME})

        # Processor Image Landmark test
        ADD_EXECUTABLE(test_processor_tracker_landmark_image test_processor_tracker_landmark_image.cpp)
        TARGET_LINK_LIBRARIES(test_processor_tracker_landmark_image ${PROJECT_NAME})

	    # Simple AHP test
	    ADD_EXECUTABLE(test_simple_AHP test_simple_AHP.cpp)
	    TARGET_LINK_LIBRARIES(test_simple_AHP ${PROJECT_NAME})

	    IF (APRILTAG_LIBRARY)
    		ADD_EXECUTABLE(test_apriltag test_apriltag.cpp)
    		TARGET_LINK_LIBRARIES(test_apriltag ${PROJECT_NAME})
    	ENDIF(APRILTAG_LIBRARY)

    ENDIF(Ceres_FOUND)

    # Testing OpenCV functions for projection of points
    ADD_EXECUTABLE(test_projection_points test_projection_points.cpp)
    TARGET_LINK_LIBRARIES(test_projection_points ${PROJECT_NAME})

    # Factor test
    ADD_EXECUTABLE(test_factor_AHP test_factor_AHP.cpp)
    TARGET_LINK_LIBRARIES(test_factor_AHP ${PROJECT_NAME})

    # ORB tracker test
    ADD_EXECUTABLE(test_tracker_ORB test_tracker_ORB.cpp)
    TARGET_LINK_LIBRARIES(test_tracker_ORB ${PROJECT_NAME})

ENDIF(vision_utils_FOUND)

# Processor Tracker Feature test
ADD_EXECUTABLE(test_processor_tracker_feature test_processor_tracker_feature.cpp)
TARGET_LINK_LIBRARIES(test_processor_tracker_feature ${PROJECT_NAME})

# Processor Tracker Landmark test
ADD_EXECUTABLE(test_processor_tracker_landmark test_processor_tracker_landmark.cpp)
TARGET_LINK_LIBRARIES(test_processor_tracker_landmark ${PROJECT_NAME})

# Processor IMU test
ADD_EXECUTABLE(test_processor_imu test_processor_imu.cpp)
TARGET_LINK_LIBRARIES(test_processor_imu ${PROJECT_NAME})

# Processor Diff-Drive test
ADD_EXECUTABLE(test_processor_diff_drive test_diff_drive.cpp)
TARGET_LINK_LIBRARIES(test_processor_diff_drive ${PROJECT_NAME})

# MPU6050 IMU test
#ADD_EXECUTABLE(test_mpu test_mpu.cpp)
#TARGET_LINK_LIBRARIES(test_mpu ${PROJECT_NAME})

# Processor IMU - Jacobian checking test
#ADD_EXECUTABLE(test_processor_imu_jacobians test_processor_imu_jacobians.cpp)
#TARGET_LINK_LIBRARIES(test_processor_imu_jacobians ${PROJECT_NAME})

# Test IMU using printed Dock
#ADD_EXECUTABLE(test_imuDock test_imuDock.cpp)
#TARGET_LINK_LIBRARIES(test_imuDock ${PROJECT_NAME})

# Test IMU with auto KF generation (Humanoids 20017)
#ADD_EXECUTABLE(test_imuDock_autoKFs test_imuDock_autoKFs.cpp)
#TARGET_LINK_LIBRARIES(test_imuDock_autoKFs ${PROJECT_NAME})

# FactorIMU - factors test
#ADD_EXECUTABLE(test_imu_constrained0 test_imu_constrained0.cpp)
#TARGET_LINK_LIBRARIES(test_imu_constrained0 ${PROJECT_NAME})

# IMU - Offline plateform test
#ADD_EXECUTABLE(test_imuPlateform_Offline test_imuPlateform_Offline.cpp)
#TARGET_LINK_LIBRARIES(test_imuPlateform_Offline ${PROJECT_NAME})

# IMU - factorIMU
#ADD_EXECUTABLE(test_factor_imu test_factor_imu.cpp)
#TARGET_LINK_LIBRARIES(test_factor_imu ${PROJECT_NAME})

# IF (laser_scan_utils_FOUND)
#     ADD_EXECUTABLE(test_capture_laser_2D test_capture_laser_2D.cpp)
#     TARGET_LINK_LIBRARIES(test_capture_laser_2D ${PROJECT_NAME})
# #ENDIF (laser_scan_utils_FOUND)

# IF(faramotics_FOUND)
#     IF (laser_scan_utils_FOUND)
#         ADD_EXECUTABLE(test_ceres_2_lasers test_ceres_2_lasers.cpp)
#         TARGET_LINK_LIBRARIES(test_ceres_2_lasers
#                                 ${pose_state_time_LIBRARIES}
#                                 ${faramotics_LIBRARIES}
#                                 ${PROJECT_NAME})
#         ADD_EXECUTABLE(test_ceres_2_lasers_polylines test_ceres_2_lasers_polylines.cpp)
#         TARGET_LINK_LIBRARIES(test_ceres_2_lasers_polylines
#                                 ${pose_state_time_LIBRARIES}
#                                 ${faramotics_LIBRARIES}
#                                 ${PROJECT_NAME})
#         ADD_EXECUTABLE(test_2_lasers_offline test_2_lasers_offline.cpp)
#         TARGET_LINK_LIBRARIES(test_2_lasers_offline
#                                 ${pose_state_time_LIBRARIES}
#                                 ${faramotics_LIBRARIES}
#                                 ${PROJECT_NAME})
#         ADD_EXECUTABLE(test_faramotics_simulation test_faramotics_simulation.cpp)
#         TARGET_LINK_LIBRARIES(test_faramotics_simulation
#                                 ${pose_state_time_LIBRARIES}
#                                 ${faramotics_LIBRARIES}
#                                 ${PROJECT_NAME})
# #        ADD_EXECUTABLE(test_autodiff test_autodiff.cpp)
# #        TARGET_LINK_LIBRARIES(test_autodiff
# #                                ${pose_state_time_LIBRARIES}
# #                                ${faramotics_LIBRARIES}
# #                                ${PROJECT_NAME})
# #        IF(Suitesparse_FOUND)
# #            ADD_EXECUTABLE(test_iQR_wolf2 solver/test_iQR_wolf2.cpp)
# #            TARGET_LINK_LIBRARIES(test_iQR_wolf2
# #                                ${pose_state_time_LIBRARIES}
# #                                ${faramotics_LIBRARIES}
# #                                ${PROJECT_NAME})
# #        ENDIF(Suitesparse_FOUND)
# #     ENDIF (laser_scan_utils_FOUND)
# # ENDIF(faramotics_FOUND)
