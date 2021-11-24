# Retrieve googletest from github & compile
add_subdirectory(gtest)


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

wolf_add_gtest(gtest_feature_apriltag gtest_feature_apriltag.cpp)
target_link_libraries(gtest_feature_apriltag ${PLUGIN_NAME})

wolf_add_gtest(gtest_landmark_apriltag gtest_landmark_apriltag.cpp)
target_link_libraries(gtest_landmark_apriltag ${PLUGIN_NAME})

# ProcessorTrackerLandmarkApriltag test
#wolf_add_gtest(gtest_processor_tracker_landmark_apriltag gtest_processor_tracker_landmark_apriltag.cpp)
#target_link_libraries(gtest_processor_tracker_landmark_apriltag ${PLUGIN_NAME})

#wolf_add_gtest(gtest_factor_apriltag gtest_factor_apriltag.cpp)
#target_link_libraries(gtest_factor_apriltag ${PLUGIN_NAME})
