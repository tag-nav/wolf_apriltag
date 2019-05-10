# Retrieve googletest from github & compile
add_subdirectory(gtest)


# Include gtest directory.
include_directories(${GTEST_INCLUDE_DIRS})

############# USE THIS TEST AS AN EXAMPLE #################
#                                                         #
# Create a specific test executable for gtest_example     #
wolf_add_gtest(gtest_example gtest_example.cpp)           #
target_link_libraries(gtest_example ${PLUGIN_NAME})      #
#                                                         #
###########################################################

wolf_add_gtest(gtest_feature_apriltag gtest_feature_apriltag.cpp)
target_link_libraries(gtest_feature_apriltag ${PLUGIN_NAME} ${wolf_LIBRARY} ${OpenCV_LIBS})

wolf_add_gtest(gtest_landmark_apriltag gtest_landmark_apriltag.cpp)
target_link_libraries(gtest_landmark_apriltag ${PLUGIN_NAME} ${wolf_LIBRARY} ${OpenCV_LIBS})

# ProcessorTrackerLandmarkApriltag test
wolf_add_gtest(gtest_processor_tracker_landmark_apriltag gtest_processor_tracker_landmark_apriltag.cpp)
target_link_libraries(gtest_processor_tracker_landmark_apriltag ${PLUGIN_NAME} ${wolf_LIBRARY} ${OpenCV_LIBS})

wolf_add_gtest(gtest_factor_autodiff_apriltag gtest_factor_autodiff_apriltag.cpp)
target_link_libraries(gtest_factor_autodiff_apriltag ${PLUGIN_NAME} ${wolf_LIBRARY} ${OpenCV_LIBS})