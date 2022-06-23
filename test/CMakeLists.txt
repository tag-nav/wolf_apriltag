# Retrieve googletest from github & compile
add_subdirectory(gtest)

############# USE THIS TEST AS AN EXAMPLE #################
#                                                         #
# Create a specific test executable for gtest_example     #
# wolf_add_gtest(gtest_example gtest_example.cpp)         #
#                                                         #
###########################################################

wolf_add_gtest(gtest_feature_apriltag gtest_feature_apriltag_pose.cpp)

wolf_add_gtest(gtest_feature_apriltag_proj gtest_feature_apriltag_proj.cpp)

wolf_add_gtest(gtest_landmark_apriltag gtest_landmark_apriltag.cpp)

wolf_add_gtest(gtest_processor_tracker_landmark_apriltag gtest_processor_tracker_landmark_apriltag.cpp)

wolf_add_gtest(gtest_factor_apriltag_proj gtest_factor_apriltag_proj.cpp)
