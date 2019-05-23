
# Vision
IF(vision_utils_FOUND)

    IF(Ceres_FOUND)
	    IF (APRILTAG_LIBRARY)
    		ADD_EXECUTABLE(demo_apriltag demo_apriltag.cpp)
    		TARGET_LINK_LIBRARIES(demo_apriltag ${PROJECT_NAME})
    	ENDIF(APRILTAG_LIBRARY)
    ENDIF(Ceres_FOUND)

ENDIF(vision_utils_FOUND)

