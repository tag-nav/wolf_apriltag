INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#=========================================
#=========================================
# Add in this section the CONDITIONAL CLUES [IF/ELSE]  
# for external libraries and move inside them the respective lines from above.  

IF (vision_utils_FOUND)
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.h
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_feature_trifocal.cpp
  )
ENDIF (vision_utils_FOUND)

IF (OPENCV_FOUND AND Apriltag_FOUND)
  SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ippe.h
      )
  SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
    ${CMAKE_CURRENT_SOURCE_DIR}/processor_tracker_landmark_apriltag.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ippe.cpp
  )
ENDIF(OPENCV_FOUND AND Apriltag_FOUND)
#=========================================
#=========================================
  
  
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.h 
  )

SET(SRCS_PROCESSOR ${SRCS_PROCESSOR}
  ${CMAKE_CURRENT_SOURCE_DIR}/processor_diff_drive.cpp 
  )

# Forward var to parent scope
# These lines always at the end
SET(HDRS_PROCESSOR ${HDRS_PROCESSOR} PARENT_SCOPE)
SET(SRCS_PROCESSOR ${SRCS_PROCESSOR} PARENT_SCOPE)
