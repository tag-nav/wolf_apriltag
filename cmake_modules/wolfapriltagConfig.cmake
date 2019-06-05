#edit the following line to add the librarie's header files
FIND_PATH(
    wolfapriltag_INCLUDE_DIRS
    NAMES apriltag.found
    PATHS /usr/local/include/iri-algorithms/wolf/plugin_apriltag)
IF(wolfapriltag_INCLUDE_DIRS)
  MESSAGE("Found wolf apriltag include dirs: ${wolfapriltag_INCLUDE_DIRS}")
ELSE(wolfapriltag_INCLUDE_DIRS)
  MESSAGE("Couldn't find wolf apriltag include dirs")
ENDIF(wolfapriltag_INCLUDE_DIRS)
FIND_LIBRARY(
    wolfapriltag_LIBRARIES
    NAMES libwolfapriltag.so libwolfapriltag.dylib
    PATHS /usr/local/lib/iri-algorithms)
IF(wolfapriltag_LIBRARIES)
  MESSAGE("Found wolf apriltag lib: ${wolfapriltag_LIBRARIES}")
ELSE(wolfapriltag_LIBRARIES)
  MESSAGE("Couldn't find wolf apriltag lib")
ENDIF(wolfapriltag_LIBRARIES)

IF (wolfapriltag_INCLUDE_DIRS AND wolfapriltag_LIBRARIES)
   SET(wolfapriltag_FOUND TRUE)
 ELSE(wolfapriltag_INCLUDE_DIRS AND wolfapriltag_LIBRARIES)
   set(wolfapriltag_FOUND FALSE)
ENDIF (wolfapriltag_INCLUDE_DIRS AND wolfapriltag_LIBRARIES)

IF (wolfapriltag_FOUND)
   IF (NOT wolfapriltag_FIND_QUIETLY)
      MESSAGE(STATUS "Found wolf apriltag: ${wolfapriltag_LIBRARIES}")
   ENDIF (NOT wolfapriltag_FIND_QUIETLY)
ELSE (wolfapriltag_FOUND)
   IF (wolfapriltag_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wolf apriltag")
   ENDIF (wolfapriltag_FIND_REQUIRED)
ENDIF (wolfapriltag_FOUND)


macro(wolf_report_not_found REASON_MSG)
  set(wolfapriltag_FOUND FALSE)
  unset(wolfapriltag_INCLUDE_DIRS)
  unset(wolfapriltag_LIBRARIES)

  # Reset the CMake module path to its state when this script was called.
  set(CMAKE_MODULE_PATH ${CALLERS_CMAKE_MODULE_PATH})

  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by
  # FindPackage() use the camelcase library name, not uppercase.
  if (wolfapriltag_FIND_QUIETLY)
    message(STATUS "Failed to find wolf apriltag- " ${REASON_MSG} ${ARGN})
  else (wolfapriltag_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find wolf apriltag - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use SEND_ERROR which emits an error
    # that prevents generation, but continues configuration.
    message(SEND_ERROR "Failed to find wolf apriltag - " ${REASON_MSG} ${ARGN})
  endif ()
  return()
endmacro(wolf_report_not_found)

if(NOT wolfapriltag_FOUND)
  wolf_report_not_found("Something went wrong while setting up wolf apriltag.")
endif(NOT wolfapriltag_FOUND)
# Set the include directories for wolf (itself).
set(wolfapriltag_FOUND TRUE)

# Now we gather all the required dependencies for Wolf Laser

FIND_PACKAGE(wolfvision REQUIRED)
list(APPEND wolfapriltag_INCLUDE_DIRS ${wolfvision_INCLUDE_DIR})
list(APPEND wolfapriltag_LIBRARIES ${wolfvision_LIBRARY})

if(NOT OpenCV_FOUND)
  FIND_PACKAGE(OpenCV REQUIRED)
  list(APPEND wolfapriltag_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
  list(APPEND wolfapriltag_LIBRARIES ${OpenCV_LIBS})
endif()

#Making sure wolf is looked for
if(NOT wolf_FOUND)
  FIND_PACKAGE(wolf REQUIRED)

  #We reverse in order to insert at the start
  list(REVERSE wolfapriltag_INCLUDE_DIRS)
  list(APPEND wolfapriltag_INCLUDE_DIRS ${wolf_INCLUDE_DIRS})
  list(REVERSE wolfapriltag_INCLUDE_DIRS)

  list(REVERSE wolfapriltag_LIBRARIES)
  list(APPEND wolfapriltag_LIBRARIES ${wolf_LIBRARIES})
  list(REVERSE wolfapriltag_LIBRARIES)
endif()