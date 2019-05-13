#edit the following line to add the librarie's header files
FIND_PATH(
    wolfapriltag_INCLUDE_DIR
    NAMES apriltag.found
    PATHS /usr/local/include/iri-algorithms/wolf/plugin_apriltag)
IF(wolfapriltag_INCLUDE_DIR)
  MESSAGE("Found wolf apriltag include dirs: ${wolfapriltag_INCLUDE_DIR}")
ELSE(wolfapriltag_INCLUDE_DIR)
  MESSAGE("Couldn't find wolf apriltag include dirs")
ENDIF(wolfapriltag_INCLUDE_DIR)
FIND_LIBRARY(
    wolfapriltag_LIBRARY
    NAMES libwolfapriltag.so
    PATHS /usr/local/lib/iri-algorithms)
IF(wolfapriltag_LIBRARY)
  MESSAGE("Found wolf apriltag lib: ${wolfapriltag_LIBRARY}")
ELSE(wolfapriltag_LIBRARY)
  MESSAGE("Couldn't find wolf apriltag lib")
ENDIF(wolfapriltag_LIBRARY)

IF (wolfapriltag_INCLUDE_DIR AND wolfapriltag_LIBRARY)
   SET(wolfapriltag_FOUND TRUE)
 ELSE(wolfapriltag_INCLUDE_DIR AND wolfapriltag_LIBRARY)
   set(wolfapriltag_FOUND FALSE)
ENDIF (wolfapriltag_INCLUDE_DIR AND wolfapriltag_LIBRARY)

IF (wolfapriltag_FOUND)
   IF (NOT wolfapriltag_FIND_QUIETLY)
      MESSAGE(STATUS "Found wolf apriltag: ${wolfapriltag_LIBRARY}")
   ENDIF (NOT wolfapriltag_FIND_QUIETLY)
ELSE (wolfapriltag_FOUND)
   IF (wolfapriltag_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wolf apriltag")
   ENDIF (wolfapriltag_FIND_REQUIRED)
ENDIF (wolfapriltag_FOUND)


macro(wolf_report_not_found REASON_MSG)
  set(wolfapriltag_FOUND FALSE)
  unset(wolfapriltag_INCLUDE_DIR)
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