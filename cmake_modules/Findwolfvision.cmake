#edit the following line to add the librarie's header files
FIND_PATH(
    vision_INCLUDE_DIRS
    NAMES vision.found
    PATHS /usr/local/include/iri-algorithms/wolf/vision)
#change INCLUDE_DIRS to its parent directory
get_filename_component(vision_INCLUDE_DIRS ${vision_INCLUDE_DIRS} DIRECTORY)
IF(vision_INCLUDE_DIRS)
  MESSAGE("Found vision include dirs: ${vision_INCLUDE_DIRS}")
ELSE(vision_INCLUDE_DIRS)
  MESSAGE("Couldn't find vision include dirs")
ENDIF(vision_INCLUDE_DIRS)

FIND_LIBRARY(
    vision_LIBRARY
    NAMES libvision.so
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iri-algorithms) 
IF(vision_LIBRARY)
  MESSAGE("Found vision lib: ${vision_LIBRARY}")
ELSE(vision_LIBRARY)
  MESSAGE("Couldn't find vision lib")
ENDIF(vision_LIBRARY)
IF (vision_INCLUDE_DIRS AND vision_LIBRARY)
   SET(vision_FOUND TRUE)
ENDIF (vision_INCLUDE_DIRS AND vision_LIBRARY)

IF (vision_FOUND)
   IF (NOT vision_FIND_QUIETLY)
      MESSAGE(STATUS "Found vision: ${vision_LIBRARY}")
   ENDIF (NOT vision_FIND_QUIETLY)
ELSE (vision_FOUND)
   IF (wolf_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find vision")
   ENDIF (wolf_FIND_REQUIRED)
ENDIF (vision_FOUND)

