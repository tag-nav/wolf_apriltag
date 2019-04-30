#edit the following line to add the librarie's header files
FIND_PATH(
    wolf_INCLUDE_DIRS
    NAMES wolf.found
    PATHS /usr/local/include/iri-algorithms/wolf/plugin_core)
#change INCLUDE_DIRS to its parent directory
# get_filename_component(wolf_INCLUDE_DIRS ${wolf_INCLUDE_DIRS} DIRECTORY)
MESSAGE("Found wolf include dirs: ${wolf_INCLUDE_DIRS}")

FIND_LIBRARY(
    wolf_LIBRARY
    NAMES wolf
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iri-algorithms) 

IF (wolf_INCLUDE_DIRS AND wolf_LIBRARY)
   SET(wolf_FOUND TRUE)
ENDIF (wolf_INCLUDE_DIRS AND wolf_LIBRARY)

IF (wolf_FOUND)
   IF (NOT wolf_FIND_QUIETLY)
      MESSAGE(STATUS "Found wolf: ${wolf_LIBRARY}")
   ENDIF (NOT wolf_FIND_QUIETLY)
ELSE (wolf_FOUND)
   IF (wolf_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find wolf")
   ENDIF (wolf_FIND_REQUIRED)
ENDIF (wolf_FOUND)

