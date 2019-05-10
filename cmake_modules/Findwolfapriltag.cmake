#edit the following line to add the librarie's header files
FIND_PATH(
    example_INCLUDE_DIRS
    # NAMES wolf.found
    PATHS /usr/local/include/iri-algorithms/wolf/example)
#change INCLUDE_DIRS to its parent directory
# get_filename_component(example_INCLUDE_DIRS ${example_INCLUDE_DIRS} DIRECTORY)
IF(example_INCLUDE_DIRS)
  MESSAGE("Found example include dirs: ${example_INCLUDE_DIRS}")
ELSE
  MESSAGE("Couldn't find example include dirs")
ENDIF

FIND_LIBRARY(
    example_LIBRARY
    NAMES libexample.so
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iri-algorithms) 
IF(example_LIBRARY)
  MESSAGE("Found example lib: ${example_LIBRARY}")
ELSE
  MESSAGE("Couldn't find example lib")
ENDIF
IF (example_INCLUDE_DIRS AND example_LIBRARY)
   SET(example_FOUND TRUE)
ENDIF (example_INCLUDE_DIRS AND example_LIBRARY)

IF (example_FOUND)
   IF (NOT example_FIND_QUIETLY)
      MESSAGE(STATUS "Found example: ${example_LIBRARY}")
   ENDIF (NOT example_FIND_QUIETLY)
ELSE (example_FOUND)
   IF (wolf_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find example")
   ENDIF (wolf_FIND_REQUIRED)
ENDIF (example_FOUND)

