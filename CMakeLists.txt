# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# MAC OSX RPATH
SET(CMAKE_MACOSX_RPATH true)

# The project name
PROJECT(apriltag)
set(PLUGIN_NAME wolf${PROJECT_NAME})

MESSAGE("Starting ${PROJECT_NAME} CMakeLists ...")

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(INCLUDE_INSTALL_DIR include/wolf)
set(LIB_INSTALL_DIR lib/)

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "DEBUG")
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++14 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++14 support.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

if(UNIX)
  # GCC is not strict enough by default, so enable most of the warnings.
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Werror=all -Werror=extra -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter -Wno-missing-field-initializers")
endif(UNIX)

IF(NOT BUILD_TESTS)
  OPTION(BUILD_TESTS "Build Unit tests" ON)
ENDIF(NOT BUILD_TESTS)

IF(NOT BUILD_DEMOS)
  OPTION(BUILD_DEMOS "Build Demos" OFF)
ENDIF(NOT BUILD_DEMOS)

IF(NOT BUILD_DOC)
  OPTION(BUILD_DOC "Build Documentation" OFF)
ENDIF(NOT BUILD_DOC)

#############
## Testing ##
#############
#
if(BUILD_TESTS)
    # Enables testing for this directory and below.
    # Note that ctest expects to find a test file in the build directory root.
    # Therefore, this command should be in the source directory root.
    #include(CTest) # according to http://public.kitware.com/pipermail/cmake/2012-June/050853.html
    enable_testing()
endif()

# Some wolf compilation options
IF((CMAKE_BUILD_TYPE MATCHES DEBUG) OR (CMAKE_BUILD_TYPE MATCHES debug) OR (CMAKE_BUILD_TYPE MATCHES Debug))
  set(_WOLF_DEBUG true)
ENDIF()

option(_WOLF_TRACE "Enable wolf tracing macro" ON)


# ============ DEPENDENCIES ============ 
FIND_PACKAGE(wolfcore REQUIRED CONFIG)
FIND_PACKAGE(wolfvision REQUIRED)
FIND_PACKAGE(apriltag REQUIRED)  # UMichigan apriltag library 

 #============ CONFIG.H ============  
set(_WOLF_ROOT_DIR ${CMAKE_SOURCE_DIR})
# variable used to compile the config.h.in file
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

# Define the directory where will be the configured config.h
SET(WOLF_CONFIG_DIR ${PROJECT_BINARY_DIR}/conf/${PROJECT_NAME}/internal)

# Create the specified output directory if it does not exist.
IF(NOT EXISTS "${WOLF_CONFIG_DIR}")
  message(STATUS "Creating config output directory: ${WOLF_CONFIG_DIR}")
  file(MAKE_DIRECTORY "${WOLF_CONFIG_DIR}")
ENDIF()
IF(EXISTS "${WOLF_CONFIG_DIR}" AND NOT IS_DIRECTORY "${WOLF_CONFIG_DIR}")
  message(FATAL_ERROR "Bug: Specified CONFIG_DIR: "
    "${WOLF_CONFIG_DIR} exists, but is not a directory.")
ENDIF()

# Configure config.h
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/internal/config.h.in "${WOLF_CONFIG_DIR}/config.h")


# ============ HEADERS ============ 
SET(HDRS_FACTOR
include/${PROJECT_NAME}/factor/factor_apriltag_proj.h
  )
SET(HDRS_FEATURE
include/${PROJECT_NAME}/feature/feature_apriltag.h
include/${PROJECT_NAME}/feature/feature_apriltag_pose.h
include/${PROJECT_NAME}/feature/feature_apriltag_proj.h
  )
SET(HDRS_LANDMARK
include/${PROJECT_NAME}/landmark/landmark_apriltag.h
  )
SET(HDRS_PROCESSOR
include/${PROJECT_NAME}/processor/processor_tracker_landmark_apriltag.h
include/${PROJECT_NAME}/processor/ippe.h
  )

# ============ SOURCES ============ 
SET(SRCS_FEATURE
  )
SET(SRCS_LANDMARK
src/landmark/landmark_apriltag.cpp
  )
SET(SRCS_PROCESSOR
src/processor/processor_tracker_landmark_apriltag.cpp
src/processor/ippe.cpp
  )
SET(SRCS_YAML
src/yaml/processor_tracker_landmark_apriltag_yaml.cpp
  )

# create the shared library
ADD_LIBRARY(${PLUGIN_NAME}
  SHARED
  ${SRCS_FEATURE}
  ${SRCS_LANDMARK}
  ${SRCS_PROCESSOR}
  ${SRCS_YAML}
  )
  
# Set compiler options
# ====================
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  message(STATUS "Using C++ compiler clang")
  target_compile_options(${PLUGIN_NAME} PRIVATE -Winconsistent-missing-override)
  # using Clang
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  message(STATUS "Using C++ compiler gnu")
  target_compile_options(${PLUGIN_NAME} PRIVATE -Wsuggest-override)
  # using GCC
endif()

#Link the created libraries
#===============EXAMPLE=========================
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} wolfcore)
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} wolfvision)
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} apriltag)

#Build tests
#===============EXAMPLE=========================
IF(BUILD_TESTS)
  MESSAGE("Building tests.")
  add_subdirectory(test)
ENDIF(BUILD_TESTS)


#install library
#=============================================================
INSTALL(TARGETS ${PLUGIN_NAME} EXPORT ${PLUGIN_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
)
install(EXPORT ${PLUGIN_NAME}Targets DESTINATION lib/${PLUGIN_NAME}/cmake)


# Configure the package installation
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake_modules/${PLUGIN_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}Config.cmake
  INSTALL_DESTINATION ${LIB_INSTALL_DIR}/${PLUGIN_NAME}/cmake
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
)

install(
  FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}Config.cmake
  DESTINATION 
  ${LIB_INSTALL_DIR}/${PLUGIN_NAME}/cmake
)

target_include_directories(${PLUGIN_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/conf>
  $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)


#install headers
INSTALL(FILES ${HDRS_FACTOR}
  DESTINATION ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/factor)
INSTALL(FILES ${HDRS_FEATURE}
  DESTINATION ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/feature)
INSTALL(FILES ${HDRS_LANDMARK}
  DESTINATION ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/landmark)
INSTALL(FILES ${HDRS_PROCESSOR}
  DESTINATION ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/processor)

INSTALL(FILES "${WOLF_CONFIG_DIR}/config.h"
  DESTINATION ${INCLUDE_INSTALL_DIR}/${PROJECT_NAME}/internal)

export(PACKAGE ${PLUGIN_NAME})

FIND_PACKAGE(Doxygen MODULE)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)
