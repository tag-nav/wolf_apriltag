# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
# MAC OSX RPATH
SET(CMAKE_MACOSX_RPATH 1)


# The project name
PROJECT(apriltag)
set(PLUGIN_NAME wolf${PROJECT_NAME})


SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX /usr/local)

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "DEBUG")
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
		message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if(UNIX)
  # GCC is not strict enough by default, so enable most of the warnings.
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Werror=all -Werror=extra -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-parameter -Wno-missing-field-initializers")
endif(UNIX)


#OPTION(BUILD_DOC "Build Documentation" OFF)
OPTION(BUILD_TESTS "Build Unit tests" ON)
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

MESSAGE("Starting ${PROJECT_NAME} CMakeLists ...")
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#CMAKE modules

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules")
MESSAGE(STATUS ${CMAKE_MODULE_PATH})

# Some wolf compilation options

IF((CMAKE_BUILD_TYPE MATCHES DEBUG) OR (CMAKE_BUILD_TYPE MATCHES debug) OR (CMAKE_BUILD_TYPE MATCHES Debug))
  set(_WOLF_DEBUG true)
ENDIF()

option(_WOLF_TRACE "Enable wolf tracing macro" ON)

# option(BUILD_EXAMPLES "Build examples" OFF)
set(BUILD_TESTS true)

# Does this has any other interest
# but for the examples ?
# yes, for the tests !
IF(BUILD_EXAMPLES OR BUILD_TESTS)
  string(TOUPPER ${PROJECT_NAME} UPPER_NAME)
  set(_WOLF_ROOT_DIR ${CMAKE_SOURCE_DIR})
ENDIF(BUILD_EXAMPLES OR BUILD_TESTS)


#find dependencies.
# ============EXAMPLE==================
FIND_PACKAGE(Eigen3 3.2.92 REQUIRED)

FIND_PACKAGE(Threads REQUIRED)

FIND_PACKAGE(wolf REQUIRED)

FIND_PACKAGE(vision_utils REQUIRED)

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(wolfvision REQUIRED)

FIND_PACKAGE(Ceres QUIET) #Ceres is not required

FIND_PACKAGE(YamlCpp REQUIRED)

FIND_PATH(APRILTAG_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
FIND_LIBRARY(APRILTAGL_LIBRARY NAMES apriltag PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}" "apriltag" ${APRILTAGL_LIBRARY_PATH})

IF(APRILTAG_LIBRARY)
    SET(Apriltag_FOUND TRUE)
    MESSAGE("apriltag Library FOUND in ${APRILTAGL_LIBRARY}: apriltag related sources will be built.")
ENDIF(APRILTAG_LIBRARY)
IF(Ceres_FOUND)
    MESSAGE("Ceres Library FOUND: Ceres related sources will be built.")
ENDIF(Ceres_FOUND)

FIND_PATH(APRILTAG_INCLUDE_DIR NAMES apriltag.h PATH_SUFFIXES "apriltag" ${APRILTAG_INCLUDE_PATH})
FIND_LIBRARY(APRILTAG_LIBRARY NAMES apriltag PATH_SUFFIXES "${CMAKE_LIBRARY_ARCHITECTURE}" "apriltag" ${APRILTAG_LIBRARY_PATH})

IF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library FOUND: yaml-cpp related sources will be built.")
ELSEIF(YAMLCPP_FOUND)
    MESSAGE("yaml-cpp Library NOT FOUND!")
ENDIF(YAMLCPP_FOUND)

FIND_PATH(
    Suitesparse_INCLUDE_DIRS
    NAMES SuiteSparse_config.h
    PATHS /usr/include/suitesparse /usr/local/include/suitesparse)
MESSAGE("Found suitesparse_INCLUDE_DIRS:" ${Suitesparse_INCLUDE_DIRS})

IF(Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND TRUE)
   MESSAGE("Suitesparse FOUND: wolf_solver will be built.")
ELSE (Suitesparse_INCLUDE_DIRS)
   SET(Suitesparse_FOUND FALSE)
   MESSAGE(FATAL_ERROR "Suitesparse NOT FOUND")
ENDIF (Suitesparse_INCLUDE_DIRS)

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
message("CONFIG DIRECTORY ${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_BINARY_DIR}/conf")

#INCLUDES SECTION
# ============EXAMPLE==================
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${wolf_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${wolfvision_INCLUDE_DIR})
include_directories(BEFORE "include")
INCLUDE_DIRECTORIES(${APRILTAG_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${vision_utils_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YAMLCPP_INCLUDE_DIR})
IF(Ceres_FOUND)
    INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
ENDIF(Ceres_FOUND)



#HEADERS
SET(HDRS_COMMON
  )
SET(HDRS_MATH
  )
SET(HDRS_UTILS
  )
SET(HDRS_CAPTURE
  )
SET(HDRS_FACTOR
include/apriltag/factor/factor_autodiff_apriltag.h
  )
SET(HDRS_FEATURE
include/apriltag/feature/feature_apriltag.h
  )
SET(HDRS_LANDMARK
include/apriltag/landmark/landmark_apriltag.h
  )
SET(HDRS_PROCESSOR
include/apriltag/processor/processor_tracker_landmark_apriltag.h
include/apriltag/processor/ippe.h
  )
SET(HDRS_SENSOR
  )
SET(HDRS_SOLVER
  )
SET(HDRS_DTASSC
  )

#SOURCES

SET(SRCS_COMMON
  )
SET(SRCS_MATH
  )
SET(SRCS_UTILS
  )
SET(SRCS_CAPTURE
  )
SET(SRCS_FACTOR
  )
SET(SRCS_FEATURE
src/feature/feature_apriltag.cpp
  )
SET(SRCS_LANDMARK
src/landmark/landmark_apriltag.cpp
  )
SET(SRCS_PROCESSOR
src/processor/processor_tracker_landmark_apriltag.cpp
src/processor/ippe.cpp
  )
SET(SRCS_SENSOR
  )
SET(SRCS_DTASSC
  )
SET(SRCS_SOLVER
  )
SET(SRCS_YAML
src/yaml/processor_tracker_landmark_apriltag_yaml.cpp
  )
#OPTIONALS
#optional HDRS and SRCS
# ==================EXAMPLE===============
# IF (Ceres_FOUND)
#     SET(HDRS_WRAPPER
#       include/base/solver_suitesparse/sparse_utils.h
#       include/base/solver/solver_manager.h
#       include/base/ceres_wrapper/ceres_manager.h
#       include/base/ceres_wrapper/cost_function_wrapper.h
#       include/base/ceres_wrapper/create_numeric_diff_cost_function.h
#       include/base/ceres_wrapper/local_parametrization_wrapper.h 
#       )
#     SET(SRCS_WRAPPER
#       src/solver/solver_manager.cpp
#       src/ceres_wrapper/ceres_manager.cpp
#       src/ceres_wrapper/local_parametrization_wrapper.cpp 
#       )
# ELSE(Ceres_FOUND)
#   SET(HDRS_WRAPPER)
#   SET(SRCS_WRAPPER)
# ENDIF(Ceres_FOUND)

# create the shared library
ADD_LIBRARY(${PLUGIN_NAME}
  SHARED
  ${SRCS_CAPTURE}
  ${SRCS_COMMON}
  ${SRCS_DTASSC}
  ${SRCS_FACTOR}
  ${SRCS_FEATURE}
  ${SRCS_LANDMARK}
  ${SRCS_MATH}
  ${SRCS_PROCESSOR}
  ${SRCS_SENSOR}
  ${SRCS_SOLVER}
  ${SRCS_UTILS}
  ${SRCS_WRAPPER}
  ${SRCS_YAML}
  )
# set(wolf_vision_LIBRARY "/usr/local/lib/iri-algorithms/libvision.so")
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${wolf_LIBRARY} ${wolfvision_LIBRARY} ${vision_utils_LIBRARY} ${APRILTAGL_LIBRARY}  ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS})
TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${YAMLCPP_LIBRARY})

#Link the created libraries
#===============EXAMPLE=========================
# IF (Ceres_FOUND)
#     TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${CERES_LIBRARIES})
# ENDIF(Ceres_FOUND)


#Build tests
#===============EXAMPLE=========================
IF (GLOG_FOUND)
    TARGET_LINK_LIBRARIES(${PLUGIN_NAME} ${GLOG_LIBRARY})
ENDIF (GLOG_FOUND)
IF (GLOG_FOUND)
    IF(BUILD_TESTS)
        MESSAGE("Building tests.")
        add_subdirectory(test)
    ENDIF(BUILD_TESTS)
ENDIF (GLOG_FOUND)

#install library

#=============================================================
INSTALL(TARGETS ${PLUGIN_NAME} EXPORT ${PLUGIN_NAME}Targets
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib/iri-algorithms
      ARCHIVE DESTINATION lib/iri-algorithms)

install(EXPORT ${PLUGIN_NAME}Targets DESTINATION lib/cmake/${PLUGIN_NAME})
#install headers
INSTALL(FILES ${HDRS_COMMON}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/common)
INSTALL(FILES ${HDRS_MATH}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/math)
INSTALL(FILES ${HDRS_UTILS}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/utils)
INSTALL(FILES ${HDRS_DTASSC}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/association)
INSTALL(FILES ${HDRS_CAPTURE}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/capture)
INSTALL(FILES ${HDRS_FACTOR}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/factor)
INSTALL(FILES ${HDRS_FEATURE}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/feature)
INSTALL(FILES ${HDRS_SENSOR}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/sensor)
INSTALL(FILES ${HDRS_PROCESSOR}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/processor)
INSTALL(FILES ${HDRS_LANDMARK}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/landmark)
INSTALL(FILES ${HDRS_WRAPPER}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/ceres_wrapper)
INSTALL(FILES ${HDRS_SOLVER_SUITESPARSE}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/solver_suitesparse)
INSTALL(FILES ${HDRS_SOLVER}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/solver)
INSTALL(FILES ${HDRS_SERIALIZATION}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/serialization)
INSTALL(FILES ${HDRS_YAML}
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/yaml)

FILE(WRITE ${PROJECT_NAME}.found "")
INSTALL(FILES ${PROJECT_NAME}.found
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME})

INSTALL(FILES "${WOLF_CONFIG_DIR}/config.h"
  DESTINATION include/iri-algorithms/wolf/plugin_${PROJECT_NAME}/${PROJECT_NAME}/internal)

INSTALL(FILES "${CMAKE_SOURCE_DIR}/cmake_modules/${PLUGIN_NAME}Config.cmake" DESTINATION "lib/cmake/${PLUGIN_NAME}")

INSTALL(DIRECTORY ${SPDLOG_INCLUDE_DIRS} DESTINATION "include/iri-algorithms/")

export(PACKAGE ${PLUGIN_NAME})

FIND_PACKAGE(Doxygen)

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
