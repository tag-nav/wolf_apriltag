# This file was copied and adapted from the ceres_solver project
# http://ceres-solver.org/

# wolf - Windowed Localization Frames
# Copyright 2016
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"

# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: 
#

# Config file for wolf - Find wolf & dependencies.
#
# This file is used by CMake when find_package(wolf) is invoked and either
# the directory containing this file either is present in CMAKE_MODULE_PATH
# (if wolf was installed), or exists in the local CMake package registry if
# the wolf build directory was exported.
#
# This module defines the following variables:
#
# wolf_FOUND / wolf_FOUND: True if wolf has been successfully
#                            found. Both variables are set as although
#                            FindPackage() only references wolf_FOUND
#                            in Config mode, given the conventions for
#                            <package>_FOUND when FindPackage() is
#                            called in Module mode, users could
#                            reasonably expect to use wolf_FOUND
#                            instead.
#
# wolf_VERSION: Version of wolf found.
#
# wolf_INCLUDE_DIRS: Include directories for wolf and the
#                     dependencies which appear in the wolf public
#                     API and are thus required to use wolf.
#
# wolf_LIBRARIES: Libraries for wolf and all
#                  dependencies against which wolf was
#                  compiled. This will not include any optional
#                  dependencies that were disabled when wolf was
#                  compiled.
#
# The following variables are also defined for legacy compatibility
# only.  Any new code should not use them as they do not conform to
# the standard CMake FindPackage naming conventions.
#
# wolf_INCLUDES = ${wolf_INCLUDE_DIRS}.

# Called if we failed to find Ceres or any of its required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(wolf_report_not_found REASON_MSG)
  # FindPackage() only references Ceres_FOUND, and requires it to be
  # explicitly set FALSE to denote not found (not merely undefined).
  set(wolf_FOUND FALSE)
  set(wolf_FOUND FALSE)
  unset(wolf_INCLUDE_DIRS)
  unset(wolf_LIBRARIES)

  # Reset the CMake module path to its state when this script was called.
  set(CMAKE_MODULE_PATH ${CALLERS_CMAKE_MODULE_PATH})

  # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by
  # FindPackage() use the camelcase library name, not uppercase.
  if (wolf_FIND_QUIETLY)
    message(STATUS "Failed to find wolf - " ${REASON_MSG} ${ARGN})
  else (wolf_FIND_REQUIRED)
    message(FATAL_ERROR "Failed to find wolf - " ${REASON_MSG} ${ARGN})
  else()
    # Neither QUIETLY nor REQUIRED, use SEND_ERROR which emits an error
    # that prevents generation, but continues configuration.
    message(SEND_ERROR "Failed to find wolf - " ${REASON_MSG} ${ARGN})
  endif ()
  return()
endmacro(wolf_report_not_found)

# Record the state of the CMake module path when this script was
# called so that we can ensure that we leave it in the same state on
# exit as it was on entry, but modify it locally.
set(CALLERS_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

# Get the (current, i.e. installed) directory containing this file.
get_filename_component(wolf_CURRENT_CONFIG_DIR
  "${CMAKE_CURRENT_LIST_FILE}" PATH)

# Reset CMake module path to the installation directory of this
# script, thus we will use the FindPackage() scripts shipped with
# wolf to find wolf' dependencies, even if the user has equivalently
# named FindPackage() scripts in their project.
set(CMAKE_MODULE_PATH ${wolf_CURRENT_CONFIG_DIR})

# Build the absolute root install directory as a relative path
# (determined when wolf was configured & built) from the current
# install directory for this this file.  This allows for the install
# tree to be relocated, after wolf was built, outside of CMake.
get_filename_component(CURRENT_ROOT_INSTALL_DIR
  ${wolf_CURRENT_CONFIG_DIR}/../../../
  ABSOLUTE)
if (NOT EXISTS ${CURRENT_ROOT_INSTALL_DIR})
  wolf_report_not_found(
    "wolf install root: ${CURRENT_ROOT_INSTALL_DIR}, "
    "determined from relative path from wolfConfig.cmake install location: "
    "${wolf_CURRENT_CONFIG_DIR}, does not exist. Either the install "
    "directory was deleted, or the install tree was only partially relocated "
    "outside of CMake after wolf was built.")
endif (NOT EXISTS ${CURRENT_ROOT_INSTALL_DIR})

# Set the include directories for wolf (itself).
set(wolf_INCLUDE_DIR "${CURRENT_ROOT_INSTALL_DIR}/include/iri-algorithms")

# if (NOT EXISTS ${wolf_INCLUDE_DIR}/wolf/wolf.h)
if (NOT EXISTS ${wolf_INCLUDE_DIR}/wolf)
  wolf_report_not_found(
    "wolf install root: ${CURRENT_ROOT_INSTALL_DIR}, "
    "determined from relative path from wolfConfig.cmake install location: "
    "${wolf_CURRENT_CONFIG_DIR}, does not contain wolf headers. "
    "Either the install directory was deleted, or the install tree was only "
    "partially relocated outside of CMake after wolf was built.")
# endif (NOT EXISTS ${wolf_INCLUDE_DIR}/wolf/wolf.h)
endif (NOT EXISTS ${wolf_INCLUDE_DIR}/wolf)
list(APPEND wolf_INCLUDE_DIRS ${wolf_INCLUDE_DIR}/wolf)

# Set the version.
set(wolf_VERSION 0.0.1)

# Eigen.

# Flag set during configuration and build of wolf.
set(wolf_EIGEN_VERSION @EIGEN_VERSION@)
# Append the locations of Eigen when wolf was built to the search path hints.
list(APPEND EIGEN_INCLUDE_DIR_HINTS /usr/include/eigen3)
# Search quietly to control the timing of the error message if not found. The
# search should be for an exact match, but for usability reasons do a soft
# match and reject with an explanation below.

find_package(Eigen3 ${wolf_EIGEN_VERSION} QUIET)
# Flag set with currently found Eigen version.
set(EIGEN_VERSION @EIGEN_VERSION@)
if (EIGEN3_FOUND)
  if (NOT EIGEN_VERSION VERSION_EQUAL wolf_EIGEN_VERSION)
    # CMake's VERSION check in FIND_PACKAGE() will accept any version >= the
    # specified version. However, only version = is supported. Improve
    # usability by explaining why we don't accept non-exact version matching.
    wolf_report_not_found("Found Eigen dependency, but the version of Eigen "
      "found (${EIGEN_VERSION}) does not exactly match the version of Eigen "
      "wolf was compiled with (${wolf_EIGEN_VERSION}). This can cause subtle "
      "bugs by triggering violations of the One Definition Rule. See the "
      "Wikipedia article http://en.wikipedia.org/wiki/One_Definition_Rule "
      "for more details")
  endif ()
  message(STATUS "Found required wolf dependency: "
    "Eigen version ${wolf_EIGEN_VERSION} in ${EIGEN3_INCLUDE_DIR}")
else (EIGEN3_FOUND)
  wolf_report_not_found("Missing required wolf "
    "dependency: Eigen version ${wolf_EIGEN_VERSION}, please set "
    "EIGEN3_INCLUDE_DIR.")
endif (EIGEN3_FOUND)
list(APPEND wolf_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# Import exported wolf targets, if they have not already been imported.
if (NOT TARGET wolf AND NOT wolf_BINARY_DIR)
  include(${wolf_CURRENT_CONFIG_DIR}/wolfTargets.cmake)
endif (NOT TARGET wolf AND NOT wolf_BINARY_DIR)
# Set the expected XX_LIBRARIES variable for FindPackage().
set(wolf_LIBRARIES wolf)

# Set legacy library variable for backwards compatibility.
set(wolf_LIBRARY ${wolf_LIBRARIES})

# Make user aware of any compile flags that will be added to their targets
# which use wolf (i.e. flags exported in the wolf target).  Only CMake
# versions >= 2.8.12 support target_compile_options().
if (TARGET ${wolf_LIBRARIES} AND
    NOT CMAKE_VERSION VERSION_LESS "2.8.12")
  get_target_property(wolf_INTERFACE_COMPILE_OPTIONS
    ${wolf_LIBRARIES} INTERFACE_COMPILE_OPTIONS)

  set(wolf_LOCATION "${CURRENT_ROOT_INSTALL_DIR}")

  # Check for -std=c++11 flags.
  if (wolf_INTERFACE_COMPILE_OPTIONS MATCHES ".*std=c\\+\\+11.*")
    message(STATUS "wolf version ${wolf_VERSION} detected here: "
      "${wolf_LOCATION} was built with C++11. wolf target will add "
      "C++11 flags to compile options for targets using it.")
  endif()
endif()

# Reset CMake module path to its state when this script was called.
set(CMAKE_MODULE_PATH ${CALLERS_CMAKE_MODULE_PATH})

# As we use wolf_REPORT_NOT_FOUND() to abort, if we reach this point we have
# found wolf and all required dependencies.
message(STATUS "Found wolf version: ${wolf_VERSION} installed in: ${CURRENT_ROOT_INSTALL_DIR}")

# Set wolf_FOUND to be equivalent to wolf_FOUND, which is set to
# TRUE by FindPackage() if this file is found and run, and after which
# wolf_FOUND is not (explicitly, i.e. undefined does not count) set
# to FALSE.
set(wolf_FOUND TRUE)
