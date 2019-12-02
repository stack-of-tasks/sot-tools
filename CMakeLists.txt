# Copyright 2012, CNRS
# Author: Florent Lamiraux
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-tools)
SET(PROJECT_DESCRIPTION "Miscellanous entities for the stack of tasks")
SET(PROJECT_URL "http://github.com/stack-of-tasks/sot-tools")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)

# Specify the project.
cmake_policy(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME}
  LANGUAGES
  CXX
  VERSION
  ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
  )

INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/header.cmake)
INCLUDE(cmake/pthread.cmake)

add_project_dependency (Boost REQUIRED
  python filesystem system thread program_options unit_test_framework)

#find_package (Eigen3 REQUIRED NO_MODULE)

#FINDPYTHON ()
#FIND_NUMPY ()
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

# Search for Boost.
SEARCH_FOR_PTHREAD()


SET(${PROJECT_NAME}_HEADERS
  src/cubic-interpolation.hh
  src/kinematic-planner.hh
  src/seqplay.hh
  src/simpleseqplay.hh)


INSTALL(FILES ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/sot/tools
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  )

# Add dependency to libsot-tools.so library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("sot-tools")

# Trigger dependency to dynamic-graph-python
ADD_PROJECT_DEPENDENCY(sot-core REQUIRED)

ADD_SUBDIRECTORY(src)
#ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_PACKAGE_FINALIZE()
