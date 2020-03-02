
# Copyright 2012, 2020 CNRS
# Author: Florent Lamiraux, Guilhem Saurel
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-tools)
SET(PROJECT_DESCRIPTION "Miscellanous entities for the stack of tasks")
SET(PROJECT_URL "http://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/eigen.cmake)

SET(CUSTOM_HEADER_DIR sot/tools)
SET(${PROJECT_NAME}_HEADERS)
SET(DOXYGEN_USE_MATHJAX YES)

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python binding" OFF)
OPTION(SUFFIX_SO_VERSION
  "Suffix shared library name by a string depending on git status of project"
  ON)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Add dependency to libsot-tools.so library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("sot-tools")

# Trigger dependency to dynamic-graph-python
ADD_PROJECT_DEPENDENCY(sot-core 4.8.1 REQUIRED PKG_CONFIG_REQUIRES "sot-core >= 4.8.1")

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON ()
  FIND_NUMPY ()
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED PKG_CONFIG_REQUIRES "dynamic-graph-python")
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
