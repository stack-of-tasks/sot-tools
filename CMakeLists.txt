
# Copyright 2012, CNRS
# Author: Florent Lamiraux
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

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

CMAKE_POLICY(SET CMP0048 OLD)
PROJECT(${PROJECT_NAME} CXX)

# Add dependency to libsot-tools.so library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("sot-tools")

# Trigger dependency to dynamic-graph-python
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0")

# Search for dependencies.
SEARCH_FOR_EIGEN()

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON ()
  FIND_NUMPY ()
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
  ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
ENDIF(BUILD_PYTHON_INTERFACE)

ADD_SUBDIRECTORY(src)
