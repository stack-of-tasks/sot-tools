
# Copyright 2012, 2020 CNRS
# Author: Florent Lamiraux, Guilhem Saurel
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-tools)
SET(PROJECT_DESCRIPTION "Miscellanous entities for the stack of tasks")
SET(PROJECT_URL "http://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python binding" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix shared library name by a string depending on git status of project" ON)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/eigen.cmake)

SET(DOXYGEN_USE_MATHJAX YES)
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

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

ADD_SUBDIRECTORY(include/sot/tools)
ADD_SUBDIRECTORY(src)
