
# Copyright 2012, CNRS
# Author: Florent Lamiraux
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/python.cmake)

FINDPYTHON ()
FIND_NUMPY ()

SET(PROJECT_NAME sot-tools)
SET(PROJECT_DESCRIPTION "Miscellanous entities for the stack of tasks")
SET(PROJECT_URL "http://github.com/laas/sot-tools")

SET(CUSTOM_HEADER_DIR sot/tools)
SET(${PROJECT_NAME}_HEADERS
)

SETUP_PROJECT()
# Trigger dependency to dynamic-graph-python
ADD_REQUIRED_DEPENDENCY("sot-core >= 2.5")
# Add dependency to libsot-tools.so library in pkg-config file.
PKG_CONFIG_APPEND_LIBS("sot-tools")

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
