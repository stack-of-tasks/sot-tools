# Copyright 2012, 2020 CNRS Author: Florent Lamiraux, Guilhem Saurel
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-tools)
set(PROJECT_DESCRIPTION "Miscellanous entities for the stack of tasks")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "sot/tools")
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PYTHON_DIR ${CUSTOM_HEADER_DIR})
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(sot-core REQUIRED)

find_package(
  Boost
  COMPONENTS unit_test_framework
  REQUIRED)

# Verbosity level
if(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))
  add_definitions(-DVP_DEBUG_MODE=${CMAKE_VERBOSITY_LEVEL} -DVP_DEBUG)
endif(NOT (\"${CMAKE_VERBOSITY_LEVEL}\" STREQUAL \"\"))

# Main Library
set(${PROJECT_NAME}_HEADERS
    include/${CUSTOM_HEADER_DIR}/cubic-interpolation.hh
    include/${CUSTOM_HEADER_DIR}/cubic-interpolation-se3.hh
    include/${CUSTOM_HEADER_DIR}/kinematic-planner.hh
    include/${CUSTOM_HEADER_DIR}/oscillator.hh
    include/${CUSTOM_HEADER_DIR}/seqplay.hh
    include/${CUSTOM_HEADER_DIR}/simpleseqplay.hh)

set(${PROJECT_NAME}_SOURCES
    src/cubic-interpolation.cc
    # src/kinematic-planner.cc
    src/cubic-interpolation-se3.cc
    src/oscillator.cc
    src/seqplay.cc
    src/simpleseqplay.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} sot-core::sot-core)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
add_subdirectory(tests)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
