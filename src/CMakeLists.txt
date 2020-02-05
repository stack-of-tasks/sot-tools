
# Copyright 2012, CNRS
# Author: Florent Lamiraux
#

SET (LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY (${LIBRARY_NAME} SHARED
  cubic-interpolation.cc
  cubic-interpolation.hh
  #kinematic-planner.cc
  #kinematic-planner.hh
  cubic-interpolation-se3.cc
  cubic-interpolation-se3.hh
  oscillator.cc
  seqplay.cc
  seqplay.hh
  simpleseqplay.cc
  simpleseqplay.hh
  )

PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} sot-core)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${LIBRARY_NAME}
    PROPERTIES SOVERSION ${PROJECT_VERSION}
    )
ENDIF()

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL (TARGETS ${LIBRARY_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

IF(BUILD_PYTHON_INTERFACE)
  DYNAMIC_GRAPH_PYTHON_MODULE ("sot/tools" ${LIBRARY_NAME} wrap)
  INSTALL(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/tools/se3.py
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/tools/quaternion.py
    DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/tools
    )
ENDIF(BUILD_PYTHON_INTERFACE)
