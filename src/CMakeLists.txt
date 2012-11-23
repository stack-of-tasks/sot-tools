
# Copyright 2012, CNRS
# Author: Florent Lamiraux
#

SET (LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY (${LIBRARY_NAME} SHARED
  cubic-interpolation.cc
  cubic-interpolation.hh
  cubic-interpolation-se3.cc
  cubic-interpolation-se3.hh
  oscillator.cc
  )

PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} sot-core)
DYNAMIC_GRAPH_PYTHON_MODULE ("sot/tools" ${LIBRARY_NAME} wrap)
INSTALL (TARGETS ${LIBRARY_NAME} DESTINATION lib)
