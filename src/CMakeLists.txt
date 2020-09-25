# Copyright 2012, 2020, CNRS
# Author: Florent Lamiraux
# Author: Guilhem Saurel
#

IF(BUILD_PYTHON_INTERFACE)
  DYNAMIC_GRAPH_PYTHON_MODULE(${PYTHON_DIR} ${PROJECT_NAME} wrap
    SOURCE_PYTHON_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/python-module.cc
    )

  SET(${PROJECT_NAME}_PYTHON
    se3.py
    quaternion.py
    )

  FOREACH(file ${${PROJECT_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/${PYTHON_DIR}" ${file})
  ENDFOREACH(file ${${PROJECT_NAME}_PYTHON})
ENDIF(BUILD_PYTHON_INTERFACE)
