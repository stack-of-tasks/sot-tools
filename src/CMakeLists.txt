
# Copyright 2012, 2020, CNRS
# Author: Florent Lamiraux
# Author: Guilhem Saurel
#

IF(BUILD_PYTHON_INTERFACE)
  SET(NO_INSTALL_OF_INIT_PY 0)
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/tools" ${PROJECT_NAME} wrap
    ${NO_INSTALL_OF_INIT_PY})

  SET(${PROJECT_NAME}_PYTHON
    __init__.py
    se3.py
    quaternion.py
    )

  FOREACH(py_file ${${PY_NAME}_PYTHON})
    PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/toolss" ${py_file})
  ENDFOREACH(py_file ${${PY_NAME}_PYTHON})

ENDIF(BUILD_PYTHON_INTERFACE)
