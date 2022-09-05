# Copyright 2012, 2020, CNRS Author: Florent Lamiraux Author: Guilhem Saurel
#

if(BUILD_PYTHON_INTERFACE)
  dynamic_graph_python_module(
    ${PYTHON_DIR} ${PROJECT_NAME} wrap SOURCE_PYTHON_MODULE
    ${CMAKE_CURRENT_SOURCE_DIR}/python-module.cc)

  set(${PROJECT_NAME}_PYTHON se3.py quaternion.py)

  foreach(file ${${PROJECT_NAME}_PYTHON})
    python_install_on_site("dynamic_graph/${PYTHON_DIR}" ${file})
  endforeach(file ${${PROJECT_NAME}_PYTHON})
endif(BUILD_PYTHON_INTERFACE)
