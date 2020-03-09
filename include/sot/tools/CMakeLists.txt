SET(${PROJECT_NAME}_HEADERS
  cubic-interpolation.hh
  cubic-interpolation-se3.hh
  kinematic-planner.hh
  seqplay.hh
  simpleseqplay.hh)

INSTALL(FILES ${${PROJECT_NAME}_HEADERS} DESTINATION include/sot/core)
