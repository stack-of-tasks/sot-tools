add_definitions(-DBOOST_TEST_DYN_LINK)

add_unit_test(test_simpleseqplay test_simpleseqplay.cc)
target_compile_definitions(
  test_simpleseqplay PRIVATE -DDATA_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(test_simpleseqplay ${PROJECT_NAME} ${Boost_LIBRARIES})

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
