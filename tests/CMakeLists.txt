
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_EXECUTABLE(test_simpleseqplay test_simpleseqplay.cc)
TARGET_LINK_LIBRARIES(test_simpleseqplay sot-core::sot-core)
TARGET_LINK_LIBRARIES(test_simpleseqplay sot-tools)
TARGET_LINK_LIBRARIES(test_simpleseqplay
  ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARIES})
ADD_TEST(test_simpleseqplay test_simpleseqplay)
