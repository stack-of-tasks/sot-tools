ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(test_simpleseqplay test_simpleseqplay.cc)
TARGET_LINK_LIBRARIES(test_simpleseqplay simpleseqplay ${Boost_LIBRARIES})
