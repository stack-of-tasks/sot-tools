#define BOOST_TEST_MODULE simpleseqplay
#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
#include <boost/utility/binary.hpp>

#include "sot/tools/simpleseqplay.hh"

namespace dg = dynamicgraph;
BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_simpleseqplay) {
  dg::sot::tools::SimpleSeqPlay aSimpleSeqPlay("simpleseqplay");

  dg::Vector initial (32);
  initial << 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, 0.0, 0.006761, 0.25847, 0.173046, -0.0002, -0.525366, 0.0, -0.0, 0.1, -0.005, -0.25847, -0.173046, 0.0002, -0.525366, 0.0, 0.0, 0.1, -0.005, 0.0, 0.0;
  aSimpleSeqPlay.currentPostureSIN_.setConstant(initial);

  BOOST_CHECK(aSimpleSeqPlay.waiting());

  aSimpleSeqPlay.load(DATA_DIRECTORY "/test");
  aSimpleSeqPlay.start();
  BOOST_CHECK(aSimpleSeqPlay.initializing());

  for (int i = 0; i < 6200; i++) {
    aSimpleSeqPlay.currentPostureSIN_.setTime(i);

    dg::Vector pos;
    aSimpleSeqPlay.postureSOUT_.recompute(i);
    pos = aSimpleSeqPlay.postureSOUT_.accessCopy();
    BOOST_CHECK_EQUAL(initial.size(), pos.size());

    BOOST_CHECK(i == 6199 || aSimpleSeqPlay.executing());
  }
  BOOST_CHECK(aSimpleSeqPlay.finished());
}

BOOST_AUTO_TEST_SUITE_END()
