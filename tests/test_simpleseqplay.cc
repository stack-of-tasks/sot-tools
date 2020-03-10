#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
#include <boost/utility/binary.hpp>

#include "simpleseqplay.hh"

namespace dg = dynamicgraph;
BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_simpleseqplay) {
  dg::sot::tools::SimpleSeqPlay aSimpleSeqPlay("simpleseqplay");

  aSimpleSeqPlay.load("test");
  aSimpleSeqPlay.start();
  for (int i = 0; i < 1000; i++) {
    dg::Vector pos;

    pos = aSimpleSeqPlay.postureSOUT_(i);
    if (i == 100) aSimpleSeqPlay.hold();

    if (i == 300) aSimpleSeqPlay.unhold();
  }
}

BOOST_AUTO_TEST_SUITE_END()
