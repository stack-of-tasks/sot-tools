#include <sot/tools/cubic-interpolation-se3.hh>
#include <sot/tools/cubic-interpolation.hh>
#include <sot/tools/kinematic-planner.hh>
#include <sot/tools/oscillator.hh>
#include <sot/tools/seqplay.hh>
#include <sot/tools/simpleseqplay.hh>

#include "dynamic-graph/python/module.hh"

namespace dgst = dynamicgraph::sot::tools;

typedef boost::mpl::vector<dgst::CubicInterpolation,
                           dgst::CubicInterpolationSE3, dgst::Oscillator,
                           dgst::Seqplay>
    entities_t;

struct register_entity {
  template <typename T>
  inline void operator()(boost::type<T>) const {
    dynamicgraph::python::exposeEntity<T>();
  }
};

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");
  boost::mpl::for_each<entities_t, boost::type<boost::mpl::_> >(
      register_entity());

  using dgst::SimpleSeqPlay;
  dynamicgraph::python::exposeEntity<SimpleSeqPlay>()
      .def("waiting", &SimpleSeqPlay::waiting)
      .def("initializing", &SimpleSeqPlay::initializing)
      .def("executing", &SimpleSeqPlay::executing)
      .def("finished", &SimpleSeqPlay::finished);
}
