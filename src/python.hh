#include <sot/tools/cubic-interpolation.hh>
#include <sot/tools/cubic-interpolation-se3.hh>
#include <sot/tools/kinematic-planner.hh>
#include <sot/tools/seqplay.hh>
#include <sot/tools/simpleseqplay.hh>

namespace dgst = dynamicgraph::sot::tools;

typedef boost::mpl::vector<
  dgst::CubicInterpolation
, dgst::CubicInterpolationSE3
, dgst::KinematicPlanner
, dgst::Seqplay
, dgst::SimpleSeqPlay
> entities_t;
