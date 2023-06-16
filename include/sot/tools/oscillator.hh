//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux,
//         Mehdi Benallegue <mehdi@benallegue.com>
//

#ifndef SOT_TOOLS_OSCILLATOR_HH
#define SOT_TOOLS_OSCILLATOR_HH

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

namespace dynamicgraph {
namespace sot {
namespace tools {

class Oscillator : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  Oscillator(const std::string name);

 protected:
  double& computeSignal(double& sout, const sigtime_t& t);
  dynamicgraph::Vector& computeVectorSignal(dynamicgraph::Vector& vsout,
                                            const sigtime_t& t);
  double value(double dt, double time, double omega, double phase,
               double amplitude, double bias);

  SignalPtr<double, sigtime_t> angularFrequencySIN_;
  SignalPtr<double, sigtime_t> magnitudeSIN_;
  SignalPtr<double, sigtime_t> phaseSIN_;
  SignalPtr<double, sigtime_t> biasSIN_;
  SignalTimeDependent<double, sigtime_t> soutSOUT_;
  SignalTimeDependent<dynamicgraph::Vector, sigtime_t> vectorSoutSOUT_;

  double epsilon_;
  bool started_;
  bool continuous_;
  double dt_;
  double lastValue_;
};  // class Oscillator

}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_TOOLS_OSCILLATOR_HH
