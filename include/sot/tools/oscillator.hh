//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux,
//         Mehdi Benallegue <mehdi@benallegue.com>
//

#ifndef SOT_TOOLS_OSCILLATOR_HH
#define SOT_TOOLS_OSCILLATOR_HH

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>

#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>

namespace dynamicgraph {
namespace sot {
namespace tools {

class Oscillator : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  Oscillator(const std::string name);

 protected:
  double& computeSignal(double& sout, const int& t);
  dynamicgraph::Vector& computeVectorSignal(dynamicgraph::Vector& vsout, const int& t);
  double value(double dt, double time, double omega, double phase, double amplitude, double bias);

  SignalPtr<double, int> angularFrequencySIN_;
  SignalPtr<double, int> magnitudeSIN_;
  SignalPtr<double, int> phaseSIN_;
  SignalPtr<double, int> biasSIN_;
  SignalTimeDependent<double, int> soutSOUT_;
  SignalTimeDependent<dynamicgraph::Vector, int> vectorSoutSOUT_;

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
