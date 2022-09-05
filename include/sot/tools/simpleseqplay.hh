//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// From Author: Florent Lamiraux, Mehdi Benallegue,
// Author: Olivier Stasse
// Simple sequence player just playing back a set of poses.
//

#ifndef SOT_TOOLS_SIMPLE_SEQPLAY_HH
#define SOT_TOOLS_SIMPLE_SEQPLAY_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <fstream>
#include <iostream>
#include <sot/core/matrix-geometry.hh>
#include <sstream>

namespace dynamicgraph {
namespace sot {
namespace tools {
namespace dg = dynamicgraph;

class SimpleSeqPlay : public dg::Entity {
 public:
  typedef int Dummy;
  dg::SignalTimeDependent<Dummy, int> firstSINTERN;
  dg::SignalTimeDependent<dg::Vector, int> postureSOUT_;

  dg::SignalPtr<dg::Vector, int> currentPostureSIN_;

  DYNAMIC_GRAPH_ENTITY_DECL();
  SimpleSeqPlay(const std::string& name);

  void load(const std::string& filename);
  void start();
  virtual std::string getDocString() const;

  bool waiting() const;
  bool initializing() const;
  bool executing() const;
  bool finished() const;

 private:
  dg::Vector& computePosture(dg::Vector& pos, int t);
  // 0: motion not started,
  // 1: going to the current position to the first position.
  // 2: motion in progress, 3: motion finished
  unsigned int state_;
  int startTime_;

  std::vector<dg::Vector> posture_;
  dg::Vector currentPosture_;

  std::vector<double> time_;
  double dt_;
  /// Time to start.
  double time_to_start_;
  // Number of iterations performed in state1.
  int it_nbs_in_state1_;

};  // class SimpleSeqPlay
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_TOOLS_SIMPLESEQPLAY_HH
