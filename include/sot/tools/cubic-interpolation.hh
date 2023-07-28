//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#ifndef SOT_TOOLS_CUBIC_INTERPOLATION_HH
#define SOT_TOOLS_CUBIC_INTERPOLATION_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

namespace dynamicgraph {
namespace sot {
namespace tools {
class CubicInterpolation : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  virtual ~CubicInterpolation();
  CubicInterpolation(const std::string& name);
  /// Start tracking
  void start(const double& duration);
  /// Reset state to 0 before starting a new motion
  void reset();
  /// Documentation
  virtual std::string getDocString() const;
  /// Set sampling period of control discretization
  void setSamplingPeriod(const double& period);

 protected:
  virtual void doStart(const double& duration);
  dynamicgraph::Signal<Vector, sigtime_t> soutSOUT_;
  dynamicgraph::Signal<Vector, sigtime_t> soutdotSOUT_;
  dynamicgraph::SignalPtr<Vector, sigtime_t> initSIN_;
  dynamicgraph::SignalPtr<Vector, sigtime_t> goalSIN_;

  Vector& computeSout(Vector& sout, const sigtime_t& inTime);
  Vector& computeSoutdot(Vector& sout, const sigtime_t& inTime);

  sigtime_t startTime_;
  double samplingPeriod_;
  double duration_;
  // 0: motion not started, 1: motion in progress, 2: motion finished
  unsigned state_;

  Vector p0_;
  Vector p1_;
  Vector p2_;
  Vector p3_;
};  // class CubicInterpolation
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_TOOLS_CUBIC_INTERPOLATION_SE3_HH
