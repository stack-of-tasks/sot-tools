//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "sot/tools/cubic-interpolation.hh"

namespace dynamicgraph {
namespace sot {
namespace tools {
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CubicInterpolation, "CubicInterpolation");
CubicInterpolation::CubicInterpolation(const std::string& name)
    : Entity(name),
      soutSOUT_("CubicInterpolation(" + name + ")::output(vector)::sout"),
      soutdotSOUT_("CubicInterpolation(" + name + ")::output(vector)::soutdot"),
      initSIN_(NULL, "CubicInterpolation(" + name + ")::input(vector)::init"),
      goalSIN_(NULL, "CubicInterpolation(" + name + ")::input(vector)::goal"),
      startTime_(0),
      samplingPeriod_(0.),
      state_(0),
      p0_(),
      p1_(),
      p2_(),
      p3_() {
  signalRegistration(soutSOUT_);
  signalRegistration(soutdotSOUT_);
  signalRegistration(initSIN_);
  signalRegistration(goalSIN_);
  soutSOUT_.setFunction(boost::bind(&CubicInterpolation::computeSout, this, _1, _2));
  soutdotSOUT_.setFunction(boost::bind(&CubicInterpolation::computeSoutdot, this, _1, _2));
  std::string docstring;
  docstring =
      "  Set sampling period of control discretization.\n"
      "\n"
      "    Input:\n"
      "      - a floating point value.\n"
      "\n";
  addCommand("setSamplingPeriod", new command::Setter<CubicInterpolation, double>(
                                      *this, &CubicInterpolation::setSamplingPeriod, docstring));
  docstring =
      "  Start tracking.\n"
      "\n"
      "    Input\n"
      "      - duration of the motion.\n"
      "\n"
      "\n  Read init and goal signals, compute output trajectory and"
      " start\n"
      "tracking.\n";
  addCommand("start", new command::Setter<CubicInterpolation, double>(*this, &CubicInterpolation::start, docstring));
  docstring =
      "  Reset interpolation before calling start again\n"
      "\n"
      "    After the end of an interpolation, goal signal is copied into\n"
      "    sout signal. Calling reset make the entity copy init signal into\n"
      "    sout signal.\n";
  addCommand("reset", command::makeCommandVoid0(*this, &CubicInterpolation::reset, docstring));
}

CubicInterpolation::~CubicInterpolation() {}

std::string CubicInterpolation::getDocString() const {
  std::string doc =
      "Perform a cubic interpolation in between two vectors.\n"
      "\n"
      "  Initial pose is given by signal 'init', Target position is given"
      " by signal\n"
      "  'goal'. Interpolation is performed with zero velocities at start"
      " and goal\n"
      "  positions.\n";
  return doc;
}

void CubicInterpolation::reset() { state_ = 0; }

Vector& CubicInterpolation::computeSout(Vector& sout, const int& inTime) {
  double t;
  switch (state_) {
    case 0:
      sout = initSIN_.accessCopy();
      break;
    case 1:
      t = (inTime - startTime_) * samplingPeriod_;
      sout = p0_ + (p1_ + (p2_ + p3_ * t) * t) * t;
      if (t >= duration_) {
        state_ = 2;
      }
      break;
    case 2:
      sout = goalSIN_.accessCopy();
    default:
      break;
  }
  return sout;
}

Vector& CubicInterpolation::computeSoutdot(Vector& soutdot, const int& inTime) {
  soutdot.resize(initSIN_.accessCopy().size());
  double t;
  switch (state_) {
    case 0:
      soutdot.setZero();
      break;
    case 1:
      t = (inTime - startTime_) * samplingPeriod_;
      soutdot = p1_ + (p2_ * 2 + p3_ * (3 * t)) * t;
      if (t >= duration_) {
        state_ = 2;
      }
      break;
    case 2:
      soutdot.setZero();
    default:
      break;
  }
  return soutdot;
}

void CubicInterpolation::setSamplingPeriod(const double& period) { samplingPeriod_ = period; }

void CubicInterpolation::start(const double& duration) { doStart(duration); }

void CubicInterpolation::doStart(const double& duration) {
  // Check that sampling period has been initialized
  if (samplingPeriod_ <= 0)
    throw ExceptionSignal(ExceptionSignal::NOT_INITIALIZED,
                          "CubicInterpolation: samplingPeriod should"
                          " be positive. Are you sure you did\n"
                          "initialize it?");
  if (state_ == 0) {
    duration_ = duration;
    startTime_ = soutSOUT_.getTime();
    double T = duration;
    // Initial position
    p0_ = initSIN_.accessCopy();
    // Initial velocity
    p1_.resize(p0_.size());
    p1_.fill(0.);
    // Goal position
    Vector P_T;
    P_T = goalSIN_.accessCopy();
    // Final velocity
    Vector D_T(P_T.size());
    D_T.fill(0.);
    p2_ = (D_T + p1_ * 2) * (-1. / T) + (P_T - p0_) * (3. / (T * T));
    p3_ = (P_T - p0_) * (-2 / (T * T * T)) + (p1_ + D_T) * (1. / (T * T));
    state_ = 1;
  }
}
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph
