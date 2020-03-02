//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "sot/tools/cubic-interpolation-se3.hh"

namespace dynamicgraph {
namespace sot {
namespace tools {
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CubicInterpolationSE3, "CubicInterpolationSE3");
CubicInterpolationSE3::CubicInterpolationSE3(const std::string& name)
    : Entity(name),
      soutSOUT_("CubicInterpolationSE3(" + name + ")::output(MatrixHomo)::sout"),
      soutdotSOUT_("CubicInterpolationSE3(" + name + ")::output(vector)::soutdot"),
      initSIN_(NULL, "CubicInterpolationSE3(" + name + ")::input(MatrixHomo)::init"),
      goalSIN_(NULL, "CubicInterpolationSE3(" + name + ")::input(MatrixHomo)::goal"),
      startTime_(0),
      samplingPeriod_(0.),
      state_(0),
      p0_(3),
      p1_(3),
      p2_(3),
      p3_(3),
      position_(3),
      soutdot_(3) {
  signalRegistration(soutSOUT_);
  signalRegistration(soutdotSOUT_);
  signalRegistration(initSIN_);
  signalRegistration(goalSIN_);
  soutSOUT_.setFunction(boost::bind(&CubicInterpolationSE3::computeSout, this, _1, _2));
  soutdot_.setZero();
  soutdotSOUT_.setConstant(soutdot_);

  std::string docstring;
  docstring =
      "\n"
      "    Set sampling period of control discretization.\n"
      "\n"
      "    Input:\n"
      "      - a floating point value.\n"
      "\n";
  addCommand("setSamplingPeriod", new command::Setter<CubicInterpolationSE3, double>(
                                      *this, &CubicInterpolationSE3::setSamplingPeriod, docstring));
  docstring =
      "\n"
      "    Start tracking.\n"
      "\n"
      "    Input\n"
      "      - duration of the motion.\n"
      "\n"
      "\n  Read init and goal signals, compute output trajectory and start\n"
      "tracking.\n";
  addCommand("start",
             new command::Setter<CubicInterpolationSE3, double>(*this, &CubicInterpolationSE3::start, docstring));
  docstring =
      "  Reset interpolation before calling start again\n"
      "\n"
      "    After the end of an interpolation, goal signal is copied into\n"
      "    sout signal. Calling reset make the entity copy init signal into\n"
      "    sout signal.\n";
  addCommand("reset", command::makeCommandVoid0(*this, &CubicInterpolationSE3::reset, docstring));
}

CubicInterpolationSE3::~CubicInterpolationSE3() {}

std::string CubicInterpolationSE3::getDocString() const {
  std::string doc =
      "Perform a cubic interpolation in SE(3) between two poses.\n"
      "\n"
      "  Initial pose is given by signal 'init', Target position is given"
      " by signal\n"
      "  'goal'. Interpolation is performed with zero velocities at start"
      " and goal\n"
      "  positions.\n";
  return doc;
}

void CubicInterpolationSE3::reset() { state_ = 0; }

sot::MatrixHomogeneous& CubicInterpolationSE3::computeSout(sot::MatrixHomogeneous& sout, const int& inTime) {
  double t;
  switch (state_) {
    case 0:
      sout = initSIN_.accessCopy();
      break;
    case 1:
      t = (inTime - startTime_) * samplingPeriod_;
      position_ = p0_ + (p1_ + (p2_ + p3_ * t) * t) * t;
      sout(0, 3) = position_(0);
      sout(1, 3) = position_(1);
      sout(2, 3) = position_(2);
      soutdot_ = p1_ + (p2_ * 2 + p3_ * (3 * t)) * t;
      soutdotSOUT_.setConstant(soutdot_);
      if (t >= duration_) {
        state_ = 2;
      }
      break;
    case 2:
      soutdot_.setZero();
      soutdotSOUT_.setConstant(soutdot_);
      sout = goalSIN_.accessCopy();
    default:
      break;
  }
  return sout;
}

void CubicInterpolationSE3::setSamplingPeriod(const double& period) { samplingPeriod_ = period; }

void CubicInterpolationSE3::start(const double& duration) { doStart(duration); }

void CubicInterpolationSE3::doStart(const double& duration) {
  // Check that sampling period has been initialized
  if (samplingPeriod_ <= 0)
    throw ExceptionSignal(ExceptionSignal::NOT_INITIALIZED,
                          "CubicInterpolationSE3: samplingPeriod should"
                          " be positive. Are you sure you did\n"
                          "initialize it?");
  if (state_ == 0) {
    duration_ = duration;
    startTime_ = soutSOUT_.getTime();
    double T = duration;
    // Initial position
    p0_(0) = initSIN_.accessCopy()(0, 3);
    p0_(1) = initSIN_.accessCopy()(1, 3);
    p0_(2) = initSIN_.accessCopy()(2, 3);
    // Initial velocity
    p1_(0) = 0.;
    p1_(1) = 0.;
    p1_(2) = 0.;
    // Goal position
    Vector P_T(3);
    P_T(0) = goalSIN_.accessCopy()(0, 3);
    P_T(1) = goalSIN_.accessCopy()(1, 3);
    P_T(2) = goalSIN_.accessCopy()(2, 3);
    // Final velocity
    Vector D_T(3);
    D_T.setZero();
    p2_ = (D_T + p1_ * 2) * (-1. / T) + (P_T - p0_) * (3. / (T * T));
    p3_ = (P_T - p0_) * (-2 / (T * T * T)) + (p1_ + D_T) * (1. / (T * T));
    state_ = 1;
  }
}
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph
