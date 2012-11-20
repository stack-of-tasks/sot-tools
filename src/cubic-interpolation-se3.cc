//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "cubic-interpolation-se3.hh"

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CubicInterpolationSE3,
					 "CubicInterpolationSE3");
      CubicInterpolationSE3::CubicInterpolationSE3 (const std::string& name) :
	Entity (name),
	soutSOUT_ ("CubicInterpolationSE3("+name+
		   ")::output(MatrixHomo)::sout"),
	soutdotSOUT_ ("CubicInterpolationSE3("+name+
		      ")::output(vector)::soutdot"),
	initSIN_
	(NULL, "CubicInterpolationSE3("+name+")::input(MatrixHomo)::init"),
	goalSIN_
	(NULL, "CubicInterpolationSE3("+name+")::input(MatrixHomo)::goal"),
	startTime_ (0), samplingPeriod_ (0.), motionStarted_ (false),
	p0_ (3), p1_ (3), p2_ (3), p3_ (3), position_ (3), soutdot_ (3)
      {
	signalRegistration (soutSOUT_);
	signalRegistration (soutdotSOUT_);
	signalRegistration (initSIN_);
	signalRegistration (goalSIN_);
	soutSOUT_.setFunction (boost::bind
			       (&CubicInterpolationSE3::computeSout,
				this, _1, _2));
	soutdot_.setZero ();
	soutdotSOUT_.setConstant (soutdot_);

	std::string docstring;
	docstring =
	  "\n"
	  "    Set sampling period of control discretization.\n"
	  "\n"
	  "    Input:\n"
	  "      - a floating point value.\n"
	  "\n";
	addCommand ("setSamplingPeriod",
		    new command::Setter <CubicInterpolationSE3, double>
		    (*this, &CubicInterpolationSE3::setSamplingPeriod, docstring));
	docstring =
	  "\n"
	  "    Start tracking.\n"
	  "\n"
	  "    Input\n"
	  "      - duration of the motion.\n"
	  "\n"
	  "\n  Read init and goal signals, compute output trajectory and start\n"
	  "tracking.\n";
	addCommand ("start",
		    new command::Setter <CubicInterpolationSE3, double>
		    (*this, &CubicInterpolationSE3::start, docstring));
      }

      CubicInterpolationSE3::~CubicInterpolationSE3 ()
      {
      }

      std::string CubicInterpolationSE3::getDocString () const
      {
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

      sot::MatrixHomogeneous&
      CubicInterpolationSE3::computeSout (sot::MatrixHomogeneous&
					  sout, const int& inTime)
      {
	if (!motionStarted_) {
	  sout = initSIN_.accessCopy ();
	} else {
	  double t = (inTime - startTime_) * samplingPeriod_;
	  if (t <= duration_) {
	    position_ = p0_ + (p1_ + (p2_ + p3_*t)*t)*t;
	    sout (0,3) = position_ (0);
	    sout (1,3) = position_ (1);
	    sout (2,3) = position_ (2);

	    soutdot_ = p1_ + (p2_*2 + p3_*(3*t))*t;
	    soutdotSOUT_.setConstant (soutdot_);
	  } else {
	    motionStarted_ = false;
	    soutdot_.setZero ();
	    soutdotSOUT_.setConstant (soutdot_);
	    sout = goalSIN_ (inTime);
	  }
	}
	return sout;
      }

      void CubicInterpolationSE3::setSamplingPeriod (const double& period)
      {
	samplingPeriod_ = period;
      }

      void CubicInterpolationSE3::start (const double& duration)
      {
	doStart (duration);
      }

      void CubicInterpolationSE3::doStart (const double& duration)
      {
	// Check that sampling period has been initialized
	if (samplingPeriod_ <= 0)
	  throw ExceptionSignal (ExceptionSignal::NOT_INITIALIZED,
				 "CubicInterpolationSE3: samplingPeriod should"
				 " be positive. Are you sure you did\n"
				 "initialize it?");
	int inTime = initSIN_.getTime ();
	if (!motionStarted_) {
	  duration_ = duration;
	  startTime_ = soutSOUT_.getTime ();
	  double T = duration;
	  // Initial position
	  p0_ (0) = initSIN_ (inTime) (0,3);
	  p0_ (1) = initSIN_ (inTime) (1,3);
	  p0_ (2) = initSIN_ (inTime) (2,3);
	  // Initial velocity
	  p1_ (0) = 0.;
	  p1_ (1) = 0.;
	  p1_ (2) = 0.;
	  // Goal position
	  maal::boost::Vector P_T (3);
	  P_T (0) = goalSIN_ (inTime) (0,3);
	  P_T (1) = goalSIN_ (inTime) (1,3);
	  P_T (2) = goalSIN_ (inTime) (2,3);
	  // Final velocity
	  maal::boost::Vector D_T (3); D_T.setZero ();
	  p2_ = (D_T + p1_*2)*(-1./T) + (P_T - p0_)*(3./(T*T));
	  p3_ = (P_T -p0_)*(-2/(T*T*T)) + (p1_ + D_T)*(1./(T*T));
	  motionStarted_ = true;
	}
      }
    } // tools
  } // namespace sot
} // namespace dynamicgraph
