//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/factory.h>

#include "cubic-interpolation.hh"

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CubicInterpolation,
					 "CubicInterpolation");
      CubicInterpolation::CubicInterpolation (const std::string& name) :
	Entity (name),
	soutSOUT_ ("CubicInterpolation("+name+
		   ")::output(vector)::sout"),
	soutdotSOUT_ ("CubicInterpolation("+name+
		      ")::output(vector)::soutdot"),
	initSIN_ (NULL, "CubicInterpolation("+name+")::input(vector)::init"),
	goalSIN_ (NULL, "CubicInterpolation("+name+")::input(vector)::goal"),
	startTime_ (0), samplingPeriod_ (0.), motionStarted_ (false),
	p0_ (), p1_ (), p2_ (), p3_ (), position_ (), soutdot_ ()
      {
	signalRegistration (soutSOUT_);
	signalRegistration (soutdotSOUT_);
	signalRegistration (initSIN_);
	signalRegistration (goalSIN_);
	soutSOUT_.setFunction (boost::bind
			       (&CubicInterpolation::computeSout,
				this, _1, _2));
	soutdotSOUT_.setFunction (boost::bind
				  (&CubicInterpolation::computeSoutdot,
				   this, _1, _2));
	std::string docstring;
	docstring =
	  "\n"
	  "    Set sampling period of control discretization.\n"
	  "\n"
	  "    Input:\n"
	  "      - a floating point value.\n"
	  "\n";
	addCommand ("setSamplingPeriod",
		    new command::Setter <CubicInterpolation, double>
		    (*this, &CubicInterpolation::setSamplingPeriod, docstring));
	docstring =
	  "\n"
	  "    Start tracking.\n"
	  "\n"
	  "    Input\n"
	  "      - duration of the motion.\n"
	  "\n"
	  "\n  Read init and goal signals, compute output trajectory and"
	  " start\n"
	  "tracking.\n";
	addCommand ("start",
		    new command::Setter <CubicInterpolation, double>
		    (*this, &CubicInterpolation::start, docstring));
      }

      CubicInterpolation::~CubicInterpolation ()
      {
      }

      std::string CubicInterpolation::getDocString () const
      {
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

      Vector& CubicInterpolation::computeSout (Vector& sout,
					       const int& inTime)
      {
	if (!motionStarted_) {
	  sout = initSIN_.accessCopy ();
	} else {
	  double t = (inTime - startTime_) * samplingPeriod_;
	  if (t <= duration_) {
	    sout = p0_ + (p1_ + (p2_ + p3_*t)*t)*t;
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

      Vector& CubicInterpolation::computeSoutdot (Vector& soutdot,
						  const int& inTime)
      {
	soutdot.resize (initSIN_.accessCopy ().size ());
	if (!motionStarted_) {
	  soutdot.setZero ();
	} else {
	  double t = (inTime - startTime_) * samplingPeriod_;
	  if (t <= duration_) {
	    soutdot = p1_ + (p2_*2 + p3_*(3*t))*t;
	  } else {
	    soutdot_.setZero ();
	  }
	}
	return soutdot;
      }

      void CubicInterpolation::setSamplingPeriod (const double& period)
      {
	samplingPeriod_ = period;
      }

      void CubicInterpolation::start (const double& duration)
      {
	doStart (duration);
      }

      void CubicInterpolation::doStart (const double& duration)
      {
	// Check that sampling period has been initialized
	if (samplingPeriod_ <= 0)
	  throw ExceptionSignal (ExceptionSignal::NOT_INITIALIZED,
				 "CubicInterpolation: samplingPeriod should"
				 " be positive. Are you sure you did\n"
				 "initialize it?");
	int inTime = initSIN_.getTime ();
	if (!motionStarted_) {
	  duration_ = duration;
	  startTime_ = soutSOUT_.getTime ();
	  double T = duration;
	  // Initial position
	  p0_ = initSIN_ (inTime);
	  // Initial velocity
	  p1_.resize (p0_.size ()); p1_.fill (0.);
	  // Goal position
	  Vector P_T;
	  P_T  = goalSIN_ (inTime);
	  // Final velocity
	  Vector D_T (P_T.size ()); D_T.fill (0.);
	  p2_ = (D_T + p1_*2)*(-1./T) + (P_T - p0_)*(3./(T*T));
	  p3_ = (P_T -p0_)*(-2/(T*T*T)) + (p1_ + D_T)*(1./(T*T));
	  motionStarted_ = true;
	}
      }
    } // tools
  } // namespace sot
} // namespace dynamicgraph
