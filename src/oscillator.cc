//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

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
    using command::makeDirectSetter;
    using command::docDirectSetter;
    using command::docDirectGetter;
    using command::makeDirectGetter;
    namespace tools {
      class Oscillator : public Entity {
	DYNAMIC_GRAPH_ENTITY_DECL();
      public:
	Oscillator (const std::string name) :
	  Entity (name),
	  angularFrequencySIN_ (0,
				"Oscillator("+name+")::input(double)::omega"),
	  magnitudeSIN_ (0, "Oscillator("+name+")::input(double)::magnitude"),
	  soutSOUT_ ("Oscillator("+name+")::output(double)::sout"),
	  dt_ (0.)
	{
	  signalRegistration (angularFrequencySIN_ << magnitudeSIN_
			      << soutSOUT_);
	  angularFrequencySIN_.setConstant (0.);
	  magnitudeSIN_.setConstant (0.);
	  soutSOUT_.addDependency (angularFrequencySIN_);
	  soutSOUT_.addDependency (magnitudeSIN_);
	  soutSOUT_.setFunction (boost::bind (&Oscillator::computeSignal,
					      this, _1, _2));
	  soutSOUT_.setNeedUpdateFromAllChildren (true);
	  soutSOUT_.setDependencyType (TimeDependency <int>::ALWAYS_READY);

	  addCommand ("setTimePeriod",
		      makeDirectSetter (*this, &dt_,
					docDirectSetter ("time period",
							 "double")));
	  addCommand ("getTimePeriod",
		      makeDirectGetter (*this, &dt_,
					docDirectGetter ("time period",
							 "double")));
	}
      protected:
	double& computeSignal (double& sout, const int& t)
	{
	  double tau = dt_ * t;
	  double omega = angularFrequencySIN_.access (t);
	  double m = magnitudeSIN_.access (t);
	  sout = m * sin (omega * tau);
	  return sout;
	}

	SignalPtr < double, int > angularFrequencySIN_;
	SignalPtr < double, int > magnitudeSIN_;
	SignalTimeDependent < double, int > soutSOUT_;

	double dt_;
      }; // class Oscillator
      
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Oscillator, "Oscillator");
    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
