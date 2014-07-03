//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux, 
//         Mehdi Benallegue <mehdi@benallegue.com>
//

#include <limits>

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


      class Oscillator : public Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();
      public:
        Oscillator (const std::string name);

      protected:

        double& computeSignal (double& sout, const int& t);
        dynamicgraph::Vector& computeVectorSignal (dynamicgraph::Vector& vsout, const int& t);
        double value(double dt, double time, double omega,
        double phase, double amplitude, double bias);

        SignalPtr < double, int > angularFrequencySIN_;
        SignalPtr < double, int > magnitudeSIN_;
        SignalPtr < double, int > phaseSIN_;
        SignalPtr < double, int > biasSIN_;
        SignalTimeDependent < double, int > soutSOUT_;
        SignalTimeDependent < dynamicgraph::Vector, int > vectorSoutSOUT_;

        double epsilon_;
        bool started_;
        bool continuous_;
        double dt_;
        double lastValue_;
      }; // class Oscillator




      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Oscillator, "Oscillator");




      Oscillator::Oscillator (const std::string name):
        Entity (name),
        angularFrequencySIN_ (0,"Oscillator("+name+")::input(double)::omega"),
        magnitudeSIN_ (0, "Oscillator("+name+")::input(double)::magnitude"),
        phaseSIN_ (0, "Oscillator("+name+")::input(double)::phase"),
        biasSIN_ (0, "Oscillator("+name+")::input(double)::bias"),
        soutSOUT_ ("Oscillator("+name+")::output(double)::sout"),
        vectorSoutSOUT_ ("Oscillator("+name+")::output(vector)::vectorSout"),
        epsilon_(1e-3),
        started_(true),
        continuous_(false),
        dt_ (0.),
        lastValue_(0.0)
      {
        signalRegistration (angularFrequencySIN_ << magnitudeSIN_
                            << phaseSIN_ << biasSIN_
                            << soutSOUT_ << vectorSoutSOUT_);
        angularFrequencySIN_.setConstant (0.);
        magnitudeSIN_.setConstant (0.);
        phaseSIN_.setConstant (0.);
        biasSIN_.setConstant (0.);

        soutSOUT_.addDependency (angularFrequencySIN_);
        soutSOUT_.addDependency (magnitudeSIN_);
        soutSOUT_.addDependency (biasSIN_);
        soutSOUT_.addDependency (phaseSIN_);
        vectorSoutSOUT_.addDependency (soutSOUT_);
        soutSOUT_.setFunction (boost::bind (&Oscillator::computeSignal,
                                            this, _1, _2));
        vectorSoutSOUT_.setFunction (boost::bind (&Oscillator::computeVectorSignal,
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

        addCommand ("setActivated",
                    makeDirectSetter (*this, &started_,
                                      docDirectSetter ("activated",
                                          "bool")));

        addCommand ("getActivated",
                    makeDirectGetter (*this, &started_,
                                      docDirectGetter ("activated",
                                          "bool")));

        /// epsilon is used to ensure there is no discontinuity when starting or stopping
        /// the oscillator. It defines the sensitivity to discontinuities
        addCommand ("setEpsilon",
                    makeDirectSetter (*this, &epsilon_,
                                      docDirectSetter ("ocillator zero-sensitivity",
                                          "double")));

        addCommand ("getEpsilon",
                    makeDirectGetter (*this, &epsilon_,
                                      docDirectGetter ("ocillator zero-sensitivity",
                                          "double")));

        addCommand ("setValue",
                    makeDirectSetter (*this, &lastValue_,
                                      docDirectSetter ("init value of the oscillator",
                                          "double")));

        addCommand ("getValue",
                    makeDirectGetter (*this, &lastValue_,
                                      docDirectGetter ("current value of the oscillator",
                                          "double")));

        addCommand ("setContinuous",
                    makeDirectSetter (*this, &continuous_,
                                      docDirectSetter ("continuous",
                                          "bool")));

        addCommand ("getContinuous",
                    makeDirectGetter (*this, &continuous_,
                                      docDirectGetter ("continuous",
                                          "bool")));
      }

      double Oscillator::value(double dt, double t, double omega,
                               double phase, double m, double bias)
      {
        double tau = dt * t;
        return  m * sin (omega * tau + phase) + bias;
      }

      dynamicgraph::Vector& Oscillator::computeVectorSignal (dynamicgraph::Vector& vsout, const int& t)
      {
        vsout.resize(1);
        vsout(0)=soutSOUT_.access(t);
        return vsout;
      }

      double& Oscillator::computeSignal (double& sout, const int& t)
      {
        double eps;

        if (continuous_)
          eps = epsilon_;
        else
          eps = std::numeric_limits<double>::max();

        double omega = angularFrequencySIN_.access (t);
        double m = magnitudeSIN_.access (t);
        double phase = phaseSIN_.access(t);
        double bias = biasSIN_.access(t);

        if (started_)
        {

          double current = value(dt_, t, omega, phase, m, bias);

          if (fabs(lastValue_-current)<eps)
            lastValue_ = sout = current;
          else
            sout = lastValue_;
        }
        else
        {
          if (fabs(lastValue_)<eps )
            lastValue_ = sout = 0;
          else
            lastValue_ = sout = value(dt_, t, omega, phase, m, bias);
        }


        return sout;
      }
    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
