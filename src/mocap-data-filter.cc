//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Alexid Mifsud <alexis.mifsud@gmail.com>
//

#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>

#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>

#include <sot/core/matrix-homogeneous.hh>

namespace dynamicgraph {
  namespace sot {
    using command::makeDirectSetter;
    using command::makeDirectGetter;
    using command::docDirectSetter;
    using command::docDirectGetter;
    namespace tools {


      class MocapDataFilter : public Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();
      public:
        MocapDataFilter (const std::string name);

      protected:

        ::dynamicgraph::sot::MatrixHomogeneous& computeSout (::dynamicgraph::sot::MatrixHomogeneous& sout, const int& t);

        SignalPtr < ::dynamicgraph::sot::MatrixHomogeneous, int > sinSIN_;
        SignalTimeDependent < ::dynamicgraph::sot::MatrixHomogeneous, int > soutSOUT_;

        double threshold_;
        ::dynamicgraph::sot::MatrixHomogeneous lastSout_;

      }; // class MocapDataFilter

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MocapDataFilter, "MocapDataFilter");

      MocapDataFilter::MocapDataFilter (const std::string name):
        Entity (name),
        sinSIN_ (0,"MocapDataFilter("+name+")::input(double)::sin"),
        soutSOUT_ ("MocapDataFilter("+name+")::output(vector)::sout"),
        threshold_(5000.)
      {
        signalRegistration (sinSIN_);
        signalRegistration (soutSOUT_);

        ::dynamicgraph::sot::MatrixHomogeneous m;
        sinSIN_.setConstant (m);

        soutSOUT_.addDependency (sinSIN_);
        soutSOUT_.setFunction (boost::bind (&MocapDataFilter::computeSout, this, _1, _2));

        addCommand ("setThreshold",
                    makeDirectSetter (*this, &threshold_,
                                      docDirectSetter ("threshold",
                                          "double")));
        addCommand ("getThreshold",
                    makeDirectGetter (*this, &threshold_,
                                      docDirectGetter ("threshold",
                                          "double")));

        lastSout_.setIdentity();

      }

      ::dynamicgraph::sot::MatrixHomogeneous& MocapDataFilter::computeSout (::dynamicgraph::sot::MatrixHomogeneous& sout, const int& inTime)
      {

        const dynamicgraph::sot::MatrixHomogeneous & sin = sinSIN_.access(inTime);

        // Velocity computation
        Vector vel; vel.resize(3);
        for (int i=0; i<3; ++i)
            vel.elementAt(i) = std::abs((sin.elementAt(i,3)-lastSout_.elementAt(i,3))/0.005);

        if(vel.elementAt(0) < threshold_ && vel.elementAt(1) < threshold_ && vel.elementAt(2) < threshold_)
        {
            sout=sin;
        }
        else
        {
            sout=lastSout_;
        }

        lastSout_ = sout;
        return sout;
      }

    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
