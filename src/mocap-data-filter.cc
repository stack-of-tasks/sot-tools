//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Alexid Mifsud <alexis.mifsud@gmail.com>
//

#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>

#include <sot/core/matrix-homogeneous.hh>

#include <iostream>

namespace dynamicgraph {
  namespace sot {
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

      }; // class MocapDataFilter

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MocapDataFilter, "MocapDataFilter");

      MocapDataFilter::MocapDataFilter (const std::string name):
        Entity (name),
        sinSIN_ (0,"MocapDataFilter("+name+")::input(double)::sin"),
        soutSOUT_ ("MocapDataFilter("+name+")::output(vector)::sout")
      {
        signalRegistration (sinSIN_);
        signalRegistration (soutSOUT_);

        ::dynamicgraph::sot::MatrixHomogeneous m;
        sinSIN_.setConstant (m);

        soutSOUT_.addDependency (sinSIN_);
        soutSOUT_.setFunction (boost::bind (&MocapDataFilter::computeSout, this, _1, _2));
      }

      ::dynamicgraph::sot::MatrixHomogeneous& MocapDataFilter::computeSout (::dynamicgraph::sot::MatrixHomogeneous& sout, const int& inTime)
      {

        const dynamicgraph::sot::MatrixHomogeneous & sin = sinSIN_.access(inTime);
        sout = sin;

        return sout;
      }

    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
