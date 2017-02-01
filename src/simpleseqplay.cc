//
// Copyright (C) 2012, 2013, 2017 LAAS-CNRS
//
// From Author: Florent Lamiraux, Mehdi Benallegue, 
// Author: Olivier Stasse
// Simple sequence player just playing back a set of poses.
//

#include <vector>
#include <stdexcept>
#include <boost/tokenizer.hpp>

#include <iostream>

#include <dynamic-graph/command-bind.h>

#include "simpleseqplay.hh"

namespace dynamicgraph
{
  namespace sot
  {
    namespace tools
    {
      using dynamicgraph::Entity;
      using dynamicgraph::command::makeCommandVoid0;
      using dynamicgraph::command::docCommandVoid0;
      using dynamicgraph::command::makeCommandVoid1;
      using dynamicgraph::command::docCommandVoid1;

      SimpleSeqPlay::SimpleSeqPlay (const std::string& name) :
        Entity (name),
        postureSOUT_ ("SimpleSeqPlay(" + name + ")::output(vector)::posture"),
        state_ (0), startTime_ (0), posture_ ()
      {
        signalRegistration (postureSOUT_ );
        postureSOUT_.setFunction (boost::bind (&SimpleSeqPlay::computePosture,
                                               this, _1, _2));
cd

        std::string docstring =
          "Load files describing a whole-body motion as reference feature "
          "trajectories\n"
          "\n"
          "  Input:\n"
          "    - string filename without extension\n"
          "\n"
          "  Data is read from the following files:\n"
          "    - posture from file \"filename.posture\",\n"
          "  The file should contain one column for time and as many columns as"
          " required\n"
          "  depending of degree-of-freedrom.\n"
          "\n";
        addCommand ("load",
                    makeCommandVoid1 (*this, &SimpleSeqPlay::load,
                                      docstring));

        addCommand ("start",
                    makeCommandVoid0 (*this, &SimpleSeqPlay::start,
                                      docCommandVoid0 ("Start motion")));
        for (size_t i=0; i<7; ++i)
        {
          facultativeFound_[i]=false;
        }
      }

      void SimpleSeqPlay::load (const std::string& filename)
      {
        using boost::escaped_list_separator;
        typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
        std::string line;
        std::string fn;

        std::ifstream file;
        unsigned int lineNumber = 0;
        int postureSize = -2;

        fn= filename + ".posture";

        // Open file
	file.open (fn.c_str ());
	if (!file.is_open ())
          {
            throw std::runtime_error (std::string ("Failed to open file ") +
                                      fn);
          }


        posture_.clear ();

        // Read posture
        while (file.good ())
        {
          std::getline (file, line);
          ++lineNumber;
          tokenizer_t tok (line, escaped_list_separator<char>('\\', '\t', '\"'));
          std::vector <double> components;
          for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it)
          {
            components.push_back (atof (it->c_str ()));
          }
          if (components.size () == 0)
          {
            break;
          }
          if (postureSize == -2)
          {
            postureSize = static_cast<int> (components.size () - 1);
          }
          else
          {
            if (postureSize != static_cast <int> (components.size ()) - 1)
            {
              std::ostringstream oss;
              oss << fn << ", line " << lineNumber << ": config of size "
                  << components.size () - 1 << ". Expecting " << postureSize
                  << ".";
              throw std::runtime_error (oss.str ());
            }
          }
          Vector config (static_cast<unsigned> (components.size () - 1));
          for (unsigned i = 1; i < components.size (); ++i)
          {
            config (i - 1) = components[i];
          }
          posture_.push_back (config);
        }
        file.close ();

        state_ = 0;
      }


      void SimpleSeqPlay::start ()
      {
        if (state_ == 0)
        {
          state_ = 1;
          startTime_ = postureSOUT_.getTime ();
        }
      }

      Vector& SimpleSeqPlay::computePosture (Vector& pos, const int& t)
      {
        if (posture_.size () == 0)
        {
          throw std::runtime_error
          ("SimpleSeqPlay posture: Signals not initialized. read files first.");
        }
        std::size_t configId;
        if (state_ == 0)
        {
          configId = 0;
        }
        else if (state_ == 1)
        {
          configId = t - startTime_;
          if (configId == posture_.size () - 1)
          {
            state_ = 2;
          }
        }
        else
        {
          configId = posture_.size () -1;
        }
        pos = posture_ [configId];
        return pos;
      }

      std::string SimpleSeqPlay::getDocString () const
      {
        return
          "Provide joint references for a whole-body motion\n"
          "\n"
          "  The reference trajectories of the joints are loaded from the file\n"
          "  using command load.\n"
          "\n"
          "  To use this entity,\n"
          "    1. call method load,\n"
          "    2. plug reference signals into robot signals and\n"
          "    3. call method start.\n"
          "  Warning: pluging signals before loading trajectories will fail.\n";
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (SimpleSeqPlay, "SimpleSeqplay");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

