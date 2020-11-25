//
// Copyright (C) 2012, 2013, 2017 LAAS-CNRS
//
// From Author: Florent Lamiraux, Mehdi Benallegue,
// Author: Olivier Stasse, Rohan Budhiraja
// Simple sequence player just playing back a set of poses.
//

#include <vector>
#include <stdexcept>
#include <boost/tokenizer.hpp>

#include <iostream>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>

#include "sot/tools/simpleseqplay.hh"

namespace dynamicgraph {
namespace sot {
using command::docDirectGetter;
using command::docDirectSetter;
using command::makeDirectGetter;
using command::makeDirectSetter;

namespace tools {
using dynamicgraph::Entity;
using dynamicgraph::command::docCommandVoid0;
using dynamicgraph::command::docCommandVoid1;
using dynamicgraph::command::makeCommandVoid0;
using dynamicgraph::command::makeCommandVoid1;

SimpleSeqPlay::SimpleSeqPlay(const std::string& name)
    : Entity(name),
      firstSINTERN(NULL, sotNOSIGNAL, "SimpleSeqPlay(" + name + ")::intern(dummy)::init"),
      postureSOUT_(boost::bind(&SimpleSeqPlay::computePosture, this, _1, _2), currentPostureSIN_,
                   "SimpleSeqPlay(" + name + ")::output(vector)::posture"),
      currentPostureSIN_(NULL, "SimpleSeqPlay(" + name + ")::input(vector)::currentPosture"),
      state_(0),
      startTime_(0),
      posture_(),
      time_(0),
      dt_(0.001),
      time_to_start_(3.0),
      it_nbs_in_state1_(0) {
  firstSINTERN.setConstant(0);
  signalRegistration(postureSOUT_);
  signalRegistration(currentPostureSIN_);

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
  addCommand("load", makeCommandVoid1(*this, &SimpleSeqPlay::load, docstring));

  addCommand("start", makeCommandVoid0(*this, &SimpleSeqPlay::start, docCommandVoid0("Start motion")));

  docstring = "Set the time between the robot current pose and the starting of the buffer \n";

  addCommand("setTimeToStart",
             makeDirectSetter(*this, &time_to_start_, docDirectSetter("Time to start of the buffer", "double")));
}

void SimpleSeqPlay::load(const std::string& filename) {
  state_ = 0;

  using boost::escaped_list_separator;
  typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
  std::string line;
  std::string fn;

  std::ifstream file;
  unsigned int lineNumber = 0;
  int postureSize = -2;
  fn = filename + ".posture";
  // Open file
  file.open(fn.c_str());
  if (!file.is_open()) {
    throw std::runtime_error(std::string("Failed to open file ") + fn);
  }

  posture_.clear();

  // Read posture
  while (file.good()) {
    std::getline(file, line);
    ++lineNumber;
    tokenizer_t tok(line, escaped_list_separator<char>('\\', ' ', '\"'));
    std::vector<double> components;
    for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
      components.push_back(atof(it->c_str()));
    }
    if (components.size() == 0) {
      break;
    }
    if (postureSize == -2) {
      postureSize = static_cast<int>(components.size() - 1);
    } else {
      if (postureSize != static_cast<int>(components.size()) - 1) {
        std::ostringstream oss;
        oss << fn << ", line " << lineNumber << ": config of size " << components.size() - 1 << ". Expecting "
            << postureSize << ".";
        throw std::runtime_error(oss.str());
      }
    }
    dg::Vector config(static_cast<unsigned>(components.size() - 1));
    for (unsigned i = 1; i < components.size(); ++i) {
      config(i - 1) = components[i];
    }
    posture_.push_back(config);
  }
  file.close();
}

void SimpleSeqPlay::start() {
  if (state_ == 0) {
    state_ = 1;
    startTime_ = postureSOUT_.getTime();
  }
}

dg::Vector& SimpleSeqPlay::computePosture(dg::Vector& pos, int t) {
  std::size_t configId;
  // If we are still waiting to start
  if (state_ == 0) {
    // return the current posture.
    pos = currentPostureSIN_.access(t);
    return pos;
  }
  if (posture_.size() == 0) {
    throw std::runtime_error("SimpleSeqPlay posture: Signals not initialized. read files first.");
  }

  // Going to the first position
  if (state_ == 1) {
    // Compute the difference between current posture and desired one.
    dg::Vector deltapos = posture_[0] - currentPostureSIN_.access(t);

    // If sufficiently closed to the first posture of the seqplay.
    if ((deltapos.norm() < 1e-4) || (((dt_ + 1) * it_nbs_in_state1_) > time_to_start_)) {
      // Switch to the next state.
      state_ = 2;
      startTime_ = postureSOUT_.getTime();
      pos = posture_[0];
    } else {
      // Tries to go closer to the first posture.
      deltapos = (deltapos * dt_) / (time_to_start_ - dt_ * it_nbs_in_state1_);
      pos = currentPostureSIN_.access(t) + deltapos;
      it_nbs_in_state1_++;
    }
    return pos;
  }
  // Tries to go through the list of postures.
  else if (state_ == 2) {
    configId = t - startTime_;
    if (configId == posture_.size() - 1) {
      state_ = 3;
    }
  } else {
    configId = posture_.size() - 1;
  }
  pos = posture_[configId];
  return pos;
}

bool SimpleSeqPlay::waiting() const { return state_ == 0; }
bool SimpleSeqPlay::initializing() const { return state_ == 1; }
bool SimpleSeqPlay::executing() const { return state_ == 2; }
bool SimpleSeqPlay::finished() const { return state_ == 3; }

std::string SimpleSeqPlay::getDocString() const {
  return "Provide joint references for a whole-body motion\n"
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

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleSeqPlay, "SimpleSeqPlay");
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph
