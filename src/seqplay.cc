//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <boost/tokenizer.hpp>

#include <dynamic-graph/command-bind.h>

#include "seqplay.hh"

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      using dynamicgraph::Entity;
      using dynamicgraph::command::makeCommandVoid0;
      using dynamicgraph::command::docCommandVoid0;
      using dynamicgraph::command::makeCommandVoid1;
      using dynamicgraph::command::docCommandVoid1;

      Seqplay::Seqplay (const std::string& name) :
	Entity (name),
	postureSOUT_ ("Seqplay(" + name + ")::output(vector)::posture"),
	leftAnkleSOUT_ ("Seqplay(" + name + ")::output(MatrixHomo)::leftAnkle"),
	rightAnkleSOUT_
	("Seqplay(" + name + ")::output(MatrixHomo)::rightAnkle"),
	leftAnkleVelSOUT_
	("Seqplay(" + name + ")::output(Vector)::leftAnkleVel"),
	rightAnkleVelSOUT_
	("Seqplay(" + name + ")::output(Vector)::rightAnkleVel"),
	comSOUT_ ("Seqplay(" + name + ")::output(vector)::com"),
	comdotSOUT_ ("Seqplay(" + name + ")::output(vector)::comdot"),
	state_ (0), startTime_ (0), posture_ (), leftAnkle_ (),
	rightAnkle_ (), com_ (), time_ (), R0_ (), R0t_ (), R1_ (), R1R0t_ ()
      {
	signalRegistration (postureSOUT_ << leftAnkleSOUT_ << rightAnkleSOUT_
			    << leftAnkleVelSOUT_ << rightAnkleVelSOUT_
			    << comSOUT_ << comdotSOUT_);
	postureSOUT_.setFunction (boost::bind (&Seqplay::computePosture,
					       this, _1, _2));
	comSOUT_.setFunction (boost::bind (&Seqplay::computeCom, this, _1, _2));
	leftAnkleSOUT_.setFunction (boost::bind (&Seqplay::computeLeftAnkle,
						 this, _1, _2));
	rightAnkleSOUT_.setFunction (boost::bind (&Seqplay::computeRightAnkle,
						  this, _1, _2));
	leftAnkleVelSOUT_.setFunction
	  (boost::bind (&Seqplay::computeLeftAnkleVel, this, _1, _2));
	rightAnkleVelSOUT_.setFunction
	  (boost::bind (&Seqplay::computeRightAnkleVel, this, _1, _2));
	comdotSOUT_.setFunction (boost::bind (&Seqplay::computeComdot, this, _1,
					      _2));

	std::string docstring =
	  "Load files describing a whole-body motion as reference feature "
	  "trajectories\n"
	  "\n"
	  "  Input:\n"
	  "    - string filename without extension\n"
	  "\n"
	  "  Data is read from the following files:\n"
	  "    - posture from file \"filename.posture\",\n"
	  "    - left ankle task from \"filename.la\",\n"
	  "    - right ankle task from \"filename.ra\",\n"
	  "    - center of mass task from \"filename.com\".\n"
	  "  Each file should contain one column for time and as many columns as"
	  " required\n"
	  "  depending on data-type, i.e.:\n"
	  "    - 17 for homogeneous matrices,\n"
	  "    -  4 for center of mass.\n"
	  "\n";
	addCommand ("load",
		    makeCommandVoid1 (*this, &Seqplay::load,
				      docstring));

	addCommand ("start",
		    makeCommandVoid0 (*this, &Seqplay::start,
				      docCommandVoid0 ("Start motion")));
      }

      void Seqplay::load (const std::string& filename)
      {
	using boost::escaped_list_separator;
	typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
	std::string line;
	std::string fn [4];
	std::ifstream file [4];
	unsigned int lineNumber = 0;
	int postureSize = -2;

	fn [0] = filename + ".posture";
	fn [1] = filename + ".la";
	fn [2] = filename + ".ra";
	fn [3] = filename + ".com";

	// Open files
	for (std::size_t i=0; i<4; i++) {
	  file [i].open (fn [i].c_str ());
	  if (!file [i].is_open ()) {
	    throw std::runtime_error (std::string ("Failed to open file ") +
				      fn [i]);
	  }
	}
	posture_.clear ();
	leftAnkle_.clear ();
	rightAnkle_.clear ();
	com_.clear ();

	// Read posture
	while (file [0].good ()) {
	  std::getline (file [0], line);
	  ++lineNumber;
	  tokenizer_t tok (line, escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) {
	    break;
	  }
	  if (postureSize == -2) {
	    postureSize = components.size () - 1;
	  } else {
	    if (postureSize != static_cast <int> (components.size ()) - 1) {
	      std::ostringstream oss;
	      oss << fn [0] << ", line " << lineNumber << ": config of size "
		  << components.size () - 1 << ". Expecting " << postureSize
		  << ".";
	      throw std::runtime_error (oss.str ());
	    }
	  }
	  Vector config (components.size () - 1);
	  for (std::size_t i=1; i<components.size (); ++i) {
	    config (i-1) = components [i];
	  }
	  posture_.push_back (config);
	}
	file [0].close ();

	// Read left ankle
	lineNumber = 0;
	while (file [1].good ()) {
	  std::getline (file [1], line);
	  ++lineNumber;
	  tokenizer_t tok (line, escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) break;
	  if (components.size () != 17) {
	    std::ostringstream oss;
	    oss << fn [1] << ", line " << lineNumber
		<< ": expecting 17 numbers, got "
		<< components.size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  MatrixHomogeneous la;
	  std::size_t i = 1;
	  for (std::size_t row = 0; row < 4; ++row) {
	    for (std::size_t col = 0; col < 4; ++col) {
	      la (row, col) = components [i];
	      ++i;
	    }
	  }
	  leftAnkle_.push_back (la);
	}
	file [1].close ();

	// Read right ankle
	lineNumber = 0;
	while (file [2].good ()) {
	  std::getline (file [2], line);
	  ++lineNumber;
	  tokenizer_t tok (line, escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) break;
	  if (components.size () != 17) {
	    std::ostringstream oss;
	    oss << fn [2] << ", line " << lineNumber
		<< ": expecting 17 numbers, got"
		<< components.size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  MatrixHomogeneous la;
	  std::size_t i = 1;
	  for (std::size_t row = 0; row < 4; ++row) {
	    for (std::size_t col = 0; col < 4; ++col) {
	      la (row, col) = components [i];
	      ++i;
	    }
	  }
	  rightAnkle_.push_back (la);
	}
	file [2].close ();

	// Read com
	lineNumber = 0;
	while (file [3].good ()) {
	  std::getline (file [3], line);
	  ++lineNumber;
	  tokenizer_t tok (line, escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) break;
	  Vector com (3);
	  if (components.size () != 4) {
	    std::ostringstream oss;
	    oss << fn [3] << ", line " << lineNumber << ": expecting 4 numbers.";
	    throw std::runtime_error (oss.str ());
	  }
	  time_.push_back (components [0]);
	  for (std::size_t i=1; i<4; ++i) {
	    com (i-1) = components [i];
	  }
	  com_.push_back (com);
	}
	file [3].close ();

	// Check that size of files is the same
	if (posture_.size () != leftAnkle_.size () ||
	    posture_.size () != rightAnkle_.size () ||
	    posture_.size () != com_.size ()) {
	  throw std::runtime_error
	    ("Seqplay: Files should be of same number of lines.");
	}
	state_ = 0;
      }

      void Seqplay::start ()
      {
	if (state_ == 0) {
	  state_ = 1;
	  startTime_ = std::max (std::max (comSOUT_.getTime (),
					   comdotSOUT_.getTime ()),
				 std::max (leftAnkleSOUT_.getTime (),
					   rightAnkleSOUT_.getTime ()));
	  startTime_ = std::max (startTime_, postureSOUT_.getTime ());
	}
      }

      Vector& Seqplay::computePosture (Vector& pos, const int& t)
      {
	if (posture_.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == posture_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	pos = posture_ [configId];
	return pos;
      }

      MatrixHomogeneous& Seqplay::computeLeftAnkle (MatrixHomogeneous& la,
						    const int& t)
      {
	if (leftAnkle_.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == posture_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	la = leftAnkle_ [configId];
	return la;
      }

      MatrixHomogeneous& Seqplay::computeRightAnkle (MatrixHomogeneous& ra,
						     const int& t)
      {
	if (rightAnkle_.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == posture_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	ra = rightAnkle_ [configId];
	return ra;
      }

      Vector& Seqplay::computeAnkleVelocity
      (Vector& velocity, const std::vector <MatrixHomogeneous>& ankleVector,
       const int& t)
      {
	velocity.resize (6); velocity.setZero ();
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == com_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}

	if ((0 < configId) && (configId < com_.size () - 1)) {
	  const MatrixHomogeneous& M1 = ankleVector [configId];
	  const MatrixHomogeneous& M0 = ankleVector [configId - 1];
	  double dt = time_ [configId] - time_ [configId -1];
	  for (std::size_t i=0; i < 3; ++i) {
	    velocity (i) = (M1 (i, 3) - M0 (i, 3)) / dt;
	  }
	  M1.extract (R1_);
	  M0.extract (R0_);
	  R0_.transpose (R0t_);
	  R1_.multiply (R0t_, R1R0t_);
	  velocity (3) = (R1R0t_ (2, 1))/dt;
	  velocity (4) = (R1R0t_ (0, 2))/dt;
	  velocity (5) = (R1R0t_ (1, 0))/dt;
	}

	return velocity;
      }

      Vector& Seqplay::computeLeftAnkleVel (Vector& velocity, const int& t)
      {
	return computeAnkleVelocity (velocity, leftAnkle_, t);
      }

      Vector& Seqplay::computeRightAnkleVel (Vector& velocity, const int& t)
      {
	return computeAnkleVelocity (velocity, rightAnkle_, t);
      }

      Vector& Seqplay::computeCom (Vector& com, const int& t)
      {
	if (com_.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == com_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	com = com_ [configId];
	return com;
      }

      Vector& Seqplay::computeComdot (Vector& comdot, const int& t)
      {
	if (com_.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == com_.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	comdot.resize (com_ [0].size ()); comdot.setZero ();
	if ((0 < configId) && (configId < com_.size () - 1)) {
	  const Vector& q_1 = com_ [configId];
	  const Vector& q_0 = com_ [configId - 1];
	  double dt = time_ [configId] - time_ [configId -1];
	  for (std::size_t i=0; i < comdot.size (); ++i) {
	    comdot (i) = (q_1 (i) - q_0 (i)) / dt;
	  }
	}
	return comdot;
      }

      std::string Seqplay::getDocString () const
      {
	return
	  "Provide task references for a whole-body motion\n"
	  "\n"
	  "  The reference trajectories of the following features is loaded from files\n"
	  "  using command load:\n"
	  "    - posture,\n"
	  "    - left ankle,\n"
	  "    - right ankle, and\n"
	  "    - center of mass.\n"
	  "\n"
	  "  To use this entity,\n"
	  "    1. call method load,\n"
	  "    2. plug reference signals into robot signals and\n"
	  "    3. call method start.\n"
	  "  Warning: pluging signals before loading trajectories will fail.\n";
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Seqplay, "Seqplay");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

