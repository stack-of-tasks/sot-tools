//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#include <vector>
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
	forceLeftFootSOUT_
	("Seqplay(" + name + ")::output(vector)::forceLeftFoot"),
	forceRightFootSOUT_
	("Seqplay(" + name + ")::output(vector)::forceRightFoot"),
	state_ (0), startTime_ (0), posture_ (), leftAnkle_ (),
	rightAnkle_ (), com_ (), time_ (), R0_ (), R0t_ (), R1_ (), R1R0t_ ()
      {
	signalRegistration (postureSOUT_ << leftAnkleSOUT_ << rightAnkleSOUT_
			    << leftAnkleVelSOUT_ << rightAnkleVelSOUT_
			    << comSOUT_ << comdotSOUT_
			    << forceLeftFootSOUT_ << forceRightFootSOUT_);
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
	forceLeftFootSOUT_.setFunction
	  (boost::bind (&Seqplay::computeForceLeftFoot, this, _1, _2));
	forceRightFootSOUT_.setFunction
	  (boost::bind (&Seqplay::computeForceRightFoot, this, _1, _2));


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
	  "    - center of mass task from \"filename.com\",\n"
	  "    - force and moment in left foot from \"filename.fl\",\n"
	  "    - force and moment in right foot from \"filename.fr\".\n"
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
	std::string fn [6];
	std::ifstream file [6];
	unsigned int lineNumber = 0;
	int postureSize = -2;

	fn [0] = filename + ".posture";
	fn [1] = filename + ".la";
	fn [2] = filename + ".ra";
	fn [3] = filename + ".com";
	fn [4] = filename + ".fl";
	fn [5] = filename + ".fr";

	// Open files
	for (std::size_t i=0; i<6; i++) {
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
	forceLeftFoot_.clear ();
	forceRightFoot_.clear ();

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
	    postureSize = static_cast<int> (components.size () - 1);
	  } else {
	    if (postureSize != static_cast <int> (components.size ()) - 1) {
	      std::ostringstream oss;
	      oss << fn [0] << ", line " << lineNumber << ": config of size "
		  << components.size () - 1 << ". Expecting " << postureSize
		  << ".";
	      throw std::runtime_error (oss.str ());
	    }
	  }
	  Vector config (static_cast<unsigned> (components.size () - 1));
	  for (unsigned i = 1; i < components.size (); ++i) {
	    config (i - 1) = components[i];
	  }
	  posture_.push_back (config);
	}
	file [0].close ();

	readAnkleFile (file [1], leftAnkle_, fn [1]);
	readAnkleFile (file [2], rightAnkle_, fn [2]);

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
	  for (unsigned i = 1; i < 4; ++i) {
	    com (i-1) = components [i];
	  }
	  com_.push_back (com);
	}
	file [3].close ();

	// Read forces
	readForceFile (file [4], forceLeftFoot_, fn [4]);
	readForceFile (file [5], forceRightFoot_, fn [5]);

	// Check that size of files is the same
	if (posture_.size () != leftAnkle_.size () ||
	    posture_.size () != rightAnkle_.size () ||
	    posture_.size () != com_.size () ||
	    posture_.size () != forceLeftFoot_.size () ||
	    posture_.size () != forceRightFoot_.size ()) {
	  std::ostringstream oss;
	  oss << "Seqplay: Files should have the same number of lines. Read"
	      << std::endl;
	  oss << "  " << posture_.size () << " lines from " << filename
	      << ".posture," << std::endl;
	  oss << "  " << leftAnkle_.size () << " lines from " << filename
	      << ".la," << std::endl;
	  oss << "  " << rightAnkle_.size () << " lines from " << filename
	      << ".ra," << std::endl;
	  oss << "  " << com_.size () << " lines from " << filename << ".com,"
	      << std::endl;
	  oss << "  " << forceLeftFoot_.size () << " lines from " << filename
	      << ".fl, and"
	      << std::endl;
	  oss << "  " << forceRightFoot_.size () << " lines from " << filename
	      << ".fr.";
	  throw std::runtime_error (oss.str ());
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
	  for (unsigned i=0; i < 3; ++i) {
	    velocity (i) = (M1 (i, 3) - M0 (i, 3)) / dt;
	  }
	  M1.extract (R1_);
	  M0.extract (R0_);
	  R0t_ = R0_.transpose ();
	  R1R0t_ = R1_*R0t_;
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
	  for (unsigned i=0; i < comdot.size (); ++i) {
	    comdot (i) = (q_1 (i) - q_0 (i)) / dt;
	  }
	}
	return comdot;
      }

      Vector& Seqplay::computeForceFoot
      (Vector& force, const std::vector <Vector>& forceVector, const int& t)
      {
	if (forceVector.size () == 0) {
	  throw std::runtime_error
	    ("Seqplay: Signals not initialized. read files first.");
	}
	std::size_t configId;
	if (state_ == 0) {
	  configId = 0;
	} else if (state_ == 1) {
	  configId = t - startTime_;
	  if (configId == forceVector.size () - 1) {
	    state_ = 2;
	  }
	} else {
	  configId = posture_.size () -1;
	}
	force = forceVector [configId];
	return force;
      }

      Vector& Seqplay::computeForceLeftFoot (Vector& force, const int& t)
      {
	return computeForceFoot (force, forceLeftFoot_, t);
      }

      Vector& Seqplay::computeForceRightFoot (Vector& force, const int& t)
      {
	return computeForceFoot (force, forceRightFoot_, t);
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

      void Seqplay::readAnkleFile (std::ifstream& file,
				   std::vector <MatrixHomogeneous>& data,
				   const std::string& filename)
      {
	using boost::escaped_list_separator;
	typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
	unsigned int lineNumber = 0;
	std::string line;

	while (file.good ()) {
	  std::getline (file, line);
	  ++lineNumber;
	  tokenizer_t tok (line,
			   escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) break;
	  if (components.size () != 17) {
	    std::ostringstream oss;
	    oss << filename << ", line " << lineNumber
		<< ": expecting 17 numbers, got "
		<< components.size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  MatrixHomogeneous la;
	  std::size_t i = 1;
	  for (unsigned row = 0; row < 4; ++row) {
	    for (unsigned col = 0; col < 4; ++col) {
	      la (row, col) = components [i];
	      ++i;
	    }
	  }
	  data.push_back (la);
	}
	file.close ();
      }

      void Seqplay::readForceFile (std::ifstream& file,
				   std::vector <Vector>& data,
				   const std::string& filename)
      {
	using boost::escaped_list_separator;
	typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
	unsigned int lineNumber = 0;
	std::string line;

	while (file.good ()) {
	  std::getline (file, line);
	  ++lineNumber;
	  tokenizer_t tok (line,
			   escaped_list_separator<char>('\\', '\t', '\"'));
	  std::vector <double> components;
	  for(tokenizer_t::iterator it=tok.begin(); it!=tok.end(); ++it) {
	    components.push_back (atof (it->c_str ()));
	  }
	  if (components.size () == 0) break;
	  if (components.size () != 7) {
	    std::ostringstream oss;
	    oss << filename << ", line " << lineNumber
		<< ": expecting 7 numbers, got "
		<< components.size () << ".";
	    throw std::runtime_error (oss.str ());
	  }
	  Vector force (6);
	  for (unsigned i = 1; i < 7; ++i) {
	    force (i-1) = components [i];
	  }
	  data.push_back (force);
	}
	file.close ();
      }


      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Seqplay, "Seqplay");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

