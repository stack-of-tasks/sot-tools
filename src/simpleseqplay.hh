//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// From Author: Florent Lamiraux, Mehdi Benallegue, 
// Author: Olivier Stasse
// Simple sequence player just playing back a set of poses.
//

#ifndef SOT_TOOLS_SIMPLE_SEQPLAY_HH
# define SOT_TOOLS_SIMPLE_SEQPLAY_HH

# include <iostream>
# include <sstream>
# include <fstream>

# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/linear-algebra.h>
# include <dynamic-graph/signal.h>
# include <sot/core/matrix-homogeneous.hh>
# include <sot/core/matrix-rotation.hh>

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      using dynamicgraph::Entity;
      using dynamicgraph::Vector;
      using dynamicgraph::Signal;
      using dynamicgraph::sot::MatrixHomogeneous;

      class Seqplay : public Entity
      {
	Signal <Vector, int> postureSOUT_;

	DYNAMIC_GRAPH_ENTITY_DECL();
	Seqplay (const std::string& name);

	void load (const std::string& filename);
	void start ();
	virtual std::string getDocString () const;

      private:
	Vector& computePosture (Vector& pos, const int& t);
	// 0: motion not started, 1: motion in progress, 2: motion finished
	unsigned int state_;
	unsigned int configId_;
	int startTime_;

	std::vector <Vector> posture_;

	bool facultativeFound_[7];

	std::vector <double> time_;

      }; // class Seqplay
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //SOT_TOOLS_SIMPLESEQPLAY_HH
