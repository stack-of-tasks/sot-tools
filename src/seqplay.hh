//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#ifndef SOT_TOOLS_SEQPLAY_HH
# define SOT_TOOLS_SEQPLAY_HH

# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/linear-algebra.h>
# include <dynamic-graph/signal.h>
# include <sot/core/matrix-homogeneous.hh>

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
	Signal <MatrixHomogeneous, int> leftAnkleSOUT_;
	Signal <MatrixHomogeneous, int> rightAnkleSOUT_;
	Signal <Vector, int> comSOUT_;
	Signal <Vector, int> comdotSOUT_;

	DYNAMIC_GRAPH_ENTITY_DECL();
	Seqplay (const std::string& name);

	void load (const std::string& filename);
	void start ();
	virtual std::string getDocString () const;

      private:
	Vector& computePosture (Vector& pos, const int& t);
	MatrixHomogeneous& computeLeftAnkle (MatrixHomogeneous& la,
					     const int& t);
	MatrixHomogeneous& computeRightAnkle (MatrixHomogeneous& ra,
					      const int& t);
	Vector& computeCom (Vector& com, const int& t);
	Vector& computeComdot (Vector& comdot, const int& t);

	// 0: motion not started, 1: motion in progress, 2: motion finished
	unsigned int state_;
	unsigned int configId_;
	int startTime_;

	std::vector <Vector> posture_;
	std::vector <MatrixHomogeneous> leftAnkle_;
	std::vector <MatrixHomogeneous> rightAnkle_;
	std::vector <Vector> com_;
	std::vector <double> time_;
      }; // class Seqplay
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //SOT_TOOLS_SEQPLAY_HH
