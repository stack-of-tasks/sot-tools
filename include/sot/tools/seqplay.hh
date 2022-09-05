//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux, Mehdi Benallegue
//

#ifndef SOT_TOOLS_SEQPLAY_HH
#define SOT_TOOLS_SEQPLAY_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal.h>

#include <fstream>
#include <iostream>
#include <sot/core/matrix-geometry.hh>
#include <sstream>

namespace dynamicgraph {
namespace sot {
namespace tools {
using dynamicgraph::Entity;
using dynamicgraph::Signal;
using dynamicgraph::Vector;
using dynamicgraph::sot::MatrixHomogeneous;

class Seqplay : public Entity {
  Signal<Vector, int> postureSOUT_;
  Signal<MatrixHomogeneous, int> leftAnkleSOUT_;
  Signal<MatrixHomogeneous, int> rightAnkleSOUT_;
  Signal<Vector, int> leftAnkleVelSOUT_;
  Signal<Vector, int> rightAnkleVelSOUT_;
  Signal<Vector, int> comSOUT_;
  Signal<Vector, int> comdotSOUT_;
  Signal<Vector, int> comddotSOUT_;
  Signal<Vector, int> forceLeftFootSOUT_;
  Signal<Vector, int> forceRightFootSOUT_;

  Signal<Vector, int> zmpSOUT_;

  DYNAMIC_GRAPH_ENTITY_DECL();
  Seqplay(const std::string& name);

  void load(const std::string& filename);
  void start();
  virtual std::string getDocString() const;

 private:
  Vector& computePosture(Vector& pos, const int& t);
  MatrixHomogeneous& computeLeftAnkle(MatrixHomogeneous& la, const int& t);
  MatrixHomogeneous& computeRightAnkle(MatrixHomogeneous& ra, const int& t);
  Vector& computeAnkleVelocity(
      Vector& velocity, const std::vector<MatrixHomogeneous>& ankleVector,
      const int& t);
  Vector& computeLeftAnkleVel(Vector& velocity, const int& t);
  Vector& computeRightAnkleVel(Vector& velocity, const int& t);
  Vector& computeCom(Vector& com, const int& t);
  Vector& computeComdot(Vector& comdot, const int& t);
  Vector& computeComddot(Vector& comdot, const int& t);
  Vector& computeZMP(Vector& comdot, const int& t);
  Vector& computeForceFoot(Vector&, const std::vector<Vector>&, const int&);
  Vector& computeForceLeftFoot(Vector& force, const int& t);
  Vector& computeForceRightFoot(Vector& force, const int& t);

  void readAnkleFile(std::ifstream&, std::vector<MatrixHomogeneous>&,
                     const std::string&);
  void readForceFile(std::ifstream&, std::vector<Vector>&, const std::string&);
  // 0: motion not started, 1: motion in progress, 2: motion finished
  unsigned int state_;
  unsigned int configId_;
  int startTime_;

  std::vector<Vector> posture_;
  std::vector<MatrixHomogeneous> leftAnkle_;
  std::vector<MatrixHomogeneous> rightAnkle_;

  std::vector<Vector> leftAnkleDot_;
  std::vector<Vector> rightAnkleDot_;

  std::vector<Vector> com_;
  std::vector<Vector> comdot_;
  std::vector<Vector> comddot_;
  std::vector<Vector> zmp_;

  bool facultativeFound_[7];

  std::vector<Vector> forceLeftFoot_;
  std::vector<Vector> forceRightFoot_;
  std::vector<double> time_;

  // Temporary variables for internal computations
  MatrixRotation R0_, R0t_, R1_, R1R0t_;
};  // class Seqplay
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_TOOLS_SEQPLAY_HH
