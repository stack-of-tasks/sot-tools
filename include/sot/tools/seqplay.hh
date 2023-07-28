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
  Signal<Vector, sigtime_t> postureSOUT_;
  Signal<MatrixHomogeneous, sigtime_t> leftAnkleSOUT_;
  Signal<MatrixHomogeneous, sigtime_t> rightAnkleSOUT_;
  Signal<Vector, sigtime_t> leftAnkleVelSOUT_;
  Signal<Vector, sigtime_t> rightAnkleVelSOUT_;
  Signal<Vector, sigtime_t> comSOUT_;
  Signal<Vector, sigtime_t> comdotSOUT_;
  Signal<Vector, sigtime_t> comddotSOUT_;
  Signal<Vector, sigtime_t> forceLeftFootSOUT_;
  Signal<Vector, sigtime_t> forceRightFootSOUT_;

  Signal<Vector, sigtime_t> zmpSOUT_;

  DYNAMIC_GRAPH_ENTITY_DECL();
  Seqplay(const std::string& name);

  void load(const std::string& filename);
  void start();
  virtual std::string getDocString() const;

 private:
  Vector& computePosture(Vector& pos, const sigtime_t& t);
  MatrixHomogeneous& computeLeftAnkle(MatrixHomogeneous& la,
                                      const sigtime_t& t);
  MatrixHomogeneous& computeRightAnkle(MatrixHomogeneous& ra,
                                       const sigtime_t& t);
  Vector& computeAnkleVelocity(
      Vector& velocity, const std::vector<MatrixHomogeneous>& ankleVector,
      const sigtime_t& t);
  Vector& computeLeftAnkleVel(Vector& velocity, const sigtime_t& t);
  Vector& computeRightAnkleVel(Vector& velocity, const sigtime_t& t);
  Vector& computeCom(Vector& com, const sigtime_t& t);
  Vector& computeComdot(Vector& comdot, const sigtime_t& t);
  Vector& computeComddot(Vector& comdot, const sigtime_t& t);
  Vector& computeZMP(Vector& comdot, const sigtime_t& t);
  Vector& computeForceFoot(Vector&, const std::vector<Vector>&,
                           const sigtime_t&);
  Vector& computeForceLeftFoot(Vector& force, const sigtime_t& t);
  Vector& computeForceRightFoot(Vector& force, const sigtime_t& t);

  void readAnkleFile(std::ifstream&, std::vector<MatrixHomogeneous>&,
                     const std::string&);
  void readForceFile(std::ifstream&, std::vector<Vector>&, const std::string&);
  // 0: motion not started, 1: motion in progress, 2: motion finished
  unsigned int state_;
  unsigned int configId_;
  sigtime_t startTime_;

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
