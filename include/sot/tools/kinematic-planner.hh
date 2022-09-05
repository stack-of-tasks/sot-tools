//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Rohan Budhiraja
//

#ifndef SOT_TOOLS_KINEMATIC_PLANNER_HH
#define SOT_TOOLS_KINEMATIC_PLANNER_HH

/* STD */
#include <complex>
#include <list>
#include <sstream>
#include <string>

/* dynamic-graph */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-base.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>

#include <sot/core/debug.hh>

/*Eigen*/
#include <Eigen/StdVector>
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/Splines>

/* BOOST */
//#include <boost/filesystem.hpp>

namespace dynamicgraph {
namespace sot {
namespace tools {

using dynamicgraph::Entity;

class KinematicPlanner : public Entity {
 public:
  DYNAMIC_GRAPH_ENTITY_DECL();
  typedef std::vector<Eigen::ArrayXd, Eigen::aligned_allocator<Eigen::ArrayXd> >
      stdVectorofArrayXd;

  typedef std::vector<Eigen::ArrayXXd,
                      Eigen::aligned_allocator<Eigen::ArrayXXd> >
      stdVectorofArrayXXd;

  /*-----SIGNALS--------*/
  typedef int Dummy;
  /*
    dynamicgraph::SignalPtr<double,int> distToDrawerSIN;
    dynamicgraph::SignalPtr<double,int> objectPositionInDrawerSIN;

    dynamicgraph::SignalTimeDependent<Dummy,int> trajectoryReadySINTERN;

    dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, int>
    upperBodyJointPositionSOUT;
    dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, int>
    upperBodyJointVelocitySOUT;
    dynamicgraph::SignalTimeDependent<dynamicgraph::Matrix, int>
    freeFlyerVelocitySOUT;
  */
  /* --- CONSTRUCTOR --- */
  KinematicPlanner(const std::string& name);
  virtual ~KinematicPlanner(void);
  // Sources
  Eigen::ArrayXd npSource;
  Eigen::ArrayXXd pSource1;
  Eigen::ArrayXXd pSource2;

  stdVectorofArrayXXd pSourceDelayed1;
  stdVectorofArrayXXd pSourceDelayed2;
  // Delays
  Eigen::ArrayXXd pDelay1;
  Eigen::ArrayXXd pDelay2;

  // Non Periodic Weights
  Eigen::ArrayXXd wNonPeriodic;  // Eigen::Array<double, 480,4>

  // Periodic Weights
  stdVectorofArrayXXd wPeriodic1;
  stdVectorofArrayXXd wPeriodic2;

  // Mean joint angles
  Eigen::ArrayXXd mJointAngle;

  // Number of Trajectories Created
  //	int nTrajectories; //30
  int nJoints;      // 16
  int nGaitCycles;  // 4
  int nTimeSteps;   // 160
  int nSources1;    // 5
  int nSources2;    // 4
  /*! @} */
  std::list<dynamicgraph::SignalBase<int>*> genericSignalRefs;

  // Load Motion Capture outputs
  template <typename Derived>
  void read2DArray(std::string& fileName, Eigen::DenseBase<Derived>& outArr);

  void setParams(const double& _distanceToDrawer,
                 const double& _objectPositionInDrawer, const std::string& dir);
  void loadSourceDelays(const std::string& dir);
  void loadTrainingParams(const std::string& dir, dynamicgraph::Matrix& q,
                          dynamicgraph::Matrix& beta3, Eigen::ArrayXd& mwwn,
                          double& sigma2, int& N, int& K);
  dynamicgraph::Vector createSubGoals(double D, double P);
  void delaySources();
  void blending();
  void smoothEnds(Eigen::Ref<Eigen::ArrayXd> tr);
  void bSplineInterpolate(Eigen::ArrayXXd& tr, int factor);
  int& runKinematicPlanner(int& dummy, int time);
  void goalAdaption(dynamicgraph::Vector& goals, const std::string&);
  void savitzkyGolayFilter(Eigen::Ref<Eigen::ArrayXXd> allJointTraj,
                           int polyOrder, int frameSize);

  bool parametersSet;
};  // class KinematicPlanner
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph

#endif  // SOT_TOOLS_KINEMATIC_PLANNER_HH
