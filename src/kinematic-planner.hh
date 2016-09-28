//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Rohan Budhiraja
//

#ifndef SOT_TOOLS_KINEMATIC_PLANNER_HH
#define SOT_TOOLS_KINEMATIC_PLANNER_HH


/* STD */
#include <string>
#include <sstream>
#include <list>
#include <complex>

/* dynamic-graph */
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-base.h>

#include <dynamic-graph/linear-algebra.h>
#include <sot/core/debug.hh>

/*Eigen*/
#include <Eigen/StdVector>
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/Splines>

/* BOOST */
//#include <boost/filesystem.hpp>

namespace dynamicgraph {  namespace sot {  namespace tools {
      
      using dynamicgraph::Entity;
      
      class KinematicPlanner : public Entity
      {
      public:
	DYNAMIC_GRAPH_ENTITY_DECL();
	typedef std::vector<Eigen::ArrayXd,Eigen::aligned_allocator<Eigen::ArrayXd> >
	stdVectorofArrayXd;

	typedef std::vector<Eigen::ArrayXXd,Eigen::aligned_allocator<Eigen::ArrayXXd> >
	stdVectorofArrayXXd;

	/* --- CONSTRUCTOR --- */
	KinematicPlanner( const std::string& name);
	virtual ~KinematicPlanner( void );
	//Sources
	Eigen::ArrayXd npSource;
	Eigen::ArrayXXd pSource1;
	Eigen::ArrayXXd pSource2;

	stdVectorofArrayXXd pSourceDelayed1;
	stdVectorofArrayXXd pSourceDelayed2;
	//Delays
	Eigen::ArrayXXd pDelay1;
	Eigen::ArrayXXd pDelay2;
	
	//Non Periodic Weights
	Eigen::ArrayXXd wNonPeriodic; //Eigen::Array<double, 480,4>
	
	//Periodic Weights
	stdVectorofArrayXXd wPeriodic1;
	stdVectorofArrayXXd wPeriodic2;

	stdVectorofArrayXXd allJointsAllGaitCycleAllTraj;

	//Mean joint angles
	Eigen::ArrayXXd mJointAngle;

	//Number of Trajectories Created
	int nTrajectories; //30
	int nJoints; //16
	int nGaitCycles; //4
	int nTimeSteps; //160
	int nSources1; //5
	int nSources2; //4
	/*! @} */
	std::list< dynamicgraph::SignalBase<int>*  > genericSignalRefs;

	//Load Motion Capture outputs
	template<typename Derived>
	void read2DArray(std::string& fileName,
			 Eigen::DenseBase<Derived>& outArr);

	void setParams( const int& _nTimeSteps, //160
			const int& _nSources, //5
			const int& _nSources2, //4
			const std::string& dir);
	void loadSourceWeightsDelays(const std::string& dir);
	void delaySources(int);
	void blending(int);
	void smoothEnds(Eigen::Ref<Eigen::ArrayXd> tr);
	void bSplineInterpolate(Eigen::ArrayXXd& tr, int factor);
	void runKinematicPlanner(void);

	void savitzkyGolayFilter(Eigen::Ref<Eigen::ArrayXXd> allJointTraj, int polyOrder, int frameSize);


	bool parametersSet;
      }; // class KinematicPlanner
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //SOT_TOOLS_KINEMATIC_PLANNER_HH
