//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Rohan Budhiraja
//

#include "kinematic-planner.hh"
#include <dynamic-graph/command-bind.h>
#include <fstream>
#include <sstream>
#include <time.h>
namespace dynamicgraph
{  namespace sot
  {   namespace tools {

  KinematicPlanner::KinematicPlanner (const std::string& name) :
    Entity (name)
  {
    sotDEBUGIN(5);

    //
    // Commands
    //
    // setParams
    const std::string docstring1 =
      "\n"
      "    Set the parameters for the Planner.\n"
      "\n"
      "      Input:\n"
      "        - INT: Number of Time Steps in one 1.6 s \n"
      "        - INT: Number of periodic sources 1 \n"
      "        - INT: Number of periodic sources 2 \n"
      "        - STRING: Path of the directory from where parameters can be loaded \n"
      "\n";
     addCommand("setParams",
    	       dynamicgraph::command::makeCommandVoid4(*this,
    						       &KinematicPlanner::setParams,
    						       docstring1));
    
    // runKinematicPlanner
    const std::string docstring2 =
      "\n"
      "    Run the Planner to generate upper joint trajectories \n"
      "\n"
      "      Input:\n"
      "        - none \n"
      "\n";
    addCommand("runKinematicPlanner",
		 dynamicgraph::command::makeCommandVoid0(*this,
				(void (KinematicPlanner::*)
				 (void))&KinematicPlanner::runKinematicPlanner,
				docstring2));
     parametersSet = false;

     //std::string dir = "/local/rbudhira/git/proyan/sot-tools/src/params2";
     //setParams(320 , 5 , 4, dir);
     //delaySources();
     //blending();

     sotDEBUGOUT(5);
   }

   KinematicPlanner::~KinematicPlanner( void ) {
     sotDEBUGIN(15);

     for( std::list< SignalBase<int>* >::iterator iter = genericSignalRefs.begin();
	  iter != genericSignalRefs.end();
	  ++iter) {
       SignalBase<int>* sigPtr = *iter;
       delete sigPtr;
     }
     sotDEBUGOUT(15);
   }

   namespace {
     template <typename Derived>
     void writeToFile(Eigen::DenseBase<Derived>& inMatrix, std::string& filePath){
       std::ofstream file(filePath.c_str());
       if (file.is_open())
	 {
	   file << inMatrix;
	 }	  
     }
   }


   template<typename Derived>
   void KinematicPlanner::read2DArray(std::string& fileName,
				      Eigen::DenseBase<Derived>& outArr){
     sotDEBUGIN(5);
     std::ifstream fileStream(fileName.c_str());
     std::string curLine;
     double _dbl;
     std::string _dblString;
     int _row=0;
     while (std::getline(fileStream,curLine)){ //'\n' delimited rows
       std::istringstream iss(curLine);
       int _col =0;
       while(std::getline(iss,_dblString,'\t')){  // tab-delimited columns
	 _dbl = atof(_dblString.c_str());
	 outArr(_row,_col) = _dbl;
	 _col++;
       }
       _row++;
     }
     sotDEBUGOUT(5);
     return;
   }

   void KinematicPlanner::setParams( const int& _nTimeSteps, //160
				     const int& _nSources1, //5
				     const int& _nSources2, //4
				     const std::string& dir) {
     sotDEBUGIN(5);

     //Currently Fixed Parameters
     //TODO: Are these variables?
     nTrajectories = 30;
     nJoints = 16;
     nGaitCycles = 4;

     nTimeSteps = _nTimeSteps; //160
     nSources1 = _nSources1; //5
     nSources2 = _nSources2; //4

     npSource.resize(nTimeSteps);
     pSource1.resize(nTimeSteps,nSources1);
     pSource2.resize(nTimeSteps,nSources2);

     pSourceDelayed1.resize(nJoints);
     pSourceDelayed2.resize(nJoints);

     //Delays
     pDelay1.resize(nJoints*nTrajectories,nSources1);
     pDelay2.resize(nJoints*nTrajectories,nSources2);

     //Non Periodic Weights
     wNonPeriodic.resize(nJoints*nTrajectories,nGaitCycles);

     //Periodic Weights
     wPeriodic1.resize(nSources1);
     for(int i=0; i<nSources1; i++)
       wPeriodic1.at(i).resize(nJoints*nTrajectories,nGaitCycles);

     wPeriodic2.resize(nSources2);
     for(int i=0; i<nSources2; i++)
       wPeriodic2.at(i).resize(nJoints*nTrajectories,nGaitCycles);

     //Mean joint angles
     mJointAngle.resize(nJoints*nTrajectories,nGaitCycles);

     loadSourceWeightsDelays(dir);

     parametersSet = true;
     sotDEBUGOUT(5);
     return;
   }

   void KinematicPlanner::loadSourceWeightsDelays(const std::string& dir) {
     sotDEBUGIN(5);

     //assert(boost::filesystem::is_directory(dir));
     //Load Source1
     std::string fileName = dir +"/"+"pSource1";
     read2DArray<Eigen::ArrayXXd>(fileName, pSource1);
     assert(pSource1.cols() == nSources1);
     assert(pSource1.rows() == nTimeSteps);

     //Load Source2
     fileName = dir+"/"+"pSource2";
     read2DArray<Eigen::ArrayXXd>(fileName, pSource2);
     assert(pSource2.cols() == nSources2);
     assert(pSource2.rows() == nTimeSteps);

     //Load Non-Periodic Source
     fileName = dir+"/"+"npSource";
     read2DArray<Eigen::ArrayXd>(fileName,npSource);
     assert(npSource.cols() == 1);
     assert(npSource.rows() == nTimeSteps);

     //Load Delays: Source1
     fileName = dir+"/"+"pDelay1";
     read2DArray<Eigen::ArrayXXd>(fileName,pDelay1);
     assert(pDelay1.cols() == nSources1);
     assert(pDelay1.rows() == nTrajectories*nJoints);

     //Load Delays: Source2
     fileName = dir+"/"+"pDelay2";
     read2DArray<Eigen::ArrayXXd>(fileName,pDelay2);
     assert(pDelay2.cols() == nSources2);
     assert(pDelay2.rows() == nTrajectories*nJoints);

     //Load mean Joint Angles
     fileName = dir+"/"+"mJointAngle";
     read2DArray<Eigen::ArrayXXd>(fileName , mJointAngle);
     assert(mJointAngle.cols() == nGaitCycles);
     assert(mJointAngle.rows() == nTrajectories*nJoints);

     //Load Joint Weights: non periodic
     fileName = dir+"/"+"wNonPeriodic";
     read2DArray<Eigen::ArrayXXd>(fileName , wNonPeriodic);
     assert(mJointAngle.cols() == nGaitCycles);
     assert(mJointAngle.rows() == nTrajectories*nJoints);

     //Load Joint weights1:
     fileName = dir+"/"+"wPeriodic1";
     wPeriodic1.resize(nSources1);
     std::string specificFile;
     for (int cnt = 0; cnt< nSources1 ; cnt++){
       std::ostringstream stm;
       stm << cnt;
       specificFile = fileName + "/" + stm.str();
       read2DArray<Eigen::ArrayXXd>(specificFile, wPeriodic1.at(cnt));
       assert(wPeriodic1.at(cnt).cols() == nGaitCycles);
       assert(wPeriodic1.at(cnt).rows() == nTrajectories*nJoints);
     }    
     //Load Joint weights2:
     fileName = dir+"/"+"wPeriodic2";
     wPeriodic2.resize(nSources2);
     for (int cnt = 0; cnt< nSources2; cnt++){
       std::ostringstream stm;
       stm << cnt;
       specificFile = fileName + "/" + stm.str();
       read2DArray<Eigen::ArrayXXd>(specificFile, wPeriodic2.at(cnt));
       assert(wPeriodic2.at(cnt).cols() == nGaitCycles);
       assert(wPeriodic2.at(cnt).rows() == nTrajectories*nJoints);
     }
     sotDEBUGOUT(5);
     return;
   }

   void KinematicPlanner::delaySources(int _currentTrajectory) {
     //TODO: Confirm if this is HalfSpectrum or not
     sotDEBUGIN(5);
     pSourceDelayed1.clear();
     pSourceDelayed2.clear();

     int cT = _currentTrajectory;

     Eigen::FFT<double> fft;
     double pi = std::acos(-1);
     //Prepare for the Delay Exponential
     Eigen::ArrayXcd V(nTimeSteps);
     for(int cur_time = 0;cur_time<nTimeSteps; cur_time++){
       std::complex<double> pi_freq; pi_freq.real() = 0;	    pi_freq.imag() = cur_time;
       V(cur_time) = pi_freq;
     }
     V *= 2*pi/nTimeSteps;

       //Do the FFT Source1
     Eigen::ArrayXXcd pSourceInFreq(nTimeSteps, nSources1);
     for (int cur_src = 0; cur_src < nSources1; cur_src++) {
       pSourceInFreq.matrix().col(cur_src) = fft.fwd(pSource1.matrix().col(cur_src));
     }

     //Do the Delay
     for (int cJ = 0; cJ<nJoints; cJ++) {
       Eigen::ArrayXXd pSourceDelayedJoint(nTimeSteps,nSources1);
       for (int cur_src = 0; cur_src < nSources1; cur_src++) {
	 // DelayedFt = Source(w) * exp^(V*pDelay1(joint, source))
	 const Eigen::ArrayXcd delayedFT = 
	   pSourceInFreq.col(cur_src)*
	   (V*pDelay1(cT*nJoints+cJ, cur_src)).exp();
	 //(V*pDelay1(cT*nJoints+cJ, cur_src)).unaryExpr<std::complex(*)(std::complex)>(&std::exp);
	 //FFT Inv
	 pSourceDelayedJoint.matrix().col(cur_src) = fft.inv(delayedFT.matrix());
       }
       pSourceDelayed1.push_back(pSourceDelayedJoint);
     }

       //Do the FFT Source 2
     pSourceInFreq.resize(Eigen::NoChange, nSources2);
     for (int cur_src = 0; cur_src < nSources2; cur_src++) {
       pSourceInFreq.matrix().col(cur_src) = fft.fwd(pSource2.matrix().col(cur_src));
     }

     //Do the Delay
     for (int cJ = 0; cJ<nJoints; cJ++) {
       Eigen::ArrayXXd pSourceDelayedJoint(nTimeSteps,nSources2);
       for (int cur_src = 0; cur_src < nSources2; cur_src++) {
	 // DelayedFt = Source(w) * exp^(V*pDelay1(joint, source))
	 const Eigen::ArrayXcd delayedFT = 
	   pSourceInFreq.col(cur_src)*
	   (V*pDelay2(cT*nJoints+cJ, cur_src)).exp();
	   //(V*pDelay2(cT*nJoints+cJ, cur_src)).unaryExpr<std::complex(*)(std::complex)>(&std::exp);
	 //FFT Inv
	 pSourceDelayedJoint.matrix().col(cur_src) = fft.inv(delayedFT.matrix());
       }
       pSourceDelayed2.push_back(pSourceDelayedJoint);
     }
     /*
       std::stringstream s;
       s << cT;
       std::string outputPath = 
       "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+s.str()+"delayedSource10.txt";
       writeToFile<Eigen::ArrayXXd>(pSourceDelayed1.at(10), outputPath);
     */    

     sotDEBUGOUT(5);
     return;
   }

   void KinematicPlanner::blending(int _currentTrajectory) {
     sotDEBUGIN(5);
     int cT = _currentTrajectory;
     Eigen::ArrayXd wCurrent;
     Eigen::ArrayXd mJointAngleCurrent;

     double pi = std::acos(-1);
     int halfTime = (int)nTimeSteps/2;
     bool specialInterpolatedWeight = false;

     Eigen::ArrayXXd allJointsOneGaitCycle(nTimeSteps,nJoints);
     Eigen::ArrayXXd allJointsAllGaitCycle(nTimeSteps*nGaitCycles,nJoints);
     allJointsOneGaitCycle.setZero();
     allJointsAllGaitCycle.setZero();

     for(int cG=0;cG<nGaitCycles;cG++) { //nGaitCycles = 4
       stdVectorofArrayXXd tobeprinted;
       
       for (int t=0;t<nTimeSteps;t++) { //nTimeSteps = 160
	 stdVectorofArrayXd currentWeight1;
	 stdVectorofArrayXd currentWeight2;
	 if (t<nTimeSteps/10){
	   //Matlab starts from 1. TODO:initial blending interpolation check
	   //Beginning of the step
	   if(cG!=0){
	     specialInterpolatedWeight = true;
	     //Not for the first step
	     //non periodic source intermediate value
	     Eigen::ArrayXd wnpbeg =
	       (npSource(nTimeSteps-1)*wNonPeriodic.col(cG-1).segment(cT*nJoints,nJoints)+
		npSource(0)*wNonPeriodic.col(cG).segment(cT*nJoints,nJoints))/2;
	     //interpolation weight
	     double ww = (std::sin(t/(nTimeSteps/10)*pi/2)+1)/2;
	     wCurrent =
	       (2*ww-1)*wNonPeriodic.col(cG).segment(cT*nJoints,nJoints)
	       +(1-(2*ww-1))*wnpbeg;
	     mJointAngleCurrent = 
	       ww*mJointAngle.col(cG).segment(cT*nJoints,nJoints)
	       +(1-ww)*mJointAngle.col(cG-1).segment(cT*nJoints,nJoints);

	     for (int cSrc1 = 0; cSrc1<nSources1; cSrc1++){
	       Eigen::ArrayXd intWeightCSrc =
		 ww*wPeriodic1.at(cSrc1).col(cG).segment(cT*nJoints,nJoints)
		 +(1-ww)*wPeriodic1.at(cSrc1).col(cG-1).segment(cT*nJoints,nJoints);
	       currentWeight1.push_back(intWeightCSrc);
	     }

	     for (int cSrc2 = 0; cSrc2<nSources2; cSrc2++){
	       Eigen::ArrayXd intWeightCSrc =
		 ww*wPeriodic2.at(cSrc2).col(cG).segment(cT*nJoints,nJoints)
		 +(1-ww)*wPeriodic2.at(cSrc2).col(cG-1).segment(cT*nJoints,nJoints);
	       currentWeight2.push_back(intWeightCSrc);
	     }

	   }
	 }

	 else if (t>nTimeSteps*9/10) {
	   //End of Step
	   if(cG!=nGaitCycles-1) {
	     //Not for the last gait cycle
	     specialInterpolatedWeight = true;
	     Eigen::ArrayXd wnpEnd =
	       -(npSource(nTimeSteps-1)*wNonPeriodic.col(cG).segment(cT*nJoints,nJoints)+
		 npSource(0)*wNonPeriodic.col(cG+1).segment(cT*nJoints,nJoints))/2;
	     double ww = (std::sin((t-nTimeSteps)/(nTimeSteps/10)*pi/2)+1)/2;
	     wCurrent =
	       (1-2*ww)*wNonPeriodic.col(cG).segment(cT*nJoints,nJoints)+2*ww*wnpEnd;
	     mJointAngleCurrent = 		  
	       (1-ww)*mJointAngle.col(cG).segment(cT*nJoints,nJoints)
	       +ww*mJointAngle.col(cG+1).segment(cT*nJoints,nJoints);

	     for (int cSrc1 = 0; cSrc1<nSources1; cSrc1++){
	       Eigen::ArrayXd intWeightCSrc =
		 (1-ww)*wPeriodic1.at(cSrc1).col(cG).segment(cT*nJoints,nJoints)
		 +ww*wPeriodic1.at(cSrc1).col(cG+1).segment(cT*nJoints,nJoints);
	       currentWeight1.push_back(intWeightCSrc);
	     }

	     for (int cSrc2 = 0; cSrc2<nSources2; cSrc2++){
	       Eigen::ArrayXd intWeightCSrc =
		 (1-ww)*wPeriodic2.at(cSrc2).col(cG).segment(cT*nJoints,nJoints)
		 +ww*wPeriodic2.at(cSrc2).col(cG+1).segment(cT*nJoints,nJoints);
	       currentWeight2.push_back(intWeightCSrc);
	     }
	   } //(cG!=nGaitCycles-1)
	 } //(t>nTimeSteps*9/10)


	   // Start Mixing all sources
	 for(int cJ=0;cJ<nJoints;cJ++) {
	   if(specialInterpolatedWeight){
	     for (int cSrc1 = 0; cSrc1<nSources1; cSrc1++)
	       allJointsOneGaitCycle(t,cJ) +=
		 pSourceDelayed1.at(cJ)(t,cSrc1)*currentWeight1.at(cSrc1)(cJ);
	     for (int cSrc2 = 0; cSrc2<nSources2; cSrc2++)
	       allJointsOneGaitCycle(t,cJ) +=
		 pSourceDelayed2.at(cJ)(t,cSrc2)*currentWeight2.at(cSrc2)(cJ);

	     allJointsOneGaitCycle(t,cJ) += npSource(t)*wCurrent(cJ);
	     allJointsOneGaitCycle(t,cJ) += mJointAngleCurrent(cJ);

	   } else {
	     for (int cSrc1 = 0; cSrc1<nSources1; cSrc1++)
	       allJointsOneGaitCycle(t,cJ) +=
		 pSourceDelayed1.at(cJ)(t,cSrc1)*wPeriodic1.at(cSrc1)(cT*nJoints+cJ,cG);
	     for (int cSrc2 = 0; cSrc2<nSources2; cSrc2++)
	       allJointsOneGaitCycle(t,cJ) +=
		 pSourceDelayed2.at(cJ)(t,cSrc2)*wPeriodic2.at(cSrc2)(cT*nJoints+cJ,cG);

	     allJointsOneGaitCycle(t,cJ) += npSource(t)*wNonPeriodic(cT*nJoints+cJ,cG);
	     allJointsOneGaitCycle(t,cJ) += mJointAngle(cT*nJoints+cJ,cG);

	   }
	 }

	 specialInterpolatedWeight = false;
       } //Finish iterate through nTimeSteps


       /*       std::stringstream s,cGs;
       s << cT;
       cGs <<cG;
       std::string outputPath = 
	 "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+
	 s.str()+
	 "onegaitcycle"+cGs.str()+".txt";
       writeToFile<Eigen::ArrayXXd>(allJointsOneGaitCycle, outputPath);
       if(cG==3){
	 std::string outputPath1 = 
	   "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+
	   s.str()+
	   "temp1.txt";
	 writeToFile<Eigen::ArrayXXd>(temp1, outputPath1);
	 std::string outputPath2 = 
	   "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+
	   s.str()+
	   "temp2.txt";
	 writeToFile<Eigen::ArrayXXd>(temp2, outputPath2);
	 std::string outputPath3 = 
	   "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+
	   s.str()+
	   "temp3.txt";
	 writeToFile<Eigen::ArrayXXd>(temp3, outputPath3);
	 std::string outputPath4 = 
	   "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+
	   s.str()+
	   "temp4.txt";
	 writeToFile<Eigen::ArrayXXd>(temp4, outputPath4);
       }
       */
       
       if(cG==0){
	 //Smoothing First Gait Cycle beginning
	 for (int cJ=0;cJ<nJoints;cJ++){
	   allJointsOneGaitCycle.col(cJ).head(halfTime).reverseInPlace();
	   smoothEnds(allJointsOneGaitCycle.col(cJ).head(halfTime));
	   allJointsOneGaitCycle.col(cJ).head(halfTime).reverseInPlace();
	 }
       }
       else if(cG==nGaitCycles-1){
	 //Smoothing Last Gait Cycle end
	 for (int cJ=0;cJ<nJoints;cJ++){
	   smoothEnds(allJointsOneGaitCycle.col(cJ).tail(halfTime));
	 }
       }

       allJointsAllGaitCycle.middleRows(cG*nTimeSteps,nTimeSteps) = allJointsOneGaitCycle;
       allJointsOneGaitCycle.setZero();
     } // Finish Iterate through gait cycles

       //Interpolate to 320 timesteps (200 state values/second)
       //bSplineInterpolate(allJointsAllGaitCycle, 2); // twice the original time steps
     
     
     //Complete Set of Upperbody joints
     Eigen::ArrayXXd allJointsAllGaitCycleNew(nTimeSteps*nGaitCycles,58);

     //Time Stamps
     allJointsAllGaitCycleNew.col(0) = 
       Eigen::ArrayXd::LinSpaced(nTimeSteps*nGaitCycles, 1, nTimeSteps*nGaitCycles);

     
     //Map calculated traj to joint kinematic chain.
     allJointsAllGaitCycleNew.col(1) = allJointsAllGaitCycle.col(0); //XVel
     allJointsAllGaitCycleNew.col(2) = allJointsAllGaitCycle.col(1); //YVel
     allJointsAllGaitCycleNew.col(3).setZero(); // Yaw vel =0
     allJointsAllGaitCycleNew.col(4) = allJointsAllGaitCycle.col(2); //Chest
     allJointsAllGaitCycleNew.col(5) = allJointsAllGaitCycle.col(3);//Chest
     allJointsAllGaitCycleNew.col(6).setZero(); // Head = 0
     allJointsAllGaitCycleNew.col(7).setZero(); // Head = 0
     allJointsAllGaitCycleNew.col(8) = allJointsAllGaitCycle.col(4);//LARM
     allJointsAllGaitCycleNew.col(9) = allJointsAllGaitCycle.col(5);//LARM
     allJointsAllGaitCycleNew.col(10) = allJointsAllGaitCycle.col(6);//LARM
     allJointsAllGaitCycleNew.col(11) = allJointsAllGaitCycle.col(7);//LARM
     allJointsAllGaitCycleNew.col(12) = allJointsAllGaitCycle.col(8);//LARM
     allJointsAllGaitCycleNew.col(13) = allJointsAllGaitCycle.col(9);//LARM
     allJointsAllGaitCycleNew.col(14).setZero();//LARM = 10deg
     allJointsAllGaitCycleNew.col(15) = allJointsAllGaitCycle.col(10);//RARM
     allJointsAllGaitCycleNew.col(16) = allJointsAllGaitCycle.col(11);//RARM
     allJointsAllGaitCycleNew.col(17) = allJointsAllGaitCycle.col(12);//RARM
     allJointsAllGaitCycleNew.col(18) = allJointsAllGaitCycle.col(13);//RARM
     allJointsAllGaitCycleNew.col(19) = allJointsAllGaitCycle.col(14);//RARM
     allJointsAllGaitCycleNew.col(20) = allJointsAllGaitCycle.col(15);//RARM
     allJointsAllGaitCycleNew.col(21).setZero();//RARM = 10deg
     
     allJointsAllGaitCycleNew.middleCols<18>(4) *= 180/pi; //Convert to Degrees
     allJointsAllGaitCycleNew.col(14).setConstant(10);//LARM Final joint = 10deg
     allJointsAllGaitCycleNew.col(21).setConstant(10);//RARM Final Joint = 10deg


     //Generate q_dot vectors
     for (int i=0;i<allJointsAllGaitCycleNew.rows()-1; i++)
       allJointsAllGaitCycleNew.row(i).segment<18>(22) = 
	 allJointsAllGaitCycleNew.row(i+1).segment<18>(4)-
	 allJointsAllGaitCycleNew.row(i).segment<18>(4);
     
     allJointsAllGaitCycleNew.row(nTimeSteps*nGaitCycles-1).segment<18>(22) = 
       allJointsAllGaitCycleNew.row(nTimeSteps*nGaitCycles-2).segment<18>(22);

     savitzkyGolayFilter(allJointsAllGaitCycleNew.leftCols<40>(), 6, 51);


     //Generate q_dot_dot vectors
     for (int i=0;i<allJointsAllGaitCycleNew.rows()-1; i++)
       allJointsAllGaitCycleNew.row(i).segment<18>(40) = 
	 (allJointsAllGaitCycleNew.row(i+1).segment<18>(22)-
	  allJointsAllGaitCycleNew.row(i).segment<18>(22))*200; //200fps
     
     allJointsAllGaitCycleNew.row(nTimeSteps*nGaitCycles-1).segment<18>(40) = 
       allJointsAllGaitCycleNew.row(nTimeSteps*nGaitCycles-2).segment<18>(40);


     allJointsAllGaitCycleAllTraj.push_back(allJointsAllGaitCycleNew);
     /*
     std::stringstream s;
     s << cT;
     std::string outputPath = 
       "/local/rbudhira/git/proyan/sot-tools/outputCpp/output"+s.str()+".txt";
     writeToFile<Eigen::ArrayXXd>(allJointsAllGaitCycleNew, outputPath);
     */
   }

   void KinematicPlanner::smoothEnds(Eigen::Ref<Eigen::ArrayXd> tr) {
     //smoothends - takes 1d traj tr and enforces 
     //zero velocity and acceleration at the end
     double pi = std::acos(-1);
     int N = (int)tr.size();
     Eigen::ArrayXd shiftedTr(N);
     shiftedTr(0)=0;
     shiftedTr.tail(N-1) = tr.head(N-1);

     //tr=cumsum([tr(1); diff(tr).*(cos([0:N-2]'/(N-1)*pi)+1)/2]);
     tr -= shiftedTr;
     for(int i=1;i<N;i++){
       tr(i) *= (std::cos((i-1)/(N-1)*pi)+1)/2;
       tr(i) += tr(i-1);
     }
     return;
   }

   void KinematicPlanner::bSplineInterpolate(Eigen::ArrayXXd& tr, int factor){
     const int splDeg = 3;
     const int splDim = 1; //Cubic Splines in 1 dimension
     Eigen::MatrixXd trCopy = tr;

     tr.resize(nTimeSteps*nGaitCycles*factor, Eigen::NoChange);

     Eigen::RowVectorXd x_vec(nTimeSteps*nGaitCycles);
     Eigen::RowVectorXd x_vec_new(nTimeSteps*nGaitCycles*factor);
     for(int i=0;i<nTimeSteps*nGaitCycles;i++)
       x_vec(i) = i/(nTimeSteps*nGaitCycles);
     for(int i=0;i<nTimeSteps*nGaitCycles*factor;i++)
       x_vec_new(i) = i/(nTimeSteps*nGaitCycles*factor);

     for(int cJ=0;cJ<nJoints;cJ++) {
       Eigen::Spline<double, splDim> spl = 
	 Eigen::SplineFitting<Eigen::Spline<double, splDim> >::
	 Interpolate(trCopy.col(cJ).transpose(),
		     splDeg,
		     x_vec);
       for(int t=0;t<nTimeSteps*nGaitCycles*factor;t++) tr(t, cJ) = spl(x_vec_new(t))(0);
     }
     return;
   }
      
      /*   void KinematicPlanner::goalAdaption(void){
     Eigen::ArrayXd baseSeries(nTrajectories);
     for(int i=0;i<nTrajectories;i++) baseSeries(i) = i;

     Eigen::ArrayXXd goals(nTrajectories,nGaitCycles);

     goals.col(0)= (baseSeries/(nTrajectories-1))*(0.55-0.5)+0.5;
     goals.col(1)= (baseSeries/(nTrajectories-1))*(0.35-0.1)+0.1;
     goals.col(2)= (baseSeries/(nTrajectories-1))*(0.75-0.64)+0.64;
     goals.col(3)= (baseSeries/(nTrajectories-1))*(0.232-0.062)+0.062;


     ////////////////////////////////////////////////////////////////////
     load for the first 2 steps
     //(periodic weights + non periodic weight + mean)*2 steps
     int K = nJoints*(nSources1+nSources2+2)*2; 
     Eigen::ArrayXXd M(nTrajectories,K);
     for (int cT=0;cT<nTrajectories;cT++) { 
       Eigen::ArrayXd test_point = goals.block<1,2>(cT,0);
       for (int k=0;k<K;k++){
	 Eigen::MatrixXd X(q.rows(),3);
	 X.middleCols<2>(0) = q.matrix()-Eigen::MatrixXd::Ones(q.rows, 1)*test_point;
	 X.cols(2) = Eigen::MatrixXd::Ones(q.rows,1);
	 Eigen::MatrixXd Wkern =
	   (-(((X*X.transpose()).diagonal())/sigma2)).exp().asDiagonal();
	 M(cT,k) = 
	   (((X.transpose()*Wkern*X)*X.transpose()*Wkern).inverse()*
	    beta.at(k).col(q.cols()))(q.cols());
       }
     }

     Eigen::MatrixXd wwn(nTrajectories, K);
     for (int jn =0;jn<K;jn++) wwn.col(jn) = M.col(jn)+mwwn(jn);

     //2 steps taken
     for (int cT=0;cT<nTrajectories;cT++) {
       //first two steps
       for (int j=0;j<2;j++){
	 for (int cs1=0;cs1<nSources1;cs1++) {
	   wPeriodic1.at(cs1).block(cT*nJoints,j,nJoints,1) =
	     wwn.block(cT,j*(nJoints*(nSources1+nSources2+2))+cs1*nJoints,1,nJoints).transpose();
	 }
	 for (int cs2=0;cs2<nSources2;cs2++) {
	   wPeriodic2.at(cs2).block(cT*nJoints,j,nJoints,1) =
	     wwn.block(cT,
		       j*(nJoints*(nSources1+nSources2+2))+nJoints*nSources1+cs2*nJoints,
		       1,
		       nJoints).transpose();
	 }
	 wNonPeriodic.block(cT*nJoints,j,nJoints,1) =
	   wwn.block(cT,
		     j*(nJoints*(nSources1+nSources2+2))+nJoints*(nSources1+nSources2),
		     1,
		     nJoints).transpose();
	 mJointAngle.block(cT*nJoints,j,nJoints,1) = 
	   wwn.block(cT,
		     j*(nJoints*(nSources1+nSources2+2))+nJoints*(nSources1+nSources2+1),
		     1,
		     nJoints).transpose();
       }
     }
     /////////////////////////////////////////////////////////////////////
     load for 3rd step
     ////////////////////////////////////////////////////////////////////
     //(periodic weights + non periodic weight + mean)*1 step = 3rd step
     int K = nJoints*(nSources1+nSources2+2);
     Eigen::ArrayXXd M(nTrajectories,K);
     for (int cT=0;cT<nTrajectories;cT++) { 
       Eigen::ArrayXd test_point = goals.block<1,2>(cT,1);
       for (int k=0;k<K;k++){
	 Eigen::MatrixXd X(q.rows(),3);
	 X.middleCols<2>(0) = q.matrix()-Eigen::MatrixXd::Ones(q.rows, 1)*test_point;
	 X.cols(2) = Eigen::MatrixXd::Ones(q.rows,1);
	 Eigen::MatrixXd Wkern =
	   (-(((X*X.transpose()).diagonal().asDiagonal())/sigma2)).exp();
	 M(cT,k) = 
	   (((X.transpose()*Wkern*X)*X.transpose()*Wkern).inverse()*
	    beta.at(k).col(q.cols()))(q.cols());
       }
     }

     Eigen::MatrixXd wwn(nTrajectories, K);
     for (int jn =0;jn<K;jn++) wwn.col(jn) = M.col(jn)+mwwn(jn);

     //3rd step taken
     for (int cT=0;cT<nTrajectories;cT++) {
       //first two steps
       for (int cs1=0;cs1<nSources1;cs1++) {
	 wPeriodic1.at(cs1).block(cT*nJoints,2,nJoints,1) =
	   wwn.block(cT,cs1*nJoints,1,nJoints).transpose();
       }
       for (int cs2=0;cs2<nSources2;cs2++) {
	 wPeriodic2.at(cs2).block(cT*nJoints,2,nJoints,1) =
	   wwn.block(cT, nJoints*nSources1+cs2*nJoints, 1, nJoints).transpose();
       }
       wNonPeriodic.block(cT*nJoints,2,nJoints,1) =
	 wwn.block(cT,nJoints*(nSources1+nSources2), 1, nJoints).transpose();
       mJointAngle.block(cT*nJoints,j,nJoints,1) = 
	 wwn.block(cT,nJoints*(nSources1+nSources2+1), 1, nJoints).transpose();
     }
     /////////////////////////////////////////////////////////////////////
     
     load for 4th step
		    ////////////////////////////////////////////////////////////////////
		    //(periodic weights + non periodic weight + mean)*1 step = 4th step
    int K = nJoints*(nSources1+nSources2+2);
    Eigen::ArrayXXd M(nTrajectories,K);
    for (int cT=0;cT<nTrajectories;cT++) { 
      Eigen::ArrayXd test_point = goals.block<1,2>(cT,2);
      for (int k=0;k<K;k++){
	Eigen::MatrixXd X(q.rows(),3);
	X.middleCols<2>(0) = q.matrix()-Eigen::MatrixXd::Ones(q.rows, 1)*test_point;
	X.cols(2) = Eigen::MatrixXd::Ones(q.rows,1);
	Eigen::MatrixXd Wkern =
	  (-(((X*X.transpose()).diagonal().asDiagonal())/sigma2)).exp();
	M(cT,k) = 
	  (((X.transpose()*Wkern*X)*X.transpose()*Wkern).inverse()*
	   beta.at(k).col(q.cols()))(q.cols());
      }
    }

    Eigen::MatrixXd wwn(nTrajectories, K);
    for (int jn =0;jn<K;jn++) wwn.col(jn) = M.col(jn)+mwwn(jn);

    //3rd step taken
    for (int cT=0;cT<nTrajectories;cT++) {
      //first two steps
      for (int cs1=0;cs1<nSources1;cs1++) {
	wPeriodic1.at(cs1).block(cT*nJoints,3,nJoints,1) =
	  wwn.block(cT,cs1*nJoints,1,nJoints).transpose();
      }
      for (int cs2=0;cs2<nSources2;cs2++) {
	wPeriodic2.at(cs2).block(cT*nJoints,3,nJoints,1) =
	  wwn.block(cT, nJoints*nSources1+cs2*nJoints, 1, nJoints).transpose();
      }
      wNonPeriodic.block(cT*nJoints,3,nJoints,1) =
	wwn.block(cT,nJoints*(nSources1+nSources2), 1, nJoints).transpose();
      mJointAngle.block(cT*nJoints,3,nJoints,1) = 
	wwn.block(cT,nJoints*(nSources1+nSources2+1), 1, nJoints).transpose();
    }
    /////////////////////////////////////////////////////////////////////
  }

*/

  void KinematicPlanner::savitzkyGolayFilter(Eigen::Ref<Eigen::ArrayXXd> allJointTraj,
					     int polyOrder,
					     int frameSize){
    assert(polyOrder < frameSize);
    assert(frameSize <= allJointTraj.rows()); //Length of the inputs is >= frameSize
    assert(frameSize%2 == 1); //frameSize must be odd

    ///////////////////////////////////////////
    //Calc the projection matrix
    
    //Create the vandermonde matrix
    Eigen::ArrayXXd vanderArr(frameSize, polyOrder+1);
    vanderArr.col(0).setOnes();
    vanderArr.col(1) = Eigen::ArrayXd::LinSpaced(frameSize, -(frameSize-1)/2, (frameSize-1)/2);
    for (int i = 2;i<polyOrder+1; i++) vanderArr.col(i) = vanderArr.col(i-1)*vanderArr.col(1);
    sotDEBUG(15)<<"Printing the VanderArr"<<std::endl;
    sotDEBUG(15)<<vanderArr<<std::endl;

    //Cholesky factor and projection
    Eigen::MatrixXd upperT = (vanderArr.matrix().transpose()*vanderArr.matrix()).llt().matrixU();
    sotDEBUG(15)<<"###########################################"<<std::endl;
    sotDEBUG(15)<<"Printing the Cholesky decomp"<<std::endl;
    sotDEBUG(15)<<upperT<<std::endl;
    Eigen::MatrixXd A = vanderArr.matrix()* upperT.inverse();

    sotDEBUG(15)<<"###########################################"<<std::endl;
    sotDEBUG(15)<<"Printing A"<<std::endl;
    sotDEBUG(15)<<A<<std::endl;
    
    Eigen::MatrixXd proj = A*A.transpose();
    ///////////////////////////////////////////

    for (int cJ=0;cJ<allJointTraj.cols(); cJ++){
      //transient on:
      Eigen::VectorXd yit = (proj.bottomRows((frameSize+1)/2)*
			     allJointTraj.matrix().col(cJ).head(frameSize).reverse()).reverse();
      
      //steady state:
      Eigen::MatrixXd xb(frameSize, allJointTraj.rows()-frameSize+1);
      for(int i=0;i<allJointTraj.rows()-frameSize+1; i++)
	xb.col(i) = allJointTraj.col(cJ).segment(i,frameSize);
      Eigen::VectorXd yss = (proj.row((frameSize-1)/2)*
			     xb.middleCols(1, xb.cols()-2)).transpose();
    
      //transient off:
      Eigen::VectorXd yot = (proj.topRows((frameSize+1)/2)*
			     allJointTraj.matrix().col(cJ).tail(frameSize).reverse()).reverse();

      allJointTraj.col(cJ) << yit, yss, yot;
    }
  }



  void KinematicPlanner::runKinematicPlanner(void){
    if(parametersSet){
      //    goalAdaption();
      for (int cT=0;cT<nTrajectories; cT++) { //nTrajectories = 30
	long int t1 = clock();
	delaySources(cT);
	blending(cT);
	long int t2 = clock();
	std::cerr<<(t2-t1)<<" clicks of CPU proc"
		 <<" for trajectory:"<<cT
		 <<" at clocks per sec:"<<CLOCKS_PER_SEC
		 <<" which is "<<(double)(t2-t1)/CLOCKS_PER_SEC<<"sec"
		 <<std::endl;
      }
    }
    else{
      std::cerr<<"Set Parameters via setParams first"<<std::endl;
    }
    return;
  }



  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (KinematicPlanner, "KinematicPlanner");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph
