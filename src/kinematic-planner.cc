//
// Copyright (C) 2016 LAAS-CNRS
//
// Author: Rohan Budhiraja
//

#include "sot/tools/kinematic-planner.hh"
#include <dynamic-graph/command-bind.h>
#include <fstream>
#include <sstream>
#include <time.h>
namespace dynamicgraph {
namespace sot {
namespace tools {

KinematicPlanner::KinematicPlanner(const std::string& name)
    : Entity(name)
/*
  ,distToDrawerSIN(NULL,"KP("+name+")::input(double)::distanceToDrawer")
  ,objectPositionInDrawerSIN(NULL,"KP("+name+")::input(double)::objectPosinDrawer")
  ,trajectoryReadySINTERN(boost::bind(&KinematicPlanner::runKinematicPlanner,this,_1,_2),
  distToDrawerSIN<<objectPositionInDrawerSIN,
  "KP("+name+")::intern(dummy)::newtoneuler" )
  ,upperBodyJointPositionSOUT(boost::bind(&KinematicPlanner::computeubJointPosition,this,_1,_2),
  trajectoryReadySINTERN,
  "KP("+name+")::output(vector)::upperBodyJointPosition" )
  ,upperBodyJointVelocitySOUT(boost::bind(&KinematicPlanner::computeubJointVelocity,this,_1,_2),
  trajectoryReadySINTERN,
  "KP("+name+")::output(vector)::upperBodyJointVelocity" )
  ,freeFlyerVelocitySOUT(boost::bind(&KinematicPlanner::computeffVelocity,this,_1,_2),
  trajectoryReadySINTERN,
  "KP("+name+")::output(vector)::freeFlyerVelocity" )
*/
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
  addCommand("setParams", dynamicgraph::command::makeCommandVoid3(*this, &KinematicPlanner::setParams, docstring1));

  // runKinematicPlanner
  const std::string docstring2 =
      "\n"
      "    Run the Planner to generate upper joint trajectories \n"
      "\n"
      "      Input:\n"
      "        - none \n"
      "\n";
  addCommand("runKinematicPlanner",
             dynamicgraph::command::makeCommandVoid0(
                 *this, (void (KinematicPlanner::*)(void)) & KinematicPlanner::runKinematicPlanner, docstring2));
  parametersSet = false;

  // std::string dir = "/local/rbudhira/git/proyan/sot-tools/src/params2";
  // setParams(320 , 5 , 4, dir);
  // delaySources();
  // blending();

  sotDEBUGOUT(5);
}

KinematicPlanner::~KinematicPlanner(void) {
  sotDEBUGIN(15);

  for (std::list<SignalBase<int>*>::iterator iter = genericSignalRefs.begin(); iter != genericSignalRefs.end();
       ++iter) {
    SignalBase<int>* sigPtr = *iter;
    delete sigPtr;
  }
  sotDEBUGOUT(15);
}

namespace {
template <typename Derived>
void writeToFile(Eigen::DenseBase<Derived>& inMatrix, std::string& filePath) {
  std::ofstream file(filePath.c_str());
  if (file.is_open()) {
    file << inMatrix;
  }
}
}  // namespace

template <typename Derived>
void KinematicPlanner::read2DArray(std::string& fileName, Eigen::DenseBase<Derived>& outArr) {
  sotDEBUGIN(5);
  std::ifstream fileStream(fileName.c_str());
  std::string curLine;
  double _dbl;
  std::string _dblString;
  int _row = 0;
  while (std::getline(fileStream, curLine)) {  //'\n' delimited rows
    std::istringstream iss(curLine);
    int _col = 0;
    while (std::getline(iss, _dblString, '\t')) {  // tab-delimited columns
      _dbl = atof(_dblString.c_str());
      outArr(_row, _col) = _dbl;
      _col++;
    }
    _row++;
  }
  sotDEBUGOUT(5);
  return;
}

void KinematicPlanner::setParams(const double& _distanceToDrawer, const double& _objectPositionInDrawer,
                                 const std::string& dir) {
  sotDEBUGIN(5);

  // Currently Fixed Parameters
  // TODO: Are these variables?
  //     nTrajectories = 30;
  nJoints = 16;
  nGaitCycles = 4;

  nTimeSteps = 160;  //_nTimeSteps; //160
  nSources1 = 5;     //_nSources1; //5
  nSources2 = 4;     //_nSources2; //4

  npSource.resize(nTimeSteps);
  pSource1.resize(nTimeSteps, nSources1);
  pSource2.resize(nTimeSteps, nSources2);

  pSourceDelayed1.resize(nJoints);
  pSourceDelayed2.resize(nJoints);

  // Delays
  pDelay1.resize(nJoints, nSources1);
  pDelay2.resize(nJoints, nSources2);

  // Non Periodic Weights
  wNonPeriodic.resize(nJoints, nGaitCycles);

  // Periodic Weights
  wPeriodic1.resize(nSources1);
  for (int i = 0; i < nSources1; i++) wPeriodic1.at(i).resize(nJoints, nGaitCycles);

  wPeriodic2.resize(nSources2);
  for (int i = 0; i < nSources2; i++) wPeriodic2.at(i).resize(nJoints, nGaitCycles);

  // Mean joint angles
  mJointAngle.resize(nJoints, nGaitCycles);

  loadSourceDelays(dir);
  dynamicgraph::Vector goalsNew = createSubGoals(_distanceToDrawer, _objectPositionInDrawer);
  goalAdaption(goalsNew, dir);
  delaySources();

  parametersSet = true;
  sotDEBUGOUT(5);
  return;
}

void KinematicPlanner::loadTrainingParams(const std::string& dir, dynamicgraph::Matrix& q, dynamicgraph::Matrix& beta3,
                                          Eigen::ArrayXd& mwwn, double& sigma2, int& N, int& K) {
  Eigen::Vector3d sigmaSqNK;
  std::string fileName = dir + "/" + "sigmaSqNK";
  read2DArray<Eigen::Vector3d>(fileName, sigmaSqNK);
  assert(sigmaSqNK.cols() == 1);
  assert(sigmaSqNK.rows() == 3);

  sigma2 = sigmaSqNK(0);
  N = (int)sigmaSqNK(1);
  K = (int)sigmaSqNK(2);

  fileName = dir + "/" + "q";
  read2DArray<dynamicgraph::Matrix>(fileName, q);
  assert(q.cols() == 2);
  assert(q.rows() == N);

  fileName = dir + "/" + "mwwn";
  read2DArray<Eigen::ArrayXd>(fileName, mwwn);
  assert(mwwn.cols() == 1);
  assert(mwwn.rows() == K);

  fileName = dir + "/" + "beta3";
  read2DArray<dynamicgraph::Matrix>(fileName, beta3);
  assert(beta3.cols() == K);
  assert(beta3.rows() == N);

  return;
}

void KinematicPlanner::loadSourceDelays(const std::string& dir) {
  sotDEBUGIN(5);

  // assert(boost::filesystem::is_directory(dir));
  // Load Source1
  std::string fileName = dir + "/" + "pSource1";
  read2DArray<Eigen::ArrayXXd>(fileName, pSource1);
  assert(pSource1.cols() == nSources1);
  assert(pSource1.rows() == nTimeSteps);

  // Load Source2
  fileName = dir + "/" + "pSource2";
  read2DArray<Eigen::ArrayXXd>(fileName, pSource2);
  assert(pSource2.cols() == nSources2);
  assert(pSource2.rows() == nTimeSteps);

  // Load Non-Periodic Source
  fileName = dir + "/" + "npSource";
  read2DArray<Eigen::ArrayXd>(fileName, npSource);
  assert(npSource.cols() == 1);
  assert(npSource.rows() == nTimeSteps);

  // Load Delays: Source1
  fileName = dir + "/" + "pDelay1";
  read2DArray<Eigen::ArrayXXd>(fileName, pDelay1);
  assert(pDelay1.cols() == nSources1);
  assert(pDelay1.rows() == nJoints);

  // Load Delays: Source2
  fileName = dir + "/" + "pDelay2";
  read2DArray<Eigen::ArrayXXd>(fileName, pDelay2);
  assert(pDelay2.cols() == nSources2);
  assert(pDelay2.rows() == nJoints);

  sotDEBUGOUT(5);
  return;
}

/*

void KinematicPlanner::loadTrainingWeights(const std::string& dir) {
sotDEBUGIN(5);
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

}
*/

void KinematicPlanner::delaySources() {
  // TODO: Confirm if this is HalfSpectrum or not
  sotDEBUGIN(5);
  pSourceDelayed1.clear();
  pSourceDelayed2.clear();

  Eigen::FFT<double> fft;
  double pi = std::acos(-1);
  // Prepare for the Delay Exponential
  Eigen::ArrayXcd V(nTimeSteps);
  for (int cur_time = 0; cur_time < nTimeSteps; cur_time++) {
    std::complex<double> pi_freq;
    pi_freq.real(0);
    pi_freq.imag(cur_time);
    V(cur_time) = pi_freq;
  }
  V *= 2 * pi / nTimeSteps;

  // Do the FFT Source1
  Eigen::ArrayXXcd pSourceInFreq(nTimeSteps, nSources1);
  for (int cur_src = 0; cur_src < nSources1; cur_src++) {
    pSourceInFreq.matrix().col(cur_src) = fft.fwd(pSource1.matrix().col(cur_src));
  }

  // Do the Delay
  for (int cJ = 0; cJ < nJoints; cJ++) {
    Eigen::ArrayXXd pSourceDelayedJoint(nTimeSteps, nSources1);
    for (int cur_src = 0; cur_src < nSources1; cur_src++) {
      // DelayedFt = Source(w) * exp^(V*pDelay1(joint, source))
      const Eigen::ArrayXcd delayedFT = pSourceInFreq.col(cur_src) * (V * pDelay1(cJ, cur_src)).exp();
      //(V*pDelay1(cT*nJoints+cJ, cur_src)).unaryExpr<std::complex(*)(std::complex)>(&std::exp);
      // FFT Inv
      pSourceDelayedJoint.matrix().col(cur_src) = fft.inv(delayedFT.matrix());
    }
    pSourceDelayed1.push_back(pSourceDelayedJoint);
  }

  // Do the FFT Source 2
  pSourceInFreq.resize(Eigen::NoChange, nSources2);
  for (int cur_src = 0; cur_src < nSources2; cur_src++) {
    pSourceInFreq.matrix().col(cur_src) = fft.fwd(pSource2.matrix().col(cur_src));
  }

  // Do the Delay
  for (int cJ = 0; cJ < nJoints; cJ++) {
    Eigen::ArrayXXd pSourceDelayedJoint(nTimeSteps, nSources2);
    for (int cur_src = 0; cur_src < nSources2; cur_src++) {
      // DelayedFt = Source(w) * exp^(V*pDelay1(joint, source))
      const Eigen::ArrayXcd delayedFT = pSourceInFreq.col(cur_src) * (V * pDelay2(cJ, cur_src)).exp();
      //(V*pDelay2(cT*nJoints+cJ, cur_src)).unaryExpr<std::complex(*)(std::complex)>(&std::exp);
      // FFT Inv
      pSourceDelayedJoint.matrix().col(cur_src) = fft.inv(delayedFT.matrix());
    }
    pSourceDelayed2.push_back(pSourceDelayedJoint);
  }

  sotDEBUGOUT(5);
  return;
}

void KinematicPlanner::blending() {
  sotDEBUGIN(5);
  Eigen::ArrayXd wCurrent;
  Eigen::ArrayXd mJointAngleCurrent;

  double pi = std::acos(-1);
  int halfTime = (int)nTimeSteps / 2;
  bool specialInterpolatedWeight = false;

  Eigen::ArrayXXd allJointsOneGaitCycle(nTimeSteps, nJoints);
  Eigen::ArrayXXd allJointsAllGaitCycle(nTimeSteps * nGaitCycles, nJoints);
  allJointsOneGaitCycle.setZero();
  allJointsAllGaitCycle.setZero();

  for (int cG = 0; cG < nGaitCycles; cG++) {  // nGaitCycles = 4

    for (int t = 0; t < nTimeSteps; t++) {  // nTimeSteps = 160
      stdVectorofArrayXd currentWeight1;
      stdVectorofArrayXd currentWeight2;
      if (t < nTimeSteps / 10) {
        // Matlab starts from 1. TODO:initial blending interpolation check
        // Beginning of the step
        if (cG != 0) {
          specialInterpolatedWeight = true;
          // Not for the first step
          // non periodic source intermediate value
          Eigen::ArrayXd wnpbeg =
              (npSource(nTimeSteps - 1) * wNonPeriodic.col(cG - 1) + npSource(0) * wNonPeriodic.col(cG)) / 2;
          // interpolation weight
          double ww = (std::sin(t / (nTimeSteps / 10) * pi / 2) + 1) / 2;
          wCurrent = (2 * ww - 1) * wNonPeriodic.col(cG) + (1 - (2 * ww - 1)) * wnpbeg;
          mJointAngleCurrent = ww * mJointAngle.col(cG) + (1 - ww) * mJointAngle.col(cG - 1);

          for (int cSrc1 = 0; cSrc1 < nSources1; cSrc1++) {
            Eigen::ArrayXd intWeightCSrc =
                ww * wPeriodic1.at(cSrc1).col(cG) + (1 - ww) * wPeriodic1.at(cSrc1).col(cG - 1);
            currentWeight1.push_back(intWeightCSrc);
          }

          for (int cSrc2 = 0; cSrc2 < nSources2; cSrc2++) {
            Eigen::ArrayXd intWeightCSrc =
                ww * wPeriodic2.at(cSrc2).col(cG) + (1 - ww) * wPeriodic2.at(cSrc2).col(cG - 1);
            currentWeight2.push_back(intWeightCSrc);
          }
        }
      }

      else if (t > nTimeSteps * 9 / 10) {
        // End of Step
        if (cG != nGaitCycles - 1) {
          // Not for the last gait cycle
          specialInterpolatedWeight = true;
          Eigen::ArrayXd wnpEnd =
              -(npSource(nTimeSteps - 1) * wNonPeriodic.col(cG) + npSource(0) * wNonPeriodic.col(cG + 1)) / 2;
          double ww = (std::sin((t - nTimeSteps) / (nTimeSteps / 10) * pi / 2) + 1) / 2;
          wCurrent = (1 - 2 * ww) * wNonPeriodic.col(cG) + 2 * ww * wnpEnd;
          mJointAngleCurrent = (1 - ww) * mJointAngle.col(cG) + ww * mJointAngle.col(cG + 1);

          for (int cSrc1 = 0; cSrc1 < nSources1; cSrc1++) {
            Eigen::ArrayXd intWeightCSrc =
                (1 - ww) * wPeriodic1.at(cSrc1).col(cG) + ww * wPeriodic1.at(cSrc1).col(cG + 1);

            currentWeight1.push_back(intWeightCSrc);
          }

          for (int cSrc2 = 0; cSrc2 < nSources2; cSrc2++) {
            Eigen::ArrayXd intWeightCSrc =
                (1 - ww) * wPeriodic2.at(cSrc2).col(cG) + ww * wPeriodic2.at(cSrc2).col(cG + 1);
            currentWeight2.push_back(intWeightCSrc);
          }
        }  //(cG!=nGaitCycles-1)
      }    //(t>nTimeSteps*9/10)

      // Start Mixing all sources
      for (int cJ = 0; cJ < nJoints; cJ++) {
        if (specialInterpolatedWeight) {
          for (int cSrc1 = 0; cSrc1 < nSources1; cSrc1++)
            allJointsOneGaitCycle(t, cJ) += pSourceDelayed1.at(cJ)(t, cSrc1) * currentWeight1.at(cSrc1)(cJ);
          for (int cSrc2 = 0; cSrc2 < nSources2; cSrc2++)
            allJointsOneGaitCycle(t, cJ) += pSourceDelayed2.at(cJ)(t, cSrc2) * currentWeight2.at(cSrc2)(cJ);

          allJointsOneGaitCycle(t, cJ) += npSource(t) * wCurrent(cJ);
          allJointsOneGaitCycle(t, cJ) += mJointAngleCurrent(cJ);

        } else {
          for (int cSrc1 = 0; cSrc1 < nSources1; cSrc1++)
            allJointsOneGaitCycle(t, cJ) += pSourceDelayed1.at(cJ)(t, cSrc1) * wPeriodic1.at(cSrc1)(cJ, cG);
          for (int cSrc2 = 0; cSrc2 < nSources2; cSrc2++)
            allJointsOneGaitCycle(t, cJ) += pSourceDelayed2.at(cJ)(t, cSrc2) * wPeriodic2.at(cSrc2)(cJ, cG);

          allJointsOneGaitCycle(t, cJ) += npSource(t) * wNonPeriodic(cJ, cG);
          allJointsOneGaitCycle(t, cJ) += mJointAngle(cJ, cG);
        }
      }

      specialInterpolatedWeight = false;
    }  // Finish iterate through nTimeSteps

    if (cG == 0) {
      // Smoothing First Gait Cycle beginning
      for (int cJ = 0; cJ < nJoints; cJ++) {
        allJointsOneGaitCycle.col(cJ).head(halfTime).reverseInPlace();
        smoothEnds(allJointsOneGaitCycle.col(cJ).head(halfTime));
        allJointsOneGaitCycle.col(cJ).head(halfTime).reverseInPlace();
      }
    } else if (cG == nGaitCycles - 1) {
      // Smoothing Last Gait Cycle end
      for (int cJ = 0; cJ < nJoints; cJ++) {
        smoothEnds(allJointsOneGaitCycle.col(cJ).tail(halfTime));
      }
    }

    allJointsAllGaitCycle.middleRows(cG * nTimeSteps, nTimeSteps) = allJointsOneGaitCycle;
    allJointsOneGaitCycle.setZero();
  }  // Finish Iterate through gait cycles

  // Complete Set of Upperbody joints
  Eigen::ArrayXXd allJointsAllGaitCycleNew(nTimeSteps * nGaitCycles, 58);

  // Time Stamps
  allJointsAllGaitCycleNew.col(0) = Eigen::ArrayXd::LinSpaced(nTimeSteps * nGaitCycles, 1, nTimeSteps * nGaitCycles);

  // Map calculated traj to joint kinematic chain.
  allJointsAllGaitCycleNew.col(1) = allJointsAllGaitCycle.col(0);    // XVel
  allJointsAllGaitCycleNew.col(2) = allJointsAllGaitCycle.col(1);    // YVel
  allJointsAllGaitCycleNew.col(3).setZero();                         // Yaw vel =0
  allJointsAllGaitCycleNew.col(4) = allJointsAllGaitCycle.col(2);    // Chest
  allJointsAllGaitCycleNew.col(5) = allJointsAllGaitCycle.col(3);    // Chest
  allJointsAllGaitCycleNew.col(6).setZero();                         // Head = 0
  allJointsAllGaitCycleNew.col(7).setZero();                         // Head = 0
  allJointsAllGaitCycleNew.col(8) = allJointsAllGaitCycle.col(4);    // LARM
  allJointsAllGaitCycleNew.col(9) = allJointsAllGaitCycle.col(5);    // LARM
  allJointsAllGaitCycleNew.col(10) = allJointsAllGaitCycle.col(6);   // LARM
  allJointsAllGaitCycleNew.col(11) = allJointsAllGaitCycle.col(7);   // LARM
  allJointsAllGaitCycleNew.col(12) = allJointsAllGaitCycle.col(8);   // LARM
  allJointsAllGaitCycleNew.col(13) = allJointsAllGaitCycle.col(9);   // LARM
  allJointsAllGaitCycleNew.col(14).setZero();                        // LARM = 10deg
  allJointsAllGaitCycleNew.col(15) = allJointsAllGaitCycle.col(10);  // RARM
  allJointsAllGaitCycleNew.col(16) = allJointsAllGaitCycle.col(11);  // RARM
  allJointsAllGaitCycleNew.col(17) = allJointsAllGaitCycle.col(12);  // RARM
  allJointsAllGaitCycleNew.col(18) = allJointsAllGaitCycle.col(13);  // RARM
  allJointsAllGaitCycleNew.col(19) = allJointsAllGaitCycle.col(14);  // RARM
  allJointsAllGaitCycleNew.col(20) = allJointsAllGaitCycle.col(15);  // RARM
  allJointsAllGaitCycleNew.col(21).setZero();                        // RARM = 10deg

  allJointsAllGaitCycleNew.middleCols<18>(4) *= 180 / pi;  // Convert to Degrees
  allJointsAllGaitCycleNew.col(14).setConstant(10);        // LARM Final joint = 10deg
  allJointsAllGaitCycleNew.col(21).setConstant(10);        // RARM Final Joint = 10deg

  // Generate q_dot vectors
  for (int i = 0; i < allJointsAllGaitCycleNew.rows() - 1; i++)
    allJointsAllGaitCycleNew.row(i).segment<18>(22) =
        allJointsAllGaitCycleNew.row(i + 1).segment<18>(4) - allJointsAllGaitCycleNew.row(i).segment<18>(4);

  allJointsAllGaitCycleNew.row(nTimeSteps * nGaitCycles - 1).segment<18>(22) =
      allJointsAllGaitCycleNew.row(nTimeSteps * nGaitCycles - 2).segment<18>(22);

  savitzkyGolayFilter(allJointsAllGaitCycleNew.leftCols<40>(), 6, 51);

  // Generate q_dot_dot vectors
  for (int i = 0; i < allJointsAllGaitCycleNew.rows() - 1; i++)
    allJointsAllGaitCycleNew.row(i).segment<18>(40) =
        (allJointsAllGaitCycleNew.row(i + 1).segment<18>(22) - allJointsAllGaitCycleNew.row(i).segment<18>(22)) *
        200;  // 200fps

  allJointsAllGaitCycleNew.row(nTimeSteps * nGaitCycles - 1).segment<18>(40) =
      allJointsAllGaitCycleNew.row(nTimeSteps * nGaitCycles - 2).segment<18>(40);
}

void KinematicPlanner::smoothEnds(Eigen::Ref<Eigen::ArrayXd> tr) {
  // smoothends - takes 1d traj tr and enforces
  // zero velocity and acceleration at the end
  double pi = std::acos(-1);
  int N = (int)tr.size();
  Eigen::ArrayXd shiftedTr(N);
  shiftedTr(0) = 0;
  shiftedTr.tail(N - 1) = tr.head(N - 1);

  // tr=cumsum([tr(1); diff(tr).*(cos([0:N-2]'/(N-1)*pi)+1)/2]);
  tr -= shiftedTr;
  for (int i = 1; i < N; i++) {
    tr(i) *= (std::cos((i - 1) / (N - 1) * pi) + 1) / 2;
    tr(i) += tr(i - 1);
  }
  return;
}

void KinematicPlanner::bSplineInterpolate(Eigen::ArrayXXd& tr, int factor) {
  const int splDeg = 3;
  const int splDim = 1;  // Cubic Splines in 1 dimension
  Eigen::MatrixXd trCopy = tr;

  tr.resize(nTimeSteps * nGaitCycles * factor, Eigen::NoChange);

  Eigen::RowVectorXd x_vec(nTimeSteps * nGaitCycles);
  Eigen::RowVectorXd x_vec_new(nTimeSteps * nGaitCycles * factor);
  for (int i = 0; i < nTimeSteps * nGaitCycles; i++) x_vec(i) = i / (nTimeSteps * nGaitCycles);
  for (int i = 0; i < nTimeSteps * nGaitCycles * factor; i++) x_vec_new(i) = i / (nTimeSteps * nGaitCycles * factor);

  for (int cJ = 0; cJ < nJoints; cJ++) {
    Eigen::Spline<double, splDim> spl =
        Eigen::SplineFitting<Eigen::Spline<double, splDim> >::Interpolate(trCopy.col(cJ).transpose(), splDeg, x_vec);
    for (int t = 0; t < nTimeSteps * nGaitCycles * factor; t++) tr(t, cJ) = spl(x_vec_new(t))(0);
  }
  return;
}

void KinematicPlanner::goalAdaption(dynamicgraph::Vector& goals, const std::string& dir) {
  ////////////////////////////////////////////////////////////////////
  //     load for the first 2 steps

  //(periodic weights + non periodic weight + mean)*2 steps
  //     int K = nJoints*(nSources1+nSources2+2)*2;
  {  // local scope01

    const std::string dir01 = dir + "/adaption/01";
    dynamicgraph::Matrix q(30, 2);
    dynamicgraph::Matrix beta3(30, 288);
    Eigen::ArrayXd mwwn(288);
    double sigma2;
    int N;
    int K;

    loadTrainingParams(dir01, q, beta3, mwwn, sigma2, N, K);
    Eigen::ArrayXd M(K);
    Eigen::ArrayXd test_point(2);
    test_point << goals(0), goals(1);
    for (int k = 0; k < K; k++) {
      Eigen::MatrixXd X(q.rows(), 3);
      X.col(0) = q.col(0) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(0);
      X.col(1) = q.col(1) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(1);
      X.col(2) = Eigen::MatrixXd::Ones(q.rows(), 1);
      Eigen::VectorXd _expdiag = (-(((X * X.transpose()).diagonal().matrix()) / sigma2)).exp();
      Eigen::MatrixXd Wkern = _expdiag.asDiagonal();
      M(k) = (((X.transpose() * Wkern * X) * X.transpose() * Wkern).inverse() * beta3.col(k))(q.cols());
    }

    Eigen::ArrayXd wwn = M + mwwn;

    // 2 steps taken
    // first two steps
    for (int j = 0; j < 2; j++) {
      for (int cs1 = 0; cs1 < nSources1; cs1++) {
        wPeriodic1.at(cs1).col(j) = wwn.segment(j * (nJoints * (nSources1 + nSources2 + 2)) + cs1 * nJoints, nJoints);
      }
      for (int cs2 = 0; cs2 < nSources2; cs2++) {
        wPeriodic2.at(cs2).col(j) =
            wwn.segment(j * (nJoints * (nSources1 + nSources2 + 2)) + nJoints * nSources1 + cs2 * nJoints, nJoints);
      }
      wNonPeriodic.col(j) =
          wwn.segment(j * (nJoints * (nSources1 + nSources2 + 2)) + nJoints * (nSources1 + nSources2), nJoints);

      mJointAngle.col(j) =
          wwn.segment(j * (nJoints * (nSources1 + nSources2 + 2)) + nJoints * (nSources1 + nSources2 + 1), nJoints);
    }
  }  // local scope01

  {  // local scope3
    //     load for 3rd step
    const std::string dir3 = dir + "/adaption/2";
    dynamicgraph::Matrix q(30, 2);
    dynamicgraph::Matrix beta3(30, 144);
    Eigen::ArrayXd mwwn(144);
    double sigma2;
    int N;
    int K;

    loadTrainingParams(dir3, q, beta3, mwwn, sigma2, N, K);

    //(periodic weights + non periodic weight + mean)*1 step = 3rd step
    //     int K = nJoints*(nSources1+nSources2+2);
    Eigen::ArrayXd M(K);
    Eigen::ArrayXd test_point(2);
    test_point << goals(1), goals(2);
    for (int k = 0; k < K; k++) {
      Eigen::MatrixXd X(q.rows(), 3);
      X.col(0) = q.col(0) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(0);
      X.col(1) = q.col(1) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(1);
      X.col(2) = Eigen::MatrixXd::Ones(q.rows(), 1);
      Eigen::VectorXd _expdiag = (-(((X * X.transpose()).diagonal().matrix()) / sigma2)).exp();
      Eigen::MatrixXd Wkern = _expdiag.asDiagonal();
      M(k) = (((X.transpose() * Wkern * X) * X.transpose() * Wkern).inverse() * beta3.col(k))(q.cols());
    }

    Eigen::ArrayXd wwn = M + mwwn;

    // 3rd step taken
    // first two steps
    for (int cs1 = 0; cs1 < nSources1; cs1++) {
      wPeriodic1.at(cs1).col(2) = wwn.segment(cs1 * nJoints, nJoints);
    }
    for (int cs2 = 0; cs2 < nSources2; cs2++) {
      wPeriodic2.at(cs2).col(2) = wwn.segment(nJoints * nSources1 + cs2 * nJoints, nJoints);
    }
    wNonPeriodic.col(2) = wwn.segment(nJoints * (nSources1 + nSources2), nJoints);
    mJointAngle.col(2) = wwn.segment(nJoints * (nSources1 + nSources2 + 1), nJoints);

  }  // local scope3

  {  // local scope4

    // load for 4th step
    //(periodic weights + non periodic weight + mean)*1 step = 4th step
    const std::string dir4 = dir + "/adaption/3";
    dynamicgraph::Matrix q(30, 2);
    dynamicgraph::Matrix beta3(30, 144);
    Eigen::ArrayXd mwwn(144);
    double sigma2;
    int N;
    int K;

    loadTrainingParams(dir4, q, beta3, mwwn, sigma2, N, K);
    //    int K = nJoints*(nSources1+nSources2+2);
    Eigen::ArrayXd M(K);
    Eigen::ArrayXd test_point(2);
    test_point << goals(2), goals(3);

    for (int k = 0; k < K; k++) {
      Eigen::MatrixXd X(q.rows(), 3);
      X.col(0) = q.col(0) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(0);
      X.col(1) = q.col(1) - Eigen::MatrixXd::Ones(q.rows(), 1) * test_point(1);
      X.col(2) = Eigen::MatrixXd::Ones(q.rows(), 1);
      Eigen::VectorXd _expdiag = (-(((X * X.transpose()).diagonal().matrix()) / sigma2)).exp();
      Eigen::MatrixXd Wkern = _expdiag.asDiagonal();

      M(k) = (((X.transpose() * Wkern * X) * X.transpose() * Wkern).inverse() * beta3.col(k))(q.cols());
    }

    Eigen::ArrayXd wwn = M + mwwn;

    // 3rd step taken
    for (int cs1 = 0; cs1 < nSources1; cs1++) {
      wPeriodic1.at(cs1).col(3) = wwn.segment(cs1 * nJoints, nJoints);
    }
    for (int cs2 = 0; cs2 < nSources2; cs2++) {
      wPeriodic2.at(cs2).col(3) = wwn.segment(nJoints * nSources1 + cs2 * nJoints, nJoints);
    }
    wNonPeriodic.col(3) = wwn.segment(nJoints * (nSources1 + nSources2), nJoints);
    mJointAngle.col(3) = wwn.segment(nJoints * (nSources1 + nSources2 + 1), nJoints);
  }  // local scope4
}

void KinematicPlanner::savitzkyGolayFilter(Eigen::Ref<Eigen::ArrayXXd> allJointTraj, int polyOrder, int frameSize) {
  assert(polyOrder < frameSize);
  assert(frameSize <= allJointTraj.rows());  // Length of the inputs is >= frameSize
  assert(frameSize % 2 == 1);                // frameSize must be odd

  // Calc the projection matrix

  // Create the vandermonde matrix
  Eigen::ArrayXXd vanderArr(frameSize, polyOrder + 1);
  vanderArr.col(0).setOnes();
  vanderArr.col(1) = Eigen::ArrayXd::LinSpaced(frameSize, -(frameSize - 1) / 2, (frameSize - 1) / 2);
  for (int i = 2; i < polyOrder + 1; i++) vanderArr.col(i) = vanderArr.col(i - 1) * vanderArr.col(1);
  sotDEBUG(15) << "Printing the VanderArr" << std::endl;
  sotDEBUG(15) << vanderArr << std::endl;

  // Cholesky factor and projection
  Eigen::MatrixXd upperT = (vanderArr.matrix().transpose() * vanderArr.matrix()).llt().matrixU();
  sotDEBUG(15) << "Printing the Cholesky decomp" << std::endl;
  sotDEBUG(15) << upperT << std::endl;
  Eigen::MatrixXd A = vanderArr.matrix() * upperT.inverse();

  sotDEBUG(15) << "Printing A" << std::endl;
  sotDEBUG(15) << A << std::endl;

  Eigen::MatrixXd proj = A * A.transpose();

  for (int cJ = 0; cJ < allJointTraj.cols(); cJ++) {
    // transient on:
    Eigen::VectorXd yit =
        (proj.bottomRows((frameSize + 1) / 2) * allJointTraj.matrix().col(cJ).head(frameSize).reverse()).reverse();

    // steady state:
    Eigen::MatrixXd xb(frameSize, allJointTraj.rows() - frameSize + 1);
    for (int i = 0; i < allJointTraj.rows() - frameSize + 1; i++)
      xb.col(i) = allJointTraj.col(cJ).segment(i, frameSize);
    Eigen::VectorXd yss = (proj.row((frameSize - 1) / 2) * xb.middleCols(1, xb.cols() - 2)).transpose();

    // transient off:
    Eigen::VectorXd yot =
        (proj.topRows((frameSize + 1) / 2) * allJointTraj.matrix().col(cJ).tail(frameSize).reverse()).reverse();

    allJointTraj.col(cJ) << yit, yss, yot;
  }
}

dynamicgraph::Vector KinematicPlanner::createSubGoals(double D, double P) {
  dynamicgraph::Vector goalsNew(4);
  goalsNew(3) = P;
  goalsNew(1) = (D - 1.75) * 0.532 + 0.1;
  goalsNew(2) = (D - goalsNew(1) - 1.64) / 2 + 0.636;
  goalsNew(0) = (D - goalsNew(1) - 1.64) / 4 + 0.5;
  return goalsNew;
}

int& KinematicPlanner::runKinematicPlanner(int& dummy, int time) {
  sotDEBUGIN(15);
  assert(parametersSet && "Set Parameters via setParams first");
  //    double distanceToDrawer = distToDrawerSIN.access(time);
  //    double objectPositionInDrawer = objectPositionInDrawerSIN.access(time);
  //    dynamicgraph::Vector goalsNew = createSubGoals(distanceToDrawer,objectPositionInDrawer);
  //    delaySources();
  blending();
  return dummy;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(KinematicPlanner, "KinematicPlanner");
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph
