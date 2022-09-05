//
// Copyright (C) 2012, 2013 LAAS-CNRS
//
// Author: Florent Lamiraux, Mehdi Benallegue
//

#include "sot/tools/seqplay.hh"

#include <dynamic-graph/command-bind.h>

#include <boost/tokenizer.hpp>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace dynamicgraph {
namespace sot {
namespace tools {
using dynamicgraph::Entity;
using dynamicgraph::command::docCommandVoid0;
using dynamicgraph::command::docCommandVoid1;
using dynamicgraph::command::makeCommandVoid0;
using dynamicgraph::command::makeCommandVoid1;

Seqplay::Seqplay(const std::string& name)
    : Entity(name),
      postureSOUT_("Seqplay(" + name + ")::output(vector)::posture"),
      leftAnkleSOUT_("Seqplay(" + name + ")::output(MatrixHomo)::leftAnkle"),
      rightAnkleSOUT_("Seqplay(" + name + ")::output(MatrixHomo)::rightAnkle"),
      leftAnkleVelSOUT_("Seqplay(" + name + ")::output(Vector)::leftAnkleVel"),
      rightAnkleVelSOUT_("Seqplay(" + name +
                         ")::output(Vector)::rightAnkleVel"),
      comSOUT_("Seqplay(" + name + ")::output(vector)::com"),
      comdotSOUT_("Seqplay(" + name + ")::output(vector)::comdot"),
      comddotSOUT_("Seqplay(" + name + ")::output(vector)::comddot"),
      forceLeftFootSOUT_("Seqplay(" + name +
                         ")::output(vector)::forceLeftFoot"),
      forceRightFootSOUT_("Seqplay(" + name +
                          ")::output(vector)::forceRightFoot"),
      zmpSOUT_("Seqplay(" + name + ")::output(vector)::zmp"),
      state_(0),
      startTime_(0),
      posture_(),
      leftAnkle_(),
      rightAnkle_(),
      com_(),
      time_(),
      R0_(),
      R0t_(),
      R1_(),
      R1R0t_() {
  signalRegistration(postureSOUT_ << leftAnkleSOUT_ << rightAnkleSOUT_
                                  << leftAnkleVelSOUT_ << rightAnkleVelSOUT_
                                  << comSOUT_ << comdotSOUT_ << comddotSOUT_
                                  << forceLeftFootSOUT_ << forceRightFootSOUT_
                                  << zmpSOUT_);
  postureSOUT_.setFunction(boost::bind(&Seqplay::computePosture, this, _1, _2));
  comSOUT_.setFunction(boost::bind(&Seqplay::computeCom, this, _1, _2));
  leftAnkleSOUT_.setFunction(
      boost::bind(&Seqplay::computeLeftAnkle, this, _1, _2));
  rightAnkleSOUT_.setFunction(
      boost::bind(&Seqplay::computeRightAnkle, this, _1, _2));
  leftAnkleVelSOUT_.setFunction(
      boost::bind(&Seqplay::computeLeftAnkleVel, this, _1, _2));
  rightAnkleVelSOUT_.setFunction(
      boost::bind(&Seqplay::computeRightAnkleVel, this, _1, _2));
  comdotSOUT_.setFunction(boost::bind(&Seqplay::computeComdot, this, _1, _2));
  comddotSOUT_.setFunction(boost::bind(&Seqplay::computeComddot, this, _1, _2));

  zmpSOUT_.setFunction(boost::bind(&Seqplay::computeZMP, this, _1, _2));

  forceLeftFootSOUT_.setFunction(
      boost::bind(&Seqplay::computeForceLeftFoot, this, _1, _2));
  forceRightFootSOUT_.setFunction(
      boost::bind(&Seqplay::computeForceRightFoot, this, _1, _2));

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
  addCommand("load", makeCommandVoid1(*this, &Seqplay::load, docstring));

  addCommand("start", makeCommandVoid0(*this, &Seqplay::start,
                                       docCommandVoid0("Start motion")));
  for (size_t i = 0; i < 7; ++i) {
    facultativeFound_[i] = false;
  }
}

void Seqplay::load(const std::string& filename) {
  using boost::escaped_list_separator;
  typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
  std::string line;
  std::string fn[4];
  std::string facultativefn[7];

  std::ifstream file[4];
  std::ifstream facultativeFile[7];
  unsigned int lineNumber = 0;
  int postureSize = -2;

  fn[0] = filename + ".posture";
  fn[1] = filename + ".la";
  fn[2] = filename + ".ra";
  fn[3] = filename + ".com";

  // Open files
  for (std::size_t i = 0; i < 4; i++) {
    file[i].open(fn[i].c_str());
    if (!file[i].is_open()) {
      throw std::runtime_error(std::string("Failed to open file ") + fn[i]);
    }
  }

  facultativefn[0] = filename + ".fl";
  facultativefn[1] = filename + ".fr";
  facultativefn[2] = filename + ".comdot";
  facultativefn[3] = filename + ".comddot";
  facultativefn[4] = filename + ".zmp";
  facultativefn[5] = filename + ".ladot";
  facultativefn[6] = filename + ".radot";

  // Open facultative files
  for (std::size_t i = 0; i < 7; i++) {
    facultativeFile[i].open(facultativefn[i].c_str());
    if (facultativeFile[i].is_open()) {
      facultativeFound_[i] = true;
    } else {
      facultativeFound_[i] = false;
    }
  }

  // both feet forces must be defined together
  if (facultativeFound_[0] != facultativeFound_[1]) {
    throw std::runtime_error(
        std::string("File ") +
        (facultativeFound_[0] ? facultativefn[1] : facultativefn[0]) +
        " failed to open");
  }

  // both ankle velocity must be defined together
  if (facultativeFound_[5] != facultativeFound_[6]) {
    throw std::runtime_error(
        std::string("File ") +
        (facultativeFound_[5] ? facultativefn[6] : facultativefn[5]) +
        " failed to open");
  }

  posture_.clear();
  leftAnkle_.clear();
  rightAnkle_.clear();
  com_.clear();
  comdot_.clear();
  comddot_.clear();
  zmp_.clear();
  leftAnkleDot_.clear();
  rightAnkleDot_.clear();

  forceLeftFoot_.clear();
  forceRightFoot_.clear();

  // Read posture
  while (file[0].good()) {
    std::getline(file[0], line);
    ++lineNumber;
    tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
    std::vector<double> components;
    for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
      components.push_back(atof(it->c_str()));
    }
    if (components.size() == 0) {
      break;
    }
    if (postureSize == -2) {
      postureSize = static_cast<int>(components.size() - 1);
    } else {
      if (postureSize != static_cast<int>(components.size()) - 1) {
        std::ostringstream oss;
        oss << fn[0] << ", line " << lineNumber << ": config of size "
            << components.size() - 1 << ". Expecting " << postureSize << ".";
        throw std::runtime_error(oss.str());
      }
    }
    Vector config(static_cast<unsigned>(components.size() - 1));
    for (unsigned i = 1; i < components.size(); ++i) {
      config(i - 1) = components[i];
    }
    posture_.push_back(config);
  }
  file[0].close();

  readAnkleFile(file[1], leftAnkle_, fn[1]);
  readAnkleFile(file[2], rightAnkle_, fn[2]);

  // Read com
  lineNumber = 0;
  while (file[3].good()) {
    std::getline(file[3], line);
    ++lineNumber;
    tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
    std::vector<double> components;
    for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
      components.push_back(atof(it->c_str()));
    }
    if (components.size() == 0) break;
    Vector com(3);
    if (components.size() != 4) {
      std::ostringstream oss;
      oss << fn[3] << ", line " << lineNumber << ": expecting 4 numbers.";
      throw std::runtime_error(oss.str());
    }
    time_.push_back(components[0]);
    for (unsigned i = 1; i < 4; ++i) {
      com(i - 1) = components[i];
    }
    com_.push_back(com);
  }
  file[3].close();

  // Read forces
  if (facultativeFound_[0]) {
    readForceFile(facultativeFile[0], forceLeftFoot_, facultativefn[4]);
    readForceFile(facultativeFile[1], forceRightFoot_, facultativefn[5]);
  }

  // Read com velocity
  if (facultativeFound_[2]) {
    lineNumber = 0;
    while (facultativeFile[2].good()) {
      std::getline(facultativeFile[2], line);
      ++lineNumber;
      tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
      std::vector<double> components;
      for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
        components.push_back(atof(it->c_str()));
      }
      if (components.size() == 0) break;
      Vector comdot(3);
      if (components.size() != 4) {
        std::ostringstream oss;
        oss << facultativefn[2] << ", line " << lineNumber
            << ": expecting 4 numbers.";
        throw std::runtime_error(oss.str());
      }

      for (unsigned i = 1; i < 4; ++i) {
        comdot(i - 1) = components[i];
      }
      comdot_.push_back(comdot);
    }
    facultativeFile[2].close();
  }

  // Read com acceleration
  if (facultativeFound_[3]) {
    lineNumber = 0;
    while (facultativeFile[3].good()) {
      std::getline(facultativeFile[3], line);
      ++lineNumber;
      tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
      std::vector<double> components;
      for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
        components.push_back(atof(it->c_str()));
      }
      if (components.size() == 0) break;
      Vector comddot(3);
      if (components.size() != 4) {
        std::ostringstream oss;
        oss << facultativefn[3] << ", line " << lineNumber
            << ": expecting 4 numbers.";
        throw std::runtime_error(oss.str());
      }

      for (unsigned i = 1; i < 4; ++i) {
        comddot(i - 1) = components[i];
      }
      comddot_.push_back(comddot);
    }
    facultativeFile[3].close();
  }

  // left ankle velocity
  if (facultativeFound_[5]) {
    lineNumber = 0;
    while (facultativeFile[5].good()) {
      std::getline(facultativeFile[5], line);
      ++lineNumber;
      tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
      std::vector<double> components;
      for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
        components.push_back(atof(it->c_str()));
      }
      if (components.size() == 0) break;
      Vector ankledot(7);
      if (components.size() != 7) {
        std::ostringstream oss;
        oss << facultativefn[5] << ", line " << lineNumber
            << ": expecting 7 numbers.";
        throw std::runtime_error(oss.str());
      }

      for (unsigned i = 1; i < 7; ++i) {
        ankledot(i - 1) = components[i];
      }
      leftAnkleDot_.push_back(ankledot);
    }
    facultativeFile[5].close();
  }

  // right ankle velocity
  if (facultativeFound_[6]) {
    lineNumber = 0;
    while (facultativeFile[6].good()) {
      std::getline(facultativeFile[6], line);
      ++lineNumber;
      tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
      std::vector<double> components;
      for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
        components.push_back(atof(it->c_str()));
      }
      if (components.size() == 0) break;
      Vector ankledot(7);
      if (components.size() != 7) {
        std::ostringstream oss;
        oss << facultativefn[6] << ", line " << lineNumber
            << ": expecting 7 numbers.";
        throw std::runtime_error(oss.str());
      }

      for (unsigned i = 1; i < 7; ++i) {
        ankledot(i - 1) = components[i];
      }
      rightAnkleDot_.push_back(ankledot);
    }
    facultativeFile[6].close();
  }

  // Read zmp
  if (facultativeFound_[4]) {
    lineNumber = 0;
    while (facultativeFile[4].good()) {
      std::getline(facultativeFile[4], line);
      ++lineNumber;
      tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
      std::vector<double> components;
      for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
        components.push_back(atof(it->c_str()));
      }
      if (components.size() == 0) break;
      Vector zmp(3);
      if (components.size() != 4) {
        std::ostringstream oss;
        oss << facultativefn[3] << ", line " << lineNumber
            << ": expecting 4 numbers.";
        throw std::runtime_error(oss.str());
      }

      for (unsigned i = 1; i < 4; ++i) {
        zmp(i - 1) = components[i];
      }
      zmp_.push_back(zmp);
    }
    facultativeFile[4].close();
  }

  // Check that size of files is the same
  if (posture_.size() != leftAnkle_.size() ||
      posture_.size() != rightAnkle_.size() || posture_.size() != com_.size() ||
      (facultativeFound_[0] && posture_.size() != forceLeftFoot_.size()) ||
      (facultativeFound_[1] && posture_.size() != forceRightFoot_.size()) ||
      (facultativeFound_[2] && posture_.size() != comdot_.size()) ||
      (facultativeFound_[3] && posture_.size() != comddot_.size()) ||
      (facultativeFound_[4] && posture_.size() != zmp_.size()) ||
      (facultativeFound_[5] && posture_.size() != leftAnkleDot_.size()) ||
      (facultativeFound_[6] && posture_.size() != rightAnkleDot_.size())) {
    std::ostringstream oss;
    oss << "Seqplay: Files should have the same number of lines. Read"
        << std::endl;
    oss << "  " << posture_.size() << " lines from " << filename << ".posture,"
        << std::endl;
    oss << "  " << leftAnkle_.size() << " lines from " << filename << ".la,"
        << std::endl;
    oss << "  " << rightAnkle_.size() << " lines from " << filename << ".ra,"
        << std::endl;
    oss << "  " << com_.size() << " lines from " << filename << ".com,"
        << std::endl;
    if (facultativeFound_[0]) {
      oss << "  " << forceLeftFoot_.size() << " lines from " << filename
          << ".fl" << std::endl;
      oss << "  " << forceRightFoot_.size() << " lines from " << filename
          << ".fr" << std::endl;
    }

    if (facultativeFound_[2]) {
      oss << "  " << comdot_.size() << " lines from " << filename << ".comdot"
          << std::endl;
    }

    if (facultativeFound_[3]) {
      oss << "  " << comddot_.size() << " lines from " << filename << ".comddot"
          << std::endl;
    }

    if (facultativeFound_[4]) {
      oss << "  " << zmp_.size() << " lines from " << filename << ".zmp"
          << std::endl;
    }

    if (facultativeFound_[5]) {
      oss << "  " << leftAnkleDot_.size() << " lines from " << filename
          << ".ladot" << std::endl;
      oss << "  " << leftAnkleDot_.size() << " lines from " << filename
          << ".radot" << std::endl;
    }

    throw std::runtime_error(oss.str());
  }
  state_ = 0;
}

void Seqplay::start() {
  if (state_ == 0) {
    state_ = 1;
    startTime_ =
        std::max(std::max(comSOUT_.getTime(), comdotSOUT_.getTime()),
                 std::max(leftAnkleSOUT_.getTime(), rightAnkleSOUT_.getTime()));
    startTime_ = std::max(startTime_, postureSOUT_.getTime());
  }
}

Vector& Seqplay::computePosture(Vector& pos, const int& t) {
  if (posture_.size() == 0) {
    throw std::runtime_error(
        "Seqplay posture: Signals not initialized. read files first.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == posture_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  pos = posture_[configId];
  return pos;
}

MatrixHomogeneous& Seqplay::computeLeftAnkle(MatrixHomogeneous& la,
                                             const int& t) {
  if (leftAnkle_.size() == 0) {
    throw std::runtime_error(
        "Seqplay leftAnkle: Signals not initialized. read files first.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == posture_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  la = leftAnkle_[configId];
  return la;
}

MatrixHomogeneous& Seqplay::computeRightAnkle(MatrixHomogeneous& ra,
                                              const int& t) {
  if (rightAnkle_.size() == 0) {
    throw std::runtime_error(
        "Seqplay rightAnkle: Signals not initialized. read files first.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == posture_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  ra = rightAnkle_[configId];
  return ra;
}

Vector& Seqplay::computeAnkleVelocity(
    Vector& velocity, const std::vector<MatrixHomogeneous>& ankleVector,
    const int& t) {
  velocity.resize(6);
  velocity.setZero();
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == com_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }

  if (configId > 0 && configId < com_.size() - 1) {
    const MatrixHomogeneous& M1 = ankleVector[configId + 1];
    const MatrixHomogeneous& M0 = ankleVector[configId];
    double dt = 1 / (time_[configId + 1] - time_[configId]);
    for (unsigned i = 0; i < 3; ++i) {
      velocity(i) = (M1(i, 3) - M0(i, 3)) * dt;
    }
    R1_ = M1.linear();
    R0_ = M0.linear();
    R0t_ = R0_.transpose();
    R1R0t_ = R1_ * R0t_;
    velocity(3) = (R1R0t_(2, 1)) * dt;
    velocity(4) = (R1R0t_(0, 2)) * dt;
    velocity(5) = (R1R0t_(1, 0)) * dt;
  }

  return velocity;
}

Vector& Seqplay::computeLeftAnkleVel(Vector& velocity, const int& t) {
  // if there is no file, the velocity is computed using finite differences
  if (!facultativeFound_[5]) {
    return computeAnkleVelocity(velocity, leftAnkle_, t);
  } else {
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == posture_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    velocity = leftAnkleDot_[configId];
    return velocity;
  }
}

Vector& Seqplay::computeRightAnkleVel(Vector& velocity, const int& t) {
  if (!facultativeFound_[6]) {
    return computeAnkleVelocity(velocity, rightAnkle_, t);
  } else {
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == posture_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    velocity = rightAnkleDot_[configId];
    return velocity;
  }
}

Vector& Seqplay::computeCom(Vector& com, const int& t) {
  if (com_.size() == 0) {
    throw std::runtime_error(
        "Seqplay com: Signals not initialized. read files first.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == com_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  com = com_[configId];
  return com;
}

Vector& Seqplay::computeZMP(Vector& zmp, const int& t) {
  if (zmp_.size() == 0) {
    throw std::runtime_error("Seqplay zmp: Signals not initialized.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == com_.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  zmp = zmp_[configId];
  return zmp;
}

Vector& Seqplay::computeComdot(Vector& comdot, const int& t) {
  if (facultativeFound_[2]) {
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == posture_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    comdot = comdot_[configId];
    return comdot;
  } else {
    // finite differences

    if (com_.size() == 0) {
      throw std::runtime_error(
          "Seqplay comdot: Signals not initialized. read files first.");
    }
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == com_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    comdot.resize(com_[0].size());
    comdot.setZero();
    if (configId > 0 && configId < com_.size() - 1) {
      const Vector& q_1 = com_[configId + 1];
      const Vector& q_0 = com_[configId];
      double dt = 1 / (time_[configId + 1] - time_[configId]);
      comdot = (q_1 - q_0) * dt;
    }
    return comdot;
  }
}

Vector& Seqplay::computeComddot(Vector& comddot, const int& t) {
  if (facultativeFound_[3]) {
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == posture_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    comddot = comddot_[configId];
    return comddot;
  } else {
    // finite differences
    if (com_.size() == 0) {
      throw std::runtime_error(
          "Seqplay comddot: Signals not initialized. read files first.");
    }
    std::size_t configId;
    if (state_ == 0) {
      configId = 0;
    } else if (state_ == 1) {
      configId = t - startTime_;
      if (configId == com_.size() - 1) {
        state_ = 2;
      }
    } else {
      configId = posture_.size() - 1;
    }
    comddot.resize(com_[0].size());
    comddot.setZero();
    if (configId > 0 && configId < com_.size() - 2) {
      Vector qdot_1;
      Vector qdot_0;

      double dt_0 = 1 / (time_[configId + 1] - time_[configId]);

      if (facultativeFound_[2]) {
        qdot_1 = comdot_[configId + 1];
        qdot_0 = comdot_[configId];
      } else {
        const Vector& q_2 = com_[configId + 2];
        const Vector& q_1 = com_[configId + 1];
        const Vector& q_0 = com_[configId];
        double dt_1 = 1 / (time_[configId + 2] - time_[configId + 1]);

        qdot_1 = (q_2 - q_1) * dt_1;
        qdot_0 = (q_1 - q_0) * dt_0;
      }

      for (int i = 0; i < comddot.size(); ++i) {
        comddot(i) = (qdot_1(i) - qdot_0(i)) * dt_0;
      }
    }
    return comddot;
  }
}

Vector& Seqplay::computeForceFoot(Vector& force,
                                  const std::vector<Vector>& forceVector,
                                  const int& t) {
  if (forceVector.size() == 0) {
    throw std::runtime_error(
        "Seqplay foot force: Force signals not initialized.");
  }
  std::size_t configId;
  if (state_ == 0) {
    configId = 0;
  } else if (state_ == 1) {
    configId = t - startTime_;
    if (configId == forceVector.size() - 1) {
      state_ = 2;
    }
  } else {
    configId = posture_.size() - 1;
  }
  force = forceVector[configId];
  return force;
}

Vector& Seqplay::computeForceLeftFoot(Vector& force, const int& t) {
  return computeForceFoot(force, forceLeftFoot_, t);
}

Vector& Seqplay::computeForceRightFoot(Vector& force, const int& t) {
  return computeForceFoot(force, forceRightFoot_, t);
}

std::string Seqplay::getDocString() const {
  return "Provide task references for a whole-body motion\n"
         "\n"
         "  The reference trajectories of the following features is loaded "
         "from files\n"
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

void Seqplay::readAnkleFile(std::ifstream& file,
                            std::vector<MatrixHomogeneous>& data,
                            const std::string& filename) {
  using boost::escaped_list_separator;
  typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
  unsigned int lineNumber = 0;
  std::string line;

  while (file.good()) {
    std::getline(file, line);
    ++lineNumber;
    tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
    std::vector<double> components;
    for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
      components.push_back(atof(it->c_str()));
    }
    if (components.size() == 0) break;
    if (components.size() != 17) {
      std::ostringstream oss;
      oss << filename << ", line " << lineNumber
          << ": expecting 17 numbers, got " << components.size() << ".";
      throw std::runtime_error(oss.str());
    }
    MatrixHomogeneous la;
    std::size_t i = 1;
    for (unsigned row = 0; row < 4; ++row) {
      for (unsigned col = 0; col < 4; ++col) {
        la(row, col) = components[i];
        ++i;
      }
    }
    data.push_back(la);
  }
  file.close();
}

void Seqplay::readForceFile(std::ifstream& file, std::vector<Vector>& data,
                            const std::string& filename) {
  using boost::escaped_list_separator;
  typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
  unsigned int lineNumber = 0;
  std::string line;

  while (file.good()) {
    std::getline(file, line);
    ++lineNumber;
    tokenizer_t tok(line, escaped_list_separator<char>('\\', '\t', '\"'));
    std::vector<double> components;
    for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); ++it) {
      components.push_back(atof(it->c_str()));
    }
    if (components.size() == 0) break;
    if (components.size() != 7) {
      std::ostringstream oss;
      oss << filename << ", line " << lineNumber
          << ": expecting 7 numbers, got " << components.size() << ".";
      throw std::runtime_error(oss.str());
    }
    Vector force(6);
    for (unsigned i = 1; i < 7; ++i) {
      force(i - 1) = components[i];
    }
    data.push_back(force);
  }
  file.close();
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Seqplay, "Seqplay");
}  // namespace tools
}  // namespace sot
}  // namespace dynamicgraph
