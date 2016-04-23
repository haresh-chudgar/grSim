/*
 * Copyright 2016 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "communicator.h"
#include "trajectoryplanner.h"
#include "gvector.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"

#include <eigen3/Eigen/Core>

class Robot
{
 public:
  Robot(Communicator* communicator, TrajectoryPlanner* planner, const bool team, const int id);

  ~Robot();
    
  int dribbleToLocation(GVector::vector3d<double> location);
  int flatKickBallToLocation(GVector::vector2d<double> location, double speed);
  int lobKickBallToLocation(GVector::vector2d<double> location, double height);
  int goToLocation(GVector::vector3d<double> location);

  //these setters should probably be removed or made private once testing is done
  void setAngularVelocity(double vAngular);
  void setTangentVelocity(double vTangent);
  void setNormalVelocity(double vNormal);
  void setKickSpeed(double speedX, double speedZ);
  void setSpinner(double on);

  void setCurrentState(Eigen::Vector3d currentState);
  Eigen::Vector3d getCurrentState();
  
  bool sendVelocityCommands();
 private:
  QUdpSocket udpsocket;
  QHostAddress _addr;
  quint16 _port;
  Communicator* _communicator;
  TrajectoryPlanner* _planner;
  
  Eigen::Vector3d _currentState;
  bool hasBall;
  bool kicking;
  
  const bool team;
  int playerID;
  double velTangent, velNormal, velAngular;
  double kickSpeedX, kickSpeedZ;
  bool spinnerOn;

  double maxDistanceErrorSquared;
  double maxOrientationError;

  double kGravity;

  int executeKickBallToLocation(GVector::vector2d<double> location, double speed, double height);
};

#endif // ROBOT_H
