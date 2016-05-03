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
#include "pathplanner.h"
#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "pid_controller.h"

#include <eigen3/Eigen/Core>

class Robot
{
 public:
  Robot(Communicator* communicator, PathPlanner* planner, const bool team, const int id);

  ~Robot();
    
  int goToLocation(int currentFrame, Eigen::Vector3d location);
  int dribbleToLocation(Eigen::Vector3d location);
  int flatKickBallToLocation(Eigen::Vector2d location, double speed);
  int lobKickBallToLocation(Eigen::Vector2d location, double height);
  int lobKickBallToLocation(Eigen::Vector2d location, double height, double distOfMaxHeight);
  
  int execute();
  
  //these setters should probably be removed or made private once testing is done
  void setAngularVelocity(double vAngular);
  void setXVelocity(double velX);
  void setYVelocity(double velY);
  void setKickSpeed(double speedX, double speedZ);
  void setSpinner(bool on);
  void stopMoving();
  
  void setCurrentState(Eigen::Vector3d currentState, Eigen::Vector3d currentVelocity);
  Eigen::Vector3d CurrentState();
  Eigen::Vector3d CurrentVelocity();
  
  bool sendVelocityCommands();
  
  const bool isYellowTeam;
  int playerID;
  
 private:
  QHostAddress _addr;
  quint16 _port;
  Communicator* _communicator;
  PathPlanner* _planner;
  PIDController controller;
  
  int currentWaypointIndex;
  double currentTime;
  int currentFrame;
  Eigen::Vector3d _currentState;
  Eigen::Vector3d _currentVelocity;
  
  Eigen::Vector3d initialLocation;
  Eigen::Vector3d desiredLocation;
  std::vector<Eigen::Vector3d> coeffecients;
  bool hasBall;
  bool kicking;
  
  double vX, vY, vAngular;
  double vTangent, vNormal;
  double kickSpeedX, kickSpeedZ;
  bool spinnerOn;

  double maxDistanceErrorSquared;
  double maxOrientationError;

  double kGravity;
  
  double _maxVelocity;
  double _maxAngularVelocity;
  
  int executeKickBallToLocation(Eigen::Vector2d location, double speed, double height, double distOfMaxHeight);
};

#endif // ROBOT_H
