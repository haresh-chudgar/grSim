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

#include "robot.h"
#include "pathplanner.h"

Robot::Robot(Communicator* communicator, PathPlanner* planner, bool team, int id)
:_communicator(communicator), isYellowTeam(team), playerID(id), _planner(planner)
{
  
  _addr = "127.0.0.1";
  _port = (unsigned short)(20011);
}

Robot::~Robot(){}

void Robot::setCurrentState(Eigen::Vector3d currentState, Eigen::Vector3d currentVelocity) {
  _currentState = currentState;
  _currentVelocity = currentVelocity;
}

Eigen::Vector3d Robot::CurrentState() {
  return _currentState;
}

Eigen::Vector3d Robot::CurrentVelocity() {
  return _currentVelocity;
}

int Robot::dribbleToLocation(Eigen::Vector3d location) {
  return 0;
}

int Robot::flatKickBallToLocation(Eigen::Vector2d location, double speed) {
  return 0;
}


int Robot::lobKickBallToLocation(Eigen::Vector2d location, double height) {
  return 0;
}

int Robot::lobKickBallToLocation(Eigen::Vector2d location, double height, double distOfMaxHeight) {
  return 0;
}


int Robot::executeKickBallToLocation(Eigen::Vector2d location, double speed, double height, double distOfMaxHeight) {
  return 0;
}


void Robot::setSpinner(bool on) {
  spinnerOn = on;
}

void Robot::setKickSpeed(double vX, double vZ) {
  kickSpeedX = vX;
  kickSpeedZ = vZ;
  fprintf(stderr, "setKickSpeed %f %f", kickSpeedX, kickSpeedZ);
}

void Robot::stopMoving() {
  vAngular = 0;
  vX = 0;
  vY = 0;
}

// Executes the robot's movement towards the desiredLocation
// Returns
//    1   - success
//    0   - working
//    -1  - failure
int Robot::execute() {
  Eigen::Vector3d dist = (CurrentState()-desiredLocation);

  // handles wraparound
  while (dist[2] > M_PI) {
    dist[2] -= 2*M_PI;
  }

  while (dist[2] < -M_PI) {
    dist[2] += 2*M_PI;
  }
  

  //fprintf(stderr, "error comps %f %f %f\n", dist[0], dist[1], dist[2]);
  //fprintf(stderr, "CurrentError %f\n", dist.norm());
  if (dist.norm() < 0.1) {
    return 1;
  }
  currentTime += (1.0/60.0);
//  fprintf(stderr, "currentTime %f\n", currentTime);

  coeffecients = _planner->FindPath(CurrentState()/1000, desiredLocation/1000);
  // TODO(KARL) check if planner can generate trajectory, return -1 if not

  Eigen::Vector3d desiredState;
  desiredState[0] = desiredLocation[0] + coeffecients[0][0] / currentTime;
  desiredState[1] = desiredLocation[1] + coeffecients[0][1] / currentTime;
  desiredState[2] = desiredLocation[2] + coeffecients[0][2] / currentTime;
//  fprintf(stderr, "coeffecients %f %f %f\n", coeffecients[0][0], coeffecients[0][1], coeffecients[0][2]);
//  fprintf(stderr, "coeffecients/time %f %f %f\n", coeffecients[0][0] / currentTime, coeffecients[0][1] / currentTime, coeffecients[0][2] / currentTime);

//  fprintf(stderr, "Robot::goToLocation currentState: %f,%f,%f\n", CurrentState()[0], CurrentState()[1], CurrentState()[2]);
//  fprintf(stderr, "Robot::goToLocation desiredLocation: %f,%f,%f\n", desiredLocation[0], desiredLocation[1], desiredLocation[2]);
//  fprintf(stderr, "Robot::goToLocation desiredState: %f,%f,%f\n", desiredState[0], desiredState[1], desiredState[2]);
  _currentVelocity = controller.ComputeCommandVelo(CurrentState(), desiredState, _currentVelocity, Eigen::Vector3d(0,0,0));
  vAngular = _currentVelocity[2];
  vX = _currentVelocity[0];
  vY = _currentVelocity[1];
  sendVelocityCommands();
  return 0;
}

int Robot::goToLocation(int currentFrame, Eigen::Vector3d location) {
  
  currentTime = 0.2;
  desiredLocation = location;
  initialLocation = CurrentState();
  controller = PIDController();

  return execute();
}

// Sends the robot's velocity commands to the simulator
// All velocities are relative to the global coordinate frame
bool Robot::sendVelocityCommands() {//double vAngular, double vX, double vY, double kickSpeedX, double kickSpeedZ, bool spinnerOn) {
  //vX /= 10000.;
  //vY /= 10000.;
  

  //fprintf(stderr, "original command: %f, %f, %f   %f, %f  %f\n", vX, vY, vAngular, _currentState[2], cos(_currentState[2]), sin(_currentState[2]));

  // vTangent is in the direction the robot faces
  double vTangent =  vX * cos(_currentState[2]) + vY * sin(_currentState[2]);
  vTangent /= 10000.;
  double vNormal = - vX * sin(_currentState[2]) + vY * cos(_currentState[2]);
  vNormal /= 10000.;
  //fprintf(stderr, "transformed command: %f, %f, %f %f\n", vTangent, vNormal, vAngular, _currentState[2]);
  
  grSim_Packet packet;
  
  packet.mutable_commands()->set_isteamyellow(isYellowTeam);
  packet.mutable_commands()->set_timestamp(0.0);
  grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
  command->set_id(playerID);

  command->set_wheelsspeed(false);

  //either individual wheel velocities need to be set or tangent/normal/angular
  
  command->set_wheel1(0);
  command->set_wheel2(0);
  command->set_wheel3(0);
  command->set_wheel4(0);

  // Velocity commands are in the robot's reference frame
  command->set_veltangent(vTangent);
  command->set_velnormal(vNormal);
  command->set_velangular(vAngular);
  
  command->set_kickspeedx(kickSpeedX);
  command->set_kickspeedz(kickSpeedZ);

  command->set_spinner(spinnerOn);

  QByteArray dgram;
  dgram.resize(packet.ByteSize());
  packet.SerializeToArray(dgram.data(), dgram.size());
  _communicator->sendPacket(dgram);
  //udpsocket.writeDatagram(dgram, _addr, _port);

  //_communicator->sendPacket(dgram);

  vX = 0;
  vX = 0;
  vAngular = 0;
  kickSpeedX = 0;
  kickSpeedZ = 0;

  return true;
}
