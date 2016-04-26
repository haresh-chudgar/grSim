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

Robot::Robot(Communicator* communicator, TrajectoryPlanner* planner, bool team, int id)
:_communicator(communicator), _planner(planner), isYellowTeam(team), playerID(id)
{
  _addr = "127.0.0.1";
  _port = (unsigned short)(20011);
}

Robot::~Robot(){}

void Robot::setCurrentState(Eigen::Vector3d currentState) {
  _currentState = currentState;
}

Eigen::Vector3d Robot::CurrentState() {
  return _currentState;
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

void Robot::execute() {
  currentTime += (1/60);
  
  Eigen::Vector3d desiredState;
  desiredState[0] = desiredLocation[0] + coeffecients[0][0] / currentTime;
  desiredState[1] = desiredLocation[1] + coeffecients[0][1] / currentTime;
  desiredState[2] = desiredLocation[2] + coeffecients[0][2] / currentTime;
  fprintf(stderr, "coeffecients %f %f %f\n", coeffecients[0][0], coeffecients[0][1], coeffecients[0][2]);
  //fprintf(stderr, "Robot::goToLocation currentState: %f,%f,%f\n", CurrentState()[0], CurrentState()[1], CurrentState()[2]);
  //fprintf(stderr, "Robot::goToLocation desiredState: %f,%f,%f\n", desiredState[0], desiredState[1], desiredState[2]);
  _currentVelocity = controller.ComputeCommandVelo(CurrentState(), desiredState, _currentVelocity, Eigen::Vector3d(0,0,0));
  sendVelocityCommands(_currentVelocity[2], _currentVelocity[0], _currentVelocity[1], 0, 0, 0);
}

bool Robot::goToLocation(int currentFrame, Eigen::Vector3d location) {
  
  currentTime = 0.2;
  desiredLocation = location;
  initialLocation = CurrentState();
  
  coeffecients = _planner->GenerateTrajectory(CurrentState(), 
					      desiredLocation, 
					      Eigen::Vector3d(), Eigen::Vector3d(), 
					      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >());
  
  controller = PIDController();

  Eigen::Vector3d desiredState;
  desiredState[0] = desiredLocation[0] + coeffecients[0][0] / currentTime;
  desiredState[1] = desiredLocation[1] + coeffecients[0][1] / currentTime;
  desiredState[2] = desiredLocation[2] + coeffecients[0][2] / currentTime;
  
  //fprintf(stderr, "Robot::goToLocation currentState: %f,%f,%f\n", CurrentState()[0], CurrentState()[1], CurrentState()[2]);
  //fprintf(stderr, "Robot::goToLocation desiredState: %f,%f,%f\n", desiredState[0], desiredState[1], desiredState[2]);
  _currentVelocity = controller.ComputeCommandVelo(CurrentState(), desiredState, _currentVelocity, Eigen::Vector3d(0,0,0));
  sendVelocityCommands(_currentVelocity[2], _currentVelocity[0], _currentVelocity[1], 0, 0, 0);
  return true;
}

// Sends the robot's velocity commands to the simulator
// All velocities are relative to the robot
bool Robot::sendVelocityCommands(double vAngular, double vTangent, double vNormal, double kickSpeedX, double kickSpeedZ, bool spinnerOn) {
  
  vTangent /= 10000.;
  vNormal /= 10000.;
  
  //fprintf(stderr, "original command: %f, %f, %f %f\n", vTangent, vNormal, vAngular, _currentState[2]);
  
  double vT = vTangent, vN = vNormal;
  vTangent = vT * cos(_currentState[2]) - vN * sin(_currentState[2]);
  vNormal = vT * sin(_currentState[2]) + vN * cos(_currentState[2]);
  
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
  return true;
}
