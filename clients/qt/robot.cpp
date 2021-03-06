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
#include "soccerfieldinfo.h"

Robot::Robot( bool isYellowTeam, int id)
: isYellowTeam(isYellowTeam), playerID(id), currentFrame(0)
{
  
//   _addr = "127.0.0.1";
//   _port = (unsigned short)(20011);
  vTangent = 0;
  vNormal = 0;
  vX = 0;
  vY = 0;
  vAngular = 0;
  kickSpeedX = 0;
  kickSpeedZ = 0;
  spinnerOn = false;
}

Robot::~Robot(){}

void Robot::setCurrentState(Eigen::Vector3d currentState, Eigen::Vector3d currentVelocity) {
  _currentPosition = currentState;
  _currentVelocity = currentVelocity;
}

Eigen::Vector3d Robot::CurrentState() {
  return _currentPosition;
}

Eigen::Vector3d Robot::CurrentVelocity() {
  return _currentVelocity;
}

int Robot::dribbleToLocation(Eigen::Vector3d location) {
  
  _maxVelocity = PIDController::MAX_TRANS_VEL/10;
  _maxAngularVelocity = PIDController::MAX_ROT_VEL/ 10;
  
  currentTime = 0.2;
  desiredLocation = location;
  initialLocation = CurrentState();
  controller = PIDController();

  return execute();
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
  vTangent = 0;
  vNormal = 0;
  sendVelocityCommands();
}

// TODO this shouldn't be used for checks, should be called from one loop for all bots
// Executes the robot's movement towards the desiredLocation
// Returns
//    1   - success
//    0   - working
//    -1  - failure
int Robot::execute() {
  
  currentTime += (1.0/60.0);
  //  fprintf(stderr, "currentTime %f\n", currentTime);

  //fprintf(stderr, "CurrentState: %f, %f, %f\ndesiredLocation: %f, %f, %f\n", CurrentState()[0], CurrentState()[1], CurrentState()[2], desiredLocation[0], desiredLocation[1], desiredLocation[2]);
  
  //if(currentFrame == 0) 
  {
    currentWaypointIndex = 0;
    //Pass parameters in millimeters
    PathPlanner planner = PathPlanner(playerID, isYellowTeam);
    coeffecients = planner.FindPath(CurrentState(), desiredLocation);
    //fprintf(stderr, "Size of coeffecients: %d\n", coeffecients.size());
  }
  currentFrame += 1;
  //currentFrame = (currentFrame + 1) % 10;

  Eigen::Vector3d desiredState;
  desiredState[0] = coeffecients[currentWaypointIndex][0];
  desiredState[1] = coeffecients[currentWaypointIndex][1];
  desiredState[2] = coeffecients[currentWaypointIndex][2];
  
  fprintf(stderr, "desiredState: %f, %f, %f\n", desiredState[0], desiredState[1], desiredState[2]);
  
  // Computing velocities
  _currentVelocity = controller.ComputeCommandVelo(CurrentState(), desiredState, _currentVelocity, Eigen::Vector3d(0,0,0));
  vAngular = _currentVelocity[2];
  vX = _currentVelocity[0];
  vY = _currentVelocity[1];
  
  vTangent =  vX * cos(_currentPosition[2]) + vY * sin(_currentPosition[2]);
  vTangent = (abs(vTangent)/vTangent) * min(abs(vTangent), _maxVelocity);
  vTangent /= 1000.;
  vNormal = min(- vX * sin(_currentPosition[2]) + vY * cos(_currentPosition[2]), _maxVelocity);
  vNormal = (abs(vNormal)/vNormal) * min(abs(vNormal), _maxVelocity);
  vNormal /= 1000.;
  //fprintf(stderr, "transformed command: %f, %f, %f\n", vTangent, vNormal, vAngular);
  //vAngular = min(abs(vAngular), _maxVelocity);
  
  sendVelocityCommands();
  
  double dist = Eigen::Vector2d((CurrentState() - desiredState)[0], (CurrentState() - desiredState)[1]).norm();
  fprintf(stderr, "Distance to desired location: %f \n", dist);
  //TODO units?
  if(dist < 3) {
    ++currentWaypointIndex;
    if(currentWaypointIndex > coeffecients.size() - 1) {
      fprintf(stderr, "Executed move to location!\n");
      return 1;
    }
  }
  
  return 0;
}

int Robot::goToLocation(int currentFrame, Eigen::Vector3d location) {
  
  currentTime = 0.2;
  desiredLocation = location;
  initialLocation = CurrentState();
  controller = PIDController();

  _maxVelocity = PIDController::MAX_TRANS_VEL;
  _maxAngularVelocity = PIDController::MAX_ROT_VEL;
  
  return execute();
}

// Sends the robot's velocity commands to the simulator
// All velocities are relative to the global coordinate frame
bool Robot::sendVelocityCommands() {
  fprintf(stderr, "sendValocity: %f, %f, %f\n", vTangent, vNormal, vAngular);
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
  
  SoccerFieldInfo::Instance()->communicator->sendPacket(dgram);
  
  //udpsocket.writeDatagram(dgram, _addr, _port);

  //_communicator->sendPacket(dgram);

  vTangent = 0;
  vNormal = 0;
  vX = 0;
  vY = 0;
  vAngular = 0;
  kickSpeedX = 0;
  kickSpeedZ = 0;

  return true;
}
