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
:_communicator(communicator), _planner(planner), team(team), playerID(id),
    udpsocket(0)
{
  _addr = "127.0.0.1";
  _port = (unsigned short)(20011);
  velTangent = 0;
  velNormal = 0;
  velAngular = 0;
  kickSpeedX = 0;
  kickSpeedZ = 0;
  spinnerOn = false;
}

Robot::~Robot(){}

void Robot::setCurrentState(Eigen::Vector3d currentState) {
  _currentState = currentState;
}

// TODO(Whoever is responsible for getting the state) get if robot has ball
Eigen::Vector3d Robot::getCurrentState() {
  return _currentState;
}


// sets the velocity commands for the robot to dribble the ball to a location
// Precondition: The robot must have the ball
// Postcondition: The robot has the ball and is within the acceptable 
//    error margin of the given pos and orientation
// args   : location - the {x, y, theta} the robot should arrive at 
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::dribbleToLocation(GVector::vector3d<double> location) {
  if (!hasBall) {
    return -1;
  }
  spinnerOn = true;

  double distErrorSquared = (location[0]-_currentState[0])*(location[0]-_currentState[0])+
    (location[1]-_currentState[1])*(location[1]-_currentState[1]);

  double orientationError = fabs(location[2] - _currentState[2]);

  if (distErrorSquared < maxDistanceErrorSquared && orientationError<maxOrientationError) {
    return 1;
  }

  // TODO(Karl) set desired velocities to output of trajectory planner once 
  //  the interface has been defined

  return 0;
}

// kicks the ball to a location flat along the floor.
// Equivilant of calling lobKickBallToLocation(location, speed, 0)
// see lobKickBallToLocation for more information
int Robot::kickBallToLocation(GVector::vector2d<double> location, double speed) {
  return lobKickBallToLocation(location, speed, 0.0);
}



int Robot::lobKickBallToLocation(GVector::vector2d<double> location, double speed, double height) {
  
}

int Robot::goToLocation(GVector::vector3d<double> location) {
  
}


void Robot::setTangentVelocity(double vTangent) {
  velTangent = vTangent;
}

void Robot::setNormalVelocity(double vNormal) {
  velNormal = vNormal;
}

void Robot::setAngularVelocity(double vAngular) {
  velAngular = vAngular;
}

void Robot::setKickSpeed(double speedX, double speedZ) {
  kickSpeedX = speedX;
}

void Robot::setSpinner(double on) {
  spinnerOn = on;
}

// Sends the robot's velocity commands to the simulator
// All velocities are relative to the robot
bool Robot::sendVelocityCommands() {
  grSim_Packet packet;
  
  packet.mutable_commands()->set_isteamyellow(team);
  packet.mutable_commands()->set_timestamp(0.0);
  grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
  command->set_id(playerID);

  command->set_wheelsspeed(false);

  //either individucal wheel velocities need to be set or tangent/normal/angular
  
  command->set_wheel1(0);
  command->set_wheel2(0);
  command->set_wheel3(0);
  command->set_wheel4(0);


  // Velocity commands are in the robot's reference frame
  command->set_veltangent(velTangent);
  command->set_velnormal(velNormal);
  command->set_velangular(velAngular);


  command->set_kickspeedx(kickSpeedX);
  command->set_kickspeedz(kickSpeedZ);


  command->set_spinner(spinnerOn);

  QByteArray dgram;
  dgram.resize(packet.ByteSize());
  packet.SerializeToArray(dgram.data(), dgram.size());
  udpsocket.writeDatagram(dgram, _addr, _port);

  //_communicator->sendPacket(dgram);
  return true;
}
