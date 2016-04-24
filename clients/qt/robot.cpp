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
  kGravity = 9.81;
}

Robot::~Robot(){}

void Robot::setCurrentState(Eigen::Vector3d currentState) {
  _currentState = currentState;
}

// TODO(Whoever is responsible for getting the state) get if robot has ball and if it is kicking
Eigen::Vector3d Robot::getCurrentState() {
  return _currentState;
}


// sets the velocity commands for the robot to dribble the ball to a location
// Precondition: The robot must have the ball
// Postcondition: The robot has the ball and is within the acceptable 
//    error margin of the given pos and orientation
// Assumes: There are no additional velocity constraints on the robot when
//    it is dribbling the ball
// args   : location - the {x, y, theta} the robot should arrive at 
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::dribbleToLocation(GVector::vector3d<double> location) {
  if (!hasBall) {
    return -1;
  }
  spinnerOn = true;

  return goToLocation(location);
}

// sets the velocity commands for the robot to kick the ball to a location 
//    along the floor
// Precondition: The robot must have the ball
// Postcondition: The robot has kicked the ball towards the location
// args   : location  - the {x, y} the ball should arrive at 
//          speed    - the horizontal speed of the ball
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::flatKickBallToLocation(GVector::vector2d<double> location, double speed) {
  return executeKickBallToLocation(location, speed, -1, -1);
}


// sets the velocity commands for the robot to kick the ball to a location 
//    through the air
// Precondition: The robot must have the ball
// Postcondition: The robot has kicked the ball towards the location
// args   : location  - the {x, y} the ball should arrive at 
//          height    - the maximum height of the ball
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::lobKickBallToLocation(GVector::vector2d<double> location, double height) {
  return executeKickBallToLocation(location, -1, height, -1);
}

// sets the velocity commands for the robot to kick the ball to a location 
//    through the air
// Precondition: The robot must have the ball
// Postcondition: The robot has kicked the ball towards the location
// args   : location        - the {x, y} the ball should arrive at 
//          height          - the maximum height of the ball
//          distOfMaxHeight - the distance the ball reaches the max height, -1 if flat kick
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::lobKickBallToLocation(GVector::vector2d<double> location, double height, double distOfMaxHeight) {
  return executeKickBallToLocation(location, -1, height, distOfMaxHeight);
}


// Private helper method that performs the kicking of the ball
// Precondition   : The robot must have the ball
// Postcondition  : The robot has kicked the ball towards the location
// args   : location        - the {x, y} the ball should arrive at 
//          speed           - the horizontal speed of the ball, -1 if lob kick
//          height          - the maximum height of the ball, -1 if flat kick
//          distOfMaxHeight - the distance the ball reaches the max height, -1 if flat kick
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::executeKickBallToLocation(GVector::vector2d<double> location, double speed, double height, double distOfMaxHeight) {
  // Takes advantage of the simulator's implementation of kicking that only has
  // the ball interact with the kicker during the first part of the kicking
  // sequence
  if (kicking) {
    return 1;
  }
  
  double xDist = location[0] - _currentState[0];
  double yDist = location[1] - _currentState[1];
  double desiredTheta = atan2(yDist, xDist);

  GVector::vector3d<double> desiredLoc(_currentState[0], _currentState[1], desiredTheta);
  int dribbleResult = dribbleToLocation(desiredLoc);
  if (dribbleResult != 1) {
    return dribbleResult;
  }

  //TODO(Karl): Talk with team planner about how they want invalid vels handled

  if(height < 0) {  // if kick is a flat kick
    kickSpeedX = speed;
    kickSpeedZ = 0;
  } else if (distOfMaxHeight < -1) { // if is a lob kick with only max height given
    kickSpeedZ = sqrt(2*height/kGravity);
    kickSpeedX = sqrt(xDist*xDist+yDist*yDist)*kGravity/(2*kickSpeedZ);
  } else { // if is a lob kick with distOfMaxHeight and height given
    kickSpeedZ = sqrt(2*height/kGravity);
    kickSpeedX = distOfMaxHeight*kGravity/(2*kickSpeedZ);
  }
  
  return 0;
}


// sets the velocity commands for the robot to move to a location
// Postcondition: The robot is within the acceptable 
//    error margin of the given pos and orientation
// args   : location - the {x, y, theta} the robot should arrive at 
// returns: 0   - still in progress
//          1   - success
//          -1  - failure
int Robot::goToLocation(GVector::vector3d<double> location) {
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
