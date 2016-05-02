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

#include "gotoballplay.h"
#include "evaluation.h"
#include <iostream>

GoToBallPlay::GoToBallPlay() : ballAcquiringRobot(false), _complete(false), doneMoving(false){ }

//Precondition: Team should not have ball
bool GoToBallPlay::Applicable() {
  
  //bool doesRobotHaveBall = Evaluation::TeamHavingBall(&robot);
  if(SoccerFieldInfo::Instance()->_teamInBallPossession == false && SoccerFieldInfo::Instance()->_robotWithBall._isYellow != _isYellowTeam) {
  //if(doesRobotHaveBall == true && robot._isYellow == _isYellowTeam) {
    fprintf(stderr, "GoToBallPlay::Applicable! \n");
    return true;
  }
  return false;
}

bool GoToBallPlay::CompleteCondition() {

  if (!doneMoving) {
    return false;
  }
  
  if(SoccerFieldInfo::Instance()->_teamInBallPossession == true && SoccerFieldInfo::Instance()->_robotWithBall._isYellow == _isYellowTeam)
    return true;
  return false;

  bool has_ball = false;
  
  bool doesRobotHaveBall = Evaluation::TeamHavingBall(&ballAcquiringRobot);

  if(doesRobotHaveBall == true && ballAcquiringRobot._isYellow == _isYellowTeam) {
    has_ball = true;
  }
  fprintf(stderr, "GoToBallPlay::CompleteCondition %d\n", has_ball);
  return has_ball;
}

bool GoToBallPlay::Success() {  
  return false;
}


//TODO currently hard coded, need conversion between BotState and robotId
void GoToBallPlay::AssignRoles() { 
  
  double distance = Evaluation::ClosestRobotToBall(_isYellowTeam, &ballAcquiringRobot);
  assignments = vector<int>(6);
  assignments[3] = 1;
}

void GoToBallPlay::Execute() { 
  // Roles are just numbers, can be enums in each class if necessary
  // used inside state machine
  for(size_t i = 0; i < 6; i++) {
    if(assignments[i] == 1) {
      if(states[i] == 0) { // Moving to Ball
        Eigen::Vector3d ball = SoccerFieldInfo::Instance()->ball;
	fprintf(stderr, "************ball location: %f %f %f\n",ball[0], ball[1], ball[2]);
        Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = ball;
	
        // sets the desired angle to the robot's current angle
        //goal[2] = robot[2];

        // sets the desired angle so the robot points at the goal
        goal[2] = atan2(robot[1]-goal[1], robot[0]-goal[0])+M_PI;
	fprintf(stderr, "Before: %f, %f\n", goal[0], goal[1]);
	goal = Evaluation::GetGoalPositionToBall(goal[2]);
	fprintf(stderr, "After: %f, %f, %f\n", goal[0], goal[1], goal[2] * 180 / M_PI);
	
        // Call Move
        _team->at(i)->goToLocation(1, goal);
        _team->at(i)->setSpinner(0);
        //_team->at(i)->goToLocation(1, ball);
        states[i] = 1;
      } else if(states[i] == 1) { // Checking for location
	  Eigen::Vector3d vel = _team->at(i)->CurrentVelocity();
	  //fprintf(stderr, "robotVel %f %f %f\n", vel[0], vel[1], vel[2]);
	  if (_team->at(i)->execute() == 1) {
	    states[i] = 2;
	  }
      } else if(states[i] == 2) { 
	// Kicking
	//_team->at(i)->setSpinner(1);
	
	_team->at(i)->stopMoving();
	//_team->at(i)->dribbleToLocation(Eigen::Vector3d(_team->at(i)->CurrentState()[0], _team->at(i)->CurrentState()[1], 225 * M_PI /180));
	states[i] = 3;
	/*
	Eigen::Vector2d r = Eigen::Vector2d(_team->at(i)->CurrentState()[0], _team->at(i)->CurrentState()[1]);
	Eigen::Vector2d loc = Eigen::Vector2d(SoccerFieldInfo::Instance()->ball[0], SoccerFieldInfo::Instance()->ball[1]);
	double distance = (r - loc).norm();
	fprintf(stderr, "Distance: %f\n", distance);
	if(distance > 120) {
	  states[i]++;
	  fprintf(stderr, "Last State\n");
	} else {
	  // Call Kick
	  fprintf(stderr, "Kicking\n");
	  _team->at(i)->setKickSpeed(2, 0);
	  _team->at(i)->sendVelocityCommands();
	  
	}*/
      } else {
	Eigen::Vector3d ballVel = SoccerFieldInfo::Instance()->ballVelocity;
	//fprintf(stderr, "BallVel %f %f %f\n", ballVel[0], ballVel[1], ballVel[2]);

	//if (_team->at(i)->execute() == 1) {
          doneMoving = true;
        //}
      }
    } 
  frames_running++;
  }
}

bool GoToBallPlay::Complete() {
  if(_complete){
    //fprintf(stderr, "GoToBallPlay::Complete returning true");
    return true;
  } else if(this->CompleteCondition()){
    _complete = true;
    this->UpdateWeight();
  }
  //fprintf(stderr, "GoToBallPlay::Complete %d\n", _complete);
  return _complete;
}

void GoToBallPlay::Begin(vector<Robot*>* team) {
  assignments.clear();
  states.clear();
  _team = team;
  for(size_t i = 0; i < _team->size(); i++) {
    states.push_back(0);
  }
  // First bot is always the goalie (more complex plays could pull the goalie to another role)
  assignments.push_back(9);
  this->AssignRoles();
  this->Execute();
  fprintf(stderr, "GoToBallPlay::Begin\n");
}

// Should in general be used by all plays to update based on success
void GoToBallPlay::UpdateWeight() {
  //TODO Write weight updating
}
