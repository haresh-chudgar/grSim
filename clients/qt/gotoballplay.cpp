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

GoToBallPlay::GoToBallPlay() : ballAcquiringRobot(false), _complete(false) { }

//Precondition: Team should not have ball
bool GoToBallPlay::Applicable() {
  bool has_ball = false;
    
  BotState robot(_isYellowTeam);
  bool doesRobotHaveBall = Evaluation::TeamHavingBall(&robot);
  if(doesRobotHaveBall == true && robot._isYellow == _isYellowTeam) {
    has_ball = true;
  }
  return !has_ball;
}

bool GoToBallPlay::CompleteCondition() {
  
  bool has_ball = false;
  
  bool doesRobotHaveBall = Evaluation::TeamHavingBall(&ballAcquiringRobot);
  if(doesRobotHaveBall == true && ballAcquiringRobot._isYellow == _isYellowTeam) {
    has_ball = true;
  }
  //fprintf(stderr, "GoToBallPlay::CompleteCondition %d\n", has_ball);
  return has_ball;
}

bool GoToBallPlay::Success() {  
  return false;
}

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
	
        Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = (ball);

        // sets the desired angle to the robot's current angle
        //goal[2] = robot[2];

        // sets the desired angle so the robot points at the goal
        goal[2] = atan2(robot[0]-goal[0], robot[1]-goal[1]);
//        fprintf(stderr, "Goal and robot  %f %f %f  %f %f %f\n", goal[0], goal[1], goal[2], robot[0], robot[1], robot[2]);
      
        //Eigen::Vector3d offset = (robot-ball);
	      //offset.normalize();
        //offset = offset*70;
        //goal += offset;
        //offset.normalize();
        //goal(2) = acos(offset(0));
        // Call Move
        _team->at(i)->goToLocation(1, goal);
        
        //_team->at(i)->goToLocation(1, ball);
        states[i] = 1;
      } else if(states[i] == 1) { // Checking for location
        Eigen::Vector3d ball = SoccerFieldInfo::Instance()->ball;
	//ball[2] = 0;
        Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = (ball);

        // sets the desired angle to the robot's current angle
        //goal[2] = robot[2];

        // sets the desired angle so the robot points at the goal
        goal[2] = atan2(robot[0]-goal[0], robot[1]-goal[1]);
        
//        fprintf(stderr, "Goal and robot  %f %f %f  %f %f %f\n", goal[0], goal[1], goal[2], robot[0], robot[1], robot[2]);
        //Eigen::Vector3d offset = (robot-ball);
	      //offset.normalize();
        //offset = offset*70;
        //goal += offset;
        //offset.normalize();
	      //goal(2) = acos(offset(0));
        //if((robot - goal).norm() < .01){
        if((robot - goal).norm() < .01){
          states[i] = 2;
        }
        _team->at(i)->execute();
        
      } else if(states[i] == 2) { // Kicking
        fprintf(stderr, "Done with moving");
        // Call Kick
        states[i]++;
      } else {
        // do nothing
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
}

// Should in general be used by all plays to update based on success
void GoToBallPlay::UpdateWeight() {
  // Write weight updating
}
