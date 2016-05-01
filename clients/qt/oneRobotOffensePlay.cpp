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

#include "oneRobotOffensePlay.h"
#include "evaluation.h"
#include <iostream>

OneRobotOffensePlay::OneRobotOffensePlay() : offenseRobot(false), _complete(false){}

//Precondition: Team should have ball
bool OneRobotOffensePlay::Applicable() {
  bool has_ball = false;
  _isYellowTeam = true;
  BotState robot(_isYellowTeam);
  bool doesRobotHaveBall = Evaluation::TeamHavingBall(&robot);
  
  if(doesRobotHaveBall == true && robot._isYellow == _isYellowTeam) {
    has_ball = true;
  }
  
  fprintf(stderr, "OneRobotOffensePlay::Applicable %d %d %d %d %d\n", has_ball, doesRobotHaveBall, robot._isYellow == _isYellowTeam, robot._isYellow, _isYellowTeam);
  return has_ball;
}

bool OneRobotOffensePlay::Complete() {
  return _complete;
}

bool OneRobotOffensePlay::CompleteCondition() {
  return _complete;
}

bool OneRobotOffensePlay::Success() {
  return false;
}


//TODO currently hard coded, need conversion between BotState and robotId
void OneRobotOffensePlay::AssignRoles() {
  BotState robot(_isYellowTeam);
  Evaluation::TeamHavingBall(&robot);

  
  assignments = vector<int>(6);
  assignments[3] = 1;
}

void OneRobotOffensePlay::Begin(vector<Robot*>* team) {
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
  fprintf(stderr, "OneRobotOffensePlay::Begin\n");
}

void OneRobotOffensePlay::Execute() {
  for(size_t i = 0; i < 6; i++) {
    if(assignments[i] == 1) { // The offensive robot
      if(states[i] == 0) {
        double searchSize = 250;
        int particleCount = 1000;
        // needs to be tested
        int robotId = i;
        if (!_isYellowTeam) {
          i +=6;
        }

        vector<double> targetStart(2, 0);
        vector<double> targetEnd(2, 0);
        targetStart[1] = 0.35 * 1000;
        targetEnd[1] = -0.35 * 1000;
        // Only x coords of goals change
        if (_isYellowTeam) {
          targetStart[0] = -3 * 1000;
          targetEnd[0] = -3 * 1000;
        } else {
          targetStart[0] = 3 * 1000;
          targetEnd[0] = 3 * 1000;
        }

        
        MatrixXd ownTeamPos(6, 2);
        MatrixXd otherTeamPos(6, 2);
        
        std::vector<BotState> *otherTeamBots;
        if (_isYellowTeam) {
          otherTeamBots = SoccerFieldInfo::Instance()->blueTeamBots;
        } else {
          otherTeamBots = SoccerFieldInfo::Instance()->yellowTeamBots;
        }


       
        for (int k = 0; k<6; k++){
         
          ownTeamPos(k, 0) = _team->at(k)->CurrentState()[0];
          ownTeamPos(k, 1) = _team->at(k)->CurrentState()[1];
            
          

          otherTeamPos(k, 0) = otherTeamBots->at(k)._position[0];
          otherTeamPos(k, 1) = otherTeamBots->at(k)._position[1];
        }

        

        Eigen::VectorXd shootingPos = Evaluation::shotEvaluator(searchSize, 
            particleCount, robotId, targetStart, targetEnd, ownTeamPos, otherTeamPos);


        Eigen::Vector3d goal;
        goal[0] = shootingPos[0];
        goal[1] = shootingPos[1];
        goal[2] = (shootingPos[2] + shootingPos[3]) / 2;

        fprintf(stderr, "%f  %f %f %f\n", goal[0], goal[1], shootingPos[2], shootingPos[3]);

        _team->at(i)->goToLocation(1, goal);
        states[i] = 1;
      } else if (states[i] == 1) {
        if (_team->at(i)->execute() == 1) {
          states[i] = 2;
        }
      } else if (states[i] == 2) {
        fprintf(stderr, "OneRobotOffensePlay::Execute  kicking\n");
        _team->at(i)->setKickSpeed(10, 0);
        _team->at(i)->stopMoving();
        _team->at(i)->sendVelocityCommands();
      } else if (states[i] == 3) {
        _complete = true;
      } else {
        fprintf(stderr, "OneRobotOffensePlay::Execute Invalid state %d", states[i]);
      }
    }
  }
}

// Should in general be used by all plays to update based on success
void OneRobotOffensePlay::UpdateWeight() {
  //TODO Write weight updating
}

