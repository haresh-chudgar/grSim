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

	double searchSize = 10;
        int particleCount = 1000;
        Eigen::VectorXd shootingPos = Evaluation::shotEvaluator(searchSize, particleCount, robotId, targetStart, targetEnd, ownTeamPos, otherTeamPos);
	fprintf(stderr, "Calculating shooting angle!\n");
	Eigen::Vector3d goal = _team->at(i)->CurrentState();
	fprintf(stderr, "Robot State: %f  %f %f\n", goal[0], goal[1], goal[2]);
	
	goal = SoccerFieldInfo::Instance()->ball;
	goal[2] = (shootingPos[2] + shootingPos[3]) / 2;
// 	double changeInAngle = (goal[2] - _team->at(i)->CurrentState()[2]) / 2;
//         goal[0] = 2 * 89 * sin(changeInAngle) * cos(changeInAngle);
//         goal[1] = 2 * 89 * sin(changeInAngle) * sin(changeInAngle);
// 	
	goal = Evaluation::GetGoalPositionToBall(goal[2]);
	Eigen::Vector3d goalStateToKickFrom = goal;
	wayPointsToGoalState.push_back(goalStateToKickFrom);
	fprintf(stderr, "Final goal: %f, %f, %f\n", goalStateToKickFrom[0], goalStateToKickFrom[1], goalStateToKickFrom[2] * 180 / M_PI);
	
	Eigen::Vector2d robotAxis(cos(_team->at(i)->CurrentState()[2]), sin(_team->at(i)->CurrentState()[2]));
	Eigen::Vector2d goalSateAxis(cos(goalStateToKickFrom[2]), sin(goalStateToKickFrom[2]));
	if(robotAxis.dot(goalSateAxis) > 0) {
	  //Go Directly after backing up
	} else {
	  //Decide how to go?
	  //Rotate robot axis by 90 degrees
	  double temp = robotAxis[0]; //-sin(theta)
	  robotAxis[0] = -robotAxis[1];//cos(theta)
	  robotAxis[1] = temp;
	  Eigen::Vector3d wayPoint;
	  if(robotAxis.dot(goalSateAxis) >= 0) {
	    wayPoint = Evaluation::GetGoalPositionToBall(_team->at(i)->CurrentState()[2] + M_PI/2);
	  } else {
	    wayPoint = Evaluation::GetGoalPositionToBall(_team->at(i)->CurrentState()[2] - M_PI/2);
	  }
	  wayPoint[0] -= 400 * cos(wayPoint[2]);
	  wayPoint[1] -= 400 * sin(wayPoint[2]);
	  fprintf(stderr, "Waypoint: %f, %f, %f\n", wayPoint[0], wayPoint[1], wayPoint[2] * 180 / M_PI);
	  wayPointsToGoalState.push_back(wayPoint);
	}
	
        _team->at(i)->setSpinner(false);
	
	//Go back along current axis
	goal = _team->at(i)->CurrentState();
	goal[0] -= 90 * cos(goal[2]);
	goal[1] -= 90 * sin(goal[2]);
	
        _team->at(i)->goToLocation(1, goal);
	states[i] = 1;
	
      } else if (states[i] == 1) {
        if (_team->at(i)->execute() == 1) {
	  _team->at(i)->setSpinner(true);
	  if(wayPointsToGoalState.size() > 0) {
	    Eigen::Vector3d intermediateGoal = wayPointsToGoalState.back();
	    _team->at(i)->goToLocation(1, intermediateGoal);
	    wayPointsToGoalState.pop_back();
	  } else {
	    states[i] = 2;
	  }
        }
      } else if (states[i] == 2) {
	_team->at(i)->setSpinner(true);
	if (_team->at(i)->execute() == 1) {
	  fprintf(stderr, "OneRobotOffensePlay::Execute  kicking %f, %f, %f\n", _team->at(i)->CurrentState()[0], _team->at(i)->CurrentState()[1], _team->at(i)->CurrentState()[2]);
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
	    
	  }
	}
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

