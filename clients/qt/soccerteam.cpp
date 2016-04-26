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

#include "soccerteam.h"
#include "soccerfieldinfo.h"

using namespace std;
using namespace Eigen;

SoccerTeam::SoccerTeam(const bool team, // true = yellow, false = blue
                       Communicator* communicator, 
                       TrajectoryPlanner* planner,
                       const int num_robots):
  _team(team),
  _communicator(communicator),
  _planner(planner), 
  _playbook(playbook),
  _num_robots(num_robots) {

  this->StartRobots(num_robots);
  has_ball = false;
  scored = false;
}

SoccerTeam::~SoccerTeam() {

}

// Callback for incoming state information from the simulator
void SoccerTeam::SimCallback(int frameNumber, Vector3d ball, vector<BotState> *blueRobots, vector<BotState> *yellowRobots) {
  
  kdtree2_array bRobotPositions(extents[blueRobots->size()][2]), yRobotPositions(extents[yellowRobots->size()][2]);
  
  vector<BotState>::iterator iter1 = yellowRobots->begin();
  vector<Robot*>::iterator iter2 = _robots.begin();
  for(int index = 0;iter1!=yellowRobots->end();++iter1,++iter2, ++index) {
    if(_team == (*iter1)._isYellow)
      (*iter2)->setCurrentState((*iter1)._position);
    yRobotPositions[index][0] = (*iter1)._position[0];
    yRobotPositions[index][1] = (*iter1)._position[1];
  }
  delete yellowTeamTree;
  yellowTeamTree = new kdtree2(yRobotPositions, true);
  
  iter1 = blueRobots->begin();
  iter2 = _robots.begin();
  for(int index = 0;iter1!=blueRobots->end();++iter1,++iter2) {
    if(_team == (*iter1)._isYellow)
      (*iter2)->setCurrentState((*iter1)._position);
    bRobotPositions[index][0] = (*iter1)._position[0];
    bRobotPositions[index][1] = (*iter1)._position[1];
  }
  delete blueTeamTree;
  blueTeamTree = new kdtree2(bRobotPositions, true);
  
  // if(_play->Complete()){
  //   _play = playbook.PlaySelection();
  // }
  
  // Makes robots send velocity commands to simulator
  for(size_t i = 0; i < _robots.size(); i++){
    _robots[i]->sendVelocityCommands();
  }
// }
}

// Initialize the robots for the team
void SoccerTeam::StartRobots(int num_robots) {
  for(int i = 0; i < num_robots; i++) {
    _robots.push_back(new Robot(_communicator, _planner, _team, i));
  }
  //Could potentially initialize the positions of the robots if desired (from a list of positions)
}

void SoccerTeam::FindKickAnglesOf(Robot* robot) {
  if(robot->isYellowTeam == true) { 
    
  }
}

struct DefenseBot
{
    double _angle;
    BotState _botPosition;

    DefenseBot(double angle, BotState& botPosition) : _angle(angle), _botPosition(botPosition) {}

    bool operator < (const DefenseBot& bot) const
    {
        return (_angle < bot._angle);
    }
};

void SoccerTeam::EvaluateDefenseManeuver() {
  
  //Assuming defense is yellow team, offense is blue team
  kdtree2_result_vector result;
  std::vector<float> queryPoint(2);
  queryPoint[0] = SoccerFieldInfo::Instance()->ball[0];
  queryPoint[1] = SoccerFieldInfo::Instance()->ball[1];
  blueTeamTree->n_nearest(queryPoint, (int)SoccerFieldInfo::Instance()->blueTeamBots->size(), result);
  
  /*Find robot who has the ball - striker*/
  Eigen::Vector2d striker = Eigen::Vector2d(blueTeamTree->the_data[result[0].idx][0],
					      blueTeamTree->the_data[result[0].idx][1]);
  
  /*Find chance of striker kicking to the goal*/
  //Find defenders which are in front of the striker
  std::vector<DefenseBot> defenseBots;
  std::vector<BotState>::iterator iter = SoccerFieldInfo::Instance()->yellowTeamBots->begin();
  for(; iter != SoccerFieldInfo::Instance()->yellowTeamBots->end(); iter++) {
    //If the robot's x co ordinate is greater than striker's
    if((*iter)._position[0] > striker[0]) {
      double angle = atan2((*iter)._position[1] - striker[1], (*iter)._position[0] - striker[0]);
      defenseBots.push_back(DefenseBot(angle, *iter));
    }
  }
  std::sort(defenseBots.begin(), defenseBots.end());
  
  //Find angle of striker to both sides of the goal post
  double angle1 = atan2(0.35 - striker[1], 3.0 - striker[0]);
  double angle2 = atan2(-0.35 - striker[1], 3.0 - striker[0]);
  if(angle2 < angle1) {
    double temp = angle2;
    angle2 = angle1;
    angle1 = temp;
  }
  int startIndex = -1;
  std::vector<DefenseBot>::iterator dIter = defenseBots.begin();
  for(; dIter != defenseBots.end(); dIter++, startIndex++) {
    if((*dIter)._angle > angle1)
      break;
  }
  /*
  int endIndex = defenseBots.size();
  for(dIter = defenseBots.begin(); dIter != defenseBots.rend(); dIter++, endIndex--) {
    if((*dIter)._angle > angle1)
      break;
  }
  */
  //Find another enemy robot with the best chance of kicking to the goal
  //Find enemy robots near the ball
  
  //Find ideal positions to defend goal post
  //Place one robot to intercept pass to the 2nd best enemy robot
}


















