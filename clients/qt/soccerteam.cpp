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

SoccerTeam::SoccerTeam(const bool isYellowTeam,
                       Communicator* communicator, 
                       PlayBook* playbook,
                       const int num_robots):
  _isYellowTeam(isYellowTeam),
  _communicator(communicator),
  _playbook(playbook),
  _num_robots(num_robots),
  _play(NULL){
  
  this->StartRobots(num_robots);
  has_ball = false;
  scored = false;
}

SoccerTeam::~SoccerTeam() {

}

// Callback for incoming state information from the simulator
void SoccerTeam::SimCallback(int frameNumber, Vector3d ball, vector<BotState> *blueRobots, vector<BotState> *yellowRobots) {
  
  // Save robot states based on isYellowTeam
  if(_isYellowTeam == true) {
    vector<BotState>::iterator iter1 = yellowRobots->begin();
    vector<Robot*>::iterator iter2 = _robots.begin();
    for(;iter1!=yellowRobots->end();++iter1,++iter2) {
      (*iter2)->setCurrentState((*iter1)._position, (*iter1)._velocity);
    }
  } else {
    vector<BotState>::iterator iter1 = blueRobots->begin();
    vector<Robot*>::iterator iter2 = _robots.begin();
    for(;iter1!=blueRobots->end();++iter1,++iter2) {
      Robot *r = *iter2;
      r->setCurrentState((*iter1)._position, (*iter1)._velocity);
    }
  }
  
  // Play Selection and handling
  if(_play == NULL || _play->Complete()) {
    if (_play->Complete()) {
      _playbook->UpdateWeight(_play, 1);
    }
    _play = _playbook->PlaySelection();
    _play->_isYellowTeam = _isYellowTeam;
    _play->Begin(&_robots);
  } else {
    _play->Execute();
  }
  
  /*
  for(size_t i = 0; i < _robots.size(); i++) {
    _robots[i]-> sendVelocityCommands();
  }
  */
}

// Initialize the robots for the isYellowTeam
void SoccerTeam::StartRobots(int num_robots) {
  for(int i = 0; i < num_robots; i++) {
    PathPlanner* planner = new PathPlanner(i, _isYellowTeam);
    _robots.push_back(new Robot(_communicator, planner, _isYellowTeam, i));
  }
}

















