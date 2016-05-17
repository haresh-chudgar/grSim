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
                       PlayBook* playbook,
                       const int num_robots):
  _isYellowTeam(isYellowTeam),
  _playbook(playbook),
  _num_robots(num_robots),
  _play(NULL){
  
  //this->StartRobots(num_robots);
  has_ball = false;
  scored = false;
}

SoccerTeam::~SoccerTeam() {

}

// Callback for incoming state information from the simulator
void SoccerTeam::SimCallback(int frameNumber, Vector3d ball, vector<Robot*>* blueRobots, vector<Robot*>* yellowRobots) {
  
  //Updating team specific predicates
  _possession = Evaluation::TeamHavingBall(_isYellowTeam);
  
  // Play Selection and handling
  if(_play == NULL || _play->Complete()) {
    if (_play->Complete()) {
      _playbook->UpdateWeight(_play, 1);
    }
    _play = _playbook->PlaySelection();
    _play->_isYellowTeam = _isYellowTeam;
    if(_isYellowTeam){
      _play->Begin(SoccerFieldInfo::Instance()->yellow_bots);
    } else {
      _play->Begin(SoccerFieldInfo::Instance()->blue_bots);
    }
  } else {
    _play->Execute();
  }
  
  /*
  for(size_t i = 0; i < _robots.size(); i++) {
    _robots[i]-> sendVelocityCommands();
  }
  */
}

// Initialize the robots for the team
// void SoccerTeam::StartRobots(int num_robots) {
//   for(int i = 0; i < num_robots; i++) {
//     PathPlanner* planner = new PathPlanner(i, _isYellowTeam);
//     _robots.push_back(new Robot(_communicator, planner, _isYellowTeam, i));
//   }
// }

















