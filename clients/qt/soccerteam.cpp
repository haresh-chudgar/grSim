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
using namespace std;
using namespace Eigen;

SoccerTeam::SoccerTeam(const bool team, // true = yellow, false = blue
                       Communicator* communicator, 
                       TrajectoryPlanner* planner,
                       const int num_robots):
  _team(team),
  _communicator(communicator),
  _planner(planner), 
  _num_robots(num_robots) {

  this->StartRobots(num_robots);
}

SoccerTeam::~SoccerTeam() {

}

// Callback for incoming state information from the simulator
void SoccerTeam::SimCallback(int frameNumber, Vector3d ball, vector<Vector3d> *blueRobots, vector<Vector3d> *yellowRobots) {
  
  if(_team == true) {
    vector<Vector3d>::iterator iter1 = yellowRobots->begin();
    vector<Robot*>::iterator iter2 = _robots.begin();
    for(;iter1!=yellowRobots->end();++iter1,++iter2) {
      (*iter2)->setCurrentState(*iter1);
    }
  } else {
    vector<Vector3d>::iterator iter1 = blueRobots->begin();
    vector<Robot*>::iterator iter2 = _robots.begin();
    for(;iter1!=blueRobots->end();++iter1,++iter2) {
      Robot *r = *iter2;
      r->setCurrentState(*iter1);
    }
  }
  
//   if(_plan) {
//     // Plan a gameplan
    
//     // Execute Plan (send new commands to each robot)
//   }
  
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
