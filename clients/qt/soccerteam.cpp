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

      _robots = this->StartRobots(num_robots);
}

SoccerTeam::~SoccerTeam() {

}

// Callback for incoming state information from the simulator
void SoccerTeam::SimCallback(Vector3d ball, vector<Vector3d> robots, vector<Vector3d> robots_2) {
//   if(_plan) {
//     // Plan a gameplan
    
//     // Execute Plan (send new commands to each robot)
//   }
  

//   for(size_t i = 0; i < _robots.size(); i++){
//     robots[i].Execute();
//   }
// }
}

// Initialize the robots for the team
vector<Robot*> SoccerTeam::StartRobots(int num_robots) {
  vector<Robot*> robots;
  for(int i = 0; i < num_robots; i++) {
    robots.push_back(new Robot(_communicator, _planner, _team, i));
  }
  //Could potentially initialize the positions of the robots if desired (from a list of positions)
}