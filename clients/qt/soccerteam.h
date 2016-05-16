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

#ifndef SOCCERTEAM_H
#define SOCCERTEAM_H
 
#include "playbook.h"
#include "communicator.h"
#include "robot.h"
#include "pathplanner.h"
#include <eigen3/Eigen/Core>
#include "kdtree2.h"
#include "BotState.h"


using namespace std; 
using namespace Eigen;
class Robot;
class PlayBook;
class Play;

class SoccerTeam{
public:
    SoccerTeam(const bool isYellowTeam, 
               Communicator* communicator,
               PlayBook* playbook,
               const int num_robots);
    ~SoccerTeam();
    void SimCallback(int frameNumber, Vector3d ball, vector<BotState> *blueRobots, vector<BotState> *yellowRobots);
    void StartRobots(int num_robots);
    vector<Robot*> _robots;
    bool has_ball;
    bool scored;
    Play* _play;
private:
  const bool _isYellowTeam;
  Communicator* _communicator;
  const int _num_robots;
  PlayBook* _playbook;
  
  void FindKickAnglesOf(Robot* robot);
};

#endif // SOCCERTEAM_H
