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

#ifndef SOCCERFIELDINFO_H
#define SOCCERFIELDINFO_H

#include "messages_robocup_ssl_wrapper.pb.h"
#include <QUdpSocket>
#include <QMutex>
#include <eigen3/Eigen/Core>
#include <vector>
#include "soccerteam.h"
#include "BotState.h"

#define BALL_RADIUS 21.5 //In millimeters
#define ROBOT_RADIUS 89 //In millimeters

using namespace std;
class SoccerTeam;
class SoccerFieldInfo
{
public:
  static SoccerFieldInfo* Instance();
  static void CreateInstance(SoccerTeam* blueTeam, SoccerTeam* yellowTeam);
  
  void receive(char* buffer, int size);
  
  Eigen::Vector3d ball;
  Eigen::Vector3d ballVelocity;
  
  bool _teamInBallPossession;
  BotState _robotWithBall;
  
  std::vector<BotState> *yellowTeamBots;
  std::vector<BotState> *blueTeamBots;
  SoccerTeam *_blueTeam, *_yellowTeam;
private:
  SoccerFieldInfo(SoccerTeam* blueTeam, SoccerTeam* yellowTeam);
  ~SoccerFieldInfo();
  const double kFrameRate;
  int frameNumber;
};

#endif // SOCCERFIELDINFO_H
