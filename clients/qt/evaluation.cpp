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

#include "evaluation.h"
#include "kdtree2.h"
#include  <math.h>
#include <cfloat>

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

bool Evaluation::TeamHavingBall(BotState *robot) {
  
  if(SoccerFieldInfo::Instance()->ball[2] > 0)
    return false;
  
  double distToNearestYellowBot = DBL_MAX;
  BotState nearestYellowBot(true);
  std::vector<BotState> *team = SoccerFieldInfo::Instance()->yellowTeamBots;
  std::vector<BotState>::iterator iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestYellowBot) {
      nearestYellowBot = *iter;
      distToNearestYellowBot = distToBall;
    }
  }
  
  double distToNearestBlueBot = DBL_MAX;
  BotState nearestBlueBot(false);
  team = SoccerFieldInfo::Instance()->blueTeamBots;
  iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestBlueBot) {
      nearestBlueBot = *iter;
      distToNearestBlueBot = distToBall;
    }
  }
  
  if(distToNearestBlueBot > 0.0215000000 && distToNearestYellowBot > 0.0215000000) {
    return false;
  }
  if(distToNearestBlueBot < distToNearestYellowBot) {
    *robot = nearestBlueBot;
  } else {
    *robot = nearestYellowBot;
  }
  return true;
}

std::vector< KickAngles > Evaluation::EvaluateKickDirection(bool isYellowTeamKicking, Eigen::Vector2d kickFrom, Eigen::Vector2d kickToStart, Eigen::Vector2d kickToEnd)
{
  std::vector<BotState> *kickingTeam, *defendingTeam;
  if(isYellowTeamKicking) {
    kickingTeam = SoccerFieldInfo::Instance()->yellowTeamBots;
    defendingTeam = SoccerFieldInfo::Instance()->blueTeamBots;
  } else {
    kickingTeam = SoccerFieldInfo::Instance()->blueTeamBots;
    defendingTeam = SoccerFieldInfo::Instance()->yellowTeamBots;
  }
  
  /*Find chance of striker kicking between the points*/
  //Find defenders which are in front of the striker
  std::vector<DefenseBot> defenseBots;
  double vx = (kickToStart[0] + kickToEnd[0] / 2) - kickFrom[0];
  double vy = (kickToStart[1] + kickToEnd[1] / 2) - kickFrom[1];
  Eigen::Vector2d kickVector(vx / sqrt(vx*vx + vy*vy), vy / sqrt(vx*vx + vy*vy));
  
  std::vector<BotState>::iterator iter = defendingTeam->begin();
  for(; iter != defendingTeam->end(); iter++) {
    vx = (*iter)._position[0] - kickFrom[0];
    vy = (*iter)._position[1] - kickFrom[1];
    Eigen::Vector2d obstacleVector(vx / sqrt(vx*vx + vy*vy), vy / sqrt(vx*vx + vy*vy));
    double dotProduct = obstacleVector[0]*kickVector[0] + obstacleVector[1]*kickVector[1];
    if(dotProduct >= 0) {
      double angle = atan2((*iter)._position[1] - kickFrom[1], (*iter)._position[0] - kickFrom[0]);
      defenseBots.push_back(DefenseBot(angle, *iter));
    }
  }
  if(defenseBots.size() == 0)
    return std::vector<KickAngles>();
  
  //Sort the bots according to the angles with x axis
  std::sort(defenseBots.begin(), defenseBots.end());
  
  //Find angle of striker to both sides of the kick region
  Eigen::Vector3d side1 = Eigen::Vector3d(kickToStart[0], kickToStart[1], 0), side2 = Eigen::Vector3d(kickToEnd[0], kickToEnd[1], 0);
  double angle1 = atan2(kickToStart[1] - kickFrom[1], kickToStart[0] - kickFrom[0]);
  double angle2 = atan2(kickToEnd[1] - kickFrom[1], kickToEnd[0] - kickFrom[0]);
  if(angle2 < angle1) {
    double temp = angle2;
    angle2 = angle1;
    angle1 = temp;
    
    Eigen::Vector3d tempV;
    tempV = side1;
    side1 = side2;
    side2 = side1;
  }
  
  //Find bots at the two extremes
  int startIndex = -1;
  std::vector<DefenseBot>::iterator dIter = defenseBots.begin();
  for(; dIter != defenseBots.end(); dIter++, startIndex++) {
    if((*dIter)._angle > angle1)
      break;
  }
  int endIndex = defenseBots.size();
  std::vector<DefenseBot>::reverse_iterator rIter = defenseBots.rbegin();
  for(; rIter != defenseBots.rend(); rIter++, endIndex--) {
    if((*rIter)._angle < angle2)
      break;
  }
  
  std::vector<KickAngles> kickOpenings;
  //If a robot is present just before the line between bot and kick point of min angle
  if(startIndex > -1 && startIndex < defenseBots.size()) {
    kickOpenings.push_back(KickAngles(defenseBots.at(startIndex)._angle - angle1, BotState(isYellowTeamKicking, side1), defenseBots.at(startIndex)._botPosition));
    ++startIndex;
  }
  //If a robot is present just after the line between bot and kick point of max angle
  if(endIndex < defenseBots.size() && endIndex > -1) {
    kickOpenings.push_back(KickAngles(angle2 - defenseBots.at(endIndex)._angle, defenseBots.at(endIndex)._botPosition, BotState(isYellowTeamKicking, side2)));
    --endIndex;
  }
  
  if(startIndex > -1 && startIndex < defenseBots.size() && endIndex < defenseBots.size() && endIndex > -1) {
    for(int index = startIndex; index < endIndex; index++) {
      kickOpenings.push_back(KickAngles(defenseBots.at(index+1)._angle - defenseBots.at(index)._angle, defenseBots.at(index)._botPosition, defenseBots.at(index+1)._botPosition));
    }
  }
  
  return kickOpenings;
}

double Evaluation::ClosestRobotToBall(bool isTeamYellow, BotState* robot) {
  
  double distToNearestBot = DBL_MAX;
  
  std::vector<BotState> *team = SoccerFieldInfo::Instance()->yellowTeamBots;
  if(isTeamYellow == false) {
    team = SoccerFieldInfo::Instance()->blueTeamBots;
  }
  
  std::vector<BotState>::iterator iter = team->begin();
  for(;iter!=team->end();++iter) {
    double distToBall = (*iter).distanceToLocation(SoccerFieldInfo::Instance()->ball);
    if(distToBall < distToNearestBot) {
      *robot = *iter;
      distToNearestBot = distToBall;
    }
  }
  
  return distToNearestBot;
}


















