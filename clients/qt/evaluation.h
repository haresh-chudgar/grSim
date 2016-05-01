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

#ifndef EVALUATION_H
#define EVALUATION_H

#include <eigen3/Eigen/Core>
#include <vector>
#include "soccerfieldinfo.h"

typedef struct _KickAngles  {
  double _opening;
  BotState _p1, _p2;
  _KickAngles(double opening, BotState p1, BotState p2):_opening(opening), _p1(p1), _p2(p2) {
  }
}KickAngles;

typedef struct _InterceptInfo {
  double _time;
  Eigen::Vector3d _interceptLocation;
  int _robotId;
  _InterceptInfo(double time, Eigen::Vector3d interceptLocation, int robotId):
  _time(time), _interceptLocation(interceptLocation), _robotId(robotId) {}
  
  bool operator < (const _InterceptInfo& bot) const
  {
    return (_time < bot._time);
  }
}InterceptInfo;

class Evaluation
{
public:
  static double ClosestRobotToBall(bool isTeamYellow, BotState* robot);
  static bool TeamHavingBall(BotState *robot);
  static std::vector<KickAngles> EvaluateKickDirection(bool isYellowTeamKicking, Eigen::Vector2d kickFrom, Eigen::Vector2d kickToStart, Eigen::Vector2d kickToEnd);
  static bool FindInterceptingRobots(bool isTeamYellow, std::vector<InterceptInfo> *interceptingBots);
  Eigen::MatrixXd openAngleFinder(std::vector<double> shooterPosition, int shooterInd, std::vector<double> targetSt, std::vector<double> targetEn, Eigen::MatrixXd robPosition_OwnTeam, Eigen::MatrixXd robPosition_Opponent);
  Eigen::VectorXd shotEvaluator(double queryRegion, int Num_queryPoints, int shooterInd, std::vector<double> targetSt, std::vector<double> targetEn, Eigen::MatrixXd robPosition_OwnTeam, Eigen::MatrixXd robPosition_Opponent);  
};

#endif // EVALUATION_H
