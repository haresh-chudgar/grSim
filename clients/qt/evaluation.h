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
#include "robot.h"

typedef struct _KickAngles  {
  double _opening;
  Robot* _p1; 
  Robot* _p2;
  _KickAngles(double opening, Robot* p1, Robot* p2):_opening(opening), _p1(p1), _p2(p2) {
  }
}KickAngles;

typedef struct _InterceptInfo {
  double _time;
  Eigen::Vector3d _interceptLocation;
  Eigen::Vector3d _interceptLocation1;
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
  static bool FindInterceptingRobots(bool isTeamYellow, std::vector<InterceptInfo> *interceptingBots);
  static int ClosestRobotToBall(bool isTeamYellow);
  static int TeamHavingBall(bool isTeamYellow);
  static std::vector<KickAngles> EvaluateKickDirection(bool isYellowTeamKicking, Eigen::Vector2d kickFrom, Eigen::Vector2d kickToStart, Eigen::Vector2d kickToEnd);
  static Eigen::MatrixXd openAngleFinder(std::vector<double> shooterPosition, int shooterInd, std::vector<double> targetSt, std::vector<double> targetEn, Eigen::MatrixXd robPosition_OwnTeam, Eigen::MatrixXd robPosition_Opponent);
  static Eigen::VectorXd shotEvaluator(double queryRegion, int Num_queryPoints, int shooterInd, std::vector<double> targetSt, std::vector<double> targetEn, Eigen::MatrixXd robPosition_OwnTeam, Eigen::MatrixXd robPosition_Opponent);    
  static bool isOutOfField(Eigen::Vector3d pos);
  static Eigen::Vector3d GetGoalPositionToBall(double targetAngle);
};

#endif // EVALUATION_H
