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

class Evaluation
{
  static std::vector<KickAngles> EvaluateKickDirection(bool isYellowTeamKicking, Eigen::Vector2d kickFrom, Eigen::Vector2d kickToStart, Eigen::Vector2d kickToEnd);
};

#endif // EVALUATION_H
