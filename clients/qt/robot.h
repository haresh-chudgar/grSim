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

#ifndef ROBOT_H
#define ROBOT_H

#include "communicator.h"
#include "trajectoryplanner.h"
#include "gvector.h"

class Robot
{
public:
    Robot(Communicator* communicator, TrajectoryPlanner* planner, char* team, int id);
    ~Robot();
    
    bool dribbleToLocation(GVector::vector2d<double> location);
    bool kickBallToLocation(GVector::vector2d<double> location);
    bool goToLocation(GVector::vector2d<double> location);
private:
  Communicator* _communicator;
  TrajectoryPlanner* _planner;
  
  const char* teamName;
  int playerID;
};

#endif // ROBOT_H