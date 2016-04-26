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

#include "trajectoryplanner.h"
#include <vector>
#include <iostream>

TrajectoryPlanner::TrajectoryPlanner(){}

TrajectoryPlanner::~TrajectoryPlanner() {}

std::vector<std::vector<double> > TrajectoryPlanner::GenerateTrajectory(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos,                  								     Eigen::Vector3d start_velo, Eigen::Vector3d goal_velo,
							std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > waypoints) {

  start_pos_ = start_pos;
  start_velo_ = start_velo;
  goal_pos_ = goal_pos;
  goal_velo_ = goal_velo;

  std::vector<std::vector<double> > path;
  std::vector<double> path_x;
  std::vector<double> path_y;
  std::vector<double> path_theta;
  if (waypoints.size() != 0) {
    std::cout << "need to handle waypoints still" << std::endl;
  }
  else { //naive implementation
    Eigen::Vector3d opp_goal_dir = start_pos_ - goal_pos_;
    path_x.push_back(opp_goal_dir(0)/MAX_TRANS_VEL);
    path_y.push_back(opp_goal_dir(1)/MAX_TRANS_VEL);
    path_theta.push_back(opp_goal_dir(2)/MAX_ROT_VEL);
  }
  path.push_back(path_x);
  path.push_back(path_y);
  path.push_back(path_theta);
  return path;
}
