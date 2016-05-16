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

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <eigen3/Eigen/Core>
#include <vector>
#include "rrtnode.h"
using namespace std;
class PathPlanner {
  public:
    PathPlanner(const size_t robot_ID, const bool isYellowTeam);
    ~PathPlanner();
   
    std::vector<Eigen::Vector3d> FindPath(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos);
    Eigen::Vector3d start_pos_;	//In millimeters
    Eigen::Vector3d goal_pos_;	//In millimeters

  private:
    double distToLineSeg(Eigen::Vector2d v, Eigen::Vector2d w, Eigen::Vector2d p);
    bool checkCollisions(Eigen::Vector3d v, Eigen::Vector3d w, vector<Eigen::Vector2d> dyn_obj_loc);
    pair<RRTNode, RRTNode> NearestNodes(vector<RRTNode> FRRT, vector<RRTNode> BRRT);
    vector<Eigen::Vector3d> FindWaypoints(vector<RRTNode> FRRT, vector<RRTNode> BRRT, RRTNode F_terminal, RRTNode B_terminal);
    vector<Eigen::Vector3d> SmoothWaypoints(vector<Eigen::Vector3d> waypoints, vector<Eigen::Vector2d> dyn_obj_loc);
    double fRand(double fMin, double fMax);

    bool isYellowTeam_;
    size_t robot_ID_;
    static constexpr double x_lim_ = 3000; //mm
    static constexpr double y_lim_ = 2000; //mm
    static constexpr int node_lim_ = 1000; //Number of nodes allowed in tree
    static constexpr double destination_epsilon_ = 5.0; //mm
    static constexpr double goal_bias_ = 0.15;
};

#endif
