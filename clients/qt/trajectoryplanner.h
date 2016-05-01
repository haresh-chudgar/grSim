
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

#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <eigen3/Eigen/Core>
#include <vector>


class TrajectoryPlanner {
  public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();
   
    std::vector<std::vector<double> > GenerateTrajectory(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos,
                      					 Eigen::Vector3d start_velo, Eigen::Vector3d goal_velo, 
						std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > waypoints);

    Eigen::Vector3d start_pos_;
    Eigen::Vector3d start_velo_; 
    Eigen::Vector3d goal_pos_;
    Eigen::Vector3d goal_velo_;
    //vector<pair<Eigen::Vector3d, Eigen::Vector3d> > waypoints_;

  private:
    static const double MAX_TRANS_VEL = 5.0; // m/s
    static const  double MAX_ROT_VEL = 4*3.14159; // rad/s

};

#endif // TRAJECTORYPLANNER_H
