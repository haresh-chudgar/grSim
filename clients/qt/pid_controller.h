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

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <vector>


class PIDController {
  public:
    PIDController();
    ~PIDController();
   
    Eigen::Vector3d ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos,
				       Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo);

    std::vector<double> ComputeCommandVoltage();

  private:
    static const double MAX_TRANS_VEL = 5.0; // m/s
    static const  double MAX_ROT_VEL = 4*3.14159; // rad/s

};

#endif 
