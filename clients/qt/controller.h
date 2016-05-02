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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <vector>


class Controller {
  public:
    Controller();
    ~Controller();
   
    Eigen::Vector3d ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos,
				       Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo);

    std::vector<double> ComputeCommandVoltage();
  private:
    double Normalize(double norm_me, double w_f);
    std::vector<double> DetermineBangBangCaseAndUpdate(double wdot_0, double w_f, bool rotation);
    std::vector<double> Case1(double wdot_0, double w_f);
    std::vector<double> Case2(double wdot_0, double w_f);
    std::vector<double> Case3(double wdot_0, double w_f);
    std::vector<double> Case4(double wdot_0, double w_f);
    std::vector<double> Case5(double wdot_0, double w_f);
    
    static constexpr double mu_ = 0.8; // friction coeff
    static constexpr double g_ = 9.81; // m/s^2
    static constexpr double m_ = 2.7; // kg
    static constexpr double J_ = 0.0085; // m^2/kg
    static constexpr double l_ = 0.08; // m (radius)
    static constexpr double h_ = 0.05; // m 
    //Below values need to be for single dimensions, ie x,y,z,theta
    static constexpr double MAX_TRANS_ACC = 5.0; // m/s^2
    static constexpr double MAX_ROT_ACC = 4*3.14159; // rad/s^2
    static constexpr double MAX_TRANS_VEL = 5.0; // m/s
    static constexpr double MAX_ROT_VEL = 4*3.14159; // rad/s
};

#endif 
