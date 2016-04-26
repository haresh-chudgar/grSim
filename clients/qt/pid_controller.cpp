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

#include "pid_controller.h"
#include <vector>
#include <iostream>

PIDController::PIDController() {}

PIDController::~PIDController() {}

Eigen::Vector3d PIDController::ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos,                  								     Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo) {
  Eigen::Vector3d command;


  return command;
}

std::vector<double> PIDController::ComputeCommandVoltage() {
  //need to implement for individual motor control
  std::vector<double> voltages;
  return voltages;
}
