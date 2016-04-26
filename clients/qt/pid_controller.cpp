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

PIDController::PIDController() {
  integral_error_ << 0,0,0;
  derivative_error_ << 0,0,0;
  prev_error_ << 0,0,0;
}

PIDController::~PIDController() {}

Eigen::Vector3d PIDController::ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos,                  								     Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo) {
  Eigen::Vector3d command;
  Eigen::Vector3d error = curr_pos - des_pos;
  command = Kp*error + Ki*integral_error_ + Kd*derivative_error_;
  Eigen::Vector2d trans(curr_velo(0), curr_velo(1));
  if (trans.norm() < MAX_TRANS_VEL && curr_velo(2) < MAX_ROT_VEL) {
    integral_error_ += error*dt;
  }
  derivative_error_ += (error - prev_error_)/dt;
  prev_error_ = error;
  return command;
}

std::vector<double> PIDController::ComputeCommandVoltage() {
  //need to implement for individual motor control
  std::vector<double> voltages;
  return voltages;
}

void PIDController::ResetError() {
  integral_error_ << 0,0,0;
  derivative_error_ << 0,0,0;
  prev_error_ << 0,0,0;
}
