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
#include <stdio.h>
#include <vector>
#include <iostream>

PIDController::PIDController() {
  integral_error_ << 0,0,0;
  derivative_error_ << 0,0,0;
  prev_error_ << 0,0,0;
  
  Kp = -3.00;
  Ki = -0.001;
  Kd = -0.001;
  dt = 1.0/60.0;
}

PIDController::~PIDController() {}

Eigen::Vector3d PIDController::ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos, Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo) {
  Eigen::Vector3d command;
  //fprintf(stderr, "PIDController::ComputeCommandVelo curr_pos %f %f %f\n", curr_pos[0], curr_pos[1], curr_pos[2]);
  //fprintf(stderr, "PIDController::ComputeCommandVelo des_pos %f %f %f\n", des_pos[0], des_pos[1], des_pos[2]);
  Eigen::Vector3d error = curr_pos - des_pos;

  //fixes wraparound
  if (fabs(error[2] + M_PI*2) < fabs(error[2])) {
    error[2] = error[2] + M_PI*2;
  } else if (fabs(error[2] - M_PI*2) < fabs(error[2])) {
    error[2] = error[2] - M_PI*2;
  }

  //fprintf(stderr, "PIDController::ComputeCommandVelo error: %f,%f,%f\n", error[0], error[1], error[2]);
  //fprintf(stderr, "Commands: %f %f %f\n", Kp*error[2], Ki*integral_error_[2], Kd*derivative_error_[2]);
  command = Kp*error + Ki*integral_error_ + Kd*derivative_error_;
  Eigen::Vector2d trans(curr_velo(0), curr_velo(1));
  if (trans.norm() < MAX_TRANS_VEL && curr_velo(2) < MAX_ROT_VEL) {
    integral_error_ += error*dt;
  }
  derivative_error_ = (error - prev_error_)/dt;
  //fprintf(stderr, "DError %f %f %f\n", derivative_error_[0], derivative_error_[1], derivative_error_[2]);
  prev_error_ = error;

  Eigen::Vector2d transSpeed(command[0], command[1]);
  if(transSpeed.norm() > MAX_TRANS_VEL) {
    fprintf(stderr, "*********************MAX_TRANS_VEL**********************");
    transSpeed.normalize();
    command[0] = transSpeed[0] * MAX_TRANS_VEL;
    command[1] = transSpeed[1] * MAX_TRANS_VEL;
  }
  if(command[2] > MAX_ROT_VEL) {
    command[2] = MAX_ROT_VEL;
  }
//  fprintf(stderr, "PIDController::ComputeCommandVelo %f,%f,%f\n", command[0], command[1], command[2]);
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
