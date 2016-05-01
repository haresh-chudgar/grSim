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

//following algorithm outlined by O. Purwin and R. D'Andrea presented in 
//Robotics and Autonomous Systems (2006)

//TODO: Finish handling the case of rotation vs. translation, ie different limits
#include "controller.h"
#include <vector>
#include <iostream>

Controller::Controller() {
}

Controller::~Controller() {}

double Controller::Normalize(double norm_me, double w_f) {
  if (w_f < 0) {
    return -norm_me;
  }
  else {
    return norm_me;
  }
}

std::vector<double> Controller::Case1(double wdot_0, double w_f) {
  std::vector<double> update;
  double q = MAX_TRANS_ACC;
  double t = -wdot_0/MAX_TRANS_ACC;
  double w = -(wdot_0*wdot_0)/(2*MAX_TRANS_ACC);
  double wdot = 0.0;
  update.push_back(q);
  update.push_back(t);
  update.push_back(w);
  update.push_back(wdot);
  return update;
}

std::vector<double> Controller::Case2(double wdot_0, double w_f) {
  std::vector<double> update;

  double wdot_1 = sqrt(w_f*MAX_TRANS_ACC + (wdot_0*wdot_0)/2);
  double t1 = (MAX_TRANS_VEL - wdot_0)/MAX_TRANS_ACC;
  double t11 = (wdot_1 - wdot_0)/MAX_TRANS_ACC;
  
  double q, t, w, wdot;
  if (t1 < t11) {
    q = MAX_TRANS_ACC;
    t = t1;
    w = wdot_0*t + 0.5*MAX_TRANS_ACC*t*t;
    wdot = MAX_TRANS_VEL;
  }
  else {
    q = MAX_TRANS_ACC;
    t = t11;
    w = w_f/2 + (wdot_0*wdot_0)/(2*MAX_TRANS_ACC);
    wdot = MAX_TRANS_VEL;
  }
  update.push_back(q);
  update.push_back(t);
  update.push_back(w);
  update.push_back(wdot);
  return update;
}

std::vector<double> Controller::Case3(double wdot_0, double w_f) {
  std::vector<double> update;
  double q = 0.0;
  double t = w_f/MAX_TRANS_VEL - MAX_TRANS_VEL/(2*MAX_TRANS_ACC);
  double w = w_f - (MAX_TRANS_VEL*MAX_TRANS_VEL)/(2*MAX_TRANS_ACC);
  double wdot = MAX_TRANS_VEL;
  update.push_back(q);
  update.push_back(t);
  update.push_back(w);
  update.push_back(wdot);
  return update;
}

std::vector<double> Controller::Case4(double wdot_0, double w_f) {
  std::vector<double> update;
  double q = -MAX_TRANS_ACC;
  double t = wdot_0/MAX_TRANS_ACC;
  double w = (wdot_0*wdot_0)/(2*MAX_TRANS_ACC);
  double wdot = 0.0;
  update.push_back(q);
  update.push_back(t);
  update.push_back(w);
  update.push_back(wdot);
  return update;
}

std::vector<double> Controller::Case5(double wdot_0, double w_f) {
  std::vector<double> update;
  double q = -MAX_TRANS_ACC;
  double t = (wdot_0 - MAX_TRANS_VEL)/MAX_TRANS_ACC;
  double w = (wdot_0*wdot_0 - MAX_TRANS_VEL*MAX_TRANS_VEL)/(2*MAX_TRANS_ACC);
  double wdot = MAX_TRANS_VEL;
  update.push_back(q);
  update.push_back(t);
  update.push_back(w);
  update.push_back(wdot);
  return update;
}

std::vector<double> Controller::DetermineBangBangCaseAndUpdate(double wdot_0, double w_f, bool rotation) {
  if (!rotation) {
    if (wdot_0 < 0.0) { //going the wrong way!
      return Case1(wdot_0, w_f);
    }
    else if (wdot_0 < MAX_TRANS_VEL && w_f > (wdot_0*wdot_0)/(2*MAX_TRANS_ACC)) { //speed up
      return Case2(wdot_0, w_f);
    }
    else if (wdot_0 == MAX_TRANS_VEL && w_f > (wdot_0*wdot_0)/(2*MAX_TRANS_ACC)) { //cruise
      return Case3(wdot_0, w_f);
    }
    else if (wdot_0 <= MAX_TRANS_VEL && w_f <= (wdot_0*wdot_0)/(2*MAX_TRANS_ACC)) { //slow down to reach goal
      return Case4(wdot_0, w_f);
    }
    else if (wdot_0 > MAX_TRANS_VEL) { //exceeds velocity limits
      return Case5(wdot_0, w_f);
    }
    else {
      std::vector<double> update;
      return update;
    }
  }
  else {
    if (wdot_0 < 0.0) { //going the wrong way!
      return Case1(wdot_0, w_f);
    }
    else if (wdot_0 < MAX_ROT_VEL && w_f > (wdot_0*wdot_0)/(2*MAX_ROT_ACC)) { //speed up
      return Case2(wdot_0, w_f);
    }
    else if (wdot_0 == MAX_ROT_VEL && w_f > (wdot_0*wdot_0)/(2*MAX_ROT_ACC)) { //cruise
      return Case3(wdot_0, w_f);
    }
    else if (wdot_0 <= MAX_ROT_VEL && w_f <= (wdot_0*wdot_0)/(2*MAX_ROT_ACC)) { //slow down to reach goal
      return Case4(wdot_0, w_f);
    }
    else if (wdot_0 > MAX_ROT_VEL) { //exceeds velocity limits
      return Case5(wdot_0, w_f);
    }
    else {
      std::vector<double> update;
      return update;
    }
  }
}

Eigen::Vector3d Controller::ComputeCommandVelo(Eigen::Vector3d curr_pos, Eigen::Vector3d des_pos,                  								     Eigen::Vector3d curr_velo, Eigen::Vector3d des_velo) {
  Eigen::Vector3d controls;
  
  //Initialize the problem and constraints
  double wx_0 = curr_pos(0);
  double wdotx_0 = curr_pos(0);
  double wx_f = des_pos(0);
  double wdotx_f = 0.0; //des_velo(0) if went non-zero terminal velocities
  
  double wy_0 = curr_pos(1);
  double wdoty_0 = curr_pos(1);
  double wy_f = des_pos(1);
  double wdoty_f = 0.0; //des_velo(1) if went non-zero terminal velocities

  double wth_0 = curr_pos(2);
  double wdotth_0 = curr_pos(2);
  double wth_f = des_pos(2);
  double wdotth_f = 0.0; //des_velo(2) if went non-zero terminal velocities

  double tx = 0;
  double ty = 0;
  double tth = 0;
  
  double qx, qy, qth;

  double t_eps = 0.005; // m
  double r_eps = 0.005; // rad
  double tv_eps = 0.01; // m/s
  double rv_eps = 0.005; // rad/s

  //normalize so we work with desired values strictly positive
  wdotx_0 = Normalize(wdotx_0, wx_f);
  wx_f = Normalize(wx_f, wx_f);
  wdoty_0 = Normalize(wdoty_0, wy_f);
  wy_f = Normalize(wy_f, wy_f);
  wdotth_0 = Normalize(wdotth_0, wth_f);
  wth_f = Normalize(wth_f, wth_f);

  while (fabs(wx_f) > t_eps && fabs(wdotx_0) < tv_eps) {
    //update: q(t), t', w', wdot'
    std::vector<double> update = DetermineBangBangCaseAndUpdate(wdotx_0, wx_f, false);
    qx = update[0];
    tx += update[1];
    wx_f -= update[2];
    wdotx_0 = update[3];
    wdotx_0 = Normalize(wdotx_0, wx_f);
    wx_f = Normalize(wx_f, wx_f);
  }
    
  while (fabs(wy_f) > t_eps && fabs(wdoty_0) < tv_eps) {
    //update: q(t), t', w', wdot'
    std::vector<double> update = DetermineBangBangCaseAndUpdate(wdoty_0, wy_f, false);
    qy = update[0];
    ty += update[1];
    wy_f -= update[2];
    wdoty_0 = update[3];
    wdoty_0 = Normalize(wdotx_0, wy_f);
    wy_f = Normalize(wy_f, wy_f);
  }

  while (fabs(wth_f) > t_eps && fabs(wdotth_0) < tv_eps) {
    //update: q(t), t', w', wdot'
    std::vector<double> update = DetermineBangBangCaseAndUpdate(wdotth_0, wth_f, true);
    qth = update[0];
    tth += update[1];
    wth_f -= update[2];
    wdotth_0 = update[3];
    wdotth_0 = Normalize(wdotth_0, wth_f);
    wth_f = Normalize(wth_f, wth_f);
  }

  //synchronize times

 return controls;

}

std::vector<double> Controller::ComputeCommandVoltage() {
  //need to implement for individual motor control
  std::vector<double> voltages;
  return voltages;
}

