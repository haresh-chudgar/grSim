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

#include "pathplanner.h"
#include "soccerfieldinfo.h"
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//TODO: Use KD-Tree to store points
//      Cache waypoints in the manner of J. Bruce and M. Veloso in ERRT (2002)
//      Improve smoothing
PathPlanner::PathPlanner(size_t robot_ID, bool isYellowTeam) { // isYellowTeam = isYellow
  robot_ID_ = robot_ID;
  isYellowTeam_ = isYellowTeam;
}

PathPlanner::~PathPlanner() {}

double PathPlanner::distToLineSeg(Eigen::Vector2d v, Eigen::Vector2d w, Eigen::Vector2d p) {
  double t = (p-v).dot(w-v)/(w-v).dot(w-v);
  if (t < 0.0) {
    return sqrt((p-v).dot(p-v));  // Beyond the 'v' end of the segment
  }
  else if (t > 1.0) {
    return sqrt((p-w).dot(p-w));  // Beyond the 'w' end of the segment
  }
  Eigen::Vector2d proj = v + t*(w-v);  // Projection falls on the segment
  return sqrt((p-proj).dot(p-proj));
}

bool PathPlanner::checkCollisions(Eigen::Vector3d v, Eigen::Vector3d w, 
                                       vector<Eigen::Vector2d> dyn_obj_loc) {
  double dst;
  Eigen::Vector2d v_(v(0),v(1));
  Eigen::Vector2d w_(w(0),w(1));
  for (size_t i=0; i<dyn_obj_loc.size(); i++) {
    dst = distToLineSeg(v_, w_, dyn_obj_loc[i]);
    if (dst <= ROBOT_RADIUS*2) {
      return true;
    }
  }
  return false;
}

pair<RRTNode, RRTNode> PathPlanner::NearestNodes(vector<RRTNode> FRRT, vector<RRTNode> BRRT) {
  double min_dst = 2*x_lim_ + 2*y_lim_; //greater than farthest distance possible
  std::pair<RRTNode, RRTNode> nearest_nodes = make_pair(FRRT[0],BRRT[0]);
  
  Eigen::Vector3d f_cand;
  Eigen::Vector3d b_cand;
  
  for (size_t i=0; i<FRRT.size(); i++) {
    f_cand = FRRT[i].getPose();
    for (size_t j=0; j<BRRT.size(); j++) {
      b_cand = BRRT[i].getPose();
      double dst = sqrt(pow((f_cand(0)-b_cand(0)),2) + pow((f_cand(1)-b_cand(1)),2));
      if (dst < min_dst) {
        min_dst = dst;
        nearest_nodes.first = FRRT[i];
        nearest_nodes.second = BRRT[j];
      }
    }
  }
  return nearest_nodes;
}

vector<Eigen::Vector3d> PathPlanner::SmoothWaypoints(vector<Eigen::Vector3d> waypoints, 
						     vector<Eigen::Vector2d> dyn_obj_loc) {
  vector<Eigen::Vector3d> smoothed_waypoints;
  int end_index = 1;
  while (end_index != waypoints.size()-1) {
    for (size_t i=waypoints.size()-1; i>end_index; i--) {
      if (!checkCollisions(waypoints[end_index], waypoints[i], dyn_obj_loc)) {
        smoothed_waypoints.push_back(waypoints[i]);
        end_index = i;
      }
    }
  }
  //fprintf(stderr, "PathPlanner::SmoothWaypoints %d\n", smoothed_waypoints.size());
  return smoothed_waypoints;
}

vector<Eigen::Vector3d> PathPlanner::FindWaypoints(vector<RRTNode> FRRT, vector<RRTNode> BRRT,
					 RRTNode F_terminal, RRTNode B_terminal) {
  vector<Eigen::Vector3d> waypoints;
  vector<Eigen::Vector3d> temp_waypoints;

  temp_waypoints.push_back(F_terminal.getPose());
  int parent = F_terminal.getParent();
  while (parent >= 0) {
    RRTNode prev = FRRT[parent];
    temp_waypoints.push_back(prev.getPose());
    parent = prev.getParent();
  }

  for (int i=temp_waypoints.size()-1; i>=0; i--) {
    waypoints.push_back(temp_waypoints[i]);
  }

  waypoints.push_back(B_terminal.getPose());
  parent = B_terminal.getParent();
  while (parent >= 0) {
    RRTNode prev = BRRT[parent];
    waypoints.push_back(prev.getPose());
    parent = prev.getParent();
  }
  
  return waypoints;
}

double PathPlanner::fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

std::vector<Eigen::Vector3d> PathPlanner::FindPath(Eigen::Vector3d start_pos, Eigen::Vector3d goal_pos) {
  vector<Eigen::Vector3d> path;
  vector<Eigen::Vector3d> temp_path;
  //srand(time(NULL));
 
  vector<BotState>* blue_isYellowTeam = SoccerFieldInfo::Instance()->blueTeamBots;
  vector<BotState>* yellow_isYellowTeam = SoccerFieldInfo::Instance()->yellowTeamBots;

  vector<Eigen::Vector2d> dyn_obj_loc;
  Eigen::Vector2d new_loc; //assume all objects have the same radii
  for (size_t i=0; i<blue_isYellowTeam->size(); i++) {
    if (isYellowTeam_) {
      new_loc << blue_isYellowTeam[0][i]._position(0), blue_isYellowTeam[0][i]._position(1);
      dyn_obj_loc.push_back(new_loc);
    }
    else if (robot_ID_ != i) {
      new_loc << blue_isYellowTeam[0][i]._position(0), blue_isYellowTeam[0][i]._position(1);
      dyn_obj_loc.push_back(new_loc);
    }
  }
  for (size_t i=0; i<yellow_isYellowTeam->size(); i++) {
    if (!isYellowTeam_) {
      new_loc << yellow_isYellowTeam[0][i]._position(0), yellow_isYellowTeam[0][i]._position(1);
      dyn_obj_loc.push_back(new_loc);
    }
    else if (robot_ID_ != i) {
      new_loc << yellow_isYellowTeam[0][i]._position(0), yellow_isYellowTeam[0][i]._position(1);
      dyn_obj_loc.push_back(new_loc);
    }
  }

  start_pos_ = start_pos;
  goal_pos_ = goal_pos;

  //check that goal state is not also a collision state
  if (fabs(goal_pos_(0)) > x_lim_ || fabs(goal_pos_(1)) > y_lim_) {
    //fprintf(stderr, "%f  %f %f %f\n", goal_pos_(0), x_lim_, goal_pos_(1), y_lim_);
    std::cout << "Goal out of bounds, returning empty path." << std::endl;
    return path;
  }
  for (size_t i=0; i<dyn_obj_loc.size(); i++) {
    double dst = sqrt(pow((goal_pos_(0)-dyn_obj_loc[i](0)),2) + pow((goal_pos_(1)-dyn_obj_loc[i](1)),2));
    if (dst <= ROBOT_RADIUS*2) {
      std::cout << "Goal located within obstacle... proceeding anyway." << std::endl;
    }
  }

  //try direct solution first
  if (!checkCollisions(start_pos_, goal_pos_, dyn_obj_loc)) {
    path.push_back(goal_pos_);
    //fprintf(stderr, "found a path \n");
    return path;
  }

  int num_nodes = 2;
  double alpha, beta, theta;
  vector<RRTNode> FRRT;
  vector<RRTNode> BRRT;
  RRTNode Froot = RRTNode(-1, start_pos_);
  RRTNode Broot = RRTNode(-1, goal_pos_);
  FRRT.push_back(Froot);
  BRRT.push_back(Broot);

  Eigen::Vector3d expand;
  Eigen::Vector3d target;
  Eigen::Vector3d f_cand;
  Eigen::Vector3d b_cand;

  vector<Eigen::Vector3d> waypoints;

  while (num_nodes < node_lim_) { 
    //check for completion
    pair<RRTNode, RRTNode> nearest = NearestNodes(FRRT, BRRT);
    f_cand = nearest.first.getPose();
    b_cand = nearest.second.getPose();
    double dst = sqrt(pow((f_cand(0)-b_cand(0)),2) + pow((f_cand(1)-b_cand(1)),2));
    if (dst < destination_epsilon_) {
      //we did it!
      temp_path = FindWaypoints(FRRT, BRRT, nearest.first, nearest.second);
      //path = SmoothWaypoints(temp_path, dyn_obj_loc);
      //fprintf(stderr, "found a path \n");
      return temp_path;
      //return path;
    }
    //fprintf(stderr, "path complete check\n");

    //Try to expand forward tree
    alpha = fRand(0.0, 1.0);
    //fprintf(stderr, "alpha: %f \n", alpha);
    if (alpha < goal_bias_) {
      //find nearest node pair in trees and try to link
      pair<RRTNode, RRTNode> nearest = NearestNodes(FRRT, BRRT);
      expand = nearest.first.getPose(); //forward tree node
      target = nearest.second.getPose(); //backward tree node
      //fprintf(stderr, "target: %f %f %f\n", target[0], target[1], target[2]);
      //fprintf(stderr, "expand: %f %f %f\n", expand[0], expand[1], expand[2]);
      if (!checkCollisions(expand, target, dyn_obj_loc)) {
        //we did it!
        temp_path = FindWaypoints(FRRT, BRRT, nearest.first, nearest.second);
        //path = SmoothWaypoints(temp_path, dyn_obj_loc);
	//fprintf(stderr, "found a path \n");
        return temp_path;
	//return path;
      }
    }
    else {
      //choose expansion node uniformly at random from all nodes
      int node_index = rand()%FRRT.size();
      expand = FRRT[node_index].getPose();
      beta = fRand(20, 200); //mm
      theta = fRand(-3.14159, 3.14159);
      Eigen::Vector3d extension(cos(theta),sin(theta),0);
      extension *= beta;
      target = expand + extension;
      if (!checkCollisions(expand, target, dyn_obj_loc)) {
        //add to tree
        RRTNode new_node = RRTNode(node_index, target);
        FRRT.push_back(new_node);
	////fprintf(stderr, "new_node: %f %f %f\n", target[0], target[1], target[2]);
        num_nodes++;
      }
    }
    //fprintf(stderr, "expanded forwards tree\n");

    //Try to expand backward tree
    alpha = fRand(0.0, 1.0);
    if (alpha < goal_bias_) {
      //find nearest node pair in trees and try to link
      pair<RRTNode, RRTNode> nearest = NearestNodes(FRRT, BRRT);
      expand = nearest.first.getPose(); //forward tree node
      target = nearest.second.getPose(); //backward tree node
      if (!checkCollisions(expand, target, dyn_obj_loc)) {
        //we did it!
        temp_path = FindWaypoints(FRRT, BRRT, nearest.first, nearest.second);
        //path = SmoothWaypoints(temp_path, dyn_obj_loc);
	//fprintf(stderr, "found a path \n");
	return temp_path;
        //return path;
      }
    }
    else {
      //choose expansion node uniformly at random from all nodes
      int node_index = rand()%BRRT.size();
      expand = BRRT[node_index].getPose();
      beta = fRand(20, 200);
      theta = fRand(-3.14159, 3.14159);
      Eigen::Vector3d extension(cos(theta),sin(theta),0);
      extension *= beta;
      target = expand + extension;
      if (!checkCollisions(expand, target, dyn_obj_loc)) {
        //add to tree
        RRTNode new_node = RRTNode(node_index, target);
        BRRT.push_back(new_node);
	////fprintf(stderr, "new_node: %f %f %f\n", target[0], target[1], target[2]);
        num_nodes++;
      }
    }   
    //fprintf(stderr, "expanded backwards tree\n");

    //fprintf(stderr, "Num nodes: %d\n", num_nodes);
  }
  if (num_nodes > node_lim_) {
    std::cout << "maximum number of nodes expanded, returning empty list" << std::endl;
  }
  
  return path;
}
