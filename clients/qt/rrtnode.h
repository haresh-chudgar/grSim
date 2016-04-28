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

#ifndef RRTNODE_H
#define RRTNODE_H

#include <vector>
#include <eigen3/Eigen/Core>

class RRTNode {
  public:
    RRTNode(int parent, Eigen::Vector3d pose);
    ~RRTNode();
    
    int getParent();
    Eigen::Vector3d getPose();
    
  private:
    int parent_;
    Eigen::Vector3d pose_;

};

#endif 
