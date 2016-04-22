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

#ifndef SOCCERFIELDINFO_H
#define SOCCERFIELDINFO_H

#include "messages_robocup_ssl_wrapper.pb.h"
#include <QUdpSocket>
#include <QMutex>
#include <eigen3/Eigen/Core>
#include <vector>

using namespace std;

class SoccerFieldInfo
{
public:
    SoccerFieldInfo(QUdpSocket* fieldInfosocket);
    SoccerFieldInfo();
    ~SoccerFieldInfo();
    
    bool receive();
    QUdpSocket* _fieldInfosocket;
private:
  
  char* in_buffer;
  Eigen::Vector3d ball;
  std::vector<Eigen::Vector3d> yellowTeam;
  std::vector<Eigen::Vector3d> blueTeam;
};

#endif // SOCCERFIELDINFO_H
