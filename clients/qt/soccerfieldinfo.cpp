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

#include "soccerfieldinfo.h"
#include <QtNetwork>
#include <QObject>
#include <qiodevice.h>

static SoccerFieldInfo* _instance = NULL;

SoccerFieldInfo* SoccerFieldInfo::Instance() {
  return _instance;
}

void SoccerFieldInfo::CreateInstance(SoccerTeam* blueTeam, SoccerTeam* yellowTeam) {
  _instance = new SoccerFieldInfo(blueTeam, yellowTeam);
}
  
SoccerFieldInfo::SoccerFieldInfo(SoccerTeam* blueTeam, SoccerTeam* yellowTeam)
:_blueTeam(blueTeam), _yellowTeam(yellowTeam)
{
  blueTeamBots = new std::vector<Eigen::Vector3d>(6);
  yellowTeamBots = new std::vector<Eigen::Vector3d>(6);
}

SoccerFieldInfo::~SoccerFieldInfo()
{}


void SoccerFieldInfo::receive(char* buffer, int size)
{
  QHostAddress sender;
  quint16 port;
  SSL_WrapperPacket packet;
  
  if(size == 0)
    return;

  packet.ParseFromArray(buffer, size);
  if(packet.has_detection()) {
    SSL_DetectionFrame frame = packet.detection();
    google::protobuf::RepeatedPtrField<SSL_DetectionBall>::const_iterator ballState = frame.balls().begin();
    ball[0] = (*ballState).x();
    ball[1] = (*ballState).y();
    ball[2] = (*ballState).z();
    
    google::protobuf::RepeatedPtrField<SSL_DetectionRobot>::const_iterator iter = frame.robots_blue().begin();
    for(;iter!=frame.robots_blue().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      blueTeamBots->at(robot.robot_id())[0] = robot.x();
      blueTeamBots->at(robot.robot_id())[1] = robot.y();
      blueTeamBots->at(robot.robot_id())[2] = robot.orientation();
    }
    
    iter = frame.robots_yellow().begin();
    for(;iter!=frame.robots_yellow().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      yellowTeamBots->at(robot.robot_id())[0] = robot.x();
      yellowTeamBots->at(robot.robot_id())[1] = robot.y();
      yellowTeamBots->at(robot.robot_id())[2] = robot.orientation();
    }
    _blueTeam->SimCallback(frame.frame_number(), ball, blueTeamBots, yellowTeamBots);
    _yellowTeam->SimCallback(frame.frame_number(), ball, blueTeamBots, yellowTeamBots);
    fprintf(stderr, "Received info for frame %d\n", frame.frame_number());
  }
}
