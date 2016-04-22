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

SoccerFieldInfo::SoccerFieldInfo(QUdpSocket* fieldInfosocket, SoccerTeam* blueTeam, SoccerTeam* yellowTeam)
:_fieldInfosocket(fieldInfosocket), _blueTeam(team1), _yellowTeam(team2)
{
  in_buffer = new char [65536];
  _blueTeam = new std::vector<Eigen::Vector3d>(6);
  _yellowTeam = new std::vector<Eigen::Vector3d>(6);
}

SoccerFieldInfo::SoccerFieldInfo()
{
  in_buffer = new char [65536];
  _fieldInfosocket = NULL;
  blueTeam = new std::vector<Eigen::Vector3d>(6);
  yellowTeam = new std::vector<Eigen::Vector3d>(6);
}

SoccerFieldInfo::~SoccerFieldInfo()
{
}


/*
void SoccerFieldInfo::StopListening() {
  if(_socket->state() == QUdpSocket::BoundState)
    _socket->leaveMulticastGroup(*_net_address, *_net_interface);
}
*/
 
void SoccerFieldInfo::receive()
{
  QHostAddress sender;
  quint16 port;
  SSL_WrapperPacket packet;
  int size = 0;
  while (_fieldInfosocket->hasPendingDatagrams())
  {
    size = _fieldInfosocket->readDatagram(in_buffer, 65536, &sender, &port);
  }
  if(size == 0)
    return;

  packet.ParseFromArray(in_buffer, size);
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
  }
  
}

