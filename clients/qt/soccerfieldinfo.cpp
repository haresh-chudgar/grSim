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

SoccerFieldInfo::SoccerFieldInfo(QUdpSocket* fieldInfosocket)
:_fieldInfosocket(fieldInfosocket)
{
  in_buffer = new char [65536];
  blueTeam = std::vector<Eigen::Vector3d>(6);
  yellowTeam = std::vector<Eigen::Vector3d>(6);
}

SoccerFieldInfo::SoccerFieldInfo()
{
  in_buffer = new char [65536];
  _fieldInfosocket = NULL;
  blueTeam = std::vector<Eigen::Vector3d>(6);
  yellowTeam = std::vector<Eigen::Vector3d>(6);
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
 
bool SoccerFieldInfo::receive()
{
  QHostAddress sender;
  quint16 port;
  SSL_WrapperPacket packet;
  while (_fieldInfosocket->hasPendingDatagrams())
  {
    int size = _fieldInfosocket->readDatagram(in_buffer, 65536, &sender, &port);
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
	Eigen::Vector3d v = blueTeam.at(0);
	blueTeam.at(robot.robot_id())[0] = robot.x();
	blueTeam.at(robot.robot_id())[1] = robot.y();
	blueTeam.at(robot.robot_id())[2] = robot.orientation();
      }
      
      iter = frame.robots_blue().begin();
      for(;iter!=frame.robots_blue().end();iter++) {
	SSL_DetectionRobot robot = *iter;
	blueTeam.at(robot.robot_id())[0] = robot.x();
	blueTeam.at(robot.robot_id())[1] = robot.y();
	blueTeam.at(robot.robot_id())[2] = robot.orientation();
      }
    }
  }
}

