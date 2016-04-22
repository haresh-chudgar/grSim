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

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <QtNetwork>

class Communicator
{
public:
  Communicator(QUdpSocket* udpsocket);
  ~Communicator();
  
  void reconnectUdp(QHostAddress addr, quint16 port);
  void sendPacket(QByteArray dgram);
  void disconnectUdp();
  
private:
  QUdpSocket* _udpsocket;
  QHostAddress _addr;
  quint16 _port;
};

#endif // COMMUNICATOR_H
