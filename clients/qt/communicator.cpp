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

#include "communicator.h"

Communicator::Communicator(QUdpSocket* udpsocket) {
  _udpsocket = udpsocket;
}

Communicator::~Communicator() {
  _udpsocket->close();
}

void Communicator::reconnectUdp(QHostAddress addr, quint16 port) {
  _addr = addr;
  _port = port;
}

void Communicator::sendPacket(QByteArray dgram) {
  _udpsocket->writeDatagram(dgram, _addr, _port);
}

void Communicator::disconnectUdp() {
  _udpsocket->close();
}