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
#include "evaluation.h"

static SoccerFieldInfo* _instance = NULL;

SoccerFieldInfo* SoccerFieldInfo::Instance() {
  return _instance;
}

void SoccerFieldInfo::CreateInstance(SoccerTeam* blueTeam, SoccerTeam* yellowTeam) {
  _instance = new SoccerFieldInfo(blueTeam, yellowTeam);
}
  
SoccerFieldInfo::SoccerFieldInfo(SoccerTeam* blueTeam, SoccerTeam* yellowTeam)
:_blueTeam(blueTeam), _yellowTeam(yellowTeam), kFrameRate(1.0/60.0), _robotWithBall(false), _teamInBallPossession(false), frameNumber(0)
{
  blueTeamBots = new vector<BotState>();
  yellowTeamBots = new vector<BotState>();
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
    for(;ballState!=frame.balls().end();++ballState) {

      double prevX = ball[0];
      double prevY = ball[1];
      double prevZ = ball[2];

      ball[0] = (*ballState).x();
      ball[1] = (*ballState).y();
      ball[2] = (*ballState).z();
      if(frameNumber == 0) {
	ballVelocity[0] = 0;
	ballVelocity[1] = 0;
	ballVelocity[2] = 0;
	++frameNumber;
      } else {
	ballVelocity[0] = ((ball[0] - prevX) / kFrameRate);
	ballVelocity[1] = ((ball[1] - prevY) / kFrameRate);
	ballVelocity[2] = ((ball[2] - prevZ) / kFrameRate);
      }
      
      fprintf(stderr, "Ball Speed(mm/s): %f, %f %f\n", ballVelocity[0], ballVelocity[1], ballVelocity[2]);
      //fprintf(stderr,"Ball Loc: %f, %f, %f\n", (*ballState).x(), (*ballState).y(), (*ballState).z());
    }

    google::protobuf::RepeatedPtrField<SSL_DetectionRobot>::const_iterator iter = frame.robots_blue().begin();
    while(blueTeamBots->size() < frame.robots_blue_size()) {
      blueTeamBots->push_back(BotState(false));
    }
    while(blueTeamBots->size() > frame.robots_blue_size()) {
      blueTeamBots->pop_back();
    }

    for(;iter!=frame.robots_blue().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      double prevX = blueTeamBots->at(robot.robot_id())._position[0];
      double prevY = blueTeamBots->at(robot.robot_id())._position[1];
      double prevTheta = blueTeamBots->at(robot.robot_id())._position[2];

      blueTeamBots->at(robot.robot_id())._position[0] = robot.x();
      blueTeamBots->at(robot.robot_id())._position[1] = robot.y();
      blueTeamBots->at(robot.robot_id())._position[2] = robot.orientation();
      blueTeamBots->at(robot.robot_id())._id = robot.robot_id();

      
      blueTeamBots->at(robot.robot_id())._velocity[0] = (robot.x()-prevX) / kFrameRate / 1000;
      blueTeamBots->at(robot.robot_id())._velocity[1] = (robot.y()-prevY) / kFrameRate / 1000;
      blueTeamBots->at(robot.robot_id())._velocity[2] = (robot.orientation()-prevTheta) / kFrameRate;
    }
    
    iter = frame.robots_yellow().begin();
    while(yellowTeamBots->size() < frame.robots_blue_size()) {
      yellowTeamBots->push_back(BotState(true));
    }
    while(yellowTeamBots->size() > frame.robots_blue_size()) {
      yellowTeamBots->pop_back();
    }
    for(;iter!=frame.robots_yellow().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      double prevX = yellowTeamBots->at(robot.robot_id())._position[0];
      double prevY = yellowTeamBots->at(robot.robot_id())._position[1];
      double prevTheta = yellowTeamBots->at(robot.robot_id())._position[2];

      yellowTeamBots->at(robot.robot_id())._position[0] = robot.x();
      yellowTeamBots->at(robot.robot_id())._position[1] = robot.y();
      yellowTeamBots->at(robot.robot_id())._position[2] = robot.orientation();
      yellowTeamBots->at(robot.robot_id())._id = robot.robot_id();

      yellowTeamBots->at(robot.robot_id())._velocity[0] = (robot.x()-prevX) / kFrameRate / 1000;
      yellowTeamBots->at(robot.robot_id())._velocity[1] = (robot.y()-prevY) / kFrameRate / 1000;
      yellowTeamBots->at(robot.robot_id())._velocity[2] = (robot.orientation()-prevTheta) / kFrameRate;
    }
    _blueTeam->SimCallback(frame.frame_number(), ball, blueTeamBots, yellowTeamBots);
    _yellowTeam->SimCallback(frame.frame_number(), ball, blueTeamBots, yellowTeamBots);
    
    BotState robotHavingBall(false);
    bool isBallInPossession = Evaluation::TeamHavingBall(&robotHavingBall);
    if(isBallInPossession == false) {
      _teamInBallPossession = false;
      //Dont update robotWithBall to keep the information about which robot kicked it
      //fprintf(stderr, "Ball out of possession\n");
    }
    else {
      _teamInBallPossession = true;
      _robotWithBall = robotHavingBall;
      //fprintf(stderr, "Ball possession %d %d\n", _robotWithBall._id, _robotWithBall._isYellow);
    }
  }
}

