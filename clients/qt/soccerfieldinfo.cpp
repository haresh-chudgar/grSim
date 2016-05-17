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

//TODO replace botstate with robot*
static SoccerFieldInfo* _instance = NULL;

SoccerFieldInfo* SoccerFieldInfo::Instance() {
  return _instance;
}

void SoccerFieldInfo::CreateInstance(SoccerTeam* blueTeam, SoccerTeam* yellowTeam, Communicator* communicator) {
  _instance = new SoccerFieldInfo(blueTeam, yellowTeam, communicator);
}
  
SoccerFieldInfo::SoccerFieldInfo(SoccerTeam* blueTeam, SoccerTeam* yellowTeam, Communicator* communicator)
:_blueTeam(blueTeam), _yellowTeam(yellowTeam), communicator(communicator), kFrameRate(1.0/60.0), _isYellowTeamInBallPossession(false), frameNumber(0)
{
  blue_bots = new vector<Robot*>();
  yellow_bots = new vector<Robot*>();
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
      
      //fprintf(stderr, "Ball Speed(mm/s): %f, %f %f\n", ballVelocity[0], ballVelocity[1], ballVelocity[2]);
      //fprintf(stderr,"Ball Loc: %f, %f, %f\n", (*ballState).x(), (*ballState).y(), (*ballState).z());
    }

    google::protobuf::RepeatedPtrField<SSL_DetectionRobot>::const_iterator iter = frame.robots_blue().begin();
    
    if(frameNumber == 0) {
      for(size_t i = 0; i < frame.robots_blue_size(); i++) {
        blue_bots->push_back(new Robot(false, i));
      }
      for(size_t i = 0; i < frame.robots_yellow_size(); i++) {
        yellow_bots->push_back(new Robot(true, i));
      }
    }

    for(;iter!=frame.robots_blue().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      double prevX = blue_bots->at(robot.robot_id())->_currentPosition[0];
      double prevY = blue_bots->at(robot.robot_id())->_currentPosition[1];
      double prevTheta = blue_bots->at(robot.robot_id())->_currentPosition[2];

      blue_bots->at(robot.robot_id())->_currentPosition[0] = robot.x();
      blue_bots->at(robot.robot_id())->_currentPosition[1] = robot.y();
      blue_bots->at(robot.robot_id())->_currentPosition[2] = robot.orientation();

      blue_bots->at(robot.robot_id())->_currentVelocity[0] = (robot.x()-prevX) / kFrameRate / 1000;
      blue_bots->at(robot.robot_id())->_currentVelocity[1] = (robot.y()-prevY) / kFrameRate / 1000;
      blue_bots->at(robot.robot_id())->_currentVelocity[2] = (robot.orientation()-prevTheta) / kFrameRate;
    }
    
    for(;iter!=frame.robots_yellow().end();iter++) {
      SSL_DetectionRobot robot = *iter;
      double prevX = yellow_bots->at(robot.robot_id())->_currentPosition[0];
      double prevY = yellow_bots->at(robot.robot_id())->_currentPosition[1];
      double prevTheta = yellow_bots->at(robot.robot_id())->_currentPosition[2];

      yellow_bots->at(robot.robot_id())->_currentPosition[0] = robot.x();
      yellow_bots->at(robot.robot_id())->_currentPosition[1] = robot.y();
      yellow_bots->at(robot.robot_id())->_currentPosition[2] = robot.orientation();

      yellow_bots->at(robot.robot_id())->_currentVelocity[0] = (robot.x()-prevX) / kFrameRate / 1000;
      yellow_bots->at(robot.robot_id())->_currentVelocity[1] = (robot.y()-prevY) / kFrameRate / 1000;
      yellow_bots->at(robot.robot_id())->_currentVelocity[2] = (robot.orientation()-prevTheta) / kFrameRate;
    }
    _blueTeam->SimCallback(frame.frame_number(), ball, blue_bots, yellow_bots);
    _yellowTeam->SimCallback(frame.frame_number(), ball, blue_bots, yellow_bots);
    
    //TODO ball possession needs to be reworked (possibly moved)
//     robot robotHavingBall(false, 90);
//     bool isBallInPossession = Evaluation::TeamHavingBall(&robotHavingBall);
//     if(isBallInPossession == false) {
//       _isYellowTeamInBallPossession = false;
//       //Dont update robotWithBall to keep the information about which robot kicked it
//       //fprintf(stderr, "Ball out of possession\n");
//     }
//     else {
//       _isYellowTeamInBallPossession = true;
//       _robotWithBall = robotHavingBall;
//       //fprintf(stderr, "Ball possession %d %d\n", _robotWithBall._id, _robotWithBall._isYellow);
//     }
  }
}

