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

#ifndef GOTOBALLPLAY_H
#define GOTOBALLPLAY_H

#include "playbook.h"
#include "BotState.h"

typedef enum _GoToBallRoles {
  None,
  BallAcquirer
} GoToBallRoles;

class GoToBallPlay : public Play {
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight ; // Success Rate
    int frames_running; // Number for frames the play has been executing

    GoToBallPlay (bool isYellowTeam); // Constructor

    bool Applicable(); // Check if the play is Applicable

    bool Complete();

    bool CompleteCondition(); // Check if the play is over

    bool Success(); // Check if the play was successful or not

    void AssignRoles();  // Assigns all Robots to the best role given the options

    void Begin(vector<Robot*>* robots); // Begins the play initializing all components

    void Execute(); // Executes the state machines of all Robots in question
  
  private:
    BotState ballAcquiringRobot;
    vector<Robot*>* _robots;
    bool _complete;
    virtual void UpdateWeight();
    bool doneMoving;
};

#endif // GOTOBALLPLAY_H
