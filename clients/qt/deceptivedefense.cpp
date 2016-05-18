#include "deceptivedefense.h"
#include "evaluation.h"
#include "soccerfieldinfo.h"

//DeceptiveDefense::DeceptiveDefense():_complete(false) {

//}

DeceptiveDefense::DeceptiveDefense(bool isTeamYellow):Play(isTeamYellow),_complete(false) {}

DeceptiveDefense::~DeceptiveDefense() {

}

// Check if the play is Applicable
bool DeceptiveDefense::Applicable() {
  // NEW VERSION: checks if opponent has the ball, if so, it is applicable. This should be more nuanced in the future.  
  int placeholder = 1;
  if (placeholder) { //how to get whether yellow team has the ball or not?
    return true;
  }
  return false;

  /*
  OLD VERSION: 
  fprintf(stderr, "Defense Applicability: %f, %f, %f\n", SoccerFieldInfo::Instance()->ballVelocity.norm(), SoccerFieldInfo::Instance()->ballVelocity[0], SoccerFieldInfo::Instance()->ballVelocity[1]);
  
  if(SoccerFieldInfo::Instance()->ballVelocity.norm() > 0.5*1000 && SoccerFieldInfo::Instance()->ballVelocity[0] < 0) {
    fprintf(stderr, "Defense Applicability Returning true\n");
    return true;
  }
  return false;
  if(SoccerFieldInfo::Instance()->_isYellowTeamInBallPossession == false && SoccerFieldInfo::Instance()->_robotWithBall._isYellow != _isYellowTeam) {
    //fprintf(stderr, "Applicable!\n");
    return true;
  }
  return false;
  */
}

bool DeceptiveDefense::Complete() {
  return CompleteCondition();
}

// Check if the play is over
bool DeceptiveDefense::CompleteCondition() {
  if(SoccerFieldInfo::Instance()->_isYellowTeamInBallPossession == true) {
    fprintf(stderr, "Complete!\n");
    return true;
  }
  return false;
}

// Check if the play was successful or not
bool DeceptiveDefense::Success() {
  return true;
}

// Assigns all Robots to the best role given the options
void DeceptiveDefense::AssignRoles() {
  std::vector<InterceptInfo> interceptingBots;
  bool isFound = Evaluation::FindInterceptingRobots(_isYellowTeam, &interceptingBots);
  assignments = vector<int>(6);
  if(isFound == true) {
    /*
    std::vector<InterceptInfo>::iterator iter = interceptingBots.begin();
    for(;iter != interceptingBots.end();++iter) {
      fprintf(stderr, "Intercepting bot: %d, %f, %f, %f %f\n", (*iter)._robotId, (*iter)._time, (*iter)._interceptLocation[0], (*iter)._interceptLocation[1], (*iter)._interceptLocation[2]);
      if((*iter)._robotId == 0) {
	intInfo = *iter;
	break;
      }
    }
    */
    
    InterceptInfo intInfo = interceptingBots.at(0);
    intInfo = interceptingBots.at(1);
    //assignments[intInfo._robotId] = 1;
    Eigen::Vector2d ballVel = Eigen::Vector2d(SoccerFieldInfo::Instance()->ballVelocity[0], SoccerFieldInfo::Instance()->ballVelocity[1]);
    ballVel.normalize();
    _interceptLocation = intInfo._interceptLocation;// - Eigen::Vector3d(ballVel[0], ballVel[1], 0)* 5;
    _robots->at(intInfo._robotId)->setSpinner(1);
    _robots->at(intInfo._robotId)->goToLocation(1, _interceptLocation);
    fprintf(stderr, "Chosen Intercepting bot: %d, %f, %f, %f %f\n", intInfo._robotId, intInfo._time, intInfo._interceptLocation[0], intInfo._interceptLocation[1], intInfo._interceptLocation[2]);
    
    assignments[intInfo._robotId] = 1;
    intInfo = interceptingBots.at(0);
    _interceptLocation = intInfo._interceptLocation;
    fprintf(stderr, "Chosen Intercepting bot: %d, %f, %f, %f %f\n", intInfo._robotId, intInfo._time, intInfo._interceptLocation[0], intInfo._interceptLocation[1], intInfo._interceptLocation[2]);
    //_isYellowTeam->at(intInfo._robotId)->setSpinner(1);
    //_isYellowTeam->at(intInfo._robotId)->goToLocation(1, _interceptLocation);
    
  }
  else {
    fprintf(stderr, "Intercepting bot not found \n");
  }
}

// Begins the play initializing all components
void DeceptiveDefense::Begin(vector<Robot*>* isYellowTeam) {
  assignments.clear();
  states.clear();
  _isYellowTeam = isYellowTeam;
  
  for(size_t i = 0; i < _robots->size(); i++) {
    states.push_back(0);
  }
  
  // First bot is always the goalie (more complex plays could pull the goalie to another role)
  assignments.push_back(9);
  this->AssignRoles();
  this->Execute();
}

// Executes the state machines of all Robots in question
void DeceptiveDefense::Execute() {
  // Roles are just numbers, can be enums in each class if necessary
  // used inside state machine
  for(size_t i = 0; i < 6; i++) {
    if(assignments[i] == 1) {
      if(states[i] == 0) { // Moving to intercept
	
        _robots->at(i)->goToLocation(1, _interceptLocation);
        _robots->at(i)->setSpinner(1);
	states[i] = 1;
      } else if(states[i] == 1) { // Checking for location
        if (_robots->at(i)->execute() == 1) {
          states[i] = 2;
        }
      } else if(states[i] == 2) { // Kicking
	fprintf(stderr, "Done with moving\n");
        _robots->at(i)->setSpinner(0);
        
        _robots->at(i)->setKickSpeed(1, 1);
        _robots->at(i)->sendVelocityCommands();
        // Call Kick
        states[i]++;
      }  else {
	  Eigen::Vector3d ballVel = SoccerFieldInfo::Instance()->ballVelocity;
	  //fprintf(stderr, "BallVel %f %f %f\n", ballVel[0], ballVel[1], ballVel[2]);
        // do nothing
      }
    }
  }
  frames_running++;
}

// Should in general be used by all plays to update based on success
void DeceptiveDefense::UpdateWeight() {
  // Write weight updating
}
