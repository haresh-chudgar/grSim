#include "deceptivedefense.h"
#include "evaluation.h"

DeceptiveDefense::DeceptiveDefense():_complete(false) {

}

DeceptiveDefense::DeceptiveDefense(bool isTeamYellow):_isYellowTeam(isTeamYellow),_complete(false) {}

DeceptiveDefense::~DeceptiveDefense() {

}

// Check if the play is Applicable
bool DeceptiveDefense::Applicable() {
  fprintf(stderr, "Checking applicability of defense %d, %d\n", SoccerFieldInfo::Instance()->_teamInBallPossession, SoccerFieldInfo::Instance()->_robotWithBall._isYellow);
  if(SoccerFieldInfo::Instance()->_teamInBallPossession == false && SoccerFieldInfo::Instance()->_robotWithBall._isYellow != _isYellowTeam) {
    //fprintf(stderr, "Applicable!\n");
    return true;
  }
  return false;
}

bool DeceptiveDefense::Complete() {
  return CompleteCondition();
}

// Check if the play is over
bool DeceptiveDefense::CompleteCondition() {
  if(SoccerFieldInfo::Instance()->_teamInBallPossession == true && SoccerFieldInfo::Instance()->_robotWithBall._isYellow == _isYellowTeam) {
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
    assignments[interceptingBots.at(2)._robotId] = 1;
    _interceptLocation = interceptingBots.at(0)._interceptLocation;
  }
}

// Begins the play initializing all components
void DeceptiveDefense::Begin(vector<Robot*>* team) {
  assignments.clear();
  states.clear();
  _team = team;
  
  for(size_t i = 0; i < _team->size(); i++) {
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
	Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = _interceptLocation;
	
	goal[2] = atan2(robot[1]-goal[1], robot[0]-goal[0])+M_PI;
	Eigen::Vector3d offset = (robot-goal);
	offset[2] = 0;
	      offset.normalize();
        offset = offset*70;
        goal += offset;
	
	_team->at(i)->goToLocation(1, goal);
        _team->at(i)->setSpinner(1);
	states[i] = 1;
      } else if(states[i] == 1) { // Checking for location
	if (_team->at(i)->execute() == 1) {
          states[i] = 2;
        }
      } else if(states[i] == 2) { // Kicking
	fprintf(stderr, "Done with moving\n");
        _team->at(i)->setSpinner(0);
        
        _team->at(i)->setKickSpeed(1, 1);
        _team->at(i)->sendVelocityCommands();
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