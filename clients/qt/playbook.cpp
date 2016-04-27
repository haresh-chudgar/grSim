#include "playbook.h"
#include "gotoballplay.h"
#include "soccerfieldinfo.h"
#include "evaluation.h"

using namespace std; 
// ==== Plays ====

Play::Play() {}

//---Example play---
ExamplePlay::ExamplePlay() { }

bool ExamplePlay::Applicable() {
  // Lets say Example play is only applicable if the team has the ball
  bool has_ball;
  return has_ball;
  // Can be based on team predicates, predicates based on analyzing opponents state
  // or more complicated functions per class
}

bool ExamplePlay::CompleteCondition() {
  // Lets say Example play is complete as soon as we don't have the ball
  bool has_ball;
  return !has_ball;
  // Can be based on team predicates, predicates based on analyzing opponents state
  // or more complicated functions per class
}

// More complex analysis of success/failure might be nice
//(overall negative effect in ground lost for instance)

bool ExamplePlay::Success() {  
  // Lets say Example play is successful if we scored because of this play
   bool scored;
  return scored;
  // Can be based on team predicates, predicates based on analyzing opponents state
  // or more complicated functions per class
}

void ExamplePlay::AssignRoles() { 
  // Roles are just numbers, can be enums in each class if necessary
  // used inside state machine
  for(size_t i = 1; i < _team->size() - 1; i++) {
    bool closestToBall;
    bool forward;
    // If the robot has the ball he's the priority (the kicker)
    if(closestToBall) {
      assignments.push_back(0); 
    } else if(forward) {
      // Other forward robots get roles as potential pass targets
      assignments.push_back(1); 
    } else{
      // Others remain on defense
      assignments.push_back(2);
    }
  }
}

void ExamplePlay::Execute() { 
  // Roles are just numbers, can be enums in each class if necessary
  // used inside state machine
  for(size_t i = 1; i < _team->size() - 1; i++) {
    if(assignments[i] == 0) {
      // Kicker
    } else if (assignments[i] == 1) {
      // Potential pass target
    } else if (assignments[i] == 2) {
      // defense
    } else if(assignments [i] == 9) {
      // GOALIE
    }
  }
  frames_running++;
}

bool ExamplePlay::Complete() {
  if(_complete){
    return true;
  } else if(this->CompleteCondition()){
    _complete = true;
    this->UpdateWeight();
  }
}

void ExamplePlay::Begin(vector<Robot*>* team) {
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

// Should in general be used by all plays to update based on success
void ExamplePlay::UpdateWeight() {
  // Write weight updating
}


//---MoveToKick play (rigged)---
MoveToKick::MoveToKick() { }

bool MoveToKick::Applicable() {
  return true;
}

bool MoveToKick::CompleteCondition() {
  return states[0] == 3;
}

// More complex analysis of success/failure might be nice
//(overall negative effect in ground lost for instance)

bool MoveToKick::Success() {  
  return states[0] == 3;
}

void MoveToKick::AssignRoles() { 
  assignments[0] = 1;
}

void MoveToKick::Execute() { 
  // Roles are just numbers, can be enums in each class if necessary
  // used inside state machine
  for(size_t i = 0; i < 1; i++) {
    if(assignments[i] == 0) {
      if(states[i] == 0) { // Moving to Ball
        Eigen::Vector3d ball = SoccerFieldInfo::Instance()->ball;
        Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = (ball-robot);
        Eigen::Vector3d offset = (ball-robot);
	offset.normalize();
        offset = offset*0.07;
        goal -= offset;
        offset.normalize();
        goal(2) = acos(offset(0));
        // Call Move
        states[i] = 1;
      } else if(states[i] == 1) { // Checking for location
        Eigen::Vector3d ball = SoccerFieldInfo::Instance()->ball;
	Eigen::Vector3d robot = _team->at(i)->CurrentState();
        Eigen::Vector3d goal = (ball-robot);
        Eigen::Vector3d offset = (ball-robot);
	offset.normalize();
        offset = offset*0.07;
        goal -= offset;
        offset.normalize();
        goal(2) = acos(offset(0));
        if((robot - goal).norm() < .01){
          states[i] = 2;
        }
      } else if(states[i] == 2) { // Kicking
        // Call Kick
        states[i]++;
      } else {
        // do nothing
      }
    } 
  frames_running++;
  }
}

bool MoveToKick::Complete() {
  if(_complete){
    return true;
  } else if(this->CompleteCondition()){
    _complete = true;
    this->UpdateWeight();
  }
}

void MoveToKick::Begin(vector<Robot*>* team) {
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

// Should in general be used by all plays to update based on success
void MoveToKick::UpdateWeight() {
  // Write weight updating
}

// ===== PlayBook =====
PlayBook::PlayBook() { }

Play* PlayBook::PlaySelection() {
  vector<Play*> legal_plays;
  // Finds all legal plays
  for(size_t i = 0; i < _plays.size(); i++) {
    if(_plays[i]->Applicable()) {
      legal_plays.push_back(_plays[i]);
    }
  }
  // Selects a play based on the weights of plays
  double r = ((double) rand() / (RAND_MAX)) + 1;
  for(size_t i = 0; i < legal_plays.size(); i++) {
    if(legal_plays[i]->weight > r) {
      return legal_plays[i];
    } else {
      r -= legal_plays[i]->weight;
    }
  }
  if(legal_plays.size() == 0) 
    return NULL;
  else
    return legal_plays.at(0); //Haresh: Hardcoding to first play
}

void PlayBook::ResetWeights() {
  for(size_t i = 0; i < _plays.size(); i++) {
    _plays[i]->weight = 1 / _plays.size();
  }
}

void PlayBook::AddPlay(Play* play) {
  _plays.push_back(play);
}

PlayBook PlayBook::TheYellowBook(){
  PlayBook the_book;

  //the_book.AddPlay(new MoveToKick());

  the_book.AddPlay(new GoToBallPlay());
  the_book.ResetWeights();
  return the_book;
}

PlayBook PlayBook::TheBlueBook(){
  PlayBook the_book;
  // Add all created blue team plays to the_book
  //the_book.AddPlay(new ExamplePlay());
  the_book.ResetWeights();
  return the_book;
}
