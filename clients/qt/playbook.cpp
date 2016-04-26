#include "playbook.h"
#include "soccerfieldinfo.h"
using namespace std; 
// ==== Plays ====

Play::Play() {}

// Should in general be used by all plays to update based on success
void Play::UpdateWeight() {
  // Write weight updating
}

void Play::Begin(vector<Robot>* team) {
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

bool Play::Complete() {
  if(_complete){
    return true;
  } else if(this->CompleteCondition()){
    _complete = true;
    this->UpdateWeight();
  }
}

bool Play::Applicable() {
  return false;
}

bool Play::Success() {
  return false;
}

void Play::AssignRoles() {
}

void Play::Execute() {
}

bool Play::CompleteCondition() {
  return false;
}

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

void ExamplePlay::Begin(vector<Robot>* team) {
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
}

void PlayBook::ResetWeights() {
  for(size_t i = 0; i < _plays.size(); i++) {
    _plays[i]->weight = 1 / _plays.size();
  }
}

void PlayBook::AddPlay(Play* play) {
  _plays.push_back(play);
}

PlayBook TheYellowBook(){
  PlayBook the_book;
  // Add all created yellow team plays to the_book
  return the_book;
  the_book.AddPlay(new ExamplePlay());
  the_book.ResetWeights();
}

PlayBook TheBlueBook(){
  PlayBook the_book;
  // Add all created blue team plays to the_book
  the_book.ResetWeights();
  return the_book;
}

