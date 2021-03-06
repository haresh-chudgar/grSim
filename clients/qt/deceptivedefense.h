#ifndef DECEPTIVEDEFENSE_H
#define DECEPTIVEDEFENSE_H

#include "playbook.h"

class DeceptiveDefense : public Play
{
public:
//  DeceptiveDefense();
  DeceptiveDefense(bool isTeamYellow);
  ~DeceptiveDefense();

  bool Applicable(); // Check if the play is Applicable

  bool Complete();

  bool CompleteCondition(); // Check if the play is over

  bool Success(); // Check if the play was successful or not

  void AssignRoles();  // Assigns all Robots to the best role given the options

  void Begin(vector<Robot*>* isYellowTeam); // Begins the play initializing all components

  void Execute(); // Executes the state machines of all Robots in question
  
  bool _isYellowTeam;
private:
  vector<Robot*>* _robots;
  bool _complete;
  Eigen::Vector3d _interceptLocation;
  virtual void UpdateWeight();
};

#endif // DECEPTIVEDEFENSE_H
