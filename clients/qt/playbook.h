#ifndef PLAYBOOK_H
#define PLAYBOOK_H

#include "robot.h"
using namespace std; 

class Play{
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight; // Success Rate
    int frames_running; // Number for frames the play has been executing

    Play(); // Constructor

    virtual bool Applicable() = 0; // Check if the play is Applicable

    virtual bool Complete() = 0; // Check if the play is over

    virtual bool CompleteCondition()= 0; // Check if the play is over

    virtual bool Success()= 0; // Check if the play was successful or not

    virtual void AssignRoles()= 0;  // Assigns all Robots to the best role given the options

    virtual void Begin(vector<Robot*>* team)= 0; // Begins the play initializing all components

    virtual void Execute()= 0; // Executes the state machines of all Robots in question
  
  private:
    bool _complete;
    vector<Robot*>* _team;
    virtual void UpdateWeight() = 0;
};

class ExamplePlay : public Play {
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight ; // Success Rate
    int frames_running; // Number for frames the play has been executing

    ExamplePlay(); // Constructor

    bool Applicable(); // Check if the play is Applicable

    bool Complete();

    bool CompleteCondition(); // Check if the play is over

    bool Success(); // Check if the play was successful or not

    void AssignRoles();  // Assigns all Robots to the best role given the options

    void Begin(vector<Robot*>* team); // Begins the play initializing all components

    void Execute(); // Executes the state machines of all Robots in question
  
  private:
    vector<Robot*>* _team;
    bool _complete;
    virtual void UpdateWeight();
};

class PlayBook{
  public:
    PlayBook();

    Play* PlaySelection(); // Selects the best legal play

    void ResetWeights(); // Reset the weights of all plays

    void AddPlay(Play* play); // Adds a play to the PlayBook
    
  private:
    vector<Play*> _plays;
};

#endif // PLAYBOOK_H