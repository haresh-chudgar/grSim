#ifndef PLAYBOOK_H
#define PLAYBOOK_H

#include "robot.h"
#include <eigen3/Eigen/Core>

using namespace std; 
class Robot;
class Play{
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight; // Success Rate
    double weightN; // Normalized weight
    int frames_running; // Number for frames the play has been executing

    
    Play(bool isYellowTeam); // Constructor

    virtual bool Applicable() = 0; // Check if the play is Applicable

    virtual bool Complete(); // Check if the play is over

    virtual bool CompleteCondition()= 0; // Check if the play is over

    virtual bool Success()= 0; // Check if the play was successful or not

    virtual void AssignRoles()= 0;  // Assigns all Robots to the best role given the options

    virtual void Begin(vector<Robot*>* robots); // Begins the play initializing all components

    virtual void Execute()= 0; // Executes the state machines of all Robots in question
  
    bool _isYellowTeam;
    
    vector<Robot*>* _robots;
  private:
    bool _complete;
    
};

class ExamplePlay : public Play {
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight ; // Success Rate
    int frames_running; // Number for frames the play has been executing

    ExamplePlay(bool isYellowTeam); // Constructor

    bool Applicable(); // Check if the play is Applicable

    bool CompleteCondition(); // Check if the play is over

    bool Success(); // Check if the play was successful or not

    void AssignRoles();  // Assigns all Robots to the best role given the options

    void Execute(); // Executes the state machines of all Robots in question
  
  private:
    vector<Robot*>* _isYellowTeam;
    bool _complete;
};

class MoveToKick : public Play {
  public:
    vector<int> assignments; // Which Robots are assigned to which of the plays roles
    vector<int> states; // The state of each state machine
    double weight ; // Success Rate
    int frames_running; // Number for frames the play has been executing

    MoveToKick(bool isYellowTeam); // Constructor

    bool Applicable(); // Check if the play is Applicable

    bool Complete();

    bool CompleteCondition(); // Check if the play is over

    bool Success(); // Check if the play was successful or not

    void AssignRoles();  // Assigns all Robots to the best role given the options

    void Begin(vector<Robot*>* isYellowTeam); // Begins the play initializing all components

    void Execute(); // Executes the state machines of all Robots in question
  
  private:
    vector<Robot*>* _isYellowTeam;
    bool _complete;
};

class PlayBook{
  public:
    PlayBook(Play* play);

    Play* PlaySelection(); // Selects the best legal play

    void ResetWeights(); // Reset the weights of all plays

    void AddPlay(Play* play); // Adds a play to the PlayBook
    void UpdateWeight(Play* play, int success);
    static PlayBook TheYellowBook();
    static PlayBook TheBlueBook();
  private:
    vector<Play*> _plays;
    vector<Play*> _legal_plays;
    
};



#endif // PLAYBOOK_H
