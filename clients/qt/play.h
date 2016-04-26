#ifndef PLAYBOOK_H
#define PLAYBOOK_H

#include "soccerteam.h"
#include "soccerfieldinfo.h"
#include "playbook.h"
using namespace std; 


class PlayBook{
  public:
    // Reference to soccerfield info?
    PlayBook();

    Play PlaySelection(); // Selects the best legal play

    void ResetWeights(); // Reset the weights of all plays

    void AddPlay(); // Adds a play to the PlayBook
    
  private:
    vector<Play> _plays;
};

#endif // PLAYBOOK_H