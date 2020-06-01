#ifndef _GLOBAL_UI_H_
#define _GLOBAL_UI_H_

#pragma once
// a lua script can access all the functionalities of the ogrefltk framework.
// this is for enabling command line mode execution. e.g. batch optimization.
// this class should be written in a general way.

// can access to FlLayout, FlChoiceWin, Loader, MotionPanel, luawrapper, etc...(only inside the lua script, or a lua console.)

#include "../WrapperLua/LUAwrapper.h"
class lunaStack;
class LUAwrapper;
class FlChoiceWins;
class GlobalUI : public LUAwrapper::Worker
{
  TStrings _param;
  LUAwrapper* _L;
 public:
  GlobalUI(FlChoiceWins* wins, int argc, char* argv[]);
  ~GlobalUI();

  void releaseScript();
  virtual int work(TString const& workname, lunaStack& L);
};
#endif
