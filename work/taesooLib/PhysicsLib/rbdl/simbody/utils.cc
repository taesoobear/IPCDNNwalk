
#include "physicsLib.h"
#include "utils.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>

static Timer sTimer;
Timer* gTimer = &sTimer;
double gTimeAtStart = 0;

FILE* gLogFile = NULL;
const int cLogBufferSize = 1024;

void gTimerInit() {
  sTimer.mCurrentTime = 0.;
  sTimer.mDeltaTime = 0.;
  sTimer.mFrameTime = 0.;
  sTimer.mPaused = false;

  gTimeAtStart = gGetCurrentTime();
}

void GuiInputState_Init(GuiInputState* input_state) {
  input_state->mouseX = 0;
  input_state->mouseY = 0;
  input_state->mouseButton = 0;
  input_state->mouseScroll = 0;
  input_state->key = 0;
}

void LoggingInit() {
  // check if protot.log exists and rename it to protot.log.1 if found
  std::ifstream source("protot.log", std::ios::binary);

  if (source.good()) {
    std::ofstream destination(
        "protot.log.1",
        std::ios::binary | std::ios::trunc);

    destination << source.rdbuf();
  }

  source.close();

  gLogFile = fopen("protot.log", "w+");
}
