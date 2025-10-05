#pragma once

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t mousedX;
  int32_t mousedY;
  int32_t mouseX;
  int32_t mouseY;
  uint8_t mouseButton;
  int32_t mouseScroll;
  char key;
} GuiInputState;

void GuiInputState_Init(GuiInputState* input_state);


typedef struct Timer {
  float mCurrentTime;
  float mFrameTime;
  float mDeltaTime;
  bool mPaused;
}Timer;

extern Timer* gTimer;
void gTimerInit();

inline double gGetCurrentTime() {
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  return (double)(spec.tv_sec) + spec.tv_nsec * 1.0e-9;
}

extern double gTimeAtStart;

inline double gGetTimeSinceStart() { return gGetCurrentTime() - gTimeAtStart; }

extern FILE* gLogFile;

void LoggingInit();

extern const int cLogBufferSize;

inline void gLog(const char* format, ...) {
  assert(gLogFile != NULL);

  fprintf(stdout, "%11.6f: ", gGetTimeSinceStart());
  fprintf(gLogFile, "%11.6f: ", gGetTimeSinceStart());

  va_list argptr;

  char buffer[cLogBufferSize];
  va_start(argptr, format);
  vsnprintf(buffer, cLogBufferSize, format, argptr);
  va_end(argptr);

  fprintf(stdout, "%s\n", buffer);
  fprintf(gLogFile, "%s\n", buffer);

  fflush(gLogFile);
}

inline float gRandomFloat() { return ((float)(rand()) / (float)(RAND_MAX)); }

#ifdef __cplusplus
}
#endif
