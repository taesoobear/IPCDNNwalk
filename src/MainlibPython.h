#pragma once

class MotionPanel;

#include "MainLib/OgreFltk/timesensor.h"
#include "MainLib/OgreFltk/RE.h"
#include "MLExport.h"
ML_EXPORT void MainlibPythonSetGlobal(RE::Globals* g, MotionPanel* w, FrameSensor::Trigger* t, FlLayout* l);
ML_EXPORT int getIntegerReturnValue();
ML_EXPORT void setIntegerReturnValue(int i);

enum handle_message { M_FRAME_MOVE, M_TRIGGERED, M_ON_DRAW, M_CALLBACK, M_HANDLE};	


