#ifndef STDAFX_OGREFLTK_H
#define STDAFX_OGREFLTK_H
#pragma once

#pragma warning(disable:4819) // utf-8 without BOM is fine, isn't it M$?

#include "../BaseLib/baselib.h"
#include "../MainLib/MainLib.h"

#include "../MainLib/OgreFltk/FltkAddon.h"
#include "../MainLib/OgreFltk/FlLayout.h"

typedef int STATUS;
#define SUCCESS 1
#define FAILURE 0


#ifdef USE_MOTION_BLENDING
#define MotionInterpolator MotionBlending
#else
#define MotionInterpolator MotionInterpolation
#endif

void dprintf(bool bExpression,const char* pszFormat, ...);

#endif
