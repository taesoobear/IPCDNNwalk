// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.
//
#ifndef STDAFX_OGREFLTK_H
#define STDAFX_OGREFLTK_H
#pragma once

#pragma warning(disable:4819) // utf-8 without BOM is fine, isn't it M$?

#include "../BaseLib/baselib.h"
#include "../MainLib/MainLib.h"

#include "../MainLib/OgreFltk/FltkAddon.h"
#include "../MainLib/OgreFltk/FlLayout.h"

#ifdef USE_PYTHON_EMBEDDING
#include "../MainLibPython/MLExport.h"
#endif

#ifdef USE_RAGDOLL
#include <NxPhysics.h>
#endif



typedef int STATUS;
#define SUCCESS 1
#define FAILURE 0


// MotionBlending클래스와 MotionInterpolation클래스는 인터페이스가 유사하다. 둘중 어느것을 사용할지 결정한다.
//#define USE_MOTION_BLENDING

#ifdef USE_MOTION_BLENDING
#define MotionInterpolator MotionBlending
#else
#define MotionInterpolator MotionInterpolation
#endif

void dprintf(bool bExpression,const char* pszFormat, ...);

#endif
