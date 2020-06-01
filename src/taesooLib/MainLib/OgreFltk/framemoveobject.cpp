// FrameMoveObject.cpp: implementation of the FrameMoveObject class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "framemoveobject.h"
#include "RE.h"
#include "renderer.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

FrameMoveObject::FrameMoveObject()
{
}

FrameMoveObject::~FrameMoveObject()
{
	if(RE::rendererValid()){
		RE::renderer().removeFrameMoveObject(this);
		RE::renderer().removeAfterFrameMoveObject(this);
	}
}

