// framemoveobject.h: interface for the FrameMoveObject class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_FrameMoveObject_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
#define AFX_FrameMoveObject_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class FrameMoveObject
{
public:
	FrameMoveObject();
	virtual ~FrameMoveObject();

	virtual int FrameMove(float fElapsedTime)=0;
};

#endif // !defined(AFX_FrameMoveObject_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
