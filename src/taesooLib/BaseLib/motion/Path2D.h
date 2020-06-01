#pragma once

class Path2D
{
	vector3N mPos;
	quaterN mOri;	// y component
	vector3N mDV;
	quaterN mDQ;
	int mStart;
public:
	// start > 0 because mStartPos and mStartOri are from start-1.
	Path2D(Motion const& mot, int start, int end);
	virtual ~Path2D(void);

	// copy pos and ori back to the motion assuming that motion's offset_q and height information is correct.
	void setMotion(Motion & mot, int start);
	void setMotionDelta(Motion & mot, int start);
	vector3& pos(int i) const	{ return mPos[i+1-mStart];}
	quater& ori(int i) const	{ return mOri[i+1-mStart];}
	vector3& dv(int i) const	{ return mDV[i-mStart];}
	quater& dq(int i) const		{ return mDQ[i-mStart];}

	int size()	{ return mDV.size();}
	int start()	{ return mStart;}

	// update ori(i) based on dq(i) 
	void updateOri(int i);
	// update pos(i) based on dv(i) 
	void updatePos(int i);
	// update ori(i) and pos(i) based on dq(i) and dv(i)
	void updatePath(int i);
	// update dv(i) based on pos(i) and pos(i-1) and ori(i-1)
	void updateDV(int i);
	// update dq(i) based on ori(i-1) and ori(i)
	void updateDQ(int i);
};

