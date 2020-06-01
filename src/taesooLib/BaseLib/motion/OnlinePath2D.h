#pragma once

#include "../utility/TOnlineArray.h"
#include "MotionUtil.h"
class OnlinePath2D
{
public:
	struct Frame
	{	
		vector3 mPos;
		quater mOri;	// y component
		vector3 mDV;
		quater mDQ;
	};

	TOnlineArray<Frame> mPath;

	// forgot frames before maxCapacity frames for memory efficiency.
	OnlinePath2D(int n=0, int maxCapacity=100);

	// pos, vel: n by 2 matrix
	OnlinePath2D(matrixn const & pos, matrixn const& vel);
	OnlinePath2D(matrixn const & pos, vectorn const& ori);
	virtual ~OnlinePath2D();

	const vector3& pos(int i) const	{ return mPath[i].mPos;}
	const quater& ori(int i) const	{ return mPath[i].mOri;}
	const vector3& dv(int i) const	{ return mPath[i].mDV;}
	const quater& dq(int i) const	{ return mPath[i].mDQ;}
	MotionUtil::Coordinate coord(int i) const { return MotionUtil::Coordinate(ori(i), pos(i));}

	vector3& pos(int i) 			{ return mPath[i].mPos;}
	quater& ori(int i) 				{ return mPath[i].mOri;}
	vector3& dv(int i) 				{ return mPath[i].mDV;}
	quater& dq(int i)				{ return mPath[i].mDQ;}

	int size()	{ return mPath.size();}

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
