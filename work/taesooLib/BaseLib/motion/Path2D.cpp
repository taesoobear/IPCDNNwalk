#include "stdafx.h"
#include "Motion.h"
#include "MotionLoader.h"
#include "Path2D.h"
#include "../BaseLib/math/Operator.h"

namespace sop
{
	// x가 0에서는 기울기가 0이고, x>start일때는 기울기가 1이되는 부드러운 함수. x>=0에서 정의됨.
	m_real responseFunction(m_real x, m_real start)
	{
		// 앞부분에 y=ax^2를 잘라서 사용함.
		// 2a*start=1.0
		m_real a=0.5/start;

		if(x<start)
		{
			return a*SQR(x);
		}		
		return x-start+a*SQR(start);
	}

	// x가 b와 c사이에서는 값이 0가나오고, a보다 작을때나, d보다 클때는 기울기가 1이된다.
	m_real offsetFunction(m_real x, m_real a, m_real b, m_real c, m_real d)
	{
		if(x<b)
			//b-x>start 즉,  b-start>x
			return responseFunction(b-x, b-a)*-1;
		else if(x>c)
			return responseFunction(x-c, d-c);
		else
			return 0;
	}
}

// update dq(i) based on ori(i-1) and ori(i)
void Path2D::updateDQ(int i)
{
	quater &dq_i=dq(i);
	quater inv_q;
	inv_q.inverse(ori(i-1));
	dq_i.mult(ori(i), inv_q);
	dq_i.align(quater(1,0,0,0));	// m_dq should be small.
}

Path2D::Path2D(Motion const& mot, int start, int end)
{
	mStart=start;
	mPos.setSize(end-start+1);
	mOri.setSize(end-start+1);	// y component
	mDV.setSize(end-start);
	mDQ.setSize(end-start);

	mot.pose(start-1).decomposeRot();

	mPos[0]=mot.pose(start-1).m_aTranslations[0];
	mPos[0].y=0;
	mOri[0]=mot.pose(start-1).m_rotAxis_y;

	for(int i=start; i<end; i++)
	{
		vector3& dv_i=mDV[i-start];
		quater& dq_i=mDQ[i-start];
		pos(i)=mot.pose(i).m_aTranslations[0];
		pos(i).y=0;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation m_dv and offset_y
		///////////////////////////////////////////////////////////////////////////////
		dv_i = (mot.pose(i).m_aTranslations[0] - mot.pose(i-1).m_aTranslations[0]);
		dv_i.y = 0;
	
		quater inv_q;
		inv_q.inverse(mot.pose(i-1).m_rotAxis_y);
		dv_i.rotate(inv_q,dv_i);

		///////////////////////////////////////////////////////////////////////////////
		//  calculation rotAxis_y	pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q 
		//							pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
		//			  		  thus, pose(i).m_dq = pose(i).m_rotAxis_y * (pose(i-1).m_rotAxis_y)^(-1)
		///////////////////////////////////////////////////////////////////////////////
		
		mot.pose(i).decomposeRot();
		ori(i)=mot.pose(i).m_rotAxis_y;

		dq_i.mult(mot.pose(i).m_rotAxis_y, inv_q);
		dq_i.align(quater(1,0,0,0));	// m_dq should be small.
	}
}

void Path2D::updateDV(int i)
{
	vector3& dv_i=dv(i);
	///////////////////////////////////////////////////////////////////////////////
	//  calculation dv
	///////////////////////////////////////////////////////////////////////////////
	dv_i.sub(pos(i), pos(i-1));
	dv_i.y = 0;

	quater inv_q;
	inv_q.inverse(ori(i-1));
	dv_i.rotate(inv_q,dv_i);
}

Path2D::~Path2D(void)
{
}

void Path2D::updateOri(int i)
{
	ori(i).mult(dq(i), ori(i-1));
}

void Path2D::updatePos(int i)
{
	vector3 dvg;
	dvg.rotate(ori(i-1), dv(i));
	pos(i).add(pos(i-1), dvg);
}

void Path2D::updatePath(int i)
{
	updateOri(i);
	updatePos(i);
}

void Path2D::setMotion(Motion & mot, int start)
{
	for(int i=0; i<size(); i++)
	{
		mot.pose(start+i).m_aTranslations[0].x=pos(start+i).x;
		mot.pose(start+i).m_aTranslations[0].z=pos(start+i).z;
		mot.pose(start+i).m_aRotations[0].mult(ori(start+i), mot.pose(start+i).m_offset_q);
	}
}

void Path2D::setMotionDelta(Motion & mot, int start)
{
	for(int i=0; i<size(); i++)
	{
		mot.pose(start+i).m_dv=dv(start+i);
		mot.pose(start+i).m_dq=dq(start+i);
	}
}

