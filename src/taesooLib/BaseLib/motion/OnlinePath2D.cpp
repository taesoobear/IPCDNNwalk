#include "stdafx.h"
#include "OnlinePath2D.h"
#include "../BaseLib/math/intervals.h"
OnlinePath2D::OnlinePath2D(int n, int maxCapacity)
:mPath(n, maxCapacity)
{
}

OnlinePath2D::OnlinePath2D(matrixn const & matpos, vectorn const& vecori)
:mPath(matpos.rows(), matpos.rows()*2)
{
	for(int i=0; i<matpos.rows(); i++)
	{
		pos(i).x=matpos[i][0];
		pos(i).y=0.0;
		pos(i).z=matpos[i][1];		
	}

	for(int i=0; i<matpos.rows(); i++)
	{
		ori(i).setRotation(vector3(0,1,0), vecori[i]);
	}

	for(int i=1; i<matpos.rows(); i++)
	{
		updateDV(i);
		updateDQ(i);
	}

	dv(0)=dv(1);
	dq(0)=dq(1);

}
OnlinePath2D::OnlinePath2D(matrixn const & matpos, matrixn const& vel)
:mPath(matpos.rows(), matpos.rows()*2)
{
	for(int i=0; i<matpos.rows(); i++)
	{
		pos(i).x=matpos[i][0];
		pos(i).y=0.0;
		pos(i).z=matpos[i][1];		
	}

	ASSERT(matpos.rows()==vel.rows());

	bitvectorn invalid;
	invalid.resize(matpos.rows());
	invalid.clearAll();
	for(int i=0; i<matpos.rows(); i++)
	{
		vector3 v;
		v.x=vel[i][0];
		v.y=0;
		v.z=vel[i][1];
		vector3 nv;
		nv.normalize(v);
		if(nv.length()<0.5 || v.length()<0.0001)
			invalid.setAt(i);
		else
			ori(i).setAxisRotation(vector3(0,1,0), vector3(0,0,1), nv);
	}

	intIntervals invalidInt;
	invalidInt.runLengthEncode(invalid);

	for(int i=0; i<invalidInt.size(); i++)
	{
		int start=invalidInt.start(i);
		int end=invalidInt.end(i);

		if(start==0)
		{
			for(int i=0; i<end; i++)
				ori(i)=ori(end);
		}
		else if(end==matpos.rows())
		{
			for(int i=start; i<end; i++)
				ori(i)=ori(start-1);
		}
		else
		{
			interval iii(start-1, end);
			for(int i=start; i<end; i++)
			{
				m_real t=iii.uninterpolate(m_real(i));
				ori(i).safeSlerp( ori(start-1), ori(end),t);
			}
		}
	}

	for(int i=1; i<matpos.rows(); i++)
	{
		updateDV(i);
		updateDQ(i);
	}

	dv(0)=dv(1);
	dq(0)=dq(1);
}

OnlinePath2D::~OnlinePath2D(void)
{

}

void OnlinePath2D::updateDV(int i)
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

void OnlinePath2D::updateOri(int i)
{
	ori(i).mult(dq(i), ori(i-1));
}

void OnlinePath2D::updatePos(int i)
{
	vector3 dvg;
	dvg.rotate(ori(i-1), dv(i));
	pos(i).add(pos(i-1), dvg);
}

void OnlinePath2D::updatePath(int i)
{
	updateOri(i);
	updatePos(i);
}

// update dq(i) based on ori(i-1) and ori(i)
void OnlinePath2D::updateDQ(int i)
{
	quater &dq_i=dq(i);
	quater inv_q;
	inv_q.inverse(ori(i-1));
	dq_i.mult(ori(i), inv_q);
	dq_i.align(quater(1,0,0,0));	// m_dq should be small.
}
