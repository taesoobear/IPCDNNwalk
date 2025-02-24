#include "stdafx.h"
#include "../math/mathclass.h"
#include "MotionRetarget.h"
#ifdef USE_NR
#include "../math/nr/nr.h"
#else
#include "../math/optimize.h"
#endif
#include "MotionUtil.h"
#include "Motion.h"
#include "MotionDOF.h"
#include "MotionLoader.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/utility/TOnlineArray.h"
#include "../BaseLib/math/OperatorStitch.h"
#include "Path2D.h"
using namespace MotionUtil;
quater MotionUtil::DeltaRep::I(1,0,0,0);

namespace v1
{
	struct inverseFunction : public _op
	{
		inverseFunction(){}
		virtual void calc(vectorn& c, const vectorn& a) const;
	};

	void inverseFunction ::calc(vectorn& c, const vectorn& a) const
	{
		m_real minTime=a[0];
		m_real maxTime=a[a.size()-1];

		ASSERT(isSimilar(minTime,0.0));

		// c 길이가 정수가 되도록 rounding한다. 1을 더하는 이유는, 정수로 바꾸면 연속된 동작세그먼트간 1프레임 오버랩이 생기기 때문.
		int nDesiredLen=ROUND(maxTime)+1;

		vectorn aa(a);
		aa*=1.0/maxTime*((m_real)nDesiredLen-1.0);
		
		ASSERT(nDesiredLen>=1);

		int curInterval=0;

		c.setSize(nDesiredLen);
		for(int i=0; i<nDesiredLen; i++)
		{
			m_real t=m_real (i);
			int j=curInterval+1;
			for(; j<aa.size(); j++)
			{
				if(aa[j]>=t-FERR)
					break;
			}

			curInterval=j-1;

			m_real frac=((m_real)i-aa[curInterval])/(aa[j]-aa[curInterval]);

			c[i]=(m_real(curInterval)+frac);
		}
	}

}


namespace m0
{
	// curve의 가속도를 최대한 유지하면서, a[time]--> mCon이 되도록 고친다.
	// curve의 시작 위치와, 시작, 끝 속도는 변하지 않는다.
	struct adjustOnline: public _op
	{
		adjustOnline(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, 모든 a[time]--> mCon이 되도록 고친다. 즉 constraint개수제한 없음.
	// curve의 시작 위치와, 시작, 끝 속도는 변하지 않는다.
	struct adjustOnlineMultiCon: public _op
	{
		adjustOnlineMultiCon(intvectorn const& time, matrixn const & con):mTime(time), mCon(con){}

		intvectorn const& mTime;
		matrixn const& mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, a[time]--> mCon이 되도록 고친다.
	// curve의 시작 위치와, 시작 속도는 변하지 않는다.
	struct adjustOnline2: public _op
	{
		adjustOnline2(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, a[time]-->mCon이 되도록 고친다.
	// curve의 시작위치와 끝 위치는 변하지 않는다.
	struct adjust: public _op
	{
		adjust(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 속도를 최대한 유지하면서, 그 합이 특정 값이 되도록 한다.
	struct adjustSumOnline: public _op
	{
		adjustSumOnline(int time, vectorn const & con):mTime(time), mDelta(con){}

		int mTime;
		vectorn mDelta;
		virtual void calc(matrixn& c) const;
	};

	void adjustOnline::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=4;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		
		vectorn weight;
		weight.setSize(nsample-2);
		HessianQuadratic h(nsample);
		intvectorn index;
		vectorn coef;
		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(3, 1.0, -2.0, 1.0);

			weight[i]=SQR(sop::clampMap(i, 0, mTime, 1.0, 2.2));

			coef*=weight[i];
			h.addSquaredH(index, coef);
		}

		H=h.H;

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[2][row-1]=-1.0;
		A[3][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
//#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		m::LUinvert( invAugmented,Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			h.R.setAllValue(0);
			for(int i=0; i<nsample-2; i++)
			{
				index.setValues(3, i, i+1,i+2);
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*c[i]);
				coef*=weight[i];
				h.addSquaredR(index, coef);
			}
			d.range(0, h.R.size())=h.R;

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2)-f(row-1);
			d[row+3]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjustOnlineMultiCon::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=3+mCon.rows();
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<nsample; i++)
		{
			if(0<=i && i<nsample-2)
			{
				H[i][i]+=2.0;
				H[i][i+1]-=4.0;
				H[i][i+2]+=2.0;
			}
			if(1<=i && i<nsample-1)
			{
				H[i][i-1]-=4.0;
				H[i][i]+=8.0;
				H[i][i+1]-=4.0;
			}
			if(2<=i && i<nsample)
			{
				H[i][i-2]+=2.0;
				H[i][i-1]-=4.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[2][row-1]=-1.0;

		for(int i=0; i<mTime.size(); i++)
			A[3+i][mTime[i]]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		m::LUinvert(invAugmented,Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			for(int i=0; i<nsample; i++)
			{
				d[i]=0;
				if(0<=i && i<nsample-2)
					d[i]+=2.0*c[i];
				if(1<=i && i<nsample-1)
					d[i]-=4.0*c[i-1];
				if(2<=i && i<nsample)
					d[i]+=2.0*c[i-2];
			}

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2)-f(row-1);

			for(int i=0; i<mTime.size(); i++)
				d[row+3+i]=mCon[i][dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjust::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=5;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<nsample; i++)
		{
			if(0<=i && i<nsample-2)
			{
				H[i][i]+=2.0;
				H[i][i+1]-=4.0;
				H[i][i+2]+=2.0;
			}
			if(1<=i && i<nsample-1)
			{
				H[i][i-1]-=4.0;
				H[i][i]+=8.0;
				H[i][i+1]-=4.0;
			}
			if(2<=i && i<nsample)
			{
				H[i][i-2]+=2.0;
				H[i][i-1]-=4.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[3][row-1]=1.0;
		A[4][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		m::LUinvert(invAugmented,Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			for(int i=0; i<nsample; i++)
			{
				d[i]=0;
				if(0<=i && i<nsample-2)
					d[i]+=2.0*c[i];
				if(1<=i && i<nsample-1)
					d[i]-=4.0*c[i-1];
				if(2<=i && i<nsample)
					d[i]+=2.0*c[i-2];
			}

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2);
			d[row+3]=f(row-1);
			d[row+4]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjustOnline2::calc(matrixn& curve) const
	{
		// minimize velocity difference
		// objective function to be minimized :
		// f=sum(r'. -r.)^2

		// constraint :
		// r.(start) == r'.(start)		
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=3;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);

		HessianQuadratic h(row);
		intvectorn index;
		vectorn coef;

		// set hessian matrix. ( sum_0^{n-2} {(x[i]-x[i+1]-c[i])^2} 의 hessian(=gradient의 계수).
		for(int i=0; i<nsample-1; i++)
		{
			index.setValues(2, i, i+1);
			coef.setValues(2, -1.0, 1.0);
			h.addSquaredH(index, coef);
		}

		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(3, 1.0, -2.0, 1.0);
			h.addSquaredH(index, coef);
		}

		H=h.H;

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
//#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		m::LUinvert(invAugmented,Augmented);
#endif

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-1; i++)
			{
				index.setValues(2, i, i+1);
				coef.setValues(3, -1.0, 1.0, -1.0*(f(i+1)-f(i)));
				h.addSquaredR(index, coef);
			}

			for(int i=0; i<row-2; i++)
			{
				index.setValues(3, i, i+1, i+2);
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*(f(i)-2.0*f(i+1)+f(i+2)));
				h.addSquaredR(index, coef);
			}

			d.range(0, nsample)=h.R;

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}
	void adjustSumOnline::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'. -r.)^2
		//  f=sum ( (x[i]-x[i-1])-(f[i]-f[i-1]))^2 

		// constraint :
		// r(start) == r'(start)
		// sum_start^time (r(i))== mDelta;

		// LAGRANGIAN MULTIPLIER version
		//    Augmented * w      =    d ,               w=(X, lamda)
		//   (H A^T)     (X)     =   (d)
		//   (A 0  )     (lamda)

		const int numCon=3;
		int xsize=curve.rows();
		matrixn Augmented(xsize+numCon, xsize+numCon);
		matrixnView H=Augmented.range(0,xsize, 0, xsize);
		matrixnView A=Augmented.range(xsize, xsize+numCon, 0, xsize);
		matrixnView At=Augmented.range(0, xsize, xsize, xsize+numCon);

		vectorn x, d(xsize+numCon);
		// set hessian matrix. ( f의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<xsize; i++)
		{
			if(1<=i && i<xsize)
			{
				H[i][i]+=2.0;
				H[i][i-1]-=2.0;
			}
			if(0<=i && i<xsize-1)
			{
				H[i][i+1]-=2.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;	// con1
		A.row(1).range(0, mTime+1).setAllValue(1.0);	// con2
		A[2][xsize-1]=1.0;	// con3
		At.transpose(A);

		Augmented.range(xsize, xsize+numCon, xsize, xsize+numCon).setAllValue(0.0);

#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		m::LUinvert( invAugmented,Augmented);
#endif

		vectorn c(xsize-1);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]


			for(int i=0; i<xsize-1; i++)
				c[i]=f(i+1)-f(i);

			// set constraint
			d[xsize]=f(0);
			d[xsize+1]=0;
			for(int i=0; i<mTime+1; i++)
				d[xsize+1]+=f(i);
			d[xsize+1]+=mDelta[dim];
			d[xsize+2]=f(xsize-1);
			// set hessian residuals.
			for(int i=0; i<xsize; i++)
			{
				d[i]=0;
				if(1<=i && i<xsize)
					d[i]=2.0*c[i-1];
				if(0<=i && i<xsize-1)
					d[i]-=2.0*c[i];
			}

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif

			// save results.
			for(int i=0; i<xsize; i++)
				f(i)=x[i];

		}
	}
}

class DeltaRepMotion:public MotionUtil::DeltaRep
{
	public:
	Motion& mTarget;
	DeltaRepMotion(Motion& mot): mTarget(mot){}

	virtual void decomposeRot(int iframe)
	{
		mTarget.pose(iframe).decomposeRot();
	}
	virtual void composeRot(int i)
	{
		Posture& pose=mTarget.pose(i);
		pose.m_aRotations[0].mult(pose.m_rotAxis_y, pose.m_offset_q);
	}
	virtual quater & rotY(int iframe) { return mTarget.pose(iframe).m_rotAxis_y; }
	virtual quater & dQ(int iframe) { return mTarget.pose(iframe).m_rotAxis_y; }
	virtual vector3 pos(int iframe) const { return mTarget.pose(iframe).m_aTranslations[0]; }
	virtual void setPos(int iframe, double px, double pz)
	{
		mTarget.pose(iframe).m_aTranslations[0].x=px;
		mTarget.pose(iframe).m_aTranslations[0].z=pz;
	}
	virtual int numFrames() const
	{
		return mTarget.numFrames();
	}
	virtual void calcInterFrameDifference(int startFrame) { mTarget.CalcInterFrameDifference(startFrame );}
	virtual void reconstructPos(int startFrame){ mTarget._reconstructPosByDifference(startFrame );}
	virtual void reconstructAll(int startFrame){ mTarget.ReconstructDataByDifference(startFrame );}

};

class DeltaRepMotionDOF: public MotionUtil::DeltaRep
{
	public:
	MotionDOF& mTarget;
	class PoseDelta
	{
		public:
		vector3 m_dv;
		/// 이전 프레임과 root의 planar 오리엔테이션 차이(이전 프레임의 local 좌표계에서 표현)
		/** pose(i).m_rotAxis_y= pose(i).m_dq * pose(i-1).m_rotAxis_y */
		quater m_dq;
		m_real m_offset_y;		//!< m_dv가 포함 하지 않는 y value
		quater m_offset_q;		//!< m_dq가 포함하지 않는 x 및 z 방향 오리엔테이션 정보, 즉 기울기로 local 좌표계에서 정의된다.
		/// y 방향 회전만을 포함하는 rotation
		/** m_aRotations[0] = m_rotAxis_y*m_offset_q */
		quater m_rotAxis_y;
	};
	TOnlineArray<PoseDelta> pose;
	DeltaRepMotionDOF(MotionDOF& mot, int start): mTarget(mot),
	pose(mot.numFrames(), mot.numFrames()-start+2) 
	{
		calcInterFrameDifference(start -1);
	}

	virtual void decomposeRot(int iframe)
	{
		quater rootOrientation=mTarget.row(iframe).toQuater(3);
		rootOrientation.decompose(pose(iframe).m_rotAxis_y, pose(iframe).m_offset_q);
	}
	virtual void composeRot(int i)
	{
		quater rootOrientation;
		rootOrientation.mult(pose(i).m_rotAxis_y, pose(i).m_offset_q);
		rootOrientation.normalize();
		mTarget.row(i).setQuater(3, rootOrientation);
	}
	virtual quater & rotY(int iframe) { return pose(iframe).m_rotAxis_y; }
	virtual quater & dQ(int iframe) { return pose(iframe).m_rotAxis_y; }
	virtual vector3 pos(int iframe) const { return mTarget.row(iframe).toVector3(0);}
	virtual void setPos(int iframe, double px, double pz)
	{
		mTarget.row(iframe).set(0, px);
		mTarget.row(iframe).set(2, pz);
	}
	virtual int numFrames() const
	{
		return mTarget.numFrames();
	}
	virtual void calcInterFrameDifference(int stFrm) {

		if(numFrames()==0)
			return;
		int i;

		ASSERT(stFrm<numFrames());
		ASSERT(stFrm>-1) ;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation for frame stFrm
		///////////////////////////////////////////////////////////////////////////////
		decomposeRot(stFrm);

		for(i=stFrm+1; i<numFrames(); i++)
		{
			///////////////////////////////////////////////////////////////////////////////
			//  calculation m_dv and offset_y
			///////////////////////////////////////////////////////////////////////////////
			pose(i).m_dv = pos(i) - pos(i-1);
			pose(i).m_dv.y = 0;

			quater inv_q;
			inv_q.inverse(pose(i-1).m_rotAxis_y);
			pose(i).m_dv.rotate(inv_q,pose(i).m_dv);
			pose(i).m_offset_y = pos(i).y;

			///////////////////////////////////////////////////////////////////////////////
			//  calculation rotAxis_y	pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q
			//							pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
			//			  		  thus, pose(i).m_dq = pose(i).m_rotAxis_y * (pose(i-1).m_rotAxis_y)^(-1)
			///////////////////////////////////////////////////////////////////////////////

			decomposeRot(i);




			pose(i).m_dq.mult(pose(i).m_rotAxis_y, inv_q);
			pose(i).m_dq.align(quater(1,0,0,0));	// m_dq should be small.

		}
	}
	virtual void reconstructPos(int iframe)
	{
		if(iframe > numFrames()-1) return;
		int i;

		/* backup
		vector3 prev_v, dv;
		prev_v = keyvalue[iframe].m_aTranslations[0];

		for(i=iframe+1; i<numFrames(); i++)
		{
			dv.rotate(keyvalue[i-1].m_rotAxis_y, keyvalue[i].m_dv);
			prev_v = prev_v+dv;
			keyvalue[i].m_aTranslations[0] = prev_v;
			keyvalue[i].m_aTranslations[0].y = keyvalue[i].m_offset_y;
		}
		*/
		// simplified and optimized
		vector3 dv;
		for(i=iframe+1; i<numFrames(); i++)
		{
			dv.rotate(pose(i-1).m_rotAxis_y, pose(i).m_dv);
			mTarget.row(i).set(0, mTarget(i-1,0)+ dv.x);
			mTarget.row(i).set(1, pose(i).m_offset_y);
			mTarget.row(i).set(2, mTarget(i-1,2)+ dv.z);
		}
	}
	virtual void reconstructAll(int iframe)
	{ 
		if(iframe<0) iframe=0;
		if(iframe > numFrames()-1) return;
		int i;

		//	pose(iframe).m_aRotations[0].mult(pose(iframe).m_rotAxis_y, pose(iframe).m_offset_q);
		decomposeRot(iframe);

		for(i=iframe+1; i<numFrames(); i++)
		{
			//	pose(i).m_dq * pose(i-1).m_rotAxis_y = pose(i).m_rotAxis_y
			pose(i).m_rotAxis_y.mult(pose(i).m_dq, pose(i-1).m_rotAxis_y);
			pose(i).m_rotAxis_y.normalize();
			composeRot(i);
		}
		reconstructPos(iframe);
	}


};

RetargetOnline2D::RetargetOnline2D(Motion& target, int start, int eRetargetQMethod)
	:mRetargetQMethod(eRetargetQMethod)
{
	mTarget=new DeltaRepMotion(target);
	getCurve(start);
}
RetargetOnline2D::RetargetOnline2D(MotionDOF& target, int start, int eRetargetQMethod)
	:mRetargetQMethod(eRetargetQMethod)
{
	mTarget=new DeltaRepMotionDOF(target, start);
	getCurve(start);
}

RetargetOnline2D::~RetargetOnline2D()
{
	delete mTarget;
}

void RetargetOnline2D::getCurve(int start)
{	
	mStart=start-2;	// 첫 두프레임은 변하지 않는다.
	if(mStart<1) mStart=1;
}

void RetargetOnline2D::adjust(int time, quater const& oriY, vector3 const& pos2D)
{
	// constrained optimization for root orientation.
	// acceleration difference should be minimized.

	// constriant: positional constraint, orientational constraint.
	

	adjust(time, oriY);
	adjust(time, pos2D);
}

void RetargetOnline2D::adjust(int time, vector3 const& pos2D)
{
	vectorn con(2);
	con[0]=pos2D.x;
	con[1]=pos2D.z;

	matrixn curve;

	curve.setSize(mTarget->numFrames()-mStart, 2);

	for(int i=mStart; i<mTarget->numFrames(); i++)
	{
		curve[i-mStart][0]=mTarget->pos(i).x;
		curve[i-mStart][1]=mTarget->pos(i).z;
	}
	m0::adjustOnline(time-mStart,  con).calc(curve);

	for(int i=mStart; i<mTarget->numFrames(); i++)
	{
		mTarget->setPos(i,curve[i-mStart][0], curve[i-mStart][1]);
	}
}


void RetargetOnline2D::adjust(int time, quater const& oriY)
{
	quater qdiff;
	qdiff.difference(mTarget->rotY(time), oriY);

	m_real deltarot=qdiff.rotationAngleAboutAxis(vector3(0,1,0));
	adjust(time, deltarot);
}

void RetargetOnline2D::adjustSafe(int time, m_real deltarot)
{
	mCurve.setSize(mTarget->numFrames()-mStart,1);

	for(int i=mStart; i<mTarget->numFrames(); i++)
	{
		mTarget->decomposeRot(i);
	}
	mCurve[0][0]=mTarget->rotY(mStart).rotationAngleAboutAxis(vector3(0,1,0));

	quater q;
	for(int i=mStart+1; i<mTarget->numFrames(); i++)
	{
		q.difference(mTarget->rotY(i-1), mTarget->rotY(i));
		mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
	}

	vectorn con(1);
	con[0]=deltarot+mCurve[time-mStart][0];
	m0::adjust(time-mStart, con).calc(mCurve.range(0, mCurve.rows(), 0, 1).lval());


	for(int i=mStart; i<mTarget->numFrames(); i++)
	{
		mTarget->rotY(i).setRotation(vector3(0,1,0), mCurve[i-mStart][0]);
		mTarget->composeRot(i);
	}

}
void RetargetOnline2D::adjust(int time, m_real deltarot)
{
	if(mRetargetQMethod==RETARGET_ROTY)
	{
		mCurve.setSize(mTarget->numFrames()-mStart,1);

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->decomposeRot(i);
		}
		mCurve[0][0]=mTarget->rotY(mStart).rotationAngleAboutAxis(vector3(0,1,0));

		quater q;
		for(int i=mStart+1; i<mTarget->numFrames(); i++)
		{
			q.difference(mTarget->rotY(i-1), mTarget->rotY(i));
			mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
		}

		vectorn con(1);
		con[0]=deltarot+mCurve[time-mStart][0];
		m0::adjustOnline(time-mStart, con).calc(mCurve.range(0, mCurve.rows(), 0, 1).lval());


		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->rotY(i).setRotation(vector3(0,1,0), mCurve[i-mStart][0]);		
			mTarget->composeRot(i);
		}

		mTarget->reconstructPos(mStart-1);
	}
	else
	{
		mTarget->calcInterFrameDifference(mStart-1);

		// dv, drot
		mCurve.setSize(mTarget->numFrames()-mStart,1 );

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mCurve[i-mStart][0]=mTarget->dQ(i).rotationAngleAboutAxis(vector3(0,1,0));
		}

		vectorn con(1);
		con[0]=deltarot;
		m0::adjustSumOnline(time-mStart, con).calc(mCurve.range(0, mCurve.rows(), 0, 1).lval());

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->dQ(i).setRotation(vector3(0, mCurve[i-mStart][0],0));
		}

		mTarget->reconstructAll(mStart-1);
	}
}

void RetargetOnline2D::adjust(int criticalTimeBefore, int criticalTimeAfter, intvectorn& times)
{
	Motion& inout=dynamic_cast<DeltaRepMotion*>(mTarget)->mTarget;
	int playEnd=mStart+2;

	//    playEnd         criticalTimeBefore
	// ---|---------------|-------|
	//    | adjustMot     |
	//                    |leftMot|


	// case 1:
	// ---|-----------|-------|
	//                criticalTimeAfter


	// case 2:
	// ---|-------------------|-------|
	//                criticalTimeAfter


	int leftMotionSize=inout.numFrames()-criticalTimeBefore;

	if(leftMotionSize<0) 
	{
		printf("strange error\n");
		return;
	}
//#define USE_ONLINE_ADJUST
#ifdef USE_ONLINE_ADJUST
	// online adjust 사용.

	int start=playEnd-1;	// 첫 한프레임은 변하지 않는다.

	// timewarping function
	//                             +   (y=x)                                      + y=P(x)
	//                           .                                              .
	//                         .                                              .
	//                       .                                             .
	//                     *                                            * 
	//                   .                                          .
	//                 .                                         .
	//			     .                                        .
	//             .                                        .
	//           .                                        .
	//           0 1 2 3 4 5 6 7 8 9            ->        0 1 2 3 4 5 6 7 8 9 0 1 2
	//			 PE        C         N                    PE            C'          N'

	// timewarping function을 P라하면, 
	// P(C')=C,
	// P(0)=0
	// P(N'-1)=N 이 나오는 P(x), x=0,1,...,N' 을 구해야한다.

	// 하지만 부드러운 P를 구하기가 쉽지 않다. (N'이 미정이기 때문)

	// 반면, P의 역함수 Q는 구하기 쉽다.
	// -->  Q(0)=0, Q(C)=C' 인 임의의 부드러운 함수를 구하면 됨.
	// 이로부터 N'을 구하고,

	vectorn Q;
	Q.linspace(0, inout.numFrames()-start-1, inout.numFrames()-start);
	
	m0::adjustOnline a(criticalTimeBefore-start, vectorn(1, criticalTimeAfter-start)); 
	a.calc(Q.cols());

	int N_=ROUND(Q[Q.size()-1])+start+1;

	// 다시 P를 만든다.
	vectorn P;
	P.linspace(0, N_-start-1, N_-start);
	
	m0::adjustOnline b(criticalTimeAfter-start, vectorn(1, criticalTimeBefore-start));
	b.calc(P.cols());

	// P(N'-1)=N-1이라는 보장이 없다. 하지만 비슷할것이다. 노멀라이즈 한다.
	P*=(inout.numFrames()-start-1)/P[P.size()-1];

	P+=start;

	for(int i=times.size()-1; i>=0; i--)
	{
		if(times[i]>=start)
			times[i]=P.argNearest(times[i])+start;
		else break;
	}

	static Motion temp2;
	MotionUtil::timewarpingLinear(temp2, inout, P);

	inout.Resize(inout.numFrames()+P.size()-Q.size());

	for(int i=start; i<start+P.size(); i++)
		inout.pose(i)=temp2.pose(i-start);

#else
	// USE_PIECEWISE_LINEAR_TIMEWARPING
	static Motion temp2;
	vectorn timewarpFunction(criticalTimeAfter-playEnd+1);
	timewarpFunction.linspace(playEnd, criticalTimeBefore);	

	for(int i=times.size()-1; i>=0; i--)
	{
		if(times[i]>=criticalTimeBefore)
			times[i]+=criticalTimeAfter-criticalTimeBefore;
		else if(times[i]>=playEnd)
			times[i]=timewarpFunction.argNearest(times[i])+playEnd;
		else break;
	}

	MotionUtil::timewarpingLinear(temp2, inout, timewarpFunction);

	if(criticalTimeAfter>criticalTimeBefore)
	{
		// increasing size
		int numFrameOld=inout.numFrames();
		inout.Resize(inout.numFrames()+criticalTimeAfter-criticalTimeBefore);
		for(int i=numFrameOld-1; i>=criticalTimeBefore; i--)
			inout.pose(i-numFrameOld+inout.numFrames())=inout.pose(i);

		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.pose(i)=temp2.pose(i-playEnd);
	}
	else
	{
		// decreasing size
		for(int i=0; i<leftMotionSize; i++)
			inout.pose(criticalTimeAfter+i)=inout.pose(criticalTimeBefore+i);
		inout.Resize(inout.numFrames()+criticalTimeAfter-criticalTimeBefore);


		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.pose(i)=temp2.pose(i-playEnd);
	}
#endif
}

void decompose(quater const& q, vector3& qq)
{
	quater qy, qoffset;
	q.decompose(qy, qoffset);
	qq[0]=qy.rotationAngle(vector3(0,1,0));
	qoffset.align(quater(1,0,0,0));
	vector3 rot=qoffset.rotationVector();
	qq[1]=rot.x;
	qq[2]=rot.z;
}

void inverseDecompose(quater& q, vector3 const& qq)
{
	quater qy, qoffset;
	qy.setRotation(vector3(0,1,0), qq[0]);
	qoffset.setRotation(vector3(qq[1],0,qq[2]));
	q.mult(qy, qoffset);
}

void alignAngle(m_real prev, m_real& next)
{
	m_real nprev=prev;
	// prev는 [-PI,PI]에 있다고 보장을 못한다. 따라서 이범위에 들어오는 nprev를 구한다.
	while(nprev>M_PI+FERR)
		nprev-=2.0*M_PI;
	while(nprev<-1.0*M_PI-FERR)
		nprev+=2.0*M_PI;

	if(ABS(nprev-next)>M_PI)
	{
		if(next>nprev)
			next-=2.0*M_PI;
		else
			next+=2.0*M_PI;
	}

	// 다시 원래의 prev범위로 되돌린다.
	next+=prev-nprev;
}

void quaterlinstitch_decompose(quaterN const& a, quaterN const& b, quaterN & c)
{
	vector3N ma, mb, mc;

	ma.setSize(a.rows());
	mb.setSize(b.rows());

	for(int i=0; i<ma.rows(); i++)
		decompose(a.row(i), ma.row(i));

	for(int i=0; i<mb.rows(); i++)
		decompose(b.row(i), mb.row(i));

	// -pi < ma[0], mb[0]< pi
	// we need to align them so that inter frame difference do not exceeds pi

	for(int i=0; i<ma.rows()-1; i++)
		alignAngle(ma[i][0], ma[i+1][0]);
	alignAngle(ma[ma.rows()-1][0], mb[0][0]);
	for(int i=0; i<mb.rows()-1; i++)
		alignAngle(mb[i][0], mb[i+1][0]);

	mc.linstitch(ma, mb);

	c.setSize(mc.rows());
	for(int i=0; i<c.rows(); i++)
		inverseDecompose(c.row(i), mc.row(i));
}

vector3 quaterLog(quater const& a, quater const& ref)
{
	quater b;
	b.difference(ref, a);
	return b.log();
}

void quaterlinstitch_log(quaterN const& a, quaterN const& b, quaterN & c)
{
	vector3N ma, mb, mc;

	ma.setSize(a.rows());
	mb.setSize(b.rows());

	quater refCoord;
	refCoord.safeSlerp(a.row(a.rows()-1), b.row(0), 0.5);

	for(int i=0; i<ma.rows(); i++)
		ma.row(i)=quaterLog(a.row(i), refCoord);

	for(int i=0; i<mb.rows(); i++)
		mb.row(i)=quaterLog(b.row(i), refCoord);

	mc.linstitch(ma, mb);

	c.setSize(mc.rows());
	for(int i=0; i<c.rows(); i++)
		c.row(i).mult(mc.row(i).exp(), refCoord);
}





void MotionUtil::RetargetOnline2D::adjustToPath(Path2D & path, int thr, m_real angleThr, m_real distThr)
{
	mCurve.setSize(mTarget->numFrames()-mStart, 2);// first allocate memory
	//first retarget orientations.	
	{
		mCurve.setSize(mTarget->numFrames()-mStart,1);

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->decomposeRot(i);
		}
		mCurve[0][0]=mTarget->rotY(mStart).rotationAngleAboutAxis(vector3(0,1,0));

		quater q;
		for(int i=mStart+1; i<mTarget->numFrames(); i++)
		{
			q.difference(mTarget->rotY(i-1), mTarget->rotY(i));
			mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
		}

		matrixn mCon;
		mCon.setSize(mTarget->numFrames()-mStart, 1);
		mCon.setAllValue(FLT_MAX);

		int pathEnd=path.start()+path.size();
		mCon[path.start()-1-mStart][0]=path.ori(path.start()-1).rotationAngleAboutAxis(vector3(0,1,0));
		for(int i=path.start(); i<path.start()+path.size(); i++)
			mCon[i-mStart][0]=
			mCon[i-1-mStart][0]+path.dq(i).rotationAngleAboutAxis(vector3(0,1,0));
		
		//printf("mStart %d pathstart %d pathend %d \n curveend %d", mStart, pathStart, pathEnd, mTarget->numFrames());
		
		int numCon=(pathEnd-path.start())/(int)thr+1;  // eg if thr==10 and 10<=(pathEnd-start)<20 , then numCon=2
		if(ABS(mCurve[pathEnd-1-mStart]-mCon[pathEnd-1-mStart])<angleThr)
			numCon=1;
		intvectorn time(numCon);
		matrixn con(numCon, 1);

		
		for(int i=0; i<numCon; i++)
		{
			// i=-1일때 pathStart, i=numCon-1일때 pathEnd-1가 나오면 됨.
			int t= ROUND((m_real(i+1)/m_real(numCon))*m_real(pathEnd-1-path.start()))+path.start();
			time[i]=t-mStart;
			con[i][0]=mCon[t-mStart][0];
		}
		
		m0::adjustOnlineMultiCon(time, con).calc(mCurve);

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->rotY(i).setRotation(vector3(0,1,0), mCurve[i-mStart][0]);		
			mTarget->composeRot(i);
		}

		mTarget->reconstructPos(mStart-1);
	}

	//then retarget positions
	{
		mCurve.setSize(mTarget->numFrames()-mStart, 2);

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mCurve[i-mStart][0]=mTarget->pos(i).x;
			mCurve[i-mStart][1]=mTarget->pos(i).z;
		}

		matrixn mCon;
		mCon.setSize(mTarget->numFrames()-mStart, 2);
		mCon.setAllValue(FLT_MAX);
		int pathStart=path.start()-1;	// -1부터 얻어올수 있다.
		int pathEnd=pathStart+path.size()+1;

		for(int i=pathStart; i<pathEnd; i++)
		{
			mCon[i-mStart][0]=path.pos(i).x;
			mCon[i-mStart][1]=path.pos(i).z;
		}

		//printf("mStart %d pathstart %d pathend %d \n curveend %d", mStart, pathStart, pathEnd, mTarget->numFrames());

		int numCon=(pathEnd-pathStart)/(int)thr+1;  // eg if thr==10 and 10<=(pathEnd-start)<20 , then numCon=2
		if(mCurve.row(pathEnd-1-mStart).distance(mCon.row(pathEnd-1-mStart))<distThr)
			numCon=1;
		intvectorn time(numCon);
		matrixn con(numCon, 2);

		for(int i=0; i<numCon; i++)
		{
			// i=-1일때 pathStart, i=numCon-1일때 pathEnd-1가 나오면 됨.
			int t= ROUND((m_real(i+1)/m_real(numCon))*m_real(pathEnd-1-pathStart))+pathStart;
			time[i]=t-mStart;
			con[i][0]=mCon[t-mStart][0];
			con[i][1]=mCon[t-mStart][1];
		}

		m0::adjustOnlineMultiCon(time, con).calc(mCurve);

		for(int i=mStart; i<mTarget->numFrames(); i++)
		{
			mTarget->setPos(i,mCurve[i-mStart][0], mCurve[i-mStart][1]);
		}
	}
}
