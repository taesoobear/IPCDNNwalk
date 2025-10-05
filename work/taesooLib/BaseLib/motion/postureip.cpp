// PostureIP.cpp: implementation of the PostureIP class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "postureip.h"
#include "MotionLoader.h"
#include "ConstraintMarking.h"
#include "../BaseLib/math/Filter.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/OperatorQuater.h"
#include "my.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Posture::Posture()
:m_dv(0.f,0.f,0.f)
,m_dq(1.f,0.f,0.f,0.f)
,m_offset_y(0.f)
,m_offset_q(1.f,0.f,0.f,0.f)
,m_rotAxis_y(1.f,0.f,0.f,0.f)
{
}

Posture::Posture(const Posture& other)
:m_dv(0.f,0.f,0.f)
,m_dq(1.f,0.f,0.f,0.f)
,m_offset_y(0.f)
,m_offset_q(1.f,0.f,0.f,0.f)
,m_rotAxis_y(1.f,0.f,0.f,0.f)
{
	Clone(&other);
}
Posture::Posture(const MotionLoader& other)
:m_dv(0.f,0.f,0.f)
,m_dq(1.f,0.f,0.f,0.f)
,m_offset_y(0.f)
,m_offset_q(1.f,0.f,0.f,0.f)
,m_rotAxis_y(1.f,0.f,0.f,0.f)
{
	Init(other.numRotJoint(), other.numTransJoint());
	other.getPose(*this);
}

vector3 Posture::front()
{
	vector3 front;
	vector3 lfront(0,0,1);
	front.rotate(m_aRotations[0], lfront);
	return front;
}

transf Posture::rootTransformation() const
{
	return transf(m_aRotations[0], m_aTranslations[0]);
}

void Posture::setRootTransformation(transf const& rootTransf)
{
	m_aRotations[0]=rootTransf.rotation;
	m_aRotations[0].normalize();	
	// 위 라인이 없이는 setRootTransformation(rootTransformation())을 한 30번만 반복하면 m_aRotation의 길이가 2를 넘어선다-.-

	m_aTranslations[0]=rootTransf.translation;
}

void Posture::decomposeRot() const
{
	m_aRotations[0].decompose(m_rotAxis_y, m_offset_q);
}

void Posture::Init(int numRotJoint, int numTransJoint)
{
	m_aRotations.setSize(numRotJoint);
	m_aTranslations.setSize(numTransJoint);
}

void Posture::identity()
{
	for(int i=0; i<m_aRotations.size(); i++)
		m_aRotations[i].identity();

	for(int i=0; i<m_aTranslations.size(); i++)
		m_aTranslations[i].setValue(0,0,0);
}

void Posture::pack(BinaryFile & bf, int version) const
{
	if(version==1)
		bf.packInt(numRotJoint()*-1);
	else
		bf.packInt(numRotJoint());

	if(version>=3)
	{
		bf.packInt(numTransJoint());
		for(int i=0; i<numTransJoint(); i++)
			bf.pack(m_aTranslations[i]);
	}
	else
		bf.pack(m_aTranslations[0]);
	
	bf.pack(m_conToeL);
	bf.pack(m_conToeR);

	for(int i=0; i<numRotJoint(); i++)
		bf.pack(m_aRotations[i]);

	bf.pack(constraint);
	bf.pack(m_dv);			
	bf.pack(m_dq);			 
	bf.packFloat(m_offset_y);		//!< m_dv가 포함 하지 않는 y value
	bf.pack(m_offset_q);
	bf.pack(m_rotAxis_y);

	if(version>1)
	{
		bf.pack(m_additionalLinear);
		bf.pack(m_additionalQuater);
	}
}

void Posture::unpack(BinaryFile & bf, int version) 
{
	int numRotJoint=bf.unpackInt();	

	if(version==1)
	{
		if(numRotJoint>=0) version=0;
		else numRotJoint*=-1;
	}
	
	if(version>=3)
	{
		int numTransJoint=bf.unpackInt();
		Init(numRotJoint, numTransJoint);
		for(int i=0; i<numTransJoint; i++)
			bf.unpack(m_aTranslations[i]);
	}
	else
	{
		Init(numRotJoint, 1);
		bf.unpack(m_aTranslations[0]);
	}

	if(version>0)
	{
		bf.unpack(m_conToeL);
		bf.unpack(m_conToeR);
	}

	for(int i=0; i<numRotJoint; i++)
		bf.unpack(m_aRotations[i]);

	bf.unpack(constraint);
	bf.unpack(m_dv);			
	bf.unpack(m_dq);			 
	bf.unpackFloat(m_offset_y);		//!< m_dv가 포함 하지 않는 y value
	bf.unpack(m_offset_q);
	bf.unpack(m_rotAxis_y);

	if(version>1)
	{
		bf.unpack(m_additionalLinear);
		bf.unpack(m_additionalQuater);
	}
}

Posture::~Posture()
{
}

Posture* Posture::clone() const
{
	return new Posture(*this);
}

void Posture::_clone(const Posture* pPosture)
{
	if(numRotJoint()!= pPosture->numRotJoint())
	{
		m_aRotations.setSize(pPosture->numRotJoint());
	}

	if(numTransJoint()!= pPosture->numTransJoint())
	{
		m_aTranslations.setSize(pPosture->numTransJoint());
	}

	for(int i=0; i<numTransJoint(); i++)
	{
		m_aTranslations[i]=pPosture->m_aTranslations[i];
	}

	for(int i=0; i<numRotJoint(); i++)
	{
		m_aRotations[i]=pPosture->m_aRotations[i];
	}

	m_dv = pPosture->m_dv;
	m_dq = pPosture->m_dq;
	m_offset_y = pPosture->m_offset_y;
	m_offset_q = pPosture->m_offset_q;
	m_rotAxis_y = pPosture->m_rotAxis_y;
	constraint = pPosture->constraint;

	m_conToeL = pPosture->m_conToeL;
	m_conToeR = pPosture->m_conToeR;

	m_additionalLinear = pPosture->m_additionalLinear ;
	// 쓰고 싶은대로 쓸것. size는 4의 배수. quaternion blending이 각 4컬럼씩마다 잘라서 수행된다.
	m_additionalQuater = pPosture->m_additionalQuater ;

}


void Posture::Blend(const Posture& a, const Posture& b, m_real weight)
{
	ASSERT(a.numRotJoint() == b.numRotJoint());
	ASSERT(a.numTransJoint() == b.numTransJoint());
	if(numRotJoint()!=a.numRotJoint() ||
		numTransJoint()!=a.numTransJoint())
	{
		Init(a.numRotJoint(), a.numTransJoint());
	}

	m_dv.interpolate(weight, a.m_dv, b.m_dv);

	for(int i=0; i<numTransJoint(); i++)
		m_aTranslations[i].interpolate(weight, a.m_aTranslations[i], b.m_aTranslations[i]);	

	for(int i=0; i<numRotJoint(); i++)
		m_aRotations[i].safeSlerp(a.m_aRotations[i], b.m_aRotations[i], weight);
	
	m_dq.safeSlerp(a.m_dq, b.m_dq, weight);
	m_offset_q.safeSlerp( a.m_offset_q, b.m_offset_q, weight);
	m_rotAxis_y.safeSlerp(a.m_rotAxis_y, b.m_rotAxis_y, weight);
	m_offset_y = (1.0f-weight)*a.m_offset_y + weight*b.m_offset_y;

	m_conToeL.interpolate(weight, a.m_conToeL, b.m_conToeL);
	m_conToeR.interpolate(weight, a.m_conToeR, b.m_conToeR);

	if(weight>0.5)
	{
		constraint = b.constraint;
	}
	else
	{
		constraint = a.constraint;
	}

	v::interpolate(m_additionalLinear, weight, a.m_additionalLinear, b.m_additionalLinear);
	v::interpolateQuater(m_additionalQuater,weight, a.m_additionalQuater, b.m_additionalQuater);
}

void quater_blend(quater const& refq, quater& q, vectorn const& w, matrixn& mat)
{
	// method 1: logarithm map
	/*	vector3 logSum(0,0,0);
	quater diff;

	for(int i=0; i<mat.rows(); i++)
	{
	diff.difference(refq, mat.row(i).toQuater());
	logSum+=diff.log()*w[i];
	}

	diff.exp(logSum);
	q.mult(diff, refq);
	*/
	/*
	// method 2: sin method
	q=refq;*/

	// method 3: renormalization

	quater sum(0,0,0,0);
	quater diff;

	for(int i=0; i<mat.rows(); i++)
	{
		diff.difference(refq, mat.row(i).toQuater());
		sum+=diff*w[i];
	}

	sum.normalize();
	q.mult(sum, refq);
}
void quater_blend(quater& q, vectorn const& w, matrixn& mat)
{
	quater refq;
	refq.blend(w, mat);

	quater_blend(refq, q, w, mat);	
}



void Posture::Blend(const Posture** apPostures, const vectorn& weight)
{
	int n=weight.size();
	const Posture& first=*apPostures[0];

	if(numRotJoint()!=first.numRotJoint() ||
		numTransJoint()!=first.numTransJoint())
	{
		Init(first.numRotJoint(), first.numTransJoint());
	}

	vector3 sum_dv, sum_rootTrans;

	m_dv.setValue(0,0,0);
	for(int j=0; j<numTransJoint(); j++)
		m_aTranslations[j].setValue(0,0,0);
	m_offset_y=0;
	m_conToeL.setValue(0,0,0);
	m_conToeR.setValue(0,0,0);
	
	m_real actualWeightL=0;
	m_real actualWeightR=0;

	// linear terms
	for(int i=0; i<n; i++)
	{
		m_dv.multadd(apPostures[i]->m_dv, weight[i]);
		for(int j=0; j<numTransJoint(); j++)
			m_aTranslations[j].multadd(apPostures[i]->m_aTranslations[j], weight[i]);
		m_offset_y+= apPostures[i]->m_offset_y*weight[i];
		if(apPostures[i]->constraint[CONSTRAINT_LEFT_TOE])
		{
			m_conToeL.multadd(apPostures[i]->m_conToeL, weight[i]);
			actualWeightL+=weight[i];
		}
		if(apPostures[i]->constraint[CONSTRAINT_RIGHT_TOE])
		{
			m_conToeR.multadd(apPostures[i]->m_conToeR, weight[i]);
			actualWeightR+=weight[i];
		}
	}

	if(actualWeightL!=0)
		m_conToeL/=actualWeightL;
	
	if(actualWeightR!=0)
		m_conToeR/=actualWeightR;

	// constraint selecting
	constraint = apPostures[0]->constraint;	

	// constraint blending
/*	vectorn constraintSum(NUM_CONSTRAINT);
	constraintSum.setAllValue(0);

	for(int c=0; c<NUM_CONSTRAINT; c++)
	{
		for(int i=0; i<n; i++)
			constraintSum[c]+=(m_real)((int)(apPostures[i]->constraint[c]))*weight[i];
        if(constraintSum[c]>0.5 && apPostures[0]->constraint[c]) constraint.SetAt(c) ; 
	}
*/

	matrixn tempQuaterMat;
	tempQuaterMat.setSize(n,4);

	// sphere terms
	for(int i=0; i<numRotJoint(); i++)
	{
		for(int j=0; j<n; j++)
			tempQuaterMat.row(j).assign(apPostures[j]->m_aRotations[i]);

		quater_blend(m_aRotations[i], weight, tempQuaterMat);
	}

	for(int j=0; j<n; j++)
		tempQuaterMat.row(j).assign(apPostures[j]->m_dq);
	quater_blend(m_dq, weight, tempQuaterMat);
	m_dq.align(quater(1,0,0,0));

	for(int j=0; j<n; j++)
		tempQuaterMat.row(j).assign(apPostures[j]->m_offset_q);
	quater_blend(m_offset_q,weight, tempQuaterMat);

	for(int j=0; j<n; j++)
		tempQuaterMat.row(j).assign(apPostures[j]->m_rotAxis_y);
	quater_blend(m_rotAxis_y, weight, tempQuaterMat);

	// linear terms
	m_additionalLinear.setSize(apPostures[0]->m_additionalLinear.size());
	m_additionalLinear.setAllValue(0);

	vectorn temp;
	for(int i=0; i<n; i++)
	{
		ASSERT(m_additionalLinear.size()==apPostures[i]->m_additionalLinear.size());
		temp.mult(apPostures[i]->m_additionalLinear, weight[i]);
		m_additionalLinear+=temp;
	}

	//quaternion terms
	m_additionalQuater.setSize(apPostures[0]->m_additionalQuater.size());
	int nq=m_additionalQuater.size()/4;
	for(int qq=0; qq<nq; qq++)
	{
		
		
		for(int j=0; j<n; j++)
		{
			ASSERT(m_additionalQuater.size()==apPostures[j]->m_additionalQuater.size());
			tempQuaterMat.row(j).assign(apPostures[j]->m_additionalQuater.toQuater(qq*4));
		}
		quater tq;
		quater_blend(tq, weight, tempQuaterMat);
		m_additionalQuater.range(qq*4, (qq+1)*4)=tq;
	}
//	for(int i=0; i<n; i++)
//	{
//		vector3 tmpPos;
//		apPostures[i]->decomposeRot();
//		tmpPos=apPostures[i]->m_oppenentPos;
//		tmpPos.rotate(apPostures[i]->m_rotAxis_y);
//		m_oppenentPos+=tmpPos*weight[i];
//	}
//	decomposeRot();
//	m_oppenentPos.rotate(m_rotAxis_y.inverse());

}

void Posture::Align(const Posture& other)
{
	m_aTranslations[0]=other.m_aTranslations[0];
	m_aTranslations[0].y=m_offset_y;
	m_rotAxis_y=other.m_rotAxis_y;
	m_aRotations[0] = m_rotAxis_y * m_offset_q;
}


void dep_toLocalDirection(Posture const& p, const vector3& dv, vector3& dir, bool bOnlyVerticalAxis) 
{
	quater inv_q;

	if(bOnlyVerticalAxis)
	{
		p.decomposeRot();
		inv_q.inverse(p.m_rotAxis_y);
	}
	else
		inv_q.inverse(p.m_aRotations[0]);


	dir.rotate(inv_q, dv);
}

void dep_toGlobalDirection(Posture const& p, const vector3& dir, vector3& dv, bool bOnlyVerticalAxis) 
{
	if(bOnlyVerticalAxis)
	{
		p.decomposeRot();
		dv.rotate(p.m_rotAxis_y, dir);
	}
	else
		dv.rotate(p.m_aRotations[0], dir);

 
}

void dep_toLocal(Posture& p, const vector3& pos, vector3& localPos)
{
	vector3 dv;
	dv.difference(p.m_aTranslations[0], pos);
	
	dep_toLocalDirection(p, dv, localPos);
}

void dep_toGlobal(Posture& p, const vector3& pos, vector3& GlobalPos)
{
	dep_toGlobalDirection(p, pos, GlobalPos);
	GlobalPos+=p.m_aTranslations[0];
	//GlobalPos.add(m_aTranslations[0]);
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
/*
PostureIP::PostureIP() :Node()
{
	NodeType=POSTUREIP;
}
}
*/


vector3 dep_toLocal(Posture& p,const vector3& pos)	{ vector3 localPos; dep_toLocal(p, pos, localPos); return localPos;}
vector3 dep_toGlobal(Posture& p,const vector3& pos)	{ vector3 globalPos; dep_toGlobal(p, pos, globalPos); return globalPos;}
