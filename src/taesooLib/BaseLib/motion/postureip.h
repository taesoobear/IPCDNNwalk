#ifndef _POSTUREIP_H_
#define _POSTUREIP_H_
// PostureIP.h: interface for the Posture class.

#if _MSC_VER>1000
#pragma once
#endif

#include "node.h"

enum {CONSTRAINT_LEFT_FOOT=0, CONSTRAINT_RIGHT_FOOT, CONSTRAINT_LEFT_HAND, CONSTRAINT_RIGHT_HAND,
CONSTRAINT_LEFT_TOE,CONSTRAINT_LEFT_HEEL, CONSTRAINT_RIGHT_TOE, CONSTRAINT_RIGHT_HEEL,
CONSTRAINT_INTERACTION, CONSTRAINT_CRI_INTERACTION,WHICH_FOOT_IS_SUPPORTED,IS_FOOT_SUPPORTED, POSE_IS_INVALID,
CONSTRAINT_LEFT_FINGERTIP, CONSTRAINT_RIGHT_FINGERTIP, IS_DISCONTINUOUS, NUM_CONSTRAINT};

enum {R_IS_SUPPORTED=0, L_IS_SUPPORTED=1};
enum {ONE_FOOT_IS_SUPPORTED,NO_FOOT_IS_SUPPORTED};

class MotionLoader;
//! BVH에 저장된 Matrix정보 저장
/*! Rendering시 AlzzaPostureIP에서 이 정보를 사용해 animation을 수행한다. 즉, 이 클래스는 PostureIP와 AlzzaPostureIP에서 동시에 사용되기 때문에, 깔끔하고 compact한 상태를 항상 유지하여야 한다.
	\ingroup group_motion
*/
class Posture
{
public:
	Posture();
	Posture(const Posture& other);
	Posture(const MotionLoader& skel); // init current pose
	virtual ~Posture();

	virtual void Init(int numRotJoint, int numTransJoint);
	void identity();
	int numRotJoint() const	{ return m_aRotations.size();}
	int numTransJoint() const	{ return m_aTranslations.size();}

	vector3N m_aTranslations;	// !<numTransJoint() 만큼의 translation을 갖는다.
	quaterN m_aRotations;	//!< numRotJoint()만큼의 rotation을 갖는다.


	Posture& operator=(const Posture& other)	{ Clone(&other); return *this;}
	void assignConstraintOnly(const Posture& other)	{ constraint=other.constraint;}


	virtual Posture* clone() const;
	virtual void Blend(const Posture& a, const Posture& b, m_real t);	//!< a*(1-t)+b*(t)
	virtual void Blend(const Posture& b, m_real t)	{	Blend(*this, b, t);}
	virtual void Blend(const Posture** apPostures, const vectorn& aWeights);
	virtual void Align(const Posture& other);

	void pack(BinaryFile & bf, int version) const;
	void unpack(BinaryFile & bf, int version) ;

	vector3 front();
	transf rootTransformation() const;
	void setRootTransformation(transf const& rootTransf);

	// m_aRotations[0]->m_rotAxis_y*m_offset_q로 decompose
	virtual void decomposeRot() const;


	BaseLib::BitArray constraint;

	///////////////////////////////////////////////
	// Inter-frame difference관련 시작
	///////////////////////////////////////////////

	/// 이전 프레임과 root의 planar 차이(이전 프레임의 local 좌표계에서 표현)
	/** pose(i).m_aTranslations[0] = pose(i-1).m_aTranslations[0] + pose(i-1).m_rotAxis_y.rotate(pose(i).m_dv) */
	vector3 m_dv;
	/// 이전 프레임과 root의 planar 오리엔테이션 차이(이전 프레임의 local 좌표계에서 표현)
	/** pose(i).m_rotAxis_y= pose(i).m_dq * pose(i-1).m_rotAxis_y */
	quater m_dq;

	// decomposeRot 의 결과가 저장되는 변수로 pose와 직접적으로 상관이 없어서 mutable로 함.
	mutable m_real m_offset_y;		//!< m_dv가 포함 하지 않는 y value
	mutable quater m_offset_q;		//!< m_dq가 포함하지 않는 x 및 z 방향 오리엔테이션 정보, 즉 기울기로 local 좌표계에서 정의된다.
	/// y 방향 회전만을 포함하는 rotation
	/** m_aRotations[0] = m_rotAxis_y*m_offset_q */
	mutable quater m_rotAxis_y;


	///////////////////////////////////////////////
	// 아래 변수들은 쓰고 싶은대로 쓸것.
	///////////////////////////////////////////////

	// linear term은 linear블렌딩이 사용된다.
	vectorn m_additionalLinear;
	// size는 4의 배수. quaternion blending이 각 4컬럼씩마다 잘라서 수행된다.
	vectorn m_additionalQuater;



	// 아래 함수는 dynamic type checking을 하지 않는다. 왠만하면 operator=이나 clone()함수를 사용할것.
	void _clone(const Posture* pPosture);

	// type checking. (derived class에서 type checking 구현)
	virtual void Clone(const Posture* pPosture)	{_clone(pPosture);}

	// constraint marking.cpp 
	vector3 m_conToeL;
	vector3 m_conToeR;
	vector3& conPosition(int constraint){return (constraint==CONSTRAINT_LEFT_TOE)?m_conToeL:m_conToeR;}


protected:

};


// deprecated
// 아래 함수들 대신 MotionUtil::Coordinate 사용.
void dep_toLocalDirection(Posture const&, const vector3& gdir, vector3& ldir, bool bOnlyVerticalAxis=false) ;
void dep_toGlobalDirection(Posture const&, const vector3& ldir, vector3& gdir, bool bOnlyVerticalAxis=false) ;
void dep_toLocal(Posture&,const vector3& pos, vector3& localPos);
void dep_toGlobal(Posture&,const vector3& pos, vector3& globalPos);
vector3 dep_toLocal(Posture&,const vector3& pos);
vector3 dep_toGlobal(Posture&,const vector3& pos);

#endif
