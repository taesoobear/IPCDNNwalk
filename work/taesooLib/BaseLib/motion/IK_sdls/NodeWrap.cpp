#include "stdafx.h"
#include "../../math/mathclass.h"
#include "../../math/optimize.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "../FullbodyIK.h"
#include "../Motion.h"
#include "../MotionUtil.h"
#include "../MotionLoader.h"
#include "../IKSolver.h"
#include "NodeWrap.h"
#include <memory.h>
#include "../../BaseLib/motion/intersectionTest.h"
#include "../../BaseLib/motion/Liegroup.h"
#include "../../BaseLib/math/Operator_NR.h"
void IK_sdls::LoaderToTree::getTheta(double *m_x)
{
	int c=0;
	for(int i=0; i<mNode.size(); i++)
	{
		IK_sdls::NodeWrap& n=mNode[i];
		for(int j=0; j< n.axes.length();j++)
		{
			ASSERT(n.node[j]->GetJointNum()==c);
			m_x[c]=n.node[j]->GetTheta();
			c++;
		}
	}
	ASSERT(c==mTree.GetNumJoint());
}
#include <iostream>
using namespace std;

void IK_sdls::LoaderToTree::updateBoneLength(MotionLoader const& loader)
{
	for(int i=2; i<loader.numBone(); i++)
	{
		Node* link=getNode(i).node[0];
		//cout<<i <<":"<<link->_local.translation <<"->"<<loader.bone(i).getOffsetTransform().translation<<endl;
		link->_local.translation=loader.bone(i).getOffsetTransform().translation;
	}
}
int IK_sdls::LoaderToTree::getVarIndexByAxis(int treeIndex, const char *axis)
{
	auto& nodes=getNode(treeIndex);
	int dofIndex=nodes.axes.findChar(0, axis[0]);
	return nodes.node[dofIndex]->GetJointNum();
}

void IK_sdls::LoaderToTree::setTheta(const double *m_x)
{
	int c=0;
	for(int i=0; i<mNode.size(); i++)
	{
		IK_sdls::NodeWrap& n=mNode[i];
		for(int j=0; j< n.axes.length();j++)
		{
			ASSERT(n.node[j]->GetJointNum()==c);
			n.node[j]->SetTheta(m_x[c]);
			c++;
		}
	}
	ASSERT(c==mTree.GetNumJoint());
}

static void calcJTchain(matrixn& JT, IK_sdls::LoaderToTree& tree, int newroot, int tail, vector3 const& gpos)
{
	matrixn temp;
	vector3 lpos=tree.globalFrame(tail).toLocalPos(gpos);
	vector3 lpos2=tree.globalFrame(newroot).toLocalPos(gpos);
	tree.calcJacobianTransposeAt(JT, tail,lpos);
	tree.calcJacobianTransposeAt(temp, newroot,lpos2);
	JT-=temp;
}

void IK_sdls::LoaderToTree::calcJacobianTransposeAt(matrixn& JT, int chainRootbone, int chainTailBone, vector3 const& localpos_tailbone)
{
	int nJoint = mTree.GetNumJoint();
	double totalmass=0.0;
	// compute jacobian
	JT.setSize(nJoint, 3);
	JT.setAllValue(0.0);
	calcJTchain(JT, *this, chainRootbone, chainTailBone, globalFrame(chainTailBone).toGlobalPos(localpos_tailbone));
}
static inline double dot(double* a, Liegroup::dse3 const& b)
{
	double out=0;
	for (int i=0; i<6; i++)
		out+=a[i]*b[i];
	return out;
}

static inline void radd(::vectorn & v, Liegroup::dse3 const& vv)
{
	for (int i=0; i<6; i++)
		v(i)+=vv[i];
}
static inline Liegroup::dse3 mult(matrixn const& in, Liegroup::dse3 const& in2)
{
	Liegroup::dse3 out;
	for (int i=0; i<6; i++)
	{
		out[i]=dot(&in(i,0), in2);
	}	
	return out;
}

void IK_sdls::LoaderToTree::calcMomentumJacobianTranspose(const VRMLloader& l, matrixn& JT)
{
	VRMLTransform* b;
	int nJoint = mTree.GetNumJoint();
	JT.setSize(nJoint, 6);
	JT.setAllValue(0.0);

	::vector3 COM;
	double totalMass;
	COM=calcCOM(l);
	matrixn bJ(6, nJoint);
	matrixn j1T, j2T;
	matrixn dAd_T(6,6);
	for(int i=1; i<l.numBone(); i++) {
		b=&l.VRMLbone(i);

		vector3 localpos(0,0,0);
		//vector3 localpos=b->localCOM();

		calcJacobianTransposeAt(j1T, i, localpos);
		calcRotJacobianTranspose(j2T, i);

		Node* link = getLastNode(i);
		matrix3 R(link->globalFrame().rotation);
		matrix3 invR;
		invR.inverse(R);

		for(int j=0; j<bJ.cols(); j++)
		{
			bJ.column(j).setVec3(0, invR*j2T.row(j).toVector3(0));
			bJ.column(j).setVec3(3, invR*j1T.row(j).toVector3(0));
		}

		matrix4 bodyT(link->globalFrame());
		//matrix4 bodyT(link->globalFrame()*transf(b->localCOM()));
		matrix4 invBodyT;
		invBodyT.inverse(bodyT);

		// T=inverse(-COM*bodyT)
		matrix4 T=invBodyT*Liegroup::toSE3(COM);
		Liegroup::dAd(dAd_T, T); 

		VRMLTransform& bone=*b;
		double mass=bone.mass();
		Liegroup::Inertia bodyI(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		//Liegroup::Inertia bodyI(mass, bone.momentsOfInertia(), vector3(0,0,0));

		for (int j=0; j<bJ.cols(); j++){
			Liegroup::se3 cj=Liegroup::to_se3(bJ.column(j));
			Liegroup::dse3 temp=bodyI*cj;
			radd(JT.row(j).lval(),mult( dAd_T,temp));
		}
	}
}
double IK_sdls::LoaderToTree::calcInertia(const VRMLloader& loader, vectorn& inertia) const
{
	::vector3 com(0,0,0);
	m_real totalMass=0.0;

	Liegroup::Inertia I;

	// T_(-COM)*T_G*T(lCOM)*local_position_WT_COM
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform& bone=loader.VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mass();
		const Node* link = getLastNode(ibone);
		com+=link->globalFrame().toGlobalPos(bone.localCOM())*mass;
		totalMass+=mass;

		Liegroup::Inertia Ii(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		Ii=Ii.transform(link->globalFrame().inverse()); // inverse is there because of dAd transformation
		//Ii=Ii.transform(toGMBS(chain2.global(bone).inverse()));
		//printf("Ii: %f %f %f %f %f %f, %f\n", body->I._I[0],body->I._I[1],body->I._I[2],body->I._I[3],body->I._I[4],body->I._I[5],body->I._m);
		//printf("Ii: %f %f %f %f %f %f, %f\n", Ii._I[0],Ii._I[1],Ii._I[2],Ii._I[3],Ii._I[4],Ii._I[5],Ii._m);
		for (int i=0;i<6; i++) I._I[i]+=Ii._I[i];
		for (int i=0;i<3; i++) I._r[i]+=Ii._r[i];
		I._m+=Ii._m;
	}

	com/=totalMass;

	I=I.transform(transf(quater(1,0,0,0), com)); // inv(T_{com*-1}) == T_com
	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	inertia.setValues(10, I._I[0], I._I[1], I._I[2],  I._I[3],I._I[4],I._I[5],I._m, I._r[0], I._r[1], I._r[2]);
	return totalMass;
}

void IK_sdls::LoaderToTree::calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT, int chainRootBone, int chainTailBone)
{
	VRMLTransform* tail=&loader.VRMLbone(chainTailBone);
	VRMLTransform* newroot=&loader.VRMLbone(chainRootBone);
	VRMLTransform* i;
	vector3 gpos;

	i=tail;
	int nJoint = mTree.GetNumJoint();

	double totalmass=0.0;
	// compute jacobian
	JT.setSize(nJoint, 3);
	JT.setAllValue(0.0);
	matrixn temp;
	while (i->treeIndex()!=1)
	{
		gpos=globalFrame(i->treeIndex()).toGlobalPos(i->localCOM());
		calcJTchain(temp, *this, newroot->treeIndex(), i->treeIndex(), gpos);
		JT+=temp*i->mass();
		totalmass+=i->mass();
		i=(VRMLTransform*)(i->parent());
	}
	i=newroot;
	temp.setSize(nJoint,3);
	MatrixRmnView tempm(temp);
	while (i->treeIndex()!=1)
	{
		// reversed joints
		gpos=globalFrame(i->treeIndex()).toGlobalPos(i->localCOM());
		Node* n = getLastNode(newroot->treeIndex());
		Node* np = getLastNode(i->parent()->treeIndex());
		tempm.SetZero();
		n->calcSubtreeJacobian(np, tempm, 0, gpos);
		JT-=temp*i->mass();
		totalmass+=i->mass();
		i=(VRMLTransform*)(i->parent());
	}
	JT*=1.0/totalmass;
}

Liegroup::dse3 IK_sdls::LoaderToTree::calcMomentumCOMfromPose(const VRMLloader& loader, double delta_t, BoneForwardKinematics &chain1)
{
	::vector3 com(0,0,0);
	Liegroup::dse3 H(0,0,0,0,0,0);

	double totalMass=0.0;
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform& bone=loader.VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mass();
		com+=chain1.global(bone).toGlobalPos(bone.localCOM())*mass;
		totalMass+=mass;
		Node* n = getLastNode(ibone);
		//Liegroup::se3 V=transf_twist(chain1.global(bone), chain2.global(bone),delta_t);
		Liegroup::se3 V=Liegroup::twist_nonlinear(chain1.global(bone), n->globalFrame(),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		H+=(I*V).inv_dAd(chain1.global(bone));
	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	Liegroup::dse3 v;
	v.dAd(transf(quater(1,0,0,0), com), H);
	return v;
}
Liegroup::se3 IK_sdls::LoaderToTree::calcInverseInertiaTimesMomentumCOM(const VRMLloader& loader)
{
	vectorn I;
	double m=calcInertia(loader, I);
	vector3 r(I(7), I(8), I(9));
	matrixn I6;
	I6.setValues(6,6, 
			I(0),I(3), I(4), 0.0, -r.z, r.y,
			I(3),I(1), I(5), r.z, 0.0,  -r.x,
			I(4),I(5), I(2), -r.y, r.x, 0.0 ,
			0.0, r.z, -r.y, m,   0.0 , 0.0,
			-r.z,0.0,  r.x, 0.0,  m,  0.0,
			r.y, -r.x, 0.0, 0.0,  0.0,  m);

	matrixn invI;
	m::LUinvert(invI,I6);
	vectorn Hcom(6);
	vectorn deltaS(6);
	Liegroup::dse3 hcom=calcMomentumCOM(loader);
	memcpy(&Hcom(0), &hcom._m[0], sizeof(double)*6);
	deltaS.column().mult(invI, Hcom.column());
	Liegroup::se3 ds;
	ds.W()=deltaS.toVector3(0);
	ds.V()=deltaS.toVector3(3);
	return ds;
}
Liegroup::dse3 IK_sdls::LoaderToTree::calcMomentumCOMtoPose(const VRMLloader& loader, double delta_t, BoneForwardKinematics &chain2)
{
	::vector3 com(0,0,0);
	Liegroup::dse3 H(0,0,0,0,0,0);

	double totalMass=0.0;
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform& bone=loader.VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mass();
		Node* n = getLastNode(ibone);
		com+=n->globalFrame().toGlobalPos(bone.localCOM())*mass;
		totalMass+=mass;
		Liegroup::se3 V=Liegroup::twist_nonlinear(n->globalFrame(), chain2.global(bone),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		H+=(I*V).inv_dAd(n->globalFrame());
	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	Liegroup::dse3 v;
	v.dAd(transf(quater(1,0,0,0), com), H);
	return v;
}
Liegroup::dse3 IK_sdls::LoaderToTree::calcMomentumCOM(const VRMLloader& loader)
{
	::vector3 com(0,0,0);
	Liegroup::dse3 H(0,0,0,0,0,0);

	double totalMass=0.0;
//#define TEST_LINM
#ifdef TEST_LINM
	vector3 linm(0,0,0);
#endif
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform& bone=loader.VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mass();
		Node* n = getLastNode(ibone);
#ifdef TEST_LINM
		{
			vector3 v=getWorldVelocity(ibone);
			vector3 w=getWorldAngVel(ibone);
			linm+=mass*(v+w.cross(n->_global.rotation*bone.localCOM()));
		}
#endif
		com+=n->globalFrame().toGlobalPos(bone.localCOM())*mass;
		totalMass+=mass;
		Liegroup::se3 V(n->bodyAngVel(), n->bodyLinVel());
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		H+=(I*V).inv_dAd(n->globalFrame());
	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	Liegroup::dse3 v;
	v.dAd(transf(quater(1,0,0,0), com), H);

#ifdef TEST_LINM
	cout << v.M() <<","<<v.F()<<","<<linm<<endl;
	exit(0);
#endif
	return v;
}


vector3 IK_sdls::LoaderToTree::calcCOM(const VRMLloader& loader)
{
	double totalmass=0.0;
	vector3 COM(0.0,0.0,0.0);
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		IK_sdls::Node* n=getLastNode(ibone);
		VRMLTransform* i=&loader.VRMLbone(ibone);
		COM+=(n->globalFrame()*i->localCOM())*i->mass();
		totalmass+=i->mass();
	}
	COM*=1.0/totalmass;
	return COM;
}
void IK_sdls::LoaderToTree::calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT)
{
	int nJoint = mTree.GetNumJoint();

	double totalmass=0.0;
	// compute jacobian
	JT.setSize(nJoint, 3);
	JT.setAllValue(0.0);
	matrixn temp;
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform* i=&loader.VRMLbone(ibone);
		calcJacobianTransposeAt(temp, ibone, i->localCOM());
		JT+=temp*i->mass();
		totalmass+=i->mass();
	}
	JT*=1.0/totalmass;
}
void IK_sdls::LoaderToTree::updateCOM_grad_S_JT(vectorn & grad, const VRMLloader& loader, vector3 const& deltaS, double wx, double wy, double wz)
{
	int nJoint = mTree.GetNumJoint();
	double totalmass=0.0;
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform* i=&loader.VRMLbone(ibone);
		totalmass+=i->mass();
	}

	Msg::verify(grad.size()==nJoint,"grad size error");

	/*
	matrixn temp;
	double* g=&grad[0];
	for(int ibone=1; ibone<loader.numBone(); ibone++)
	{
		VRMLTransform* b=&loader.VRMLbone(ibone);
		calcJacobianTransposeAt(temp, ibone, b->localCOM());
		temp*=(b->mass()/totalmass);
		for(int i=0; i<nJoint; i++)
			g[i]+= (b->mass()/totalmass)*(deltaS%(temp.row(i).toVector3(0)));
	}
	*/
	matrixn temp;
	calcCOMjacobianTranspose(loader, temp);
	for(int i=0; i<nJoint; i++)
	{
		auto J=temp.row(i).toVector3(0);
		grad(i)+=wx* deltaS.x*J.x + wy*deltaS.y*J.y+wz*deltaS.z*J.z;
	}

}
void IK_sdls::LoaderToTree::poseToQ(vectorn const& v, vectorn& out) 
{
	int rdof=v.size(); out.setSize(rdof); 
	out.setVec3(0, v.toVector3(0)); 
	out[rdof-1]=v(3); 
	out.setVec3(3, v.toVector3(4)); 
	out.range(6, rdof-1)=v.range(7, rdof);
}
void IK_sdls::LoaderToTree::dposeToDQ(quater const& rootOri, vectorn const& v, vectorn& out) 
{
	int rdof=v.size(); out.setSize(rdof-1); 
	out.setVec3(3, rootOri*v.toVector3(0)); // linvel
	out.setVec3(0, rootOri*v.toVector3(4)); // angvel
	out.range(6, rdof-1)=v.range(7, rdof);
}

void IK_sdls::LoaderToTree::setQuaterQ(const double* q)
{
	if(!USE_EULER_ROOT)
		setQ(q);
	else
	{
		// not implmented yet
		Msg::error("not implemented yet");
		assert(false);
	}
}
void IK_sdls::LoaderToTree::setEulerQ(const double* q)
{
	if (USE_EULER_ROOT)
		setQ(q);
	else
	{
		IK_sdls::Node* n;

		int sj=0;
		n=getJoint(0);
		if (n->IsFreeJoint())
		{
			IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)n);

			fj->_local.translation.x=q[0];
			fj->_local.translation.y=q[1];
			fj->_local.translation.z=q[2];
			fj->_local.rotation.setRotation("ZXY", (double*)&q[3]);
			sj=6;
		}

		for(int j=sj,nj=numJoint(); j<nj; j++)
		{
			n=getJoint(j);
			n->SetTheta(q[mDQindex[j]]);
		}
		mTree.Compute();// updates effector positions too.
	}

}
// NOTE that this behaves differently for the euler-root and quaternion-root mode.
void IK_sdls::LoaderToTree::setQ(const double* q)
{
	IK_sdls::Node* n;

	int sj=0;
	n=getJoint(0);
	if (n->IsFreeJoint())
	{
		IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)n);
		fj->_local.translation.x=q[0];
		fj->_local.translation.y=q[1];
		fj->_local.translation.z=q[2];
		fj->_local.rotation.w=q[_nDOF]; // redundant coordinate
		fj->_local.rotation.x=q[3];
		fj->_local.rotation.y=q[4];
		fj->_local.rotation.z=q[5];
		sj=6;
	}

	for(int j=sj,nj=numJoint(); j<nj; j++)
	{
		n=getJoint(j);
		n->SetTheta(q[mDQindex[j]]);
	}
	mTree.Compute();// updates effector positions too.
}
void IK_sdls::LoaderToTree::getQ(double* q)
{
	for(int i=0; i<_nDOF; i++) q[i]=0;
	IK_sdls::Node* n;

	int sj=0;
	n=getJoint(0);
	if (n->IsFreeJoint())
	{
		IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)n);
		q[0]= fj->_local.translation.x;
		q[1]= fj->_local.translation.y;
		q[2]= fj->_local.translation.z;
		q[_nDOF]=fj->_local.rotation.w; // redundant coordinate;
		q[3]= fj->_local.rotation.x;
		q[4]= fj->_local.rotation.y;
		q[5]= fj->_local.rotation.z;
		sj=6;
	}

	for(int j=sj,nj=numJoint(); j<nj; j++)
	{
		n=getJoint(j);
		q[mDQindex[j]]=n->GetTheta();
	}
}
void IK_sdls::LoaderToTree::setDQ(const double* dq)
{
	IK_sdls::Node* n;

	int sj=0;
	n=getJoint(0);
	if (n->IsFreeJoint())
	{
		IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)n);
		vector3 v,w;
		memcpy(&w.x, dq, sizeof(double)*3);
		memcpy(&v.x, dq+3, sizeof(double)*3);
		quater& R=fj->_local.rotation;
		quater invR;
		invR.inverse(R);
		fj->SetJointVel(invR*v, invR*w);
		sj=6;
	}

	for(int j=sj,nj=numJoint(); j<nj; j++)
	{
		n=getJoint(j);
		n->SetDTheta(dq[mDQindex[j]]);
	}
	mTree.ComputeDS(mTree.GetRoot()); // update velocity
}
void IK_sdls::LoaderToTree::getDQ(double* dq)
{
	for(int i=0; i<_nDOF; i++) dq[i]=0;
	IK_sdls::Node* n;

	int sj=0;
	n=getJoint(0);
	if (n->IsFreeJoint())
	{
		IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)n);
		vector3 v, w;
		quater& R=fj->_local.rotation;
		fj->GetJointVel(v, w);
		v.rotate(R);
		w.rotate(R);
		memcpy(dq, &w.x, sizeof(double)*3);
		memcpy(dq+3, &v.x, sizeof(double)*3);
		sj=6;
	}

	for(int j=sj,nj=numJoint(); j<nj; j++)
	{
		n=getJoint(j);
		dq[mDQindex[j]]=n->GetDTheta();
	}
}

void IK_sdls::LoaderToTree::setPoseDOF(IK_sdls::Tree & mTree, MotionDOFinfo const& mDofInfo, vectorn const& pose, std::vector<IK_sdls::NodeWrap>& mNode)
{
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				m_real* trans=&pose[mDofInfo.startT(treeIndex)];
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
					mNode[i].node[c]->SetTheta(trans[c]);

				rotStartIndex+=channels.length();
			}
			//else { // root의 경우 둘다 가지고 있을 수 있음.

				if(mDofInfo.hasQuaternion(treeIndex))
				{
					if(mNode[i].node[0]->IsBallJoint())
					{
						mNode[i].node[0]->_local.rotation=pose.toQuater(mDofInfo.startR(treeIndex));
					}
					else
					{
						m_real euler[3];
						TString channels=bone->getRotationalChannels();
						//printf("channels %s\n", channels.ptr());
						pose.toQuater(mDofInfo.startR(treeIndex)).getRotation(channels, euler);

						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
				}
				else
				{
					m_real* euler=&pose[mDofInfo.startR(treeIndex)];
					TString channels=bone->getRotationalChannels();

					if(channels.length()>0 && channels[0]=='A')
					{
						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
					else
					{
						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
				}
			//}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			((IK_sdls::FreeJoint*)mNode[i].node[0])->setTheta(MotionDOF::rootTransformation(pose));
		}
	}

}
void IK_sdls::LoaderToTree::setVelocity(MotionDOFinfo const& mDofInfo, vectorn const& pose)
{
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				m_real* trans=&pose[mDofInfo.startT(treeIndex)];
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
					mNode[i].node[c]->SetDTheta(trans[c]);

				rotStartIndex+=channels.length();
			}
			//else {

				if(mDofInfo.hasQuaternion(treeIndex))
				{
					ASSERT(mNode[i].node[0]->IsBallJoint());
					IK_sdls::BallJoint* bj=((IK_sdls::BallJoint*)mNode[i].node[0]);
					bj->SetJointVel3DOF(pose.toVector3(mDofInfo.startR(treeIndex)+1));
				}
				else
				{
					m_real* euler=&pose[mDofInfo.startR(treeIndex)];
					TString channels=bone->getRotationalChannels();

					for(int c=0; c<channels.length(); c++)
						mNode[i].node[rotStartIndex+c]->SetDTheta(euler[c]);
				}
			//}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			((IK_sdls::FreeJoint*)mNode[i].node[0])->SetJointVel(
				pose.toVector3(0), pose.toVector3(4));
		}
	}
	mTree.ComputeDS(mTree.GetRoot());
}
static void _getDQ(IK_sdls::Node* node, double* dq)
{
	if(!node) return;
	ASSERT(node->IsJoint());
	dq[node->GetJointNum()]=node->GetDTheta();
	_getDQ(node->left, dq);
	_getDQ(node->right, dq);
}
void IK_sdls::LoaderToTree::getDTheta(double* dq)
{
	//if(mNode[i].node[0]->IsFreeJoint())
	if(mTree.GetRoot()->IsFreeJoint())
	{
		vector3 lin_vel;
		vector3 ang_vel;
		mTree.GetRoot()->GetJointVel(lin_vel, ang_vel);
		memcpy(&dq[0], &ang_vel, sizeof(double)*3);
		memcpy(&dq[3], &lin_vel, sizeof(double)*3);
		_getDQ(mTree.GetRoot()->left, dq);
		_getDQ(mTree.GetRoot()->right, dq);
	}
	else
		_getDQ(mTree.GetRoot(), dq);
}
double IK_sdls::LoaderToTree::computeConstraintError(double verticalCoef)
{
	int nEffector = mTree.GetNumEffector();
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 * nEffector;
	int nCol = nJoint;

	// compute jacobian
	IK_sdls::Effector* n;

	VectorRn dS;			// delta s
	dS.SetLength(3*nEffector);

	double fx = 0.0;

	for(int ii=0; ii<mTree.effectors.size(); ii++)
	{
		n = mTree.effectors[ii];
		int i = n->GetEffectorNum();
		assert(i==ii);
		// Compute the delta S value (differences from end effectors to target positions.
		n->computeDeltaS(dS, &mTree);
		fx+=dS.GetTriple(ii).squaredLength();
	}
	if(verticalCoef>0.001)
	{
		for(int ii=0; ii<mTree.effectors.size(); ii++)
		{
			n = mTree.effectors[ii];
			double dSy=dS.GetTriple(ii).y;
			fx+=verticalCoef+SQR(dSy);
		}
	}
	return fx;
}
void IK_sdls::LoaderToTree::computeConstraintErrorGradient(double* g, double verticalCoef)
{
	int nEffector = mTree.GetNumEffector();
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 * nEffector;
	int nCol = nJoint;
	int N=nCol;

	// compute jacobian
	IK_sdls::Effector* n;

	VectorRn dS;			// delta s
	dS.SetLength(3*nEffector);

	MatrixRmn Jend;		// Jacobian matrix based on end effector positions
	Jend.SetSize(nRow, nCol);
	Jend.SetZero();
	for(int ii=0; ii<mTree.effectors.size(); ii++)
	{
		n = mTree.effectors[ii];
		int i = n->GetEffectorNum();
		assert(i==ii);
		n->computeDeltaS(dS, &mTree);
		n->calcJacobian(&mTree, Jend, i, n->GetS());
	}
	for(int i=0; i<N; i++){
		g[i]=0.0;
	}
	/*  how to compute the gradient

		J*dq=dp

		-- Q=(q0, q1, ..., qn)

		-- absolute position
		objective function f =             
		sum_i  |dS_i|^2 
		= sum_i |X(Q)-X0|^2
		= sum_i (X(Q)-X0)' (X(Q)-X0)      
		= sum_i ((x(Q)-x0)^2 +(y(Q)-y0)^2+(z(Q)-z0)^2)  

		Jacobian= dX/dQ
		= { dx/dQ
		dy/dQ
		dz/dQ}

		object gradient = sum_i df_i/dQ

		If we look at only one term:
		df/dQ = d(     X(Q)-X0)' (X(Q)-X0)        )/dQ
		= 2 (X(Q)-x0)'dX/dQ 		-- using proposition 2 of matrixCalculus.pdf.
		-- see MatrixCalculus.pdf (in the references/kinematic_motion folder.)

		-- plane distance constraint
		objective function f =
		sum_i | X(Q)' * normal + d0 |^2

		= sum_i  (X(Q)' *normal +d0)*(X(Q)' *normal +d0)
		= sum_i  ( X(Q)' * normal * normal' * X(Q) + 2*d0*X(Q)'*normal +d0*d0)

		if we look at only one term f_i = X(Q)' * normal * normal' * X(Q) + -2*d0*X(Q)'*normal +d0*d0 :
		df_i/dQ=2 X(Q)'*(normal*normal') *dX/dQ+   -- using proposition 13 of matrixCalculus.pdf (normal*normal' == A == A')
		+ 2*d0*normal'* dX/dQ           -- using proposition 57

		-- relative position

		objective function = sum_i  |dS_i|^2 
		= sum_i ((x1(q)-x2(q))^2 +(y1(q)-y2(q))^2+(z1(q)-z2(q)^2)

		Jacobian1= dX1/dQ={ dx1/dq
		dy1/dq
		dz1/dq}


		object gradient = sum_i df_i/dQ

		If we look at only one term:
		df/dQ = d(     X1(Q)-X2(Q))' (X1(Q)-X2(Q)) )/dQ
		= 2 (X1(Q)-X2(Q))'(dX1/dQ-dX2/dQ)          -- using proposition 2 of matrixCalculus.pdf.
		*/

	for(int i=0; i<N; i++){
		for(int ii=0; ii<mTree.effectors.size(); ii++)
		{
			n=mTree.effectors[ii];
			VectorR3 j;
			Jend.GetTriple(ii, i, &j);
			// absolute position
			g[i]-= 2.0*dS.GetTriple(ii)%j;
		}	
	}
	if(verticalCoef>0.001)
	{
		for(int i=0; i<N; i++){
			for(int ii=0; ii<mTree.effectors.size(); ii++)
			{
				n=mTree.effectors[ii];
				VectorR3 j;
				Jend.GetTriple(ii, i, &j);
				{
					// absolute or relative position
					g[i]-= 2.0*dS.GetTriple(ii).y*j.y;
				}
			}	
		}
	}

}
void IK_sdls::LoaderToTree::integrate(MotionDOFinfo const& mDofInfo, vectorn const& dtheta, double timeStep)
{
	// this function does not call setVelocity. If global velocities are necessary, call it yourself.
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				m_real* trans=&dtheta[mDofInfo.startT(treeIndex)];
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
					((SlideJoint*)mNode[i].node[c])->AddToTheta(trans[c]*timeStep);

				rotStartIndex+=channels.length();
			}
			if(mDofInfo.hasQuaternion(treeIndex))
			{
				auto* n=((IK_sdls::BallJoint*)mNode[i].node[0]);
				n->Integrate(dtheta.toVector3(mDofInfo.startR(treeIndex)+1), timeStep);
			}
			else
			{
				m_real* euler=&dtheta[mDofInfo.startR(treeIndex)];
				TString channels=bone->getRotationalChannels();

				for(int c=0; c<channels.length(); c++)
					((HingeJoint*)mNode[i].node[rotStartIndex+c])->AddToTheta(euler[c]*timeStep);
			}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			((IK_sdls::FreeJoint*)mNode[i].node[0])->Integrate(dtheta, timeStep);
		}
	}
	
	// Update the positions and rotation axes of all joints/effectors
	mTree.Compute(); 
}

void IK_sdls::LoaderToTree::integrate(MotionDOFinfo const& mDofInfo, double timeStep)
{
	// this function does not call setVelocity. If global velocities are necessary, call it yourself.
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
				{
					auto* n=((SlideJoint*)mNode[i].node[c]);
					n->AddToTheta(n->GetDTheta()*timeStep);
				}

				rotStartIndex+=channels.length();
			}
			if(mDofInfo.hasQuaternion(treeIndex))
			{
				ASSERT(getNode(treeIndex).node[0]->IsBallJoint());
				auto* n=((IK_sdls::BallJoint*)mNode[i].node[0]);

				//cout << "before" <<n->_local.rotation<<endl;
				n->Integrate(n->GetJointVel3DOF(), timeStep);
				//cout << "after" <<n->_local.rotation<<endl;
			}
			else
			{
				TString channels=bone->getRotationalChannels();

				for(int c=0; c<channels.length(); c++)
				{
					auto* n=((HingeJoint*)mNode[i].node[rotStartIndex+c]);
					n->AddToTheta(n->GetDTheta()*timeStep);
				}
			}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			vectorn dtheta(7);
			vector3 lin_vel, ang_vel;
			auto* n=((IK_sdls::FreeJoint*)mNode[i].node[0]);

			n->GetJointVel(lin_vel, ang_vel);
			dtheta.setVec3(0, lin_vel);
			dtheta.setVec3(4, ang_vel);
			n->Integrate(dtheta, timeStep);
		}
	}
	
	// Update the positions and rotation axes of all joints/effectors
	mTree.Compute(); 
}

void IK_sdls::LoaderToTree::calcEffectorJacobianTranspose(matrixn& J)
{
	int nEffector = mTree.GetNumEffector();
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 * nEffector;
	int nCol = nJoint;
	// compute jacobian
	IK_sdls::Effector* n;

	J.setSize(nCol, nRow);
	assert(&J[1][0]-&J[0][0]==nRow);
	MatrixRmnView Jend(J);

	for(int ii=0; ii<mTree.effectors.size(); ii++)
	{
		n = mTree.effectors[ii];
		int i = n->GetEffectorNum();
		assert(i==ii);
		n->calcJacobian(&mTree, Jend, i, n->GetS());
	}
}

void IK_sdls::LoaderToTree::calcJacobianTransposeAt(matrixn& J, int ibone, vector3 const& localpos)
{
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 ;
	int nCol = nJoint;
	// compute jacobian
	IK_sdls::Node* n;

	J.setSize(nCol, nRow);
	assert(&J[1][0]-&J[0][0]==nRow);
	MatrixRmnView Jend(J);

	n = getLastNode(ibone);
	vector3 target=n->globalFrame()*localpos;
	n->calcJacobian(Jend, 0, target);

}
void IK_sdls::LoaderToTree::updateGrad_S_JT(double* g,vector3 const& deltaS, int ibone, vector3 const& localpos)
{
	int nJoint = mTree.GetNumJoint();
	// compute jacobian
	IK_sdls::Node* m;

	m = getLastNode(ibone);
	vector3 target=m->globalFrame()*localpos;
	while ( m ) {
		int j = m->GetJointNum();
		if ( !m->IsFrozen() ) {
			m->_updateGrad_S_JT(g, deltaS, target);
		}
		m = m->realparent;
	}
}
void IK_sdls::LoaderToTree::updateGrad_S_JT_residual(double* g,vector3 const& deltaS_lpos, int ibone)
{
	int nJoint = mTree.GetNumJoint();
	// compute jacobian
	IK_sdls::Node* m;

	m = getLastNode(ibone);
	while ( m ) {
		int j = m->GetJointNum();
		if ( !m->IsFrozen() ) {
			m->_updateGrad_S_JT_residual(g, deltaS_lpos);
		}
		m = m->realparent;
	}
}

void IK_sdls::LoaderToTree::getJacobianSparsity(boolN& hasValue, int ibone)
{
	int nJoint = mTree.GetNumJoint();
	int nCol = nJoint;
	// compute jacobian
	IK_sdls::Node* n;

	hasValue.resize(nCol);

	n = getLastNode(ibone);
	n->getJacobianSparsity(hasValue);
}
void IK_sdls::LoaderToTree::calcRotJacobianTranspose(matrixn& J, int ibone)
{
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 ;
	int nCol = nJoint;
	// compute jacobian
	IK_sdls::Node* n;

	J.setSize(nCol, nRow);
	assert(&J[1][0]-&J[0][0]==nRow);
	MatrixRmnView Jend(J);

	n = getLastNode(ibone);
	n->calcRotJacobian(Jend, 0);
}

void IK_sdls::LoaderToTree::calcGlobalJacobianTransposeAt(matrixn& J, int ibone, vector3 const& localpos)
{
	int nJoint = mTree.GetNumJoint();
	int nRow = 3 ;
	int nCol = nJoint;
	// compute jacobian
	IK_sdls::Node* n;

	// J is actually jacobian-transpose;
	J.setSize(nCol, nRow);
	assert(&J[1][0]-&J[0][0]==nRow);
	MatrixRmnView Jend(J);

	n = getLastNode(ibone);
	vector3 target=n->globalFrame()*localpos;
	n->calcJacobian(Jend, 0, target);


	// overwrite the root part.
	vector3 axis0(0,0,0);
	vector3 lin;
	IK_sdls::Node* baseLink=getLastNode(1);
	for(int i=0; i<3; i++)
	{
		//axis0=baseLink->R* axis; // gmbs convention (local axis)
		axis0[i]=1.0; // sml convention (global axis)
		lin.cross(axis0, target-baseLink->GetS());

		Jend.SetTriple( 0, i+3, axis0);
		Jend.SetTriple( 0, i, lin); 
		axis0[i]=0.0;
	}
}

void IK_sdls::LoaderToTree::findAxes(boolN& hasValue, vector3 const& axis)
{
	hasValue.resize(numJoint());
	hasValue.setAllValue(false);
	for(int i=0, ni=numJoint(); i<ni; i++)
	{
		auto* j=getJoint(i);
		if (j->IsHingeJoint())
		{
			auto a=((IK_sdls::HingeJoint*)j)->GetW();
			if (ABS(a%axis)>0.86) //60 degree
				hasValue.set(i, true);
		}
	}
}
void IK_sdls::LoaderToTree::getVelocity(MotionDOFinfo const& mDofInfo, vectorn & pose)
{
	pose.setSize(mDofInfo.numDOF());
	mTree.ComputeDQfromDS(mTree.GetRoot());
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				m_real* trans=&pose[mDofInfo.startT(treeIndex)];
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
					trans[c]=mNode[i].node[c]->GetDTheta();

				rotStartIndex+=channels.length();
			}
			//else {
				if(mDofInfo.hasQuaternion(treeIndex))
				{
					Msg::error("not implemented yet");
				}
				else
				{
					m_real* euler=&pose[mDofInfo.startR(treeIndex)];
					TString channels=bone->getRotationalChannels();

					for(int c=0; c<channels.length(); c++)
						euler[c]=mNode[i].node[rotStartIndex+c]->GetDTheta();
				}
			//}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			vector3 v,w;
			((IK_sdls::FreeJoint*)mNode[i].node[0])->GetJointVel(v, w);
			pose.setVec3(0, v);
			pose.setVec3(4, w);
		}
	}
}

void IK_sdls::LoaderToTree::getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn& output, std::vector<IK_sdls::NodeWrap>const& mNode)
{
	output.resize(mDofInfo.numDOF());
	// set output
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{

				TString channels=bone->getTranslationalChannels();
				int startT=mDofInfo.startT(treeIndex);
				for(int c=0; c<channels.length(); c++)
					output[startT+c]=mNode[i].node[c]->GetTheta();
				rotStartIndex+=channels.length();
			}
			//else {

				TString channels=bone->getRotationalChannels();

				if(mDofInfo.hasQuaternion(treeIndex))
				{
					if(mNode[i].node[0]->IsBallJoint())
					{
						output.range(mDofInfo.startR(treeIndex), mDofInfo.endR(treeIndex)).assign(mNode[i].node[0]->_local.rotation);
					}
					else
					{
						quater q;
						m_real euler[3];
						for(int c=0; c<channels.length(); c++)
							euler[c]=mNode[i].node[c+rotStartIndex]->GetTheta();
						q.setRotation(channels, euler);
						output.range(mDofInfo.startR(treeIndex),
								mDofInfo.endR(treeIndex)).assign(q);
					}
				}
				else
				{
					int startR=mDofInfo.startR(treeIndex);
					if(channels.length()>0 && channels[0]=='A')
					{
						for(int c=0; c<channels.length(); c++)
							output[startR+c]=mNode[i].node[c+rotStartIndex]->GetTheta();
					}
					else
					{
						for(int c=0; c<channels.length(); c++)
							output[startR+c]=mNode[i].node[c+rotStartIndex]->GetTheta();
					}
				}
			//}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			const transf& theta=((IK_sdls::FreeJoint*)mNode[i].node[0])->getTheta();
			MotionDOF::setRootTransformation(output, theta);
		}
	}
}
#define TEST_RELATIVE 0
IK_sdls ::LoaderToTree:: LoaderToTree(MotionLoader& skeleton, bool useEulerRoot, bool useFixedRootPos)
{
	// empty
	std::vector<MotionUtil::Effector> effectors;
	std::vector<MotionUtil::RelativeConstraint> constraints;
	_init(skeleton, effectors, constraints, useEulerRoot, useFixedRootPos);
}

IK_sdls ::LoaderToTree:: LoaderToTree(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints,
				bool useEulerRoot, 
				bool useFixedRootPos
		)
{
	_init(skeleton, mEffectors, constraints, useEulerRoot, useFixedRootPos);
}
IK_sdls ::LoaderToTree:: LoaderToTree()
{
}
void IK_sdls::LoaderToTree::_init(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints,
				bool useEulerRoot, 
				bool useFixedRootPos
		)

{
	USE_EULER_ROOT=useEulerRoot;
	USE_FIXED_ROOTPOS=useFixedRootPos;
	skeleton.UpdateInitialBone();
	_originalLoader=&skeleton;

	/*
	   아래 주석 처리된 코드와 유사한 내용이 리커시브하게 수행된다.
	// pelvis.
	bone=&skeleton.getBoneByVoca(MotionLoader::HIPS);

	mNode.push_back(NodeWrap());
	mNode.back().createNodes(bone, "ZXY");
	mTree.InsertRoot(mNode.back().node[0]);
	mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
	mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
	parent=mNode.back().node[2];

	// righthip
	bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTHIP);
	mNode.push_back(NodeWrap());
	mNode.back().createNodes(bone, "ZXY");
	mTree.InsertLeftChild(parent, mNode.back().node[0]);
	mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
	mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
	parent=mNode.back().node[2];

	// knee
	bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTKNEE);
	mNode.push_back(NodeWrap());
	mNode.back().createNodes(bone, "ZXY");
	mTree.InsertLeftChild(parent, mNode.back().node[0]);
	mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
	mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
	parent=mNode.back().node[2];

	// ankle
	bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTANKLE);

	mNode.push_back(NodeWrap());
	mNode.back().createNodes(bone, "EFFECTOR");
	mTree.InsertLeftChild(parent, mNode.back().node[0]);

	*/

	// Effector랑 관련이 없는 bone은 IK 트리만들때 제외하기 위해, 먼저 각 본이 effector와 관련이 되있는지 아닌지를 mEffectorAttached에 저장한다.
	mEffectorAttached.resize(skeleton.GetNumTreeNode());
	mBoneToNode.setSize(skeleton.GetNumTreeNode());
	mBoneToNode.setAllValue(-1);



	if (mEffectors.size()==0)
	{
		mEffectorAttached.setAllValue(true);
		for(int i=1; i<skeleton.numBone(); i++)
		{
			Bone* bone=&skeleton.bone(i);
			// every node except SITE nodes.
			if (bone->rotJointIndex()==-1 && bone->child()==NULL)
			{
				//printf("except %s\n", bone->NameId);
				mEffectorAttached.clearAt(i);
			}
		}
	}
	else
	{
		mEffectorAttached.clearAll();

		for(int i=0; i<mEffectors.size(); i++)
		{
			for(Bone* bone=mEffectors[i].bone; bone!=NULL; bone=bone->parent())
				mEffectorAttached.setAt(bone->GetIndex());
		}

		for(int i=0; i<constraints.size(); i++)
		{
			ASSERT(constraints[i].eType==MotionUtil::RelativeConstraint::RELATIVE_POSITION);
			for(Bone* bone=constraints[i].bone1; bone!=NULL; bone=bone->parent())
				mEffectorAttached.setAt(bone->GetIndex());
			for(Bone* bone=constraints[i].bone2; bone!=NULL; bone=bone->parent())
				mEffectorAttached.setAt(bone->GetIndex());
		}

#if TEST_RELATIVE
		// test relative constraint
		bone1=&mSkeleton.getBoneByName("lhand");
		bone2=&mSkeleton.getBoneByName("lfoot");
		for(Bone* bone=bone1; bone!=NULL; bone=bone->parent())
			mEffectorAttached.setAt(bone->GetIndex());
		for(Bone* bone=bone2; bone!=NULL; bone=bone->parent())
			mEffectorAttached.setAt(bone->GetIndex());
#endif
	}

	Bone* bone;
	vector3 zero (0,0,0);

	copyTree(&skeleton.getBoneByRotJointIndex(0), NULL);



	_init_part2(skeleton, mEffectors, constraints);
#if TEST_RELATIVE
	// test relative constraint
	{
		printf("here\n");
		Bone *bone1, *bone2;
		bone1=&mSkeleton.getBoneByName("lhand");
		bone2=&mSkeleton.getBoneByName("lfoot");
		mEffectorNode.push_back(IK_sdls::NodeWrap());
		mEffectorNode.back().createRelativeConstraint(bone1, bone2, vector3(0.000000,-0.053740,0.111624));
		mTree.InsertRelativeConstraint(
				mNode[mBoneToNode[bone1->GetIndex()]].back(), 
				mNode[mBoneToNode[bone2->GetIndex()]].back(), 
				(IK_sdls::RelativeConstraint*)mEffectorNode.back().node[0]);
	}
#endif

	mTree.Init();
	//mTree.Compute();

	_nDOF=skeleton.dofInfo.numActualDOF();
	mDQindex.setSize(mTree.GetNumJoint());
	mDQindex.setAllValue(-1);
	mNodeTraverse.resize(mTree.GetNumJoint());
	for(int i=0; i<mNodeTraverse.size(); i++)
		mNodeTraverse[i]=NULL;
	for(int bi=1, ni=skeleton.numBone(); bi<ni; bi++)
	{
		int i=mBoneToNode[bi];
		if(i!=-1){
			IK_sdls::NodeWrap& n=mNode[i];
			if (n.node[0]->IsFreeJoint())
			{
				mDQindex[0]=0;
				mDQindex[1]=1;
				mDQindex[2]=2;
				mDQindex[3]=3;
				mDQindex[4]=4;
				mDQindex[5]=5;
				mNodeTraverse[0]=n.node[0];
			}
			else {
				int sq=skeleton.dofInfo.startDQ(bi);

				if(n.node[0]->IsBallJoint())
				{
					for(int j=0; j< 3; j++)
						mDQindex[n.node[0]->GetJointNum()+j]=sq+j;
					mNodeTraverse[n.node[0]->GetJointNum()]=n.node[0];
				}
				else
				{
					for(int j=0; j< n.axes.length();j++)
					{
						mDQindex[n.node[j]->GetJointNum()]=sq+j;
						mNodeTraverse[n.node[j]->GetJointNum()]=n.node[j];
					}
				}
			}
		}
	}
#ifdef _DEBUG
	mTree.Print();
	compareTrees(vector3(0,0,0));
#endif
}

void IK_sdls ::LoaderToTree:: _init_part2(MotionLoader const& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints)
{
	for(int i=0; i<mEffectors.size(); i++)
	{
		Bone* bone=mEffectors[i].bone;

		mEffectorNode.push_back(IK_sdls::NodeWrap());
		mEffectorNode.back().createEffector(bone, mEffectors[i].localpos);

		mTree.InsertEffector(mNode[mBoneToNode[bone->GetIndex()]].back(), (IK_sdls::Effector*)mEffectorNode.back().node[0]);
	}
	/*
	_nonEffector_constraints.resize(0);
	for(int i=0; i<constraints.size(); i++)
	{
		MotionUtil::RelativeConstraint& c=constraints[i];
		if(c.eType==MotionUtil::RelativeConstraint::PLANE_DISTANCE ||
		c.eType==MotionUtil::RelativeConstraint::HALF_SPACE)
		{
			mEffectorNode.push_back(IK_sdls::NodeWrap());
			mEffectorNode.back().createEffector(c.bone1, c.localpos1);
			IK_sdls::Effector* e=(IK_sdls::Effector* )mEffectorNode.back().node[0];
			e->constraint=&c;
			mTree.InsertEffector(mNode[mBoneToNode[c.bone1->GetIndex()]].back(), e);
		}
		else if(c.eType==MotionUtil::RelativeConstraint::RELATIVE_POSITION)
		{
			mEffectorNode.push_back(IK_sdls::NodeWrap());
			mEffectorNode.back().createRelativeConstraint(c.bone1, c.bone2, c.localpos1);
			mTree.InsertRelativeConstraint(
					mNode[mBoneToNode[c.bone1->GetIndex()]].back(), 
					mNode[mBoneToNode[c.bone2->GetIndex()]].back(), 
					(IK_sdls::RelativeConstraint*)mEffectorNode.back().node[0]);
		}
		else 
		{
		{
			_nonEffector_constraints.push_back(&c);
		}
	}
	*/
}
void IK_sdls ::LoaderToTree::copyTree(Bone* bone, 
		IK_sdls::Node* parent)
{
	if(bone->numChannels()==0)
	{
		Msg::verify(!mEffectorAttached[bone->GetIndex()], "fixed joint??? unsupported skeleton structures. use removeFixedJoints");
		return;
	}
	else if(mEffectorAttached[bone->GetIndex()])
	{
		if(parent==NULL)
		{
			mNode.push_back(IK_sdls::NodeWrap());

			if(USE_EULER_ROOT || bone->numChannels()<=3){
				TString channels;
				if(USE_FIXED_ROOTPOS){
					// rotational joints only.
					channels=bone->getRotationalChannels();
					mNode.back().createNodes(bone, channels);
				}
				else {
					TString channels1=bone->getRotationalChannels();
					TString channels2=bone->getTranslationalChannels();
					channels=channels2+channels1;
					mNode.back().createNodes(bone, channels2, channels1);
				}

				mTree.InsertRoot(mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
			}
			else
			{
				// quaternion
				mNode.back().createFreeNode(bone);
				mTree.InsertRoot(mNode.back().node[0]);
			}
			mBoneToNode[bone->GetIndex()]=mNode.size()-1;
		}
		else
		{
			mNode.push_back(IK_sdls::NodeWrap());
			if(bone->getSkeleton().dofInfo.hasQuaternion(bone->treeIndex()))
			{
				mNode.back().createBallJoint(bone);
				mTree.InsertChild_automatic(parent, mNode.back().node[0]);
			}
			else
			{
				TString channels1=bone->getRotationalChannels();
				TString channels2=bone->getTranslationalChannels();
				TString channels=channels2+channels1;
				mNode.back().createNodes(bone, channels2, channels1);
				mTree.InsertChild_automatic(parent, mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
			}

			mBoneToNode[bone->GetIndex()]=mNode.size()-1;
		}


		if(bone->m_pChildHead)
		{
			copyTree((Bone*)bone->m_pChildHead, mNode[mBoneToNode[bone->GetIndex()]].back());
		}
	}

	if(bone->m_pSibling)
	{
		copyTree((Bone*)bone->m_pSibling, mNode[mBoneToNode[bone->m_pParent->GetIndex()]].back());
	}
}
void IK_sdls::LoaderToTree::compareTrees(vector3 trans)
	{
		for(int i=0; i<mNode.size(); i++)
		{
			printf("bone %s (%d)\n", mNode[i].bone->NameId, i);
			
			if(mNode[i].node[0]->IsJoint())
			{
				printf("node %d:%d:%s\n", mNode[i].node[0]->GetJointNum(), mNode[i].node[0]->GetParentJointNum(), (mNode[i].node[0]->GetS()+trans).output().c_str());
				printf("NODE %d:%d:%s\n", mNode[i].back()->GetJointNum(), mNode[i].back()->GetParentJointNum(), (mNode[i].back()->GetS()+trans).output().c_str());
			}
			else
			{
				printf("efct %d:%d:%s\n", mNode[i].node[0]->GetEffectorNum(),mNode[i].node[0]->GetParentJointNum(), (mNode[i].node[0]->GetS()+trans).output().c_str());
			}
			printf("bone %d:%s\n", mNode[i].bone->GetIndex(), mNode[i].bone->getTranslation().output().c_str());
		}
	}

IK_sdls::LoaderToTree_selected::LoaderToTree_selected(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos)
{
	_init( skeleton, mEffectors,  selectedJoints, useEulerRoot, useFixedRootPos);
}

void IK_sdls::LoaderToTree_selected::_init_part1(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos)
{
	USE_EULER_ROOT=useEulerRoot;
	USE_FIXED_ROOTPOS=useFixedRootPos;

	mSelectedBone.resize(skeleton.numBone());
	mSelectedBone.clearAll();
	for(int i=0; i<selectedJoints.size() ; i++)
		mSelectedBone.setAt(selectedJoints[i]);
	mSelectedJointIndexToTreeIndex.resize(skeleton.dofInfo.numDOF()); // conservative.
	mSelectedJointIndexToTreeIndex.setAllValue(-1);

	skeleton.UpdateInitialBone();

	// Effector랑 관련이 없는 bone은 IK 트리만들때 제외하기 위해, 먼저 각 본이 effector와 관련이 되있는지 아닌지를 mEffectorAttached에 저장한다.
	mEffectorAttached.resize(skeleton.GetNumTreeNode());
	mBoneToNode.setSize(skeleton.GetNumTreeNode());
	mBoneToNode.setAllValue(-1);

	if (mEffectors.size()==0)
	{
		mEffectorAttached.setAllValue(true);
	}
	else
	{
		mEffectorAttached.clearAll();

		for(int i=0; i<mEffectors.size(); i++)
		{
			for(Bone* bone=mEffectors[i].bone; bone!=NULL; bone=bone->parent())
				mEffectorAttached.setAt(bone->GetIndex());
		}
	}
	Bone* bone;
	vector3 zero (0,0,0);

	copyTree(&skeleton.getBoneByRotJointIndex(0), NULL);
}

void IK_sdls::LoaderToTree_selected::_init_part2(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors)
{
	Bone* bone;
	for(int i=0; i<mEffectors.size(); i++)
	{
		bone=mEffectors[i].bone;

		mEffectorNode.push_back(IK_sdls::NodeWrap());
		mEffectorNode.back().createEffector(bone, mEffectors[i].localpos);

		mTree.InsertEffector(mNode[mBoneToNode[bone->GetIndex()]].back(), (IK_sdls::Effector*)mEffectorNode.back().node[0]);
	}
	// update local pos
	//
	for(int i=0; i<mNode.size(); i++)
	{
		m_real euler[3];
		if(mNode[i].node[0]->IsJoint())
		{
			IK_sdls::Node* node=mNode[i].node[0];
			int treeIndex=mSelectedJointIndexToTreeIndex[node->GetJointNum()];
			if(treeIndex==1)
			{
				node->_local.translation=vector3(0,0,0);
			}
			else
			{
			  //printf("%d %d %d\n", i, node->GetJointNum(), node->GetParentJointNum());
			  //printf("%s\n", mSelectedJointIndexToTreeIndex.output().ptr());
				int parentTreeIndex=mSelectedJointIndexToTreeIndex[node->GetParentJointNum()];
				node->_local.translation=skeleton.bone(parentTreeIndex).getFrame().toLocalPos(skeleton.bone(treeIndex).getFrame().translation);
			}
		}
	}
	for(int i=0; i<mEffectors.size(); i++)
	{
		IK_sdls::Node* node=mEffectorNode[i].node[0];
		Bone& bone=skeleton.bone(mSelectedJointIndexToTreeIndex[node->GetParentJointNum()]);

		node->_local.translation=bone.getFrame().toLocalPos(mEffectors[i].bone->getFrame().toGlobalPos(mEffectors[i].localpos));
	}


	mTree.Init();
	mTree.Compute();
//#define _DEBUG
#ifdef _DEBUG
	printf("%d %d\n", (int)mNode.size(), skeleton.dofInfo.numBone());
	mTree.Print();
	for(int i=0; i<mTree.GetNumJoint() ; i++)
	{
		if(mSelectedJointIndexToTreeIndex[i]!=-1)
			printf("%d %s\n", i, skeleton.bone(mSelectedJointIndexToTreeIndex[i]).NameId);
	}
#endif
}
void IK_sdls::LoaderToTree_selected::_init(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos)
{
	_init_part1(skeleton, mEffectors, selectedJoints, useEulerRoot, useFixedRootPos);
	_init_part2(skeleton, mEffectors);
}

void IK_sdls::LoaderToTree_selected::copyTree(Bone* bone, IK_sdls::Node* parent)
{

	if(bone->numChannels()==0)
	{
		Msg::verify(!mEffectorAttached[bone->GetIndex()], "fixed joint??? unsupported skeleton structures. use removeFixedJoints");
		return;
	}
	else if(mEffectorAttached[bone->GetIndex()])
	{
		if(parent==NULL)
		{
			mNode.push_back(IK_sdls::NodeWrap());
			if(USE_EULER_ROOT){
				TString channels;
				if(USE_FIXED_ROOTPOS){
					// rotational joints only.
					channels=bone->getRotationalChannels();
					mNode.back().createNodes(bone, channels);
				}
				else {
					TString channels1=bone->getRotationalChannels();
					TString channels2=bone->getTranslationalChannels();
					channels=channels2+channels1;
					mNode.back().createNodes(bone, channels2, channels1);
				}

				mTree.InsertRoot(mNode.back().node[0]);
				mSelectedJointIndexToTreeIndex[mNode.back().node[0]->GetJointNum()]=bone->treeIndex();
				for(int c=1; c<channels.length(); c++)
				{
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
					mSelectedJointIndexToTreeIndex[mNode.back().node[c]->GetJointNum()]=bone->treeIndex();
				}
			}
			else{
				// quaternion
				mNode.back().createFreeNode(bone);
				mTree.InsertRoot(mNode.back().node[0]);
				mSelectedJointIndexToTreeIndex[mNode.back().node[0]->GetJointNum()]=bone->treeIndex();
			}
			mBoneToNode[bone->GetIndex()]=mNode.size()-1;
		}
		else
		{
			if(mSelectedBone[bone->treeIndex()])
			{
				mNode.push_back(IK_sdls::NodeWrap());
				TString channels=bone->getRotationalChannels();
				mNode.back().createNodes(bone, channels);

				mTree.InsertChild_automatic(parent, mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
				mBoneToNode[bone->GetIndex()]=mNode.size()-1;

				for(int c=0; c<channels.length(); c++)
					mSelectedJointIndexToTreeIndex[mNode.back().node[c]->GetJointNum()]=bone->treeIndex();
			}
			else
			{
				// reuse parent node
				mBoneToNode[bone->GetIndex()]=mBoneToNode[bone->parent()->treeIndex()];
			}
		}


		if(bone->m_pChildHead)
		{
			copyTree((Bone*)bone->m_pChildHead, mNode[mBoneToNode[bone->GetIndex()]].back());
		}
	}

	if(bone->m_pSibling)
	{
		copyTree((Bone*)bone->m_pSibling, mNode[mBoneToNode[bone->m_pParent->GetIndex()]].back());
	}
}

void IK_sdls::LoaderToTree_selected::setPoseDOF(MotionDOFinfo const& mDofInfo, vectorn & pose)
{
	for(int i=2; i<mDofInfo.numBone(); i++)
	{
		if(mEffectorAttached[i] && !mSelectedBone[i])
		{
			// fixed joints
			pose.range(mDofInfo.startT(i), mDofInfo.endR(i)).setAllValue(0);
		}
	}
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		//todoif(mSelectedJointIndexToTreeIndex[i]!=-1)
		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				m_real* trans=&pose[mDofInfo.startT(treeIndex)];
				TString channels=bone->getTranslationalChannels();

				for(int c=0; c<channels.length(); c++)
					mNode[i].node[c]->SetTheta(trans[c]);

				rotStartIndex+=channels.length();
			}
			//else
			{
				if(mDofInfo.hasQuaternion(treeIndex))
				{
					if (mNode[i].node[0]->IsBallJoint())
					{
						mNode[i].node[0]->_local.rotation=pose.toQuater(mDofInfo.startR(treeIndex));
					}
					else
					{
						m_real euler[3];
						TString channels=bone->getRotationalChannels();
						pose.toQuater(mDofInfo.startR(treeIndex)).getRotation(channels, euler);

						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
				}
				else
				{
					m_real* euler=&pose[mDofInfo.startR(treeIndex)];
					TString channels=bone->getRotationalChannels();

					if(channels.length()>0 && channels[0]=='A')
					{
						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
					else{
						for(int c=0; c<channels.length(); c++)
							mNode[i].node[rotStartIndex+c]->SetTheta(euler[c]);
					}
				}
			}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			((IK_sdls::FreeJoint*)mNode[i].node[0])->setTheta(MotionDOF::rootTransformation(pose));
		}
	}

}
void IK_sdls::LoaderToTree_selected::getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn& output)
{
	// set output
	for(int i=0; i<mNode.size(); i++)
	{
		Bone* bone=mNode[i].bone;
		int treeIndex=bone->GetIndex();

		if(mNode[i].node[0]->IsJoint())
		{
			int rotStartIndex=0;
			if (mNode[i].node[0]->IsSlideJoint())
			{
				TString channels=bone->getTranslationalChannels();
				int startT=mDofInfo.startT(treeIndex);
				for(int c=0; c<channels.length(); c++)
					output[startT+c]=mNode[i].node[c]->GetTheta();
				rotStartIndex+=channels.length();
			}
			//else 
			{

				TString channels=bone->getRotationalChannels();

				if(mDofInfo.hasQuaternion(treeIndex))
				{
					if(mNode[i].node[0]->IsBallJoint())
					{
						output.range(mDofInfo.startR(treeIndex), mDofInfo.endR(treeIndex)).assign(mNode[i].node[0]->_local.rotation);
					}
					else
					{
						quater q;
						m_real euler[3];
						for(int c=0; c<channels.length(); c++)
							euler[c]=mNode[i].node[c+rotStartIndex]->GetTheta();
						q.setRotation(channels, euler);
						output.range(mDofInfo.startR(treeIndex),
								mDofInfo.endR(treeIndex)).assign(q);
					}
				}
				else
				{
					int startR=mDofInfo.startR(treeIndex);
					for(int c=0; c<channels.length(); c++)
						output[startR+c]=mNode[i].node[c+rotStartIndex]->GetTheta();
				}
			}
		}
		else if(mNode[i].node[0]->IsFreeJoint())
		{
			const transf& theta=((IK_sdls::FreeJoint*)mNode[i].node[0])->getTheta();
			MotionDOF::setRootTransformation(output, theta);
		}
	}
}

void IK_sdls::LoaderToTree::getSphericalState(MotionDOFinfo const& dofInfo, vectorn& q, vectorn& dq) const
{
	int nquat=dofInfo.numSphericalJoint();
	int ndof=dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;
	int dqsindex=qsindex;

	q.setSize(ndof);
	dq.setSize(ndof-nquat);

	for(int ibone=1; ibone<dofInfo.numBone(); ibone++)
	{
		int ndof=dofInfo.numDOF(ibone);
		if(getNode(ibone).node[0]->IsFreeJoint())
		{
			IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)getNode(ibone).node[0]);
			vector3 v, w;
			fj->GetJointVel(v, w);

			auto& T=getLastNode(ibone)->globalFrame();
			// translation
			q.setVec3(qindex, T.translation);
			dq.setVec3(qindex, v);
			qindex+=3;

			// rotational
			q.setQuater(qsindex, T.rotation);
			dq.setVec3(dqsindex,w);
			dqsindex+=3;
			qsindex+=4;
		}
		else if(ndof==4)
		{
			const auto* node=getLastNode(ibone);
			const auto* pnode=getLastNode(dofInfo.parentIndex(ibone));
			const auto& T=node->globalFrame();
			const auto& pT=pnode->globalFrame();

			//cout <<"node"<<node<<pnode<<endl;
			//cout << ibone<<T<<endl<<dofInfo.parentIndex(ibone)<<pT<<endl;

			const auto& loc_ang_vel=node->bodyAngVel();
			const auto& ploc_ang_vel=pnode->bodyAngVel();

			// pT*_local=T
			transf _local=pT.inverse()*T;

			quater t_rel_att;
			vector3 rel_ang_vel;

			t_rel_att.inverse(_local.rotation);

			// loc_ang_vel= B+rel_ang_vel
			// -> rel_ang_vel= -B+loc_ang_vel
			rel_ang_vel.rotate(t_rel_att, ploc_ang_vel);
			rel_ang_vel *=-1; // -B
			rel_ang_vel += loc_ang_vel ;
			//printf("%s %s\n", loc_ang_vel.output().ptr(), rel_ang_vel.output().ptr());
			q.setQuater(qsindex, _local.rotation);
			dq.setVec3(dqsindex, rel_ang_vel);
			dqsindex+=3;
			qsindex+=4;
		}
		else
		{
			ASSERT(ndof>0); // 
			for(int j=0; j<ndof; j++)
			{
				q[qindex]=getNode(ibone).node[j]->GetTheta();
				dq[qindex]=getNode(ibone).node[j]->GetDTheta();
				qindex++;
			}
		}
	}
	ASSERT(qindex==dofInfo.numDOF()-nquat*4);
	ASSERT(dqsindex==dq.size());
	ASSERT(qsindex==q.size());
}

void IK_sdls::LoaderToTree::getSphericalQ(MotionDOFinfo const& dofInfo, vectorn& q) const
{
	int nquat=dofInfo.numSphericalJoint();
	int ndof=dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;

	q.setSize(ndof);

	for(int ibone=1; ibone<dofInfo.numBone(); ibone++)
	{
		int ndof=dofInfo.numDOF(ibone);
		if(getNode(ibone).node[0]->IsFreeJoint())
		{
			IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)getNode(ibone).node[0]);

			auto& T=getLastNode(ibone)->globalFrame();
			// translation
			q.setVec3(qindex, T.translation);
			qindex+=3;

			// rotational
			q.setQuater(qsindex, T.rotation);
			qsindex+=4;
		}
		else if(ndof==4)
		{
			const auto* node=getLastNode(ibone);
			const auto* pnode=getLastNode(dofInfo.parentIndex(ibone));
			const auto& T=node->globalFrame();
			const auto& pT=pnode->globalFrame();

			// pT*_local=T
			quater _local=pT.rotation.inverse()*T.rotation;

			q.setQuater(qsindex, _local);
			qsindex+=4;
		}
		else
		{
			ASSERT(ndof>0); // 
			for(int j=0; j<ndof; j++)
			{
				q[qindex]=getNode(ibone).node[j]->GetTheta();
				qindex++;
			}
		}
	}
	ASSERT(qindex==dofInfo.numDOF()-nquat*4);
	ASSERT(qsindex==q.size());
}

void IK_sdls::LoaderToTree::setSphericalState(MotionDOFinfo const& dofInfo, const vectorn& q, const vectorn& dq) 
{
	int nquat=dofInfo.numSphericalJoint();
	int ndof=dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;
	int dqsindex=qsindex;

	ASSERT(q.size()==ndof);
	ASSERT(dq.size()==ndof-nquat);

	for(int ibone=1; ibone<dofInfo.numBone(); ibone++)
	{
		int ndof=dofInfo.numDOF(ibone);
		if(getNode(ibone).node[0]->IsFreeJoint())
		{
			IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)getNode(ibone).node[0]);
			vector3 v, w;

			// translation
			fj->_local.translation=q.toVector3(qindex);
			v=dq.toVector3(qindex);
			qindex+=3;

			// rotational
			fj->_local.rotation=q.toQuater(qsindex);
			w=dq.toVector3(dqsindex);
			dqsindex+=3;
			qsindex+=4;

			fj->SetJointVel(v, w);
		}
		else if(ndof==4)
		{
			ASSERT(getNode(ibone).node[0]->IsBallJoint());
			IK_sdls::BallJoint* bj=((IK_sdls::BallJoint*)getNode(ibone).node[0]);
			bj->_local.rotation= q.toQuater(qsindex);
			bj->SetJointVel3DOF(dq.toVector3(dqsindex));
			dqsindex+=3;
			qsindex+=4;
		}
		else
		{
			ASSERT(ndof>0); // 
			for(int j=0; j<ndof; j++)
			{
				getNode(ibone).node[j]->SetTheta(q[qindex]);
				getNode(ibone).node[j]->SetDTheta(dq[qindex]);
				qindex++;
			}
		}
	}
	ASSERT(qindex==dofInfo.numDOF()-nquat*4);
	ASSERT(dqsindex==dq.size());
	ASSERT(qsindex==q.size());
	// update positions
	mTree.Compute();
	// update velocities
	mTree.ComputeDS(mTree.GetRoot()); // update velocity

}

void IK_sdls::LoaderToTree::setSphericalQ(MotionDOFinfo const& dofInfo, const vectorn& q) 
{
	int nquat=dofInfo.numSphericalJoint();
	int ndof=dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;

	ASSERT(q.size()==ndof);

	for(int ibone=1; ibone<dofInfo.numBone(); ibone++)
	{
		int ndof=dofInfo.numDOF(ibone);
		if(getNode(ibone).node[0]->IsFreeJoint())
		{
			IK_sdls::FreeJoint* fj= ((IK_sdls::FreeJoint*)getNode(ibone).node[0]);

			// translation
			fj->_local.translation=q.toVector3(qindex);
			qindex+=3;

			// rotational
			fj->_local.rotation=q.toQuater(qsindex);
			qsindex+=4;
		}
		else if(ndof==4)
		{
			ASSERT(getNode(ibone).node[0]->IsBallJoint());
			IK_sdls::BallJoint* bj=((IK_sdls::BallJoint*)getNode(ibone).node[0]);
			bj->_local.rotation= q.toQuater(qsindex);
			qsindex+=4;
		}
		else
		{
			ASSERT(ndof>0); // 
			for(int j=0; j<ndof; j++)
			{
				getNode(ibone).node[j]->SetTheta(q[qindex]);
				qindex++;
			}
		}
	}
	ASSERT(qindex==dofInfo.numDOF()-nquat*4);
	ASSERT(qsindex==q.size());
	mTree.Compute();
}
