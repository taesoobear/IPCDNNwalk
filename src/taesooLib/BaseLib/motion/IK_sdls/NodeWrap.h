// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
#ifndef NODEWRAP_H_
#define NODEWRAP_H_



#include "Node.h"
#include "Tree.h"
#include "Jacobian.h"
#include "../FullbodyIK.h"
#include "../VRMLloader.h"
#include "../Liegroup.h"
namespace IK_sdls
{
struct NodeWrap
{
	NodeWrap() {
		for(int i=0; i<6; i++) 
			node[i]=NULL; 
		bone=NULL;
		bone2=NULL;
	}
	IK_sdls::Node* node[6];
	::TString axes;
	IK_sdls::Node* back()
	{
		return axes.length()?node[axes.length()-1]:NULL;
	}
	const IK_sdls::Node* back() const
	{
		return axes.length()?node[axes.length()-1]:NULL;
	}
	void createEffector(Bone* _bone, vector3 localPos)
	{
		bone=_bone;
		axes="EFFECTOR";
		node[0]=new IK_sdls::Effector(localPos);
	}
	/*
	 * moved to FullbodyIK_MotionDOF
	void createRelativeConstraint(Bone* _bone, Bone* _bone2, vector3 localPos1)
	{
		bone=_bone;
		bone2=_bone2;
		axes="RELCON";
		node[0]=new IK_sdls::RelativeConstraint(localPos1);
	}
	*/
	void createFreeNode(Bone* _bone)
	{
		bone=_bone;
		axes="F";
		node[0]=new IK_sdls::FreeJoint();
	}
	void createBallJoint(Bone* _bone)
	{
		bone=_bone;
		axes="B";
		node[0]=new IK_sdls::BallJoint(bone->getOffsetTransform().translation);
	}
	void createNodes(Bone* _bone, const char* _axes)
	{
		bone=_bone;
		axes=_axes;

		//printf("_bone %s %s\n", _bone->NameId, axes.ptr());
		ASSERT(axes.length()<=3);
		vector3 unitx (1,0,0);
		vector3 unity (0,1,0);
		vector3 unitz (0,0,1);
		vector3 ax;
		Msg::verify(axes.length()>0, "fixed");
		for(int i=0, len=axes.length(); i<len; i++)
		{

			if(axes[i]=='Z')
				ax=unitz;
			else if(axes[i]=='Y')
				ax=unity;
			else if(axes[i]=='A')
			{
				ax=_bone->getArbitraryAxis(i);
				printf("_bone %s %d %s\n", _bone->NameId, i, ax.output().c_str());
			}
			else
				ax=unitx;

			if (i==0)
				node[i]=new IK_sdls::HingeJoint(bone->getOffsetTransform().translation, ax);
			else
				node[i]=new IK_sdls::HingeJoint(vector3(0,0,0), ax);
		}
	}
	void createNodes(Bone* _bone, const char* _taxes, const char* _raxes)
	{
		bone=_bone;
		TString taxes=_taxes;
		TString raxes=_raxes;
		axes=taxes+raxes;

		//printf("_bone %s %s\n", _bone->NameId, axes.ptr());
		ASSERT(axes.length()<=6);
		vector3 unitx (1,0,0);
		vector3 unity (0,1,0);
		vector3 unitz (0,0,1);
		vector3 ax;
		Msg::verify(axes.length()>0, "fixed");
		for(int i=0, len=axes.length(); i<len; i++)
		{

			if(axes[i]=='Z')
				ax=unitz;
			else if(axes[i]=='Y')
				ax=unity;
			else if(axes[i]=='A')
				ax=_bone->getArbitraryAxis(i);
			else
				ax=unitx;

			if(i<taxes.length())
			{
				if (i==0)
					node[i]=new IK_sdls::SlideJoint(bone->getOffsetTransform().translation, ax);
				else
					node[i]=new IK_sdls::SlideJoint(vector3(0,0,0), ax);
			}
			else
			{
				if (i==0)
					node[i]=new IK_sdls::HingeJoint(bone->getOffsetTransform().translation, ax);
				else
					node[i]=new IK_sdls::HingeJoint(vector3(0,0,0), ax);
			}
		}
	}
	Bone* bone;
	Bone* bone2;
};
class LoaderToTree
{
	bool USE_EULER_ROOT, USE_FIXED_ROOTPOS;
	int _nDOF;
	protected:
		void copyTree(Bone* bone, IK_sdls::Node* parent);
		void compareTrees(vector3 trans);
	public:
		inline int nDOF() const { return _nDOF;}
		inline int nJoints()const  { return mTree.GetNumJoint();}
		// set useEulerRoot to false, if you do not know what to do.
		// set useFixedRootPos to false, if you do not know what to do.
		LoaderToTree(MotionLoader& skeleton, bool useEulerRoot, bool useFixedRootPos);
		// when mEffectors.size()!=0, this only copies those bones that are related to the effectors.
		LoaderToTree(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints, bool useEulerRoot, bool useFixedRootPos);
		LoaderToTree();
		~LoaderToTree(){}

		void _init(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints, bool useEulerRoot, bool useFixedRootPos);
		void _init_part2(MotionLoader const& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints);

		void updateBoneLength(MotionLoader const&loader);

		// forward kinematics
		// pose: x,y,z,qw,qx,qy,qz,theta1,theta2,...,thetaN)
		inline void setPoseDOF(MotionDOFinfo const& mDofInfo, vectorn const& pose) 
		{ 
			setPoseDOF(mTree, mDofInfo, pose, mNode); 
			//mTree.ComputeTree(mTree.GetRoot()); -> does not update effector positions
			mTree.Compute();// updates effector positions too.
		}
		inline void computeTree() { mTree.Compute(); }


		// compatible with get/setLinkData
		void setPoseDOF(IK_sdls::Tree & mTree, MotionDOFinfo const& mDofInfo, vectorn const& pose, std::vector<IK_sdls::NodeWrap>& mNode);
		// DTheta: root joint contains body-local linear and angular velocities. compatible with TRL simulators' pose and dpose from getLinkData.
		void setVelocity(MotionDOFinfo const& mDofInfo, vectorn const& dtheta);
		inline void setLinkData(vectorn const& pose, vectorn const& dpose)
		{
			vectorn q, dq;
			poseToQ(pose, q);
			dposeToDQ(pose.toQuater(3), dpose, dq);
			setQuaterQ(&q[0]); setDQ(dq);
		}
		// inverse kinematics
		inline void getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn & pose) 
		{
			getPoseDOF(mDofInfo, pose, mNode);
		}
		void getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn& pose, std::vector<IK_sdls::NodeWrap>const& mNode);
		void getVelocity(MotionDOFinfo const& mDofInfo, vectorn & dtheta);

		const transf & globalFrame(int ibone) const { return getLastNode(ibone)->globalFrame();}
		

		// for skeletons with spherical joints, get/setSphericalState is often more convenient than getPoseDOF/getVelocity.
		/* 
		 * our spherical state packing is different!!!
		 *
		 *  in our case, linear parts appear first, and then 3 DOF ball joints (YUP).
		 *         root                                     |     3           3         ...   
		 q_linear= [x, y, z, hinge1, hinge2, hinge3, hinge4]
		 q_quat  =                                          [qw,qx,qy,qz, qw2,qx2,qy2,qz2, qw3,qx3,qy3,qz3, ....]
		 q= [q_linear, q_quat]
		dq=	[dx,dy,dz,dhinge1,dhinge2,dhinge3,dhinge4,        wx,wy,wz,     wx2,wy2,wz2,     wx3,wy3,wz3, ...
		tau is packed in the same way with dq. (but in body coordinate)
		*/
		void getSphericalState(MotionDOFinfo const& spherical_dofInfo, vectorn& q, vectorn& dq) const; // works even when "this" is created from a different dofinfo.
		void setSphericalState(MotionDOFinfo const& spherical_dofInfo, const vectorn& q, const vectorn& dq) ; // works only when "this" is created from the same dofinfo.
		void getSphericalQ(MotionDOFinfo const& spherical_dofInfo, vectorn& q) const;
		void setSphericalQ(MotionDOFinfo const& spherical_dofInfo, const vectorn& q);
		
		inline vector3 getWorldVelocity(int ibone){ IK_sdls::Node* n=getLastNode(ibone); return n->_global.rotation*n->bodyLinVel(); }
		inline vector3 getWorldAngVel(int ibone) { IK_sdls::Node* n=getLastNode(ibone); return n->_global.rotation*n->bodyAngVel(); }
		inline vector3 getWorldVelocity(int ibone, vector3 const& localpos)
		{
			return getWorldVelocity(ibone)+getWorldAngVel(ibone).cross(globalFrame(ibone).rotation*localpos); 
		}

		// this function does not update node velocities
		// setPoseDOF, integrate(dtheta), getPoseDOF
		void integrate(MotionDOFinfo const& mDofInfo, vectorn const& dtheta, double timestep);

		// this function does not update node velocities
		// setPoseDOF, setVelocity, integrate(), getPoseDOF, getVelocity
		// or setSphericalState, integrate(), getSphericalState
		void integrate(MotionDOFinfo const& mDofInfo, double timestep);

		// query / conversion 
		// the Q/DQ-related functions do not work for ball joints (except a free root joint). Use get/setSpherical* functions for ball joints.
		// q : compatible with TRL simulators' getQ, setQ. but not with sim.getLinkData(JOINT_VALUE, pose) --> use sim.poseToDQ
		// dq : compatible with TRL simulators' getDQ, setDQ. but not with sim.getLinkData(JOINT_VELOCITY, dpose) --> use sim.dposeToDQ
		// R0*DTheta[0:3] == dq[3:6] (global linear velocity)
		// R0*DTheta[3:6] == dq[0:3] (global angular velocity)
		// note that angular velocity appears first here.
		// SDFAST style packing (x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw), which is different from the MotionDOF/PoseDOF format  (x,y,z,qw,qx,qy,qz, theta1, ..., thetaN)
		void setEulerQ(const double* q); //(x,y,z,rz,rx,ry,theta1,theta2,...,thetaN)
		void setQuaterQ(const double* q); //(x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw)
		void setQ(const double* q);
		void getQ(double* q); // get internal representation (euler angles or a quaternion for rootOri)
		void setDQ(const double* dq); // (wx,wy,wz, vx,vy,vz, dtheat1, dtheta2, ..., dthetaN), w,v in global.
		void getDQ(double* dq);
		inline void getDQ( vectorn& dq) { getDQ(&dq[0]); }
		inline void getQ( vectorn& q) { getQ(&q[0]); }
		inline void setQ( vectorn const& q) { setQ(&q[0]); }
		inline void setDQ( vectorn const& q) { setDQ(&q[0]); }

		static void poseToQ(vectorn const& pose, vectorn& q);
		static void dposeToDQ(quater const& rootOri, vectorn const& dpose, vectorn& dq) ;

		// get theta/dtheta which is compatible with EffectorJacobian. 
		// theta/dtheta are internal representations. incompatible with simulators. 
		// theta is a sparse subvector of q
		void getTheta(double* x);
		void setTheta(const double* x);
		void getDTheta(double* dq);
		inline void getDTheta( vectorn& dq) { getDTheta(&dq[0]); }
		inline void getTheta( vectorn& q) { getTheta(&q[0]); }
		inline void setTheta( vectorn const& q) { setTheta(&q[0]); }

		// error : dS%dS + verticalCoef*dS.y*dS.y
		double computeConstraintError(double verticalCoef=0.0);
		void computeConstraintErrorGradient(double* g, double verticalCoef=0.0);

		
		IK_sdls::Tree mTree;

		inline int numJoint() const { return mNodeTraverse.size();}
		inline int numBone() const { return mBoneToNode.size();}
		// faster than mTree.GetJoint(jointindex)
		inline IK_sdls::Node* getJoint(int jointindex) { return mNodeTraverse[jointindex];}
		inline NodeWrap& getNode(int treeIndex) { return mNode[mBoneToNode[treeIndex]]; }
		inline const NodeWrap& getNode(int treeIndex) const { return mNode[mBoneToNode[treeIndex]]; }
		// get the last node node that usually contains a collision geometry.
		IK_sdls::Node* getLastNode(int treeIndex) { return getNode(treeIndex).back();}
		const IK_sdls::Node* getLastNode(int treeIndex) const { return getNode(treeIndex).back();}
		IK_sdls::Node* getNode(int treeIndex, int dofIndex) { return mNode[mBoneToNode[treeIndex]].node[dofIndex]; }
		const IK_sdls::Node* getNode(int treeIndex, int dofIndex) const { return mNode[mBoneToNode[treeIndex]].node[dofIndex]; }
		int getVarIndex(int treeIndex, int dofIndex=0)const { return mNode[mBoneToNode[treeIndex]].node[dofIndex]->GetJointNum(); }
		int getVarEndIndex(int treeIndex)const { if(treeIndex==mBoneToNode.size()-1) return nDOF(); return mNode[mBoneToNode[treeIndex+1]].node[0]->GetJointNum(); }
		int getVarIndexByAxis(int treeIndex, const char *axis);

		// compute the transpose of the effector jacobian matrix.
		// after "setPoseDOF", call this.
		void calcEffectorJacobianTranspose(matrixn& Jt);
		// computes jacobian. this assumes that the ibone is in the subtree.
		void calcJacobianTransposeAt(matrixn& Jt, int ibone, vector3 const& localpos);
		void updateGrad_S_JT(double* g,vector3 const& deltaS, int ibone, vector3 const& localpos);
		inline void updateGrad_S_JT(vectorn& g,vector3 const& deltaS, int ibone, vector3 const& localpos) { updateGrad_S_JT(&g[0], deltaS, ibone, localpos); }
		void updateGrad_S_JT_residual(double* g,vector3 const& deltaS, int ibone);
		void getJacobianSparsity(boolN& hasValue, int ibone);
		void findAxes(boolN& hasValue, vector3 const& axis); // are the current joint axes similar to the user-given axis?

		// computes jacobian for calculating global angular velocity
		void calcRotJacobianTranspose(matrixn& Jt, int ibone);
		// This currently assumes that the chainRootBone and chainTailBone are in a different chain to the root bone.
		void calcJacobianTransposeAt(matrixn& Jt, int chainRootbone, int chainTailBone, vector3 const& localpos_tailbone);

		// loader is used only for obtaining the mass and inertia
		// to modify the COM of the skeleton.
		void calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT);
		void updateCOM_grad_S_JT(vectorn & grad, const VRMLloader& loader, vector3 const& deltaS, double wx, double wy, double wz);

		// to modify the COM of a chain. chainRootBone may not be the root. This currently assumes that the chainRootBone and chainTailBone are in a different chain to the root bone.
		void calcCOMjacobianTranspose(const VRMLloader& loader, matrixn& JT, int chainRootBone, int chainTailBone);
		void calcMomentumJacobianTranspose(const VRMLloader& loader, matrixn& JT);
		// momentum from the pose of chain1 to the pose of "this"
		Liegroup::dse3 calcMomentumCOMfromPose(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain1); 
		// momentum from the pose of "this" to the pose of "chain2"
		Liegroup::dse3 calcMomentumCOMtoPose(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain2); 
		// momentum from DQ
		Liegroup::dse3 calcMomentumCOM(const VRMLloader& loader); 
		Liegroup::se3 calcInverseInertiaTimesMomentumCOM(const VRMLloader& loader); 

		// returns total_mass and I s.t.
		//  r=vector3(I(7), I(8), I(9))
		// 	I6=[I(0),I(3), I(4), 0, -r.z, r.y;
		//		I(3),I(1), I(5), r.z, 0,  -r.x;
		//		I(4),I(5), I(2), -r.y, r.x, 0 ;
		//		0, r.z, -r.y, m,   0 , 0;
		//		-r.z,0,  r.x, 0,  m,  0;
		//		r.y, -r.x, 0, 0,  0,  m]
		double calcInertia(const VRMLloader& loader, vectorn& inertia) const; 

		vector3 calcCOM(const VRMLloader& loader);
		
		// only this function is compatible with TRL simulators
		// in this case, J * dq =dot x, where dq[0:3] is the global angular velocity, dq[3:6] is the global linear velocity.
		// so the aforementioned dq is incompatible with getDQ, setDQ function (regardless of the use of eulerRoot or not). 
		// For that purpose (setDQ, getDQ), use the above function (calcJacobianTransposeAt).
		void calcGlobalJacobianTransposeAt(matrixn& Jt, int ibone, vector3 const& localpos);

		std::vector<IK_sdls::NodeWrap> mNode;
		std::vector<IK_sdls::NodeWrap> mEffectorNode;
		bitvectorn mEffectorAttached;
		intvectorn mBoneToNode;
		intvectorn mDQindex; // Tree::jointIndex to DQindex
		std::vector<IK_sdls::Node*> mNodeTraverse;
		inline intvectorn const& getDQindex() {return mDQindex;}
		//std::vector<MotionUtil::RelativeConstraint*> _nonEffector_constraints;
};
class LoaderToTree_selected
{
	bool USE_EULER_ROOT, USE_FIXED_ROOTPOS;
	protected:
		void copyTree(Bone* bone, IK_sdls::Node* parent);
		void _init(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos);
		// copyt tree
		void _init_part1(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos);
		// attach effectors
		void _init_part2(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors);
	public:
		LoaderToTree_selected(){}
		//LoaderToTree_selected(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, std::vector<MotionUtil::RelativeConstraint>& constraints, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos);
		LoaderToTree_selected(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& mEffectors, intvectorn const& selectedJoints, bool useEulerRoot, bool useFixedRootPos);
		~LoaderToTree_selected(){}

		// all fixed joints will be reset to 0.
		void setPoseDOF(MotionDOFinfo const& mDofInfo, vectorn & pose );
		void getPoseDOF(MotionDOFinfo const& mDofInfo, vectorn& pose);

		IK_sdls::Tree mTree;

		std::vector<IK_sdls::NodeWrap> mNode;
		std::vector<IK_sdls::NodeWrap> mEffectorNode;
		bitvectorn mEffectorAttached;
		intvectorn mBoneToNode;
		bitvectorn mSelectedBone;
		intvectorn mSelectedJointIndexToTreeIndex;
		int getActualTreeIndex(int treeIndex)
		{
			IK_sdls::Node* node=mNode[mBoneToNode[treeIndex]].node[0];
			if(!node) return -1;
			int actualTreeIndex=mSelectedJointIndexToTreeIndex[node->GetJointNum()];
			return actualTreeIndex;
		}
		NodeWrap& getNode(int treeIndex)
		{
			return mNode[mBoneToNode[treeIndex]];
		}
};
}

#endif
