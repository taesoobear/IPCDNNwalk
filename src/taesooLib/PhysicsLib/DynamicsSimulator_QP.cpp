
// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include "physicsLib.h"
#include <vector>
#include <map>
#include <algorithm>
#include "../BaseLib/motion/VRMLloader.h"
#include "DynamicsSimulator_QP.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/math/Operator_NR.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#include "convexhull/graham.h"
inline void RMat_assign(matrixn & out, matrix3 const& in)
{
	out.setSize(3,3);

	for(int i=0; i<out.rows(); i++)
		for(int j=0; j<out.cols(); j++)
			out(i,j)=in.m[i][j];
}
namespace OpenHRP {
class ConstrainedPoints_impl: public ConstrainedPoints
{
	public:
	DynamicsSimulator* sim;

	static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = false;

	void setContactConstraintPoints(const LinkPair& linkPair, ConstrainedLinkPair& clp, CollisionPointSequence& collisionPoints)
	{
		std::vector<ConstraintPoint>& constraintPoints = clp.constraintPoints;
		constraintPoints.clear();
		int numExtractedPoints = 0;
		int numContactsInPair = collisionPoints.size();

		for(int j=0; j < numContactsInPair; ++j){

			CollisionPoint& collision = collisionPoints[j];
			constraintPoints.push_back(ConstraintPoint());
			ConstraintPoint& contact = constraintPoints.back();

			contact.point= collision.position;
			contact.normalTowardInside[1]= collision.normal;
			contact.normalTowardInside[0]= -contact.normalTowardInside[1];
			contact.depth = collision.idepth;

			numExtractedPoints++;
			contact.globalIndex = globalNumConstraintVectors++;

			// check velocities
			::vector3 v[2];
			for(int k=0; k < 2; ++k){
				::vector3 lpos= sim->getWorldState(linkPair.charIndex[k])._global(linkPair.link[k]->treeIndex()).toLocalPos(contact.point);
				v[k]=sim->getWorldVelocity(linkPair.charIndex[k], linkPair.link[k], lpos);
			}
			contact.relVelocityOn0 = v[1] - v[0];
			int k=0;
			contact.orientationOn0 = sim->getWorldState(linkPair.charIndex[k])._global(linkPair.link[k]->treeIndex()).rotation;


			contact.normalProjectionOfRelVelocityOn0 = contact.normalTowardInside[1]% contact.relVelocityOn0;

			::vector3 v_tangent(contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside[1]);

			contact.globalFrictionIndex = globalNumFrictionVectors;

			const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-4;
			const bool ENABLE_STATIC_FRICTION = true;
			const bool ONLY_STATIC_FRICTION_FORMULATION = (true && ENABLE_STATIC_FRICTION);
			const double muStatic=1.4;
			const double muDynamic=0.7;

			double vt_square = v_tangent% v_tangent;
			static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
			bool isSlipping = (vt_square > vsqrthresh);
			contact.mu = isSlipping ? muDynamic : muStatic;


			if( !ONLY_STATIC_FRICTION_FORMULATION && isSlipping){
				contact.numFrictionVectors = 1;
				double vt_mag = sqrt(vt_square);
				::vector3 t1(v_tangent / vt_mag);
				::vector3 t2(contact.normalTowardInside[1].cross(t1));
				::vector3 t3(t2.cross(contact.normalTowardInside[1]));
				contact.frictionVector[0][0] = t3;
				contact.frictionVector[0][0] .normalize();
				contact.frictionVector[0][1] = -contact.frictionVector[0][0];

			} else {
				if(ENABLE_STATIC_FRICTION){
					contact.numFrictionVectors = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? 2 : 4);
					setFrictionVectors(contact);
				} else {
					contact.numFrictionVectors = 0;
				}
			}
			globalNumFrictionVectors += contact.numFrictionVectors;
		}
	}
	void setFrictionVectors(ConstraintPoint& contact)
	{
		::vector3 u(0.0);
		int minAxis = 0;
		::vector3& normal = contact.normalTowardInside[0];
		if(false)
		{
			// use axis-aligned u vector.
			for(int i=1; i < 3; i++){
				if(fabs(normal[i]) < fabs(normal[minAxis])){
					minAxis = i;
				}
			}
			u[minAxis] = 1.0;
		}
		else
		{
			// use one of local coordinates
			::vector3 axes[3];
			axes[0]=contact.orientationOn0*::vector3(1,0,0);
			axes[1]=contact.orientationOn0*::vector3(0,1,0);
			axes[2]=contact.orientationOn0*::vector3(0,0,1);
			for(int i=1; i < 3; i++){
				if(fabs(axes[i]%normal[i]) < fabs(axes[i]%normal[minAxis])){
					minAxis = i;
				}
			}
			//printf("%s\n", axes[0].output().ptr());
			u=axes[minAxis];
		}

		::vector3 t1(normal.cross( u));
		t1 /= t1.length();
		::vector3 t2(normal.cross( t1));
		t2 /= t2.length();

		contact.frictionVector[0][0] = t1;
		contact.frictionVector[1][0] = t2;

		if(STATIC_FRICTION_BY_TWO_CONSTRAINTS){
			contact.frictionVector[0][1] = -contact.frictionVector[0][0];
			contact.frictionVector[1][1] = -contact.frictionVector[1][0];
		} else {
			contact.frictionVector[2][0] = -contact.frictionVector[0][0];
			contact.frictionVector[3][0] = -contact.frictionVector[1][0];

			contact.frictionVector[0][1] = contact.frictionVector[2][0];
			contact.frictionVector[1][1] = contact.frictionVector[3][0];
			contact.frictionVector[2][1] = contact.frictionVector[0][0];
			contact.frictionVector[3][1] = contact.frictionVector[1][0];
		}
	}
	public:


	ConstrainedPoints_impl(DynamicsSimulator* _sim) {sim=_sim;}
	void addAllPoints(std::vector<LinkPair> const& collisionCheckLinkPairs, CollisionSequence& collisions)
	{
		for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i)
		{
			const LinkPair& linkPair = collisionCheckLinkPairs[i];
			CollisionPointSequence& points = collisions[i].points;

			if(points.size() > 0)
			{
				constrainedLinkPairs.resize(constrainedLinkPairs.size()+1);
				ConstrainedLinkPair & clp=constrainedLinkPairs.back();
				clp.linkPair=&linkPair;
				clp.ilinkPair=i;
				setContactConstraintPoints(linkPair, clp, points);
			}
		}
	}

	virtual void solve(std::vector<LinkPair> const& collisionCheckLinkPairs, CollisionSequence& collisions)
	{
		globalNumConstraintVectors = 0;
		globalNumFrictionVectors = 0;
		constrainedLinkPairs.clear();

		bool convexClipping=true;
		if(convexClipping)
		{
			GrahamScan convexHull=GrahamScan();
			std::vector<intvectorn > all_points; 
			all_points.resize(collisionCheckLinkPairs.size());
			int c=0;
			for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i)
			{
				const LinkPair& linkPair = collisionCheckLinkPairs[i];
				CollisionPointSequence& points = collisions[i].points;
				int numContactsInPair = points.size();
				for(size_t j=0; j<numContactsInPair; j++)
				{
					convexHull.add_point(std::pair<double,double>(points[j].position.x, points[j].position.z));
					c++;
				}
			}
			if (c>3)
			{
				convexHull.partition_points();
				convexHull.build_hull();
				matrixn out;
				convexHull.get_hull(out);
				// brute force search for support points
				for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i){
					const LinkPair& linkPair = collisionCheckLinkPairs[i];
					CollisionPointSequence& points = collisions[i].points;
					int numContactsInPair = points.size();
					for(size_t j=0; j<numContactsInPair; j++){
						int found=false;
						for(size_t k=0; k<out.rows()-1; k++){
							double x=points[j].position.x;
							double z=points[j].position.z;
							double x2=out(k,0);	
							double z2=out(k,1);	

							if( (x2-x)*(x2-x)+(z2-z)*(z2-z) < 1e-8)
							{
								found=true;
								break;
							}	
						}
						if(found)
							all_points[i].push_back(j);
						
					}
				}
				CollisionPointSequence temp;
				for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i){

					const LinkPair& linkPair = collisionCheckLinkPairs[i];
					CollisionPointSequence& points = collisions[i].points;

					if(all_points[i].size()>0)
					{
						temp.resize(all_points[i].size());
						for(int j=0; j<all_points[i].size(); j++)
						{
							temp[j]=points[all_points[i][j]];
						}
						constrainedLinkPairs.resize(constrainedLinkPairs.size()+1);
						ConstrainedLinkPair & clp=constrainedLinkPairs.back();
						clp.linkPair=&linkPair;
						clp.ilinkPair=i;
						setContactConstraintPoints(linkPair, clp, temp);
					}
				}
				//printf("%s\n", out.output().ptr());
			}
			else 
			{
				addAllPoints(collisionCheckLinkPairs, collisions);
			}
		}
		else 
		{
			addAllPoints( collisionCheckLinkPairs, collisions);
		}
		//globalNumContactNormalVectors = globalNumConstraintVectors;
	}
};
}
using namespace OpenHRP;
#define USE_CONSTRAINED_POINTS
#ifdef USE_CONSTRAINED_POINTS
#endif
DynamicsSimulator_QP::DynamicsSimulator_QP(DynamicsSimulator* sim)
{
	_contacts=new ConstrainedPoints_impl(sim);
}
DynamicsSimulator_QP::~DynamicsSimulator_QP()
{
	delete _contacts;
}

void DynamicsSimulator_QP::getContactBases(std::vector<ContactBasis>& basis, double frictionCoef) const
{
#ifdef USE_CONSTRAINED_POINTS
	ConstrainedPoints_impl* solver=(ConstrainedPoints_impl*)_contacts;
	std::vector<ConstrainedPoints_impl::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	int link_pair_count=0;
	int constraint_point_count=0;
	for (int i=0; i<n ; ++i){
		ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			VRMLTransform* link=linkPair->linkPair->link[ipair];
			if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
				std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				if (numConstraintPoints>0) link_pair_count++;
				constraint_point_count+=numConstraintPoints ;
			}
		}
	}
	basis.resize(constraint_point_count);
	constraint_point_count=0;
	for (int i=0; i<n ; ++i){
		ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			VRMLTransform* link=linkPair->linkPair->link[ipair];
			if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
				std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				int ibody=linkPair->linkPair->charIndex[ipair];
				int ibone=link->treeIndex();
				for(int k=0; k < numConstraintPoints; ++k){
					ConstrainedPoints_impl::ConstraintPoint& constraint = constraintPoints[k];
					ContactBasis& bb=basis[constraint_point_count++];
					bb.ibody=ibody;
					bb.ibone=ibone;
					bb.ilinkpair=linkPair->ilinkPair;
					bb.globalpos=constraint.point;
					bb.depth=constraint.depth;
					bb.normal=constraint.normalTowardInside[ipair];
					bb.frictionNormal.setSize(constraint.numFrictionVectors);
					for(int j=0; j < constraint.numFrictionVectors; ++j){
						bb.frictionNormal[j]=constraint.frictionVector[j][ipair];
						//printf("%s\n", bb.frictionNormal[j].output().ptr());
						if(frictionCoef!=0.0){
							bb.frictionNormal[j]+=frictionCoef*bb.normal;
							bb.frictionNormal[j].normalize();
						}
					}
					bb.globalIndex=constraint.globalIndex;
					bb.globalFrictionIndex=solver->globalNumConstraintVectors + constraint.globalFrictionIndex;
					bb.relvel=constraint.relVelocityOn0*-1;
				}
			}
		}
	}
#else
	
	CFSImpl_LCP* solver=(CFSImpl_LCP*)AISTsim->world.contactForceSolver.impl;
	std::vector<CFSImpl_LCP::LinkPair*>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	int link_pair_count=0;
	int constraint_point_count=0;
	for (int i=0; i<n ; ++i){
		CFSImpl_LCP::LinkPair* linkPair=clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			Link* link=linkPair->link[ipair];
			if (link->jointType!=Link::FIXED_JOINT ){
				CFSImpl_LCP::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				if (numConstraintPoints>0) link_pair_count++;
				constraint_point_count+=numConstraintPoints ;
			}
		}
	}
	basis.resize(constraint_point_count);
	constraint_point_count=0;
	for (int i=0; i<n ; ++i){
		CFSImpl_LCP::LinkPair* linkPair=clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			Link* link=linkPair->link[ipair];
			if (link->jointType!=Link::FIXED_JOINT ){
				CFSImpl_LCP::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				int ijoint=link->jointId+1;
				int ibody=linkPair->bodyIndex[ipair];
				int ibone=_cinfo[ibody]->_links[ijoint]->boneId;
				for(int k=0; k < numConstraintPoints; ++k){
					CFSImpl_LCP::ConstraintPoint& constraint = constraintPoints[k];
					ContactBasis& bb=basis[constraint_point_count++];
					bb.ibody=ibody;
					bb.ibone=ibone;
					bb.ilinkpair=linkPair->index;
					bb.globalpos=toBase(constraint.point);
					bb.depth=constraint.depth;
					bb.normal=toBase(constraint.normalTowardInside[ipair]);
					bb.frictionNormal.setSize(constraint.numFrictionVectors);
					for(int j=0; j < constraint.numFrictionVectors; ++j){
						bb.frictionNormal[j]=toBase(constraint.frictionVector[j][ipair]);
						//printf("%s\n", bb.frictionNormal[j].output().ptr());
						if(frictionCoef!=0.0){
							bb.frictionNormal[j]+=frictionCoef*bb.normal;
							bb.frictionNormal[j].normalize();
						}
					}
					bb.globalIndex=constraint.globalIndex;
					bb.globalFrictionIndex=solver->globalNumConstraintVectors + constraint.globalFrictionIndex;
					bb.relvel=toBase(constraint.relVelocityOn0)*-1;
				}
			}
		}
	}
#endif
}
void DynamicsSimulator_QP::calcContactBasisAll(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double frictionCoef)
{
	ConstrainedPoints_impl* solver=(ConstrainedPoints_impl*)_contacts;
	std::vector<ConstrainedPoints_impl::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	vectorn vFe(6);
	
	// contact-lcp.xoj page 3
	int a=solver->globalNumConstraintVectors+solver->globalNumFrictionVectors;
	
	DynamicsSimulator* sim=solver->sim;
	
	
	matrixn V_temp;
	matrixn dotV_temp;
	{
		int ichara=0;
		int N=sim->_characters[ichara]->skeleton->dofInfo.numActualDOF(); // excluding unused 4 th dimension.

		v_all.setSize(6*link_pair_count, a);
		dot_v_all.setSize(6*link_pair_count, a);
		V_temp.setSize(6, a);
		dotV_temp.setSize(6,a);

		matrixn R, dAd, R_dAd, dot_R_dAd;
		R_dAd.setSize(6,6);
		R_dAd.setAllValue(0);
		R_dAd.diag().setAllValue(1);
		dot_R_dAd.setSize(6,6);
		dot_R_dAd.setAllValue(0);

		int c_pair=0;
		for (int i=0; i<n ; ++i){
			ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
			for (int ipair=0; ipair<2; ++ipair){
				VRMLTransform* link=linkPair->linkPair->link[ipair];
				if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
					std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
					int numConstraintPoints = constraintPoints.size();
					if (numConstraintPoints>0)
					{
						int ibody=linkPair->linkPair->charIndex[ipair];
						int ibone=link->treeIndex();
						// printf("ijoint: %d %d\n", ijoint, ibody);
						//std::vector<BodyPart*>& cinfo=_cinfo[ibody]->_links;
						{
							// calc V_i
							V_temp.setAllValue(0);
							dotV_temp.setAllValue(0);
							// R is to revert rotation made in calcJacobian (convert to local jacobian)
							// e.g. J'*R = (R'*J)'
							// or more intuitively,
							// actually R*dAd(T_global) = dAd(T_global.GetPosition())
							// so just transforming contact position while preserving global orientation.

							// RMat_assign(R, cinfo[ijoint]->body->T_global.GetRotation());

							//dse3 Fe=dAd(cinfo[ijoint]->body->T_global, dse3(toGMBS(cross(constraint.point, f)),toGMBS(f)));
							for(int k=0; k < numConstraintPoints; ++k){
								ConstrainedPoints_impl::ConstraintPoint& constraint = constraintPoints[k];
								::vector3 n,p,pcn;
								n=constraint.normalTowardInside[ipair];
								p=constraint.point;
								// printf("n: %s\n", n.output().ptr());
								int globalIndex = constraint.globalIndex;
								vectornView vk=V_temp.column(globalIndex);
								vectornView dot_vk=dotV_temp.column(globalIndex);
								pcn.cross(p,n);
								vk.setVec3(0, vk.toVector3(0)+pcn);
								vk.setVec3(3, vk.toVector3(3)+n);
								::vector3 dotp=constraint.relVelocityOn0*-1;
								/*
								{
									::vector3 relvel2;
									VRMLloader* l=_characters[0]->skeleton;
									int ibone=cinfo[ijoint]->boneId;
									::vector3 lpos= getWorldState(0)._global(ibone).toLocalPos(p);
									getWorldVelocity(0, &l->VRMLbone(ibone),lpos, relvel2);
									printf("vv%s==%s\n", dotp.output().ptr(), relvel2.output().ptr());
								}
								*/
								pcn.cross(dotp,n);
								dot_vk.setVec3(0,dot_vk.toVector3(0)+pcn);
								::vector3 n0=n;

								for(int j=0; j < constraint.numFrictionVectors; ++j){
									vectornView vk=V_temp.column(solver->globalNumConstraintVectors + constraint.globalFrictionIndex + j);
									vectornView dot_vk= dotV_temp.column(solver->globalNumConstraintVectors + constraint.globalFrictionIndex + j);
									n=constraint.frictionVector[j][ipair];
									// printf("f%d: %s\n", j, n.output().ptr());

									if (frictionCoef!=0.0 ){
										n=(n+frictionCoef*n0); // taesoo modified: friction coef 1
										n.normalize();
									}
									pcn.cross(p,n);
									vk.setVec3(0, vk.toVector3(0)+pcn);
									vk.setVec3(3, vk.toVector3(3)+n);
									pcn.cross(dotp,n);
									dot_vk.setVec3(0,dot_vk.toVector3(0)+pcn);
								}


							}
							//dAd.range(0,3,0,3).transpose(R);
							//dAd.range(3,6,0,3).setAllValue(0);
							//dAd.range(0,3,3,6).multAtBt(R, b);
							//dAd.range(3,6,3,6).transpose(R);
							//R_dAd.range(0,3,3,6).skew(cinfo[ijoint]->body->T_global.GetPosition());
							//GBodyRigid* body=getBody(cinfo, &_characters[ibody]->skeleton->VRMLbone(ibone));
							VRMLTransform* b=&(sim->skeleton(ibody).VRMLbone(ibone));
							RMat_assign(R_dAd.range(0,3,3,6).lval(),Liegroup::skew(sim->getWorldState(ibody)._global(ibone).translation*-1));



							matrixnView V_i=v_all.range(c_pair*6, (c_pair+1)*6,0,a);
							V_i.mult(R_dAd, V_temp);

							// dotV_i =R_dAd*dotV_temp + dotR_dAd*V_temp
							RMat_assign(dot_R_dAd.range(0,3,3,6).lval(), Liegroup::skew(sim->getWorldVelocity(ibody, b, vector3(0,0,0))*-1));
							matrixnView dot_V_i=dot_v_all.range(c_pair*6, (c_pair+1)*6,0,a);
							dot_V_i.mult(dot_R_dAd, V_temp);
							dot_V_i+=R_dAd*dotV_temp;
						}
						c_pair++;
					}
				}
			}
		}
	}
}
void DynamicsSimulator_QP::calcContactBoneIndex(int link_pair_count, intvectorn& boneIndex)
{
	ConstrainedPoints_impl* solver=(ConstrainedPoints_impl*)_contacts;
	std::vector<ConstrainedPoints_impl::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	// contact-lcp.xoj page 3
	int a=solver->globalNumConstraintVectors+solver->globalNumFrictionVectors;


	
	DynamicsSimulator* sim=solver->sim;
	
	boneIndex.setSize(a);
	{
		int ichara=0;
		int N=sim->_characters[ichara]->skeleton->dofInfo.numActualDOF(); // excluding unused 4 th dimension.
		int c_pair=0;
		for (int i=0; i<n ; ++i){
			ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
			for (int ipair=0; ipair<2; ++ipair){
				VRMLTransform* link=linkPair->linkPair->link[ipair];
				if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
					std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
					int numConstraintPoints = constraintPoints.size();
					if (numConstraintPoints>0) {
						int ibody=linkPair->linkPair->charIndex[ipair];
						int ibone=link->treeIndex();
						//std::vector<BodyPart*>& cinfo=_cinfo[ibody]->_links;
						for(int k=0; k < numConstraintPoints; ++k){
							ConstrainedPoints_impl::ConstraintPoint& constraint = constraintPoints[k];
							int globalIndex = constraint.globalIndex;
							boneIndex[globalIndex]=ibone;
							for(int j=0; j < constraint.numFrictionVectors; ++j){
								int globalIndex=solver->globalNumConstraintVectors + constraint.globalFrictionIndex + j;
								boneIndex[globalIndex]=ibone;
							}
							
						}
					}
				}
			}
		}
	}

}

int DynamicsSimulator_QP::getNumContactLinkPairs() const
{
#ifdef USE_CONSTRAINED_POINTS
	ConstrainedPoints_impl* solver=(ConstrainedPoints_impl*)_contacts;
	std::vector<ConstrainedPoints_impl::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	int link_pair_count=0;
	for (int i=0; i<n ; ++i){
		ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			VRMLTransform* link=linkPair->linkPair->link[ipair];
			if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
				std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				if (numConstraintPoints>0) link_pair_count++;
			}
		}
	}
	return link_pair_count;
#else
	CFSImpl_LCP* solver=(CFSImpl_LCP*)AISTsim->world.contactForceSolver.impl;
	std::vector<CFSImpl_LCP::LinkPair*>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	int link_pair_count=0;
	for (int i=0; i<n ; ++i){
		CFSImpl_LCP::LinkPair* linkPair=clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			Link* link=linkPair->link[ipair];
			if (link->jointType!=Link::FIXED_JOINT ){
				CFSImpl_LCP::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				if (numConstraintPoints>0) link_pair_count++;
			}
		}
	}
	return link_pair_count;
#endif
}

// ys
void DynamicsSimulator_QP::getLinkPairBodiesBones(intvectorn& ibodies, intvectorn& ibones) const
{
#ifdef USE_CONSTRAINED_POINTS
	ConstrainedPoints_impl* solver=(ConstrainedPoints_impl*)_contacts;
	std::vector<ConstrainedPoints_impl::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	int link_pair_count=0;
	for (int i=0; i<n ; ++i){
		ConstrainedPoints_impl::ConstrainedLinkPair* linkPair=&clinks[i];
		for (int ipair=0; ipair<2; ++ipair){
			VRMLTransform* link=linkPair->linkPair->link[ipair];
			if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
				std::vector<ConstrainedPoints_impl::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();
				if (numConstraintPoints>0)
				{
				   	link_pair_count++;

					int ibody=linkPair->linkPair->charIndex[ipair];
					int ibone=link->treeIndex();
					ibodies.pushBack(ibody);
					ibones.pushBack(ibone);
				}
			}
		}
	}
#endif
}

