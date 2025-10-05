// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
/** @file DynamicsSimulator/server/DynamicsSimulator_Trbdl_penalty.cpp
 *
 */
#include "physicsLib.h"
#include "../TRL/eigenSupport.h"
#include "DynamicsSimulator.h"
#include "Body.h"
#include <vector>
#include <map>
#include <algorithm>

#include "DynamicsSimulator_Trbdl_penalty.h"
#include "BodyInfo.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/motion/VRMLloader_internal.h"

using namespace std;
#include <rbdl/Logging.h>



vectornView Trbdl::DynamicsSimulator_Trbdl_penalty::_Q(int ichara) { return vecView(bodyInfo(ichara).Q);}
vectornView Trbdl::DynamicsSimulator_Trbdl_penalty::_QDot(int ichara) { return vecView(bodyInfo(ichara).QDot);}
vector3 Trbdl::DynamicsSimulator_Trbdl_penalty::_bodyW(int ichara, int treeIndex) const { auto& bi=bodyInfo(ichara); return toVector3(bi.model.v[bi.links[treeIndex].bodyId], 0);}
vector3 Trbdl::DynamicsSimulator_Trbdl_penalty::_bodyV(int ichara, int treeIndex) const { auto& bi=bodyInfo(ichara); return toVector3(bi.model.v[bi.links[treeIndex].bodyId], 3);}

Trbdl::BodyInfo& Trbdl::DynamicsSimulator_Trbdl_penalty::bodyInfo(int ichara) { return *bodyInfoArray[ichara];}
const Trbdl::BodyInfo& Trbdl::DynamicsSimulator_Trbdl_penalty::bodyInfo(int ichara) const { return *bodyInfoArray[ichara];}
Trbdl::Link& Trbdl::DynamicsSimulator_Trbdl_penalty::getLink(int ichara, int ibone) { return bodyInfoArray[ichara]->links[ibone];}

const vectorn & Trbdl::DynamicsSimulator_Trbdl_penalty::getLastSimulatedPose(int ichara) const { 
	auto& bi=bodyInfo(ichara);
	if (!bi.lastSimulatedPoseCached)
	{
		((DynamicsSimulator*)(this))->getPoseDOF(ichara, _getLastSimulatedPose(ichara));
		bi.lastSimulatedPoseCached=true;
	}
	return _getLastSimulatedPose(ichara);
}
// #define INTEGRATOR_DEBUG
static const int debugMode = false;
static const bool enableTimeMeasure = false;

EIGEN_STRONG_INLINE Vector3d getAngularPart(SpatialVector const& v) { return Vector3d(v.data()[0], v.data()[1], v.data()[2]); }
EIGEN_STRONG_INLINE void setAngularPart(SpatialVector & v, Vector3d const& vv) { v[0]=vv[0]; v[1]=vv[1]; v[2]=vv[2]; }

EIGEN_STRONG_INLINE Vector3d getLinearPart(SpatialVector const& v) { return Vector3d(v.data()[3], v.data()[4], v.data()[5]); }
EIGEN_STRONG_INLINE void setLinearPart(SpatialVector & v, Vector3d const& vv) { v[3]=vv[0]; v[4]=vv[1]; v[5]=vv[2]; }

inline transf trl_transf(SpatialTransform const& X)
{
	transf out;
	// X.E is stored transposed, but Quaternion::fromMatrix considers that
	// when converting to Quaternion.
	out.rotation=Trbdl::toBase(Quaternion::fromMatrix(X.E));
	out.translation=toBase(X.r);
	return out;
}
void calcPointJacobian6D ( Model &model, const VectorNd &Q, unsigned int body_id, const Vector3d &point_position, MatrixNd &G, bool update_kinematics) {

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	SpatialTransform point_trans =
		SpatialTransform (Matrix3d::Identity(),
				CalcBodyToBaseCoordinates (model,
					Q,
					body_id,
					point_position,
					false));

	assert (G.rows() == 6 && G.cols() == model.qdot_size );

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		if(model.mJoints[j].mJointType != JointTypeCustom){
			if (model.mJoints[j].mDoFCount == 1) {
				G.block(0,q_index, 6, 1)
					= point_trans.apply( model.X_base[j].inverse().apply( model.S[j])).block(0,0,6,1);

				// all tested ok
				cout << j <<":"<< G.block(0,q_index, 6, 1).transpose() <<endl;
				cout <<"=="<<
					 ((point_trans * model.X_base[j].inverse()).toMatrix() * model.S[j]).transpose()<<endl;

				cout <<"=="<< trl_se3(model.S[j]).Ad(trl_transf(point_trans).inverse()*trl_transf(model.X_base[j])) <<endl;
				cout <<"=="<< (Liegroup::Ad(trl_transf(point_trans).inverse()*trl_transf(model.X_base[j]))*model.S[j]).transpose() <<endl;


			} else if (model.mJoints[j].mDoFCount == 3) {
				G.block(0, q_index, 6, 3)
					= ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(0,0,6,3);

				// all tested ok
				cout << j <<":"<< G.block(0, q_index, 6, 3).transpose()<<endl;
				cout <<"=="<< (Liegroup::Ad(trl_transf(point_trans).inverse()*trl_transf(model.X_base[j]))*model.multdof3_S[j]).block(0,0,6,3).transpose() <<endl;
			}
		} else {
			unsigned int k = model.mJoints[j].custom_joint_index;

			G.block(0, q_index, 6, model.mCustomJoints[k]->mDoFCount)
				= ((point_trans * model.X_base[j].inverse()).toMatrix() * model.mCustomJoints[k]->S).block( 0,0,6,model.mCustomJoints[k]->mDoFCount);
		}

		j = model.lambda[j];
	}
}
void calcPointJacobianDot6D(Model& model, const Math::VectorNd& Q, const Math::VectorNd& QDot, unsigned int body_id, const Vector3d& point_position, Math::MatrixNd& G, bool update_kinematics)
{
    // update the Kinematics if necessary
    if (update_kinematics)
    {
        UpdateKinematicsCustom(model, &Q, &QDot, nullptr);
    }

	SpatialTransform point_trans =
		SpatialTransform (Matrix3d::Identity(),
				CalcBodyToBaseCoordinates (model,
					Q,
					body_id,
					point_position,
					false));

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

    Math::SpatialVector v_p = -1. * CalcPointVelocity6D(model, Q, QDot, body_id, point_position, false);

    assert(G.rows() == 6 && G.cols() == model.qdot_size);

    unsigned int j = reference_body_id;

    while (j != 0)
    {
        if (model.mJoints[j].mJointType != JointTypeCustom)
        {
			transf X=trl_transf(point_trans).inverse()*trl_transf(model.X_base[j]);
			// X = X1 * X2
			// dotX= dotX1 *X2 + X1*dotX2
			//     = [ 0 v_p ]*[X2.r X2.p]+ X1*dotX2
			//       [ 0   0 ] [0       1]
			//     = [ 0 v_p ] + [ I  X1.p] [dotX2.r dotX2.p]
			//       [ 0   0 ]   [ 0    1 ] [ 0         0   ]
			//     = [ 0 v_p ] + [ dotX2.r dotX2.p]
			//       [ 0   0 ]   [   0          0 ] 
			matrix4 dotX;
			//dotX.mult(matrix4(trl_transf(point_trans).inverse()), calcDotT(trl_transf(model.X_base[j]), trl_se3(model.v[j]))); same
			dotX=calcDotT(trl_transf(model.X_base[j]), trl_se3(model.v[j]));
			dotX.leftMultTranslation(trlView(getLinearPart(v_p)));
			
            if (model.mJoints[j].mDoFCount == 1)
            {
				G.col(model.mJoints[j].q_index)= eigenView(trl_se3(model.S_o[j]).Ad(X))+Liegroup::dot_Ad(X, dotX)*model.S[j];
            }
            else if (model.mJoints[j].mDoFCount == 3)
            {
				G.block(0, model.mJoints[j].q_index, 6, 3)= Liegroup::Ad(X)*model.multdof3_S_o[j] + Liegroup::dot_Ad(X, dotX)*model.multdof3_S[j];
            }
        }
        else
        {
			Msg::error("not implemented yet");
        }

        j = model.lambda[j];
    }

}
#define FREE_JOINT_QSIZE 6
void Trbdl::BodyInfo::clearExternalForces()
{
	f_ext.resize(model.a.size());
	for(int i=0; i<f_ext.size(); i++)
		f_ext[i]=SpatialVector::Zero();
}
void Trbdl::BodyInfo::initialize()
{
	Q = VectorNd::Zero ((size_t) model.q_size);
	QDot = VectorNd::Zero ((size_t) model.qdot_size);
	QDDot = VectorNd::Zero ((size_t) model.qdot_size);
	Tau = VectorNd::Zero ((size_t) model.qdot_size);
	kps=VectorNd::Zero((size_t)model.qdot_size);
	kds=VectorNd::Zero((size_t)model.qdot_size);
	clearExternalForces();
}
void Trbdl::BodyInfo::integrateQuater(int Qindex, int jointId, double _timestep)
{
	ASSERT(model.mJoints[jointId].q_index==Qindex);

#if 0
	Quaternion q_t0=model.GetQuaternion(jointId, Q);

	Vector3d rel_ang_vel=QDot.segment<3>(Qindex);

	//cout <<"rel_ang_vel2"<<rel_ang_vel<<endl;

	Vector3d p_ang_vel=q_t0.conjugate().rotate(rel_ang_vel); // actually means (R_t1* rel_ang_vel).  rbdl math library uses really annoying convention!!!

	//cout <<"p_ang_vel"<<p_ang_vel<< "=="<< toBase(q_t0)*toBase(rel_ang_vel)<<endl;

	Quaternion q_t1=q_t0.timeStep(p_ang_vel, _timestep);
	q_t1=toRBDL(toBase(q_t1).normalized());

	model.SetQuaternion(jointId, q_t1, Q);

	rel_ang_vel=q_t1.rotate(p_ang_vel); // actually means (R_t1.inverse()) * p_ang_vel 
	QDot.segment<3>(Qindex)=rel_ang_vel;
#else

	Quaternion q0=model.GetQuaternion(jointId, Q);
    Vector3d omega(QDot.segment<3>(Qindex));

    Quaternion qd = q0.omegaToQDot(omega);
    Quaternion q1 = (q0 + qd * _timestep).normalized();
    ASSERT(!isnan(q1.squaredNorm()));


	model.SetQuaternion(jointId, q1, Q);
	QDot.segment<3>(Qindex)=omega;
#endif
}

Trbdl::DynamicsSimulator_Trbdl_penalty::DynamicsSimulator_Trbdl_penalty(const char* coldet)
:DynamicsSimulator_penaltyMethod(coldet)
{
	if(debugMode){
		cout << "DynamicsSimulator_Trbdl_penalty::DynamicsSimulator_Trbdl_penalty()" << endl;
	}

	_timestep=1e-3;
	_currentTime=0.0;
}


Trbdl::DynamicsSimulator_Trbdl_penalty::~DynamicsSimulator_Trbdl_penalty()
{
	if(debugMode){
		cout << "DynamicsSimulator_Trbdl_penalty::~DynamicsSimulator_Trbdl_penalty()" << endl;
	}
	for(int i=0; i<bodyInfoArray.size(); i++)
		delete bodyInfoArray[i];
}
void forwardKinematics (Trbdl:: BodyInfo& bi, Model &model, const VectorNd &Q, const VectorNd &QDot)
{
	unsigned int i = 0;

	// Reset the velocity of the dummy root body
	model.v[0]=bi.v0;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];

		// Basically: 
		//SpatialTransform X_J = SpatialTransform (model.GetQuaternion (joint_id, q).toMatrix(), Vector3d (0., 0., 0.));
		//model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
		//also updates V_j
		jcalc (model, i, Q, QDot);

		//The matrix i X p(i) ∈ R 6×6 is the spatial transformation matrix that transforms the basis of spatial vectors from that of frame p(i) into that of frame i.
		// 
		// X_J' = Ad(T(r,t))= [ r            0]
		//                    [ r*skew(-t)   r]
		//
		// SpatialTransform operator* (const SpatialTransform &XT) const { return SpatialTransform (E * XT.E, XT.r + XT.E.transpose() * r); }

		model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];

		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];
	}
}


// TRL 과 비교.
//				LOG << "X_base ("<<i<<"):"<<endl;          
//              LOG << "X.E = "<<trans(link->R)<<endl;    
//              LOG << "X.r = "<<link->p<<endl;          
//              LOG << "SpatialVelocity ("<<i<<") w "<< bodyV.W() << " v "<< bodyV.V()<<endl;
//              LOG << "w_v ("<<i<<") w "<< link->w << " v "<< link->vo<<endl;
//              LOG<< "c: "<<link->cw << link->cv<<endl;
//              LOG<< "Iww: "<<link->Iww << endl;
//              LOG<< "Iwv: "<<link->Iwv << endl;
//              LOG<< "Ivv: "<<link->Ivv << endl;
//              LOG<< "wIA =[Iww  Iwv]"<<endl<< "        [Iwv' Ivv]"<<endl;
//              LOG <<"pA: "<< Liegroup::dse3(link->ptau, link->pf).inv_dAd(X_base.inverse()) <<endl; // same
//              LOG <<"pA: "<< Liegroup::dse3(link->ptau, link->pf).dAd(X_base) <<endl;
//              LOG << "w_pA: "<<link->ptau << " pf: "<<link->pf<<endl;
//				위에까지 검증 완료.
//
//== calcABMphase1 ==
//       position and velocityFK: 
//               fk,
//               compute wc,
//               Iww,
//               Iwv
//               PF (bias force : velocity-related)
//
//       add gravity -> compute Ivv (mass), update PF
//
//== calcABMphase2 ==
//from child to parent
//
//2. 
//calc  hh, dd, uu:
//
//                       HH=I*S         (6x1)
//
//                       dd= S' * H + Jm2           -> scalar
//
//                           phase2   part1       part2
//                             uu = -H' * CV   - S'*PF             ---- (1)
//
//
//== calcABMPhase3 (root) ==
//from root to children
//            Articulated Inertia  
//
//                | Ivv  trans(Iwv) | * | dvo | = |- pf   |
//                | Iwv     Iww     |   | dw  |   |- ptau |
//
//       in short:               ->  I * DVO + PF =0,  where PF is the bias force.
// 
//
//       H =[hhv' hhw']'
//       H= IS = I * [sv' sw']'
//
// 
//               calcABMphase3 (non-root)
//               ddq = inv_dd*(uu - (dot(hhv, parent->dvo) + dot(hhw, parent->dw))) ,
//
//               so, in short, according to (1):
//
//               ddq = inv_dd*(- H' * CV - S' *[pf', ptau'] - H'*parent->DVO ) ;
//
//               dvo = parent->dvo + cv + sv_ddq;
//               dw  = parent->dw  + cw + sw_ddq;
//
//               -> 
//               DVO = parent->DVO + CV + S*ddq,
//               where CV= dS*dq
//
//
//               가속도 계산 완료. 적분하면 시뮬레이션 끝.
//
// 
//
//             void Link3::fk(Link* parent)
//               {
//                       if(parent){
//
//                               ASSERT(jointType==Link::SPHERE_JOINT);
//                               // see fk.cpp Joint::calc_position 
//                               R  = parent->R * rel_R;
//                               // see fk.cpp Joint::calc_velocity
//                               //
//                               // calc_velocity, where
//                               // w == R* loc_ang_vel
//                               // or
//                               // invR*w = loc_ang_vel
//                               //
//                               // thus, 
//                               // loc_ang_vel= inv_rel_R* parent->loc_ang_vel+ rel_ang_vel
//                               // 양변에 R곱하면->
//                               //  w = R*inv_rel_R*parent->loc_ang_vel + R*rel_ang_vel
//                               //    = parent->R * parent->loc_ang_vel + R*rel_ang_vel
//                               //    = parent-> w + R*rel_ang_vel
//
//                               vector3 arm;
//                               arm = parent->R * b;
//                               p  = arm  + parent->p;
//
//                               //  [sw, sv]=Ad(T) * [I, 0])
//                               //
//                               //  Ad(T)= [ R            0 ]
//                               //           skew(p)*R    R ]
//                               //sw=parent->R; // p_ang_vel
//                               sw=R; // rel_ang_vel
//                               sv=skew(p)*sw;
//
//                               sw_dq=sw* rel_ang_vel; // sw*dq
//                               sv_dq=cross(p, sw_dq);
//
//                               w  = parent->w + sw_dq;
//
//                               // v == R * loc_lin_vel
//                               vo = sv_dq + parent->vo;
//
//                               //  to verify..
//                               // loc_lin_vel=inv_rel_R* ( cross( parent->loc_ang_vel, b) + parent->loc_lin_vel )+ rel_lin_vel;
//                               // 의 양변에 R곱하면->
//                               // v = parent->R * cross( invParentR*parent->w, b) + parent-> v  + R*rel_lin_vel
//                               //         = parent->R * skew(invParentR*parent->w)*b + parent ->v  + R*rel_lin_vel
//                               //         =  skew(parent->w)* (parent->R*b) + parent->v + R*rel_lin_vel
//
//                               // but rel_lin_vel=0 here so
//
//                               //v = cross(parent->w, parent->R*b) + parent->v ;
//                               //vo= v - cross(w, p);
//                               //cout <<"vo" <<vo <<" == " << cross(p,  R* rel_ang_vel) +parent->vo << endl;
//
//                               cv = cross(parent->w, sv_dq)+cross(parent->vo, sw_dq);
//                               cw = cross(parent->w, sw_dq);
//                       }
//
//                       Link::fk(parent);
//               }
//
//
//Link3::calcABMPhase2Part1(){
//                       // M= [ Ivv Iwv']
//                       //    [ Iwv Iww ]
//
//                       // [hhv] = I  *   [sv]
//                       // [hhw]          [sw]
//      
//                       // or in RBDL, model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
//                       // (but in RBDL, IA=[Iww  Iwv] (Ivv is the mass, Iww is the inertia)
//                       //                  [Iwv' Ivv]
//                       //
//                       // in short, H = I*S
//                       //
//                       // DIFFERENCE: in RBDL, IA is in body coordinate.
//                       // so after conversion
//                       //      w_Ia -> X_base.toMatrixTranspose()* Ia *X_base.toMatrix()
//                       //                      = Ad(X_base)*Ia*Ad(X_base)'
//                       //      w_s -> Ad(T)*s
//                       //
//                       hhv = Ivv * sv + trans(Iwv) * sw;
//                       hhw = Iwv * sv + Iww * sw;
//
//                       dd = trans(sv)* hhv + trans(sw)*hhw ;
//                       dd._11+=Jm2;
//                       dd._22+=Jm2;
//                       dd._33+=Jm2;
//                       inv_dd.inverse(dd);
//#ifdef RBDL_ENABLE_LOGGING
//                       {
//                               LOG<<"w_S("<<dqIndex/3+1<<"):"<<sw << sv<<std::endl;
//                               transf X_base=transf(R.toQuater(), p);
//                               Liegroup::se3 S_column0=Liegroup::se3(vector3(sw._11, sw._21, sw._31), vector3(sv._11, sv._21, sv._31)).Ad(X_base.inverse());
//                               LOG << "S("<<dqIndex/3+1<<"):"<<S_column0<<endl;
//                               LOG<<"w_U("<<dqIndex/3+1<<"):"<<hhw << hhv<<std::endl;
//                               LOG<<"Dinv:"<<inv_dd<<std::endl;
//                               // 여기까지 모두 검증 완료.
//                       }
//#endif
//Link3::updateParentAInertia(Link* link) const 
//{ 
//
//       // [Iww  Iwv]-=[hhw] D_i [hhw' hhv']
//       // [Iwv' Ivv]  [hhv]
//       //
//       // in rbdl, Ia[lambda]+=X_lambda[i]'*(Ia-U*Dinv*U')*X_lambda[i]
//       // but here, everything is in the spatial coordinate system.
//       const matrix3& D_i=inv_dd;
//       link->Iww += Iww - hhw*D_i*trans(hhw);
//       link->Iwv += Iwv - hhw*D_i*trans(hhv);
//       link->Ivv += Ivv - hhv*D_i*trans(hhv);
//}
// TRL과 비교 끝.


// removed unnecessary codes for readability
void forwardDynamics (Trbdl::BodyInfo& bi,  Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, const VectorNd &K_i, VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA) {
	//LOG << "-------- " << __func__ << " --------" << std::endl;

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	// Reset the velocity of the root body
	model.v[0]=bi.v0;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

#if 1
		//modified by taesoo to use X_base[lambda]
		model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];
#else
		if (lambda != 0)
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		else
			model.X_base[i] = model.X_lambda[i];

		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];
#endif

		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
		model.I[i].setSpatialMatrix (model.IA[i]);

		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
			model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
		}
	}

	//LOG << "--- first loop ---" << std::endl;
	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 1
				&& model.mJoints[i].mJointType != JointTypeCustom) {

			model.U[i] = model.IA[i] * model.S[i];
			model.d[i] = model.S[i].dot(model.U[i]);
#if 1
			// equivalent inertia
			//model.d[i]+=MA;
			model.d[i]+=K_i[q_index]; // k_i
#endif
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia =    model.IA[i]
					- model.U[i]
					* (model.U[i] / model.d[i]).transpose();
				
				Ia.diagonal().array()+=MA;

				SpatialVector pa =  model.pA[i]
					+ Ia * model.c[i]
					+ model.U[i] * model.u[i] / model.d[i];

				model.IA[lambda].noalias()
					+= model.X_lambda[i].toMatrixTranspose()
					* Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias()
					+= model.X_lambda[i].applyTranspose(pa);
			}
		} else if (model.mJoints[i].mDoFCount == 3
				&& model.mJoints[i].mJointType != JointTypeCustom) {
			model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
#if 1
				// taesoo
				//
			Eigen::Matrix<double, 3,3> jm2=Eigen::Matrix<double, 3,3>::Identity(3,3)*
				(K_i[q_index]);

			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
					* model.multdof3_U[i]+jm2).inverse().eval();
#else
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() 
					* model.multdof3_U[i]).inverse().eval();
#endif
			Vector3d tau_temp(Tau.block(q_index,0,3,1));
			model.multdof3_u[i] = tau_temp
				- model.multdof3_S[i].transpose() * model.pA[i];

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i]
					- model.multdof3_U[i]
					* model.multdof3_Dinv[i]
					* model.multdof3_U[i].transpose();
				Ia.diagonal().array()+=MA;
				SpatialVector pa = model.pA[i]
					+ Ia
					* model.c[i]
					+ model.multdof3_U[i]
					* model.multdof3_Dinv[i]
					* model.multdof3_u[i];
				model.IA[lambda].noalias()
					+= model.X_lambda[i].toMatrixTranspose()
					* Ia
					* model.X_lambda[i].toMatrix();

				model.pA[lambda].noalias()
					+= model.X_lambda[i].applyTranspose(pa);
			}
		} else if (model.mJoints[i].mJointType == JointTypeCustom) {
			unsigned int kI   = model.mJoints[i].custom_joint_index;
			unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;
			model.mCustomJoints[kI]->U =
				model.IA[i] * model.mCustomJoints[kI]->S;

			model.mCustomJoints[kI]->Dinv
				= (model.mCustomJoints[kI]->S.transpose()
						* model.mCustomJoints[kI]->U).inverse().eval();
			VectorNd tau_temp(Tau.block(q_index,0,dofI,1));
			model.mCustomJoints[kI]->u = tau_temp
				- model.mCustomJoints[kI]->S.transpose() * model.pA[i];

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i]
					- (model.mCustomJoints[kI]->U
							* model.mCustomJoints[kI]->Dinv
							* model.mCustomJoints[kI]->U.transpose());
				Ia.diagonal().array()+=MA;
				SpatialVector pa =  model.pA[i]
					+ Ia * model.c[i]
					+ (model.mCustomJoints[kI]->U
							* model.mCustomJoints[kI]->Dinv
							* model.mCustomJoints[kI]->u);

				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
					* Ia
					* model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
			}
		}
	}

	model.a[0] = spatial_gravity * -1.+bi.a0;
	//cout<<bi.a0.transpose()<<endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];

		if (model.mJoints[i].mDoFCount == 1
				&& model.mJoints[i].mJointType != JointTypeCustom) {
			QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
			model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
		} else if (model.mJoints[i].mDoFCount == 3
				&& model.mJoints[i].mJointType != JointTypeCustom) {
			Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
			QDDot[q_index] = qdd_temp[0];
			QDDot[q_index + 1] = qdd_temp[1];
			QDDot[q_index + 2] = qdd_temp[2];
			model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
		} else if (model.mJoints[i].mJointType == JointTypeCustom) {
			unsigned int kI = model.mJoints[i].custom_joint_index;
			unsigned int dofI=model.mCustomJoints[kI]->mDoFCount;

			VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
				* (  model.mCustomJoints[kI]->u
						- model.mCustomJoints[kI]->U.transpose()
						* model.a[i]);

			for(unsigned int z=0; z < dofI; ++z){
				QDDot[q_index+z] = qdd_temp[z];
			}

			model.a[i] = model.a[i]
				+ model.mCustomJoints[kI]->S * qdd_temp;
		}
	}
	// restore.
	model.a[0] = bi.a0;
	// calculate accelerations only. 
	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

		if(model.mJoints[i].mJointType != JointTypeCustom){
			if (model.mJoints[i].mDoFCount == 1) {
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			} else if (model.mJoints[i].mDoFCount == 3) {
				Vector3d omegadot_temp (QDDot[q_index],
						QDDot[q_index + 1],
						QDDot[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			}
		} else {
			unsigned int custom_index = model.mJoints[i].custom_joint_index;
			const CustomJoint* custom_joint = model.mCustomJoints[custom_index];
			unsigned int joint_dof_count = custom_joint->mDoFCount;

			model.a[i] = model.a[i]
				+ ( model.mCustomJoints[custom_index]->S
						* QDDot.block(q_index, 0, joint_dof_count, 1));
		}
	}
}
void integratePos(VRMLloader& l, Trbdl::BodyInfo& bi, double _timestep)
{
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;
	int Qindex=0;
	// integrate
	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		auto& link=bi.links[i];
		//switch(b.HRPjointType(0))
		switch(b.mJoint->jointType)
		{
			case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					// translationl
					auto qd=bi.QDot.segment<3>(Qindex);
					auto q=bi.Q.segment<3>(Qindex);

					//cout <<"body w "<< bi.model.v[link.bodyId].segment<3>(0)<<endl;
					//cout <<"body v "<< bi.model.v[link.bodyId].segment<3>(3)<<endl;
					//cout <<"world v "<< qd<<endl;

					q+=_timestep*qd;

					bi.integrateQuater(Qindex+3, link.jointId, _timestep);
					// rotational
					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					bi.integrateQuater(Qindex, link.jointId, _timestep);
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) 
					{
						double& qd=bi.QDot[Qindex];
						double& q=bi.Q[Qindex];
						q+=_timestep*qd;
						Qindex++;
					}
				}
				break;
		}

	}
}

void integrate(VRMLloader& l, Trbdl::BodyInfo& bi, double _timestep)
{
    // semi-implicit eulers
    bi.QDot += _timestep * bi.QDDot;
	integratePos(l, bi, _timestep);
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::_registerCharacter
(
 const char *name,
 OpenHRP::CharacterInfo const& chara
 )
{
	collisionDetector->addModel(name, chara);

	_characters.push_back(new DynamicsSimulator::Character(chara.loader));

	int ichara=_characters.size()-1;

	bodyInfoArray.resize(ichara+1);
	bodyInfoArray[ichara]=new BodyInfo();
	auto& cinfo=*bodyInfoArray[ichara];

	cinfo.links.resize(chara.loader->numBone());

	auto& links=cinfo.links;

	//cout << "bone id, bodyId, jointId:"<<endl;

	int _lastJointId=-1;
	for(int i=1, ni=chara.loader->numBone(); i<ni; i++)
	{
		VRMLTransform& b=chara.loader->VRMLbone(i);
		// mass, com, inertia
		Matrix3d I=eigenView(b.momentsOfInertia());
		links[i].body=Body(b.mass(), eigenView(b.localCOM()), I);

		int numInternalJoints=1;
		switch(b.HRPjointType(0))
		{
			case HRP_JOINT::FREE:
				{
					OpenHRP::LinkInfo dummyLink;

					links[i].joint=Joint(JointTypeFloatingBase);
					int bodyId=cinfo.model.AddBody(0, Xtrans(Vector3d(0,0,0)),links[i].joint, links[i].body); 
					links[i].bodyId=bodyId;
					links[i].jointId=cinfo.model.mJoints.size()-1;
					numInternalJoints=2;
					_lastJointId=cinfo.model.mJoints.size()-1;
					//cout << "free bone "<< i<<"\t"<<bodyId<<"\t"<< links[i].jointId<<"\t"<<numInternalJoints<< endl;

					continue;
				}
				break;
			case HRP_JOINT::ROTATE:
				// yzx, zxy, x, zxy, y,
				if(b.getRotationalChannels()=="ZXY")
					links[i].joint=Joint(JointTypeEulerZXY);
				else if(b.getRotationalChannels()=="ZYX")
					links[i].joint=Joint(JointTypeEulerZYX);
				else if(b.getRotationalChannels()=="XYZ")
					links[i].joint=Joint(JointTypeEulerXYZ);
				else if(b.getRotationalChannels()=="YXZ")
					links[i].joint=Joint(JointTypeEulerYXZ);
				else if(b.getRotationalChannels()=="YZX")
				{
					links[i].joint=Joint(
							SpatialVector (0., 1., 0., 0., 0., 0.),
							SpatialVector (0., 0., 1., 0., 0., 0.),
							SpatialVector (1., 0., 0., 0., 0., 0.)
							);
					numInternalJoints=3;
				}
				else if(b.getRotationalChannels()=="YX")
				{
					links[i].joint=Joint(
							SpatialVector (0., 1., 0., 0., 0., 0.),
							SpatialVector (1., 0., 0., 0., 0., 0.)
							);
					numInternalJoints=2;
				}
				else if(b.getRotationalChannels()=="ZX")
				{
					links[i].joint=Joint(
							SpatialVector (0., 0., 1., 0., 0., 0.),
							SpatialVector (1., 0., 0., 0., 0., 0.)
							);
					numInternalJoints=2;
				}
				else if(b.getRotationalChannels()=="AA")
				{
					auto a1=b.getArbitraryAxis(0);
					auto a2=b.getArbitraryAxis(1);
					links[i].joint=Joint(
							SpatialVector (a1.x, a1.y, a1.z, 0., 0., 0.),
							SpatialVector (a2.x, a2.y, a2.z, 0., 0., 0.)
							);
					numInternalJoints=2;
				}
				else if(b.getRotationalChannels().length()==1)
				{
					char a=b.getRotationalChannels()[0];
					switch(a)
					{
						case 'X':
							links[i].joint=Joint(JointTypeRevoluteX);
							break;
						case 'Y':
							links[i].joint=Joint(JointTypeRevoluteY);
							break;
						case 'Z':
							links[i].joint=Joint(JointTypeRevoluteZ);
							break;
						case 'A':
							{
								links[i].joint=Joint(JointTypeRevoluteX);
								auto* joint=&links[i].joint;
								joint->mJointType=JointTypeRevolute;
								auto axis=b.getArbitraryAxis(0);
								joint->mJointAxes[0][0]=axis.x;
								joint->mJointAxes[0][1]=axis.y;
								joint->mJointAxes[0][2]=axis.z;
							}
							break;

					}
				}
				else
				{
					Msg::error("joint %s not implemented yet", b.getRotationalChannels().ptr());
				}
				break;
			case HRP_JOINT::FIXED:
				links[i].joint=Joint(JointTypeFixed);
				break;
			case HRP_JOINT::BALL:
				links[i].joint=Joint(JointTypeSpherical);
				break;
			default:
				ASSERT(false);
		}
		int pid=0;
		vector3 offset=b.getOffsetTransform().translation;
		if(i>1 )
		{
			pid=cinfo.links[b.parent()->treeIndex()].bodyId;
			//offset-=((VRMLTransform*)b.parent())->localCOM();
		}
		int bodyId=cinfo.model.AddBody(pid, Xtrans(eigenView(offset)),links[i].joint, links[i].body); 

		if(cinfo.model.IsFixedBodyId(bodyId)) bodyId-=cinfo.model.fixed_body_discriminator;

		links[i].bodyId=bodyId;
		links[i].jointId=cinfo.model.mJoints.size()-numInternalJoints;

		ASSERT(cinfo.model.mJoints.size()==_lastJointId+numInternalJoints+1);
		_lastJointId=cinfo.model.mJoints.size()-1;


		//cout << "bone "<< i<<"\t"<<bodyId<<"\t"<< links[i].jointId<<"\t"<<numInternalJoints<< endl;
	}


	if(chara.loader->VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED)
	{
		// somehow this is ignored, so it needs to be manually set.
		auto &bi=bodyInfo(ichara);
		bi.model.X_base[0].E=toRBDL(quater(1,0,0,0)).toMatrix();
		bi.model.X_base[0].r=eigenView(chara.loader->VRMLbone(1).getOffsetTransform().translation);
	}

	auto &bi=bodyInfo(ichara);
	bi.initialize();
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::_calcMassMatrix(int ichara, matrixn& out, vectorn & b)
{
	int totaldof=dof(ichara);
	out.setSize(totaldof, totaldof);
	b.setSize(totaldof);

	Math::MatrixNd H(totaldof, totaldof);
	Math::VectorNd C(totaldof);
	Math::VectorNd QDDot(totaldof);

	auto& bi=bodyInfo(ichara);
	bool update_kinematics = false; // always already updated.

	// we set QDDot to zero to compute C properly with the InverseDynamics
	// method.
	QDDot.setZero();

	InverseDynamics (bi.model, bi.Q, bi.QDot, QDDot, C, NULL);

	H.setZero();
	CompositeRigidBodyAlgorithm(bi.model, bi.Q, H, update_kinematics );
	eigenView(b)=C;
	eigenView(out)=H;

}
void Trbdl::DynamicsSimulator_Trbdl_penalty::_calcJacobianAt(int ichara, int ibone, matrixn& jacobian, vector3 const& localpos)
{
	auto& bodyi=bodyInfo(ichara);
	auto& link=bodyi.links[ibone];
	int bi=link.bodyId;
	int totaldof=dof(ichara);

	Math::MatrixNd G(6, totaldof);
	G.setZero();

	Vector3d pos=eigenView(localpos);
	CalcPointJacobian6D(bodyi.model, bodyi.Q, bi, pos, G, false); //original
	//calcPointJacobian6D(bodyi.model, bodyi.Q, bi, pos, G, false); // for testing

	jacobian.setSize(6, totaldof);
	eigenView(jacobian)=G;
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::_enableDotJocobianComputation(int ichara)
{
	auto& bodyi=bodyInfo(ichara);
	bodyi.model.calculate_S_o=true;
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::_calcDotJacobianAt(int ichara, int ibone, matrixn& dotjacobian, vector3 const& localpos)
{
	auto& bodyi=bodyInfo(ichara);
	auto& link=bodyi.links[ibone];
	int bi=link.bodyId;
	int totaldof=dof(ichara);

	Msg::verify(bodyi.model.calculate_S_o, "call _enableDotJocobianComputation and initSimulation first!!!");

	Math::MatrixNd G(6, totaldof);
	G.setZero();

	Vector3d pos=eigenView(localpos);
	calcPointJacobianDot6D(bodyi.model, bodyi.Q, bodyi.QDot, bi, pos, G, false);

	dotjacobian.setSize(6, totaldof);
	eigenView(dotjacobian)=G;
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::setTimestep(double ts)
{
	_timestep=ts;
}
double Trbdl::DynamicsSimulator_Trbdl_penalty::getTimestep()
{
	return _timestep;
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setCurrentTime(double t)
{
	_currentTime=t;
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::init(
		double timeStep,
		OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{	
	_timestep=timeStep;

	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		bodyInfo(i).model.gravity=eigenView(_gravity);
	}
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::initSimulation()
{
	if(debugMode){
		cout << "DynamicsSimulator_Trbdl_penalty::initSimulation()" << endl;
	}
#ifdef RBDL_ENABLE_LOGGING
	OutputToFile("output_rbdl.log", "::initSimulation");
#endif

	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		auto &bi=bodyInfo(i);

		forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
		bi.clearExternalForces();
	}

	_updateCharacterPose();
#ifdef RBDL_ENABLE_LOGGING
	OutputToFile("output_trl.log", LogOutput.str().c_str());
	ClearLogOutput();
	OutputToFile("output_rbdl.log", "::initSimulationEnd");
#endif
}

//#include "../BaseLib/utility/QPerformanceTimer.h"

void Trbdl::DynamicsSimulator_Trbdl_penalty::getBodyVelocity(int ichara, VRMLTransform* b, Liegroup::se3& V) const 
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	const auto& v=bi.model.v[link.bodyId];
	V.W()=toVector3(v,0);
	V.V()=toVector3(v,3);
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& velocity) const
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	const transf& T=getWorldState(ichara).global(*b);
	const auto& v=bi.model.v[link.bodyId];

	// Vp = Ad(SE3(R,0), Ad(Inv(SE3(Eye,p_)), V))
	//    = Ad(SE3(R,0) * Inv(SE3(Eye,p_)), V)
	//    = Ad(SE3(R, -R*p_), V)
	//    = se3(R*w, R*([w]*p + v))
	// where (w,v) = V, R = orientation of {body} w.r.t. {global}, and p_ = a position vector w.r.t. {body}
	velocity= T.rotation*(toVector3(v,0).cross(localpos)+toVector3(v,3));

	/* tested ok
	Vector3d l2=eigenView(localpos);
	Vector3d vel2=::CalcPointVelocity((Model&)bi.model, bi.Q, bi.QDot, link.bodyId, l2, true);

	velocity=::toBase(vel2);

	cout<< "worldpos:"<<		CalcBodyToBaseCoordinates ((Model&)bi.model, bi.Q, link.bodyId, l2, false)<<endl;
	*/
/* 
   all tested ok.
	Vector3d l2=eigenView(localpos);
	Vector3d vel2=CalcPointVelocity((Model&)bi.model, bi.Q, bi.QDot, link.bodyId, l2, true);
	cout << velocity << vel2<<endl;
	cout << "test1:"<<CalcPointVelocity((Model&)bi.model, bi.Q, bi.QDot, link.bodyId, Vector3d(0,0,0), true)
		<<"=="<<T.rotation*(toVector3(v,3))<<endl;
	cout << "test2:"<<CalcPointVelocity((Model&)bi.model, bi.Q, bi.QDot, link.bodyId, Vector3d(1,0,0), true)
		<<"=="<<T.rotation*(toVector3(v,0).cross(vector3(1,0,0))+toVector3(v,3))<<endl;
	cout <<"test3:"<<matrix3(T.rotation) <<"=="<<bi.model.X_base[link.bodyId].E.transpose()<<endl;
	cout <<"test4:"<<T.translation <<"=="<<bi.model.X_base[link.bodyId].r.transpose()<<endl;

	if (ichara==0) cout << "worldvel at "<<currentTime()<<" :"<<b->treeIndex()<<", "<<velocity << endl;
	*/
}	

void Trbdl::DynamicsSimulator_Trbdl_penalty::getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	const transf& T=getWorldState(ichara).global(*b);
	const auto & v=bi.model.v[link.bodyId];

	angvel= T.rotation*(toVector3(v,0));

	//if(b->treeIndex()==1) cout <<"root angvel"<< v.segment<3>(0)<<"=="<< bi.QDot.segment<3>(3)<<endl;

}

void Trbdl::DynamicsSimulator_Trbdl_penalty::getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	eigenView(acc)=calcPointAcceleration ( (RigidBodyDynamics::Model&)bi.model, link.bodyId, eigenView(localpos)) ;
	/* tested ok
	cout <<"acc" << acc <<" "<<link.bodyId<<"=="<< CalcPointAcceleration ( (RigidBodyDynamics::Model&)bi.model, bi.Q, bi.QDot, bi.QDDot, link.bodyId, eigenView(localpos), false) <<endl;
	//SO3 R(T_global.GetRotation());
	//Vec3 w(V.GetW()), v(V.GetV()), dw(dV.GetW()), dv(dV.GetV());
	//return R*(Cross(w,Cross(w,p_)) + Cross(w,v) + Cross(dw,p_) + dv);
	*/
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::addForceToBone
(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force)
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	auto& f_ext=bi.f_ext[link.bodyId];


	transf& T=getWorldState(ichara)._global(*b);
	::vector3 f=T.toGlobalDir(force);
	::vector3 p=T.toGlobalPos(localpos);

	//if (ichara==0) cout << "contact at "<<currentTime()<<" :"<<b->treeIndex()<<", "<<p <<", "<<f<<endl;

	f_ext.segment<3>(0)+=eigenView(p.cross(f));
	f_ext.segment<3>(3)+=eigenView(f);


	/* tested ok
	Liegroup::dse3 F0(p.cross(f), f);
	Liegroup::dse3 bodyF=F0.dAd(T);

	//if (ichara==0) cout << "force at "<<currentTime()<<" :"<<link.bodyId<<", "<<F0.M() <<", "<< F0.F();
	//if (ichara==0) cout << "body force at "<<currentTime()<<" :"<<link.bodyId<<", "<<bodyF.M() <<", "<< bodyF.F();
	*/
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::addGlobalForceToBone
(int ichara, int treeindex, ::vector3 const& p, ::vector3 const& f)
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[treeindex];
	auto& f_ext=bi.f_ext[link.bodyId];

	f_ext.segment<3>(0)+=eigenView(p.cross(f));
	f_ext.segment<3>(3)+=eigenView(f);
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::addWorldTorqueToBone(int ichara, VRMLTransform* b, ::vector3 const& world_torque)
{
	auto& bi=bodyInfo(ichara);
	auto& link=bi.links[b->treeIndex()];
	transf& T=getWorldState(ichara)._global(*b);
	auto& f_ext=bi.f_ext[link.bodyId];
	f_ext.segment<3>(0)+=eigenView(world_torque);
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::_stepKinematic(int ichara, vectorn const& QDDot)
{

	VRMLloader& l=skeleton(ichara);
	auto& bi=bodyInfo(ichara);
	// clear external forces
	bi.clearExternalForces();

	bi.QDDot=eigenView(QDDot);
	integrate(l, bi,_timestep);
	forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
	_currentTime+=_timestep;
	_updateCharacterPose();

}
bool Trbdl::DynamicsSimulator_Trbdl_penalty::stepSimulation()
{
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);

	_contactForces.clear();
	_calcContactForce(*collisions);


	// set external forces
	for(int i=0; i<_contactForces.size(); i++)
	{		
		ContactForce& c=_contactForces[i];
		addForceToBone(c.chara, c.bone,c.p,c.f);

		// Trbdl uses inertial frame for external forces.
		::vector3 gtau=getWorldState(c.chara)._global(*c.bone).toGlobalDir(c.tau);
		addWorldTorqueToBone(c.chara, c.bone, gtau);
	}

	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);

		auto& bi=bodyInfo(ichara);

		// CalcNextState
		//ForwardDynamics (bi.model, bi.Q, bi.QDot, bi.Tau, bi.QDDot, &bi.f_ext); // original code
		//ForwardDynamicsLagrangian (bi.model, bi.Q, bi.QDot, bi.Tau, bi.QDDot, Math::LinearSolverPartialPivLU, &bi.f_ext); // original code
		forwardDynamics (bi, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_timestep, bi.QDDot, &bi.f_ext, 0.05); // modified code

		// clear external forces
		bi.clearExternalForces();
		
		integrate(l, bi,_timestep);

		forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
	}
	_currentTime+=_timestep;


#ifdef RBDL_ENABLE_LOGGING
	OutputToFile("output_rbdl.log", "::stepSimulation");
	OutputToFile("output_rbdl.log", LogOutput.str().c_str());
	ClearLogOutput();
#endif

	_updateCharacterPose();
	return true;
}




void Trbdl::DynamicsSimulator_Trbdl_penalty::setGVector
(
 const ::vector3& wdata
 )
{
	_gravity=-1*wdata;

	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		bodyInfo(i).model.gravity=eigenView(_gravity);
	}
}


void Trbdl::DynamicsSimulator_Trbdl_penalty::getGVector
(
 ::vector3& g
 )
{
	g = -1*_gravity;
}


// output is compatible to MotionDOF class.
void Trbdl::DynamicsSimulator_Trbdl_penalty::getLinkData(int ichara, LinkDataType t, vectorn& out)
{
	VRMLloader& l=skeleton(ichara);
	int ndof=l.dofInfo.numDOF();
	out.setSize(ndof);
	if (t==OpenHRP::DynamicsSimulator::JOINT_VALUE)
	{
		int qindex=0;
		int Qindex=0;
		auto& bi=bodyInfo(ichara);
		for(int i=1; i<l.numBone(); i++)
		{
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			auto& link=bi.links[i];
			switch(b.mJoint->jointType)
			{
				case HRP_JOINT::FIXED: 
					continue;
				case HRP_JOINT::FREE:
				{
					eigenView(out).segment<3>(qindex)=bi.Q.segment<3>(Qindex);
					out.setQuater(qindex+3, toBase(bi.model.GetQuaternion(link.jointId, bi.Q)));


					qindex+=7;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
				case HRP_JOINT::BALL:
				{
					out.setQuater(qindex, toBase(bi.model.GetQuaternion(link.jointId, bi.Q)));
					qindex+=4;
					Qindex+=3;
				}
				break;
				case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) out[i+qindex]=bi.Q[i+Qindex];
					Qindex+=nq;
					qindex+=nq;
				}
				break;
			}
		}
	}
	else if(t==OpenHRP::DynamicsSimulator::JOINT_VELOCITY)
	{
		int qindex=0;
		int Qindex=0;
		auto& bi=bodyInfo(ichara);
		for(int i=1; i<l.numBone(); i++)
		{
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			auto& link=bi.links[i];
			switch(b.mJoint->jointType)
			{
				case HRP_JOINT::FIXED: 
					continue;
				case HRP_JOINT::FREE:
				{
					quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));
					out.setVec3(qindex, R.inverse()*toVector3(bi.QDot, Qindex));
					out.setVec3(qindex+4, toVector3(bi.QDot, Qindex+3));

					qindex+=7;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
				case HRP_JOINT::BALL:
				{
					eigenView(out.range(qindex+1, qindex+4))=bi.QDot.segment<3>(Qindex);
					qindex+=4;
					Qindex+=3;
				}
				break;
				case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) out[i+qindex]=bi.QDot[i+Qindex];
					Qindex+=nq;
					qindex+=nq;
				}
				break;
			}
		}
	}
	else
	ASSERT(false);
}

// output is compatible to MotionDOF class.
void Trbdl::DynamicsSimulator_Trbdl_penalty::setLinkData(int ichara, LinkDataType t, vectorn const& in)
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int ndof=l.dofInfo.numDOF();

	if (t==OpenHRP::DynamicsSimulator::JOINT_VALUE)
	{
		int qindex=0;
		int Qindex=0;
		auto& bi=bodyInfo(ichara);
		for(int i=1; i<l.numBone(); i++)
		{
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			auto& link=bi.links[i];
			switch(b.mJoint->jointType)
			{
				case HRP_JOINT::FIXED: 
					continue;
				case HRP_JOINT::FREE:
				{
					bi.Q.segment<3>(Qindex)=eigenView(in).segment<3>(qindex);
					bi.model.SetQuaternion(link.jointId, toRBDL(in.toQuater(qindex+3)), bi.Q);

					qindex+=7;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
				case HRP_JOINT::BALL:
				{
					bi.model.SetQuaternion(link.jointId, toRBDL(in.toQuater(qindex)), bi.Q);
					qindex+=4;
					Qindex+=3;
				}
				break;
				case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) bi.Q[i+Qindex]=in[i+qindex];
					Qindex+=nq;
					qindex+=nq;
				}
				break;
			}
		}
	}
	else if(t==OpenHRP::DynamicsSimulator::JOINT_VELOCITY)
	{
		int qindex=0;
		int Qindex=0;
		auto& bi=bodyInfo(ichara);
		for(int i=1; i<l.numBone(); i++)
		{
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			auto& link=bi.links[i];
			switch(b.mJoint->jointType)
			{
				case HRP_JOINT::FIXED: 
					continue;
				case HRP_JOINT::FREE:
				{
					quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));

					bi.QDot.segment<3>(Qindex)=toEigen(R*in.toVector3(0));
					bi.QDot.segment<3>(Qindex+3)=toEigen(in.toVector3(4));
					qindex+=7;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
				case HRP_JOINT::BALL:
				{
					bi.QDot.segment<3>(Qindex)=eigenView(in.range(qindex+1, qindex+4));
					qindex+=4;
					Qindex+=3;
				}
				break;
				case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) bi.QDot[i+Qindex]=in[i+qindex];
					Qindex+=nq;
					qindex+=nq;
				}
				break;
			}
		}
	}
	else if(t==OpenHRP::DynamicsSimulator::JOINT_TORQUE)
	{
		int qindex=0;
		int Qindex=0;
		auto& bi=bodyInfo(ichara);
		for(int i=1; i<l.numBone(); i++)
		{
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			auto& link=bi.links[i];
			switch(b.mJoint->jointType)
			{
				case HRP_JOINT::FIXED: 
					continue;
				case HRP_JOINT::FREE:
				{
					bi.Tau.segment<3>(Qindex)=eigenView(in).segment<3>(qindex);
					bi.Tau.segment<3>(Qindex+3)=eigenView(in).segment<3>(qindex+4);

					qindex+=7;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
				case HRP_JOINT::BALL:
				{
					bi.QDot.segment<3>(Qindex)=eigenView(in.range(qindex+1, qindex+4));
					qindex+=4;
					Qindex+=3;
				}
				break;
				case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(i);
					for(int i=0; i<nq; i++) bi.Tau[i+Qindex]=in[i+qindex];
					Qindex+=nq;
					qindex+=nq;
				}
				break;
			}
		}
	}
}

double Trbdl::DynamicsSimulator_Trbdl_penalty::currentTime() const
{
	return _currentTime;
}


void Trbdl::DynamicsSimulator_Trbdl_penalty::_updateCharacterPose()
{
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{

		VRMLloader& l=skeleton(ichara);
		auto& bodyi=bodyInfo(ichara);
		bodyi.lastSimulatedPoseCached=false;
		auto& chain=_characters[ichara]->chain;
		for(int i=1; i<l.numBone(); i++)
		{
			auto& link=bodyi.links[i];
			int bi=link.bodyId;
			bodyi._getGlobalFrame(bi, chain->_global(i));
		}
		chain->inverseKinematicsExact();
		/*
		// all test passed
		for(int i=2; i<l.numBone(); i++)
		{
		auto& link=bodyi.links[i];
		int bi=link.bodyId;
		Quaternion q=Quaternion::fromMatrix(bodyi.model.X_lambda[bi].E);

		Quaternion q2=Quaternion::fromMatrix(bodyi.model.X_base[bodyi.model.lambda[bi]].E);
		Quaternion q3=Quaternion::fromMatrix(bodyi.model.X_base[bi].E);
		cout << "test1 bone "<<i<<" :"<<toBase(q) <<"\t=="<< chain->_local(i).rotation << endl;
		cout << "test2 bone "<<i<<" :"<<toBase(q2)<<"\t=="<< chain->_global(l.bone(i).parent()->treeIndex()).rotation<<endl;
		cout << "test3 bone "<<i<<" :"<<toBase(q2)*(toBase(q))<<"\t=="<<toBase(q3)<<"=="<<	chain->_global(i).rotation << endl;
		cout << "test4 bone "<<i<<" :"<<(chain->_global(l.bone(i).parent()->treeIndex()).rotation)*(chain->_local(i).rotation)<<"=="<<toBase(q3)<<"=="<<	chain->_global(i).rotation << endl;

		// E is stored transposed, but it automatically recovers itself when converting to Quaternion.
		cout << bodyi.model.X_base[bi].E <<endl << toOpenHRP(toBase(q3))<<endl;
		}
		cout<<"Q"<<bodyi.Q.transpose()<<endl;
		*/
	}
}


int Trbdl::DynamicsSimulator_Trbdl_penalty::getSphericalState(int ichara, vectorn & q, vectorn& dq)
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int qindex=0;
	int qsindex=ndof-nquat*4;
	int dqsindex=qsindex;
	int Qindex=0;

	q.setSize(ndof);
	dq.setSize(qsindex+nquat*3);

	auto& bi=bodyInfo(ichara);
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
		 	case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					// translation
					quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));
					eigenView(q).segment<3>(qindex)=bi.Q.segment<3>(Qindex);
					dq.setVec3(qindex,R.inverse()*toVector3(bi.QDot, Qindex)); // dq : body linear vel
					qindex+=3;
					

					// rotational
					q.setQuater(qsindex, R);
					dq.setVec3(dqsindex,toVector3(bi.QDot, Qindex+3)); 
					qsindex+=4;
					dqsindex+=3;


					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					q.setQuater(qsindex, toBase(bi.model.GetQuaternion(link.jointId, bi.Q)));
					qsindex+=4;


					eigenView(dq).segment<3>(dqsindex)=bi.QDot.segment<3>(Qindex);
					dqsindex+=3;

					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						q[qindex]=bi.Q[Qindex];
						dq[qindex]=bi.QDot[Qindex];
						Qindex++;
						qindex++;
					}
				}
				break;
		}

	}

	ASSERT(Qindex==bi.QDot.size());
	ASSERT(qindex==ndof-nquat*4); 
	ASSERT(qsindex==q.size());
	ASSERT(dqsindex==dq.size());
	return qindex;
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setSphericalState(int ichara, const vectorn& q, const vectorn& dq) // packing is different from setLinkData or setQ/setDQ
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int qindex=0; // == dqindex (linear joints )
	int qsindex=ndof-nquat*4; // spherical joints (4씩증가)
	int dqsindex=qsindex; // 3씩 증가.
	int Qindex=0;

	auto& bi=bodyInfo(ichara);
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
		 	case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					// translation
					quater R=q.toQuater(qsindex);
					bi.Q.segment<3>(Qindex)=eigenView(q).segment<3>(qindex);
					bi.QDot.segment<3>(Qindex)=toEigen(R*dq.toVector3(qindex)); // QDot: world linear velocity
					qindex+=3;

					// rotation
					bi.model.SetQuaternion(link.jointId, toRBDL(R), bi.Q);
					bi.QDot.segment<3>(Qindex+3)=toEigen(dq.toVector3(dqsindex)); // QDot: body angular velocity
					qsindex+=4;
					dqsindex+=3;

					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
			case HRP_JOINT::BALL:
				{
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					bi.model.SetQuaternion(link.jointId, toRBDL(q.toQuater(qsindex)), bi.Q);
					qsindex+=4;

					bi.QDot.segment<3>(Qindex)=eigenView(dq).segment<3>(dqsindex);
					dqsindex+=3;
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						bi.Q[Qindex]=q[qindex];
						bi.QDot[Qindex]=dq[qindex];
						Qindex++;
						qindex++;
					}
				}
				break;
		}
	}

	ASSERT(Qindex==bi.QDot.size());
	ASSERT(qindex==ndof-nquat*4);
	Msg::verify(qsindex==q.size(), "incorrect input: setSphericalState of character %d", ichara );
	ASSERT(dqsindex==dq.size());
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setTau(int ichara, const vectorn& tau) // packing is different from setLinkData or setU
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int qindex=0;
	int dqsindex=ndof-nquat*4; // start of spherical joints
	int Qindex=0;

	auto& bi=bodyInfo(ichara);
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
		 	case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					dqsindex+=3;
					qindex+=3;
					Qindex+=FREE_JOINT_QSIZE;
				}
				break;
			case HRP_JOINT::BALL:
				{
					bi.Tau.segment<3>(Qindex)=eigenView(tau.toVector3(dqsindex));
					dqsindex+=3;
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					for(int i=0; i<nq; i++) 
					{
						bi.Tau[Qindex]=tau[qindex];
						Qindex++;
						qindex++;
					}
				}
				break;
		}
	}

	ASSERT(Qindex==bi.Tau.size());
	ASSERT(qindex==ndof-nquat*4);
	ASSERT(dqsindex==tau.size());
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setNonStatePoseDOF(int ichara, vectorn const& v)
{
	VRMLloader& l=skeleton(ichara);

	if(l.VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED)
	{
		auto &bi=bodyInfo(ichara);
		bi.model.X_base[0].E=toRBDL(v.toQuater(3)).toMatrix();
		bi.model.X_base[0].r=eigenView(v.toVector3(0));
	}
 	else
		Msg::error("setNonStateQ???");
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::setNonStateDQ(int ichara, vectorn const& dq)
{
	VRMLloader& l=skeleton(ichara);

	if(l.VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED)
	{
		auto &bi=bodyInfo(ichara);
		quater invR=toBase(Quaternion::fromMatrix(bi.model.X_base[0].E)).inverse();
		bi.v0.segment<3>(0)=eigenView(invR*dq.toVector3(0));
		bi.v0.segment<3>(3)=eigenView(invR*dq.toVector3(3));
	}
 	else
		Msg::error("setNonStateDQ???");
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setNonStateDDQ(int ichara, vectorn const& ddq)
{
	VRMLloader& l=skeleton(ichara);

	if(l.VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED)
	{
		auto &bi=bodyInfo(ichara);
		quater invR=toBase(Quaternion::fromMatrix(bi.model.X_base[0].E)).inverse();
		bi.a0.segment<3>(0)=eigenView(invR*ddq.toVector3(0));
		bi.a0.segment<3>(3)=eigenView(invR*ddq.toVector3(3));
	}
 	else
		Msg::error("setNonStateDDQ???");
}
transf Trbdl::DynamicsSimulator_Trbdl_penalty::getNonStateRootQ(int ichara)
{
	transf tf;
	VRMLloader& l=skeleton(ichara);

	if(l.VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED)
	{
		auto &bi=bodyInfo(ichara);
		tf.rotation=toBase(Quaternion::fromMatrix(bi.model.X_base[0].E));
		tf.translation=toBase(bi.model.X_base[0].r);
	}
 	else
		Msg::error("getNonStateQ???");
	return tf;
}



void Trbdl::DynamicsSimulator_Trbdl_penalty::getQ(int ichara, vectorn & q) const
{
	// packing is different from setLinkData or setQ/setD
	const VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int Qindex=0;

	int root_w_index=-1;
	if( l.VRMLbone(1).mJoint->jointType==HRP_JOINT::FREE)
	{
		q.setSize(ndof-nquat+1);
		root_w_index=ndof-nquat;
	}
	else
		q.setSize(ndof-nquat);

	auto& bi=bodyInfo(ichara);
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
		 	case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					// translation
					quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));
					eigenView(q).segment<3>(Qindex)=bi.Q.segment<3>(Qindex);
					
					// rotational
					q[root_w_index]=R.w;
					q[3]=R.x;
					q[4]=R.y;
					q[5]=R.z;
					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					q.setVec3(Qindex, toBase(bi.model.GetQuaternion(link.jointId, bi.Q)).rotationVector());
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						q[Qindex]=bi.Q[Qindex];
						Qindex++;
					}
				}
				break;
		}

	}

	ASSERT(Qindex==bi.QDot.size());
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::setQ(int ichara, const vectorn & q)
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();
	int Qindex=0;

	int root_w_index=ndof-nquat;

	auto& bi=bodyInfo(ichara);
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
		 	case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					// translation
					quater R;
					R.w=q[root_w_index];
					R.x=q[3];
					R.y=q[4];
					R.z=q[5];
					bi.Q.segment<3>(0)=toEigen(q.toVector3(0));
					bi.model.SetQuaternion(link.jointId, toRBDL(R), bi.Q);
					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					quater R;
					R.setRotation(q.toVector3(Qindex));
					bi.model.SetQuaternion(link.jointId, toRBDL(R), bi.Q);
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						bi.Q[Qindex]=q[Qindex];
						Qindex++;
					}
				}
				break;
		}

	}

	ASSERT(Qindex==bi.QDot.size());
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::getDQ(int ichara, vectorn & dq) const
{
	// packing is different from setLinkData or setQ/setD
	const VRMLloader& l=skeleton(ichara);
	dq.setSize(l.dofInfo.numActualDOF());
	auto &bi=bodyInfo(ichara);
	eigenView(dq)=bi.QDot;
	ASSERT(l.dofInfo.numActualDOF()==bi.QDot.size());

	if( l.VRMLbone(1).mJoint->jointType==HRP_JOINT::FREE)
	{
		int ibone=1;
		auto& link=bi.links[ibone];

		// swap 0 and 3
		vector3 w=dq.toVector3(3);
		
		// linear velocity (at 3) is already in global coordinate.
		dq.setVec3(3, dq.toVector3(0));

		quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));
		dq.setVec3(0, R*w); // body angular velocity to global
	}
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::setDQ(int ichara, const vectorn & dq)
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	auto &bi=bodyInfo(ichara);
	bi.QDot=eigenView(dq);
	ASSERT(l.dofInfo.numActualDOF()==bi.QDot.size());
	ASSERT(l.dofInfo.numActualDOF()==dq.size());

	if( l.VRMLbone(1).mJoint->jointType==HRP_JOINT::FREE)
	{
		int ibone=1;
		auto& link=bi.links[ibone];
		quater R=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));
		// linear velocity is already in global coordinate.
		Vector3d v=bi.QDot.segment<3>(3);
		// swap 0 and 3
		
		bi.QDot.segment<3>(0)=v;
		bi.QDot.segment<3>(3)=toEigen(R.inverse()*dq.toVector3(0));// world ang-vel to body.
	}
}

void Trbdl::DynamicsSimulator_Trbdl_penalty::setU(int ichara, const vectorn& U) 
{
	VRMLloader& l=skeleton(ichara);
	auto &bi=bodyInfo(ichara);
	bi.Tau=eigenView(U);
}
	
void Trbdl::DynamicsSimulator_Trbdl_penalty::setStablePDparam(int ichara, const vectorn& kp, const vectorn& kd)
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	auto &bi=bodyInfo(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();

	int qindex=0; // == dqindex
	int dqsindex=ndof-nquat*4;
	int Qindex=0; // cacheIndex

	bi.kps.resize(ndof-nquat);
	bi.kds.resize(ndof-nquat);

	auto& kps=bi.kps;
	auto& kds=bi.kds;

	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
			case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					qindex+=3;
					dqsindex+=3;

					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					kps[Qindex]=kp(dqsindex);
					kps[Qindex+1]=kp(dqsindex+1);
					kps[Qindex+2]=kp(dqsindex+2);

					kds[Qindex]=kd(dqsindex);
					kds[Qindex+1]=kd(dqsindex+1);
					kds[Qindex+2]=kd(dqsindex+2);

					dqsindex+=3;
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						kps[Qindex]=kp(qindex);
						kds[Qindex]=kd(qindex);
						qindex+=1;
						Qindex+=1;
					}
				}
				break;
		}
	}
	ASSERT(bi.kps.size()==Qindex);
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::calculateStablePDForces(int ichara, const vectorn& desired_q, vectorn & tau, bool applyRootExternalForce )
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();

	int nActualDof = ndof-nquat ;

	auto& bi=bodyInfo(ichara);
    auto& positions = bi.Q;
    auto& velocities = bi.QDot;

    vectorn proportionalTorquePlusQDotDeltaT(nActualDof);
	proportionalTorquePlusQDotDeltaT.setAllValue(0.0);
    vectorn derivativeTorque(nActualDof);
	derivativeTorque.setAllValue(0.0);

	int qindex=0;
	int qsindex=ndof-nquat*4;
	int dqsindex=qsindex;
	int Qindex=0;

	auto& kps=bi.kps;
	auto& kds=bi.kds;
	Msg::verify(kps.size()==ndof-nquat, "call setStablePDparam first");
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
			case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					qindex+=3;
					qsindex+=4;
					dqsindex+=3;

					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					auto targetPosition=desired_q.toQuater(qsindex);

					int cacheIndex=Qindex;
					float timeStep=_timestep;

					targetPosition.normalize(); // Never trust user input

					vector3 kp(
							kps[cacheIndex],
							kps[cacheIndex + 1],
							kps[cacheIndex + 2]
							);

					quater localRotation=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));

					quater posDifference = targetPosition * localRotation.inverse();
					posDifference.normalize();

					vector3 axis=posDifference.rotationVector();

					vector3 proportionalForceInParentFrame(
							kps[cacheIndex] * axis.x,
							kps[cacheIndex + 1] * axis.y,
							kps[cacheIndex + 2] * axis.z
							);
					vector3 proportionalForceInChildFrame = localRotation.inverse()*proportionalForceInParentFrame;

					proportionalTorquePlusQDotDeltaT[dqsindex] = proportionalForceInChildFrame[0] - 
						timeStep * velocities[cacheIndex] * kps[cacheIndex];
					proportionalTorquePlusQDotDeltaT[dqsindex + 1] = proportionalForceInChildFrame[1] -
						timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
					proportionalTorquePlusQDotDeltaT[dqsindex + 2] = proportionalForceInChildFrame[2] -
						timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

					derivativeTorque[dqsindex] = -kds[cacheIndex] * velocities[cacheIndex];
					derivativeTorque[dqsindex + 1] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
					derivativeTorque[dqsindex + 2] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];
					qsindex+=4;
					dqsindex+=3;
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						int cacheIndex=Qindex;
						proportionalTorquePlusQDotDeltaT[qindex] = kps[cacheIndex] * (
								desired_q[qindex] - positions[cacheIndex] - 
								_timestep * velocities[cacheIndex]
								);
						derivativeTorque[qindex] = -kds[cacheIndex] * velocities[cacheIndex];
						qindex++;
						Qindex++;
					}
				}
				break;
		}
	}

	ASSERT(Qindex==bi.QDot.size());
	ASSERT(qindex==ndof-nquat*4);

	tau= proportionalTorquePlusQDotDeltaT + derivativeTorque;
	tau.range(0,3).setAllValue(0.0);
	tau.range(qindex,qindex+3).setAllValue(0.0);



    if (applyRootExternalForce) {
		Msg::error("applyRootExternalForce not ported yet");
		ASSERT(false); // not ported yet.
	/*
        vector<float> rootForcePD(6, 0);

        PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
        PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;
        UniformQuaternion(rootGlobalRotation);

        PxVec3 rootGlobalLinearVelocity = rootLink->link->getLinearVelocity();
        PxVec3 rootGlobalAngularVelocity = rootLink->link->getAngularVelocity();
        
        PxVec3 rootGlobalProportionalLinearForcePlusQDotDeltaT(
            root_kps[0] * (targetPositions[0] - rootGlobalPosition[0] - timeStep * rootGlobalLinearVelocity[0]),
            root_kps[1] * (targetPositions[1] - rootGlobalPosition[1] - timeStep * rootGlobalLinearVelocity[1]),
            root_kps[2] * (targetPositions[2] - rootGlobalPosition[2] - timeStep * rootGlobalLinearVelocity[2])
        );
        PxVec3 rootGlobalDerivativeLinearForce(
            -root_kds[0] * rootGlobalLinearVelocity[0],
            -root_kds[1] * rootGlobalLinearVelocity[1],
            -root_kds[2] * rootGlobalLinearVelocity[2]
        );

        PxQuat rootGlobalTargetRotationUser(targetPositions[4], targetPositions[5], targetPositions[6], targetPositions[3]);
        rootGlobalTargetRotationUser.normalize();
        
        // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
        PxQuat rootGlobalTargetRotation = rootGlobalTargetRotationUser * frameTransform;

        PxQuat diffQuat = rootGlobalTargetRotation * rootGlobalRotation.getConjugate();
        if (PxAbs(diffQuat.w) < 0.70710678118f) {
            diffQuat = (-rootGlobalTargetRotation) * rootGlobalRotation.getConjugate();
        }
        UniformQuaternion(diffQuat);

        PxVec3 diffRotExpMapGlobal = QuatToExpMap(diffQuat);
        PxVec3 rootGlobalProportionalTorquePlusQDotDeltaT(
            root_kps[3] * (diffRotExpMapGlobal[0] - timeStep * rootGlobalAngularVelocity[0]),
            root_kps[4] * (diffRotExpMapGlobal[1] - timeStep * rootGlobalAngularVelocity[1]),
            root_kps[5] * (diffRotExpMapGlobal[2] - timeStep * rootGlobalAngularVelocity[2])
        );
        PxVec3 rootGlobalDerivativeTorque(
            -root_kds[3] * rootGlobalAngularVelocity[0],
            -root_kds[4] * rootGlobalAngularVelocity[1],
            -root_kds[5] * rootGlobalAngularVelocity[2]
        );

        for (int i = 0; i < 3; i++) {
            rootForcePD[i] += rootGlobalProportionalLinearForcePlusQDotDeltaT[i] + rootGlobalDerivativeLinearForce[i];
        }
        for (int i = 0; i < 3; i++) {
            rootForcePD[i + 3] += rootGlobalProportionalTorquePlusQDotDeltaT[i] + rootGlobalDerivativeTorque[i];
        }

#ifdef ENABLE_SPD_ABA
        extern bool g_ApplyABARootForce;
        extern float g_RootExternalSpatialForce[100][6];
        extern const float* g_ABA_Root_Kd;

        g_ApplyABARootForce = true;
        g_ABA_Root_Kd = root_kds.data();
        memcpy(g_RootExternalSpatialForce[_id], rootForcePD.data(), 6 * sizeof(float));
#endif
*/
    }
}
void Trbdl::DynamicsSimulator_Trbdl_penalty::calculateStablePDForces(int ichara, const vectorn& desired_q, const vectorn& desired_dq, vectorn & tau, bool applyRootExternalForce )
{
	// packing is different from setLinkData or setQ/setD
	VRMLloader& l=skeleton(ichara);
	int nquat=l.dofInfo.numSphericalJoint();
	int ndof=l.dofInfo.numDOF();

	int nActualDof = ndof-nquat ;

	auto& bi=bodyInfo(ichara);
    auto& positions = bi.Q;
    auto& velocities = bi.QDot;

    vectorn proportionalTorquePlusQDotDeltaT(nActualDof);
	proportionalTorquePlusQDotDeltaT.setAllValue(0.0);
    vectorn derivativeTorque(nActualDof);
	derivativeTorque.setAllValue(0.0);

	int qindex=0;
	int qsindex=ndof-nquat*4;
	int dqsindex=qsindex;
	int Qindex=0;

	auto& kps=bi.kps;
	auto& kds=bi.kds;
	Msg::verify(kps.size()==ndof-nquat, "call setStablePDparam first");
	for(int ibone=1; ibone<l.numBone(); ibone++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		auto& link=bi.links[ibone];
		switch(b.mJoint->jointType)
		{
			case HRP_JOINT::FIXED: 
				continue;
			case HRP_JOINT::FREE:
				{
					qindex+=3;
					qsindex+=4;
					dqsindex+=3;

					Qindex+=FREE_JOINT_QSIZE ;
				}
				break;
			case HRP_JOINT::BALL:
				{
					auto targetPosition=desired_q.toQuater(qsindex);

					int cacheIndex=Qindex;
					float timeStep=_timestep;

					targetPosition.normalize(); // Never trust user input

					vector3 kp(
							kps[cacheIndex],
							kps[cacheIndex + 1],
							kps[cacheIndex + 2]
							);

					quater localRotation=toBase(bi.model.GetQuaternion(link.jointId, bi.Q));

					quater posDifference = targetPosition * localRotation.inverse();
					posDifference.normalize();

					vector3 axis=posDifference.rotationVector();

					vector3 proportionalForceInParentFrame(
							kps[cacheIndex] * axis.x,
							kps[cacheIndex + 1] * axis.y,
							kps[cacheIndex + 2] * axis.z
							);
					vector3 proportionalForceInChildFrame = localRotation.inverse()*proportionalForceInParentFrame;

					proportionalTorquePlusQDotDeltaT[dqsindex] = proportionalForceInChildFrame[0] - 
						timeStep * velocities[cacheIndex] * kps[cacheIndex];
					proportionalTorquePlusQDotDeltaT[dqsindex + 1] = proportionalForceInChildFrame[1] -
						timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
					proportionalTorquePlusQDotDeltaT[dqsindex + 2] = proportionalForceInChildFrame[2] -
						timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

					derivativeTorque[dqsindex] = kds[cacheIndex] *( desired_dq[dqsindex]-velocities[cacheIndex]);
					derivativeTorque[dqsindex + 1] = kds[cacheIndex + 1] *( desired_dq[dqsindex+1]-velocities[cacheIndex + 1]);
					derivativeTorque[dqsindex + 2] = kds[cacheIndex + 2] *( desired_dq[dqsindex+2]-velocities[cacheIndex + 2]);
					qsindex+=4;
					dqsindex+=3;
					Qindex+=3;
				}
				break;
			case HRP_JOINT::ROTATE:
				{
					int nq=l.dofInfo.numDOF(ibone);
					ASSERT(bi.model.mJoints[link.jointId].q_index==Qindex);
					for(int i=0; i<nq; i++) 
					{
						int cacheIndex=Qindex;
						proportionalTorquePlusQDotDeltaT[qindex] = kps[cacheIndex] * (
								desired_q[qindex] - positions[cacheIndex] - 
								_timestep * velocities[cacheIndex]
								);
						derivativeTorque[qindex] =kds[cacheIndex]*(desired_dq[qindex]  - velocities[cacheIndex]);
						qindex++;
						Qindex++;
					}
				}
				break;
		}
	}

	ASSERT(Qindex==bi.QDot.size());
	ASSERT(qindex==ndof-nquat*4);

	tau= proportionalTorquePlusQDotDeltaT + derivativeTorque;
	tau.range(0,3).setAllValue(0.0);
	tau.range(qindex,qindex+3).setAllValue(0.0);



    if (applyRootExternalForce) {
		Msg::error("applyRootExternalForce not ported yet");
		ASSERT(false); // not ported yet.
	/*
        vector<float> rootForcePD(6, 0);

        PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
        PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;
        UniformQuaternion(rootGlobalRotation);

        PxVec3 rootGlobalLinearVelocity = rootLink->link->getLinearVelocity();
        PxVec3 rootGlobalAngularVelocity = rootLink->link->getAngularVelocity();
        
        PxVec3 rootGlobalProportionalLinearForcePlusQDotDeltaT(
            root_kps[0] * (targetPositions[0] - rootGlobalPosition[0] - timeStep * rootGlobalLinearVelocity[0]),
            root_kps[1] * (targetPositions[1] - rootGlobalPosition[1] - timeStep * rootGlobalLinearVelocity[1]),
            root_kps[2] * (targetPositions[2] - rootGlobalPosition[2] - timeStep * rootGlobalLinearVelocity[2])
        );
        PxVec3 rootGlobalDerivativeLinearForce(
            -root_kds[0] * rootGlobalLinearVelocity[0],
            -root_kds[1] * rootGlobalLinearVelocity[1],
            -root_kds[2] * rootGlobalLinearVelocity[2]
        );

        PxQuat rootGlobalTargetRotationUser(targetPositions[4], targetPositions[5], targetPositions[6], targetPositions[3]);
        rootGlobalTargetRotationUser.normalize();
        
        // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
        PxQuat rootGlobalTargetRotation = rootGlobalTargetRotationUser * frameTransform;

        PxQuat diffQuat = rootGlobalTargetRotation * rootGlobalRotation.getConjugate();
        if (PxAbs(diffQuat.w) < 0.70710678118f) {
            diffQuat = (-rootGlobalTargetRotation) * rootGlobalRotation.getConjugate();
        }
        UniformQuaternion(diffQuat);

        PxVec3 diffRotExpMapGlobal = QuatToExpMap(diffQuat);
        PxVec3 rootGlobalProportionalTorquePlusQDotDeltaT(
            root_kps[3] * (diffRotExpMapGlobal[0] - timeStep * rootGlobalAngularVelocity[0]),
            root_kps[4] * (diffRotExpMapGlobal[1] - timeStep * rootGlobalAngularVelocity[1]),
            root_kps[5] * (diffRotExpMapGlobal[2] - timeStep * rootGlobalAngularVelocity[2])
        );
        PxVec3 rootGlobalDerivativeTorque(
            -root_kds[3] * rootGlobalAngularVelocity[0],
            -root_kds[4] * rootGlobalAngularVelocity[1],
            -root_kds[5] * rootGlobalAngularVelocity[2]
        );

        for (int i = 0; i < 3; i++) {
            rootForcePD[i] += rootGlobalProportionalLinearForcePlusQDotDeltaT[i] + rootGlobalDerivativeLinearForce[i];
        }
        for (int i = 0; i < 3; i++) {
            rootForcePD[i + 3] += rootGlobalProportionalTorquePlusQDotDeltaT[i] + rootGlobalDerivativeTorque[i];
        }

#ifdef ENABLE_SPD_ABA
        extern bool g_ApplyABARootForce;
        extern float g_RootExternalSpatialForce[100][6];
        extern const float* g_ABA_Root_Kd;

        g_ApplyABARootForce = true;
        g_ABA_Root_Kd = root_kds.data();
        memcpy(g_RootExternalSpatialForce[_id], rootForcePD.data(), 6 * sizeof(float));
#endif
*/
    }
}
inline static void assign33(matrixn& self, const matrix3& M)
{

	self.set(0,0,M._11);
	self.set(0,1,M._12);
	self.set(0,2,M._13);
	self.set(1,0,M._21);
	self.set(1,1,M._22);
	self.set(1,2,M._23);
	self.set(2,0,M._31);
	self.set(2,1,M._32);
	self.set(2,2,M._33);
}                      

void Trbdl:: DynamicsSimulator_Trbdl_penalty::calcJacobianAt(int ichar, int ibone, matrixn& J, vector3 const & localpos)
{
	_calcJacobianAt(ichar, ibone, J, localpos);
	// swap force and torque packing
	matrixn temp;
	temp=J.slice(0,0,3,6);
	J.slice(0,0,3,6)=J.slice(0,0,0,3);
	J.slice(0,0,0,3)=temp;

	// R*bf (world force) to bf (body force)
	// J[0:6]=(I  0)*J[0:6] 
	//        (0  R)        
	matrix3 R;
	R.setFromQuaternion(getWorldState(ichar).global(1).rotation);
	assign33(J.slice(3,6,3,6).lval(),R);
}


void Trbdl:: DynamicsSimulator_Trbdl_penalty::calcDotJacobianAt(int ichar, int ibone, matrixn& DJ, vector3 const & localpos)
{
	_calcDotJacobianAt(ichar, ibone, DJ, localpos);
	// swap force and torque packing
	matrixn temp;
	temp=DJ.slice(0,0,3,6);
	DJ.slice(0,0,3,6)=DJ.slice(0,0,0,3);
	DJ.slice(0,0,0,3)=temp;

	vector3 w=_bodyW(ichar, 1);
	matrix3 R;
	R.setFromQuaternion(getWorldState(ichar).global(1).rotation);
	matrix3 skew_w;
	skew_w.setTilde(w);
	matrix3 dotR=R*skew_w; //  when w is the body angular velocity

	assign33(DJ.slice(3,6,3,6).lval(), dotR);
}

inline static void zero(matrix4 & out)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			out.m[i][j]=0;
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

void Trbdl::DynamicsSimulator_Trbdl_penalty::calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	auto& cinfo=bodyInfo(ichar);
	auto& model=cinfo.model;
	matrixn j,dj;
	jacobian.setSize(6,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(6, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);

	{
		::vector3 COM;
		::vector3 COMVEL;
		double totalMass;
		COM=calculateCOM(ichar, totalMass);
		COMVEL=calculateCOMvel(ichar, totalMass);
		matrixn bJ, bdotJ;
		matrixn dAd_T(6,6);
		matrixn dot_dAd_T(6,6);
		for(int i=1; i<l.numBone(); i++) {
			calcJacobianAt(ichar, i, bJ,vector3(0,0,0));
			calcDotJacobianAt(ichar,i,bdotJ,vector3(0,0,0));

			auto& link=cinfo.links[i];
			int bi=link.bodyId;
			transf T_global=trl_transf(model.X_base[bi]);

			matrix3 R(T_global.rotation);
			matrix3 invR;
			invR.inverse(R);

			// bJ=invR*J

			// R*invR=I
			// dotR*invR+R*dot(invR)=0
			// dot(invR)=-invR*dotR*invR
			vector3 w=_bodyW(ichar, i);
			matrix3 skew_w;
			skew_w.setTilde(w);
			matrix3 dotR=R*skew_w; //  when w is the body angular velocity
			matrix3 dotInvR=-invR*dotR*invR;

			// bdotJ=invR*dJ+dot(invR)*J
			for(int j=0; j<bdotJ.cols(); j++)
			{
				bdotJ.column(j).setVec3(0, invR*bdotJ.column(j).toVector3(0)+dotInvR*bJ.column(j).toVector3(0));
				bdotJ.column(j).setVec3(3, invR*bdotJ.column(j).toVector3(3)+dotInvR*bJ.column(j).toVector3(3));
			}

			for(int j=0; j<bJ.cols(); j++)
			{
				bJ.column(j).setVec3(0, invR*bJ.column(j).toVector3(0));
				bJ.column(j).setVec3(3, invR*bJ.column(j).toVector3(3));
			}




			transf invBodyT=T_global.inverse();
			matrix4 dotBodyT=calcDotT(T_global, trl_se3(model.v[bi]));


			// T * invT = I
			// dotT*invT+T*dotInvT=0
			// dot(invT)=-invT*dotT*invT
			
			transf T=invBodyT*transf(COM);
			Liegroup::dAd(dAd_T, T);
			matrix4 dotCOM;
			zero(dotCOM);
			dotCOM.setTranslation(COMVEL);
			matrix4 invBodyT2=invBodyT;
			matrix4 dotT= invBodyT2*dotCOM- invBodyT2*dotBodyT*invBodyT2 * transf(COM); 
			Liegroup::dot_dAd(dot_dAd_T, T, dotT);

			VRMLTransform& bone=l.VRMLbone(i);
			double mass=bone.mass();
			Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
			for (int j=0; j<bJ.cols(); j++){
				Liegroup::se3 cj=Liegroup::to_se3(bJ.column(j));
				//dse3 cj2=dAd(T, body->I*cj);
				//radd(jacobian.column(j).lval(), cj2);
				Liegroup::dse3 temp=I*cj;
				radd(jacobian.column(j).lval(),mult( dAd_T,temp));

				radd(dotjacobian.column(j).lval(), 
						mult(dot_dAd_T,temp)+mult(dAd_T,I*Liegroup::to_se3(bdotJ.column(j))));
				// dAd(T)=
				//    (     R       0 )'
				//    (   skew(T)*R R )
				// cj2= dAd(T)*I*cj
			}
		}
	}
}

