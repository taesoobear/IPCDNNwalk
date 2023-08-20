
#include "stdafx.h"
#include "DynamicsSimulator_Trbdl_impulse.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/pldprimskin.h"
#include "../OpenHRPcommon.h"
#include "../TRL/eigenSupport.h"
#include <limits>
#include "../../BaseLib/utility/QPerformanceTimer.h"
#include "../../BaseLib/motion/VRMLloader_internal.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include "../../MainLib/OgreFltk/renderer.h"
#include "../../MainLib/OgreFltk/objectList.h"
#endif

using namespace std;

static ObjectList dbg;
#if 1 // set 1 to disable profiling. set 0 to enable.
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)	
#define END_TIMER2(x)
#endif

#define USE_JACCACHE // faster when there are not too many collision points. also more room for further improvment
#define USE_PARTIAL_UPDATE
#define COLLECT_CONTACT_FORCES // for drawing, etc.

#ifndef USE_JACCACHE
#define USE_MASS_MAT // slow
#endif


//#define VERBOSE
void forwardKinematics ( Trbdl::BodyInfo& bi, Model &model, const VectorNd &Q, const VectorNd &QDot);
void forwardDynamics (Trbdl::BodyInfo& bi,  Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, const VectorNd &K_i, VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA) ;
void integratePos(VRMLloader& l, Trbdl::BodyInfo& bi, double ts);

// call after calcMinvTimesTau_preparePartial
void calcMInvTimesTau ( Model &model, const VectorNd &Q, const VectorNd &Tau, Eigen::Ref<VectorNd >QDDot) {

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].setZero();

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    model.pA[i].setZero();
  }

  // compute articulated bias forces
  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {

      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
      // LOG << "u[" << i << "] = " << model.u[i] << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialVector pa = model.pA[i] + model.U[i] * model.u[i] / model.d[i];

        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
      }
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {

      Vector3d tau_temp ( Tau[q_index],
          Tau[q_index + 1],
          Tau[q_index + 2]);
      model.multdof3_u[i] = tau_temp
        - model.multdof3_S[i].transpose()*model.pA[i];
      //      LOG << "multdof3_u[" << i << "] = "
      // << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        SpatialVector pa = model.pA[i]
          + model.multdof3_U[i]
          * model.multdof3_Dinv[i]
          * model.multdof3_u[i];

        model.pA[lambda].noalias() +=
          model.X_lambda[i].applyTranspose(pa);
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd tau_temp(Tau.block(q_index,0,dofI,1));

      model.mCustomJoints[kI]->u =
        tau_temp - ( model.mCustomJoints[kI]->S.transpose()* model.pA[i]);
      //      LOG << "mCustomJoints[kI]->u"
      // << model.mCustomJoints[kI]->u.transpose() << std::endl;
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        SpatialVector pa = model.pA[i]
          + (   model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->u);

        model.pA[lambda].noalias() +=
          model.X_lambda[i].applyTranspose(pa);
      }
    }
  }

  //  ClearLogOutput();

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i])*(model.u[i]-model.U[i].dot(model.a[i]));
      model.a[i]     = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp =
        model.multdof3_Dinv[i] * (model.multdof3_u[i]
            - model.multdof3_U[i].transpose()*model.a[i]);

      QDDot[q_index]      = qdd_temp[0];
      QDDot[q_index + 1]  = qdd_temp[1];
      QDDot[q_index + 2]  = qdd_temp[2];
      model.a[i]          = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;

      VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
        * (  model.mCustomJoints[kI]->u
            - model.mCustomJoints[kI]->U.transpose() * model.a[i]);

      for(unsigned z = 0; z < dofI; ++z){
        QDDot[q_index+z]      = qdd_temp[z];
      }

      model.a[i] =    model.a[i]
        + model.mCustomJoints[kI]->S * qdd_temp;
    }
  }
}


namespace Trbdl
{
	const double cCollisionEps = 1.0e-5;
#ifdef USE_JACCACHE
	struct JacCache {
		MatrixNd G; // jacobian for a constrained body.
		MatrixNd MinvGT;
		Math::SpatialVector pA0;
		Eigen::Matrix<double, 6, Eigen::Dynamic> U_Dinv;
#ifdef USE_PARTIAL_UPDATE
		bool constrained;
#endif

	};
#endif
	struct SimArticulatedBody{
		Trbdl::BodyInfo* _bi;
		DynamicsSimulator_Trbdl_impulse* _sim;
		bool _allBodiesFixed;
		bool _M_cached;
#ifdef USE_MASS_MAT
		MatrixNd M;
		Eigen::LLT<MatrixNd> M_llt;
#endif
#ifdef USE_JACCACHE
		boolN _jac_cached;
		std::vector<JacCache> _jac_cache;
#endif
#ifdef USE_PARTIAL_UPDATE
		// for calcMInvTimesTau_partial_chain
		std::vector <int> chain;
		std::vector<Math::SpatialVector> pA_backup;
		std::vector<double> u_backup;
		std::vector<Math::Vector3d> multdof3_u_backup;
#endif


		intvectorn limitedBones;
		VRMLloader* _l;
		SimArticulatedBody(int ichara, Trbdl::BodyInfo* _body, DynamicsSimulator_Trbdl_impulse* sim)
		{
			_sim=sim;
			_bi=_body;
			_allBodiesFixed=false;
			int numBone=sim->skeleton(ichara).numBone();
			_l=&sim->skeleton(ichara);
			if(sim->skeleton(ichara).VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED && numBone==2)
				_allBodiesFixed=true;
		}
		inline bool isStatic(){ return _allBodiesFixed;}
	};
	struct CollisionInfo {
#ifdef COLLECT_CONTACT_FORCES
		int ilinkpair=-1;
#endif
		SimArticulatedBody* mBodyA = nullptr;
		SimArticulatedBody* mBodyB = nullptr;
		int mBodyAIndex;
		int mBodyBIndex;
		//Vector3d mManifoldPoints[8];
		//int mNumManifoldPoints = 0;
		Vector3d posA = Vector3d::Zero();
		Vector3d posB = Vector3d::Zero();
		double biasVelocityA = 0.;
		double biasVelocityB = 0.;
		double accumImpulse = 0.;
		double deltaImpulse = 0.;
		Vector3d dir = Vector3d(0., 1., 0.);
		VectorNd jacA = VectorNd::Zero(1);
		VectorNd jacB = VectorNd::Zero(1);
		VectorNd MInvJacTA = VectorNd::Zero(1);
		VectorNd MInvJacTB = VectorNd::Zero(1);
		double GMInvGTA = 0.;
		double GMInvGTB = 0.;
		double accumFrictionImpulse[2] = {0.,0.};
		double deltaFrictionImpulse[2] = {0.,0.};
		Vector3d tangents[2] = {Vector3d::Zero(), Vector3d::Zero()};
		VectorNd tangentJacA[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
		VectorNd tangentJacB[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
		VectorNd tangentMInvJacTA[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
		VectorNd tangentMInvJacTB[2] = {VectorNd::Zero(1), VectorNd::Zero(1)};
		double tangentGMInvGTA[2] = {0., 0.};
		double tangentGMInvGTB[2] = {0., 0.};
		double effectiveRestitution = 1.0;
		double effectiveFriction = 0.2;

		double depth = 0.;
	};
	// assumes zero Tau.
	void calcMInvTimesTau_preparePartial (Trbdl::SimArticulatedBody* body, Model &model, const VectorNd &Q, Eigen::Ref<VectorNd > QDDot, bool update_kinematics=true) {

		// Reset the velocity of the root body
		model.v[0].setZero();
		model.a[0].setZero();

		if (update_kinematics) {
			for (unsigned int i = 1; i < model.mBodies.size(); i++) {
				//jcalc_X_lambda_S (model, model.mJointUpdateOrder[i], Q);

				model.v_J[i].setZero();
				model.v[i].setZero();
				model.c[i].setZero();
				model.pA[i].setZero();
				model.I[i].setSpatialMatrix (model.IA[i]);
			}
		}

		for (unsigned int i = 1; i < model.mBodies.size(); i++) {
			model.pA[i].setZero();
		}

		// ClearLogOutput();

		if (update_kinematics) {
			// Compute Articulate Body Inertias
			for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
				unsigned int q_index = model.mJoints[i].q_index;

				ASSERT(model.mJoints[i].mJointType != JointTypeCustom);
					if (model.mJoints[i].mDoFCount == 1  ) {
						model.U[i] = model.IA[i] * model.S[i];
						model.d[i] = model.S[i].dot(model.U[i]);
#if 0
						// equivalent inertia
						//model.d[i]+=MA;
						model.d[i]+=K_i[q_index]; // k_i
#endif
						//      LOG << "u[" << i << "] = " << model.u[i] << std::endl;
						unsigned int lambda = model.lambda[i];

						if (lambda != 0) {
							SpatialMatrix Ia = model.IA[i] -
								model.U[i] * (model.U[i] / model.d[i]).transpose();

							//Ia.diagonal().array()+=MA; // worse robustness

							model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
								* Ia
								* model.X_lambda[i].toMatrix();


						}
					} else if (model.mJoints[i].mDoFCount == 3
							&& model.mJoints[i].mJointType != JointTypeCustom) {

						model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];

#if 0
						// taesoo
						//
						Eigen::Matrix<double, 3,3> jm2=Eigen::Matrix<double, 3,3>::Identity(3,3)*
							(K_i[q_index]);

						model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
								* model.multdof3_U[i]+jm2).inverse().eval();
#else
						model.multdof3_Dinv[i] =
							(model.multdof3_S[i].transpose()*model.multdof3_U[i]).inverse().eval();
#endif
						//      LOG << "mCustomJoints[kI]->u[" << i << "] = "
						//<< model.mCustomJoints[kI]->u[i].transpose() << std::endl;

						unsigned int lambda = model.lambda[i];

						if (lambda != 0) {
							SpatialMatrix Ia = model.IA[i]
								- ( model.multdof3_U[i]
										* model.multdof3_Dinv[i]
										* model.multdof3_U[i].transpose());
							//Ia.diagonal().array()+=MA; // worse robustness .
							model.IA[lambda].noalias() +=
								model.X_lambda[i].toMatrixTranspose()
								* Ia
								* model.X_lambda[i].toMatrix();
						}
					} else if (model.mJoints[i].mJointType == JointTypeCustom) {
						unsigned int kI     = model.mJoints[i].custom_joint_index;
						// unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
						model.mCustomJoints[kI]->U = model.IA[i] * model.mCustomJoints[kI]->S;

						model.mCustomJoints[kI]->Dinv = (model.mCustomJoints[kI]->S.transpose()
								* model.mCustomJoints[kI]->U
								).inverse().eval();
						//      LOG << "mCustomJoints[kI]->u[" << i << "] = "
						//<< model.mCustomJoints[kI]->u.transpose() << std::endl;
						unsigned int lambda = model.lambda[i];

						if (lambda != 0) {
							SpatialMatrix Ia = model.IA[i]
								- ( model.mCustomJoints[kI]->U
										* model.mCustomJoints[kI]->Dinv
										* model.mCustomJoints[kI]->U.transpose());
							model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
								* Ia
								* model.X_lambda[i].toMatrix();
						}
					}
			}
		}

		// compute articulated bias forces
		for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
			unsigned int q_index = model.mJoints[i].q_index;
			ASSERT(model.mJoints[i].mJointType != JointTypeCustom);

			auto& bd=body->_jac_cache[i];
			if (model.mJoints[i].mDoFCount == 1) {

				model.u[i] =  - model.S[i].dot(model.pA[i]);
				// LOG << "u[" << i << "] = " << model.u[i] << std::endl;
				unsigned int lambda = model.lambda[i];
				if (lambda != 0) {
					bd.U_Dinv=model.U[i] / model.d[i];
					SpatialVector pa = model.pA[i] + bd.U_Dinv * model.u[i] ;

					bd.pA0=model.X_lambda[i].applyTranspose(pa);
					model.pA[lambda].noalias() += bd.pA0;
				}
			} else if (model.mJoints[i].mDoFCount == 3) {

				model.multdof3_u[i] = - model.multdof3_S[i].transpose()*model.pA[i];
				//      LOG << "multdof3_u[" << i << "] = "
				// << model.multdof3_u[i].transpose() << std::endl;
				unsigned int lambda = model.lambda[i];

				if (lambda != 0) {
					bd.U_Dinv=model.multdof3_U[i]* model.multdof3_Dinv[i];
					SpatialVector pa = model.pA[i]
						+bd.U_Dinv 
						* model.multdof3_u[i];

					bd.pA0=model.X_lambda[i].applyTranspose(pa);
					model.pA[lambda].noalias() +=bd.pA0;
				}
			} 
		}

		//  ClearLogOutput();

		for (unsigned int i = 1; i < model.mBodies.size(); i++) {
			unsigned int q_index = model.mJoints[i].q_index;
			unsigned int lambda = model.lambda[i];
			SpatialTransform X_lambda = model.X_lambda[i];

			model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 1
					&& model.mJoints[i].mJointType != JointTypeCustom) {
				QDDot[q_index] = (1./model.d[i])*(model.u[i]-model.U[i].dot(model.a[i]));
				model.a[i]     = model.a[i] + model.S[i] * QDDot[q_index];
			} else if (model.mJoints[i].mDoFCount == 3
					&& model.mJoints[i].mJointType != JointTypeCustom) {
				Vector3d qdd_temp =
					model.multdof3_Dinv[i] * (model.multdof3_u[i]
							- model.multdof3_U[i].transpose()*model.a[i]);

				QDDot[q_index]      = qdd_temp[0];
				QDDot[q_index + 1]  = qdd_temp[1];
				QDDot[q_index + 2]  = qdd_temp[2];
				model.a[i]          = model.a[i] + model.multdof3_S[i] * qdd_temp;
			} else if (model.mJoints[i].mJointType == JointTypeCustom) {
				unsigned int kI     = model.mJoints[i].custom_joint_index;
				unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;

				VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
					* (  model.mCustomJoints[kI]->u
							- model.mCustomJoints[kI]->U.transpose() * model.a[i]);

				for(unsigned z = 0; z < dofI; ++z){
					QDDot[q_index+z]      = qdd_temp[z];
				}

				model.a[i] =    model.a[i]
					+ model.mCustomJoints[kI]->S * qdd_temp;
			}
		}
	}
#ifdef USE_PARTIAL_UPDATE
	void calcMInvTimesTau_partial_chain(std::vector<int> const& chain,  Trbdl::SimArticulatedBody* body,Model &model, const VectorNd &Q, const VectorNd &Tau, Eigen::Ref<VectorNd >QDDot) 
	{
		// Reset the velocity of the root body
		model.v[0].setZero();

		auto& md=body->_jac_cache;
		// compute articulated bias forces
		for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
			unsigned int q_index = model.mJoints[i].q_index;
			auto& bd=md[i];
			if(!bd.constrained)
			{
				unsigned int lambda = model.lambda[i];
				// unmodified pA
				if (lambda != 0 && md[lambda].constrained) {
					model.pA[lambda] += bd.pA0;
				}
				continue;
			}
			ASSERT( model.mJoints[i].mJointType != JointTypeCustom);
			if (model.mJoints[i].mDoFCount == 1) {

				model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
				// LOG << "u[" << i << "] = " << model.u[i] << std::endl;
				unsigned int lambda = model.lambda[i];
				if (lambda != 0) {
					SpatialVector pa = model.pA[i] + bd.U_Dinv * model.u[i] ;
					model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
				}
			} else if (model.mJoints[i].mDoFCount == 3){
				model.multdof3_u[i] = Tau.segment<3>(q_index)
					- model.multdof3_S[i].transpose()*model.pA[i];
				unsigned int lambda = model.lambda[i];

				if (lambda != 0) {
					SpatialVector pa = model.pA[i] + bd.U_Dinv * model.multdof3_u[i];

					model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
				}
			} 
		}
		// calculate accelerations only. 
		for (unsigned int i = 1; i < model.mBodies.size(); i++) {
			auto& bd=md[i];
			unsigned int q_index = model.mJoints[i].q_index;
			unsigned int lambda = model.lambda[i];
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 1) {
				QDDot[q_index] = (1./model.d[i])*(model.u[i]-model.U[i].dot(model.a[i]));
				model.a[i]     = model.a[i] + model.S[i] * QDDot[q_index];
			} else if (model.mJoints[i].mDoFCount == 3) {
				Vector3d qdd_temp =
					model.multdof3_Dinv[i] * (model.multdof3_u[i]
							- model.multdof3_U[i].transpose()*model.a[i]);

				QDDot[q_index]      = qdd_temp[0];
				QDDot[q_index + 1]  = qdd_temp[1];
				QDDot[q_index + 2]  = qdd_temp[2];
				model.a[i]          = model.a[i] + model.multdof3_S[i] * qdd_temp;
			}
		}
	}

inline static void  prepareBackupChain(Trbdl::SimArticulatedBody* body, std::vector<int> & chain, int bii)
{
	auto& bi=*body->_bi;
	chain.resize(0);
	{
		auto& md=body->_jac_cache;
		int nb=md.size();
		chain.reserve(nb);
		int l=bii;

		while (l != 0) {
			md[l].constrained=true;
			chain.push_back(l);
			l=bi.model.lambda[l];
		}

		body->pA_backup.resize(chain.size());
		body->u_backup.resize(chain.size());
		body->multdof3_u_backup.resize(chain.size());
		for(int i=0; i<chain.size(); i++)
		{
			body->pA_backup[i]=bi.model.pA[chain[i]];
			body->u_backup[i]=bi.model.u[chain[i]];
			body->multdof3_u_backup[i]=bi.model.multdof3_u[chain[i]];
		}
	}
}

inline static void  restoreBackupChain_pA_only(Trbdl::SimArticulatedBody* body, std::vector<int> & chain)
{
	auto& bi=*body->_bi;
	for(int i=0; i<chain.size(); i++)
		bi.model.pA[chain[i]]=body->pA_backup[i];
}
inline static void  restoreBackupChain(Trbdl::SimArticulatedBody* body, std::vector<int> & chain)
{
	auto& bi=*body->_bi;
	auto& md=body->_jac_cache;
	for(int i=0; i<chain.size(); i++)
	{
		bi.model.pA[chain[i]]=body->pA_backup[i];
		bi.model.u[chain[i]]=body->u_backup[i];
		bi.model.multdof3_u[chain[i]]=body->multdof3_u_backup[i];
		md[chain[i]].constrained=false;
	}
}
#endif
}

void Trbdl::DynamicsSimulator_Trbdl_impulse::_registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo)
{
	Trbdl::DynamicsSimulator_Trbdl_penalty::_registerCharacter(name, cinfo);
	int ichara=_characters.size()-1;
	Trbdl::BodyInfo* chara=&bodyInfo(ichara);
	_bodies.resize(_bodies.size()+1);
	auto* simbody=new SimArticulatedBody(ichara, chara, this);
	_bodies.back()=simbody;

	int ibody=_bodies.size()-1;
	assert(ibody==ichara);

	auto& l=*cinfo.loader;
	for(int i=1; i<l.numBone(); i++)
	{
		auto& bone=l.VRMLbone(i);
		if(bone.HRPjointType(0)== HRP_JOINT::ROTATE && bone.mJoint->_jointRangeIsLimited())
		{
			simbody->limitedBones.pushBack(i);
		}
	}

}
// defined in DynamicsSimulator.cpp
void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output);


Trbdl::DynamicsSimulator_Trbdl_impulse::DynamicsSimulator_Trbdl_impulse(const char* coldet)
	:
		DynamicsSimulator_Trbdl_penalty(coldet)
{
	_MA=0;
	_restitution=0.0;
}
Trbdl::DynamicsSimulator_Trbdl_impulse::~DynamicsSimulator_Trbdl_impulse()
{

}

void Trbdl::DynamicsSimulator_Trbdl_impulse::initSimulation()
{
	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		auto &bi=bodyInfo(i);

		forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
		bi.clearExternalForces();
	}
	_updateCharacterPose();
}
void Trbdl::DynamicsSimulator_Trbdl_impulse::init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{
	DynamicsSimulator_Trbdl_penalty::init(timeStep, integrateOpt);

	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		auto &bi=bodyInfo(i);

		bi._timestep=_timestep;
		if(bi.kds.size()>0 && bi.kds[0]==0.0)
			bi.kds.setConstant(_MA/_timestep);
	}
	//TODO:TestCode for add connection
	//addRelativeConstraint(1,  2, vector3(0.5,0.5,0));

}


void Trbdl::DynamicsSimulator_Trbdl_impulse::setParam_restitution_MA(double r, double ma)
{
	_restitution=r;
	_MA=ma;
}
//

bool Trbdl::DynamicsSimulator_Trbdl_impulse::stepSimulation()
{

#if 0
	// calc UnconstrainedVelUpdate
	BEGIN_TIMER(velupdate);
	// set external forces
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);
		auto& bi=bodyInfo(ichara);
		forwardDynamics ( bi, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_timestep, bi.QDDot, &bi.f_ext, _MA);

		// semi-implicit eulers
		bi.QDot+=_timestep*bi.QDDot;
	}
	END_TIMER2(velupdate);
	BEGIN_TIMER(colCheck);
	// collision check rigid-rigid pairs
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	END_TIMER2(colCheck);

	BEGIN_TIMER(resolveCOl);

	clearJacCache();
	resolveCollisions(_timestep, 20 );
	END_TIMER2(resolveCOl);

	BEGIN_TIMER(integ);
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);
		auto& bi=bodyInfo(ichara);
		bi.clearExternalForces();

		integratePos(l, bi,_timestep);
	}
	END_TIMER2(integ);
	_currentTime+=_timestep;
	BEGIN_TIMER(updatepose);
	_updateCharacterPose();
	END_TIMER2(updatepose);
#else
	BEGIN_TIMER(velupdate);
	std::vector<VectorNd> tau;
	std::vector<VectorNd> q;
	std::vector<VectorNd> qdot;
	tau.resize(numSkeleton());
	q.resize(numSkeleton());
	qdot.resize(numSkeleton());
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);
		auto& bi=bodyInfo(ichara);
		tau[ichara]=bi.Tau;
		q[ichara]=bi.Q;
		forwardDynamics ( bi, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_timestep, bi.QDDot, &bi.f_ext, _MA);
		qdot[ichara]=VectorNd::Zero(bi.QDot.size()); // accumulation buffer
	}
	END_TIMER2(velupdate);// 2us

	double ts=_timestep*0.5;
	clearJacCache();

#ifdef COLLECT_CONTACT_FORCES
	{
		// collect contact forces
		int npairs=collisionDetector->getCollisionPairs().size();
		_contactForces.setSize(npairs);
		_contactPos.setSize(npairs);
		_contactForces.setAllValue(vector3(0.0,0.0,0.0));
		_contactPos.setAllValue(vector3(0.0,0.0,0.0));
	}
#endif //COLLECT_CONTACT_FORCES


	for(int iter=0; iter<3; iter++){

		// calc UnconstrainedVelUpdate
		// set external forces
		for(int ichara=0; ichara<numSkeleton(); ichara++)
		{
			VRMLloader& l=skeleton(ichara);
			auto& bi=bodyInfo(ichara);
			bi.Tau=tau[ichara];

			// semi-implicit eulers
			bi.QDot+=ts*bi.QDDot;
		}
		BEGIN_TIMER(colCheck);
		collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
		END_TIMER2(colCheck); // 9us
		BEGIN_TIMER(resolveCOl);
		resolveCollisions(ts, 7 , iter==2);
		END_TIMER2(resolveCOl); // 56, 15, 14us (most of the time in 1st prepareCon: bodyjac, minvjac)
		BEGIN_TIMER(integ);
		for(int ichara=0; ichara<numSkeleton(); ichara++)
		{
			VRMLloader& l=skeleton(ichara);
			auto& bi=bodyInfo(ichara);
			integratePos(l, bi,ts);
			qdot[ichara]+=bi.QDot;
		}
		_updateCharacterPose(); 
		END_TIMER2(integ); // 0us
	}
		BEGIN_TIMER(fk);
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);
		auto& bi=bodyInfo(ichara);
		bi.Q=q[ichara];
		bi.QDot=qdot[ichara]*(1.0/3.0);

		//resolveCollisions(ts, 20 );

		integratePos(l, bi, _timestep);
		forwardKinematics(bi, bi.model, bi.Q,bi.QDot);
		_updateCharacterPose();
		bi.clearExternalForces();
	}
		END_TIMER2(fk); //1us
#ifdef COLLECT_CONTACT_FORCES
	{
		// post-process collected contact forces
		int npairs=collisionDetector->getCollisionPairs().size();
		for(int i=0; i<npairs; i++){
			vector3& cf=_contactForces(i);
			vector3& cpf=_contactPos(i);

			if(cf.length()>1e-3)
				cpf/=cf.length();
			else
				cf.zero();
			cf/=(_timestep);
		}
	}
#endif //COLLECT_CONTACT_FORCES

	_currentTime+=_timestep;
#endif


	return true;
}

void Trbdl::DynamicsSimulator_Trbdl_impulse::drawLastContactForces(int ichar, ::vector3 const& draw_offset) const
{
	static ObjectList g_debugDraw;
    int n = collisionDetector->getCollisionPairs().size();
	double _contactForceVis=0.001;
	vector3N lines;
	double f_y=0;
    for(int i=0; i < n; ++i){
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		vector3& cf=_contactForces(i);
		if(cf.length()<1e-3) continue;
		vector3& p=_contactPos(i);

		f_y+=cf.y;
		for(int j=0; j < 2; ++j){
			if(linkPair.charIndex[j]==ichar){
				vector3 f=cf;
				if(j==0) 
					f*=-1.0;

				lines.pushBack(p*100);
				lines.pushBack((p+f*_contactForceVis)*100);
			}
		}
	}
	/*
	auto& l=skeleton(0);
	double mass=0;
	for (int i=0; i<l.numBone(); i++)
		mass+=l.VRMLbone(i).mass();

	printf("fy %f %f\n", f_y,mass);
	*/
#ifndef NO_OGRE
	g_debugDraw.registerObject("contactForce222", "LineList", "solidred", matView(lines));
	g_debugDraw.getCurrRootSceneNode()->setPosition(ToOgre(draw_offset));
#endif
}

vector3 Trbdl::DynamicsSimulator_Trbdl_impulse::getContactForce(int ichar, int ibone) const
{
    int n = collisionDetector->getCollisionPairs().size();
	vector3 f(0.0,0.0,0.0);
	if(_contactForces.size()==0) return f;
    for(int i=0; i < n; ++i){
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		vector3& cf=_contactForces(i);
		for(int j=0; j < 2; ++j){
			if(linkPair.charIndex[j]==ichar && 
				linkPair.link[j]->treeIndex()==ibone){
				if(j==0) 
					f+=cf*-1.0;
				else
					f+=cf;

			}
		}
	}
	return f;
}
/// Calculates the impulse that we apply on body_b to resolve the contact.
void CalcConstraintImpulse(
		Trbdl::SimArticulatedBody* body_a,
		Trbdl::SimArticulatedBody* body_b,
		Trbdl::CollisionInfo& cinfo) {
	//ZoneScoped;
	// Todo: add nonlinear effects * dt

	double rhs = 0.;
	if (body_a && !body_a->isStatic()) {
		rhs += cinfo.jacA .transpose().dot( body_a->_bi->QDot )+ cinfo.biasVelocityA;
	}
	if (body_b && !body_b->isStatic()) {
		rhs += -cinfo.jacB .transpose().dot( body_b->_bi->QDot) - cinfo.biasVelocityB;
	}

	double denom = cinfo.GMInvGTA + cinfo.GMInvGTB;
	assert(denom > Trbdl::cCollisionEps);

	double old_impulse = cinfo.accumImpulse;
	// TODO: is this really needed here??
	cinfo.deltaImpulse = rhs / denom;
	cinfo.accumImpulse = std::max(0., cinfo.accumImpulse + cinfo.deltaImpulse);
	cinfo.deltaImpulse = cinfo.accumImpulse - old_impulse;
}

void ApplyConstraintImpulse(
		Trbdl::SimArticulatedBody* body_a,
		Trbdl::SimArticulatedBody* body_b,
		Trbdl::CollisionInfo& cinfo) {
	//ZoneScoped;

	if (body_a && !body_a->isStatic()) {
		body_a->_bi->QDot +=
			cinfo.MInvJacTA * (-cinfo.deltaImpulse);
		assert(!isnan(body_a->_bi->QDot.squaredNorm()));
	}

	if (body_b && !body_b->isStatic()) {
		body_b->_bi->QDot += cinfo.MInvJacTB * (cinfo.deltaImpulse);
		assert(!isnan(body_b->_bi->QDot.squaredNorm()));
	}
}



/// Calculates the impulse that we apply on body_b to resolve the contact.
void CalcFrictionImpulse(
		Trbdl::SimArticulatedBody* body_a,
		Trbdl::SimArticulatedBody* body_b,
		Trbdl::CollisionInfo& cinfo) {
	//ZoneScoped;

	// Todo: add nonlinear effects * dt

	double rhs_tangent[2] = {0., 0.};
	if (body_a && !body_a->isStatic()) {
		rhs_tangent[0] += cinfo.tangentJacA[0].transpose().dot( body_a->_bi->QDot);
		rhs_tangent[1] += cinfo.tangentJacA[1].transpose().dot( body_a->_bi->QDot);
	}
	if (body_b && !body_b->isStatic()) {
		rhs_tangent[0] -= cinfo.tangentJacB[0].transpose().dot( body_b->_bi->QDot);
		rhs_tangent[1] -= cinfo.tangentJacB[1].transpose().dot( body_b->_bi->QDot);
	}


	for (int i = 0; i < 2; i++) {
		double denom = cinfo.tangentGMInvGTA[i] + cinfo.tangentGMInvGTB[i];
		assert (denom > Trbdl::cCollisionEps);
		double old_impulse = cinfo.accumFrictionImpulse[i];

		cinfo.deltaFrictionImpulse[i] = rhs_tangent[i] / denom;
		cinfo.accumFrictionImpulse[i] = cinfo.accumFrictionImpulse[i] + cinfo.deltaFrictionImpulse[i];
		if (cinfo.accumFrictionImpulse[i] >= cinfo.effectiveFriction * cinfo.accumImpulse) {
			cinfo.accumFrictionImpulse[i] = cinfo.effectiveFriction * cinfo.accumImpulse;
		}
		if (cinfo.accumFrictionImpulse[i] < -cinfo.effectiveFriction * cinfo.accumImpulse) {
			cinfo.accumFrictionImpulse[i] = -cinfo.effectiveFriction * cinfo.accumImpulse;
		}
		cinfo.deltaFrictionImpulse[i] = cinfo.accumFrictionImpulse[i] - old_impulse;

		assert (!isnan(cinfo.deltaFrictionImpulse[i]));
	}
}

void ApplyFrictionImpulse(
		Trbdl::SimArticulatedBody* body_a,
		Trbdl::SimArticulatedBody* body_b,
		Trbdl::CollisionInfo& cinfo) {
	//ZoneScoped;

	if (body_a && !body_a->isStatic()) {
		body_a->_bi->QDot +=
			cinfo.tangentMInvJacTA[0] * (-cinfo.deltaFrictionImpulse[0]);
		body_a->_bi->QDot +=
			cinfo.tangentMInvJacTA[1] * (-cinfo.deltaFrictionImpulse[1]);

		assert(!isnan(body_a->_bi->QDot.squaredNorm()));
	}

	if (body_b && !body_b->isStatic()) {
		body_b->_bi->QDot +=
			-cinfo.tangentMInvJacTB[0] * (-cinfo.deltaFrictionImpulse[0]);
		body_b->_bi->QDot +=
			-cinfo.tangentMInvJacTB[1] * (-cinfo.deltaFrictionImpulse[1]);

		assert(!isnan(body_b->_bi->QDot.squaredNorm()));
	}
}

static void sCalcTangentVectors(const Vector3d &normal, Vector3d* tangent0, Vector3d* tangent1) {
  if (fabs(normal.dot(Vector3d(0., 0., 1.))) < 0.6) {
    *tangent0 = normal.cross(Vector3d(0., 0., 1.));
    *tangent1 = tangent0->cross(normal);
  } else {
    *tangent0 = normal.cross(Vector3d(1., 0., 0.));
    *tangent1 = tangent0->cross(normal);
  }

  assert (tangent0->squaredNorm() > Trbdl::cCollisionEps);
  assert (tangent1->squaredNorm() > Trbdl::cCollisionEps);
}

void calcSpatialJacobian (
    Model &model,
    const VectorNd &Q,
    unsigned int reference_body_id,
    MatrixNd &G) {

  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int j = reference_body_id;

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom){
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index,6,1) =
              model.X_base[j]
              .inverse()
              .apply(model.S[j])
              ;
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0,q_index,6,3) =
          (model.X_base[j].inverse()
          ).toMatrix() * model.multdof3_S[j];
      }
    }else if(model.mJoints[j].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0,q_index,6,model.mCustomJoints[k]->mDoFCount ) =
        (model.X_base[j].inverse()
        ).toMatrix() * model.mCustomJoints[k]->S;
    }

    j = model.lambda[j];
  }
}

void CalcImpulseVariables(
    double dt,
	double _MA,
    Trbdl::SimArticulatedBody* body,
    unsigned int body_index,
    const Vector3d& pos,
    const Vector3d& dir,
    const Vector3d* tangents,
    const double depth,
    VectorNd* MInvJacT,
    VectorNd* jac,
    double* G_MInv_GT,
    VectorNd* tangentJac,
    VectorNd* tangentMInvJacT,
    double* tangentGMInvGT,
    double* bias_vel,
    double restitution,
	double fric) {
  if (body == nullptr || body->isStatic()) {
    jac->setZero();
    *G_MInv_GT = 0.;
    *bias_vel = 0.;
    return;
  }

  Model* model = &body->_bi->model;
  int ndof = model->qdot_size;
  const VectorNd& q = body->_bi->Q;
  const VectorNd& qdot = body->_bi->QDot;

  assert(!isnan(q.squaredNorm()));

  // Calculate local coordinates of the contact point
  //UpdateKinematicsCustom(*model, &q, nullptr, nullptr);

#ifdef USE_MASS_MAT
  MatrixNd& M=body->M;
  auto& M_llt=body->M_llt;
#endif
  if(!body->_M_cached) 
  {

#ifdef USE_MASS_MAT
	  BEGIN_TIMER(calcmass);
	  // Compute vectors and matrices of the contact system
	  M=MatrixNd::Zero(ndof, ndof);
	  CompositeRigidBodyAlgorithm(*model, q, M, false);
	  M_llt=Eigen::LLT<MatrixNd>(M);
	  END_TIMER2(calcmass);
	  body->_M_cached=true;
#endif
	  BEGIN_TIMER(calcmass2);

#ifdef USE_JACCACHE
	  body->_jac_cached.resize(model->X_base.size());
	  body->_jac_cached.setAllValue(false);
	  body->_jac_cache.resize(model->X_base.size());

#ifdef USE_PARTIAL_UPDATE
	  for(int ii=0; ii<model->X_base.size(); ii++)
		  body->_jac_cache[ii].constrained=false;
#endif

	  VectorNd qddot0(ndof);
	  calcMInvTimesTau_preparePartial(body, *model, q, qddot0);
#endif
	  END_TIMER2(calcmass2);

	}

#ifndef USE_JACCACHE  
	// an easy-to-understand implementation
	Vector3d point_local_b = CalcBaseToBodyCoordinates(*model, q, body_index, pos, false);

	MatrixNd G_constr(MatrixNd::Zero(3, ndof));
	CalcPointJacobian(*model, q, body_index, point_local_b, G_constr, false);

	(*jac) = dir.transpose() * G_constr;
	(*MInvJacT) = M_llt.solve(jac->transpose());
#else
  // a more efficient version
  MatrixNd& G2=body->_jac_cache[body_index].G;
  MatrixNd& MinvG2T=body->_jac_cache[body_index].MinvGT;
  //SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), pos);
  //Eigen::Matrix<double, 3,6> pb=point_trans.toMatrix().bottomRows(3); // (-rx, I)

  //cout << "pb"<<pb<<endl;
  if(!body->_jac_cached(body_index))
  {
	  G2=MatrixNd::Zero(6, ndof);
	  calcSpatialJacobian(*model, q, body_index, G2);
#ifdef USE_MASS_MAT
	  MinvG2T=M_llt.solve(G2.transpose());
#else
	  MinvG2T=MatrixNd::Zero(ndof, 6);
	  auto& bi=*body->_bi;

	  //BEGIN_TIMER(MINV2);
#ifndef USE_PARTIAL_UPDATE
	  // easy to understand
	  for(int i=0; i<6; i++)
		  calcMInvTimesTau(*model, q, G2.row(i).transpose(), MinvG2T.col(i));
#else
	  // more efficient
	  prepareBackupChain(body, body->chain, body_index);

	  for(int i=0; i<6; i++)
	  {
		  calcMInvTimesTau_partial_chain(body->chain, body, *model, q, G2.row(i).transpose(), MinvG2T.col(i));
		  restoreBackupChain_pA_only(body, body->chain);
	  }
	  restoreBackupChain(body, body->chain);
#endif
	  //END_TIMER2(MINV2); // 4 us

	  body->_M_cached=true;
#endif
	  body->_jac_cached.set(body_index,true);
  }
  auto calcJac =[&] ( const Vector3d& dir, VectorNd* jac, VectorNd* MInvJacT) {
	  //Eigen::Matrix<double, 6,1> dirTpT=pb.transpose()*dir;
	  Eigen::Matrix<double, 6,1> dirTpT;
	  dirTpT.segment<3>(0)=pos.cross(dir);
	  dirTpT.segment<3>(3)=dir;

	  (*jac) = G2.transpose()*dirTpT;
	  (*MInvJacT) = MinvG2T*dirTpT;
  };
  calcJac(dir, jac, MInvJacT);
#endif

#if 0
  // verification
  cout << "jac"<< (*jac).transpose()<<endl;
  MatrixNd G2_constr(MatrixNd::Zero(6, ndof));
  calcSpatialJacobian(*model, q, body_index, G2_constr);

  SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), pos);
  MatrixNd dirTp=(dir.transpose()*point_trans.toMatrix().bottomRows(3));

  // jac== jac2 tested_okay. so G2_constr can be cached.
  cout << "jac2"<< dirTp*G2_constr<<endl;
  
  // eq 1
  // M * MinvJacT=jacT
  // M * MinvJacT=(dirTpoint*G2)'
  // M * MinvJacT=G2'*dirTpoint'
  //
  // eq 2
  // M * x = G2'
  // x = invM*G2'
  // MinvJacT=x*dirTpoint'
  MatrixNd x=M_llt.solve(G2_constr.transpose());

  // jac1 == m2 tested okay.
  cout <<"minvj"<< MInvJacT->transpose()<<endl;
  cout <<x.rows()<< ","<< x.cols()<<endl;
  cout <<"minvj2"<< (x*dirTp.transpose()).transpose()<<endl;
#endif
  *G_MInv_GT = (*jac).transpose().dot (*MInvJacT);
  assert(!isnan(*G_MInv_GT));


  bool USE_SMOOTHED_CONTACT_DYNAMICS = true;
  if(USE_SMOOTHED_CONTACT_DYNAMICS)
  {
	  // near rigid setting
	  double kappa=0.01; 
	  double epsilon=0.01;

	  double kp =   (1+epsilon) / (kappa*kappa);
	  double kd = (1+epsilon) / kappa;
	  double vn=0;
	  *bias_vel= 0.6*dt*(kp * (-0 + depth)-kd *vn);
  }
  else
  {
	  double beta = 0.01;
	  double delta_slop = 0.05;
	  printf("%f \n", depth);
	  *bias_vel =  -beta / dt * std::max (0., -depth - delta_slop);
  }
  if(restitution>0.0)
  {
	  double vn=(*jac).transpose().dot(qdot);
	  *bias_vel+= vn* restitution ;
  }

if(fric>0.0){
#ifdef USE_JACCACHE
	calcJac(tangents[0], &tangentJac[0], &tangentMInvJacT[0]);
	calcJac(tangents[1], &tangentJac[1], &tangentMInvJacT[1]);
#else

  tangentJac[0] = tangents[0].transpose() * G_constr;
  tangentJac[1] = tangents[1].transpose() * G_constr;
  tangentMInvJacT[0] = M_llt.solve(tangentJac[0].transpose());
  tangentMInvJacT[1] = M_llt.solve(tangentJac[1].transpose());
#endif
  tangentGMInvGT[0] = tangentJac[0].transpose().dot( tangentMInvJacT[0]);
  tangentGMInvGT[1] = tangentJac[1].transpose().dot( tangentMInvJacT[1]);

  assert (tangentGMInvGT[0] > 0.);
  assert (tangentGMInvGT[1] > 0.);
}
}
void PrepareConstraintImpulse(
    double dt,
	double MA,
    Trbdl::SimArticulatedBody* body_a,
	Trbdl::SimArticulatedBody* body_b,
    Trbdl::CollisionInfo& cinfo) {
  //ZoneScoped;

  CalcImpulseVariables(
      dt,
	  MA,
      body_a,
      cinfo.mBodyAIndex,
      cinfo.posA,
      cinfo.dir,
      cinfo.tangents,
      cinfo.depth,
      &cinfo.MInvJacTA,
      &cinfo.jacA,
      &cinfo.GMInvGTA,
      cinfo.tangentJacA,
      cinfo.tangentMInvJacTA,
      cinfo.tangentGMInvGTA,
      &cinfo.biasVelocityA,
      cinfo.effectiveRestitution,
	  cinfo.effectiveFriction);

  CalcImpulseVariables(
      dt,
	  MA,
      body_b,
      cinfo.mBodyBIndex,
      cinfo.posB,
      cinfo.dir,
      cinfo.tangents,
      -cinfo.depth,
      &cinfo.MInvJacTB,
      &cinfo.jacB,
      &cinfo.GMInvGTB,
      cinfo.tangentJacB,
      cinfo.tangentMInvJacTB,
      cinfo.tangentGMInvGTB,
      &cinfo.biasVelocityB,
      cinfo.effectiveRestitution,
	  cinfo.effectiveFriction
	  );

}


void Trbdl::DynamicsSimulator_Trbdl_impulse::clearJacCache()
{
	for(SimArticulatedBody* pbody : _bodies){
		pbody->_M_cached=false;
	}
}
void Trbdl::DynamicsSimulator_Trbdl_impulse::resolveCollisions(double dt, int num_iter, bool noContactForceCollection)
{

	BEGIN_TIMER(resolveConPart1);
	std::vector<Trbdl::CollisionInfo> mContactPoints;

	int contactStartIndex=0;
	intmatrixn bounds;
	int noc=0;
	for(int ichara=0; ichara<_bodies.size(); ichara++){
		SimArticulatedBody* pbody = _bodies[ichara];
		auto& limited=pbody->limitedBones;
		for(int i=0; i<limited.size(); i++)
		{
			int ibone=limited(i);
			auto& bone=pbody->_l->VRMLbone(ibone);
			auto& bi=*pbody->_bi;
			auto& link=bi.links[ibone];
			double bMin=TO_RADIAN(bone.mJoint->jointRangeMin);
			double bMax=TO_RADIAN(bone.mJoint->jointRangeMax);;
			int q_index=bi.model.mJoints[link.jointId].q_index;
			double q=bi.Q[q_index];

			if(q<bMin)
			{
				bounds.pushBack(intvectorn(3, ichara, ibone*-1, int(bMin*10000.0)));
				noc++;
			}
			else if(q>bMax)
			{
				bounds.pushBack(intvectorn(3, ichara, ibone, int(bMax*10000.0)));
				noc++;
			}
			//printf("bound %d %f in %f %f\n", i, q, bMin, bMax);
		}
	}
	contactStartIndex=noc;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		OpenHRP::CollisionPointSequence& points = (*collisions)[i].points;

		//noc+=points.size();
		for(int j=0; j<points.size(); j++)
			if(points[j].idepth>Trbdl::cCollisionEps )
				noc++;


	}
	mContactPoints.resize(noc);
	noc =0;

	for(int i=0; i<bounds.rows(); i++)
	{
		Trbdl::CollisionInfo & cinfo=mContactPoints[noc++];
		int ichara=bounds(i,0);
		VRMLloader& l=skeleton(ichara);
		int ibone=bounds(i,1);
		double sign=1.0;
		if(ibone<0) { ibone*=-1; sign=-1.0;}
		auto& bone=l.VRMLbone(ibone);
		auto* ppbone=(VRMLTransform*)(bone.parent());
		int ipbone=ppbone->treeIndex();
		double bound=bounds(i,2)/10000.0;
		
		vector3 axis=bone.axis(0);
		transf stopper;
		bone.getOffset(stopper.translation);
		stopper.rotation.setRotation(axis, bound);
		stopper.leftMult(getWorldState(ichara).global(ipbone));

		vector3 curr=getWorldState(ichara).global(ibone)*bone.localCOM();
		vector3 stop=stopper*bone.localCOM();
		vector3 normal;
		normal.cross(stopper.rotation*bone.localCOM(), axis) ;
		normal.normalize();
		normal*=sign;


		cinfo.mBodyA = _bodies[ichara];
		cinfo.mBodyAIndex = bodyInfo(ichara).links[ipbone].bodyId;
		cinfo.mBodyB = _bodies[ichara];
		cinfo.mBodyBIndex = bodyInfo(ichara).links[ibone].bodyId;
		cinfo.effectiveRestitution=0.0; // a.restitution*b.restitution
		cinfo.effectiveFriction=0.0;
		cinfo.dir=eigenView(normal);
		cinfo.depth=(stop-curr)%normal;
		cinfo.posA=eigenView(stop);
		cinfo.posB=eigenView(curr);

		/*
		TString name;
		name.format("bound%d", i);
		dbg.drawAxes(stop, name.ptr(), 1.0, 100.0);
		name.format("curr%d", i);
		dbg.drawAxes(curr, name.ptr(), 0.5, 100.0);

		printf("normal %s %f\n", normal.output().ptr(), cinfo.depth);
		*/
		sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);
	}
	assert(noc==contactStartIndex);

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		OpenHRP::CollisionPointSequence& points = (*collisions)[i].points;
		for(int j=0,nj=points.size(); j<nj; j++)
		{
			if(!(points[j].idepth>Trbdl::cCollisionEps )) continue;

			Trbdl::CollisionInfo & cinfo=mContactPoints[noc++];
			auto& point=points[j];
			int ichara0=linkPair.charIndex[0];
			int ichara1=linkPair.charIndex[1];

#ifdef COLLECT_CONTACT_FORCES
			cinfo.ilinkpair=i;
#endif
			cinfo.mBodyA = _bodies[ichara0];
			cinfo.mBodyAIndex = bodyInfo(ichara0).links[linkPair.link[0]->treeIndex()].bodyId;
			cinfo.mBodyB = _bodies[ichara1];
			cinfo.mBodyBIndex = bodyInfo(ichara1).links[linkPair.link[1]->treeIndex()].bodyId;
			cinfo.effectiveRestitution=_restitution; // a.restitution*b.restitution
			cinfo.effectiveFriction=0.5;
			cinfo.dir=eigenView(point.normal);
			cinfo.depth=point.idepth;
			cinfo.posA=eigenView(point.position+point.normal*point.idepth);
			cinfo.posB=eigenView(point.position);

			sCalcTangentVectors(cinfo.dir, &cinfo.tangents[0], &cinfo.tangents[1]);

		}
	}
	ASSERT(noc==mContactPoints.size());

	END_TIMER2(resolveConPart1);
	BEGIN_TIMER(prepareCon);
	for (Trbdl::CollisionInfo& cinfo : mContactPoints) {
		PrepareConstraintImpulse(dt, _MA, cinfo.mBodyA, cinfo.mBodyB, cinfo);
	}
	END_TIMER2(prepareCon);

	BEGIN_TIMER(APPLYcon);
	for (int i = 0; i < num_iter; i++) {
		for (Trbdl::CollisionInfo& cinfo : mContactPoints) {
			if(cinfo.effectiveFriction >0.0)
			{
				CalcFrictionImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
				ApplyFrictionImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
			}

			CalcConstraintImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
			ApplyConstraintImpulse(cinfo.mBodyA, cinfo.mBodyB, cinfo);
		}
	}
	END_TIMER2(APPLYcon);

#ifdef COLLECT_CONTACT_FORCES
	if(noContactForceCollection) return;
	for (Trbdl::CollisionInfo& cinfo : mContactPoints) {
		if(cinfo.ilinkpair!=-1)
		{
			vector3& cf=_contactForces(cinfo.ilinkpair);
			vector3& cpf=_contactPos(cinfo.ilinkpair);
			vector3 midPos=(trlView(cinfo.posA)+trlView(cinfo.posB))*0.5;
			vector3 f(0.0,0.0,0.0);
			if(cinfo.effectiveFriction >0.0)
			{
				auto& tangents=cinfo.tangents;
				f+=trlView(tangents[0])*cinfo.accumFrictionImpulse[0]
					+trlView(tangents[1])*cinfo.accumFrictionImpulse[1];
			}
			f+=trlView(cinfo.dir)*cinfo.accumImpulse;
			cf+=f;
			cpf+=midPos*f.length();
		}
	}
#endif

}
