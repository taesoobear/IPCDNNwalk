
#include "stdafx.h"
#include "DynamicsSimulator_Trbdl_LCP.h"
#include "BodyInfo.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/pldprimskin.h"
#include "LCPsolver.h"
#include "../TRL/TRL_common.h"
#include "../TRL/eigenSupport.h"
#include <limits>
#include "../../BaseLib/utility/QPerformanceTimer.h"
#ifndef NO_OGRE
#include <OgreSceneNode.h>
#include "../../MainLib/OgreFltk/renderer.h"
#endif

using namespace std;

#if 1 // set 1 to disable profiling
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)	
#define END_TIMER2(x)
#endif

//#define VERBOSE
void forwardKinematics ( Trbdl::BodyInfo& bi, Model &model, const VectorNd &Q, const VectorNd &QDot);
void forwardDynamics (Trbdl::BodyInfo& bi,  Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, const VectorNd &K_i, VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA) ;
void integrate(VRMLloader& l, Trbdl::BodyInfo& bi, double ts);


namespace Trbdl
{
	class GArticulatedBodyLink;

	class GArticulatedBody : public Trbdl::GBody {
		Trbdl::BodyInfo* body;
		public:
		friend class GArticulatedBodyLink;
		bool _allBodiesFixed;
		virtual bool _isStatic();
		std::vector<SpatialVector> df_ext;
#ifdef TEST_CONTACT_CACHE
		Eigen::MatrixXd _K;
		Eigen::VectorXd _a0_6;
		intvectorn bodyIndexToXIndex;
		intvectorn bodyIndexToAIndex;
		std::vector<short> useForceOnly;
		Eigen::VectorXd _x;
		Eigen::VectorXd _a;

		// for forwardDynamics_partial_chain
		std::vector <int> chain;
		std::vector<Math::SpatialVector> pA_backup;
		std::vector<double> u_backup;
		std::vector<Math::Vector3d> multdof3_u_backup;
		int _testForceMode;// -1 : cached, >=1 : normal
#endif
		DynamicsSimulator_Trbdl_LCP* _sim;

		inline Trbdl::BodyInfo* bodyInfo() { return body;}
		public:
		GArticulatedBody(int ichara, Trbdl::BodyInfo* _body, DynamicsSimulator_Trbdl_LCP* sim);
		virtual ~GArticulatedBody();

		enum {ArticulatedBody=0};
		virtual int getType() { return ArticulatedBody;}
		// j: linkIndex != treeIndex
		inline GArticulatedBodyLink& getlinksData(int j) { return (GArticulatedBodyLink&)(*linksData[j]);}
		virtual void calcAccelsWithoutExtForce();
		virtual void initialize();
		virtual void calcAccels();
		void clear_df_ext();


		void clearSkipInformation();
	};
}

bool Trbdl::GArticulatedBody::_isStatic() { return _allBodiesFixed;}
inline static Math::Vector3d _getAccelerationNoCache(Model& model, unsigned int body_id, vector3 const& contactPoint){
	Vector3d bodyPoint=Trbdl::calcBaseToBodyCoordinates (model, body_id,eigenView(contactPoint));
#ifdef VERBOSE
	cout <<"getacc"<< bodyId()<<" "<< contactPoint<<" "<<trlView(Trbdl::calcPointAcceleration(bi->model, bodyId(), bodyPoint))<<endl;
#endif
	return Trbdl::calcPointAcceleration ( model, body_id, bodyPoint); 
}
void forwardDynamics_preparePartial(Trbdl::GArticulatedBody& artiBody, Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, const VectorNd &K_i, VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA) 
{
	//LOG << "-------- " << __func__ << " --------" << std::endl;

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	// Reset the velocity of the root body
	model.v[0]=artiBody.bodyInfo()->v0;
	model.a[0]=artiBody.bodyInfo()->a0;


	auto& md=artiBody.bodyInfo()->model_data;
	Msg::verify(md.size()==model.mBodies.size(),"size error!!!");
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
		auto& bd=md[i];
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 1
				&& model.mJoints[i].mJointType != JointTypeCustom) {

			model.U[i] = model.IA[i] * model.S[i];
			model.d[i] = model.S[i].dot(model.U[i]); // sa.innerProduct(Is)
			// equivalent inertia
			//model.d[i]+=MA;
			model.d[i]+=K_i[q_index]; // stIs += k_i              (k_i==kd*dt)
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia =    model.IA[i]
					- model.U[i]
					* (model.U[i] / model.d[i]).transpose();

				Ia.diagonal().array()+=MA;
				bd.Ia_c=Ia* model.c[i];


				double stIs=model.d[i];
				double iStIs=(stIs>1e-5)?(1.0/stIs):0.0;

				if(iStIs==0.0)
					cout<<"iStIS...???"<<endl;
				bd.U_Dinv=model.U[i]*iStIs;

				SpatialVector pa =  model.pA[i]
					+ bd.Ia_c
					+ bd.U_Dinv * model.u[i] ;

				model.IA[lambda].noalias()
					+= model.X_lambda[i].toMatrixTranspose()
					* Ia * model.X_lambda[i].toMatrix();

				bd.pA0=model.X_lambda[i].applyTranspose(pa);
				model.pA[lambda] += bd.pA0;
			}
		} else if (model.mJoints[i].mDoFCount == 3
				&& model.mJoints[i].mJointType != JointTypeCustom) {
			model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
				// taesoo
			Eigen::Matrix<double, 3,3> jm2=Eigen::Matrix<double, 3,3>::Identity(3,3)*
				(K_i[q_index]);

			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
					* model.multdof3_U[i]+jm2).inverse().eval();
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
				bd.Ia_c=Ia*model.c[i];

				bd.U_Dinv=model.multdof3_U[i]* model.multdof3_Dinv[i];

				SpatialVector pa = model.pA[i]
					+ bd.Ia_c
					+ bd.U_Dinv
					* model.multdof3_u[i];
				model.IA[lambda].noalias()
					+= model.X_lambda[i].toMatrixTranspose()
					* Ia
					* model.X_lambda[i].toMatrix();

				bd.pA0=model.X_lambda[i].applyTranspose(pa);
				model.pA[lambda] += bd.pA0;
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
				bd.Ia_c=Ia*model.c[i];

				bd.U_Dinv=(model.mCustomJoints[kI]->U)* model.mCustomJoints[kI]->Dinv;

				SpatialVector pa =  model.pA[i]
					+ bd.Ia_c
					+ bd.U_Dinv 
					* model.mCustomJoints[kI]->u;

				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
					* Ia
					* model.X_lambda[i].toMatrix();
				bd.pA0=model.X_lambda[i].applyTranspose(pa);
				model.pA[lambda] += bd.pA0;
			}
		}
	}

	model.a[0] = spatial_gravity * -1.+artiBody.bodyInfo()->a0;
	//cout<<artiBody.bodyInfo()->a0.transpose()<<endl;

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
	model.a[0]=artiBody.bodyInfo()->a0;
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

	artiBody._precomputed=true;
}
// only external forces are modified from the previous call to forwardDynamics_preparePartial.
void forwardDynamics_partial( Trbdl::GArticulatedBody& artiBody, Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau, const VectorNd &K_i, VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA, bool visit_all_nodes=true) {
	//LOG << "-------- " << __func__ << " --------" << std::endl;

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);
	BEGIN_TIMER(abainit);

	unsigned int i = 0;

	// Reset the velocity of the root body
	model.v[0]=artiBody.bodyInfo()->v0;

	auto& md=artiBody.bodyInfo()->model_data;

	ASSERT(md.size()==model.mBodies.size());
	// will visit only visited nodes again.
	if(visit_all_nodes)
	{
		for(int i=0; i<md.size(); i++)
			md[i].visited=true;
	}

	int nb=md.size();
	for (i = 1; i < nb; i++) {
		auto& bd=md[i];
		unsigned int lambda = bd.lambda;
		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (bd.visited && (*f_ext)[i] != SpatialVector::Zero()) {
			model.pA[i] -= model.X_base[i].applyAdjoint((*f_ext)[i]);
			ASSERT(bd.visited==true);
		}
	}

	END_TIMER2(abainit);
	BEGIN_TIMER(abaphase1);
	//LOG << "--- first loop ---" << std::endl;
	for (i = nb - 1; i > 0; i--) {
		auto& bd=md[i];
		if(!bd.visited)
		{
			unsigned int lambda = bd.lambda;
			// unmodified pA
			if (lambda != 0) {
				model.pA[lambda] += bd.pA0;
			}
			continue;
		}
		unsigned int q_index = bd.q_index;
		ASSERT(model.mJoints[i].mJointType != JointTypeCustom);

		if (bd.ndof==1)
		{

			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

			unsigned int lambda = bd.lambda;
			if (lambda != 0) {

				SpatialVector pa =  model.pA[i] + bd.Ia_c + bd.U_Dinv* model.u[i] ;
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
			}
		} else if (bd.ndof==3)
		{
			model.multdof3_u[i] = Tau.segment<3>(q_index) - model.multdof3_S[i].transpose() * model.pA[i];

			unsigned int lambda = bd.lambda;
			if (lambda != 0) {
				SpatialVector pa = model.pA[i] + bd.Ia_c + bd.U_Dinv* model.multdof3_u[i];
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
			}
		} 
	}

	END_TIMER2(abaphase1);
	BEGIN_TIMER(abaphase2);
	// apply gravity
	{
		auto& a=model.a;
		a[0] = spatial_gravity * -1.+artiBody.bodyInfo()->a0; 

		int ni=md.size();
		for (i = 1; i < ni; i++) {
			auto& bd=md[i];
			if(!bd.visited) continue;
			unsigned int q_index = bd.q_index;
			unsigned int lambda = bd.lambda;

			a[i] = model.X_lambda[i].apply(a[lambda]) + model.c[i];

			if (bd.ndof==1)
			{
				QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(a[i]));
				a[i] += model.S[i] * QDDot[q_index];
			} else if (bd.ndof==3)
			{
				QDDot.segment<3>(q_index)= model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * a[i]);
				a[i] += model.multdof3_S[i] * QDDot.segment<3>(q_index);
			} 
		}
	}

	END_TIMER2(abaphase2);
	BEGIN_TIMER(abaphase3);
	// restore.
	model.a[0]=artiBody.bodyInfo()->a0;
	// calculate accelerations only. 
	for (i = 1; i < md.size(); i++) {
		auto& bd=md[i];
		if(!bd.visited) continue;
		unsigned int q_index = bd.q_index;
		unsigned int lambda = bd.lambda;
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

		if (bd.ndof == 1) {
			model.a[i] += model.S[i] * QDDot[q_index];
		} else if (bd.ndof == 3) {
			model.a[i] += model.multdof3_S[i] * QDDot.segment<3>(q_index);
		}
	}
	END_TIMER2(abaphase3);
	// apply gravity
}



#ifdef TEST_CONTACT_CACHE
void forwardDynamics_partial_chain(std::vector<int> const& chain,  Trbdl::GArticulatedBody& artiBody, Model &model, const VectorNd &Q, const VectorNd &QDot, const VectorNd &Tau,const VectorNd &K_i,  VectorNd &QDDot, std::vector<SpatialVector> *f_ext, double MA) {
	//LOG << "-------- " << __func__ << " --------" << std::endl;

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);
	BEGIN_TIMER(abainit);

	unsigned int i = 0;

	// Reset the velocity of the root body
	model.v[0]=artiBody.bodyInfo()->v0;

	auto& md=artiBody.bodyInfo()->model_data;

	ASSERT(md.size()==model.mBodies.size());
	// will visit only visited nodes again.

	int nb=md.size();
	// abaphase 1 and 2: calc pA and u
	//for (i = 1; i < nb; i++) {
	for(int ii=0; ii<chain.size(); ii++)
	{
		i=chain[chain.size()-ii-1];
		auto& bd=md[i];
		ASSERT(bd.constrained);
		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (bd.visited && (*f_ext)[i] != SpatialVector::Zero()) {
			model.pA[i] -= model.X_base[i].applyAdjoint((*f_ext)[i]);
			ASSERT(bd.visited==true);
		}
	}

	END_TIMER2(abainit);
	BEGIN_TIMER(abaphase1);
	//LOG << "--- first loop ---" << std::endl;
	for (i = nb - 1; i > 0; i--) {
	//for(int ii=0; ii<chain.size(); ii++)
	//{
	//	i=chain[ii];
		auto& bd=md[i];
		// constrained : in this chain
		// visited : in any of a constrained chain
		// printf("visited %d %d: %d %d\n", i, bd.lambda, bd.constrained, bd.visited);
		if(!bd.constrained)
		{
			unsigned int lambda = bd.lambda;
			// unmodified pA
			if (lambda != 0 && md[lambda].constrained) {
				model.pA[lambda] += bd.pA0;
			}
			continue;
		}

		//if(!bd.visited) continue; doesn't work
		unsigned int q_index = bd.q_index;
		ASSERT(model.mJoints[i].mJointType != JointTypeCustom);

		if (bd.ndof==1)
		{

			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

			unsigned int lambda = bd.lambda;
			if (lambda != 0) {

				SpatialVector pa =  model.pA[i] + bd.Ia_c + bd.U_Dinv* model.u[i] ;
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
			}
		} else if (bd.ndof==3)
		{
			model.multdof3_u[i] = Tau.segment<3>(q_index) - model.multdof3_S[i].transpose() * model.pA[i];

			unsigned int lambda = bd.lambda;
			if (lambda != 0) {
				SpatialVector pa = model.pA[i] + bd.Ia_c + bd.U_Dinv* model.multdof3_u[i];
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
			}
		} 
	}

	END_TIMER2(abaphase1);
	BEGIN_TIMER(abaphase2);
	// apply gravity (updates Qddot from u and a[lambda})
	{
		auto& a=model.a;
		a[0] = spatial_gravity * -1.+artiBody.bodyInfo()->a0; 

		int ni=md.size();
		for (i = 1; i < ni; i++) {
			auto& bd=md[i];
			if(!bd.visited) continue;
			unsigned int q_index = bd.q_index;
			unsigned int lambda = bd.lambda;

			a[i] = model.X_lambda[i].apply(a[lambda]) + model.c[i];

			if (bd.ndof==1)
			{
				QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(a[i]));
				a[i] += model.S[i] * QDDot[q_index];
			} else if (bd.ndof==3)
			{
				QDDot.segment<3>(q_index)= model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * a[i]);
				a[i] += model.multdof3_S[i] * QDDot.segment<3>(q_index);
			} 
		}
	}

	END_TIMER2(abaphase2);
	BEGIN_TIMER(abaphase3);
	// restore.
	model.a[0]=artiBody.bodyInfo()->a0;
	// calculate accelerations only. 
	for (i = 1; i < md.size(); i++) {
		auto& bd=md[i];
		if(!bd.visited) continue;
		unsigned int q_index = bd.q_index;
		unsigned int lambda = bd.lambda;
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

		if (bd.ndof == 1) {
			model.a[i] += model.S[i] * QDDot[q_index];
		} else if (bd.ndof == 3) {
			model.a[i] += model.multdof3_S[i] * QDDot.segment<3>(q_index);
		}
	}
	END_TIMER2(abaphase3);



}
#endif

class Trbdl::GArticulatedBodyLink :public Trbdl::GLink  {
	friend class GArticulatedBody;
		Trbdl::BodyInfo* bi;	
		Trbdl::GArticulatedBody* body;
		inline int bodyId() const { return index;}
	public:
		GArticulatedBodyLink(Trbdl::GArticulatedBody* _body, Trbdl::Link* _link): GLink(_body, _link->bodyId)
		{
			bi=_body->body;
			body=_body;
		}
		virtual ~GArticulatedBodyLink()
		{
		}
		virtual bool isStatic(){
			return false;
		}
		virtual void addConstraintForce(vector3 const& p, int nodeIndex, vector3 const& f)
		{
#ifdef VERBOSE
			cout <<"conforce"<< p<<" "<<f<<endl;
#endif
			auto& f_ext=bi->f_ext[bodyId()];
			f_ext.segment<3>(3)+=eigenView(f);
			f_ext.segment<3>(0)+=eigenView(p.cross(f));
		}
		virtual vector3 getVelocity(vector3 const& contactPoint, int contactPointIndex){
			Vector3d bodyPoint= calcBaseToBodyCoordinates (bi->model, bodyId(),eigenView(contactPoint));
#ifdef VERBOSE
			cout <<"getvel"<< contactPoint<<" "<<trlView(calcPointVelocity(bi->model, bodyId(), bodyPoint))<<endl;
#endif
			return trlView(calcPointVelocity(bi->model, bodyId(), bodyPoint));

			/* tested ok
			transf T=bi->globalFrame(bodyId());
			cout<<"bodyp"<<bodyPoint<<"=="<<T.toLocalPos(contactPoint)<<endl;
			*/
		}
#ifdef TEST_CONTACT_CACHE
		inline Vector3d calcPointAccelerationFromCache( Model &model, unsigned int reference_body_id, const Vector3d &body_point) 
		{
			int ci=body->bodyIndexToAIndex(bodyId());
			// transformation at the contact point.
			SpatialTransform p_X_i ( model.X_base[reference_body_id].E.transpose(), body_point);
			SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
			Vector3d a_dash = p_v_i.segment<3>(0).cross(p_v_i.segment<3>(3));
			SpatialVector p_a_i = p_X_i.apply(body->_a.segment<6>(ci));

			return p_a_i.segment<3>(3)+a_dash.segment<3>(0);
		}
#endif
		virtual vector3 getAcceleration(vector3 const& contactPoint, int contactPointIndex){
#ifdef TEST_CONTACT_CACHE
			if(body->_testForceMode>=1)
				return trlView(_getAccelerationNoCache(bi->model, bodyId(), contactPoint));
#ifdef _DEBUG
			int ci=body->bodyIndexToAIndex(bodyId());
			ASSERT(ci>=0);
#endif
			Vector3d bodyPoint=calcBaseToBodyCoordinates (bi->model, bodyId(),eigenView(contactPoint));
			return trlView(calcPointAccelerationFromCache ( bi->model, bodyId(), bodyPoint)); 
#else

			return trlView(_getAccelerationNoCache(bi->model, bodyId(), contactPoint));
#endif
			/* tested ok
			Vector3d acc1=calcPointAcceleration ( bi->model, bodyId(), bodyPoint); 
			Vector3d acc2=CalcPointAcceleration ( bi->model, bi->Q, bi->QDot, bi->QDDot, bodyId(), bodyPoint, false); 
			cout <<"acc"<<acc1<<"=="<<acc2<<endl;
			return trlView(acc1);
			*/
		}
		inline void _addTestForceNoCache(vector3 const& f, vector3 const& p, int index)
		{
			auto& df_ext=body->df_ext[bodyId()];
			df_ext.segment<3>(3)+=eigenView(f);
			df_ext.segment<3>(0)+=eigenView(p.cross(f));
		}
		virtual void addTestForce(vector3 const& f, vector3 const& p, int index)
		{
#ifdef VERBOSE
			cout <<"addTestForce"<<bodyId()<<" "<< f<<" "<<p<<endl;
#endif
			ASSERT(body->df_ext.size()>bodyId());

#ifdef TEST_CONTACT_CACHE

			int ci=body->bodyIndexToXIndex(bodyId());

			if(body->useForceOnly[bodyId()])
			{
				body->_testForceMode=bodyId()+1;
				_addTestForceNoCache(f,p, index);
			}
			else
			{
				ASSERT(body->_testForceMode<=0);
				body->_testForceMode=-1;
				body->_x.segment<3>(ci+3)+=eigenView(f);
				body->_x.segment<3>(ci+0)+=eigenView(p.cross(f));
			}
#else
			_addTestForceNoCache(f,p, index);
#endif

		}
		virtual void updateSkipInformation(GBody* body, int constraintIndex ){
			GArticulatedBody* pAB=(GArticulatedBody*)body;
			auto& md=pAB->bodyInfo()->model_data;
			int l=index;

#ifdef TEST_CONTACT_CACHE
			md[l].numConstraints++;
			md[l].lastGlobalIndex=constraintIndex ;
#endif
			while (l != 0) {
				if(md[l].visited) break;
				md[l].visited=true;
				l=md[l].lambda;
			}
		}
};

Trbdl::GArticulatedBody::GArticulatedBody(int ichara, Trbdl::BodyInfo* _body, DynamicsSimulator_Trbdl_LCP* sim):Trbdl::GBody(), body(_body)
{
	_sim=sim;
	int numBone=sim->skeleton(ichara).numBone();
	linksData.resize(numBone);
	// initialize link data
	for(int i=0; i<numBone; i++)
	{
		linksData[i]=new GArticulatedBodyLink(this, &_body->links[i]);
		cout <<_body->links[i].bodyId<<" ";
		cout <<linksData[i]->index<<" ";
	}
	_allBodiesFixed=false;
	if(sim->skeleton(ichara).VRMLbone(1).HRPjointType(0)==HRP_JOINT::FIXED && numBone==2)
		_allBodiesFixed=true;
	printf("linksData created %d %d %d\n", ichara, sim->skeleton(ichara).VRMLbone(1).HRPjointType(0), numBone);

#ifdef TEST_CONTACT_CACHE
	_testForceMode=0;
#endif
}

void Trbdl::GArticulatedBody::clearSkipInformation()
{
	body->model_data.resize(body->model.mBodies.size());
	for(int i=0; i< body->model_data.size(); i++)
	{
		auto& body_data=body->model_data[i];
		body_data.visited=false;
#ifdef TEST_CONTACT_CACHE
		body_data.numConstraints=0;
#endif
		body_data.lambda=body->model.lambda[i];
		body_data.ndof=body->model.mJoints[i].mDoFCount;
		body_data.q_index=body->model.mJoints[i].q_index;

	}
}
Trbdl::GArticulatedBody::~GArticulatedBody()
{

}

#ifdef TEST_CONTACT_CACHE
inline void fillKmatrix(int colIndex, std::vector<int> const& chain,  Trbdl::GArticulatedBody& artiBody, const VectorNd &k_i, std::vector<SpatialVector>& df_ext,std::vector<int> &bodyIndexFromCBI, vector3N const & cp)
{
	auto& bi=*artiBody.bodyInfo();
	auto& bodyIndexToConstraintIndex=artiBody.bodyIndexToXIndex;
	auto& K=artiBody._K;
	auto& _a0_6=artiBody._a0_6;

	int n_cbodies=bodyIndexFromCBI.size();
	//forwardDynamics_partial(*this, bi.model, bi.Q, bi.QDot, bi.Tau, bi.QDDot, &df_ext, _sim->_MA, false);
	forwardDynamics_partial_chain(chain,  artiBody, bi.model, bi.Q, bi.QDot, bi.Tau, k_i, bi.QDDot, &df_ext, artiBody._sim->_MA);

	for(int j=0; j<n_cbodies; j++)
	{
		// get acceleration of bodyIndexFromCBI[j]
		int bij=bodyIndexFromCBI[j];
		int cij=bodyIndexToConstraintIndex(bij);
		K.block(cij, colIndex,6,1)=bi.model.a[bij]-_a0_6.segment<6>(j*6);
	}
}

inline static void  prepareBackupChain(Trbdl::GArticulatedBody& artiBody, std::vector<int> & chain, int bii)
{
	auto& bi=*artiBody.bodyInfo();
	chain.resize(0);
	{
		auto& md=bi.model_data;
		int nb=md.size();
		chain.reserve(nb);
		int l=bii;

		while (l != 0) {
			md[l].constrained=true;
			chain.push_back(l);
			l=md[l].lambda;
		}

		for(int i=0; i<chain.size(); i++)
		{
			artiBody.pA_backup[chain[i]]=bi.model.pA[chain[i]];
			artiBody.u_backup[chain[i]]=bi.model.u[chain[i]];
			artiBody.multdof3_u_backup[chain[i]]=bi.model.multdof3_u[chain[i]];
		}
	}
}

inline static void  restoreBackupChain(Trbdl::GArticulatedBody& artiBody, std::vector<int> & chain)
{
	auto& bi=*artiBody.bodyInfo();
	auto& md=bi.model_data;
	for(int i=0; i<chain.size(); i++)
	{
		bi.model.pA[chain[i]]=artiBody.pA_backup[chain[i]];
		bi.model.u[chain[i]]=artiBody.u_backup[chain[i]];
		bi.model.multdof3_u[chain[i]]=artiBody.multdof3_u_backup[chain[i]];
		md[chain[i]].constrained=false;
	}
}
#endif
void Trbdl::GArticulatedBody::calcAccelsWithoutExtForce(){
#ifdef VERBOSE
	cout<<"calcAccelsW/OextForce"<<endl;
#endif
	auto& bi=*body;
	/*
	mass 계산은 시간 재보니 너무 오래걸림. 그냥 fd 여러번으로 풀겠음.
	{
	BEGIN_TIMER(calcMass);

	int ndof=bi.model.dof_count;
	Math::VectorNd C(ndof);
	Math::MatrixNd H(ndof,ndof);
	C.setZero();
	H.setZero();
	bi.QDDot.setZero();
	InverseDynamics (bi.model, bi.Q, bi.QDot, bi.QDDot, C, &df_ext);
	CompositeRigidBodyAlgorithm (bi.model, bi.Q, H, false);

	END_TIMER2(calcMass);
	}
	*/

	auto k_i=bi.kds*bi._timestep;
	forwardDynamics_preparePartial ( *this, bi.model, bi.Q, bi.QDot, bi.Tau, k_i, bi.QDDot, NULL, _sim->_MA);

#ifdef TEST_CONTACT_CACHE
	{
		auto& md=bi.model_data;
		int nb=md.size();
		for (int ib = 1; ib < nb; ib++) {
			auto& bd=md[ib];
			bd.constrained=false;
		}
	}

#ifdef VERBOSE
	printf("model_data size %d", (int)body->model_data.size());
#endif
	std::vector<int> bodyIndexFromCBI;
	bodyIndexFromCBI.reserve(2);

	int nrows=0;
	useForceOnly.resize(df_ext.size());
	for(int i=0; i< body->model_data.size(); i++)
	{
		auto& lcpdata=body->model_data[i];
		if(lcpdata.numConstraints) 
		{
			bodyIndexFromCBI.push_back(i);
			useForceOnly[i]=lcpdata.numConstraints==1; // because caching is slower in this case.
			//useForceOnly[i]=0;

			if(useForceOnly[i])
				nrows+=0;
			else
				nrows+=6;
		}
	}
#ifdef VERBOSE
	printf(" cached %d\n", (int)bodyIndexFromCBI.size());
#endif
	// add testforce...
	// getdefa..
	
	int n_cbodies=bodyIndexFromCBI.size();
	BEGIN_TIMER(cbi);
	_K.resize(n_cbodies*6, nrows);
	_a0_6.resize(n_cbodies*6);
	_x.resize(nrows);
	_a.resize(n_cbodies*6);
	bodyIndexToXIndex.setSize(df_ext.size());
	bodyIndexToXIndex.setAllValue(-1);
	bodyIndexToAIndex.setSize(df_ext.size());
	bodyIndexToAIndex.setAllValue(-1);


	_x.setZero();

	int irow=0;
	vector3N cp(n_cbodies);
	for(int icb=0; icb<n_cbodies; icb++)
	{
		int ibody=bodyIndexFromCBI[icb];
		bodyIndexToXIndex[ibody]=irow;
		bodyIndexToAIndex[ibody]=icb*6;
		auto* lcpdata=&body->model_data[ibody];

		_a0_6.segment<6>(icb*6)=bi.model.a[ibody];
		if (useForceOnly[ibody])
			irow+=0;
		else
			irow+=6;
	}
	ASSERT(irow==nrows);


	pA_backup.resize(df_ext.size());
	u_backup.resize(df_ext.size());
	multdof3_u_backup.resize(df_ext.size());

	for(int icb=0; icb<n_cbodies; icb++)
	{
		// apply test force k to bodyIndexFromCBI[icb]
		int bii=bodyIndexFromCBI[icb];


		if (useForceOnly[bii])
		{
		}
		else
		{
			prepareBackupChain(*this, chain, bii);
			for(int k=0; k<6; k++)
			{

				df_ext[bii][k]=1.0;
				fillKmatrix(bodyIndexToXIndex(bii)+k, chain, *this, k_i, df_ext, bodyIndexFromCBI, cp);
				df_ext[bii][k]=0.0;
			}
			restoreBackupChain(*this, chain);
		}



	}
#ifdef VERBOSE
			cout <<"#K"<< _K<<endl;
#endif
	_a=_a0_6;
	_testForceMode=-1; 
	for(int icb=0; icb<n_cbodies; icb++)
	{
		int ibody=bodyIndexFromCBI[icb];
		bi.model.a[ibody]=_a0_6.segment<6>(icb*6);
	}
	END_TIMER2(cbi);
#endif
}

void Trbdl::GArticulatedBody::initialize(){
	GBody::initialize();
	body->clearExternalForces();
	ASSERT(body->model.a.size()!=0);
	df_ext.resize(body->model.a.size());
	clear_df_ext();
}
void Trbdl::GArticulatedBody::clear_df_ext()
{
	for(int i=0; i<df_ext.size(); i++)
		df_ext[i]=SpatialVector::Zero();
#ifdef TEST_CONTACT_CACHE
	_x.setZero();
#endif
}
void Trbdl::GArticulatedBody::calcAccels()
{
#ifdef VERBOSE
	cout<<"calcAccels"<<endl;
#endif

#ifdef TEST_CONTACT_CACHE
	ASSERT(_testForceMode!=0);

	if(_testForceMode>=1)
	{
		auto& bi=*body;
		//forwardDynamics_partial (*this, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_sim->_timestep, bi.QDDot, &df_ext, _sim->_MA, false);
		//
		int bii=_testForceMode-1;
		prepareBackupChain(*this, chain, bii);
		forwardDynamics_partial_chain(chain,  *this, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_sim->_timestep, bi.QDDot, &df_ext, _sim->_MA);
		restoreBackupChain(*this, chain);
	}
	else
		_a=_K*_x+_a0_6;
#else
	auto& bi=*body;
	forwardDynamics_partial (*this, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_sim->_timestep, bi.QDDot, &df_ext, _sim->_MA, false);
#endif
	clear_df_ext();
}

void Trbdl::DynamicsSimulator_Trbdl_LCP::_registerCollisionCheckPair(int bodyIndex1, int bodyIndex2, int treeIndex1, int treeIndex2, vectorn const& param)
{
	double staticFriction=param[0];
	double slipFriction=param[1];

	const double epsilon = 0.0;
	if(bodyIndex1 >= 0 && bodyIndex2 >= 0){
		VRMLloader* skel1=&skeleton(bodyIndex1);
		VRMLloader* skel2=&skeleton(bodyIndex2);

		GLink* link1= body(bodyIndex1)->linksData[treeIndex1];
		GLink* link2= body(bodyIndex2)->linksData[treeIndex2];

		if(link1 && link2 && link1 != link2){
			bool ok = _contactForceSolver->addCollisionCheckLinkPair
				(bodyIndex1, link1, bodyIndex2, link2, staticFriction, slipFriction, epsilon);
			if(ok){

				collisionDetector->_addCollisionPair(bodyIndex1, treeIndex1, bodyIndex2, treeIndex2);
				//printf("pair added %d:%d %d:%d\n",bodyIndex1, treeIndex1, bodyIndex2,treeIndex2);
			}
		}
	}
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::registerAllCollisionCheckPairs(int bodyIndex1, int bodyIndex2, vectorn const& param)
{
	VRMLloader* skel1=&skeleton(bodyIndex1);
	VRMLloader* skel2=&skeleton(bodyIndex2);
	if (skel1!=skel2)
	{
		for(int i=1; i<skel1->numBone(); i++)
			for(int j=1; j<skel2->numBone(); j++)
				_registerCollisionCheckPair(bodyIndex1, bodyIndex2, i, j, param);
	}
	else
	{
		Msg::error("use registerCollisionCheckPair instead!!!");
	}
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::registerCollisionCheckPair
(
 const char *charName1,
 const char *linkName1,
 const char *charName2,
 const char *linkName2,
 vectorn const& param
 )
{
	int bodyIndex1 = findCharacter(charName1);
	int bodyIndex2 = findCharacter(charName2);

	if(bodyIndex1 >= 0 && bodyIndex2 >= 0){

		int treeIndex1=skeleton(bodyIndex1).getTreeIndexByName(linkName1);
		int treeIndex2=skeleton(bodyIndex2).getTreeIndexByName(linkName2);
		_registerCollisionCheckPair(bodyIndex1, bodyIndex2, treeIndex1, treeIndex2, param);
	}
}

//mod:같은 이름을 가진 서로다른 model을 위한 Method추가 //2018/04/05
void Trbdl::DynamicsSimulator_Trbdl_LCP::registerCollisionCheckPair
(
	const int bodyIndex1,
	const char *linkName1,
	const int bodyIndex2,
	const char *linkName2,
	vectorn const& param
)
{
	if(bodyIndex1 >= 0 && bodyIndex2 >= 0)
	{
		int treeIndex1=skeleton(bodyIndex1).getTreeIndexByName(linkName1);
		int treeIndex2=skeleton(bodyIndex2).getTreeIndexByName(linkName2);
		_registerCollisionCheckPair(bodyIndex1, bodyIndex2, treeIndex1, treeIndex2, param);
	}

}	


void Trbdl::DynamicsSimulator_Trbdl_LCP::get_contact_pos(int ichar, vector3N & M, OpenHRP::CollisionSequence& collisionSequence) const
{
	const VRMLloader& skel=skeleton(ichar);
	int noc =0;

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		OpenHRP::CollisionPointSequence& points = collisionSequence[i].points;
		if(linkPair.charIndex[0]==ichar )
			noc+=points.size();
		if(linkPair.charIndex[1]==ichar)
			noc+=points.size();
	}
	M.setSize(noc);
	int cc=0;
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		OpenHRP::LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		OpenHRP::CollisionPointSequence& points = collisionSequence[i].points;

		int n_point = points.size();
		if(n_point == 0) continue;
		if(linkPair.charIndex[0]!=ichar && linkPair.charIndex[1]!=ichar) continue;

		for(int j=0; j<n_point; j++)
		{
			const vector3& pos=points[j].position;  // global
			
			if(linkPair.charIndex[0]==ichar)
			{
				M[cc]=pos;
				cc++;
			}
			if(linkPair.charIndex[1]==ichar)
			{
				M[cc]=pos;
				cc++;
			}
		}
	}
	assert(cc==noc);
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::_registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo)
{
	Trbdl::DynamicsSimulator_Trbdl_penalty::_registerCharacter(name, cinfo);
	if (cinfo.loader->constraints.size()>0)
	{
		auto& array_con=cinfo.loader->constraints;
		for(int i=0; i<array_con.size(); i++)
		{
			auto& con=array_con[i];
			addRelativeConstraint(
					_characters.size()-1, 
					cinfo.loader->bone(con.ibone1),
					con.localpos1,
					cinfo.loader->bone(con.ibone2),
					con.localpos2);
		}
	}
	int ichara=_characters.size()-1;
	Trbdl::BodyInfo* chara=&bodyInfo(ichara);
	_addBody(new GArticulatedBody(ichara, chara, this));
	int ibody=_bodies.size()-1;
	assert(ibody==ichara);
}

// defined in DynamicsSimulator.cpp
void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output);

void Trbdl::DynamicsSimulator_Trbdl_LCP::_addBody(GBody* body)
{
	_bodies.resize(_bodies.size()+1);
	_bodies.back()=body;
	_bodies.back()->initialize();
}
void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output);
void Trbdl::DynamicsSimulator_Trbdl_LCP::_addEmptyCharacter(const char* name)
{
	// add empty character first
	OpenHRP::CharacterInfo cinfo;
	VRMLloader* l=new VRMLloader("../Resource/mesh/empty.wrl");
	l->name=name;
	makeCharacterInfo(*l, cinfo);
	Trbdl::DynamicsSimulator_Trbdl_penalty::_registerCharacter(cinfo.name, cinfo);
}






Trbdl::DynamicsSimulator_Trbdl_LCP::DynamicsSimulator_Trbdl_LCP()
	:
		DynamicsSimulator_Trbdl_penalty("libccd")
{
	_MA=0;
	_contactForceSolver=new LCPsolver(*this);
}
Trbdl::DynamicsSimulator_Trbdl_LCP::DynamicsSimulator_Trbdl_LCP(const char* coldet)
	:
		DynamicsSimulator_Trbdl_penalty(coldet)
{
	_MA=0;
	_contactForceSolver=new LCPsolver(*this);
}
Trbdl::DynamicsSimulator_Trbdl_LCP::~DynamicsSimulator_Trbdl_LCP()
{
	delete _contactForceSolver; _contactForceSolver=NULL;
}

void Trbdl::DynamicsSimulator_Trbdl_LCP::initSimulation()
{
	int n = numSkeleton();
	for(int i=0; i < n; ++i){
		auto &bi=bodyInfo(i);

		forwardKinematics(bi, bi.model, bi.Q, bi.QDot);
		bi.clearExternalForces();
	}
	_contactForceSolver->initialize();
	_updateCharacterPose();
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
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


	_contactForceSolver->initialize();
	if(_contactForceSolver->collisionCheckLinkPairs.size()>0)
	{
		for(int i=0; i<_contactForceSolver->collisionCheckLinkPairs.size(); i++)
		{
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[0]);
			assert(_contactForceSolver->collisionCheckLinkPairs[i].bodyData[1]);
		}
	}
}


void Trbdl::DynamicsSimulator_Trbdl_LCP::setParam_Epsilon_Kappa(double eps, double kap)
{
	_contactForceSolver->epsilon=eps;
	_contactForceSolver->kappa=kap;
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::setParam_R_B_MA(double r, double b, double ma)
{
	_contactForceSolver->_R=r;
	_MA=ma;
}

//

bool Trbdl::DynamicsSimulator_Trbdl_LCP::stepSimulation()
{
	BEGIN_TIMER(colCheck);
	// collision check rigid-rigid pairs
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	END_TIMER2(colCheck);


	BEGIN_TIMER(colSolve);
	// set external forces
	if(_contactForceSolver->solve(*collisions))
	{
		BEGIN_TIMER(colSolve2);
		for(int ichara=0; ichara<numSkeleton(); ichara++)
		{
			VRMLloader& l=skeleton(ichara);
			auto& bi=bodyInfo(ichara);
			auto* pbody=(GArticulatedBody*)body(ichara);
			if(! pbody->isStatic){
				Msg::verify(pbody->_precomputed, "not precomputed??? %d ", ichara);
				forwardDynamics_partial ( *pbody, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_timestep, bi.QDDot, &bi.f_ext, _MA);
			}
		}
		END_TIMER2(colSolve2);
	}
	else
	{
		for(int ichara=0; ichara<numSkeleton(); ichara++)
		{
			VRMLloader& l=skeleton(ichara);
			auto& bi=bodyInfo(ichara);
			forwardDynamics ( bi, bi.model, bi.Q, bi.QDot, bi.Tau, bi.kds*_timestep, bi.QDDot, &bi.f_ext, _MA);
		}
	}
	END_TIMER2(colSolve);

	BEGIN_TIMER(integ);
	for(int ichara=0; ichara<numSkeleton(); ichara++)
	{
		VRMLloader& l=skeleton(ichara);
		auto& bi=bodyInfo(ichara);
		bi.clearExternalForces();

		integrate(l, bi,_timestep);
	}
	END_TIMER2(integ);
	_currentTime+=_timestep;
	BEGIN_TIMER(updatepose);
	_updateCharacterPose();
	END_TIMER2(updatepose);

	return true;
}

vector3 Trbdl::DynamicsSimulator_Trbdl_LCP::getContactForce(int ichar, int ibone) const
{
	vectorn const& f=_f;

    int n = _contactForceSolver->constrainedLinkPairs.size();
	if(n==0) return vector3 (0,0,0);

	intvectorn convLPindex(collisionDetector->getCollisionPairs().size());
	convLPindex.setAllValue(-1);
	int nn=0;
	assert(collisions->seq.size()==convLPindex.size() );
	for(int i=0; i<convLPindex.size(); i++)
	{
		OpenHRP::CollisionPointSequence& points = (*collisions)[i].points;
		int n_point = points.size();
		if(n_point>0)
			convLPindex(nn++)=i;
	}

	assert(n==nn);
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;

	vector3 contactForce(0,0,0);

    for(int i=0; i < n; ++i){
		Trbdl::LCPsolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		const OpenHRP::LinkPair& linkPair2 = collisionDetector->getCollisionPairs()[convLPindex[i]];

		for(int j=0; j < 2; ++j){
			if(linkPair->bodyIndex[j]==ichar && 
				linkPair2.link[j]->treeIndex()==ibone){
				int ipair=j;
				Trbdl::LCPsolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				for(int i=0; i < numConstraintPoints; ++i){
					Trbdl::LCPsolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					assert(linkPair->bodyIndex[0]==linkPair2.charIndex[0]);
					assert(linkPair->bodyIndex[1]==linkPair2.charIndex[1]);

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					contactForce+=ff;
				}
			}
		}
	}
	return contactForce;
}
Liegroup::dse3 Trbdl::DynamicsSimulator_Trbdl_LCP::getCOMbasedContactForce(int ichar, int ibone) const
{
	Liegroup::dse3 out(0);

	vectorn const& f=_f;

    int n = _contactForceSolver->constrainedLinkPairs.size();
	if(n==0) return out;

	intvectorn convLPindex(collisionDetector->getCollisionPairs().size());
	convLPindex.setAllValue(-1);
	int nn=0;
	assert(collisions->seq.size()==convLPindex.size() );
	for(int i=0; i<convLPindex.size(); i++)
	{
		OpenHRP::CollisionPointSequence& points = (*collisions)[i].points;
		int n_point = points.size();
		if(n_point>0)
			convLPindex(nn++)=i;
	}

	assert(n==nn);
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;

	vector3 contactForce(0,0,0);
	vector3 contactTorque(0,0,0);

	VRMLloader* skel=_characters[ichar]->skeleton;
	VRMLTransform& bone=skel->VRMLbone(ibone);
	vector3 com=getWorldState(ichar).global(ibone)*bone.localCOM();

    for(int i=0; i < n; ++i){
		Trbdl::LCPsolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		const OpenHRP::LinkPair& linkPair2 = collisionDetector->getCollisionPairs()[convLPindex[i]];
		OpenHRP::CollisionPointSequence& points = (*collisions)[convLPindex[i]].points;

		for(int j=0; j < 2; ++j){
			if(linkPair->bodyIndex[j]==ichar){
				int ipair=j;
				Trbdl::LCPsolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				int n_point = points.size();
				Msg::verify(n_point==numConstraintPoints, "???");

				for(int i=0; i < numConstraintPoints; ++i){
					Trbdl::LCPsolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					assert(linkPair->bodyIndex[0]==linkPair2.charIndex[0]);
					assert(linkPair->bodyIndex[1]==linkPair2.charIndex[1]);

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					const vector3& pos=points[i].position;  // global
					contactForce+=ff;
					vector3 r=pos-com;
					contactTorque+=r.cross(ff);
				}
			}
		}
	}
	out.M()=contactTorque;
	out.F()=contactForce;
	return out;
}
void Trbdl::DynamicsSimulator_Trbdl_LCP::drawLastContactForces(int ichar, vector3 const& draw_offset) const 
{
	static ObjectList g_debugDraw;
	get_contact_pos(ichar, contactPos, *collisions);

	vectorn& f=_f;

	f.setSize(contactPos.size()*3);

    int n = _contactForceSolver->constrainedLinkPairs.size();
	vectorn& solution=_contactForceSolver->solution;
	int globalNumConstraintVectors=_contactForceSolver->globalNumConstraintVectors;
	int cc=0;

    for(int i=0; i < n; ++i){
		Trbdl::LCPsolver::LinkPair* linkPair = _contactForceSolver->constrainedLinkPairs[i];
		for(int j=0; j < 2; ++j){
			if(linkPair->bodyIndex[j]==ichar){

				int ipair=j;
				Trbdl::LCPsolver::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
				int numConstraintPoints = constraintPoints.size();

				for(int i=0; i < numConstraintPoints; ++i){
					Trbdl::LCPsolver::ConstraintPoint& constraint = constraintPoints[i];
					int globalIndex = constraint.globalIndex;

					vector3 ff(solution(globalIndex) * constraint.normalTowardInside(ipair));

					for(int j=0; j < 2; ++j){
						ff += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
					}

					f.setVec3(cc*3, ff);
					cc++;
				}
			}
		}
	}
	assert(cc==contactPos.size());

	if (f.size()>0){
		int noc=f.size()/3;
		double * ptr=&f(0);
		// draw forces
		vector3N lines;
		double _contactForceVis=0.001;
		for(int i = 0; i < noc; ++i) {
			//contact.set_contact_force(i, ptr+i*3);
			vector3 p=contactPos[i];
			vector3 f(ptr[i*3], ptr[i*3+1], ptr[i*3+2]);
			lines.pushBack(p*100);
			lines.pushBack((p+f*_contactForceVis)*100);
			//cout <<"f" << f <<endl;
		}
		g_debugDraw.registerObject("contactForce222", "LineList", "solidred", matView(lines));
#ifndef NO_OGRE
		g_debugDraw.getCurrRootSceneNode()->setPosition(ToOgre(draw_offset));
#endif
	}
	else
	{
		g_debugDraw.clear();
	}
}


void Trbdl::DynamicsSimulator_Trbdl_LCP::addRelativeConstraint(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2)
{
	/*
	TRL::BodyPtr targetBody = world.body(ichara);
	TRL::Body::LinkConnection linkCon;
	linkCon.link[0]=getTRLlink(targetBody, ((VRMLTransform&)bone1).lastHRPjointIndex());
	linkCon.link[1]=getTRLlink(targetBody, ((VRMLTransform&)bone2).lastHRPjointIndex());
	linkCon.point[0]=boneVector1;
	linkCon.point[1]=boneVector2;
	linkCon.numConstraintAxes=3;
	linkCon.constraintAxes[0]=vector3(1,0,0);
	linkCon.constraintAxes[1]=vector3(0,1,0);
	linkCon.constraintAxes[2]=vector3(0,0,1);
    targetBody->linkConnections.push_back(linkCon);
	*/
}

void Trbdl::DynamicsSimulator_Trbdl_LCP::removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2)
{

	/*
	TRL::BodyPtr targetBody = world.body(ichara);
	TRL::Link* targetLink1 = getTRLlink(targetBody,((VRMLTransform&)bone1).lastHRPjointIndex());
	TRL::Link* targetLink2 = getTRLlink(targetBody,((VRMLTransform&)bone2).lastHRPjointIndex());
	TRL::Body::LinkConnectionArray& targetBodyLinks =  targetBody->linkConnections;

	std::vector<TRL::Body::LinkConnection>::iterator it;

	int i = 0;

	for (it = targetBodyLinks.begin(); it != targetBodyLinks.end();){
		if(it->link[0]->index == targetLink1->index || it->link[0]->index == targetLink2->index){
			if(it->link[1]->index == targetLink1->index || it->link[1]->index == targetLink2->index){
				it = targetBodyLinks.erase(it);
			}
			else{
				it++;
			}
		}
		else{
			it++;
		}
	}
	*/
}


const matrixn& Trbdl::DynamicsSimulator_Trbdl_LCP::getMLCP() const
{
	return _contactForceSolver->Mlcp;
}
const vectorn& Trbdl::DynamicsSimulator_Trbdl_LCP::getMLCP_B() const
{
	return _contactForceSolver->B;
}
const vectorn& Trbdl::DynamicsSimulator_Trbdl_LCP::getMLCP_X() const
{
	return _contactForceSolver->solution;
}
