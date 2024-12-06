/* implemented by Taesoo Kwon
 */
#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/optimize.h"
#include "../math/Operator_NR.h"
//#include "../MainLib/gsl_addon/gsl_wrap.h"
#include "FullbodyIK_MotionDOF.h"
#include "Motion.h"
#include "MotionUtil.h"
#include "MotionLoader.h"
#include "MotionDOF.h"
#include "IKSolver.h"
#include "LimbIKinfo.h"
#include "intersectionTest.h"
#include "../BaseLib/math/Operator.h"
//#define DEBUG_DRAW
#define OBJECTIVE_FUNCTION 4   // 1 or 2 or 3 or 4
//#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../../MainLib/OgreFltk/objectList.h"
#endif

using namespace std;
namespace MotionUtil{
	void setKneeDampingCoef_RO(double ro);
	double getKneeDampingCoef_RO();
}
using namespace MotionUtil;

#include "IK_sdls/Node.h"
#include "IK_sdls/Tree.h"
#include "IK_sdls/Jacobian.h"
#include "IK_sdls/NodeWrap.h"
#include "IK_lbfgs/lbfgs.h"

class MomentumConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		vectorn Hcom;
		vectorn deltaS; // invI*Hcom 

		// when deltaS=desiredCOM-COM,
		// gradient=- 2.0*deltaS* Jcom
		//
		// when deltaS = world velocity,
		// gradient= -2.0*deltaS *Jrot (선형 근사값)
		//
		// -> 이유는 등속도 운동 가정후 1초후의 위치에 대한 미분은 속도라서.
		//
		// 마찬가지로.
		// when deltaS= invI*Hcom ==V,
		// gradient =  -2.0*deltaS* invI*Jmomentum
		//
		matrixn origJT;
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		matrixn invI;
		double weight;
		vectorn desired_vel; // desired generalized velocity
		BoneForwardKinematics orig_chain; // for calculating J and momentumCOMtoPose (world velocity). solver has the current pose.
		MomentumConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s):orig_chain((MotionLoader*)&l), loader(l),solver(s){
			deltaS.setSize(6); deltaS.setAllValue(0.0);
			Hcom.setSize(6);
			desired_vel.setSize(6);
			orig_chain.init();
		}

		inline int getJx(const double *x, vectorn& Jx)
		{

			matrixn JT;
			solver->calcMomentumJacobianTranspose(loader, JT);
			int n=JT.rows();
			vectornView _x((double*)x, JT.rows(), 1);
			Jx.setSize(6);
			Jx.column().multAtB(JT, _x.column());
			return n;
		}
		inline int getJx(vectorn const& _x, vectorn& Jx)
		{
			matrixn JT;
			solver->calcMomentumJacobianTranspose(loader, JT);
			int n=JT.rows();
			Jx.setSize(6);
			Jx.column().multAtB(JT, _x.column());
			return n;
		}
		virtual void initializeConstraint(vectorn const& posedof, double *x)
		{
			orig_chain.setPoseDOF(posedof);
			vectorn I;
			double m=solver->calcInertia(loader, I);
			vector3 r(I(7), I(8), I(9));
			matrixn I6;
			I6.setValues(6,6, 
					I(0),I(3), I(4), 0.0, -r.z, r.y,
					I(3),I(1), I(5), r.z, 0.0,  -r.x,
					I(4),I(5), I(2), -r.y, r.x, 0.0 ,
					0.0, r.z, -r.y, m,   0.0 , 0.0,
					-r.z,0.0,  r.x, 0.0,  m,  0.0,
					r.y, -r.x, 0.0, 0.0,  0.0,  m);

			m::LUinvert(invI,I6);

			//cout<<"I6"<<I6<<endl;
			solver->calcMomentumJacobianTranspose(loader, origJT);
			int n=origJT.rows();
			//vectornView _x((double*)x, n, 1);
			//orig_x=_x;
		}
		vectorn const* updateDeltaS(const double *x) { 
			// momentum mapping
			// Hcom = J*(x- x_original)
			//int n=orig_x.size();
			//matrixn jt;
			//solver->calcMomentumJacobianTranspose(loader,jt);
			////Hcom.column().multAtB(origJT, (vectornView((double*)x, n,1)-orig_x).column());
			//Hcom.column().multAtB(jt, (vectornView((double*)x, n,1)-orig_x).column());


			Liegroup::dse3 hcom=solver->calcMomentumCOMtoPose(loader, 1.0, orig_chain); 
			//Liegroup::dse3 hcom2=solver->calcMomentumCOMfromPose(loader, 1.0, orig_chain); 
			//hcom=hcom*0.5-hcom2*0.5;


			memcpy(&Hcom(0), &hcom._m[0], sizeof(double)*6);

			//cout << "hcom"<< Hcom <<","<<hcom<<endl;

#if 1
			// deltaS = invI*J*(x-x_original)
			deltaS.column().mult(invI, Hcom.column());
			deltaS+=desired_vel;
#else
			deltaS.column().add(Hcom.column(),desired_H.column());
#endif
			//cout <<"invI"<<invI<<endl;
			//cout <<"deltaS"<<deltaS<<endl;
#ifdef DEBUG_DRAW
			static ObjectList o;
			o.drawSphere(deltaS.toVector3(3)*100, "deltaS", "red");
#endif
			return &deltaS;
		}
		vectorn const* getLastDeltaS() { return &deltaS;}
		void getJacobianTranspose(matrixn& JT) {
			// invI*j*dq 
			// J = invI*j
			// JT = jt*invI'
			// solver->calcMomentumJacobianTranspose(loader,JT);
			
			matrixn jt;
			solver->calcMomentumJacobianTranspose(loader,jt);
#if 1
			JT.multABt(jt, invI);
#else
			JT=jt;
#endif
		}


		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double weight){
			double fx;
			matrixn JT;
			vectorn const* deltaS=updateDeltaS(x);
			double len=deltaS->length();
			fx=(len*len)*weight;

			getJacobianTranspose(JT);
			ASSERT(N==JT.rows());
			double w=weight;
			for(int i=0; i<N; i++)
				g[i]-= (2.0*w)*((*deltaS)%JT.row(i));
			return fx;
		}
};
class PoseConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		vectorn _x;
		int _startC;
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;

		PoseConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, vectorn const& x, int startC):loader(l),solver(s),_startC(startC), _x(x) {
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			double fx=0.0;
			// damping terms
			for (int ii=0; ii<_x.size(); ii++)
			{
				int i=ii+_startC;
				fx+=w*SQR(x[i]-_x[ii]);
				// matrixCalculus.pdf : proposition 14
				// fx+=(x-x0)'W(x-x0)
				// g+=2(x-x0)'W
				g[i]+=w*2.0*(x[i]-_x[ii]);
			}
			return fx;
		}

};
class COMConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;

		vector3 globalCOM;
		vector3 _w;
		COMConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, vector3 const& com, double wx=1.0, double wy=1.0, double wz=1.0):loader(l),solver(s) {
			globalCOM=com;
			_w.x=wx;
			_w.y=wy;
			_w.z=wz;
		}
		inline double wdot(vector3 const& a, vector3 const& b)
		{
			return a.x*b.x*_w.x+a.y*b.y*_w.y+a.z*b.z*_w.z;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			matrixn JT;
			double fx=0.0;

			vector3 deltaS;
			deltaS.sub(globalCOM, solver->calcCOM(loader));
			fx+=wdot(deltaS, deltaS)*w;
			solver->calcCOMjacobianTranspose(loader, JT);

			ASSERT(N==JT.rows());
			for(int i=0; i<N; i++)
				g[i]-= (2.0*w)*wdot(deltaS,JT.row(i).toVector3(0));
			return fx;
		}
};
class ROTConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;

		quater q2; // desiredOri
		ROTConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, quater const& q):loader(l),solver(s) {
			q2=q;
			bone1=b;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			matrixn JT;
			double fx=0.0;
			const quater& q1=solver->getLastNode(bone1->treeIndex())->globalFrame().rotation;
			matrix3 m1,m2, dotR;
			m1.setRotation(q1);
			m2.setRotation(q2);
			dotR=m2-m1;
			m1.transpose(); // invert m1

			vector3 deltaS;
			// body velocity: unskew(invR_ab*dR_ab ) (eqn 2.48)
			// world velocity: unskew(dR_ab*invR_ab)
			deltaS=(dotR*m1).unskew(); // world velocity
			fx+=deltaS.squaredLength()*w;

			solver->calcRotJacobianTranspose(JT, bone1->treeIndex());

			ASSERT(N==JT.rows());
			for(int i=0; i<N; i++)
				g[i]-= (2.0*w)*deltaS%JT.row(i).toVector3(0);
			return fx;
		}
};
//#define USE_SIMPLE_BUT_SLOW_CODE
class SkinConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _gpos2;
		vector3 _gpos;

		intvectorn const&_treeIndex;
		vector3N const& _localpos;
		vectorn  const&_weights;

		SkinConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, 
				intvectorn const& treeIndices, vector3N const& localpos, vectorn  const&weights, vector3 const& desired_pos) 
			:loader(l),solver(s),
			_treeIndex(treeIndices),
			_localpos(localpos),
			_weights(weights)
		
		{
			_gpos=desired_pos;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
#ifdef USE_SIMPLE_BUT_SLOW_CODE
			matrixn JT;
#endif
			double fx=0.0;
			vector3 deltaS;

			_gpos2.zero();

			// for each marker i
			//	 	g_i=sum_j w_ij*(R_j*l_ij+p_j)
			//
			//	 	(
			//	 		j는 마커와 관련된 모든 본, 
			//	 		(R_j,p_j)는 본j의 globalFrame
			//	 		w_ij, l_ij 는 스키닝 파라미터
			//	 	)
			// 		for each bone j
			// 			grad-=(2.0*w_ij)*(g'_i - g_i)'*J_j
			//
			for(int j=0; j<_weights.size(); j++)
			{
				const transf& tf=solver->getLastNode(_treeIndex(j))->globalFrame();
				_gpos2+=(tf*_localpos(j))*_weights(j);
			}

			deltaS.sub(_gpos, _gpos2);
			fx+=deltaS% deltaS*w;

			for(int j=0; j<_weights.size(); j++)
			{
#ifdef USE_SIMPLE_BUT_SLOW_CODE
				solver->calcJacobianTransposeAt(JT, _treeIndex(j), _localpos(j));

				ASSERT(N==JT.rows());
				for(int i=0; i<N; i++)
					g[i]-= (2.0*w*_weights(j))*deltaS%(JT.row(i).toVector3(0));
#else
				solver->updateGrad_S_JT(g, -(2.0*w*_weights(j))*deltaS, _treeIndex(j), _localpos(j));
#endif
			}

			return fx;
		}
};
class FastSkinConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		MotionLoader const& loader;
		IK_sdls::LoaderToTree* solver;
		vector3 _gpos2;

		struct MarkerInfo {
			MarkerInfo(){_pTreeIndex=NULL; _pLocalpos=NULL; _pArrayWeights=NULL;}
			const intvectorn* _pTreeIndex;
			const vector3N* _pLocalpos;
			const vectorn* _pArrayWeights;
			vector3 desired_position;
		};
		std::vector<MarkerInfo > _markerInfo;
		vector3N _all_dS;
		vector3N _all_dS_lpos;

		FastSkinConstraintInfo(MotionLoader const& l, IK_sdls::LoaderToTree* s, int numMarkers) 
			:loader(l),solver(s)
		{
			_markerInfo.resize(numMarkers);
			_all_dS.setSize(l.numBone());
			_all_dS_lpos.setSize(l.numBone());
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			double fx=0.0;

			_all_dS.range(1, _all_dS.size()).setAllValue(vector3(0,0,0));
			_all_dS_lpos.range(1, _all_dS_lpos.size()).setAllValue(vector3(0,0,0));
			for(int imarker=0; imarker<_markerInfo.size(); imarker++){
				MarkerInfo& mi=_markerInfo[imarker];
				ASSERT(mi._pTreeIndex);
				const intvectorn& _treeIndex=*mi._pTreeIndex;
				const vector3N& _localpos=*mi._pLocalpos;
				const vectorn& _weights=*mi._pArrayWeights;
				_gpos2.zero();

				// for each marker i
				//	 	g_i=sum_j w_ij*(R_j*l_ij+p_j)
				//
				//	 	(
				//	 		j는 마커와 관련된 모든 본, 
				//	 		(R_j,p_j)는 본j의 globalFrame
				//	 		w_ij, l_ij 는 스키닝 파라미터
				//	 	)
				// 		for each bone j
				// 			grad-=(2.0*w_ij)*(g'_i - g_i)'*J_j
				//
#ifdef _WIN32
				IK_sdls::Node* lastNode[10];
				ASSERT(_weights.size() < 10);
#else
				IK_sdls::Node* lastNode[_weights.size()];
#endif
				vector3 target;

				for(int j=0; j<_weights.size(); j++)
				{
					lastNode[j]=solver->getLastNode(_treeIndex(j));
					const transf& tf=lastNode[j]->globalFrame();
					target=tf*_localpos(j);
					_gpos2+=target*_weights(j);
				}

				vector3 deltaS;
				deltaS.sub(mi.desired_position, _gpos2);
				fx+=deltaS% deltaS*w;

				int nJoint = solver->mTree.GetNumJoint();
				// compute jacobian
				IK_sdls::Node* m;

				for(int j=0; j<_weights.size(); j++)
				{
					vector3 dS=-(2.0*w*_weights(j))*deltaS;
					_all_dS(_treeIndex(j))+=dS;

					// target = p + R*lpos
					// lin= cross(GetW(), target-GetS()) (hinge)
					//    = hat(GetW())*(target-GetS())
					// then, its gradient contribution becomes: deltaS%lin
					//
					// deltaS%lin (scalar) = deltaS*hatW*(p-GetS()) + deltaS*hatW*(R*lpos)
					// 			           = deltaS*hatW*(p-GetS()) + deltaS*hat(R*lpos) * (-w)
					// 
					//
					// j 두개만 가지고 식 세워보면...
					// deltaS1%lin1+deltaS2%lin2 
					// 	 =    deltaS1*hatW*(p+R*l1-GetS()) + deltaS2*hatW*(p+R*l2-GetS())
					// 	 =    (deltaS1+deltaS2)*hatW*(p-GetS())
					// 	 	+ (deltaS1*hat(R*l1)+deltaS2*hat(R*l2))*(-w)
					//
					
					const quater& R=lastNode[j]->globalFrame().rotation;
					// deltaS*hat(R*lj)== deltaS.cross(R*lj)
					_all_dS_lpos(_treeIndex(j))+=dS.cross(R*_localpos(j));

					//solver->updateGrad_S_JT(g, dS, _treeIndex(j), _localpos(j)); // 이 줄 대신 아래 두줄(1, 2)을 써도 같은 뜻임.
					//solver->updateGrad_S_JT(g, dS, _treeIndex(j), vector3(0,0,0)); // (1).      
					//solver->updateGrad_S_JT_residual(g, dS.cross(R*_localpos(j)), _treeIndex(j)); // (2).         
					// 그리고 (1,2) 대신 아래 (3,4) 쓰면 동일한 결과, 훨씬 빠른 속도 (nested for-loop 개수가 하나 줄음).


#ifdef VERIFY_ZERO_DIFF
					int nJoint = solver->mTree.GetNumJoint();
					// compute jacobian
					IK_sdls::Node* m;

					int ibone=_treeIndex(j);
					m = solver->getLastNode(ibone);
					vector3 targetg=m->globalFrame()*_localpos(j);
					vector3 target=m->globalFrame().translation;
					while ( m ) {
						int jj = m->GetJointNum();
						if ( !m->IsFrozen() ) {

							double backup=g[jj];
							m->_updateGrad_S_JT(g, dS, targetg);

							double res1=g[jj]-backup;

							g[jj]=backup;
							m->_updateGrad_S_JT(g, dS, target);
							m->_updateGrad_S_JT_residual(g, dS.cross(R*_localpos(j)));

							double res2=g[jj]-backup;

							if(!isSimilar(res1,res2))
							{
								printf("error!!! %d: %f %f", jj, res1, res2);
								exit(0);
							}
						}
						m = m->realparent;
					}
#endif


				}
			}

			for(int ibone=1, nb=loader.numBone(); ibone<nb; ibone++)
			{
				solver->updateGrad_S_JT(g, _all_dS(ibone), ibone, vector3(0,0,0)); //(3)
				solver->updateGrad_S_JT_residual(g, _all_dS_lpos(ibone), ibone); //(4)
			}
			return fx;
		}
};


class EEConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _lpos;
		vector3 _gpos;
		vector3 _w;

		EEConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos, vector3 const& gpos, double wx, double wy, double wz):loader(l),solver(s) {
			bone1=b;
			_lpos=lpos;
			_gpos=gpos;
			_w.x=wx;
			_w.y=wy;
			_w.z=wz;
		}
		inline double wdot(vector3 const& a, vector3 const& b)
		{
			return a.x*b.x*_w.x+a.y*b.y*_w.y+a.z*b.z*_w.z;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			matrixn JT;
			double fx=0.0;
			const transf& tf=solver->getLastNode(bone1->treeIndex())->globalFrame();
			vector3 deltaS;
			deltaS.sub(_gpos, tf*_lpos);
			fx+=wdot(deltaS, deltaS)*w;
			solver->calcJacobianTransposeAt(JT, bone1->treeIndex(), _lpos);

			ASSERT(N==JT.rows());
			for(int i=0; i<N; i++)
				g[i]-= (2.0*w)*wdot(deltaS,JT.row(i).toVector3(0));
			return fx;
		}
};
class DistanceConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _lpos;
		vector3 _gpos;
		double _targetDist;

		DistanceConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos, vector3 const& gpos, double targetDist):loader(l),solver(s) {
			bone1=b;
			_lpos=lpos;
			_gpos=gpos;
			_targetDist=targetDist;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			matrixn JT;
			double fx=0.0;
			const transf& tf=solver->getLastNode(bone1->treeIndex())->globalFrame();
			vector3 deltaS;
			deltaS.sub(_gpos, tf*_lpos);
			double sqrD=_targetDist*_targetDist;

			double v=deltaS%deltaS-sqrD;

			ASSERT(N==JT.rows());
			if(v>0)
			{
				//fx+=v*v*w;
				double d=deltaS.length()-_targetDist;
				fx+=w*SQR(d);
				solver->calcJacobianTransposeAt(JT, bone1->treeIndex(), _lpos);
				for(int i=0; i<N; i++) g[i]-= (2.0*w)*d*(deltaS.dir()%JT.row(i).toVector3(0));
			}
			return fx;
		}
};
class RelativeConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _lpos1;
		Bone* bone2;
		vector3 _lpos2;
		vector3 _gdelta;

		RelativeConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos1, 
				Bone* b2, vector3 const& lpos2) 
			:loader(l),solver(s),_gdelta(0,0,0)
		{

			bone1=b;
			_lpos1=lpos1;
			bone2=b2;
			_lpos2=lpos2;
		}
		RelativeConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos1, 
				Bone* b2, vector3 const& lpos2,vector3 const& delta) 
			:loader(l),solver(s)
		{

			_gdelta=delta;
			bone1=b;
			_lpos1=lpos1;
			bone2=b2;
			_lpos2=lpos2;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			double fx=0.0;

			vector3 deltaS;

			int ti1=bone1->treeIndex();
			int ti2=bone2->treeIndex();
			const transf& tf=solver->getLastNode(ti1)->globalFrame();
			const transf& tf2=solver->getLastNode(ti2)->globalFrame();

			deltaS.sub(tf2*_lpos2, tf*_lpos1);
			deltaS-=_gdelta;
			fx+=deltaS% deltaS*w;

			solver->updateGrad_S_JT(g, -(2.0*w*1)*deltaS, ti1, _lpos1);
			solver->updateGrad_S_JT(g, -(2.0*w*-1)*deltaS, ti2, _lpos2);
			return fx;
		}
};
class RelativeDistanceConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _lpos1;
		Bone* bone2;
		vector3 _lpos2;
		double _targetDist;
		vector3 _gdelta;

		RelativeDistanceConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos1, 
Bone* b2, vector3 const& lpos2, double thr) 
			:loader(l),solver(s), _targetDist(thr),_gdelta(0,0,0)
		{

			bone1=b;
			_lpos1=lpos1;
			bone2=b2;
			_lpos2=lpos2;
		}
		RelativeDistanceConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos1, 
Bone* b2, vector3 const& lpos2, vector3 const& delta, double thr) 
			:loader(l),solver(s), _targetDist(thr)
		{
			_gdelta=delta;
			bone1=b;
			_lpos1=lpos1;
			bone2=b2;
			_lpos2=lpos2;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w){
			double fx=0.0;

			vector3 deltaS;

			int ti1=bone1->treeIndex();
			int ti2=bone2->treeIndex();
			const transf& tf=solver->getLastNode(ti1)->globalFrame();
			const transf& tf2=solver->getLastNode(ti2)->globalFrame();

			deltaS.sub(tf2*_lpos2, tf*_lpos1);
			deltaS-=_gdelta;
			double v=deltaS% deltaS;
			if (v>_targetDist*_targetDist)
			{
				double d=sqrt(v)-_targetDist;
				fx+=w*SQR(d);

				solver->updateGrad_S_JT(g, -(2.0*w*d)*deltaS.dir(), ti1, _lpos1);
				solver->updateGrad_S_JT(g, -(2.0*w*-d)*deltaS.dir(), ti2, _lpos2);
			}
			return fx;
		}
};
class RelativeHSConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		IK_sdls::LoaderToTree* solver;
		Bone* bone1;
		vector3 _lpos1;
		Bone* bone2;
		vector3 _lpos2;
		vector3 _n;
		double _idepth;

		RelativeHSConstraintInfo(VRMLloader const& l, IK_sdls::LoaderToTree* s, Bone* b, vector3 const& lpos1, 
Bone* b2, vector3 const& lpos2, vector3 const& normal, double depth) 
			:loader(l),solver(s),_n(normal), _idepth(depth)
		{

			bone1=b;
			_lpos1=lpos1;
			bone2=b2;
			_lpos2=lpos2;
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w);
};
class FullbodyIK_MotionDOF_MultiTarget_lbfgs;
class EEYConstraintInfo: public MotionUtil::RelativeConstraint::ConstraintInfo
{
	public:
		VRMLloader const& loader;
		FullbodyIK_MotionDOF_MultiTarget_lbfgs* solver;
		vectorn mEffectorWeights;

		EEYConstraintInfo(VRMLloader const& l, FullbodyIK_MotionDOF_MultiTarget_lbfgs* s, vectorn const& ew):loader(l),solver(s),mEffectorWeights(ew){
		}
		virtual double calcObjectiveAndGradient(int N, double* g, const double *x, double w);
};




class FullbodyIK_MotionDOF_MultiTarget_lbfgs: public FullbodyIK_MotionDOF, public IK_sdls::LoaderToTree

{
	friend class EEYConstraintInfo;
	std::vector<MotionUtil::Effector> mEffectors;
	std::vector<MotionUtil::RelativeConstraint> mConstraints;
	MotionDOFinfo const& mDofInfo;
	MotionLoader const& mSkeleton;

	
	LBFGS::lbfgsfloatval_t *m_x0;
	vectorn mEffectorWeights;
	double _w_root1, _w_root2, _w_other;
	double _w_slide_p, _w_slide_n;
	// workspace
	VectorRn dS;			// delta s
	MatrixRmn Jend;		// Jacobian matrix based on end effector positions
	// workspace end
public:
	boolN mJacobianLock;

	void setDampingWeight(double w_root, double w_other)
	{
		_w_root1=w_root;
		_w_root2=w_root;
		_w_other=w_other;
	}
	void setRootDampingWeight(double w_root1, double w_root2)
	{
		_w_root1=w_root1;
		_w_root2=w_root2;
	}
	void setSlideDampingWeight(double w_positive, double w_negative)
	{
		_w_slide_p=w_positive;
		_w_slide_n=w_negative;
	}

	virtual void setParam(const char* type, const vectorn& input) {
		if(TString("jacobian_lock")==type)
		{
			int numbone=numBone();
			mJacobianLock.resize(nDOF());
			mJacobianLock.clearAll();
			for(int i=0; i<input.size(); i++)
			{
				int ti=input[i];
				Msg::verify(ti>=1 && ti<numbone, "setparam???");

				int sj=getVarIndex(ti);
				int ej=getVarEndIndex(ti);
				mJacobianLock.range(sj,ej).setAllValue(true);
			}
		}
		else
			Msg::print("unknown param %s", type);

	}
	virtual void setParam(const char* type, double value, double value2)
	{
		if(TString("damping_weight")==type)
			setDampingWeight(value, value2);
		else if(TString("root_damping_weight")==type)
			setRootDampingWeight(value, value2);
		else if(TString("slide_damping_weight")==type)
			setDampingWeight(value, value2);
		else
			Msg::print("unknown param %s", type);
	}
	virtual bool _changeNumEffectors(int n) { 
		mEffectors.resize(n);
		mEffectorWeights.resize(n);
		mEffectorWeights.setAllValue(1.0);
		return true;
	}
	virtual bool _changeNumConstraints(int n) { 
		mConstraints.resize(n);
		return true;
	}
	virtual int _numConstraints() const {
		return mConstraints.size();
	}
	virtual bool _setEffector(int i, Bone* bone, vector3 const& lpos) { 
		mEffectors[i].bone=bone;
		mEffectors[i].localpos=lpos;
		return true;
	}
	// actually generalized velocity constraint so that the output is mass/inertia independent.
	virtual bool _setMomentumConstraint(int i, vector3 const& ang, vector3 const& lin, double weight){
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		MomentumConstraintInfo* pInfo=new MomentumConstraintInfo((VRMLloader const&)mSkeleton,this);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::MOMENTUM, pInfo);
		pInfo->desired_vel.setVec3(0, ang);
		pInfo->desired_vel.setVec3(3, lin);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setEffectorYConstraint(int i, double weight, const vectorn& effectorWeights)
	{
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		EEYConstraintInfo* pInfo=new EEYConstraintInfo((VRMLloader const&)mSkeleton,this, effectorWeights);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setPoseConstraint(int i, vectorn const& pose, double weight, int startBoneIndex=1, int endBoneIndex=INT_MAX)
	{
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		setPoseDOF(mTree, mDofInfo, pose, mNode);
		mTree.Compute();
#ifdef _DEBUG
		for(int i=0; i<mNode.size(); i++)
		{
			Bone* bone=mNode[i].bone;
			int treeIndex=bone->GetIndex();
			assert(treeIndex==i+1);
		}
#endif
		if(endBoneIndex>mDofInfo.numBone())
			endBoneIndex=mDofInfo.numBone();
		int c=0;
		int startC, endC;
		for(int i=0; i<mNode.size(); i++)
		{
			IK_sdls::NodeWrap& n=mNode[i];
			if (i==startBoneIndex-1) startC=c;
			c+=n.axes.length();
			if (i==endBoneIndex-1-1) endC=c;
		}

		int N=c;
		vectorn x(N);
		getTheta(&x[0]);

		PoseConstraintInfo* pInfo=new PoseConstraintInfo((VRMLloader const&)mSkeleton,this, x.range(startC, endC), startC );
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::POSE, pInfo);
		mConstraints[i].weight=weight; // by default.
		return true;
	}
	virtual bool _setEffectorWeight(int i, double w=1.0) { 
		mEffectorWeights(i)=w;
		return true;
	}
	virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori, double weight) { 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		ROTConstraintInfo* pInfo=new ROTConstraintInfo((VRMLloader const&)mSkeleton,this, bone, desired_ori);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::ROT, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setPositionConstraint(int i, Bone* bone, vector3 const&lpos, vector3 const& desired_pos, double wx, double wy, double wz) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		EEConstraintInfo* pInfo=new EEConstraintInfo((VRMLloader const&)mSkeleton,this, bone, lpos, desired_pos, wx, wy, wz);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}

	virtual bool _setSkinningConstraint(int i, intvectorn const& treeIndices, vector3N const& localpos, vectorn  const&weights, vector3 const& desired_pos) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		SkinConstraintInfo* pInfo=new SkinConstraintInfo((VRMLloader const&)mSkeleton,this, treeIndices, localpos, weights, desired_pos);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}

	virtual bool _setConstraintWeight(int i, double w){ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		mConstraints[i].weight=w;
		return true;
	}
#if 1
	// testing a faster version of the above function
	virtual bool _setFastSkinningConstraint(int i, int numMarkers){
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		FastSkinConstraintInfo* pInfo=new FastSkinConstraintInfo(mSkeleton,this, numMarkers);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}
	virtual void _setFastSkinningConstraintParam(int imarker, intvectorn const& treeIndices, vector3N const& localpos, vectorn  const&weights, vector3 const& desired_pos)
	{
		FastSkinConstraintInfo* pInfo=dynamic_cast<FastSkinConstraintInfo*> (mConstraints.back().pInfo);
		FastSkinConstraintInfo::MarkerInfo& mi=pInfo->_markerInfo[imarker];
		mi._pTreeIndex=&treeIndices;
		mi._pLocalpos=&localpos;
		mi._pArrayWeights=&weights;
		mi.desired_position=desired_pos;
	}
#endif
	virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori) { 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		ROTConstraintInfo* pInfo=new ROTConstraintInfo((VRMLloader const&)mSkeleton,this, bone, desired_ori);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::ROT, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}
	virtual bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, double weight) 
	{ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		RelativeConstraintInfo* pInfo=new RelativeConstraintInfo((VRMLloader const&)mSkeleton,this, bone1, lpos1, bone2, lpos2);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& gdelta, double weight) 
	{ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		RelativeConstraintInfo* pInfo=new RelativeConstraintInfo((VRMLloader const&)mSkeleton,this, bone1, lpos1, bone2, lpos2, gdelta);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setRelativeDistanceConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, double thr, double weight) 
	{ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		RelativeDistanceConstraintInfo* pInfo=new RelativeDistanceConstraintInfo((VRMLloader const&)mSkeleton,this, bone1, lpos1, bone2, lpos2, thr);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setRelativeDistanceConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& delta, double thr, double weight) 
	{ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		RelativeDistanceConstraintInfo* pInfo=new RelativeDistanceConstraintInfo((VRMLloader const&)mSkeleton,this, bone1, lpos1, bone2, lpos2, delta, thr);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setRelativeHalfSpaceConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& global_normal, float idepth, double weight) 
	{ 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		RelativeHSConstraintInfo* pInfo=new RelativeHSConstraintInfo((VRMLloader const&)mSkeleton,this, bone1, lpos1, bone2, lpos2,global_normal, idepth);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=weight;
		return true;
	}


	//HERE
	virtual bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2) { 
		return _setRelativeConstraint(i, bone1, lpos1, bone2, vector3(0,0,0), 1);
	}
	virtual bool _setPlaneDistanceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		mConstraints[i].eType=MotionUtil::RelativeConstraint::PLANE_DISTANCE;
		mConstraints[i].bone1=bone;
		mConstraints[i].localpos1=lpos;
		mConstraints[i].normal[0]=global_normal.x;
		mConstraints[i].normal[1]=global_normal.y;
		mConstraints[i].normal[2]=global_normal.z;
		mConstraints[i].idepth=idepth;
		return true;
	}	   
	virtual bool _setDistanceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& gpos, float targetDist) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		auto* pInfo=new DistanceConstraintInfo((VRMLloader const&)mSkeleton,this, bone,lpos, gpos, targetDist );
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::OTHERS, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}
	
	virtual bool _setHalfSpaceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		mConstraints[i].eType=MotionUtil::RelativeConstraint::HALF_SPACE;
		mConstraints[i].bone1=bone;
		mConstraints[i].localpos1=lpos;
		mConstraints[i].normal[0]=global_normal.x;
		mConstraints[i].normal[1]=global_normal.y;
		mConstraints[i].normal[2]=global_normal.z;
		mConstraints[i].idepth=idepth;
		mConstraints[i].weight=1.0;
		return true;
	}	   
	virtual bool _setCOMConstraint(int i, vector3 const& com) { 
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		COMConstraintInfo* pInfo=new COMConstraintInfo((VRMLloader const&)mSkeleton,this, com);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::COM, pInfo);
		mConstraints[i].weight=1.0;
		return true;
	}	   
	virtual bool _setCOMConstraint(int i, vector3 const& com, double wx, double wy, double wz) {
		Msg::verify(mConstraints.size()>i, "_changeNumConstraints first!!!");
		COMConstraintInfo* pInfo=new COMConstraintInfo((VRMLloader const&)mSkeleton,this, com, wx, wy, wz);
		mConstraints[i].setInfo(MotionUtil::RelativeConstraint::COM, pInfo);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=1.0;
		return true;
	}	   

	virtual bool _effectorUpdated()
	{
		mTree.RemoveAllEffectors();
		_init_part2(mSkeleton, mEffectors, mConstraints);
		return true;
	}

	virtual~FullbodyIK_MotionDOF_MultiTarget_lbfgs(){ 
	}

	FullbodyIK_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& dofInfo, std::vector<MotionUtil::Effector>& effectors, std::vector<RelativeConstraint>& constraints)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),mEffectors(effectors),mConstraints(constraints),
		 IK_sdls::LoaderToTree(dofInfo.skeleton(), effectors,constraints, true,false)
	{
		m_x0=NULL;
		int n=effectors.size();
		mEffectorWeights.resize(n);
		mEffectorWeights.setAllValue(1.0);
		_w_root1=0.01;
		_w_root2=0.01;
		_w_other=0.01;
		_w_slide_p=0.1;
		_w_slide_n=0.1;
	}
	FullbodyIK_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& dofInfo)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),
		 IK_sdls::LoaderToTree(dofInfo.skeleton(), true,false)
	{
		m_x0=NULL;
		_w_root1=0.01;
		_w_root2=0.01;
		_w_other=0.01;
		_w_slide_p=0.1;
		_w_slide_n=0.1;
	}

	virtual bool _updateBoneLength(MotionLoader const& loader){
		updateBoneLength(loader);
		return true;
	}

	//FullbodyIK_MotionDOF_MultiTarget_lbfgs::
	virtual void IKsolve(vectorn const& pose, vectorn& output, vector3N const& con2)
	{
		

		vector3* con;
		int con_size;
		{
			con_size=con2.size();
			if (con_size==0)
				con=NULL;
			else
				con=&con2[0];
		}

		ASSERT(mSkeleton.numTransJoint()==1);
		// set tree. (position, orientation) : quaternion->euler

		setPoseDOF(mTree, mDofInfo, pose, mNode);
		mTree.Compute();

		// set target.
		mTree.target=con;
		int N=mTree.GetNumJoint();
		LBFGS::lbfgsfloatval_t fx;
		LBFGS::lbfgsfloatval_t *m_x;
        m_x = LBFGS::lbfgs_malloc(N);
		m_x0= LBFGS::lbfgs_malloc(N);

        if (!m_x || !m_x0) {
            printf("ERROR: Failed to allocate a memory block for variables.\n");
            return ;
        }

        /* Initialize the variables. */
		getTheta(m_x);
		getTheta(m_x0);

		/* Initialize constraints */
		for(int ii=0; ii<mConstraints.size(); ii++)
		{
			MotionUtil::RelativeConstraint* n=&mConstraints[ii];
			if(n->eType>=MotionUtil::RelativeConstraint::MOMENTUM)
				n->pInfo->initializeConstraint(pose, m_x);
		}
        /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
         */

		LBFGS::lbfgs_parameter_t param;
		LBFGS::lbfgs_parameter_init(&param);
		param.linesearch = LBFGS::LBFGS_LINESEARCH_BACKTRACKING;
		param.epsilon=1e-9;

        int ret = LBFGS::lbfgs(N, m_x, &fx, _evaluate, _progress, this, &param);

#ifdef _DEBUG
        /* Report the result. */
        printf("L-BFGS optimization terminated with status code = %d\n", ret);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, m_x[0], m_x[1]);
#endif

		setTheta(m_x);
		getPoseDOF(mDofInfo, output, mNode);
		LBFGS::lbfgs_free(m_x);
		LBFGS::lbfgs_free(m_x0);
	}
    static LBFGS::lbfgsfloatval_t _evaluate(
        void *instance,
        const LBFGS::lbfgsfloatval_t *x,
        LBFGS::lbfgsfloatval_t *g,
        const int n,
        const LBFGS::lbfgsfloatval_t step
        )
    {
        return reinterpret_cast<FullbodyIK_MotionDOF_MultiTarget_lbfgs*>(instance)->evaluate(x, g, n, step);
    }

	
	/*  how to compute the gradient
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
	double _calcObjectiveAndGradient(MotionUtil::RelativeConstraint* constraint, int N, double* g, const double *x){
		double fx=0;
		vector3 normal=vector3(constraint->normal[0], 
				constraint->normal[1],
				constraint->normal[2]);
		Plane p(normal, -constraint->idepth);

		int ti=constraint->bone1->treeIndex();
		const transf& tf=getLastNode(ti)->globalFrame();
		auto nS=tf*constraint->localpos1;

		double d=p.distance(nS);
		if (constraint->eType==MotionUtil::RelativeConstraint::PLANE_DISTANCE)
			fx+=d*d;
		else if(d>0) // MotionUtil::RelativeConstraint::HALF_SPACE; see NodeWrap.cpp also.
			fx+=d*d;

		bool hasLock=(mJacobianLock.size()==N);

#ifdef USE_SIMPLE_BUT_SLOW_CODE
		matrixn Jt;
		calcJacobianTransposeAt(Jt, ti, constraint->localpos1);
#endif

		if(constraint->eType==MotionUtil::RelativeConstraint::PLANE_DISTANCE || d>0)
		{
#ifdef USE_SIMPLE_BUT_SLOW_CODE
			for(int i=0; i<N; i++){
				if(hasLock && mJacobianLock[i]) continue;

				vector3 j=Jt.row(i).toVector3();
				// plane distance
				g[i]+=2.0*d*(normal%j);
			}
#else
			updateGrad_S_JT(g, (2.0*d)*normal, ti, constraint->localpos1);
#endif
		}
		return fx;
	}
	LBFGS::lbfgsfloatval_t evaluate(
        const LBFGS::lbfgsfloatval_t *x,
        LBFGS::lbfgsfloatval_t *g,
        const int N,
        const LBFGS::lbfgsfloatval_t step
        )
    {
#if 0 
		// slower version. This does not work when a full-body tree was initialized.
		double fx = computeConstraintError();
		computeConstraintErrorGradient(g);
#else
		// a slightly faster version.
		
		setTheta(x);
		mTree.Compute();
		int nEffector = mTree.GetNumEffector();
		int nJoint = mTree.GetNumJoint();
		int nCon= mConstraints.size();
		int nRow = 3 * nEffector;
		int nCol = nJoint;
		
		// compute jacobian
		IK_sdls::Effector* n;

		dS.SetLength(nRow+3*nCon);

		Jend.SetSize(nRow, nCol);
		Jend.SetZero();

		LBFGS::lbfgsfloatval_t fx = 0.0;
	
		for(int ii=0; ii<mTree.effectors.size(); ii++)
		{
			n = mTree.effectors[ii];
			int i = n->GetEffectorNum();
			assert(i==ii);
			// Compute the delta S value (differences from end effectors to target positions.
			n->computeDeltaS(dS, &mTree);
			fx+=mEffectorWeights(ii)*dS.GetTriple(ii).squaredLength();
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

		bool hasLock=(mJacobianLock.size()==N);
		if (hasLock)
		{
			for(int ti=1; ti<mSkeleton.numBone(); ti++)
			{
				int sj=getVarIndex(ti);
				int ej=getVarEndIndex(ti);
				if(mJacobianLock(sj))
				{
					for(int i=sj; i<ej; i++)
						getNode(ti,i-sj)->Freeze();
				}
				else
				{
					for(int i=sj; i<ej; i++)
						getNode(ti,i-sj)->UnFreeze();
				}
			}
		}
		for(int i=0; i<N; i++){
			if(hasLock && mJacobianLock[i]) continue;

			for(int ii=0; ii<mTree.effectors.size(); ii++)
			{
				n=mTree.effectors[ii];
				VectorR3 j;
				Jend.GetTriple(ii, i, &j);
				// absolute position
				g[i]-= mEffectorWeights(ii)*2.0*dS.GetTriple(ii)%j;
			}	
		}

		for(int ii=0; ii<mConstraints.size(); ii++)
		{
			VRMLloader const& loader=(VRMLloader const&)mSkeleton;
			MotionUtil::RelativeConstraint* n=&mConstraints[ii];

			if (n->eType>=MotionUtil::RelativeConstraint::MOMENTUM)
			{
				double f=n->pInfo->calcObjectiveAndGradient(N, g,x, n->weight);
				fx+=f;
			}
			else
			{
				double f=_calcObjectiveAndGradient(n, N, g,x);
				fx+=f;
			}
		}
#endif

		// damping terms
		for (int i=0; i<N; i++)
		{
			if(hasLock && mJacobianLock[i]) g[i]=0.0;

			double w;
			if (i<3) w=_w_root1;
			else if(i<6) w=_w_root2;
			else if(getJoint(i)->IsSlideJoint())
			{
				if(x[i]<0)
					w=_w_slide_n;
				else
					w=_w_slide_p;
			}
			else
			{
				w=_w_other;
			}
			fx+=w*SQR(x[i]-m_x0[i]);
			// matrixCalculus.pdf : proposition 14
			// fx+=(x-x0)'W(x-x0)
			// g+=2(x-x0)'W
			g[i]+=w*2.0*(x[i]-m_x0[i]);
		}
        return fx;
    }

    static int _progress( void *instance, const LBFGS::lbfgsfloatval_t *x, const LBFGS::lbfgsfloatval_t *g, const LBFGS::lbfgsfloatval_t fx, const LBFGS::lbfgsfloatval_t xnorm, const LBFGS::lbfgsfloatval_t gnorm, const LBFGS::lbfgsfloatval_t step, int n, int k, int ls)
    {
        return reinterpret_cast<FullbodyIK_MotionDOF_MultiTarget_lbfgs*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

    int progress( const LBFGS::lbfgsfloatval_t *x, const LBFGS::lbfgsfloatval_t *g, const LBFGS::lbfgsfloatval_t fx, const LBFGS::lbfgsfloatval_t xnorm, const LBFGS::lbfgsfloatval_t gnorm, const LBFGS::lbfgsfloatval_t step, int n, int k, int ls) 
	{
#ifdef _DEBUG
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
#endif
        return 0;
    }
};

FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info, std::vector<Effector>& effectors, std::vector<RelativeConstraint>& constraints)
{
	return new FullbodyIK_MotionDOF_MultiTarget_lbfgs(info, effectors, constraints);
}
FullbodyIK_MotionDOF* MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& info)
{
	return new FullbodyIK_MotionDOF_MultiTarget_lbfgs(info);
}
double EEYConstraintInfo::calcObjectiveAndGradient(int N, double* g, const double *x, double w)
{
	double fx = 0.0;
	auto& mTree=solver->mTree;
	IK_sdls::Effector* n;
	auto& dS=solver->dS;
	auto& Jend=solver->Jend;
	for(int ii=0; ii<mTree.effectors.size(); ii++)
	{
		n = mTree.effectors[ii];
		int i = n->GetEffectorNum();
		assert(i==ii);
		{
			vector3 ds=dS.GetTriple(ii);
			if(ds.y<0)
			{
				fx+=w*mEffectorWeights(ii)*(ds.x*ds.x+ds.z*ds.z);
				for(int i=0; i<N; i++){
					VectorR3 j;
					Jend.GetTriple(ii, i, &j);
					g[i]-= w*mEffectorWeights(ii)*2.0*(ds.x*j.x+ds.z*j.z);
				}
			}
			else
			{
				fx+=w*mEffectorWeights(ii)*ds.squaredLength();
				for(int i=0; i<N; i++){
					VectorR3 j;
					Jend.GetTriple(ii, i, &j);
					g[i]-= w*mEffectorWeights(ii)*2.0*ds%j;
				}

			}
		}
	}
	return fx;
}
double RelativeHSConstraintInfo::calcObjectiveAndGradient(int N, double* g, const double *x, double w)
{
	double fx=0.0;

	vector3 deltaS;

	int ti1=bone1->treeIndex();
	int ti2=bone2->treeIndex();
	const transf& tf=solver->getLastNode(ti1)->globalFrame();
	const transf& tf2=solver->getLastNode(ti2)->globalFrame();

	deltaS.sub(tf2*_lpos2, tf*_lpos1);
	double f=deltaS%_n-_idepth;
	if(f>0)
	{
		fx+=f*f*w;
#ifdef USE_SIMPLE_BUT_SLOW_CODE

		matrixn JT;
		solver->calcJacobianTransposeAt(JT, bone1->treeIndex(), _lpos1);
		matrixn JT2;
		solver->calcJacobianTransposeAt(JT2, bone2->treeIndex(), _lpos2);

		JT-=JT2;

		//solver->updateGrad_S_JT(g, -(2.0*w*1)*_n, ti1, _lpos1);
		//solver->updateGrad_S_JT(g, -(2.0*w*-1)*_n, ti2, _lpos2);
		ASSERT(N==JT.rows());
		auto& mJacobianLock=((FullbodyIK_MotionDOF_MultiTarget_lbfgs*)solver)->mJacobianLock;
		bool hasLock=(mJacobianLock.size()==N);
		for(int i=0; i<N; i++)
		{
			if(hasLock && mJacobianLock[i]) continue;
			g[i]-= (2.0*w)*(deltaS%_n-_idepth)*(_n%JT.row(i).toVector3(0));
		}
#else
		solver->updateGrad_S_JT(g, -(2.0*f)*_n, bone1->treeIndex(), _lpos1);
		solver->updateGrad_S_JT(g, (2.0*f)*_n, bone2->treeIndex(), _lpos2);
#endif
	}

	return fx;
}
