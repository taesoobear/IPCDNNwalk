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
	double _w_root, _w_other;
	// workspace
	VectorRn dS;			// delta s
	MatrixRmn Jend;		// Jacobian matrix based on end effector positions
	// workspace end
public:

	void setDampingWeight(double w_root, double w_other)
	{
		_w_root=w_root;
		_w_other=w_other;
	}

	virtual void setParam(const char* type, double value, double value2)
	{
		if(TString("damping_weight")==type)
			setDampingWeight(value, value2);
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
	virtual bool _setEffector(int i, Bone* bone, vector3 const& lpos) { 
		mEffectors[i].bone=bone;
		mEffectors[i].localpos=lpos;
		return true;
	}
	// actually generalized velocity constraint so that the output is mass/inertia independent.
	virtual bool _setMomentumConstraint(int i, vector3 const& ang, vector3 const& lin, double weight){
		mConstraints[i].eType=MotionUtil::RelativeConstraint::MOMENTUM;
		MomentumConstraintInfo* pInfo=new MomentumConstraintInfo((VRMLloader const&)mSkeleton,this);
		pInfo->desired_vel.setVec3(0, ang);
		pInfo->desired_vel.setVec3(3, lin);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setEffectorYConstraint(int i, double weight, const vectorn& effectorWeights)
	{
		mConstraints[i].eType=MotionUtil::RelativeConstraint::OTHERS;
		EEYConstraintInfo* pInfo=new EEYConstraintInfo((VRMLloader const&)mSkeleton,this, effectorWeights);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setPoseConstraint(int i, vectorn const& pose, double weight, int startBoneIndex=1, int endBoneIndex=INT_MAX)
	{
		mConstraints[i].eType=MotionUtil::RelativeConstraint::POSE;
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
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=weight; // by default.
	}
	virtual bool _setEffectorWeight(int i, double w=1.0) { 
		mEffectorWeights(i)=w;
		return true;
	}
	virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori, double weight) { 
		mConstraints[i].eType=MotionUtil::RelativeConstraint::ROT;
		ROTConstraintInfo* pInfo=new ROTConstraintInfo((VRMLloader const&)mSkeleton,this, bone, desired_ori);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=weight;
		return true;
	}
	virtual bool _setPositionConstraint(int i, Bone* bone, vector3 const&lpos, vector3 const& desired_pos, double wx, double wy, double wz) {
		mConstraints[i].eType=MotionUtil::RelativeConstraint::OTHERS;
		EEConstraintInfo* pInfo=new EEConstraintInfo((VRMLloader const&)mSkeleton,this, bone, lpos, desired_pos, wx, wy, wz);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=1.0;
		return true;
	}
	virtual bool _setOrientationConstraint(int i, Bone* bone, quater const& desired_ori) { 
		mConstraints[i].eType=MotionUtil::RelativeConstraint::ROT;
		ROTConstraintInfo* pInfo=new ROTConstraintInfo((VRMLloader const&)mSkeleton,this, bone, desired_ori);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=1.0;
		return true;
	}

	//HERE
	virtual bool _setRelativeConstraint(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2) { 
		mConstraints[i].eType=MotionUtil::RelativeConstraint::RELATIVE_POSITION;
		mConstraints[i].bone1=bone1;
		mConstraints[i].localpos1=lpos1;
		mConstraints[i].bone2=bone2;
		return true;
	}
	virtual bool _setPlaneDistanceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) {
		mConstraints[i].eType=MotionUtil::RelativeConstraint::PLANE_DISTANCE;
		mConstraints[i].bone1=bone;
		mConstraints[i].localpos1=lpos;
		mConstraints[i].normal[0]=global_normal.x;
		mConstraints[i].normal[1]=global_normal.y;
		mConstraints[i].normal[2]=global_normal.z;
		mConstraints[i].idepth=idepth;
		return true;
	}	   
	virtual bool _setHalfSpaceConstraint(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth) {
		mConstraints[i].eType=MotionUtil::RelativeConstraint::HALF_SPACE;
		mConstraints[i].bone1=bone;
		mConstraints[i].localpos1=lpos;
		mConstraints[i].normal[0]=global_normal.x;
		mConstraints[i].normal[1]=global_normal.y;
		mConstraints[i].normal[2]=global_normal.z;
		mConstraints[i].idepth=idepth;
		return true;
	}	   
	virtual bool _setCOMConstraint(int i, vector3 const& com) { 
		mConstraints[i].eType=MotionUtil::RelativeConstraint::COM;
		COMConstraintInfo* pInfo=new COMConstraintInfo((VRMLloader const&)mSkeleton,this, com);
		mConstraints[i].pInfo=pInfo;
		mConstraints[i].weight=1.0;
		return true;
	}	   
	virtual bool _setCOMConstraint(int i, vector3 const& com, double wx, double wy, double wz) {
		mConstraints[i].eType=MotionUtil::RelativeConstraint::COM;
		COMConstraintInfo* pInfo=new COMConstraintInfo((VRMLloader const&)mSkeleton,this, com, wx, wy, wz);
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
		_w_root=0.01;
		_w_other=0.01;
	}
	FullbodyIK_MotionDOF_MultiTarget_lbfgs(MotionDOFinfo const& dofInfo)
		:mDofInfo(dofInfo), mSkeleton(dofInfo.skeleton()),
		 IK_sdls::LoaderToTree(dofInfo.skeleton(), true,false)
	{
		m_x0=NULL;
		_w_root=0.01;
		_w_other=0.01;
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
			con=&con2[0];
			con_size=con2.size();
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
		for(int ii=0; ii<_nonEffector_constraints.size(); ii++)
		{
			MotionUtil::RelativeConstraint* n=_nonEffector_constraints[ii];
			if(n->eType>=MotionUtil::RelativeConstraint::MOMENTUM)
				n->pInfo->initializeConstraint(pose, m_x);
		}
        /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
         */
        int ret = LBFGS::lbfgs(N, m_x, &fx, _evaluate, _progress, this, NULL);

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
		int nCon= _nonEffector_constraints.size();
		int nRow = 3 * nEffector;
		int nCol = nJoint;
		
		// compute jacobian
		IK_sdls::Effector* n;

		dS.SetLength(nRow+3*nCon);

		Jend.SetSize(nRow, nCol);
		Jend.SetZero();

		LBFGS::lbfgsfloatval_t fx = 0.0;
	
		vectorn vec_dist(mTree.effectors.size());
		vec_dist.setAllValue(0);
		for(int ii=0; ii<mTree.effectors.size(); ii++)
		{
			n = mTree.effectors[ii];
			int i = n->GetEffectorNum();
			assert(i==ii);
			if (n->constraint)
			{
				dS.SetTriple(ii, vector3(0,0,0));

				Plane p(n->constraint->normal[0], 
						n->constraint->normal[1], 
						n->constraint->normal[2], 
						n->constraint->idepth);
				double d=p.distance(n->GetS());
				vec_dist[ii]=d;
				if (n->constraint->eType==MotionUtil::RelativeConstraint::PLANE_DISTANCE)
					fx+=d*d;
				else if(d>0)
					fx+=d*d;
			}
			else
			{
				// Compute the delta S value (differences from end effectors to target positions.
				n->computeDeltaS(dS, &mTree);
				fx+=mEffectorWeights(ii)*dS.GetTriple(ii).squaredLength();
			}
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
				if(n->constraint)
				{
					vector3 normal(n->constraint->normal[0],
							n->constraint->normal[1],
							n->constraint->normal[2]);

					if (n->constraint->eType==MotionUtil::RelativeConstraint::PLANE_DISTANCE)
						// plane distance
						g[i]+=2.0*(n->GetS()%normal)*(normal%j)
							+ 2.0*n->constraint->idepth*(normal%j);
					else if( vec_dist[ii]>0)
						g[i]+=2.0*(n->GetS()%normal)*(normal%j)
							+ 2.0*n->constraint->idepth*(normal%j);
				}
				else
				{
					// absolute or relative position
					g[i]-= mEffectorWeights(ii)*2.0*dS.GetTriple(ii)%j;
				}
			}	
		}

		for(int ii=0; ii<_nonEffector_constraints.size(); ii++)
		{
			VRMLloader const& loader=(VRMLloader const&)mSkeleton;
			MotionUtil::RelativeConstraint* n=_nonEffector_constraints[ii];
			double f=n->pInfo->calcObjectiveAndGradient(N, g,x, n->weight);
			fx+=f;
		}
#endif

		// damping terms
		for (int i=0; i<N; i++)
		{
			double w;
			if (i<6) w=_w_root;
			else w=_w_other;
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
		if (!n->constraint)
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
