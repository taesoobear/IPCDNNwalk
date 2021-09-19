
#ifndef _CLASS_NODE
#define _CLASS_NODE

#include "LinearR3.h"
#include "MatrixRmn.h"
typedef double m_real;
#include <memory>
#include "../../math/quater.h"
#include "../../math/transf.h"
#ifndef M_PI 
#define M_PI 3.141592
#endif
enum Purpose {HINGEJOINT, SLIDEJOINT, FREEJOINT,EFFECTOR, DUMMY_EFFECTOR};

// Taesoo Kwon completely rewrote many parts of the original tutorial code by Buss,
// and added support for SlideJoint, FreeJoint, RelativeConstraint, and velocity integration.

namespace MotionUtil
{
	struct RelativeConstraint;
}
namespace IK_sdls
{
	class Tree;

class Node {

	friend class Tree;
	friend class HingeJoint;

protected:
	Node(const vector3& localpos,  Purpose purpose);
public:

	void PrintNode(int depth);
	virtual void InitNode(){};

	// only for hingeJoint
	void SetTheta(double newTheta);
	double GetTheta();
	virtual void SetDTheta(double qd){}
	virtual double GetDTheta(){return 0;}
	void SetJointVel(vector3 const& lin_vel, vector3 const& ang_vel);  // only for free joint.
	void GetJointVel(vector3 & lin_vel, vector3 & ang_vel);
	void SetBodyVel(vector3 const& lin_vel, vector3 const& ang_vel);   // for all joint, for IK

	void calcJacobian(MatrixRmn& J, int i_row, vector3 const& target);
	void getJacobianSparsity(boolN& J);
	void calcRotJacobian(MatrixRmn& J, int i_row);
	void calcSubtreeJacobian(Node* one_above_root, MatrixRmn& J, int i_row, vector3 const& target);

	inline const transf& globalFrame() const {return _global;}
	inline const vector3& GetS() const { return _global.translation; }

	virtual void ComputeS(void); // Compute global positions
	void ComputeDS(); // update velocity (loc_ang_vel, loc_lin_vel), assuming ComputeS has been done.
	void ComputeDQfromDS(); // inverse kinematis (velocity)
	virtual void ComputeW(){}
	virtual void updateLocal(){}

	bool IsEffector() const { return purpose==EFFECTOR; } 
	bool IsJoint() const { return purpose==HINGEJOINT || purpose==SLIDEJOINT; }
	bool IsHingeJoint() const { return purpose==HINGEJOINT;}
	bool IsSlideJoint() const{ return purpose==SLIDEJOINT; }
	bool IsFreeJoint() const {return purpose==FREEJOINT;}

	int GetEffectorNum() const { return seqNumEffector; }
	int GetJointNum() const { return seqNumJoint; }
	int GetParentJointNum() const { if(realparent) return realparent->GetJointNum(); return -1;}

	bool IsFrozen() const { return freezed; }
	void Freeze() { freezed = true; }
	void UnFreeze() { freezed = false; }

	vector3 const& bodyLinVel() { return loc_lin_vel;}
	vector3 const& bodyAngVel() { return loc_ang_vel;}
	vector3 const& relLinVel() { return rel_lin_vel;}
	vector3 const& relAngVel() { return rel_ang_vel;}

	Node* left;				// left child
	Node* right;			// right sibling
	Node* realparent;		// pointer to real parent

	transf _local; // relative potision and orientation
	transf _global; // global position and orientation
	Purpose purpose;		// joint / effector / both

	// calc self-contribution only. (inherited functions, not this one, will be used.)
	virtual void _updateGrad_S_JT(double* g, vector3 const& deltaS, vector3 const& target) {}
	virtual void _updateGrad_S_JT_residual(double* g, vector3 const& deltaS_lpos){}
protected:
	// GetJointVel, SetJointVel (this is J*dq)
	vector3 rel_lin_vel;
	vector3 rel_ang_vel;
	vector3 loc_lin_vel;
	vector3 loc_ang_vel;

	bool freezed;			// Is this node frozen?
	int seqNumJoint;		// sequence number if this node is a joint
	int seqNumEffector;		// sequence number if this node is an effector

	friend class Effector;
	// calc self-contribution only. (inherited functions, not this one, will be used.)
	virtual void _calcJacobian(MatrixRmn& J, int i_row, vector3 const& endPos) {J.SetTriple(i_row,GetJointNum(), vector3(0,0,0)); }

	// calc self-contribution only. (inherited functions, not this one, will be used.)
	virtual void _getJacobianSparsity(boolN& J);
	// calc self-contribution only. (inherited functions, not this one, will be used.)
	virtual void _calcRotJacobian(MatrixRmn& J, int i_row) {J.SetTriple(i_row,GetJointNum(), vector3(0,0,0)); }
};

class HingeJoint : public Node
{
public:
	HingeJoint(const vector3 &lpos, const vector3&, double minTheta=-M_PI, double maxTheta=M_PI, double restAngle=0.);

	virtual void InitNode();
	vector3 const& globalRotationAxis() { return w;}
	double GetTheta() const { return theta; }
	double AddToTheta( double delta ) { theta += delta; return theta; }
	double GetMinTheta() const { return minTheta; }
	double GetMaxTheta() const { return maxTheta; } 
	double GetRestAngle() const { return restAngle; } ;
	void SetTheta(double newTheta) { theta = newTheta; }
	virtual void SetDTheta(double qd);
	virtual double GetDTheta();
	void SetBound(double _minTheta, double _maxTheta)
	{
		minTheta=_minTheta;
		maxTheta=_maxTheta;
	}
	virtual void updateLocal();
	virtual void ComputeW();
	virtual void _calcJacobian(MatrixRmn& J, int i_row, vector3 const& endPos);
	virtual void _updateGrad_S_JT(double* g, vector3 const& deltaS, vector3 const& target);
	virtual void _updateGrad_S_JT_residual(double* g, vector3 const& deltaS_lpos);
	virtual void _calcRotJacobian(MatrixRmn& J, int i_row);
	const vector3& GetW() const { return w; }
	const vector3& GetV() const { return v; }
protected:
	vector3 v;				// rotation axis
	vector3 w;				// Global rotation axis
	double theta;			// joint angle (radian)
	double minTheta;		// lower limit of joint angle
	double maxTheta;		// upper limit of joint angle
	double restAngle;		// rest position angle
	double qd; 				// dtheta
};

class SlideJoint : public HingeJoint
{
public:
	vector3 init_pos;
	SlideJoint(const vector3 &lpos, const vector3& axis);
	vector3 const& globalTranslationAxis() { return w;}
	virtual void updateLocal();
	virtual void SetDTheta(double qd);
	virtual double GetDTheta();
	virtual void _calcJacobian(MatrixRmn& J, int i_row, vector3 const& target);
	virtual void _updateGrad_S_JT(double* g, vector3 const& deltaS, vector3 const& target);
	virtual void _updateGrad_S_JT_residual(double* g, vector3 const& deltaS_lpos){}
	virtual void _calcRotJacobian(MatrixRmn& J, int i_row);
};

class Effector : public Node
{
public:
	MotionUtil::RelativeConstraint* constraint;
	Effector(const vector3& localpos);
	virtual void computeDeltaS(VectorRn &dS, Tree* tree);
	virtual void calcJacobian(Tree* tree, MatrixRmn& J, int i, vector3 const& target);
};

class RelativeConstraint : public Effector
{
public:
	IK_sdls::Effector* _additionalNode;
	RelativeConstraint(const vector3& localpos1);
	// contribution due to joint n.
	virtual void ComputeS(void);
	virtual void computeDeltaS(VectorRn &dS, Tree* tree);
	virtual void calcJacobian(Tree* tree, MatrixRmn& J, int i, vector3 const& target);
};


void angvel2qdot(quater& out, const quater& ep, const vector3& omega);
class FreeJoint :public Node
{
	void pre_integrate();
	void post_integrate();
public:
	FreeJoint();
	virtual void _calcJacobian(MatrixRmn& J, int i_row, vector3 const& endPos);
	virtual void _updateGrad_S_JT(double* g, vector3 const& deltaS, vector3 const& target);
	virtual void _updateGrad_S_JT_residual(double* g, vector3 const& deltaS_lpos);
	virtual void _getJacobianSparsity(boolN& J);
	virtual void _calcRotJacobian(MatrixRmn& J, int i_row);
	void  setTheta(transf const& q)
	{
		_local=q;
	} 
	const transf& getTheta() const
	{
		return _local;
	}
	// dTheta: 0 : rel_lin_vel, 4 : rel_ang_vel 
	void Integrate(vectorn const& dTheta, double timeStep);
	// dTheta: 0 : rel_lin_vel, 3 : rel_ang_vel 
	void Integrate(VectorRn const& dTheta) ;
	/*! @name Velocities and accelerations in parent joint's frame for integration */
	vector3 p_lin_vel;
	vector3 p_ang_vel;
	quater p_ep_dot; 
};
}
#endif
