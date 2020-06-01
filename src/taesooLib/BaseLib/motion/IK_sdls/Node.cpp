// Taesoo Kwon
#include "stdafx.h"
#include <math.h>

#ifdef WIN32
#include <windows.h>
#endif


#include "LinearR3.h"
#include "MathMisc.h"
#include "Node.h"
#include "Tree.h"

using namespace IK_sdls;
extern int RotAxesOn;

Node::Node(const vector3& localpos, Purpose purpose)
	:rel_lin_vel(0,0,0),
	rel_ang_vel(0,0,0)
{
	Node::freezed = false;
	Node::purpose = purpose;
	seqNumJoint = -1;
	seqNumEffector = -1;
	_local.identity();// r will be updated when this node is inserted into tree
	_local.translation=localpos;

	left = right = realparent = 0;
}

void Node::SetTheta(double newTheta) { 
	Msg::verify(IsJoint(), "settheta");
	((HingeJoint*)this)->SetTheta(newTheta);
}

void Node::SetJointVel(vector3 const& lin_vel, vector3 const& ang_vel)
{
	rel_lin_vel=lin_vel;
	rel_ang_vel=ang_vel;
}
void Node::SetBodyVel(vector3 const& lin_vel, vector3 const& ang_vel)
{
	loc_lin_vel=lin_vel;
	loc_ang_vel=ang_vel;
}

void Node::GetJointVel(vector3 & lin_vel, vector3 & ang_vel)
{
	lin_vel=rel_lin_vel;
	ang_vel=rel_ang_vel;
}

double Node::GetTheta() {
	Msg::verify(IsJoint(), "gettheta");
	return ((HingeJoint*)this)->GetTheta();
}
// Compute the global position of a single node
void Node::ComputeS(void)
{
	Node* y = this->realparent;
	updateLocal();
	if (y)
		_global.mult(y->_global,_local);
	else
		_global=_local;
}
void Node::ComputeDS() // update velocity of a single node
{
	// see fk.cpp : calc_velocity() for full details about velocity FK.
	Node* parent = this->realparent;
	if(parent)
	{
		quater t_rel_att;
		t_rel_att.inverse(_local.rotation);
		vector3 v1, v2;
		// compute loc_lin_vel
		v1.cross(parent->loc_ang_vel, _local.translation);
		v2.rotate(t_rel_att, parent->loc_lin_vel);
		loc_lin_vel.rotate(t_rel_att, v1);
		loc_lin_vel += v2;
		loc_lin_vel += rel_lin_vel;
		// compute loc_ang_vel
		loc_ang_vel.rotate(t_rel_att, parent->loc_ang_vel);
		loc_ang_vel += rel_ang_vel;
		// compute loc_com_vel
		//v1.cross(loc_ang_vel, loc_com);
		//loc_com_vel.add(loc_lin_vel, v1);
		//printf("%d :%s %s\n", GetJointNum(), loc_ang_vel.output().ptr(), rel_ang_vel.output().ptr());
		//printf("::%s %s %s %s\n", loc_lin_vel.output().ptr(), _local.translation.output(). ptr(), v1.output().ptr(),rel_lin_vel.output().ptr());
	}
	else
	{
		loc_ang_vel=rel_ang_vel;
		loc_lin_vel=rel_lin_vel;
	}
}
void Node::ComputeDQfromDS()
{
	Node* parent = this->realparent;
	if(parent)
	{
		quater t_rel_att;
		t_rel_att.inverse(_local.rotation);
		vector3 v1, v2;
		// compute loc_lin_vel
		v1.cross(parent->loc_ang_vel, _local.translation);
		v2.rotate(t_rel_att, parent->loc_lin_vel);

		//loc_lin_vel= A+v2+rel_lin_vel
		//-> rel_lin_vel=-A-v2+loc_linvel
		rel_lin_vel.rotate(t_rel_att, v1);
		rel_lin_vel *=-1; // -A
		rel_lin_vel -= v2;
		rel_lin_vel += loc_lin_vel;
		
		// loc_ang_vel= B+rel_ang_vel
		// -> rel_ang_vel= -B+loc_ang_vel
		rel_ang_vel.rotate(t_rel_att, parent->loc_ang_vel);
		rel_ang_vel *=-1; // -B
		rel_ang_vel += loc_ang_vel ;
		//printf("%s %s\n", loc_ang_vel.output().ptr(), rel_ang_vel.output().ptr());
	}
	else
	{
		rel_ang_vel=loc_ang_vel;
		rel_lin_vel=loc_lin_vel;
	}
}

void Node::calcJacobian(MatrixRmn& J, int i, vector3 const& target)
{
	// Find all ancestors (they will usually all be joints)
	// Set the corresponding entries in the Jacobians J, K.
	Node* m = this;
	while ( m ) {
		int j = m->GetJointNum();
		if ( m->IsFrozen() ) {
			J.SetTriple(i, j, VectorR3::Zero);
		}
		else {
			m->_calcJacobian(J, i, target);
		}
		m = m->realparent;
	}
}
void Node::getJacobianSparsity(boolN& J)
{
	// Find all ancestors (they will usually all be joints)
	// Set the corresponding entries in the Jacobians J, K.
	Node* m = this;
	while ( m ) {
		int j = m->GetJointNum();
		if ( m->IsFrozen() ) {
			J.set(j, false);
		}
		else {
			m->_getJacobianSparsity(J);
		}
		m = m->realparent;
	}
}
void Node::_getJacobianSparsity(boolN& J) {J.set(GetJointNum(), true);}
void Node::calcRotJacobian(MatrixRmn& J, int i)
{
	// Find all ancestors (they will usually all be joints)
	// Set the corresponding entries in the Jacobians J, K.
	Node* m = this;
	while ( m ) {
		int j = m->GetJointNum();
		if ( m->IsFrozen() ) {
			J.SetTriple(i, j, VectorR3::Zero);
		}
		else {
			m->_calcRotJacobian(J, i);
		}
		m = m->realparent;
	}
}

void Node::calcSubtreeJacobian(Node* one_above_root, MatrixRmn& J, int i_row, vector3 const& target)
{
	Node* m = this;
	while ( m!=one_above_root ) {
		int j = m->GetJointNum();
		if ( m->IsFrozen() ) {
			J.SetTriple(i_row, j, VectorR3::Zero);
		}
		else {
			m->_calcJacobian(J, i_row, target);
		}
		m = m->realparent;
	}
}

void HingeJoint::updateLocal()
{
	_local.rotation.setRotation(v, theta);
}

void HingeJoint::SetDTheta(double _qd)
{
	qd=_qd;
	rel_ang_vel.mult(v, qd);
	rel_lin_vel.zero();
	//printf("%f %s %s\n", qd, v.output().ptr(), rel_ang_vel.output().ptr());
}
double HingeJoint::GetDTheta()
{
	vector3 av(ABS(v.x), ABS(v.y), ABS(v.z));
	if(av.x>av.y)
	{
		if (av.z>av.x)
			qd=rel_ang_vel.z/v.z;
		else
			qd=rel_ang_vel.x/v.x;
	}
	else
	{
		if(av.z>av.y)
			qd=rel_ang_vel.z/v.z;
		else
			qd=rel_ang_vel.y/v.y;
	}
	//printf("%f %s %s\n", qd, v.output().ptr(), rel_ang_vel.output().ptr());
	return qd;
}
// Compute the global rotation axis of a single node
void HingeJoint::ComputeW(void)
{
	Node* y = this->realparent;
	vector3 temp;
	temp.rotate(_global.rotation,v);
	//printf("%s %s\n", w.output().ptr(), temp.output().ptr());
	w=temp;
}


void Node::PrintNode(int depth)
{
	for(int i=0; i<depth; i++) cout << " ";
	cout << "r : (" << _local.translation << ")\n";
	for(int i=0; i<depth; i++) cout << " ";
	cout << "s : (" << _global.translation << ")\n";
	cout << "v : (" << loc_lin_vel << ")\n";
	cout << "w : (" << loc_ang_vel << ")\n";
//	for(int i=0; i<depth; i++) cout << " ";
//	cout << "w : (" << w << ")\n";
	if(realparent)
	{
		for(int i=0; i<depth; i++) cout << " ";
		cout << "realparent : " << realparent->seqNumJoint << "\n";
	}
}

void HingeJoint::InitNode()
{
	theta = 0.0;
}


Effector::Effector(const vector3& localpos)
:Node(localpos,  EFFECTOR )
{
	constraint=NULL;
}
void Effector::computeDeltaS(VectorRn &dS, Tree* tree)
{
	int i = GetEffectorNum();
	const vector3& targetPos = tree->target[i];

	// Compute the delta S value (differences from end effectors to target positions.
	vector3 temp;
	temp = targetPos;
	temp -= GetS();
	dS.SetTriple(i, temp);
}

/*
void Effector::calcJacobian(Node * m, MatrixRmn& J)
{
	int i_row = GetEffectorNum();
	m->calcJacobian(J, i_row, GetS());	
}
*/

void Effector::calcJacobian(Tree* tree, MatrixRmn& J, int i, vector3 const& target)
{
	// Find all ancestors (they will usually all be joints)
	// Set the corresponding entries in the Jacobians J, K.
	Node* m = tree->GetParent(this);
	while ( m ) {
		int j = m->GetJointNum();
		assert ( 0 <=i && i<tree->GetNumEffector() && 0<=j && j<tree->GetNumJoint() );
		if ( m->IsFrozen() ) {
			J.SetTriple(i, j, VectorR3::Zero);
		}
		else {
			m->_calcJacobian(J, i, target);
		}
		m = tree->GetParent( m );
	}
}

RelativeConstraint::RelativeConstraint(const vector3& localpos)
:Effector(localpos)
{
	_additionalNode=new Effector(VectorR3::Zero);
	_additionalNode->purpose=DUMMY_EFFECTOR;
}
void RelativeConstraint::ComputeS(void)
{
	Effector::ComputeS();
	_additionalNode->ComputeS();
}
#define TEST_ONLY_MARKER2 0
void RelativeConstraint::computeDeltaS(VectorRn &dS, Tree* tree)
{
	int i = GetEffectorNum();
#if TEST_ONLY_MARKER2
	const vector3& targetPos = vector3(-1.1608 ,0.717139 ,-0.158597); 
	vector3 temp = targetPos - _additionalNode->GetS();
#else
	const vector3& targetPos = GetS();
	vector3 temp = _additionalNode->GetS()-targetPos;
#endif
	//printf("%s: ", tree->target[0].output().ptr());

	// Compute the delta S value (differences from end effectors to target positions.
	//printf("%s\n", GetS().output().ptr());
	dS.SetTriple(i, temp);
}
void RelativeConstraint::calcJacobian(Tree* tree, MatrixRmn& J, int i_row, vector3 const& target)
{
#if TEST_ONLY_MARKER2
	_additionalNode->calcJacobian(tree, J, i_row, _additionalNode->GetS());
#else
	Effector::calcJacobian(tree, J, i_row, GetS());
	
	MatrixRmn J2(3, J.GetNumColumns());
	J2.SetZero();
	_additionalNode->calcJacobian(tree, J2, 0, _additionalNode->GetS());

	for(int j=0; j<J.GetNumColumns(); j++)
	{
		J.SetTriple( i_row, j, J.GetTriple(i_row, j)-J2.GetTriple(0, j));
	}
#endif
}

/*
void RelativeConstraint::calcJacobian(Node* m, MatrixRmn& J)
{
	MatrixRmn J2(1, J.GetNumColumns());
	int i_row = GetEffectorNum();
	m->calcJacobian(J, i_row, GetS());
	//J.SetTriple(i_row,m->GetJointNum(), ...)
	m->calcJacobian(J2, 0, _additionalNode->GetS());

	if(m->IsFreeJoint())
	{
		for(int i=0; i<3; i++)
		{
			vector3 axis0=J.GetTriple(i_row, m->GetJointNum()+i);
			vector3 lin=J.GetTriple(i_row, m->GetJointNum()+i+3);

			vector3 axis0_2=J2.GetTriple(0, m->GetJointNum()+i);
			vector3 lin_2=J2.GetTriple(0, m->GetJointNum()+i+3);

			J.SetTriple(i_row, m->GetJointNum()+i, axis0_2-axis0);
			J.SetTriple(i_row, m->GetJointNum()+i+3,lin_2-lin); 
		}
	}
	else
	{
		vector3 lin=J.GetTriple(i_row,m->GetJointNum());
		vector3 lin2=J2.GetTriple(0,m->GetJointNum());

		J.SetTriple(i_row,m->GetJointNum(), lin2-lin);
	}


}
*/

void HingeJoint::_calcJacobian(MatrixRmn& J, int i_row, vector3 const& target) 
{
	// see calc_jacobian_rotate in jacobi.cpp
	vector3 lin = GetS(); // joint pos (abs_pos).
	lin-=target; // arm = joint pos - target
	lin=lin.cross(GetW()); // lin.cross(abs_pos-target, axis0)

	J.SetTriple(i_row,GetJointNum(), lin);
}
void HingeJoint::_calcRotJacobian(MatrixRmn& J, int i_row) 
{
	J.SetTriple(i_row,GetJointNum(), GetW());
}

HingeJoint::HingeJoint(const vector3& localpos, const vector3& v,  double minTheta, double maxTheta, double restAngle)
:Node(localpos,  HINGEJOINT )
{
	HingeJoint::v = v;				// Rotation axis when joints at zero angles
	theta = 0.0;
	HingeJoint::minTheta = minTheta;
	HingeJoint::maxTheta = maxTheta;
	HingeJoint::restAngle = restAngle;
}

SlideJoint::SlideJoint(const vector3& localpos, const vector3& v)
:HingeJoint(localpos, v, -10000, 10000)
{
	init_pos=localpos;
	purpose=SLIDEJOINT;
}
void SlideJoint::updateLocal()
{
	_local.translation=init_pos+v*theta;
}
void SlideJoint::SetDTheta(double _qd)
{
	qd=_qd;
	rel_lin_vel.mult(v, qd);
	rel_ang_vel.zero();
	//printf("slide %d %f %s %s\n",GetJointNum(), qd, v.output().ptr(), rel_lin_vel.output().ptr());
}
double SlideJoint::GetDTheta()
{
	vector3 av(ABS(v.x), ABS(v.y), ABS(v.z));
	// compute qd from rel_lin_vel
	if(av.x>av.y)
	{
		if (av.z>av.x)
			qd=rel_lin_vel.z/v.z;
		else
			qd=rel_lin_vel.x/v.x;
	}
	else
	{
		if(av.z>av.y)
			qd=rel_lin_vel.z/v.z;
		else
			qd=rel_lin_vel.y/v.y;
	}
	return qd;
}
void SlideJoint::_calcJacobian(MatrixRmn& J, int i_row, vector3 const& target) 
{
	// see calc_jacobian_rotate in jacobi.cpp
	/* FAIL! does not work
	if(theta>maxTheta)
	{
		if(GetW()%(target-GetS())>0)
		{
			J.SetTriple(i_row,GetJointNum(), vector3(0,0,0));
			return;
		}
	}
	if(theta<minTheta)
	{
		if(GetW()%(target-GetS())<0)
		{
			J.SetTriple(i_row,GetJointNum(), vector3(0,0,0));
			return;
		}
	}*/
	J.SetTriple(i_row,GetJointNum(), GetW());
}
void SlideJoint::_calcRotJacobian(MatrixRmn& J, int i_row) 
{
	J.SetTriple(i_row,GetJointNum(), VectorR3::Zero);
}

FreeJoint::FreeJoint()
	:Node(VectorR3::Zero, FREEJOINT)
{
}

void FreeJoint::_calcJacobian(MatrixRmn& J, int i_row, vector3 const& target)
{
	vector3 axis(0,0,0);
	vector3 axis0;
	vector3 lin;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		axis0.rotate(_global.rotation, axis);
		lin.cross(axis0, target-GetS());
		J.SetTriple(i_row, GetJointNum()+i, axis0);
		J.SetTriple(i_row, GetJointNum()+i+3,lin); 
		axis[i]=0.0;
	}
}
void FreeJoint::_getJacobianSparsity(boolN& J)
{
	for(int i=0; i<3; i++)
	{
		J.set( GetJointNum()+i, true);
		J.set( GetJointNum()+i+3,true); 
	}
}
void FreeJoint::_calcRotJacobian(MatrixRmn& J, int i_row)
{
	vector3 axis(0,0,0);
	vector3 axis0;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		axis0.rotate(_global.rotation, axis);
		J.SetTriple(i_row, GetJointNum()+i, VectorR3::Zero);
		J.SetTriple(i_row, GetJointNum()+i+3,axis0); 
		axis[i]=0.0;
	}
}

void FreeJoint::Integrate(vectorn const& dTheta, double timeStep) 
{
	rel_lin_vel=dTheta.toVector3(0)*timeStep;
	rel_ang_vel=dTheta.toVector3(4)*timeStep;

	pre_integrate();
	// integrate
	// init.cpp:
	// chain->all_value=&rel_pos [0,3)  -----------> _local.translation in my case
	// chain->all_value=&rel_ep [4,6) -------------> _local.rotation in my case
	// chain->all_value_dot=&p_lin_vel [0,3) ----------> all_value_dot is constant for IK because there is no acceleration
	// chain->all_value_dot=&p_ep_dot [4,6)
	_local.translation+=p_lin_vel;
	_local.rotation+=p_ep_dot;

	post_integrate();
}
void FreeJoint::Integrate(VectorRn const& dTheta) 
{
	// see above. 
	int i = this->GetJointNum();
	rel_lin_vel=dTheta.toVector3(i);
	rel_ang_vel=dTheta.toVector3(i+3);

	pre_integrate();
	_local.translation+=p_lin_vel;
	_local.rotation+=p_ep_dot;

	post_integrate();
}
	
void FreeJoint::pre_integrate()
{
	// compute p_lin_Vel
	p_lin_vel.rotate(_local.rotation, rel_lin_vel);
	p_ang_vel.rotate(_local.rotation, rel_ang_vel);
	angvel2qdot(p_ep_dot, _local.rotation, rel_ang_vel);
}
void FreeJoint::post_integrate()
{
	_local.rotation.normalize();
	quater ratt;
	ratt.inverse(_local.rotation);
	rel_lin_vel.rotate(ratt, p_lin_vel);
	rel_ang_vel.rotate(ratt, p_ang_vel);
}
void IK_sdls::angvel2qdot(quater& out, const quater& q, const vector3& omega)
{
	// W(t)=(0, omega.x, omega.y, omega.z)
	// qdot = 0.5 * W(t) * q
	// Using quaternion multiplication rule,...
	// qdot =0.5*[ -qx -qy -qz; qw qz -qy ; -qz qw qx; qy -qx qw] *[ x y z]'
	// .. is different from the code below. The above W(t) is the global angular velocity.
	//
	// When omega is the body angular velocity
	// qdot = 0.5 * q * W(t) 
	// Using quaternion multiplication rule,...
	
	double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
	double x = omega.x, y=omega.y, z = omega.z;
	out.w = - 0.5 * (qx*x + qy*y + qz*z);
	out.x = 0.5 * (qw*x - qz*y + qy*z);
	out.y = 0.5 * (qz*x + qw*y - qx*z);
	out.z = 0.5 * (- qy*x + qx*y + qw*z);
}
