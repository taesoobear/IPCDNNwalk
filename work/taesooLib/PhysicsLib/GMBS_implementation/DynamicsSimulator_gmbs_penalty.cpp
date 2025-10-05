 // -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include "../physicsLib.h"
#include <vector>
#include <map>
#include <algorithm>

#include "DynamicsSimulator_gmbs_penalty.h"
#include "gbody_rigid.h"
#include "gjoint_revolute.h"
#include "gjoint_free.h"
#include "gjoint_fixed.h"
#include "gjoint_prismatic.h"
#include "gjoint_revolute.h"
#include "gjoint_spherical.h"
#include "gsystem.h"
#include "rmatrix3j.h"
#include "liegroup.h"

#include "../BaseLib/motion/VRMLloader_internal.h"

//#define DEBUG_gmbs_WRAP

inline void ASSERT_SIMILAR(vector3 const& x, Vec3 const& y)
{
	ASSERT(isSimilar(x.x, y[0], 0.001));
	ASSERT(isSimilar(x.y, y[1], 0.001));
	ASSERT(isSimilar(x.z, y[2], 0.001));
}

BodyPart::BodyPart(const char* name)
{
	body=new GBodyRigid(); joint=NULL;
	body->setName(name);
	this->name=name;
	this->boneId=-1;
}

BodyPart::~BodyPart()
{
	delete body;
	delete joint;
}

class worldVel :public se3
{
};
template <class TYPE>
static void getMat3(matrix3 & out, const TYPE& in)
{	for(int i=0;i<3;i++)
		for(int j=0; j<3; j++)
			out(i,j)=in(i,j);
}


void BodyPart::addChild(BodyPart* link)
{
	children.push_back(link);
	assert(link->parent==this);
}

 
#include "../Liegroup.inl"
void zero(matrix4 & out)
{
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			out.m[i][j]=0;
}
void RMat_assign(RMatrix & out, matrixn const& in)
{
	out.ReNew(in.rows(), in.cols());

	for(int i=0; i<in.rows(); i++)
		for(int j=0; j<in.cols(); j++)
			out(i,j)=in(i,j);
}

void RMat_assign(matrixn & out, RMatrix const& in)
{
	out.setSize(in.RowSize(), in.ColSize());

	for(int i=0; i<out.rows(); i++)
		for(int j=0; j<out.cols(); j++)
			out(i,j)=in(i,j);
}
void RMat_print(RMatrix const & in)
{
	matrixn in2;
	RMat_assign(in2, in);
	printf("%s\n", in2.output().ptr());
}
void RMat_assign(matrixn & out, SO3 const& in)
{
	out.setSize(3,3);

	for(int i=0; i<out.rows(); i++)
		for(int j=0; j<out.cols(); j++)
			out(i,j)=in(i,j);
}

 //#define PROFILE
#ifdef PROFILE
#include "../BaseLib/utility/QPerformanceTimer.h"
#undef BEGIN_TIMER
#undef END_TIMER

#define DEFINE_TIMER(x) QPerformanceTimerCount x(300, #x)
#define BEGIN_TIMER(x) x.start()
#define END_TIMER(x) x.stop()
#else
#define DEFINE_TIMER(x) 
#define BEGIN_TIMER(x) 
#define END_TIMER(x) 

#endif

DEFINE_TIMER(setLinkD);
DEFINE_TIMER(initExternalForce);
DEFINE_TIMER(fwd_a);
DEFINE_TIMER(fwd_b);
DEFINE_TIMER(fwd_c);
DEFINE_TIMER(taesooFK);
DEFINE_TIMER(calcDynamics);
DEFINE_TIMER(calcDynamicsFK);

using namespace OpenHRP;
using namespace std;

// #define INTEGRATOR_DEBUG
static const int debugMode = false;
static const bool enableTimeMeasure = false;


static inline double getLimitValue(vectorn const& limitseq, double defaultValue)
{
	return (limitseq.size() == 0) ? defaultValue : limitseq[0];
}


static BodyPart* createLink
(DynamicsSimulator_gmbs_penalty& sim, BodyPart* body, int index, LinkInfoSequence const& iLinks, const matrix3& parentRs)
{
	LinkInfo const& iLink = iLinks[index];

	int jointId = iLink.jointId;

	
	BodyPart* link = new BodyPart(iLink.name.ptr());	

	sim._cinfo.back()->_links[index]=link;
	link->parent=body;
	link->jointId = jointId;

	ASSERT(jointId==index);
	
	::vector3 b =iLink.translation;
	link->Rs = (parentRs * iLink.rotation);
	const matrix3& Rs = link->Rs;

	link->jointType=iLink.jointType;

	if(jointId < 0){
		if(link->jointType == HRP_JOINT::ROTATE || 
			link->jointType == HRP_JOINT::SLIDE ||
			link->jointType == HRP_JOINT::BALL){
				std::cerr << "Warning:  Joint ID is not given to joint " << iLink.name
					 << "." << std::endl;
		}
	}

	::vector3 a(0,0,0);
	if(iLink.jointAxis=="X")
		a.x=1.0;
	else if(iLink.jointAxis=="Y")
		a.y=1.0;
	else if(iLink.jointAxis=="Z")
		a.z=1.0;
		
	::vector3 axis=Rs*a;

	if(link->jointType == HRP_JOINT::ROTATE ){
		GJointRevolute* joint=new GJointRevolute();		
		joint->setAxis(toGMBS(axis));
		link->joint=joint;
	} else if(link->jointType == HRP_JOINT::SLIDE){
		GJointPrismatic* joint=new GJointPrismatic();
		joint->setAxis(toGMBS(axis));
		link->joint=joint;		
	} else if(link->jointType == HRP_JOINT::FIXED){
		GJointFixed* joint=new GJointFixed();
		link->joint=joint;
	} else if(link->jointType == HRP_JOINT::BALL){
		GJointSpherical* joint=new GJointSpherical();
		link->joint=joint;
	} else if(link->jointType == HRP_JOINT::FREE){
		GJointFreeC2* joint=new GJointFreeC2();
		link->joint=joint;		
	}

	link->joint->setName(std::string(iLink.name.ptr()));

	// SE(3): {body} --> {com}
	SE3 T_com=SE3(SO3(), toGMBS(iLink.centerOfMass));
	if (iLink.mass==0.1){
		link->body->setMass(0, toGMBS(iLink.centerOfMass));
	}
	else
	{
		link->body->setMass(iLink.mass, iLink.inertia._11, iLink.inertia._22,iLink.inertia._33, iLink.inertia._12, iLink.inertia._13,iLink.inertia._23, T_com ); 
		//printf("ilink: %f %f %f\n", iLink.inertia._11, iLink.inertia._22, iLink.inertia._33);
		//GBodyRigid* body=link->body;
		//printf("Ii: %f %f %f %f %f %f, %f\n", body->I._I[0],body->I._I[1],body->I._I[2],body->I._I[3],body->I._I[4],body->I._I[5],body->I._m);
	}

	// double maxlimit = numeric_limits<double>::max();

	// link->ulimit  = getLimitValue(ulimit,  +maxlimit);
	// link->llimit  = getLimitValue(llimit,  -maxlimit);
	// link->uvlimit = getLimitValue(uvlimit, +maxlimit);
	// link->lvlimit = getLimitValue(lvlimit, -maxlimit);

	const ::vector3& rc = iLink.centerOfMass;
	link->centerOfMass=Rs*rc;

	// LTR'
	// relativeFrame=m_sM*_T
	// bodyFrame3=T0 * L1T1R1 * L2T2R2 * L3T3R3 (where T0 is m_sFrame of the root body)

	ASSERT(link->joint);

	link->joint->connectBodies(link->parent->body,
							   link->body);

	if(index==0)
		link->joint->setPosition(Vec3(0,0,0), // discard default root translation as they are duplicated in the motion data.
							 Vec3(0,0,0));
	else
		link->joint->setPosition(toGMBS(iLink.translation), 
							 Vec3(0,0,0));

	// a stack is used for keeping the same order of children
	std::stack<BodyPart*> children;
	
	int childIndex = iLink.daughter;
	while(childIndex != -1){
	    BodyPart* childLink = createLink(sim,link, childIndex, iLinks, Rs);
	    if(childLink){
			children.push(childLink);
		}
	    childIndex = iLinks[childIndex].sister;
	}
	while(!children.empty()){
		link->addChild(children.top());
		children.pop();
	}
	return link;
}


DynamicsSimulator_gmbs_penalty::System::System()
{
	_system=new GSystem();
	_ground=new BodyPart("ground");
}
DynamicsSimulator_gmbs_penalty::System::~System()
{

	for(int i=0; i<_links.size(); i++)
		delete(_links[i]);

	delete _system;
	delete _ground;
}
DynamicsSimulator_gmbs_penalty::DynamicsSimulator_gmbs_penalty()
:DynamicsSimulator_penaltyMethod()
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs_penalty::DynamicsSimulator_gmbs_penalty()" << endl;
	}

}


DynamicsSimulator_gmbs_penalty::~DynamicsSimulator_gmbs_penalty()
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs_penalty::~DynamicsSimulator_gmbs_penalty()" << endl;
	}
//	delete _G;

	for (int i=0; i<_cinfo.size(); i++)
		delete _cinfo[i];	
}

static void setExactStateSpherical(DynamicsSimulator_gmbs_penalty& sim, int ichara, int coord_chart, double* q, double* dq, double* ddq=NULL)
{
	std::vector<BodyPart*>& cinfo=sim._cinfo[ichara]->_links;
	VRMLloader& l=*sim._characters[ichara]->skeleton;

	if(l.VRMLbone(1).mJoint->jointType==HRP_JOINT::FREE)
	{
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		j->spherical_joint.coord_chart=(CoordinateChartForSphericalJoint)(coord_chart);
		j->set_q(q);
		j->set_dq(dq);
		if (ddq) j->set_ddq(ddq);
		j->update_short();
	}
}
void DynamicsSimulator_gmbs_penalty::test(const char* test, matrixn& out)
{
	TString id(test);
 
	GSystem* _system=_cinfo[0]->_system;
	if(id=="PositionCOM")
	{
		RMatrix DpDq;
		_system->calcDerivative_PositionCOMGlobal_Dq(DpDq);
		RMat_assign(out, DpDq);
	}
	else if(id=="getRoot_invR")
	{
		std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		SO3 invR=j->spherical_joint.inv_T.GetRotation();
		RMat_assign(out, invR);
	}
	else if(id=="getRoot_invS")
	{
		std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		SO3 invR=j->spherical_joint.inv_T.GetRotation();
		RMatrix invS=Inv(j->S);
		RMat_assign(out, invS);
	}
	else if(id=="getExactStateSpherical")
	{
		std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		out.setSize(1,13);
		out(0,0)=(double)(int)j->spherical_joint.getCoordinateChart();
		double* temp=&out(0,1);
		j->get_q(&out(0,1));
		j->get_dq(&out(0,7));
	}
	else if(id=="setExactStateSpherical")
	{
		setExactStateSpherical(*this, 0, (int)out(0,0), &out(0,1), &out(0,7));
	}
	else if (id=="PositionCOM2")
	{
		RMatrix DpDq;
		_system->calcDerivative_PositionCOMGlobal_Dq_2(DpDq);
		RMat_assign(out, DpDq);
	}
	else if (id=="setter")
	{
		out.setValues(3,2),
			3,4,
			5,2,
			3,4;
	}
	else Msg::msgBox("no %s", test);
}

void DynamicsSimulator_gmbs_penalty::System::updateDOFindexMap(GSystem* _system, std::vector<BodyPart*>const & cinfo, VRMLloader const& l )
{
	DOFindexToCoordIndex.setSize(l.dofInfo.numDOF());
	DOFindexToCoordIndex.setAllValue(-1);
	coordIndexToDOFindex.setSize(l.dofInfo.numDOF());
	coordIndexToDOFindex.setAllValue(-1);
		
	DQindexToCoordIndex.setSize(l.dofInfo.numActualDOF());
	DQindexToCoordIndex.setAllValue(-1);
	coordIndexToDQindex.setSize(l.dofInfo.numActualDOF());
	coordIndexToDQindex.setAllValue(-1);
	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);

		int sDOF=l.dofInfo.startT(i);
		int sDQ=l.dofInfo.startDQ(i);
		int nDOF=l.dofInfo.endR(i)-sDOF;

		int jointType=b.mJoint->jointType;
		int sj=b.mJoint->jointStartId;

		//printf("Bone: %s\n", b.NameId);
		if(jointType==HRP_JOINT::ROTATE)
		{
			for(int jj=0; jj<nDOF; jj++)
			{
				GJointRevolute* j=((GJointRevolute*)cinfo[jj+sj]->joint);
				//printf("%s ", j->name.c_str());
				int coordIndex=_system->getIndexOfCoordinate(&j->coordinate);
				DOFindexToCoordIndex[sDOF+jj]=coordIndex;
				coordIndexToDOFindex[coordIndex]=sDOF+jj;
				DQindexToCoordIndex[sDQ+jj]=coordIndex;
				coordIndexToDQindex[coordIndex]=sDQ+jj;
			}
		}
		else if(jointType==HRP_JOINT::SLIDE)
		{
			for(int jj=0; jj<nDOF; jj++)
			{
				GJointPrismatic* j=((GJointPrismatic*)cinfo[jj+sj]->joint);
				//printf("%s ", j->name.c_str());
				int coordIndex=_system->getIndexOfCoordinate(&j->coordinate);
				DOFindexToCoordIndex[sDOF+jj]=coordIndex;
				coordIndexToDOFindex[coordIndex]=sDOF+jj;
				DQindexToCoordIndex[sDQ+jj]=coordIndex;
				coordIndexToDQindex[coordIndex]=sDQ+jj;
			}
		}
		else 
		{
			GJoint* j=cinfo[sj]->joint;
			//printf("%s ", j->name.c_str());

			list<GCoordinate *>::iterator iter_pcoord;
			for(iter_pcoord=j->pCoordinates.begin(); iter_pcoord!=j->pCoordinates.end(); ++iter_pcoord)
			{
				//printf("%d ", _system->getIndexOfCoordinate(*iter_pcoord));
			}
		}
	}
}

void DynamicsSimulator_gmbs_penalty::_registerCharacter
(
 const char *name,
 CharacterInfo const& chara
 )
{
	int n = chara.links.size();
	LinkInfoSequence const& iLinks = chara.links;

	_cinfo.resize(_cinfo.size()+1);
	_cinfo[_cinfo.size()-1]=new System();
	std::vector<BodyPart*>& links=_cinfo.back()->_links;
	links.resize(n);
	for(int i=0;i<n; i++) links[i]=NULL;
	int rootIndex = -1;

	for(int i=0; i < n; ++i){
		if(iLinks[i].mother < 0){
			if(rootIndex < 0){
				rootIndex = i;
			} else {
				rootIndex= -1; // more than one root !
				break;
			}
		}
	}
	
	if(rootIndex!=-1){
		
		matrix3 Rs;
		Rs.identity();
		
		collisionDetector->addModel(name, chara);

		BodyPart* rootLink = createLink(*this, _cinfo.back()->_ground, rootIndex, iLinks, Rs);
		_cinfo.back()->_system->buildSystem(_cinfo.back()->_ground->body);

		_characters.push_back(new DynamicsSimulator::Character(chara.loader));
		for (int i=2; i<chara.loader->numBone(); i++)
		{
			VRMLloader const& l=*(chara.loader);
			VRMLTransform& b=(VRMLTransform&)chara.loader->bone(i);
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;
			if (nDOF==4)
				links[sj]->boneId=i;
			else
				links[sj+nDOF-1]->boneId=i;
		}
		links[0]->boneId=1;
	}
	if (_characters.size()==1)
	{
		// gmbs doesn't seem to initialze
		double q[]={0,0,0,0,0,0};
		setExactStateSpherical(*this, 0,0, q,q,q);
	}
	_cinfo.back()->updateDOFindexMap(_cinfo.back()->_system, _cinfo.back()->_links, *chara.loader);
	//AISTsim->registerCharacter (name,chara);
}

void DynamicsSimulator_gmbs_penalty::setTimestep(double timeStep)
{
	_timeStep=timeStep;
}

void DynamicsSimulator_gmbs_penalty::init(
		double timeStep,
		OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{	
	_timeStep=timeStep;
	_currTime	=0;
	_integrateOpt = integrateOpt;
	/*
	if(integrateOpt == OpenHRP::DynamicsSimulator::EULER){
		_world->SetIntegrator(VP::EULER);
	} else {
		_world->SetIntegrator(VP::RK4);		
	}*/
	for(int i=0; i<_cinfo.size(); i++)
		_cinfo[i]->_system->updateKinematics();

}

extern bool _debug_DynamicsSimulator_penaltyMethod_calcContactForce;

void DynamicsSimulator_gmbs_penalty::initSimulation()
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs_penalty::initSimulation()" << endl;
	}
	//_world->Initialize();
	for(int i=0; i<_cinfo.size(); i++)
		_cinfo[i]->_system->updateKinematics();

	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();

	_debug_DynamicsSimulator_penaltyMethod_calcContactForce=true;
	if(_debug_DynamicsSimulator_penaltyMethod_calcContactForce)
		_debugInfo="";
	_calcContactForce(*collisions);
	_debug_DynamicsSimulator_penaltyMethod_calcContactForce=false;

}

::vector3 DynamicsSimulator_gmbs_penalty::calculateCOMvel(int ichara, double& outputTotalMass)
{
	return toBase(_cinfo[ichara]->_system->getVelocityCOMGlobal());
}

::vector3 DynamicsSimulator_gmbs_penalty::calculateCOMacc(int ichara) 
{
	return toBase(_cinfo[ichara]->_system->getAccelerationCOMGlobal());
}
inline GBodyRigid* getBody(const std::vector<BodyPart*>& cinfo, VRMLTransform* b)
{
	return cinfo[b->HRPjointIndex(b->numHRPjoints()-1)]->body;
}
double DynamicsSimulator_gmbs_penalty::calcKineticEnergy() const
{
	int ichara=0;
	VRMLloader const& l=*_characters[ichara]->skeleton;
	const std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	double kineticEnergy=0;
	// double kineticEnergy2=0;
	for (int i=1; i<l.numBone(); i++){
		GBodyRigid* pbody=getBody(cinfo, &l.VRMLbone(i));
		kineticEnergy+=pbody->V*(pbody->I*pbody->V);	
		// VRMLTransform& b=l.VRMLbone(i);
		// ::vector3 lpos=b.localCOM();
		// double v=DynamicsSimulator::getWorldVelocity(0, &b,lpos ).length();
		// kineticEnergy2+=l.VRMLbone(i).mass()*v*v;
	}
	// printf("energy:%f %f\n", kineticEnergy, kineticEnergy2);
	return 0.5*kineticEnergy;
}
void DynamicsSimulator_gmbs_penalty::getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& _localpos
			, ::vector3& velocity) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	velocity =toBase(link->getVelocityLinearGlobal(toGMBS(_localpos)));
}	

void DynamicsSimulator_gmbs_penalty::getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	angvel=toBase(link->getVelocityAngularGlobal());
} 

void DynamicsSimulator_gmbs_penalty::getBodyVelocity(int ichara, VRMLTransform* b, Liegroup::se3& V) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	V.W()=toBase(link->V.GetW());
	V.V()=toBase(link->V.GetV());
}
void DynamicsSimulator_gmbs_penalty::getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& _localpos
			, ::vector3& acc) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	acc=toBase(link->getAccelerationLinearGlobal(toGMBS(_localpos)));
}

void DynamicsSimulator_gmbs_penalty::getWorldAngAcc(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	angvel=toBase(link->getAccelerationAngularGlobal());
} 

void DynamicsSimulator_gmbs_penalty::addForceToBone
(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force)
{
	ContactForce cf;
	cf.chara=ichara;
	cf.bone=b;
	cf.f=force;
	cf.p=localpos;
	_externalForces.push_back(cf);

	/*
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);

	//// VP uses inertial frame for external forces.
	::vector3 gf=getWorldState(ichara)._global(*b).toGlobalDir(force);
	::vector3 gp=getWorldState(ichara)._global(*b).toGlobalPos(localpos);
	
	link->addExternalForceGlobally(dse3(toGMBS(gp.cross(gf)), toGMBS(gf)));
	*/
}

static void _addForceToLink(DynamicsSimulator_gmbs_penalty& sim, int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force, ::vector3 const& tau)
{
	GBodyRigid* link=getBody(sim._cinfo[ichara]->_links,b);

	//// VP uses inertial frame for external forces.
	::vector3 gf=sim.getWorldState(ichara)._global(*b).toGlobalDir(force);
	::vector3 gp=sim.getWorldState(ichara)._global(*b).toGlobalPos(localpos);
	::vector3 gtau=sim.getWorldState(ichara)._global(*b).toGlobalDir(tau);
	
	// T=(I,gp), U=(R,t) where U is the global transformation of b
	// global force applied at gp:
	//    (gtau, gf)
	// spatial force:
	//    dAd(-gp)*(gtau,gf)=(gtau+gp.cross(gf), gf)
	// local force:
	//    dAd(B)*dAd((-gp))*(gtau,gf)
	//    = dAd((-gp)*B)*(gtau,gf)
	//    = dAd(R*(-lp))*(gtau, gf)
	//    (because (-gp)*B*x = R*x+t-R*lp-t = R(x-lp)
	//    = dAd(R*(-lp))*dAd(R.inv)*(ltau,lf)
	//    = dAd((-lp))*(ltau,lf)
	
	link->addExternalForceGlobally(dse3(toGMBS(gp.cross(gf)+gtau), toGMBS(gf)));
//	link->addExternalForceLocally(dse3(toGMBS(localpos.cross(force)+tau), toGMBS(force)));
//
}

static bool calc_dyna(DynamicsSimulator_gmbs_penalty& sim)
{
	list<GCoordinate *>::iterator iter_pcoord;

	//	sim._system->updateKinematics();

	BEGIN_TIMER(calcDynamicsFK);
	for(int i=0; i<sim._cinfo.size(); i++)
		sim._cinfo[i]->_system->calcDynamics();
	END_TIMER(calcDynamicsFK);
	return true;
}

static bool integrate_ee(DynamicsSimulator_gmbs_penalty& sim, double h_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	if ( !calc_dyna(sim) ) return false;

	for(int i=0; i<sim._cinfo.size(); i++)
	{
		for (iter_pcoord = sim._cinfo[i]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[i]->_system->pCoordinates.end(); iter_pcoord++) {
			(*iter_pcoord)->q += (*iter_pcoord)->dq * h_;
			(*iter_pcoord)->dq += (*iter_pcoord)->ddq * h_;
		}
	}

	return true;
}

static bool integrate_me(DynamicsSimulator_gmbs_penalty& sim, double h_)
{
	list<GCoordinate *>::iterator iter_pcoord;

	if ( !calc_dyna(sim) ) return false;

	for(int i=0; i<sim._cinfo.size(); i++)
	{
		for (iter_pcoord = sim._cinfo[i]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[i]->_system->pCoordinates.end(); iter_pcoord++) {
			(*iter_pcoord)->dq += (*iter_pcoord)->ddq * h_;
			(*iter_pcoord)->q += (*iter_pcoord)->dq * h_;		// semi-explicit integration: displacement is updated with the new velocity
		}
	}

	return true;
}

static void get_qp(DynamicsSimulator_gmbs_penalty& sim, RMatrix& qp)
{	
	int i=0;
	list<GCoordinate *>::iterator iter_pcoord;
	qp.ReSize(sim._cinfo[0]->_system->pCoordinates.size(),1);
	double * qp_=qp.GetPtr();

	for (iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); iter_pcoord++) {
		qp_[i++] = (*iter_pcoord)->q;
	}
}

static void get_dqp(DynamicsSimulator_gmbs_penalty& sim, RMatrix& dqp)
{
	int i=0;
	dqp.ReSize(sim._cinfo[0]->_system->pCoordinates.size(),1);
	double * dqp_=dqp.GetPtr();

	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); iter_pcoord++) {
		dqp_[i++] = (*iter_pcoord)->dq;
	}
}

static void get_ddqp(DynamicsSimulator_gmbs_penalty& sim, RMatrix& ddqp)
{
	int i=0;
	ddqp.ReSize(sim._cinfo[0]->_system->pCoordinates.size(),1);
	double * ddqp_=ddqp.GetPtr();

	list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); iter_pcoord++) {
		ddqp_[i++] = (*iter_pcoord)->ddq;
	}
}

static bool integrate_rk4(DynamicsSimulator_gmbs_penalty& sim, double h_)
{
	int i;
	list<GCoordinate *>::iterator iter_pcoord;
	double h = h_;
	double h2 = h_/2.;
	
	RMatrix qp_0 ;
	get_qp(sim, qp_0);
	RMatrix dqp_0 ;
	get_dqp(sim,dqp_0 );

	if ( !calc_dyna(sim) ) return false;
	RMatrix ddqp_1 ;
	get_ddqp(sim,ddqp_1 );

	for (i=0, iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); i++, iter_pcoord++) {
		(*iter_pcoord)->q = qp_0[i] + (*iter_pcoord)->dq * h2;		
		(*iter_pcoord)->dq = dqp_0[i] + (*iter_pcoord)->ddq * h2;
	}

	if ( !calc_dyna(sim) ) return false;
	RMatrix ddqp_2 ;
	get_ddqp(sim, ddqp_2);

	for (i=0, iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); i++, iter_pcoord++) {
		(*iter_pcoord)->q = qp_0[i] + (*iter_pcoord)->dq * h2;		
		(*iter_pcoord)->dq = dqp_0[i] + (*iter_pcoord)->ddq * h2;
	}

	if ( !calc_dyna(sim) ) return false;
	RMatrix ddqp_3 ;
	get_ddqp(sim, ddqp_3);

	for (i=0, iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); i++, iter_pcoord++) {
		(*iter_pcoord)->q = qp_0[i] + (*iter_pcoord)->dq * h;		
		(*iter_pcoord)->dq = dqp_0[i] + (*iter_pcoord)->ddq * h;
	}

	if ( !calc_dyna(sim) ) return false;
	RMatrix ddqp_4 ;
	get_ddqp(sim, ddqp_4);

	RMatrix ddqp = (h/6.) * (ddqp_1 + 2 * (ddqp_2 + ddqp_3) + ddqp_4);

	for (i=0, iter_pcoord = sim._cinfo[0]->_system->pCoordinates.begin(); iter_pcoord != sim._cinfo[0]->_system->pCoordinates.end(); i++, iter_pcoord++) {
		(*iter_pcoord)->q = qp_0[i] + (*iter_pcoord)->dq * h;	
		(*iter_pcoord)->dq = dqp_0[i] + ddqp[0] * h;
	}

	return true;
}




bool DynamicsSimulator_gmbs_penalty::stepSimulation()
{
	_cinfo[0]->_system->initExternalForce();

	// set external forces
	for(int i=0; i<_contactForces.size(); i++)
	{		
		ContactForce& c=_contactForces[i];
		//		addForceToBone(c.chara, c.bone,c.p,c.f);
		_addForceToLink(*this, c.chara, c.bone, c.p, c.f, c.tau);
	}

	for(int i=0; i<_externalForces.size(); i++)
	{
		if(_externalForces[i].chara!=0) continue;
		VRMLTransform* b=_externalForces[i].bone;
		int body=b->HRPjointIndex(b->numHRPjoints()-1);
		vector3 localpos=_externalForces[i].p;
		GBodyRigid* link=_cinfo[0]->_links[body]->body;
		link->addExternalForceGlobally(toGMBS(localpos), toGMBS(_externalForces[i].f));
	}

	
	//if (_integrateOpt == OpenHRP::DynamicsSimulator::EULER)
		integrate_me(*this,_timeStep);
	//else
	//	integrate_rk4(*this,_timeStep); not working

	_currTime+=_timeStep;

	for(int i=0; i<_cinfo.size(); i++)
		_cinfo[i]->_system->updateKinematics();
	_updateCharacterPose();
	
	int n = _characters.size();

		
	
#if defined(DEBUG_gmbs_WRAP) // && defined(_DEBUG) 

	VRMLloader* l=_characters[0]->skeleton;

	// test state retrival.
	for(int i=1; i<l->numBone(); i++)
	{
		VRMLTransform& b=l->VRMLbone(i);
		int body=b.HRPjointIndex(b.numHRPjoints()-1);
		const SE3 const& mat=_cinfo[ichara]->_links[body]->body->T_global;
		Vec3 v=mat.GetPosition();
//		printf("%s %f %f %f\n", b.name().ptr(), v[0], v[1], v[2]);
	}

	// test state retrival.
	for(int i=1; i<l->numBone(); i++)
	{
		VRMLTransform& b=l->VRMLbone(i);

		int body0=b.HRPjointIndex(0);
		int body=b.HRPjointIndex(b.numHRPjoints()-1);
		const SE3 const& mat=_cinfo[ichara]->_links[body]->body->T_global;
		
		
		matrix3 mat1;

		/*if (i!=1) 
		{
			SE3 lmat;

			for (int k=0; k<b.numHRPjoints(); k++)
				lmat*=_links[0][b.HRPjointIndex(k)]->joint->m_m_sRelativeFrame ;

			// compare local orientations
			mat1.setFromQuaternion(getWorldState(0).local(b).rotation);
			for(int ii=0; ii<3; ii++)
				for(int jj=0; jj<3; jj++)
					ASSERT(isSimilar(mat1.m[ii][jj], lmat(ii,jj)));
		}*/


		// compare global orientations
		mat1.setFromQuaternion(getWorldState(0).global(b).rotation);

		for(int ii=0; ii<3; ii++)
			for(int jj=0; jj<3; jj++)
				Msg::verify(isSimilar(mat1.m[ii][jj], mat(ii,jj),0.1), "global orientation error");


		// compare global joint positions:
		vector3 com=b.mSegment?b.mSegment->centerOfMass:vector3(0,0,0);
		vector3 jointPos=getWorldState(0).global(b).toGlobalPos(vector3(0,0,0));
		
		Vec3 jointPos2=mat.GetPosition();
		//ASSERT_SIMILAR(jointPos, jointPos2);

		TString nodeName;
		nodeName.format("jointPos%d", i);
		RE::moveEntity(RE::createEntity(nodeName, "sphere1010.mesh"), vector3(3,3,3), toBase(jointPos2)*100);
//		RE::moveEntity(RE::createEntity(nodeName, "sphere1010.mesh"), vector3(3,3,3), jointPos*100);
		
		

		//
		//// compare local center of mass positions
		//vector3 bodyToJoint;
		//_SDgetbtj(body, bodyToJoint);
	
		//vector3 com=b.mSegment?b.mSegment->centerOfMass:vector3(0,0,0);
		//vector3 parentcom=
		//	(((VRMLTransform*)(b.parent()))->mSegment)?
		//	((VRMLTransform*)(b.parent()))->mSegment->centerOfMass:vector3(0,0,0);

		//ASSERT_SIMILAR(bodyToJoint, com*-1);
		//// compare local joint positions
		//vector3 inboardToJoint, inboardToJoint2;
		//if(i!=1)
		//{
		//	b.getOffset(inboardToJoint);
		//	inboardToJoint-=parentcom;

		//	_SDgetitj(body0, inboardToJoint2);

		//	ASSERT_SIMILAR(inboardToJoint, inboardToJoint2);
		//}

		//// compare global joint positions
		//vector3 jointPos=getWorldState(0).global(b).translation;
		//vector3 jointPos2;
		//_SDpos(body, bodyToJoint, jointPos2);

		//ASSERT_SIMILAR(jointPos, jointPos2);

		//// compare global COM positions
		//vector3 centerOfMass=getWorldState(0).global(b).toGlobalPos(com);
		//vector3 centerOfMass2;
		//_SDpos(body, vector3(0,0,0), centerOfMass2);
		//
		//ASSERT_SIMILAR(centerOfMass, centerOfMass2);
	}

#endif


	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();
	_calcContactForce(*collisions);

	_externalForces.clear();

	return true;
}



static void ID(DynamicsSimulator_gmbs_penalty& sim, vectorn const& desiredacceleration, vectorn& controlforce)
{
	sim.setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, desiredacceleration);
	// sim._system->initExternalForce();
	int ichara=0;
	sim._cinfo[ichara]->_system->update_joint_local_info();
	sim._cinfo[ichara]->_system->fsFwdRecursion_a();
	sim._cinfo[ichara]->_system->fsBwdRecursion_b();
	sim._cinfo[ichara]->_system->fsFwdRecursion_c();
	//sim._system->updateGlobalLocationsOfBodiesAndJoints();
	sim.getLinkData(0, DynamicsSimulator::JOINT_TORQUE, controlforce);
	// vectorn theta,dtheta;
	// sim.getLinkData(0, DynamicsSimulator::JOINT_VALUE, theta);
	// sim.getLinkData(0, DynamicsSimulator::JOINT_VELOCITY, dtheta);
	// printf("%s\n%s\n:thetadtheta\n", theta.output().ptr(), dtheta.output().ptr());
	// printf("cf: %s\n", controlforce.output().ptr());
}
inline void packTau(vectorn& out, vectorn const& in)
{
	out.setSize(in.size()-1);
	out.range(0,3).assign(in.range(4,7));
	out.range(3,6).assign(in.range(0,3));
	out.range(6,out.size()).assign(in.range(7, in.size()));
}
void DynamicsSimulator_gmbs_penalty::calcMassMatrix(int ichara, matrixn& M, vectorn& b)
{
	/* differs from calcMassMatrix !!! all dw, dv, tauext, fext are w.s.p body local coordinate!
 		  |       |   | dw   |   |    |   | tauext    |
		  | out_M | * | dv   | + | b1 | = | fext      |
		  |       |   |ddq   |   |    |   | u         |
	*/
	
	_cinfo[ichara]->_system->initExternalForce();
	VRMLloader& l=*(_characters[0]->skeleton);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;

	int sj=l.VRMLbone(1).mJoint->jointStartId;
	int ej=l.VRMLbone(l.numBone()-1).mJoint->jointEndId;
	int numActualDOF=l.dofInfo.numActualDOF();
	vectorn desiredacceleration(numActualDOF+1);
	vectorn controlforce(numActualDOF+1);
	M.setSize(numActualDOF, numActualDOF);
	desiredacceleration.setAllValue(0);
	
	vectorn controlforce_backup;
	getLinkData(0, JOINT_TORQUE, controlforce_backup);
	for (int j=sj;j<ej;j++)
		cinfo[j]->joint->setPrescribed(true);

	for (int i=0; i<3; i++){
		desiredacceleration[i+4]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i+4]=0;
		packTau(M.column(i).lval(), controlforce);
	}
	for (int i=3; i<6; i++){
		desiredacceleration[i-3]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i-3]=0;
		packTau(M.column(i).lval(), controlforce);
	}
	for(int i=6; i<numActualDOF; i++){
		desiredacceleration[i+1]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i+1]=0;
		packTau(M.column(i).lval(), controlforce);
	}
	// printf("%s\n", desiredacceleration.output().ptr());
	ID(*this, desiredacceleration, controlforce);
	packTau(b, controlforce);

	for (int i=0; i<numActualDOF; i++)
		M.column(i)-=b;
	for (int j=sj;j<ej;j++)
		cinfo[j]->joint->setPrescribed(false);
	setLinkData(0, JOINT_TORQUE, controlforce_backup);
}

void DynamicsSimulator_gmbs_penalty::setGVector
(
 const ::vector3& wdata
 )
{
	int ichara=0;
	_cinfo[ichara]->_system->setGravity(toGMBS(wdata*-1));

	if(debugMode){
		cout << "DynamicsSimulator_gmbs_penalty::setGVector("
			 << wdata[0] << ", "
			 << wdata[1] << ", "
			 << wdata[2] << ")" << endl;
	}
}


void DynamicsSimulator_gmbs_penalty::getGVector
(
 ::vector3& wdata
 )
{
	int ichara=0;
	Vec3 g=_cinfo[ichara]->_system->getGravity();
	(wdata)[0] = g[0];
	(wdata)[1] = g[1];
	(wdata)[2] = g[2];

	if(debugMode){
		cout << "DynamicsSimulator_gmbs_penalty::getGVector(";
		cout << wdata[0] << ", "
			 << wdata[1] << ", "
			 << wdata[2] << ")" << endl;
	}
}


void DynamicsSimulator_gmbs_penalty::setCharacterAllJointModes
(
 const char * characterName,
 OpenHRP::DynamicsSimulator::JointDriveMode jointMode
 )
{
	//bool isHighGainMode = (jointMode == OpenHRP::DynamicsSimulator::HIGH_GAIN_MODE);

	//BodyPtr body = _world.body(characterName);

	//for(int i=1; i < body->numLinks(); ++i){
	//	body->link(i)->isHighGainMode = isHighGainMode;
	//}

	//if(debugMode){
	//	cout << "DynamicsSimulator_gmbs_penalty::setCharacterAllJointModes(";
	//	cout << characterName << ", ";
	//	cout << (isHighGainMode ? "HIGH_GAIN_MODE" : "TORQUE_MODE");
	//	cout << ")" << endl;
	//}
}




void DynamicsSimulator_gmbs_getJacobian_coordIndex(GBody* pbody, intvectorn& out);
static matrixn rotate(quater const& q, matrixn const& J)
{
	matrixn out(J.rows(), J.cols());
	for (int i=0; i<J.rows(); i+=3){
		for(int j=0; j<J.cols(); j++){
			// out.range(i, i+3, j, j+3).mult(R, J.range(i,i+3,j,j+3));
			::vector3 v= J.column(j).toVector3(i);
			v.rotate(q);
			out.column(j).setVec3(i, v);
		}
	}
	return out;
}
void DynamicsSimulator_gmbs_penalty::calcDotJacobian(int ichara, int ibone, matrixn& dotjacobian)
{
	_calcDotJacobian(ichara,ibone,dotjacobian,true);
}
void DynamicsSimulator_gmbs_penalty::calcJacobian(int ichara, int ibone, matrixn& jacobian)
{
	_calcJacobian(ichara,ibone,jacobian,true);
}

void DynamicsSimulator_gmbs_penalty::_calcDotBodyJacobian(int ichara, int ibone, matrixn& jacobian, matrixn& dotjacobian, bool update)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;
	VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
	GBodyRigid* pbody=getBody(_cinfo[ichara]->_links, &b);
	JOINTLOOP_CONSTRAINT_TYPE jlc_type;
	SE3 M;
	if(update){
		// save previous settings
		M = pbody->fJL.M1;
		jlc_type = pbody->fJL.jointLoopConstraintType;
		// calc Jacobian
		pbody->fJL.setM(SE3());
		pbody->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		pbody->fJL.update_J();
		pbody->fJL.update_dJdt();
	}
	matrixn dPdQ, dot_dPdQ;

	RMat_assign(dPdQ, pbody->fJL.jacobian);
	RMat_assign(dot_dPdQ, pbody->fJL.dotjacobian);

	if(update){
		// restore the previous settings
		pbody->fJL.setM(M);
		pbody->fJL.setJointLoopConstraintType(jlc_type);
	}

	intvectorn &coordIndex=_cinfo[ichara]->DQindexToCoordIndex;
	intvectorn &dofIndex=_cinfo[ichara]->coordIndexToDQindex;
	dotjacobian.setSize(6, l.dofInfo.numActualDOF());
	jacobian.setSize(6, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);
	jacobian.setAllValue(0);
	intvectorn coordIndex_i;
	DynamicsSimulator_gmbs_getJacobian_coordIndex(pbody, coordIndex_i);
	assert(dPdQ.cols()==coordIndex_i.size());

	// HingeJoint :
	// J=dPdQ 
	// dotJ=dot_dPdQ 
	for (int i=0; i<coordIndex_i.size(); i++){
		int idx=dofIndex(coordIndex_i(i));
		// printf("%d==%d\n", idx,coordIndex_i(i));
		if(idx!=-1){
			jacobian.column(idx).assign(dPdQ.column(i));
			dotjacobian.column(idx).assign(dot_dPdQ.column(i));
		}
	}

	matrixn invS, dot_invS;

	// S*invS=I 
	// dotS*invS+S*dot_invS=0
	// dot_invS=-invS*dotS*invS
	{
		GJoint* j=_cinfo[ichara]->_links[0]->joint;
		RMatrix invs=Inv(j->S);
		RMat_assign(invS, invs);
		matrixn dotS;
		RMat_assign(dotS, j->dS);
		dot_invS.mult(invS, dotS*invS);
		dot_invS*=-1;
	}

	// FreeJoint :
	// J=dPdQ * invS 
	// dotJ= dot_dPdQ *invS +dpDQ*dot_invS
	jacobian.range(0,jacobian.rows(),0,6).assign( dPdQ.range(0,6,0,6)*invS);
	dotjacobian.range(0,dotjacobian.rows(),0,6).assign( dot_dPdQ.range(0,6,0,6)*invS+dPdQ.range(0,6,0,6)*dot_invS);
}

void DynamicsSimulator_gmbs_penalty::_calcDotJacobian(int ichara, int ibone, matrixn& dotjacobian, bool update)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;
	VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
	GBodyRigid* pbody=getBody(_cinfo[ichara]->_links, &b);
	JOINTLOOP_CONSTRAINT_TYPE jlc_type;
	SE3 M;
	if(update){
		// save previous settings
		M = pbody->fJL.M1;
		jlc_type = pbody->fJL.jointLoopConstraintType;
		// calc Jacobian
		pbody->fJL.setM(SE3());
		pbody->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		pbody->fJL.update_J();
		pbody->fJL.update_dJdt();
	}
	matrixn dPdQ, dot_dPdQ;

	RMat_assign(dPdQ, pbody->fJL.jacobian);
	RMat_assign(dot_dPdQ, pbody->fJL.dotjacobian);

	if(update){
		// restore the previous settings
		pbody->fJL.setM(M);
		pbody->fJL.setJointLoopConstraintType(jlc_type);
	}

	quater r=getWorldState(0).global(b).rotation;

	SO3 R=pbody->T_global.GetRotation();

	// W=invR*dotR
	// dotR=RW
	SO3 dotR=R*skew(pbody->V.GetW());

	intvectorn &coordIndex=_cinfo[ichara]->DQindexToCoordIndex;
	intvectorn &dofIndex=_cinfo[ichara]->coordIndexToDQindex;
	dotjacobian.setSize(dPdQ.rows(), l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);
	intvectorn coordIndex_i;
	DynamicsSimulator_gmbs_getJacobian_coordIndex(pbody, coordIndex_i);

	assert(dPdQ.cols()==coordIndex_i.size());
	// HingeJoint :
	// J=R *dPdQ 
	// dotJ=dotR *dpdQ + R*dot_dPdQ 
	for (int i=0; i<coordIndex_i.size(); i++){
		int idx=dofIndex(coordIndex_i(i));
		// printf("%d==%d\n", idx,coordIndex_i(i));
		if(idx!=-1){
			Vec3 dw=dotR*toVec3(dPdQ.column(i), 0)+R*toVec3(dot_dPdQ.column(i),0);
			Vec3 dv=dotR*toVec3(dPdQ.column(i), 3)+R*toVec3(dot_dPdQ.column(i),3);
			dotjacobian.column(idx).setVec3(0, toBase(dw));
			dotjacobian.column(idx).setVec3(3, toBase(dv));
		}
	}

	matrixn invS, dot_invS;

	// S*invS=I 
	// dotS*invS+S*dot_invS=0
	// dot_invS=-invS*dotS*invS
	{
		GJoint* j=_cinfo[ichara]->_links[0]->joint;
		RMatrix invs=Inv(j->S);
		RMat_assign(invS, invs);
		matrixn dotS;
		RMat_assign(dotS, j->dS);
		dot_invS.mult(invS, dotS*invS);
		dot_invS*=-1;
	}

	// FreeJoint :
	// J=R *dPdQ * invS 
	// dotJ=dotR *dpdQ*invS + R*dot_dPdQ *invS +R*dpDQ*dot_invS
	// where dotR=R*skew(wo), 


	matrixn temp(dotjacobian.rows(), 6);
	for(int i=0; i<6; i++){
		Vec3 dw=dotR*toVec3(dPdQ.column(i), 0)+R*toVec3(dot_dPdQ.column(i),0);
		Vec3 dv=dotR*toVec3(dPdQ.column(i), 3)+R*toVec3(dot_dPdQ.column(i),3);
		temp.column(i).setVec3(0, toBase(dw));
		temp.column(i).setVec3(3, toBase(dv));
		dotjacobian.column(i).setVec3(0, toBase(R*toVec3(dPdQ.column(i),0)));
		dotjacobian.column(i).setVec3(3, toBase(R*toVec3(dPdQ.column(i),3)));
	}

	dotjacobian.range(0,dotjacobian.rows(),0,6).assign( temp*invS+dotjacobian.range(0,dotjacobian.rows(),0,6)*dot_invS);
}

void DynamicsSimulator_gmbs_penalty::_calcJacobian(int ichara, int ibone, matrixn& jacobian,bool update)
{
	// printf("%d\n", ibone);
	matrixn out;
	VRMLloader const& l=*_characters[ichara]->skeleton;
	VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
	GBodyRigid* pbody=getBody(_cinfo[ichara]->_links, &b);
	// cout<< pbody->pBaseJoint->getName()<<"\n";

	JOINTLOOP_CONSTRAINT_TYPE jlc_type;
	SE3 M;
	if(update){
		// save previous settings
		M = pbody->fJL.M1;
		jlc_type = pbody->fJL.jointLoopConstraintType;

		// calc Jacobian
		pbody->fJL.setM(SE3());
		pbody->fJL.jointLoopConstraintType = JOINTLOOP_ORIENTATION_POSITION;
		pbody->fJL.update_J();
	}

	RMat_assign(out, pbody->fJL.jacobian);
	// printf("out %s\n", out.output().ptr());

#ifdef _USE_BASELIB
	// J dq= bodyvelocity
	quater r=getWorldState(0).global(b).rotation;
	// R J dq= worldvelocity where R= ( r 0; 0 r)
	for(int i=0; i<out.cols(); i++)
	{
		::vector3 w=out.column(i).toVector3(0);
		::vector3 v=out.column(i).toVector3(3);

		w.rotate(r);
		v.rotate(r);
		out.column(i).setVec3(0, w);
		out.column(i).setVec3(3, v);
	}
#else
	SO3 r=pbody->T_global.GetRotation();
	for(int i=0; i<out.cols(); i++)
	{
		Vec3 w=toVec3(out.column(i),0);
		Vec3 v=toVec3(out.column(i),3);

		out.column(i).setVec3(0, toBase(r*w));
		out.column(i).setVec3(3, toBase(r*v));
	}
#endif
	if (update){

		// restore the previous settings
		pbody->fJL.setM(M);
		pbody->fJL.setJointLoopConstraintType(jlc_type);
	}

	intvectorn &coordIndex=_cinfo[ichara]->DQindexToCoordIndex;
	intvectorn &dofIndex=_cinfo[ichara]->coordIndexToDQindex;

	jacobian.setSize(out.rows(), l.dofInfo.numActualDOF());
	assert(l.dofInfo.numActualDOF()==coordIndex.size());
	
	jacobian.setAllValue(0);

	matrixn& DpDq=out;
	//-- dpdq is actually is not a full-matrix. e.g. upperbody columns are all 0 when adjusting the foot joint. 
	//-- gmbs doesn't output those zeros, but this function always output full-matrix even when it is sparse.
	intvectorn coordIndex_i;
	DynamicsSimulator_gmbs_getJacobian_coordIndex(pbody, coordIndex_i);

	// printf("coordIndex_i %s %s\n", coordIndex_i.output().ptr(), _cinfo[ichara].coordIndexToDOFindex.output().ptr()); 
	for(int i=0; i<coordIndex_i.size();i++)
	{
		int idx=dofIndex(coordIndex_i(i));// -- i: columnIndex, idx: dofIndex
		//printf("%d %d\n", idx, _cinfo[ichara].coordIndexToDOFindex[coordIndex_i(i)] );
		if( idx!=-1 )
			jacobian.column(idx).assign(DpDq.column(i));
	}

	//		-- now root joint has to be specially handled
	//		-- In gmbs, body velocity dq = S * dtheta
	//		-- where dtheta[0:3] is euler angle vel in body coord
	//		--       dtheta[3:6] is linear vel in body coord
	//		-- in our case dq[0:3] : angular displacement in world coord
	//		--             dq[3:6] : linear displacement in world coord
	//
	//		-- given x = J* dtheta, we want to find J_new such that x=J_new* dq
	//		-- -> J*dtheta=J*invS*dq
	//		-- -> J_new=J*invS

	matrixn invS;
	{
		//GBodyRigid* j=_links[ichara][0]->body;
		GJoint* j=_cinfo[ichara]->_links[0]->joint;
		RMatrix invs=Inv(j->S);
		RMat_assign(invS, invs);
	}

	matrixn J_new;
	J_new.mult(DpDq.range(0,DpDq.rows(), 0,6),invS);

	jacobian.range(0,jacobian.rows(),0,6).assign( J_new.range(0,J_new.rows(),0,6));

	// if(ibone==1) {
	// 	// cout<<jacobian.output().ptr()<<"\n";
	// 	jacobian.range(0, jacobian.rows(), 7, jacobian.cols()).setAllValue(0);
	// }
}
void DynamicsSimulator_gmbs_penalty::calcCOMjacobian(int ichar, matrixn& jacobian)
{
	if (0){
		RMatrix DpDq;
		const std::vector<BodyPart*>& cinfo=_cinfo[ichar]->_links;
		/*
		  _system->updateJacobian();
		  _system->calcDerivative_PositionCOMGlobal_Dq_2(DpDq);
		*/
		_cinfo[ichar]->_system->calcDerivative_PositionCOMGlobal_Dq(DpDq);
		RMat_assign(jacobian, DpDq);
		matrixn invS;
		{
			//GBodyRigid* j=cinfo[0]->body;
			GJoint* j=cinfo[0]->joint;
			RMatrix invs=Inv(j->S);
			RMat_assign(invS, invs);
		}
		// printf("%d %d\n", jacobian.rows(), jacobian.cols());

		jacobian.range(0,jacobian.rows(), 0,6).assign(jacobian.range(0, jacobian.rows(), 0,6)* invS);
	}
	else {
		VRMLloader const& l=*_characters[ichar]->skeleton;
		matrixn j,dj;
		jacobian.setSize(3,l.dofInfo.numActualDOF());
		jacobian.setAllValue(0);
		double totalMass=0;
		for(int i=1; i<l.numBone(); i++) {
			VRMLTransform& b=(VRMLTransform&)l.bone(i);
			int sDQ=l.dofInfo.startDQ(i);
			int eDQ=l.dofInfo.endDQ(i);
			calcJacobian(ichar, i, j);
			
			double mass=b.mass();
			totalMass+=mass;
			
			matrix3 R,invR,skew_C;
			R.setFromQuaternion(getWorldState(0).global(b).rotation);
			invR.setFromQuaternion(getWorldState(0).global(b).rotation.inverse());
			//printf("localcom:%s %s\n", b.localCOM().output().ptr(), toBase(getBody(_links[0],&b)->getPositionCOM()).output().ptr());
			skew_C.setTilde(b.localCOM()*-1);
			matrix3 Rskew_CinvR;
			Rskew_CinvR.mult(R, skew_C*invR);
			for (int k=0; k<j.cols(); k++){
				::vector3 w=j.column(k).toVector3(0);
				::vector3 v=j.column(k).toVector3(3);
				
				// body jacobian to COM global jacobian
				//    Ad(R)*Ad(c^-1)
				//     (R    0)  (   I      0   )= (R    0     )
				//     (0    R)  ( skew(-c) I   )  (Rskew(-c) R)
				//   = (   I           0   )(R    0)
				//	   (R skew(-c)invR I   )(0    R)
				//   = (   I           0   )(w)
				//     ( skew(-Rc)     I   )(v)

				
				Rskew_CinvR.rotate(w);
				//jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+mass*( w+v));
				vector3 localpos=b.localCOM();
				jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+mass*( w.cross(R*localpos)+v));
			}
		}
		// printf("totalMass: %f %f\n", _system->getMass(), totalMass);
		jacobian*=1.0/_cinfo[ichar]->_system->getMass();
	}
}
void DynamicsSimulator_gmbs_penalty::calcBoneDotJacobian2(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	matrixn j,dj;
	jacobian.setSize(6,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(6, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);
	{
		int i=ibone;
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		int sDQ=l.dofInfo.startDQ(i);
		int eDQ=l.dofInfo.endDQ(i);
		calcDotJacobian(ichar, i, dj);
		_calcJacobian(ichar, i, j,false); // no update needed
			
		matrix3 R,invR,skew_C, skewLw;
		quater r=getWorldState(0).global(b).rotation;
		R.setFromQuaternion(r);
		invR.setFromQuaternion(r.inverse());
		skew_C.setTilde(localpos*-1);
		matrix3 Rskew_CinvR;
		Rskew_CinvR.mult(R, skew_C*invR);
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
		matrix3 RskewLwSkewCinvR;
		RskewLwSkewCinvR.mult(R, skewLw*skew_C*invR);
		matrix3 Rskew_CinvRskewLwinvR;
		Rskew_CinvRskewLwinvR.mult(Rskew_CinvR,skewLw*invR);
		for (int k=0; k<j.cols(); k++){
			::vector3 w=j.column(k).toVector3(0);
			::vector3 v=j.column(k).toVector3(3);
				
			// body jacobian to COM global jacobian
			//    Ad(R)*Ad(c^-1)
			//     (R    0)  (   I      0   )(lw)= (R    0     )(lw)
			//     (0    R)  ( skew(-c) I   )(lv)  (Rskew(-c) R)(lv)
			//   = (   I           0   )(R    0)(lw)
			//	   (R skew(-c)invR I   )(0    R)(lv)
			//   = (   I           0   )(w)
			//     (R skew(-c)invR I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+( w));
			jacobian.column(k).setVec3(3, jacobian.column(k).toVector3(3)+( Rskew_CinvR*w+v));

			// dot jacobian:
			// d(Rskew(-c)invR )/dt *w +Rskew(-c)invR*dw + dv
			// d~/dt=dotR skew(-c)invR *w+ R skew(-c) dinvR/dt*w+ Rskew_CinvR*dw + dv
			//      = R skew(lw) skew(-c) invR*w - R skew(-c)*invR*skew(lw)*invR*w + Rskew_CinvR*dw + dv
			
			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+(dw));
			dotjacobian.column(k).setVec3(3, dotjacobian.column(k).toVector3(3)+(RskewLwSkewCinvR*w -Rskew_CinvRskewLwinvR*w+Rskew_CinvR*dw + dv));
		}
	}
}
void DynamicsSimulator_gmbs_penalty::calcBoneDotJacobian(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	matrixn j,dj;
	jacobian.setSize(3,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(3, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);
	{
		int i=ibone;
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		int sDQ=l.dofInfo.startDQ(i);
		int eDQ=l.dofInfo.endDQ(i);
		calcDotJacobian(ichar, i, dj);
		_calcJacobian(ichar, i, j,false); // no update needed
			
		matrix3 R,invR,skew_C, skewLw;
		quater r=getWorldState(0).global(b).rotation;
		R.setFromQuaternion(r);
		invR.setFromQuaternion(r.inverse());
		skew_C.setTilde(localpos*-1);
		matrix3 Rskew_CinvR;
		Rskew_CinvR.mult(R, skew_C*invR);
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
		matrix3 RskewLwSkewCinvR;
		RskewLwSkewCinvR.mult(R, skewLw*skew_C*invR);
		matrix3 Rskew_CinvRskewLwinvR;
		Rskew_CinvRskewLwinvR.mult(Rskew_CinvR,skewLw*invR);
		for (int k=0; k<j.cols(); k++){
			::vector3 w=j.column(k).toVector3(0);
			::vector3 v=j.column(k).toVector3(3);
				
			// body jacobian to COM global jacobian
			//    Ad(R)*Ad(c^-1)
			//     (R    0)  (   I      0   )(lw)= (R    0     )(lw)
			//     (0    R)  ( skew(-c) I   )(lv)  (Rskew(-c) R)(lv)
			//   = (   I           0   )(R    0)(lw)
			//	   (R skew(-c)invR I   )(0    R)(lv)
			//   = (   I           0   )(w)
			//     (R skew(-c)invR I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+( Rskew_CinvR*w+v));

			// dot jacobian:
			// d(Rskew(-c)invR )/dt *w +Rskew(-c)invR*dw + dv
			// d~/dt=dotR skew(-c)invR *w+ R skew(-c) dinvR/dt*w+ Rskew_CinvR*dw + dv
			//      = R skew(lw) skew(-c) invR*w - R skew(-c)*invR*skew(lw)*invR*w + Rskew_CinvR*dw + dv
			
			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+(RskewLwSkewCinvR*w -Rskew_CinvRskewLwinvR*w+Rskew_CinvR*dw + dv));
		}
	}
}
void DynamicsSimulator_gmbs_penalty::calcCOMdotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	matrixn j,dj;
	jacobian.setSize(3,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(3, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);
	double totalMass=0;
	for(int i=1; i<l.numBone(); i++) {
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		int sDQ=l.dofInfo.startDQ(i);
		int eDQ=l.dofInfo.endDQ(i);
		calcDotJacobian(ichar, i, dj);
		_calcJacobian(ichar, i, j,false); // no update needed
			
		double mass=b.mass();
		totalMass+=mass;
			
		matrix3 R,invR,skew_C, skewLw;
		quater r=getWorldState(0).global(b).rotation;
		R.setFromQuaternion(r);
		invR.setFromQuaternion(r.inverse());
		//printf("localcom:%s %s\n", b.localCOM().output().ptr(), toBase(getBody(_links[0],&b)->getPositionCOM()).output().ptr());
		skew_C.setTilde(b.localCOM()*-1);
		matrix3 Rskew_CinvR;
		Rskew_CinvR.mult(R, skew_C*invR);
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
		matrix3 RskewLwSkewCinvR;
		RskewLwSkewCinvR.mult(R, skewLw*skew_C*invR);
		matrix3 Rskew_CskewLwinvR;
		Rskew_CskewLwinvR.mult(R*skew_C,skewLw*invR);
		for (int k=0; k<j.cols(); k++){
			::vector3 w=j.column(k).toVector3(0);
			::vector3 v=j.column(k).toVector3(3);
				
			// body jacobian to COM global jacobian
			//    Ad(R)*Ad(c^-1)
			//     (R    0)  (   I      0   )(lw)= (R    0     )(lw)
			//     (0    R)  ( skew(-c) I   )(lv)  (Rskew(-c) R)(lv)
			//   = (   I           0   )(R    0)(lw)
			//	   (R skew(-c)invR I   )(0    R)(lv)
			//   = (   I           0   )(w)
			//     (R skew(-c)invR I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+mass*( Rskew_CinvR*w+v));

			// dot jacobian:
			// d(Rskew(-c)invR )/dt *w +Rskew(-c)invR*dw + dv
			// d~/dt=dotR skew(-c)invR *w+ R skew(-c) dinvR/dt*w+ Rskew_CinvR*dw + dv
			//      = R skew(lw) skew(-c) invR*w - R skew(-c)*skew(lw)*invR*w + Rskew_CinvR*dw + dv
			
			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+mass*(RskewLwSkewCinvR*w -Rskew_CskewLwinvR*w+Rskew_CinvR*dw + dv));
		}
	}
	// printf("totalMass: %f %f\n", _system->getMass(), totalMass);
	jacobian*=1.0/_cinfo[ichar]->_system->getMass();
	dotjacobian*=1.0/_cinfo[ichar]->_system->getMass();
}
Liegroup::dse3 DynamicsSimulator_gmbs_penalty::calcMomentumCOM(int ichara)
{
	::dse3 v=_cinfo[ichara]->_system->getMomentumCOM();
	Liegroup::dse3 out;
	out=v.GetArray();

	//#define TEST_CALCMOMENTUM
#ifdef TEST_CALCMOMENTUM
	Liegroup::dse3 out2=AISTsim->calcMomentumCOM(ichara);	
	v=_cinfo[ichara]->_system->getMomentumGlobal();
	printf("calcMomentumGlb: %f %f %f %f %f %f\n", v[0], v[1], v[2], v[3],v[4],v[5]);
	printf("calcMomentumCOM: %f %f %f %f %f %f\n", out[0], out[1], out[2], out[3],out[4],out[5]);
	printf("calcMomentumCOM2:%f %f %f %f %f %f\n", out2[0], out2[1], out2[2], out2[3],out2[4],out2[5]);
#endif
	return out;

}
Liegroup::se3 transf_twist(transf const& tf1, transf const& tf2, double timestep);
//Liegroup::se3 transf_twist_nonlinear(transf const& tf1, transf const& tf2, double timestep);
void DynamicsSimulator_gmbs_penalty::setInertia(int ichara, int ibone, ::vector3 const& localCOM, ::vector3 const& inertia)
{
	GSystem* _system=_cinfo[ichara]->_system;
	VRMLloader* skel=_characters[ichara]->skeleton;
	VRMLTransform& bone=skel->VRMLbone(ibone);
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	BodyPart* link=cinfo[bone.HRPjointIndex(bone.numHRPjoints()-1)];
	GBodyRigid* body=link->body;

	SE3 T_com=SE3(SO3(), toGMBS(localCOM));

	bone.setLocalCOM(localCOM);
	bone.setInertia(inertia.x, inertia.y, inertia.z);
	body->setMass(bone.mass(), bone.mSegment->momentsOfInertia._11,
	bone.mSegment->momentsOfInertia._22,
	bone.mSegment->momentsOfInertia._33,
	bone.mSegment->momentsOfInertia._12,
	bone.mSegment->momentsOfInertia._13,
	bone.mSegment->momentsOfInertia._23,
	T_com);
	link->centerOfMass=localCOM;


}
void  DynamicsSimulator_gmbs_penalty::calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const
{
	GSystem* _system=_cinfo[ichara]->_system;
	VRMLloader* skel=_characters[ichara]->skeleton;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	BoneForwardKinematics chain(skel);
	chain.init();
	::Inertia I;
	::vector3 com(0,0,0);
	m_real totalMass=0.0;

	chain.setPoseDOF(pose);

	// T_(-COM)*T_G*T(lCOM)*local_position_WT_COM
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		GBodyRigid* body=getBody(cinfo,&skel->VRMLbone(ibone));
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;

		Inertia Ii=body->I.Transform(toGMBS(chain.global(bone).inverse())); // inverse is there because of dAd transformation
		//Ii=Ii.Transform(toGMBS(chain2.global(bone).inverse()));
		//printf("Ii: %f %f %f %f %f %f, %f\n", body->I._I[0],body->I._I[1],body->I._I[2],body->I._I[3],body->I._I[4],body->I._I[5],body->I._m);
		//printf("Ii: %f %f %f %f %f %f, %f\n", Ii._I[0],Ii._I[1],Ii._I[2],Ii._I[3],Ii._I[4],Ii._I[5],Ii._m);
		for (int i=0;i<6; i++) I._I[i]+=Ii._I[i];
		for (int i=0;i<3; i++) I._r[i]+=Ii._r[i];
		I._m+=Ii._m;
	}

	com/=totalMass;

	I=I.Transform(SE3(toGMBS(com))); // inv(T_{com*-1}) == T_com
	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	inertia.setValues(10, I._I[0], I._I[1], I._I[2],  I._I[3],I._I[4],I._I[5],I._m, I._r[0], I._r[1], I._r[2]);
}

// calc momentum assuming constant velocity while steadily transforming from poseFrom to poseTo in 1 second. (0 <= t <= 1)
Liegroup::dse3 DynamicsSimulator_gmbs_penalty::calcMomentumCOMfromPose(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
{
	GSystem* _system=_cinfo[ichara]->_system;
	VRMLloader* skel=_characters[ichara]->skeleton;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	BoneForwardKinematics chain1(skel), chain2(skel);
	chain1.init();
	chain2.init();

	::vector3 com(0,0,0);
	::dse3 H(0,0,0,0,0,0);

	m_real totalMass=0.0;
	chain1.setPoseDOF(poseFrom);
	chain2.setPoseDOF(poseTo);
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		GBodyRigid* body=getBody(cinfo,&skel->VRMLbone(ibone));
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain1.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
		//Liegroup::se3 V=transf_twist(chain1.global(bone), chain2.global(bone),delta_t);
		Liegroup::se3 V=Liegroup::twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		H+=dAd(toGMBS(chain1.global(bone).inverse()), body->I* (*( ::se3* )(&V)));

	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	::dse3 v=dAd(SE3(toGMBS(com)), H);
	Liegroup::dse3 out;
	out=v.GetArray();
	return out;
}

Liegroup::dse3 DynamicsSimulator_gmbs_penalty::calcMomentumCOMfromPoseUpper(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
{
	GSystem* _system=_cinfo[ichara]->_system;
	VRMLloader* skel=_characters[ichara]->skeleton;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	BoneForwardKinematics chain1(skel), chain2(skel);
	chain1.init();
	chain2.init();

	::vector3 com(0,0,0);
	::dse3 H(0,0,0,0,0,0);

	m_real totalMass=0.0;
	chain1.setPoseDOF(poseFrom);
	chain2.setPoseDOF(poseTo);
	//for(int ibone=1; ibone<skel->numBone(); ibone++)
	for(int ibone=1; ibone<15; ibone++)
	{
		GBodyRigid* body=getBody(cinfo,&skel->VRMLbone(ibone));
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain1.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
		//Liegroup::se3 V=transf_twist(chain1.global(bone), chain2.global(bone),delta_t);
		Liegroup::se3 V=Liegroup::twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		H+=dAd(toGMBS(chain1.global(bone).inverse()), body->I* (*( ::se3* )(&V)));

	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	::dse3 v=dAd(SE3(toGMBS(com)), H);
	Liegroup::dse3 out;
	out=v.GetArray();
	return out;
}

Liegroup::dse3 DynamicsSimulator_gmbs_penalty::calcMomentumCOMfromPoseLower(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
{
	GSystem* _system=_cinfo[ichara]->_system;
	VRMLloader* skel=_characters[ichara]->skeleton;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
	BoneForwardKinematics chain1(skel), chain2(skel);
	chain1.init();
	chain2.init();

	::vector3 com(0,0,0);
	::dse3 H(0,0,0,0,0,0);

	m_real totalMass=0.0;
	chain1.setPoseDOF(poseFrom);
	chain2.setPoseDOF(poseTo);
	for(int ibone=15; ibone<skel->numBone(); ibone++)
	{
		GBodyRigid* body=getBody(cinfo,&skel->VRMLbone(ibone));
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain1.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
		//Liegroup::se3 V=transf_twist(chain1.global(bone), chain2.global(bone),delta_t);
		Liegroup::se3 V=Liegroup::twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		H+=dAd(toGMBS(chain1.global(bone).inverse()), body->I* (*( ::se3* )(&V)));

	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	::dse3 v=dAd(SE3(toGMBS(com)), H);
	Liegroup::dse3 out;
	out=v.GetArray();
	return out;
}

::vector3 calcCOM2_gmbs(OpenHRP::DynamicsSimulator_gmbs_penalty&s, int ichara)
{
	return toBase(s._cinfo[ichara]->_system->getPositionCOMGlobal());
}
::vector3 DynamicsSimulator_gmbs_penalty::calculateCOM(int ichara, double& outputTotalMass)
{
#define USE_CORRECT_VERSION
#ifdef USE_CORRECT_VERSION // but may be inconsistent with some parts of my program...
	outputTotalMass=_cinfo[ichara]->_system->getMass();
	return toBase(_cinfo[ichara]->_system->getPositionCOMGlobal());
#else

	::vector3 t=DynamicsSimulator::calculateCOM(ichara, outputTotalMass);
//	printf("%f %f: %s %s\n", outputTotalMass, _cinfo[ichara]->_system->getMass(), t.output().ptr(), toBase(_cinfo[ichara]->_system->getPositionCOMGlobal()).output().ptr());
	return t;
#endif
}
void DynamicsSimulator_gmbs_penalty::calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	std::vector<BodyPart*>& cinfo=_cinfo[ichar]->_links;
	matrixn j,dj;
	jacobian.setSize(6,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(6, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);

	if(0)
	{
		// L = J dq	
		// dotL=  J ddq + dotJ dq
		RMatrix dLdq, dLq;
		//_cinfo[ichar]->_system->calcDerivative_MomentumCOM_Dq_Ddq(dLq, dLdq);
		_cinfo[ichar]->_system->calcDerivative_MomentumCOM_Ddq(dLdq);
		RMat_assign(jacobian, dLdq);
		matrixn invS;
		{
			//GBodyRigid* j=cinfo[0]->body;
			GJoint* j=cinfo[0]->joint;
			RMatrix invs=Inv(j->S);
			RMat_assign(invS, invs);
		}

		//		-- now root joint has to be specially handled
		//		-- In gmbs, body velocity dq = S * dtheta
		//		-- where dtheta[0:3] is euler angle vel in body coord
		//		--       dtheta[3:6] is linear vel in body coord
		//		-- in our case dq[0:3] : angular displacement in world coord
		//		--             dq[3:6] : linear displacement in world coord
		//
		//		-- given x = J* dtheta, we want to find J_new such that x=J_new* dq
		//		-- -> J*dtheta=J*invS*dq
		//		-- -> J_new=J*invS
		jacobian.range(0,jacobian.rows(), 0,6).assign(jacobian.range(0, jacobian.rows(), 0,6)* invS);
		//printf("%s\n", jacobian.output().ptr()); // tested o.k.
	}
	else
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
			_calcDotBodyJacobian(ichar,i,bJ,bdotJ,true);
			GBodyRigid* body=getBody(cinfo,&l.VRMLbone(i));
			SE3 invBodyT=Inv(body->T_global);
			matrix4 dotBodyT=calcDotT(body->T_global, body->V);
			// T * invT = I
			// dotT*invT+T*dotInvT=0
			// dot(invT)=-invT*dotT*invT
			SE3 T=invBodyT*SE3(toGMBS(COM));
			dAd(dAd_T, T);
			matrix4 dotCOM;
			zero(dotCOM);
			dotCOM.setTranslation(COMVEL);
			matrix4 invBodyT2=toBase(invBodyT);
			matrix4 dotT= invBodyT2*dotCOM- invBodyT2*dotBodyT*invBodyT2 * toBase(SE3(toGMBS(COM))); 
			dot_dAd(dot_dAd_T, T, dotT);
			for (int j=0; j<bJ.cols(); j++){
				se3 cj=to_se3(bJ.column(j));
				//dse3 cj2=dAd(T, body->I*cj);
				//radd(jacobian.column(j).lval(), cj2);
				dse3 temp=body->I*cj;
				radd(jacobian.column(j).lval(),mult( dAd_T,temp));

				radd(dotjacobian.column(j).lval(), 
						mult(dot_dAd_T,temp)+mult(dAd_T,body->I*to_se3(bdotJ.column(j))));
				// dAd(T)=
				//    (     R       0 )'
				//    (   skew(T)*R R )
				// cj2= dAd(T)*I*cj
			}
		}
	}
}

static void getOrientation(GJointSpherical* j, double* state, quater& q)
{
	Vec3 euler;

	euler[0]=state[0]; euler[1]=state[1]; euler[2]=state[2];

	switch(j->getCoordinateChart())	{
	case EULER_ZYX:
		iQuat(EulerZYX(euler), &q.w);
		break;
	case EULER_ZYZ:
		iQuat(EulerZYZ(euler), &q.w);
		break;
	case EULER_XYZ:
		iQuat(EulerXYZ(euler), &q.w);
		break;
	case EULER_ZXY:
		iQuat(EulerZXY(euler), &q.w);
		break;
	}
}	

static void calcEulerFromQuater(GJointSpherical* j, quater & q, Vec3& euler)
{
	switch(j->getCoordinateChart())	{
	case EULER_ZYX:
		euler=iEulerZYX(Quat(&q.w));
		break;
	case EULER_ZYZ:
		euler=iEulerZYZ(Quat(&q.w));
		break;
	case EULER_XYZ:
		euler=iEulerXYZ(Quat(&q.w));
		break;
	case EULER_ZXY:
		euler=iEulerZXY(Quat(&q.w));
		break;
	}
}


// output is compatible to MotionDOF class.
void DynamicsSimulator_gmbs_penalty::getLinkData(int ichara, LinkDataType t, vectorn& out)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;

	out.resize(l.dofInfo.numDOF());

	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;

	
	double imag;
	::vector3 v,p;
	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
			
		
		if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			GJointFreeC2* j=(GJointFreeC2*)cinfo[b.mJoint->jointStartId]->joint;

			double temp[6];
			ASSERT(l.dofInfo.hasTranslation(i));
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sTDOF=l.dofInfo.startT(i);
			int sRDOF=l.dofInfo.startR(i);
			ASSERT(sRDOF-sTDOF==3);


			switch(t)
			{
			case DynamicsSimulator::JOINT_VALUE:
				{
					quater q;

					j->get_q(temp);

					p.x=temp[0]; p.y=temp[1]; p.z=temp[2];
					getOrientation(&j->spherical_joint, &temp[3], q);

					imag=q.w;
					v.x=q.x; v.y=q.y; v.z=q.z;
				}
				break;
			case DynamicsSimulator::JOINT_VELOCITY:
				{

					Vec3 p2, v2;

#ifdef USE_GLOBAL_ANG_VEL
					assert(false);
					v2=j->spherical_joint.T.GetRotation()*j->Sdq.GetW();
					p2=j->spherical_joint.T.GetRotation()*j->Sdq.GetV();
					v=toBase(v2);
					p=toBase(p2);
#else
					v=toBase(j->Sdq.GetW());
					p=toBase(j->Sdq.GetV());
#endif


					/*
#if defined(_DEBUG) && defined(DEBUG_gmbs_WRAP)

					getWorldAngVel(0,&b, v);
					getWorldVelocity(0,&b, vector3(0,0,0), p);

					ASSERT_SIMILAR(p, p2);
					ASSERT_SIMILAR(v, v2);

#endif




#if defined(_DEBUG) && defined(DEBUG_gmbs_WRAP)

					RMatrix dq(6,1);
					j->get_dq(dq.GetPtr());
					RMatrix worldv=j->S*dq;


					struct RMat_Temp
					{
						static void print(RMatrix const& mat)
						{
							for(int i=0; i<mat.RowSize(); i++)
							{
								cout <<i <<" [";
								for(int j=0; j<mat.ColSize(); j++)
									cout << mat(i,j) << " ";
								cout <<" ]\n";
							}
							cout<<"]";

						}

						static void print(se3 const& mat)
						{
							for(int j=0; j<6; j++)
									cout << mat[j] << " \n";
						}
					};
					RMat_Temp::print(dq);
					
					RMatrix dq1(3,1);
					j->pJoint1->get_dq(dq1.GetPtr());
					RMat_Temp::print(j->pJoint1->S*dq1);
					RMat_Temp::print(j->pJoint1->Sdq);

					j->pJoint2->get_dq(dq1.GetPtr());
					RMat_Temp::print(j->pJoint2->S*dq1);
					RMat_Temp::print(j->pJoint2->Sdq);

					RMat_Temp::print(worldv);
					RMat_Temp::print(j->Sdq);


#endif
*/

				}
				break;
			case DynamicsSimulator::JOINT_ACCELERATION:
				{
					RMatrix ddq(6,1);
					RMatrix Sddq(6,1);
					j->get_ddq(ddq.GetPtr());
					imag=0.0;
					se3& dotSdq=j->get_dSdq();
					Sddq=j->S*ddq;
					v.x=Sddq(0,0)+dotSdq[0];
					v.y=Sddq(1,0)+dotSdq[1];
					v.z=Sddq(2,0)+dotSdq[2];
					p.x=Sddq(3,0)+dotSdq[3];
					p.y=Sddq(4,0)+dotSdq[4];
					p.z=Sddq(5,0)+dotSdq[5];
				}
				break;
			case DynamicsSimulator::JOINT_TORQUE:
				{
					/*
					j->get_tau(temp);
					imag=0.0;
					v.x=temp[3]; v.y=temp[4]; v.z=temp[5];
					p.x=temp[0]; p.y=temp[1]; p.z=temp[2];

					matrix3 jacobian;
					getMat3(jacobian, j->spherical_joint.S);
					jacobian.rotate(v);// to spatial 
					*/
					GBodyRigid* body=(GBodyRigid*)(j->pRightBody);
					v.x=body->F[0];
					v.y=body->F[1];
					v.z=body->F[2];
					p.x=body->F[3];
					p.y=body->F[4];
					p.z=body->F[5];

				}
				break;
			default:
				ASSERT(0);
			}

			
			out[sTDOF]=p.x;
			out[sTDOF+1]=p.y;
			out[sTDOF+2]=p.z;
			out[sRDOF]=imag;
			out[sRDOF+1]=v.x;
			out[sRDOF+2]=v.y;
			out[sRDOF+3]=v.z;

		}
		else if(b.mJoint->jointType==HRP_JOINT::BALL)
		{			

			GJointSpherical* j=(GJointSpherical*)cinfo[b.mJoint->jointStartId]->joint;
			int sRDOF=l.dofInfo.startR(i);

			switch(t)
			{
			case DynamicsSimulator::JOINT_VALUE:
				{
					quater q;

					double temp[4];
					j->get_q(temp);
					getOrientation(j, temp, q);

					imag=q.w;
					v.x=q.x; v.y=q.y; v.z=q.z;
				}
				break;
			case DynamicsSimulator::JOINT_VELOCITY:
				{
					/*
					double temp[4];
					j->get_dq(temp);
					imag=0.0;
					v.x=temp[0]; v.y=temp[1]; v.z=temp[2];

					matrix3 jacobian;
					getMat3(jacobian, j->S);
					jacobian.rotate(v);// to spatial velocity
					*/
					Vec3 angVel;
#ifdef USE_GLOBAL_ANG_VEL
					angVel=j->T.GetRotation()*j->Sdq.GetW();
#else
					angVel=j->Sdq.GetW();
#endif
					v=toBase(angVel);
				}
				break;
			case DynamicsSimulator::JOINT_ACCELERATION:
				{
					// incorrect. need to add dotS dq
					double temp[4];
					j->get_ddq(temp);
					imag=0.0;
					v.x=temp[0]; v.y=temp[1]; v.z=temp[2];

					matrix3 jacobian;
					getMat3(jacobian, j->S);
					jacobian.rotate(v);// to twist 
				}
				break;
			case DynamicsSimulator::JOINT_TORQUE:
				{
					double temp[4];
					j->get_tau(temp);
					imag=0.0;
					v.x=temp[0]; v.y=temp[1]; v.z=temp[2];

					matrix3 jacobian;
					getMat3(jacobian, j->S);
					jacobian.rotate(v);// to spatial 
				}
				break;
			default:
				ASSERT(0);
			}

			
			out[sRDOF]=imag;
			out[sRDOF+1]=v.x;
			out[sRDOF+2]=v.y;
			out[sRDOF+3]=v.z;
		}
		else if(b.mJoint->jointType==HRP_JOINT::ROTATE)
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;

			

			for(int jj=0; jj<nDOF; jj++)
			{

				GJointRevolute* j=((GJointRevolute*)cinfo[jj+sj]->joint);
				switch(t)
				{
				case DynamicsSimulator::JOINT_VALUE:
					out[sDOF+jj]=j->coordinate.q;
					break;
				case DynamicsSimulator::JOINT_VELOCITY:
					out[sDOF+jj]=j->coordinate.dq;
					break;
				case DynamicsSimulator::JOINT_ACCELERATION:
					out[sDOF+jj]=j->coordinate.ddq;
					break;
				case DynamicsSimulator::JOINT_TORQUE:
					out[sDOF+jj]=j->coordinate.tau;
					break;

				default:
					ASSERT(0);
				}
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::SLIDE)
		{			
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;

			

			for(int jj=0; jj<nDOF; jj++)
			{

				GJointPrismatic* j=((GJointPrismatic*)cinfo[jj+sj]->joint);
				switch(t)
				{
				case DynamicsSimulator::JOINT_VALUE:
					out[sDOF+jj]=j->coordinate.q;
					break;
				case DynamicsSimulator::JOINT_VELOCITY:
					out[sDOF+jj]=j->coordinate.dq;
					break;
				case DynamicsSimulator::JOINT_ACCELERATION:
					out[sDOF+jj]=j->coordinate.ddq;
					break;
				case DynamicsSimulator::JOINT_TORQUE:
					out[sDOF+jj]=j->coordinate.tau;
					break;

				default:
					ASSERT(0);
				}
			}
		}
	}
}

// output is compatible to MotionDOF class.
void DynamicsSimulator_gmbs_penalty::setLinkData(int ichara, LinkDataType t, vectorn const& in)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;

	ASSERT(in.size()==l.dofInfo.numDOF());

	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;

	if (((VRMLTransform&)l.bone(1)).mJoint->jointType==HRP_JOINT::FREE){
	if (t==DynamicsSimulator::JOINT_VALUE)
	{
		// the execution path depends on the current value. 
		// to break such dependency, and to guarantee that the same input yields the same output,
		// invalidate spherical joint first.
		double zero[]={0,0,0,0,0,0};	
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		j->spherical_joint.coord_chart=0;
		j->set_q(zero);
		j->update_short();
	}
	else if(t==DynamicsSimulator::JOINT_VELOCITY)
	{
		double zero[]={0,0,0,0,0,0};	
		GJointFreeC2* j=(GJointFreeC2*)cinfo[0]->joint;
		j->set_dq(zero);
	}
	}
	double imag;
	::vector3 p,v;
	quater q;

	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);

		if(b.mJoint->jointType==HRP_JOINT::FIXED) continue;

		if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			GJointFreeC2* j=(GJointFreeC2* )cinfo[b.mJoint->jointStartId]->joint;

			ASSERT(l.dofInfo.hasTranslation(i));
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sTDOF=l.dofInfo.startT(i);
			int sRDOF=l.dofInfo.startR(i);
			ASSERT(sRDOF-sTDOF==3);
			p=in.toVector3(sTDOF);

			double temp[6];

			switch(t)
			{
			case DynamicsSimulator::JOINT_VALUE:
				{
					Vec3 euler;
					q=in.toQuater(sRDOF);
					
					calcEulerFromQuater(&j->spherical_joint, q, euler);

					temp[3]=euler[0];	temp[4]=euler[1];	temp[5]=euler[2];
					temp[0]=p.x;		temp[1]=p.y;		temp[2]=p.z;
					
					j->set_q(temp);
					j->update_short();
				}
				break;

			case DynamicsSimulator::JOINT_VELOCITY:
				{
					Vec3 W, V;
#ifdef USE_GLOBAL_ANG_VEL
					// body velocity
					// w=invR*wo
					// v=invR*vo
					// where [w,v]'=Sdq
					SO3 invT_global=j->spherical_joint.inv_T.GetRotation();
					
					W=invT_global*toGMBS(in.toVector3(sRDOF+1));
					V=invT_global*toGMBS(p);
#else
					W=toGMBS(in.toVector3(sRDOF+1));
					V=toGMBS(p);
					//printf("W,V=%s,%s\n", in.toVector3(sRDOF+1).output().ptr(), p.output().ptr());
#endif
					RMatrix Sdq(6,1);
					Sdq(0,0)=W[0];
					Sdq(1,0)=W[1];
					Sdq(2,0)=W[2];
					Sdq(3,0)=V[0];
					Sdq(4,0)=V[1];
					Sdq(5,0)=V[2];
					RMatrix invS=Inv(j->S);
					RMatrix dq=invS*Sdq;
					j->set_dq(dq);

				}
				break;
			case DynamicsSimulator::JOINT_ACCELERATION:
				{
#ifdef USE_GLOBAL_ANG_VEL
										// not implemented yet!
					// body velocity: v = (w,V)'
					// body acceleration: dv
					// world acceleration: dvo (not spatial acceleration)
					// vo=Rv --(1)
					// -> v=invR vo --(2)
					// derivate both side of (2)
					// dv=invR*dvo + dotinvR* vo
					// dv=invR*dvo + dotR'* vo
					// 	 =invR*dvo + (R*skew(wo))'*vo
					// 	 =invR*dvo - skew(wo)*invR*vo  --(3)
					//
					// verification: derivate both side of (1)
					// dv=invR(dvo - dotR v) 
					//   =invR(dvo - R*skew(wo)*v) 
					//   =invR*dvo - skew(wo)*invR*vo  --(5)
					//					// Of course, (3)==(4)
					//

					SO3 invR=j->spherical_joint.inv_T.GetRotation();
					SO3 R=j->spherical_joint.T.GetRotation();
					j->update(); //calculate dS, dSdq
					se3 vo=mult(R, j->Sdq);
					se3 dvo(toGMBS(v),toGMBS(p));
					
					se3 dv=mult(invR, dvo);
					dv-=mult(skew(vo.GetW())*invR, vo);
					// ddq=invS*dv - dotS_dq	
					RMatrix ddq=Inv(j->S)*toRMat(dv);
				   	ddq-=toRMat(j->dSdq);
#else
					Vec3 W,V;
					W=toGMBS(in.toVector3(sRDOF+1));
					V=toGMBS(p);
					// v=S*dq where dq is euler angle speeds
					// dv= dotS*dq+ S*ddq
					j->update(); //calculate dS, dSdq
					RMatrix Sddq(6,1);
					se3& dotSdq=j->get_dSdq();
					Sddq(0,0)=W[0]-dotSdq[0];
					Sddq(1,0)=W[1]-dotSdq[1];
					Sddq(2,0)=W[2]-dotSdq[2];
					Sddq(3,0)=V[0]-dotSdq[3];
					Sddq(4,0)=V[1]-dotSdq[4];
					Sddq(5,0)=V[2]-dotSdq[5];
					RMatrix invS=Inv(j->S);
					RMatrix ddq=invS*Sddq;
#endif
					j->set_ddq(ddq.element);
				}
				break;
			case DynamicsSimulator::JOINT_TORQUE:
				{
					v=in.toVector3(sRDOF+1);

					quater q;
					double temp[6];
					j->get_q(temp);
					getOrientation(&j->spherical_joint, &temp[3], q);
					
					//printf("%s\n", q.output().ptr());
					// convert to local force and torque
					p.rotate(q.inverse());
					v.rotate(q.inverse());

					// tau=Jt*f
					matrix3 jacobianInv;
					getMat3(jacobianInv, j->spherical_joint.S);
					//jacobianInv.inverse();
					jacobianInv.transpose();
					jacobianInv.rotate(v);
					jacobianInv.rotate(p);
					
					temp[3]=v.x;	temp[4]=v.y;	temp[5]=v.z;					
					temp[0]=p.x;	temp[1]=p.y;	temp[2]=p.z;
					j->set_tau(temp);
				}
				break;
			default:
				ASSERT(0);
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::BALL)
		{

			GJointSpherical* j=(GJointSpherical*)cinfo[b.mJoint->jointStartId]->joint;
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sRDOF=l.dofInfo.startR(i);
			imag=in[sRDOF];
			v=in.toVector3(sRDOF+1);

			double temp[3];
			switch(t)
			{
			case DynamicsSimulator::JOINT_VALUE:
				{
					Vec3 euler;
					q=in.toQuater(sRDOF);
					
					calcEulerFromQuater(j, q, euler);

					temp[0]=euler[0];	temp[1]=euler[1];	temp[2]=euler[2];

					j->set_q(temp);
					j->update_short();
				}
				break;
			case DynamicsSimulator::JOINT_VELOCITY:
				{
					Vec3 W;
#ifdef USE_GLOBAL_ANG_VEL
					SO3 invT_global=j->inv_T.GetRotation();
					W=invT_global*toGMBS(v);
#else
					W=toGMBS(v);
#endif
					RMatrix Sdq(3,1);
					Sdq(0,0)=W[0];
					Sdq(1,0)=W[1];
					Sdq(2,0)=W[2];
					RMatrix invS=Inv(j->S.Sub(0,2,0,2));
//					cout << "W"<<W<<"\n";
//					cout << "invS"<<invS<<"\n";
					RMatrix dq=invS*Sdq;
//					cout << "dq"<<dq<<"\n";
//					cout << j->S*dq<<Sdq<<"\n";
					j->set_dq(dq);
						/*
					matrix3 jacobianInv;
					getMat3(jacobianInv, j->S);
					jacobianInv.inverse();
					jacobianInv.rotate(v);
					temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					
					j->set_dq(temp);
					*/

				}
				break;
			case DynamicsSimulator::JOINT_ACCELERATION:
				{
					matrix3 jacobianInv;
					getMat3(jacobianInv, j->S);
					jacobianInv.inverse();
					jacobianInv.rotate(v);
					temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					

					j->set_ddq(temp);
				}
				break;
			case DynamicsSimulator::JOINT_TORQUE:
				{
					matrix3 jacobianInv;
					getMat3(jacobianInv, j->S);
					jacobianInv.inverse();
					jacobianInv.rotate(v);
					temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					

					j->set_tau(temp);
				}
				break;
			default:	
				ASSERT(0);
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::ROTATE)
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;

			for(int jj=0; jj<nDOF; jj++)
			{
				GJointRevolute* j=((GJointRevolute*)cinfo[jj+sj]->joint);
				switch(t)
				{
				case DynamicsSimulator::JOINT_VALUE:
					j->coordinate.q=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_VELOCITY:
					j->coordinate.dq=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_ACCELERATION:
					j->coordinate.ddq=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_TORQUE:
					j->coordinate.tau=in[sDOF+jj];
					break;

				default:
					ASSERT(0);
				}
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::SLIDE)
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;

			for(int jj=0; jj<nDOF; jj++)
			{
				GJointPrismatic* j=((GJointPrismatic*)cinfo[jj+sj]->joint);
				switch(t)
				{
				case DynamicsSimulator::JOINT_VALUE:
					j->coordinate.q=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_VELOCITY:
					j->coordinate.dq=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_ACCELERATION:
					j->coordinate.ddq=in[sDOF+jj];
					break;
				case DynamicsSimulator::JOINT_TORQUE:
					j->coordinate.tau=in[sDOF+jj];
					break;

				default:
					ASSERT(0);
				}
				
			}
		}
	}

	if(t!=DynamicsSimulator::JOINT_TORQUE){
		// update local jacobian
		std::list<GJoint*>::iterator iter_pjoint;
		for (iter_pjoint = _cinfo[ichara]->_system->pJoints.begin(); iter_pjoint != _cinfo[ichara]->_system->pJoints.end();	 iter_pjoint++) {
			if ( !((*iter_pjoint)->isConstantScrew()) ) (*iter_pjoint)->update_short();
		}
	}
	//		_system->updateKinematics();
}

//#include "ForwardDynamicsABM.h"
//void DynamicsSimulator_gmbs_penalty::forwardKinamaticsAcc()
//{
//	int ichara=0;
//
//	// calc link->vo, w, R, p, sw, sv, v, cv, cw, wc, ...
//	((ForwardDynamicsABM*)_world.forwardDynamics(ichara).get())->calcABMPhase1();
//
//	// calc link->dvo, dw
//	OpenHRP::BodyPtr body=_world.body(ichara);
//	const LinkTraverse& traverse = body->linkTraverse();
//	int n = traverse.numLinks();
//    for(int i=1; i < n; ++i){
//
//        Link* link = traverse[i];
//        Link* parent = link->parent;
//		
//        link->dvo = parent->dvo + link->cv + link->sv * link->ddq;
//        link->dw  = parent->dw  + link->cw + link->sw * link->ddq;
//    }
//
//	// update taesoo's data structure
//	_updateCharacterPose();
//}

double DynamicsSimulator_gmbs_penalty::currentTime() const
{
	return _currTime	;
}


void DynamicsSimulator_gmbs_penalty::inverseDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, vectorn& controlforce)
{
	setLinkData(0, JOINT_VALUE, theta);
	setLinkData(0, JOINT_VELOCITY, dtheta);
	setLinkData(0, JOINT_ACCELERATION, desiredacceleration);
	int ichara=0;
	_cinfo[ichara]->_system->updateKinematics();

	VRMLloader& l=*(_characters[0]->skeleton);
	VRMLTransform& b=l.VRMLbone(1);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;

	int sj=l.VRMLbone(1).mJoint->jointStartId;
	int ej=l.VRMLbone(l.numBone()-1).mJoint->jointEndId;

	for (int j=sj;j<ej;j++)
		cinfo[j]->joint->setPrescribed(true);

	_updateCharacterPose();
	_cinfo[ichara]->_system->initExternalForce();
	_cinfo[ichara]->_system->calcDynamics(true);
	getLinkData(0, JOINT_TORQUE, controlforce);
}


void DynamicsSimulator_gmbs_penalty::hybridDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce)
{
	BEGIN_TIMER(setLinkD);
	setLinkData(0, JOINT_VALUE, theta);
	setLinkData(0, JOINT_VELOCITY, dtheta);
	setLinkData(0, JOINT_ACCELERATION, desiredacceleration);
	_cinfo[0]->_system->updateKinematics();

	VRMLloader& l=*(_characters[0]->skeleton);
	VRMLTransform& b=l.VRMLbone(1);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;

	int sj=l.VRMLbone(2).mJoint->jointStartId;
	int ej=l.VRMLbone(l.numBone()-1).mJoint->jointEndId;

#ifdef PROFILE
	static int c=0;
	if (c++==300) { c=0; printf("dof %d %d\n",theta.size(), ej);}
#endif

	cinfo[0]->joint->setPrescribed(false);
		
	for (int j=sj;j<ej;j++)
		cinfo[j]->joint->setPrescribed(true);

	END_TIMER(setLinkD);
	
	BEGIN_TIMER(taesooFK);
	_updateCharacterPose();
	END_TIMER(taesooFK);

	BEGIN_TIMER(initExternalForce);
	_cinfo[0]->_system->initExternalForce();

	for(int i=0; i<contacts.size(); i++){
		ContactForce const& cq=contacts[i];
		if(cq.chara==0) {
			_addForceToLink(*this, 0, cq.bone, cq.p, cq.f, cq.tau);
		}
	}

	END_TIMER(initExternalForce);
	BEGIN_TIMER(calcDynamics);
	_cinfo[0]->_system->calcDynamics(true);

	// _system->update_joint_local_info();

	// BEGIN_TIMER(fwd_a);
	// _system->fsFwdRecursion_a(); 
	// END_TIMER(fwd_a);//11ms -> 5ms after minimal code optimization.
	// BEGIN_TIMER(fwd_b);
	// _system->fsBwdRecursion_b(); 
	// END_TIMER(fwd_b);//8ms
	// BEGIN_TIMER(fwd_c);
	// _system->fsFwdRecursion_c(); 
	// END_TIMER(fwd_c);//7ms

	// _system->update_joint_global_info(); 

	END_TIMER(calcDynamics);//26ms
		
	getLinkData(0, JOINT_TORQUE, controlforce);
}

void DynamicsSimulator_gmbs_penalty::hybridDynamics2(bitvectorn const& isActuated, vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce)
{
	setLinkData(0, JOINT_VALUE, theta);
	setLinkData(0, JOINT_VELOCITY, dtheta);
	setLinkData(0, JOINT_ACCELERATION, desiredacceleration);
	_cinfo[0]->_system->updateKinematics();

	VRMLloader& l=*(_characters[0]->skeleton);
	VRMLTransform& b=l.VRMLbone(1);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[0]->_links;

	for (int i=1;i< l.numBone(); i++)
	{
		int sj=l.VRMLbone(i).mJoint->jointStartId;
		int ej=l.VRMLbone(i).mJoint->jointEndId;

		for (int j=sj; j<ej; j++)
			cinfo[j]->joint->setPrescribed(isActuated[i]);
	}
		
	END_TIMER(setLinkD);
	
	BEGIN_TIMER(taesooFK);
	_updateCharacterPose();
	END_TIMER(taesooFK);

	BEGIN_TIMER(initExternalForce);
	_cinfo[0]->_system->initExternalForce();

	for(int i=0; i<contacts.size(); i++){
		ContactForce const& cq=contacts[i];
		if(cq.chara==0) {
			_addForceToLink(*this, 0, cq.bone, cq.p, cq.f, cq.tau);
		}
	}

	END_TIMER(initExternalForce);
	BEGIN_TIMER(calcDynamics);
	_cinfo[0]->_system->calcDynamics(true);

	// _system->update_joint_local_info();

	// BEGIN_TIMER(fwd_a);
	// _system->fsFwdRecursion_a(); 
	// END_TIMER(fwd_a);//11ms -> 5ms after minimal code optimization.
	// BEGIN_TIMER(fwd_b);
	// _system->fsBwdRecursion_b(); 
	// END_TIMER(fwd_b);//8ms
	// BEGIN_TIMER(fwd_c);
	// _system->fsFwdRecursion_c(); 
	// END_TIMER(fwd_c);//7ms

	// _system->update_joint_global_info(); 

	END_TIMER(calcDynamics);//26ms
		
	getLinkData(0, JOINT_TORQUE, controlforce);
}
