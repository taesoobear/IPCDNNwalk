// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include "../physicsLib.h"
#include <vector>
#include <map>
#include <algorithm>

#include "DynamicsSimulator_gmbs.h"
//#include "../AIST_implementation/DynamicsSimulator_impl.h"
//#include "../AIST_implementation/ForwardDynamics.h"
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
#include "../Liegroup.inl"

#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/math/Operator_NR.h"
#include "../convexhull/graham.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
//#define DEBUG_gmbs_WRAP

inline void ASSERT_SIMILAR(::vector3 const& x, Vec3 const& y)
{
	ASSERT(isSimilar(x.x, y[0], 0.001));
	ASSERT(isSimilar(x.y, y[1], 0.001));
	ASSERT(isSimilar(x.z, y[2], 0.001));
}


static bool useLocalExtForceInMassMat=true;
template <class TYPE>
static void getMat3(matrix3 & out, const TYPE& in)
{	for(int i=0;i<3;i++)
		for(int j=0; j<3; j++)
			out(i,j)=in(i,j);
}

 
#ifdef USE_AIST_SIM
inline Vec3 toGMBS(OpenHRP::vector3 const& v)
{
	return Vec3 (v(0), v(1), v(2));
}
#endif
void zero(matrix4 & out);
void RMat_assign(RMatrix & out, matrixn const& in);
void RMat_assign(matrixn & out, RMatrix const& in);
void RMat_print(RMatrix const & in);
void RMat_assign(matrixn & out, SO3 const& in);

/*
inline OpenHRP::matrix33 toOpenHRP(quater const& q )
{
	matrix4 out;
	out.setRotation(q);
	return toOpenHRP(out);
}
*/

 //#define PROFILE
#ifdef PROFILE
#include "../BaseLib/utility/QPerformanceTimer.h"
//#undef BEGIN_TIMER
//#undef END_TIMER
//#define DEFINE_TIMER(x) QPerformanceTimerCount x(300, #x)
//#define BEGIN_TIMER(x) x.start()
//#define END_TIMER(x) x.stop()
#define DEFINE_TIMER(x) 
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
(DynamicsSimulator_gmbs& sim, BodyPart* body, int index, LinkInfoSequence const& iLinks, const matrix3& parentRs)
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
DynamicsSimulator_gmbs::System::System()
{
	_system=new GSystem();
	_ground=new BodyPart("ground");
}
DynamicsSimulator_gmbs::System::~System()
{

	for(int i=0; i<_links.size(); i++)
		delete(_links[i]);

	delete _system;
	delete _ground;
}

DynamicsSimulator_gmbs::DynamicsSimulator_gmbs(const char* coldettype)
:DynamicsSimulator(coldettype),
DynamicsSimulator_QP(this)
{
#ifdef USE_AIST_SIM
	AISTsim=new DynamicsSimulator_impl();
#endif
}
DynamicsSimulator_gmbs::DynamicsSimulator_gmbs(bool useSimpleColdet)
:DynamicsSimulator((useSimpleColdet)?"SIMPLE":"BULLET"),
DynamicsSimulator_QP(this)
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs::DynamicsSimulator_gmbs()" << endl;
	}

	//AISTsim=new DynamicsSimulator_impl();
}


DynamicsSimulator_gmbs::~DynamicsSimulator_gmbs()
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs::~DynamicsSimulator_gmbs()" << endl;
	}
//	delete _G;


	for (int i=0; i<_cinfo.size(); i++)
		delete _cinfo[i];	
	//delete AISTsim;
}

static void setExactStateSpherical(DynamicsSimulator_gmbs& sim, int ichara, int coord_chart, double* q, double* dq, double* ddq=NULL)
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
void DynamicsSimulator_gmbs::test(const char* test, matrixn& out)
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


void DynamicsSimulator_gmbs::registerCollisionCheckPair
(
 const char *charName1,
 const char *linkName1,
 const char *charName2,
 const char *linkName2,
 vectorn const& param
 )
{
	double staticFriction=param[0];
	double slipFriction=param[1];
	LinkPair linkPair ;
	linkPair.charName1  = charName1;
	linkPair.linkName1 = linkName1;
	linkPair.charName2  =charName2;
	linkPair.linkName2 = linkName2;
	linkPair.param=param;
	collisionDetector->addCollisionPair(linkPair, false, false);
	//AISTsim->registerCollisionCheckPair(charName1, linkName1, charName2, linkName2, param);
}
void DynamicsSimulator_gmbs::setCollisionMargin(int ilinkpair, double a)
{
	collisionDetector->setMargin(ilinkpair, a);
}
void DynamicsSimulator_gmbs::setAllCollisionMargin(vectorn const& a)
{
	collisionDetector->setMarginAll(a);
}
void DynamicsSimulator_gmbs::getAllCollisionMargin(vectorn & a)
{
	collisionDetector->getMarginAll(a);
}
void DynamicsSimulator_gmbs::System::updateDOFindexMap(GSystem* _system, std::vector<BodyPart*>const & cinfo, VRMLloader const& l )
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

void DynamicsSimulator_gmbs::_registerCharacter
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

void DynamicsSimulator_gmbs::setTimestep(double timeStep)
{
	_timeStep=timeStep;
	//AISTsim->setTimestep(timeStep);
}

void DynamicsSimulator_gmbs::init(
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
	//AISTsim->init(timeStep, integrateOpt);
}

extern bool _debug_DynamicsSimulatorMethod_calcContactForce;

void DynamicsSimulator_gmbs::initSimulation()
{
	if(debugMode){
		cout << "DynamicsSimulator_gmbs::initSimulation()" << endl;
	}
	//_world->Initialize();
	for(int i=0; i<_cinfo.size(); i++)
		_cinfo[i]->_system->updateKinematics();

	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();
	_calcContactForce(*collisions);

#ifdef USE_AIST_SIM
	AISTsim->initSimulation();
	getLinkData(0, JOINT_VELOCITY, _tempVel);
	AISTsim->setLinkData(0, JOINT_VALUE, _tempPose);
	AISTsim->setLinkData(0, JOINT_VELOCITY, _tempVel);
	AISTsim->world.bodyInfoArray[0].forwardDynamics->initialize(); 
	AISTsim->world.contactForceSolver.clearExternalForces(); // already done above
	AISTsim->world.contactForceSolver.solve(*collisions);
#endif
	_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
}

::vector3 DynamicsSimulator_gmbs::calculateCOMvel(int ichara, double& outputTotalMass)
{
	return toBase(_cinfo[ichara]->_system->getVelocityCOMGlobal());
}

::vector3 DynamicsSimulator_gmbs::calculateCOMacc(int ichara) 
{
	return toBase(_cinfo[ichara]->_system->getAccelerationCOMGlobal());
}
inline GBodyRigid* getBody(const std::vector<BodyPart*>& cinfo, VRMLTransform* b)
{
	return cinfo[b->HRPjointIndex(b->numHRPjoints()-1)]->body;
}
double DynamicsSimulator_gmbs::calcKineticEnergy() const
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
void DynamicsSimulator_gmbs::getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& _localpos
			, ::vector3& velocity) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	velocity =toBase(link->getVelocityLinearGlobal(toGMBS(_localpos)));
}	

void DynamicsSimulator_gmbs::getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	angvel=toBase(link->getVelocityAngularGlobal());
} 

void DynamicsSimulator_gmbs::getBodyVelocity(int ichara, VRMLTransform* b, Liegroup::se3& V) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	V.W()=toBase(link->V.GetW());
	V.V()=toBase(link->V.GetV());
}

void DynamicsSimulator_gmbs::getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& _localpos
			, ::vector3& acc) const
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	acc=toBase(link->getAccelerationLinearGlobal(toGMBS(_localpos)));
}

void DynamicsSimulator_gmbs::getWorldAngAcc(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);
	angvel=toBase(link->getAccelerationAngularGlobal());
} 

void DynamicsSimulator_gmbs::addForceToBone
(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force)
{
	GBodyRigid* link=getBody(_cinfo[ichara]->_links,b);

	//// VP uses inertial frame for external forces.
	::vector3 gf=getWorldState(ichara)._global(*b).toGlobalDir(force);
	::vector3 gp=getWorldState(ichara)._global(*b).toGlobalPos(localpos);
	
	link->addExternalForceGlobally(dse3(toGMBS(gp.cross(gf)), toGMBS(gf)));

}

static void _addForceToLink(DynamicsSimulator_gmbs& sim, int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force, ::vector3 const& tau)
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

//#include "../AIST_implementation/ConstraintForceSolver_impl.h"
//#include "../AIST_implementation/ForwardDynamicsABM.h"
//#include "../AIST_implementation/AIST_common.h"			

#include "../BaseLib/math/Operator.h"



void DynamicsSimulator_gmbs::calcContactJacobianAll(matrixn &J_all, matrixn & dotJ_all, matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double frictionCoef) {
	ConstrainedPoints* solver=_contacts;
	std::vector<ConstrainedPoints::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
	int n=clinks.size();
	vectorn vFe(6);
	
	// contact-lcp.xoj page 3
	int a=solver->globalNumConstraintVectors+solver->globalNumFrictionVectors;
	


	matrixn V_temp;
	matrixn dotV_temp;
	{
		int ichara=0;
		int N=_characters[ichara]->skeleton->dofInfo.numActualDOF(); // excluding unused 4 th dimension.

		J_all.setSize(6*link_pair_count, N);
		dotJ_all.setSize(6*link_pair_count, N);
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
			ConstrainedPoints::ConstrainedLinkPair* linkPair=&clinks[i];
			for (int ipair=0; ipair<2; ++ipair){
				VRMLTransform* link=linkPair->linkPair->link[ipair];
				if (link->HRPjointType(0)!=HRP_JOINT::FIXED){
					std::vector<ConstrainedPoints::ConstraintPoint>& constraintPoints = linkPair->constraintPoints;
					int numConstraintPoints = constraintPoints.size();
					if (numConstraintPoints>0)
						{
							int ibody=linkPair->linkPair->charIndex[ipair];
							int ibone=link->treeIndex();
							// printf("ijoint: %d %d\n", ijoint, ibody);
							std::vector<BodyPart*>& cinfo=_cinfo[ibody]->_links;
							matrixnView J_i=J_all.range( c_pair*6, (c_pair+1)*6,0, N);
							matrixnView dotJ_i=dotJ_all.range( c_pair*6, (c_pair+1)*6,0, N);
							calcJacobian(ibody,ibone, J_i);
							calcDotJacobian(ibody,ibone, dotJ_i);
							{
								// calc V_i
								V_temp.setAllValue(0);
								dotV_temp.setAllValue(0);
								// J*dq : global velocity (!= spatial velocity)
								// invR*J*dq : body velocity (where invR= (inv_r 0; 0 inv_r))
								// Ad(T_global)*invR*J*dq : spatial velocity measured at T_global (contact point)
								// -> dual space
								// J'*R*dAd(T_global) * global_contact_force =  joint torque
								//
								// or more intuitively,
								// R*dAd(T_global) = dAd(T_global.GetPosition())
								// so just transforming contact position while preserving global orientation.
								//
								// Note:
								// R is to revert rotation made in calcJacobian (convert to local jacobian)
								// e.g. J'*R = (R'*J)'

								// RMat_assign(R, cinfo[ijoint]->body->T_global.GetRotation());

								//dse3 Fe=dAd(cinfo[ijoint]->body->T_global, dse3(toGMBS(cross(constraint.point, f)),toGMBS(f)));
								for(int k=0; k < numConstraintPoints; ++k){
									ConstrainedPoints::ConstraintPoint& constraint = constraintPoints[k];
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
										//  printf("f%d: %s\n", j, n.output().ptr());
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
								GBodyRigid* body=getBody(cinfo, &_characters[ibody]->skeleton->VRMLbone(ibone));
								RMat_assign(R_dAd.range(0,3,3,6).lval(),skew(body->T_global.GetPosition()*-1));



								matrixnView V_i=v_all.range(c_pair*6, (c_pair+1)*6,0,a);
								V_i.mult(R_dAd, V_temp);

								// dotV_i =R_dAd*dotV_temp + dotR_dAd*V_temp
								RMat_assign(dot_R_dAd.range(0,3,3,6).lval(), skew(body->getVelocityLinearGlobal()*-1));
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


void DynamicsSimulator_gmbs::getLCPmatrix(matrixn& A, vectorn& b)
{

#ifdef USE_AIST_SIM
	CFSImpl_LCP* solver=(CFSImpl_LCP*)AISTsim->world.contactForceSolver.impl;
	toBase(A, solver->Mlcp);
	toBase(b, solver->b);
#endif
}

void DynamicsSimulator_gmbs::getLCPsolution(vectorn& out)
{
#ifdef USE_AIST_SIM
	CFSImpl_LCP* solver=(CFSImpl_LCP*)AISTsim->world.contactForceSolver.impl;
	toBase(out, solver->solution);
#endif
}

// ys start
void DynamicsSimulator_gmbs::stepKinematicMuscle_integrate(vectorn const& ddq, double dt)
{
	vectorn ddq2;
	ddq2.setSize(ddq.size()+1);
	ddq2.setVec3(4, ddq.toVector3(0));//toBase(dw.GetW()));
	ddq2.setVec3(0, ddq.toVector3(3));//toBase(dw.GetV()));
	ddq2.range(7,ddq2.size())=ddq.range(6,ddq.size());
	setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, ddq2);

	int ichara=0;
	list<GCoordinate *>::iterator iter_pcoord;
	
	for (iter_pcoord = _cinfo[ichara]->_system->pCoordinates.begin(); iter_pcoord != _cinfo[ichara]->_system->pCoordinates.end(); iter_pcoord++) {
		(*iter_pcoord)->dq += (*iter_pcoord)->ddq * dt;
		(*iter_pcoord)->q += (*iter_pcoord)->dq * dt;		// semi-explicit integration: displacement is updated with the new velocity
	}
	_currTime+=dt;
	_cinfo[ichara]->_system->updateKinematics();
	_updateCharacterPose();
}

void DynamicsSimulator_gmbs::stepKinematicMuscle_updateother()
{
	int ichara=0;
		collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
		_contactForces.clear();
		_calcContactForce(*collisions);

#ifdef USE_AIST_SIM
		getLinkData(0, JOINT_VELOCITY, _tempVel);
		AISTsim->setLinkData(0, JOINT_VALUE, _tempPose);
		AISTsim->setLinkData(0, JOINT_VELOCITY, _tempVel);
		AISTsim->world.bodyInfoArray[0].forwardDynamics->initialize(); 
		AISTsim->world.contactForceSolver.clearExternalForces(); // already done above
		AISTsim->world.contactForceSolver.solve(*collisions);
#endif
		_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
	
		_cinfo[ichara]->_system->initExternalForce();
}
// ys end

bool DynamicsSimulator_gmbs::stepKinematic(vectorn const& ddq, vectorn const& tau, bool integrate)
{
	vectorn ddq2;
	ddq2.setSize(ddq.size()+1);
	ddq2.setVec3(4, ddq.toVector3(0));//toBase(dw.GetW()));
	ddq2.setVec3(0, ddq.toVector3(3));//toBase(dw.GetV()));
	ddq2.range(7,ddq2.size())=ddq.range(6,ddq.size());
	setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, ddq2);

	int ichara=0;
	if(integrate)
	{
		list<GCoordinate *>::iterator iter_pcoord;
		
		for (iter_pcoord = _cinfo[ichara]->_system->pCoordinates.begin(); iter_pcoord != _cinfo[ichara]->_system->pCoordinates.end(); iter_pcoord++) {
			(*iter_pcoord)->dq += (*iter_pcoord)->ddq * _timeStep;
			(*iter_pcoord)->q += (*iter_pcoord)->dq * _timeStep;		// semi-explicit integration: displacement is updated with the new velocity
		}
		_currTime+=_timeStep;
		_cinfo[ichara]->_system->updateKinematics();
		_updateCharacterPose();
		int n = _characters.size();

		collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
		_contactForces.clear();
		_calcContactForce(*collisions);

#ifdef USE_AIST_SIM
		getLinkData(0, JOINT_VELOCITY, _tempVel);
		AISTsim->setLinkData(0, JOINT_VALUE, _tempPose);
		AISTsim->setLinkData(0, JOINT_VELOCITY, _tempVel);
		AISTsim->world.bodyInfoArray[0].forwardDynamics->initialize(); 
		AISTsim->world.contactForceSolver.clearExternalForces(); // already done above
		AISTsim->world.contactForceSolver.solve(*collisions);
#endif
		_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
	
		_cinfo[ichara]->_system->initExternalForce();
	}
	else {
		_cinfo[ichara]->_system->updateKinematics();
		_updateCharacterPose();
	}
	

	return true;
}
static bool isZero(dse3 const& v)
{
	bool isZero=true;
	for (int i=0; i<6; i++)
		if(v[i]!=0.0) {
				isZero=false;
				break;
		}
	return  isZero;
}
void DynamicsSimulator_gmbs::collectExternalForce(int ichara, vectorn & extForce)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;
	vectorn vFe(6);
	matrixn J,temp;
	extForce.setSize(l.dofInfo.numActualDOF());
	extForce.setAllValue(0);
	for(int ibone=1,nbone=l.numBone(); ibone<nbone; ++ibone){ // 
		VRMLTransform& b=(VRMLTransform&)l.bone(ibone);
		GBodyRigid* pbody=getBody(_cinfo[ichara]->_links, &b);
		dse3& Fe=pbody->Fe;
		if(!isZero(Fe))
		{
			_calcJacobian(ichara,ibone, J, true);
			dse3 AdR_Fe=mult(pbody->T_global.GetRotation(), Fe);
			vFe.setValues(Fe.GetArray());
			temp.mult(vFe.row(), J);
			extForce+=temp.row(0);
		}	
	}
}
bool DynamicsSimulator_gmbs::stepSimulation()
{
	// AIST: 0. LCP solve
	//       1. calcABMLastHalf()
	//       2. integration
	//       3. calcABMFirstHalf()
	//       4. body->setVirtualJointForces()
	//       5. collisionDetection

	// UT:   0. LCP solve
	//       1. chain->update(timeStep)
	//       2. integrate
	//       3. CalcPosition, Velocity

	bool useMassMatrix=false;
	bool useAISTmassMat=false;
	if(useMassMatrix){
		// use mass matrix
		matrixn m,b;
		b.setSize(_characters[0]->skeleton->dofInfo.numActualDOF(),1);

#ifdef USE_AIST_SIM
		if (useAISTmassMat)
			AISTsim->calcMassMatrix2(0, m,b); // AISTsim->calcMassMatrix2 is different from this->calcMassMatrix2.
		else
#endif
			calcMassMatrix3(0, m,b.column(0).lval()); // this->calcMassMatrix2 and 3 is the same but 2 is much much faster.
		
		// m ddq + b = tau
		vectorn tau(b.rows());
		tau.setAllValue(0);
		#define USE_ALL_MATRIX
#ifdef USE_ALL_MATRIX // the most difficult to understand but necessary step for implementing QP
		assert(useAISTmassMat==false);
		if (1)
		{
			// calc tau
						// printf("calc tau\n");
			ConstrainedPoints* solver=(ConstrainedPoints*)_contacts;
			std::vector<ConstrainedPoints::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
			int n=clinks.size();
			vectorn vFe(6);
			matrixn J, temp;
	
			// contact-lcp.xoj page 3
			int link_pair_count=getNumContactLinkPairs();
			if (link_pair_count>0){
				matrixn JtV;
				matrixn J,dotJ,V,dotV;
				calcContactJacobianAll(J,dotJ,V, dotV, link_pair_count,0.5);
				JtV.multAtB(J,V);
				
				int a=solver->globalNumConstraintVectors+solver->globalNumFrictionVectors;
				matrixn solution(a,1);
				solution.setAllValue(0);
				/*
				for (int i=0; i<a; i++)
					solution(i,0)=solver->solution(i);
					*/
				tau=(JtV*solution).column(0);
			}
		}
		if (1) // enable setLinkData torque (usually output from PD-servo)
		{
			int ichara=0;
			VRMLloader const& l=*_characters[ichara]->skeleton;
			std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;
			for(int i=1; i<l.numBone(); i++) {
				VRMLTransform& b=(VRMLTransform&)l.bone(i);
				if(b.mJoint->jointType==HRP_JOINT::ROTATE) {
					int sj=b.mJoint->jointStartId;
					int sDOF=l.dofInfo.startT(i);
					int nDOF=l.dofInfo.endR(i)-sDOF;

					for(int jj=0; jj<nDOF; jj++) {
						GJointRevolute* j=((GJointRevolute*)cinfo[jj+sj]->joint);
						tau[sDOF+jj-1]+=j->coordinate.tau; //-1 is there to skip the unused 4th DOF.
						j->coordinate.tau=0;
					}
				}
			}
			// printf("tau: %s\n", tau.output().ptr());
		}
		

#else
		if (1)
		{
			// calc tau
			// printf("calc tau\n");
			ConstrainedPoints* solver=(ConstrainedPoints*)_contacts;
			std::vector<ConstrainedPoints::ConstrainedLinkPair>& clinks=solver->constrainedLinkPairs;
			int n=clinks.size();
			vectorn vFe(6);
			matrixn J, temp;
			for (int i=0; i<n ; ++i){
				CFSImpl_LCP::LinkPair* linkPair=clinks[i];
				for (int ipair=0; ipair<2; ++ipair){
					Link* link=linkPair->link[ipair];
					if (link->jointType!=Link::FIXED_JOINT){
						int ijoint=link->jointId+1;
						int ibody=linkPair->bodyIndex[ipair];
						// printf("ijoint: %d %d\n", ijoint, ibody);
						std::vector<BodyPart*>& cinfo=_links[ibody];
						calcJacobian(ibody,cinfo[ijoint]->boneId, J);
						#define USE_COLLECTED_RESULT
#ifdef USE_COLLECTED_RESULT  // the easiest to understand
						dse3 Fe=dAd(cinfo[ijoint]->body->T_global, dse3(toGMBS(link->tauext),toGMBS(link->fext)));
						Fe=mult(cinfo[ijoint]->body->T_global.GetRotation(), Fe);
						vFe.setValues(Fe.GetArray());
						// J*dq=dx	and J'*Fe=tau (dual relationship)
						temp.mult(vFe.row(), J);
						if(useAISTmassMat){
							//uses a different coordinate system and state packing.
							vectornView _tau=temp.row(0);
							dse3 rootTau(toGMBS(_tau.toVector3(0)),toGMBS(_tau.toVector3(3)));
							rootTau=mult(cinfo[0]->body->T.GetRotation(), rootTau);
							_tau.setVec3(3, toBase(GetM(rootTau)));
							_tau.setVec3(0, toBase(GetF(rootTau)));
							tau+=temp.row(0);
						}
						else if(!useLocalExtForceInMassMat){
							//uses a different coordinate system and state packing.
							vectornView _tau=temp.row(0);
							dse3 rootTau(toGMBS(_tau.toVector3(0)),toGMBS(_tau.toVector3(3)));
							rootTau=mult(cinfo[0]->body->T.GetRotation(), rootTau);
							_tau.setVec3(0, toBase(GetM(rootTau)));
							_tau.setVec3(3, toBase(GetF(rootTau)));
							tau+=temp.row(0);
						}
						else
							tau+=temp.row(0);
#else
						CFSImpl_LCP::ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
						int numConstraintPoints = constraintPoints.size();
						for(int i=0; i < numConstraintPoints; ++i){
							CFSImpl_LCP::ConstraintPoint& constraint = constraintPoints[i];
							int globalIndex = constraint.globalIndex;
							dvector& solution=solver->solution;
							OpenHRP::vector3 f(solution(globalIndex) * constraint.normalTowardInside[ipair]);

							for(int j=0; j < constraint.numFrictionVectors; ++j){
								f += solution(solver->globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector[j][ipair];
							}

							dse3 Fe=dAd(cinfo[ijoint]->body->T_global, dse3(toGMBS(cross(constraint.point, f)),toGMBS(f)));
							Fe=mult(cinfo[ijoint]->body->T_global.GetRotation(), Fe);
							vFe.setValues(Fe.GetArray());
							// J*dq=dx	and J'*Fe=tau (dual relationship)
							temp.mult(vFe.row(), J);
							vectornView _tau=temp.row(0);
							dse3 rootTau(toGMBS(_tau.toVector3(0)),toGMBS(_tau.toVector3(3)));
							rootTau=mult(cinfo[0]->body->T.GetRotation(), rootTau);
							_tau.setVec3(4, toBase(GetM(rootTau)));
							_tau.setVec3(0, toBase(GetF(rootTau)));
							tau+=temp.row(0);
						}
#endif
					}
				}
			}	
			// printf("calc tau finished\n");
		}
		if (1) // enable setLinkData torque (usually output from PD-servo)
		{
			int ichara=0;
			VRMLloader const& l=*_characters[ichara]->skeleton;
			std::vector<BodyPart*>& cinfo=_links[ichara];
			for(int i=1; i<l.numBone(); i++) {
				VRMLTransform& b=(VRMLTransform&)l.bone(i);
				if(b.mJoint->jointType==HRP_JOINT::ROTATE) {
					int sj=b.mJoint->jointStartId;
					// int sDOF=l.dofInfo.startT(i);
					// int nDOF=l.dofInfo.endR(i)-sDOF;
					int sDOF=l.dofInfo.startDQ(i);
					int nDOF=l.dofInfo.endDQ(i)-sDOF;

					for(int jj=0; jj<nDOF; jj++) {
						GJointRevolute* j=((GJointRevolute*)cinfo[jj+sj]->joint);
						tau[sDOF+jj]+=j->coordinate.tau;
						j->coordinate.tau=0;
					}
				}
			}
			// printf("tau: %s\n", tau.output().ptr());
		}
#endif
		vectorn ddq,nb;
		nb=(b.column(0))*-1+tau;
		// printf("1;%s %s\n", b.output().ptr(), nb.output().ptr());
		m::LUsolve(m, nb, ddq);
		//m::PIsolve(m, nb, ddq);
		//printf("2;%s %s\n", b.output().ptr(),/*m.output().ptr(),*/ ddq.output().ptr());
		
		if(useAISTmassMat){
			// conversion:
			// AIST's v and dv is different from gmbs' ones (denoted by w and dw here).
			// AIST's v=Rw
			// AIST's dv=Rdw+dotRw
			int ichara=0;
			VRMLloader const& l=*_characters[ichara]->skeleton;
			GJointFreeC2* j=(GJointFreeC2* )_cinfo[ichara]->_links[((VRMLTransform&)l.bone(1)).mJoint->jointStartId]->joint;
			SO3 invR=j->spherical_joint.inv_T.GetRotation();
			SO3 R=j->spherical_joint.T.GetRotation();
			se3 dv(toGMBS(ddq.toVector3(3)), toGMBS(ddq.toVector3(0)));
			Vec3 ww=j->Sdq.GetW();
			// dv-=dotR w
			dv-=mult(R*skew(ww), j->Sdq);
			se3 dw=mult(invR, dv);
			vectorn ddq2;
			ddq2.setSize(ddq.size()+1);
			ddq2.setVec3(4, toBase(dw.GetW()));
			ddq2.setVec3(0, toBase(dw.GetV()));
			ddq2.range(7,ddq2.size())=ddq.range(6,ddq.size());
			setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, ddq2);
		}
		else
		{
			vectorn ddq2;
			ddq2.setSize(ddq.size()+1);
			ddq2.setVec3(4, ddq.toVector3(0));//toBase(dw.GetW()));
			ddq2.setVec3(0, ddq.toVector3(3));//toBase(dw.GetV()));
			ddq2.range(7,ddq2.size())=ddq.range(6,ddq.size());
			setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, ddq2);
		}
	}
	else {
	
		int ichara=0;
		_cinfo[ichara]->_system->update_joint_local_info();

		_cinfo[ichara]->_system->fsFwdRecursion_a();
		_cinfo[ichara]->_system->fsBwdRecursion_b();
		_cinfo[ichara]->_system->fsFwdRecursion_c();

		_cinfo[ichara]->_system->updateGlobalLocationsOfBodiesAndJoints();

	}
	{
		list<GCoordinate *>::iterator iter_pcoord;
		int ichara=0;
		for (iter_pcoord = _cinfo[ichara]->_system->pCoordinates.begin(); iter_pcoord != _cinfo[ichara]->_system->pCoordinates.end(); iter_pcoord++) {
			(*iter_pcoord)->dq += (*iter_pcoord)->ddq * _timeStep;
			(*iter_pcoord)->q += (*iter_pcoord)->dq * _timeStep;		// semi-explicit integration: displacement is updated with the new velocity
		}
	}

	_currTime+=_timeStep;

	int ichara=0;
	_cinfo[ichara]->_system->updateKinematics();
	_updateCharacterPose();
	
	int n = _characters.size();

	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();
	_calcContactForce(*collisions);

	_contacts->solve(collisionDetector->getCollisionPairs(),*collisions);
#ifdef USE_AIST_SIM
	getLinkData(0, JOINT_VELOCITY, _tempVel);
	AISTsim->setLinkData(0, JOINT_VALUE, _tempPose);
	AISTsim->setLinkData(0, JOINT_VELOCITY, _tempVel);
	AISTsim->world.bodyInfoArray[0].forwardDynamics->initialize(); 
	AISTsim->world.contactForceSolver.clearExternalForces(); // already done above
	AISTsim->world.contactForceSolver.solve(*collisions);
	CFSImpl_LCP* solver=(CFSImpl_LCP*)AISTsim->world.contactForceSolver.impl;
	std::vector<CFSImpl_LCP::LinkPair*>& clinks=solver->constrainedLinkPairs;

	if(!useMassMatrix){
		_cinfo[ichara]->_system->initExternalForce();
		{
			int n=clinks.size();
			for (int i=0; i<n ; ++i){
				CFSImpl_LCP::LinkPair* linkPair=clinks[i];
				for (int ipair=0; ipair<2; ++ipair){
					Link* link=linkPair->link[ipair];
					if (link->jointType!=Link::FIXED_JOINT){
						int ijoint=link->jointId+1;
						int ibody=linkPair->bodyIndex[ipair];
						std::vector<BodyPart*>& cinfo=_cinfo[ibody]->_links;
						cinfo[ijoint]->body->addExternalForceGlobally(dse3(toGMBS(link->tauext),toGMBS(link->fext)));
					}
				}
			}	
		}
	}
#endif

	return true;
}

void DynamicsSimulator_gmbs::calcMassMatrix(int ichara,matrixn& out)
{
	/*
 		  |       |   | dv   |   |    |   | fext      |
		  | out_M | * | dw   | + | b1 | = | tauext    |
		  |       |   |ddq   |   |    |   | u         |
	*/
	//AISTsim->calcMassMatrix(0, out);
	
}
static void ID(DynamicsSimulator_gmbs& sim, vectorn const& desiredacceleration, vectorn& controlforce)
{
	sim.setLinkData(0, DynamicsSimulator::JOINT_ACCELERATION, desiredacceleration);
	int ichara=0;
	//sim._cinfo[ichara]->_system->initExternalForce();
	sim._cinfo[ichara]->_system->update_joint_local_info();
	sim._cinfo[ichara]->_system->fsFwdRecursion_a();
	sim._cinfo[ichara]->_system->fsBwdRecursion_b();
	sim._cinfo[ichara]->_system->fsFwdRecursion_c();
	//sim._system->updateGlobalLocationsOfBodiesAndJoints();
	sim.getLinkData(0, DynamicsSimulator::JOINT_TORQUE, controlforce);

	//vectorn theta,dtheta;
	//sim.getLinkData(0, DynamicsSimulator::JOINT_VALUE, theta);
	//sim.getLinkData(0, DynamicsSimulator::JOINT_VELOCITY, dtheta);
	//printf("%s\n%s\n:thetadtheta\n", theta.output().ptr(), dtheta.output().ptr());
	//printf("cf: %s\n", controlforce.output().ptr());
}

inline void packTau(vectorn& out, vectorn const& in)
{
	out.setSize(in.size()-1);
	out.range(0,3).assign(in.range(4,7));
	out.range(3,6).assign(in.range(0,3));
	out.range(6,out.size()).assign(in.range(7, in.size()));
}

void DynamicsSimulator_gmbs::calcMassMatrix3(int ichara, matrixn& M, vectorn& b)
{
	/* differs from calcMassMatrix !!! all dw, dv, tauext, fext are w.r.t body local coordinate!
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
void DynamicsSimulator_gmbs::calcMassMatrix2(int ichara, matrixn& M, vectorn& b)
{
#ifdef USE_AIST_SIM
	/* differs from calcMassMatrix !!! all dw, dv, tauext, fext are w.s.p body local coordinate!
 		  |       |   | dw   |   |    |   | tauext    |
		  | out_M | * | dv   | + | b1 | = | fext      |
		  |       |   |ddq   |   |    |   | u         |
	*/
	AISTsim->calcMassMatrix2(ichara, M,b.column().lval());
	static matrixn temp;
	static vectorn tempb;

	// swap tau and moment rows
	temp.assign(M.range(0,M.rows(),0,3));
	M.range(0,M.rows(),0,3).assign(M.range(0,M.rows(),3,6));
	M.range(0,M.rows(),3,6).assign(temp);
	temp.assign(M.range(0,3,0,M.cols()));
	M.range(0,3,0,M.cols()).assign(M.range(3,6,0,M.cols()));
	M.range(3,6,0,M.cols()).assign(temp);
	tempb.assign(b.range(0,3));
	b.range(0,3).assign(b.range(3,6));
	b.range(3,6).assign(tempb);

	//  convert to body-local ddq[0:6]
	VRMLloader const& l=*_characters[ichara]->skeleton;
	GJointFreeC2* j=(GJointFreeC2* )_cinfo[ichara]->_links[0]->joint;
	matrixn R,tmp;
	RMat_assign(R, j->spherical_joint.T.GetRotation());
//	matrixn R2;
//	RMat_assign(R2, _links[ichara][0]->body->T.GetRotation());
//	printf("%s %s\n", R.output().ptr(), R2.output().ptr());
	::vector3 v;
	
	// cout << "body v:"<<_links[ichara][0]->body->V<<"\n";
	// cout << "body sdq:"<<_links[ichara][0]->body->Sdq<<"\n";
	// cout << "joint sdq:"<<j->Sdq<<"\n";

	Vec3 ww=j->Sdq.GetW();
	se3 dotR_Sdq=mult(j->spherical_joint.T.GetRotation()*skew(ww), j->Sdq);
	for(int i=0; i<M.rows(); i++){
		// b[i]+=dot(M.row(i).range(0,6), dotR_Sdq)
		b[i]+=M(i,0)*dotR_Sdq[0]+M(i,1)*dotR_Sdq[1]+M(i,2)*dotR_Sdq[2]+M(i,3)*dotR_Sdq[3]+M(i,4)*dotR_Sdq[4]+M(i,5)*dotR_Sdq[5];
	}
	for(int i=0; i<M.rows(); i++){
		tmp.mult(M.range(i,i+1,0,3),R);
		M.range(i,i+1,0,3).assign(tmp);
		tmp.mult(M.range(i,i+1,3,6),R);
		M.range(i,i+1,3,6).assign(tmp);
	}
	if (useLocalExtForceInMassMat){
		// coordinate transform to use local external forces
		// M ddq + b =R JtV lambda
		// R' M ddq + R' b = JtV lambda

		matrixn Rt;
		Rt.transpose(R);
		M.range(0,3, 0, M.cols()).mult(Rt, M.range(0,3,0,M.cols()));
		M.range(3,6, 0, M.cols()).mult(Rt, M.range(3,6,0,M.cols()));
		temp.mult(Rt, b.range(0,3).column());
		b.range(0,3).assign(temp.column(0));
		temp.mult(Rt, b.range(3,6).column());
		b.range(3,6).assign(temp.column(0));
	}
#endif
}
void DynamicsSimulator_gmbs::setGVector
(
 const ::vector3& wdata
 )
{
	int ichara=0;
	_cinfo[ichara]->_system->setGravity(toGMBS(wdata*-1));

	if(debugMode){
		cout << "DynamicsSimulator_gmbs::setGVector("
			 << wdata[0] << ", "
			 << wdata[1] << ", "
			 << wdata[2] << ")" << endl;
	}
	//AISTsim->setGVector(wdata);
}


void DynamicsSimulator_gmbs::getGVector
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
		cout << "DynamicsSimulator_gmbs::getGVector(";
		cout << wdata[0] << ", "
			 << wdata[1] << ", "
			 << wdata[2] << ")" << endl;
	}
}


void DynamicsSimulator_gmbs::setCharacterAllJointModes
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
	//	cout << "DynamicsSimulator_gmbs::setCharacterAllJointModes(";
	//	cout << characterName << ", ";
	//	cout << (isHighGainMode ? "HIGH_GAIN_MODE" : "TORQUE_MODE");
	//	cout << ")" << endl;
	//}
}




void DynamicsSimulator_gmbs_getJacobian_coordIndex(GBody* pbody, intvectorn& out)
{
	out.setSize(pbody->fJL.pCoordinates.size());

	list<GCoordinate *>::iterator iter_pcoord;
	int idx=0;
	for(iter_pcoord=pbody->fJL.pCoordinates.begin(); iter_pcoord!=pbody->fJL.pCoordinates.end(); ++iter_pcoord)
	{
		out(idx)=(*iter_pcoord)->system_idx_coord;
		idx++;
	}
	// cout <<"jacobi "<< pbody->pBaseJoint->getName()<< " " << pbody->fJL.pCoordinates.size() << out.output().ptr()<<"\n";
}
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
void DynamicsSimulator_gmbs::calcDotJacobian(int ichara, int ibone, matrixn& dotjacobian)
{
	_calcDotJacobian(ichara,ibone,dotjacobian,true);
}
void DynamicsSimulator_gmbs::calcJacobian(int ichara, int ibone, matrixn& jacobian)
{
	_calcJacobian(ichara,ibone,jacobian,true);
}

void DynamicsSimulator_gmbs::_calcDotBodyJacobian(int ichara, int ibone, matrixn& jacobian, matrixn& dotjacobian, bool update)
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

void DynamicsSimulator_gmbs::_calcDotJacobian(int ichara, int ibone, matrixn& dotjacobian, bool update)
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

void DynamicsSimulator_gmbs::_calcJacobian(int ichara, int ibone, matrixn& jacobian,bool update)
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
void DynamicsSimulator_gmbs::calcCOMjacobian(int ichar, matrixn& jacobian)
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

				
				vector3 localpos=b.localCOM();
				jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+mass*( w.cross(R*localpos)+v));
			}
		}
		// printf("totalMass: %f %f\n", _system->getMass(), totalMass);
		jacobian*=1.0/_cinfo[ichar]->_system->getMass();
	}
}
void DynamicsSimulator_gmbs::calcBoneDotJacobian2(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian)
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
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
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
			//   = (   I           0   )(w)
			//     ( skew(-Rc)     I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+( w));
			jacobian.column(k).setVec3(3, jacobian.column(k).toVector3(3)+( w.cross(R*localpos)+v));

			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+(dw));
			dotjacobian.column(k).setVec3(3, dotjacobian.column(k).toVector3(3)+(dw.cross(R*localpos)+w.cross(R*skewLw*localpos)+dv));
		}
	}
}
void DynamicsSimulator_gmbs::calcBoneDotJacobian(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian)
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
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
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
			//   = (   I           0   )(w)
			//     ( skew(-Rc)     I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+( w.cross(R*localpos)+v));

			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+(dw.cross(R*localpos)+w.cross(R*skewLw*localpos)+dv));
		}
	}
}
void DynamicsSimulator_gmbs::calcCOMdotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
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
		::vector3 lw=toBase(getBody(_cinfo[ichar]->_links,&b)->V.GetW());
		skewLw.setTilde(lw);
		vector3 localpos=b.localCOM();
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
			//   = (   I           0   )(w)
			//     ( skew(-Rc)     I   )(v)
				
			jacobian.column(k).setVec3(0, jacobian.column(k).toVector3(0)+mass*( w.cross(R*localpos)+v));

			
			
			::vector3 dw=dj.column(k).toVector3(0);
			::vector3 dv=dj.column(k).toVector3(3);
			dotjacobian.column(k).setVec3(0, dotjacobian.column(k).toVector3(0)+mass*(dw.cross(R*localpos)+w.cross(R*skewLw*localpos)+dv));
		}
	}
	// printf("totalMass: %f %f\n", _system->getMass(), totalMass);
	jacobian*=1.0/_cinfo[ichar]->_system->getMass();
	dotjacobian*=1.0/_cinfo[ichar]->_system->getMass();
}

Liegroup::dse3 DynamicsSimulator_gmbs::calcMomentumGlobal(int ichara)
{
	::dse3 v=_cinfo[ichara]->_system->getMomentumGlobal();
	Liegroup::dse3 out;
	out=v.GetArray();
	return out;
}
Liegroup::dse3 DynamicsSimulator_gmbs::calcMomentumCOM(int ichara)
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

Liegroup::se3 transf_twist(transf const& tf1, transf const& tf2, double timestep)
{
	matrix4 t1, t2, inv_t1,v;
	t1.setRotation(tf1.rotation);
	t1.leftMultTranslation(tf1.translation);
	t2.setRotation(tf2.rotation);
	t2.leftMultTranslation(tf2.translation);
	::matrix4 dotT;
	dotT.sub(t2,t1);
	dotT*=1.0/timestep;
	inv_t1.inverse(t1);
	v.mult(inv_t1,dotT);
	return Liegroup::se3(::vector3(v._23*-1, v._13, v._12*-1), 
						::vector3(v._14, v._24, v._34));
}
//Liegroup::se3 transf_twist_nonlinear(transf const& tf1, transf const& tf2, double timestep);

void DynamicsSimulator_gmbs::setInertia(int ichara, int ibone, ::vector3 const& localCOM, ::vector3 const& inertia)
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
void  DynamicsSimulator_gmbs::calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const
{
#if 1
	VRMLloader* skel=_characters[ichara]->skeleton;
	BoneForwardKinematics chain(skel);
	chain.init();
	::vector3 com(0,0,0);
	m_real totalMass=0.0;

	Liegroup::Inertia I;
	chain.setPoseDOF(pose);

	// T_(-COM)*T_G*T(lCOM)*local_position_WT_COM
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;


		Liegroup::Inertia Ii(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		Ii=Ii.transform(chain.global(bone).inverse()); // inverse is there because of dAd transformation
		//Ii=Ii.transform(toGMBS(chain2.global(bone).inverse()));
		//printf("Ii: %f %f %f %f %f %f, %f\n", body->I._I[0],body->I._I[1],body->I._I[2],body->I._I[3],body->I._I[4],body->I._I[5],body->I._m);
		//printf("Ii: %f %f %f %f %f %f, %f\n", Ii._I[0],Ii._I[1],Ii._I[2],Ii._I[3],Ii._I[4],Ii._I[5],Ii._m);
		for (int i=0;i<6; i++) I._I[i]+=Ii._I[i];
		for (int i=0;i<3; i++) I._r[i]+=Ii._r[i];
		I._m+=Ii._m;
	}

	com/=totalMass;

	I=I.transform(transf(quater(1,0,0,0), com)); // inv(T_{com*-1}) == T_com
	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	inertia.setValues(10, I._I[0], I._I[1], I._I[2],  I._I[3],I._I[4],I._I[5],I._m, I._r[0], I._r[1], I._r[2]);
#else
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
#endif
}

void  DynamicsSimulator_gmbs::calcInertiaUpper(int ichara,vectorn const& pose, vectorn& inertia) const
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
	//for(int ibone=1; ibone<skel->numBone(); ibone++)
	for(int ibone=1; ibone<15; ibone++)
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


void  DynamicsSimulator_gmbs::calcInertiaLower(int ichara,vectorn const& pose, vectorn& inertia) const
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
	//for(int ibone=1; ibone<skel->numBone(); ibone++)
	for(int ibone=15; ibone<skel->numBone(); ibone++)
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
Liegroup::dse3 DynamicsSimulator_gmbs::calcMomentumCOMfromPose(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
{
#if 1
	VRMLloader* skel=_characters[ichara]->skeleton;
	BoneForwardKinematics chain1(skel), chain2(skel);
	chain1.init();
	chain2.init();

	::vector3 com(0,0,0);
	Liegroup::dse3 H(0,0,0,0,0,0);

	m_real totalMass=0.0;
	chain1.setPoseDOF(poseFrom);
	chain2.setPoseDOF(poseTo);
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=chain1.global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
		//Liegroup::se3 V=transf_twist(chain1.global(bone), chain2.global(bone),delta_t);
		//Liegroup::se3 V=transf_twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
		Liegroup::se3 V=Liegroup::twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
		//printf("%d : %s, %f %f %f\n", ibone, V.W().output().ptr(), body->V[0], body->V[1], body->V[2]);
		//printf("%d : %s, %f %f %f\n", ibone, V.V().output().ptr(), body->V[3], body->V[4], body->V[5]);

		Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		H+=(I*V).inv_dAd(chain1.global(bone));
	}

	com/=totalMass;

	//printf("I: %f %f %f %f %f %f, %f,%f\n", I._I[0],I._I[1],I._I[2],I._I[3],I._I[4],I._I[5],I._m, totalMass);

	Liegroup::dse3 v;
	v.dAd(transf(quater(1,0,0,0), com), H);
	return v;
#else
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
		Liegroup::se3 V=transf_twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
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
#endif
}

Liegroup::dse3 DynamicsSimulator_gmbs::calcMomentumCOMfromPoseUpper(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
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
		//Liegroup::se3 V=transf_twist_nonlinear(chain1.global(bone), chain2.global(bone),delta_t);
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

Liegroup::dse3 DynamicsSimulator_gmbs::calcMomentumCOMfromPoseLower(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo)
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

::vector3 calcCOM2_gmbs(OpenHRP::DynamicsSimulator_gmbs&s, int ichara)
{
	return toBase(s._cinfo[ichara]->_system->getPositionCOMGlobal());
}
::vector3 DynamicsSimulator_gmbs::calculateCOM(int ichara, double& outputTotalMass)
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
void DynamicsSimulator_gmbs::calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
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
void DynamicsSimulator_gmbs::getLinkData(int ichara, LinkDataType t, vectorn& out)
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
void DynamicsSimulator_gmbs::setLinkData(int ichara, LinkDataType t, vectorn const& in)
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
//void DynamicsSimulator_gmbs::forwardKinamaticsAcc()
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

double DynamicsSimulator_gmbs::currentTime() const
{
	return _currTime	;
}

void DynamicsSimulator_gmbs::setCurrentTime(double t)
{
	_currTime	=t;
}

void DynamicsSimulator_gmbs::inverseDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, vectorn& controlforce)
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


/*
void DynamicsSimulator_gmbs::hybridDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce)
{
	BEGIN_TIMER(setLinkD);
	setLinkData(0, JOINT_VALUE, theta);
	setLinkData(0, JOINT_VELOCITY, dtheta);
	setLinkData(0, JOINT_ACCELERATION, desiredacceleration);
	_system->updateKinematics();

	VRMLloader& l=*(_characters[0]->skeleton);
	VRMLTransform& b=l.VRMLbone(1);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;

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
	_system->initExternalForce();

	for(int i=0; i<contacts.size(); i++){
		ContactForce const& cq=contacts[i];
		if(cq.chara==0) {
			_addForceToLink(*this, 0, cq.bone, cq.p, cq.f, cq.tau);
		}
	}

	END_TIMER(initExternalForce);
	BEGIN_TIMER(calcDynamics);
	_system->calcDynamics(true);

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

void DynamicsSimulator_gmbs::hybridDynamics2(bitvectorn const& isActuated, vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce)
{
	setLinkData(0, JOINT_VALUE, theta);
	setLinkData(0, JOINT_VELOCITY, dtheta);
	setLinkData(0, JOINT_ACCELERATION, desiredacceleration);
	_system->updateKinematics();

	VRMLloader& l=*(_characters[0]->skeleton);
	VRMLTransform& b=l.VRMLbone(1);
	OpenHRP::DynamicsSimulator& s=*this;
	std::vector<BodyPart*>& cinfo=_cinfo[ichara]->_links;

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
	_system->initExternalForce();

	for(int i=0; i<contacts.size(); i++){
		ContactForce const& cq=contacts[i];
		if(cq.chara==0) {
			_addForceToLink(*this, 0, cq.bone, cq.p, cq.f, cq.tau);
		}
	}

	END_TIMER(initExternalForce);
	BEGIN_TIMER(calcDynamics);
	_system->calcDynamics(true);

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

*/
