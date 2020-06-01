#include "physicsLib.h"
#include "OpenHRPcommon.h"
#include "DynamicsSimulator.h"
#include "../MainLib/OgreFltk/MotionManager.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#include "CollisionDetector.h"

extern bool _debug_DynamicsSimulator_penaltyMethod_calcContactForce;
bool _debug_DynamicsSimulator_penaltyMethod_calcContactForce=true;
#ifdef INCLUDE_OPCODE_DETECTOR
#include "../CollisionDetector.h"
#endif
// taesoo. global parameters.


void makeCharacterInfo(VRMLloader const& input, OpenHRP::CharacterInfo& output)
{
	output.loader=(VRMLloader*)&input;
	output.name=input.name;
	output.url=input.url;
	output.info=input.info;

	int numJoint=((VRMLTransform& )input.bone(input.numBone()-1)).mJoint->jointEndId;

	output.links.resize(numJoint);
	for(int i=1; i<input.numBone(); i++)
		{
			VRMLTransform* ll=(VRMLTransform*)&input.bone(i);

			int numHRPjoints=ll->numHRPjoints();
			ASSERT(numHRPjoints>0);

			OpenHRP::LinkInfo& ls=output.links[ll->mJoint->jointStartId];
			OpenHRP::LinkInfo& le=output.links[ll->mJoint->jointEndId-1];

			ASSERT(ll->mJoint);
			ls.translation=ll->mJoint->translation;
			ls.rotation.setFromQuaternion(
										  quater(ll->mJoint->rotation.w(),
												 vector3(ll->mJoint->rotation.x(),
														 ll->mJoint->rotation.y(),
														 ll->mJoint->rotation.z())));


			if(ll->mSegment)
				{
					le.mass=ll->mSegment->mass;
					le.centerOfMass=ll->mSegment->centerOfMass;
					le.inertia=ll->mSegment->momentsOfInertia;
				}	// otherwise use default.
			else
				{
					ll->mSegment=new HRP_SEGMENT();
					ll->mSegment->mass=le.mass;
					ll->mSegment->centerOfMass=le.centerOfMass;
					ll->mSegment->momentsOfInertia=le.inertia;
				}
			le.rotorInertia=0;
			le.gearRatio=1;
			le.equivalentInertia=0;

			int js=ll->mJoint->jointStartId;
			int je=ll->mJoint->jointEndId;
			for(int jointID=js; jointID<je; jointID++)
				{
					OpenHRP::LinkInfo& l=output.links[jointID];

					l.jointType=ll->HRPjointType(jointID-js);//mJoint->jointType;
					/*if(l.jointType==HRP_JOINT::ROTATE)
					  l.jointAxis=ll->mJoint->jointAxis.subString(jointID-js,jointID-js+1);
					  else
					  l.jointAxis=ll->mJoint->jointAxis;*/
					l.jointAxis=ll->HRPjointAxis(jointID-js);
					l.jointId=jointID;
					l.name=ll->HRPjointName(jointID-js);
					if(jointID!=js)
						l.mother=jointID-1;
					if(jointID!=je-1)
						l.daughter=jointID+1;
				}
			if(!ll->isRootBone())
				ls.mother=((VRMLTransform* )ll->parent())->mJoint->jointEndId-1;

			if(ll->mShape)
				le.mesh=&ll->mShape->mesh;

			for(Bone* s=ll->sibling(); s ; s=s->sibling())
				{
					HRP_JOINT* j=((VRMLTransform* )s)->mJoint;
					if(j)
						{
							ls.sister=j->jointStartId;
							break;
						}
				}

			for(Bone* c=ll->child(); c; c=c->sibling())
				{
					HRP_JOINT* j=((VRMLTransform* )c)->mJoint;
					if(j)
						{
							le.daughter=j->jointStartId;
							break;
						}
				}
		}
}

namespace OpenHRP
{
	static double _depthMax=0.01;	// 1cm
	namespace globals
	{
		bool usePenaltyMethod=false;
	}

	DynamicsSimulator::Character::Character(VRMLloader* skel)
	{
		skeleton=skel;
		chain=new BoneForwardKinematics(skel);
		chain->init();
	}

	DynamicsSimulator::Character::~Character()
	{
		delete chain;
	}



	void DynamicsSimulator::Character::setChain(const vectorn& poseDOF)
	{
		chain->setPose(skeleton->dofInfo.setDOF(poseDOF));
	}

	void DynamicsSimulator::Character::getChain(vectorn& poseDOF)
	{
		static Posture p;
		chain->getPoseFromGlobal(p);
		skeleton->dofInfo.getDOF(p, poseDOF);
	}
#ifndef NO_BULLET
	OpenHRP::CollisionDetector* createCollisionDetector_bullet();
	OpenHRP::CollisionDetector* createCollisionDetector_bullet2(DynamicsSimulator*psim);
	OpenHRP::CollisionDetector* createCollisionDetector_simple();
#endif
	//OpenHRP::CollisionDetector* createCollisionDetector_simple2();
	OpenHRP::CollisionDetector* createCollisionDetector_libccd();

	DynamicsSimulator::DynamicsSimulator(const char* type)
	{
		TString tid(type);
		tid.makeUpper();
#ifndef NO_BULLET
		if(tid=="SIMPLE")
			collisionDetector =createCollisionDetector_simple();
		else if(tid=="BULLET")
			collisionDetector =createCollisionDetector_bullet();
		else if(tid=="BULLET2")
			collisionDetector =createCollisionDetector_bullet2(this);
		else 
#else
		if(tid=="SIMPLE")
		{
			ASSERT(false);
			//collisionDetector =createCollisionDetector_simple2();
		}
		else
#endif
			if(tid=="LIBCCD")
			collisionDetector =createCollisionDetector_libccd();
		else if(tid=="OPCODE")
		{
#ifdef INCLUDE_OPCODE_DETECTOR
				// resolve collisionchecker object
				CollisionDetectorFactory* collisionDetectorFactory=
					new CollisionDetectorFactory();

				collisionDetector = collisionDetectorFactory->create();
#else
				Msg::error("CollisionDetector is not included in the project");
				collisionDetector=NULL;
#endif
		}
		else
			Msg::error("unknown collision detector type");


		collisions = new CollisionSequence;
	}
	DynamicsSimulator::DynamicsSimulator(bool useSimpleColdet)
	{
#ifndef NO_BULLET
		if (useSimpleColdet)
			collisionDetector =createCollisionDetector_simple();
		else
#endif
			//collisionDetector =createCollisionDetector_bullet();
			collisionDetector =createCollisionDetector_libccd();

		collisions = new CollisionSequence;

	}

	DynamicsSimulator::~DynamicsSimulator()
	{
		delete collisionDetector;
		delete collisions;
		for(int i=0; i<_characters.size(); i++)
			delete _characters[i];
	}

	void DynamicsSimulator::_updateCharacterPose()
	{
		int n = _characters.size();

		for(int i=n-1; i>=0; i--)
			{
				vectorn& _tempPose=_characters[i]->_tempPose;
				getLinkData(i, DynamicsSimulator::JOINT_VALUE, _tempPose);
				_characters[i]->setChain(_tempPose);
			}
	}
	vector3 DynamicsSimulator::calculateCOM(int ichara, double& outputTotalMass)
	{
		VRMLloader* skel=_characters[ichara]->skeleton;
		BoneForwardKinematics* chain=_characters[ichara]->chain;

		vector3 com(0,0,0);

		m_real totalMass=0.0;
		for(int ibone=1; ibone<skel->numBone(); ibone++)
			{
				VRMLTransform& bone=skel->VRMLbone(ibone);
				ASSERT(bone.mSegment);
				double mass=bone.mSegment->mass;
				com+=chain->global(bone).toGlobalPos(bone.mSegment->centerOfMass)*mass;
				totalMass+=mass;
			}

		com/=totalMass;
		outputTotalMass=totalMass;
		return com;
	}

void DynamicsSimulator::registerContactQueryBone(int index, VRMLTransform* bone)
{
	if(_boneToCQindex.size()==0)
		_boneToCQindex.setAllValue(skeleton(0).numBone(), -1);

	_contactQueryBones.resize(std::max((int)_contactQueryBones.size(), (int)index+1));
	_contactQueryBones[index].bone=bone;
	_contactQueryBones[index].coef=1.0;

	_boneToCQindex(bone->treeIndex())=index;
}
bool DynamicsSimulator::queryContact(int index)
{
	return _contactQueryBones[index].contact;
}

vectorn DynamicsSimulator::queryContacts()
{
	vectorn out(_contactQueryBones.size());

	for(int i=0; i<_contactQueryBones.size(); i++)
	{
		out(i)=int(_contactQueryBones[i].contact);
	}
	return out;
}

std::vector<DynamicsSimulator::CQInfo> const&	DynamicsSimulator::queryContactInfoAll()
{
	return _contactQueryBones;
}
void DynamicsSimulator::_updateContactInfo(CollisionSequence& corbaCollisionSequence)
{
	for(size_t i=0; i<_contactQueryBones.size(); i++)
	{
		_contactQueryBones[i].contact=false;
		_contactQueryBones[i].depth=0;
	}

	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = corbaCollisionSequence[i].points;

		int n_point = points.size();
		if(n_point == 0) continue;

		double k,d;
		if(linkPair.param.size()>3)
		{
			k=linkPair.param[2];
			d=linkPair.param[3];
		}
		else
		{
			k=10000;
			d=1000;
		}
		for(int j=0; j<n_point; j++)
		{
			int cqindex=(_boneToCQindex.size())?_boneToCQindex[linkPair.link[0]->treeIndex()]:-1;
			if(cqindex!=-1)
			{
				double depth=points[j].idepth;

				CQInfo& cq=_contactQueryBones[cqindex];
				cq.contact=true;
				// _contactQueryBones[cqindex].depth=std::max(_contactQueryBones[cqindex].depth, depth);
				cq.depth+=depth;

			}
		}
	}
}
vectorn DynamicsSimulator::queryContactDepths()
{
	vectorn out(_contactQueryBones.size());

	for(int i=0; i<_contactQueryBones.size(); i++)
	{
		out(i)=(_contactQueryBones[i].depth);
	}
	return out;
}
void DynamicsSimulator::registerCharacter(VRMLloader*l)
{
	OpenHRP::CharacterInfo cinfo;
	makeCharacterInfo(*l, cinfo);
	_registerCharacter(cinfo.name, cinfo);
}

void DynamicsSimulator::createObstacle(OBJloader::Geometry const& mesh)
{
	OpenHRP::CharacterInfo cinfo;
	VRMLloader* l=new VRMLloader(mesh, true);
	RE::motionManager().createMotionLoaderExt(l->name, (MotionLoader*)l);
	registerCharacter(l);
}
void DynamicsSimulator::createFreeBody(OBJloader::Geometry const& mesh)
{
	OpenHRP::CharacterInfo cinfo;
	VRMLloader* l=new VRMLloader(mesh, false);
	RE::motionManager().createMotionLoaderExt(l->name, (MotionLoader*)l);
	registerCharacter(l);
}


vector3 DynamicsSimulator::calculateCOMvel(int ichara, double& outputTotalMass)
{
	VRMLloader* skel=_characters[ichara]->skeleton;
	BoneForwardKinematics* chain=_characters[ichara]->chain;

	vector3 com(0,0,0);

	m_real totalMass=0.0;
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		com+=getWorldVelocity(ichara, &bone, bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
	}

	com/=totalMass;
	return com;
}

vector3 DynamicsSimulator::calculateZMP(int ichara)
{
	VRMLloader* skel=_characters[ichara]->skeleton;
	BoneForwardKinematics* chain=_characters[ichara]->chain;

	vector3 zmp;
	float g=-9.8;		//meter/second^2

	vector3 ri, ri__;
	m_real mi, A, B, C, D, E;
	A= B= C= D= E= 0.f;
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		ri=chain->global(bone).toGlobalPos(bone.mSegment->centerOfMass);
		ri__=getWorldAcceleration(ichara, &bone, bone.mSegment->centerOfMass);
		mi=bone.mSegment->mass;

		A+=mi*(ri__.y-g  )*ri.x;
		B+=mi*(ri__.x-0.f)*ri.y;
		C+=mi*(ri__.y-g  );
		D+=mi*(ri__.y-g  )*ri.z;
		E+=mi*(ri__.z-0.f)*ri.y;
	}

	zmp.x=(A-B)/C;
	zmp.y=0;
	zmp.z=(D-E)/C;	
	return zmp;
}

BoneForwardKinematics& DynamicsSimulator::getWorldState(int ichara)
{
	return *_characters[ichara]->chain;
}

void DynamicsSimulator::setWorldState(int ichara)
{
	vectorn &_tempPose=_characters[ichara]->_tempPose;
	_characters[ichara]->getChain(_tempPose);
	setLinkData(ichara, JOINT_VALUE, _tempPose);
}

void DynamicsSimulator::getWorldPosition(int ichara,VRMLTransform* b, vector3 const& localpos, vector3& pos) const
{
	pos=((DynamicsSimulator*)(this))->getWorldState(ichara).global(*b).toGlobalPos(localpos);
}

vector3 DynamicsSimulator::getWorldVelocity(int ichara,VRMLTransform* b, vector3 const& localpos) const
{
	vector3 out;
	getWorldVelocity(ichara, b, localpos, out);
	return out;
}

::vector3 DynamicsSimulator::getWorldAngVel(int ichara, VRMLTransform* b) const
{
	vector3 out;
	getWorldAngVel(ichara, b, out);
	return out;
}
::vector3 DynamicsSimulator::getWorldAngAcc(int ichara, VRMLTransform* b) const
{
	vector3 out;
	getWorldAngAcc(ichara, b, out);
	return out;
}

::vector3 DynamicsSimulator::getWorldAcceleration(int ichara,VRMLTransform* b, ::vector3 const& localpos) const
{
	vector3 out;
	getWorldAcceleration(ichara, b, localpos, out);
	return out;
}

::vector3 DynamicsSimulator::getWorldPosition(int ichara, VRMLTransform* b, ::vector3 const& localpos) const
{
	return ((DynamicsSimulator*)(this))->getWorldState(ichara).global(*b).toGlobalPos(localpos);
}

void DynamicsSimulator::addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force)
{
	throw std::runtime_error("adddForceTobone not implemented yet");
}

void DynamicsSimulator::setFrictionCoef(int contactQueryIndex, double coef)
{
    _contactQueryBones[contactQueryIndex].coef=coef;
}



std::vector<DynamicsSimulator::ContactForce> &	DynamicsSimulator::queryContactAll()
{
	return _contactForces;
}
void DynamicsSimulator::_calcContactForce(CollisionSequence& corbaCollisionSequence)
{
	// By default, this function uses penalty method. Each simulator implementations needs to override this function.
	_updateContactInfo( corbaCollisionSequence);
	for(size_t i=0; i < collisionDetector->getCollisionPairs().size(); ++i)
	{
		LinkPair const& linkPair = collisionDetector->getCollisionPairs()[i];
		CollisionPointSequence& points = corbaCollisionSequence[i].points;

		int n_point = points.size();
		if(n_point == 0) continue;

		double k,d;
		if(linkPair.param.size()>3)
		{
			k=linkPair.param[2];
			d=linkPair.param[3];
		}
		else
		{
			k=10000;
			d=1000;
		}
		for(int j=0; j<n_point; j++)
		{
			const vector3& normal=points[j].normal;	
			const vector3& pos=points[j].position;
			double depth=points[j].idepth;


			transf const& frame1=getWorldState(linkPair.charIndex[0])._global(*linkPair.link[0]);
			transf const& frame2=getWorldState(linkPair.charIndex[1])._global(*linkPair.link[1]);

			vector3 lpos1, lpos2;
			lpos1=frame1.toLocalPos(pos);
			lpos2=frame2.toLocalPos(pos);
			vector3 vel1, vel2;
			getWorldVelocity(linkPair.charIndex[0],linkPair.link[0], lpos1, vel1);

			
			getWorldVelocity(linkPair.charIndex[1],linkPair.link[1], lpos2, vel2);

			
			//RE::output(TString("vel1")+linkPair.link[0]->name(), vel1.output().ptr());
			//RE::output(TString("vel2")+linkPair.link[1]->name(), vel2.output().ptr());

			vector3 relvel;
			relvel.sub(vel1, vel2);

			vector3 avel1, avel2;
			getWorldAngVel(linkPair.charIndex[0], linkPair.link[0], avel1);
			getWorldAngVel(linkPair.charIndex[1], linkPair.link[1], avel2);

			vector3 relavel;
			relavel.sub(avel1, avel2);

			double vn=normal%relvel;	// normal rel vel
			double avn=normal%relavel;


//#define LINEAR_CONTACT_FORCE
#ifdef LINEAR_CONTACT_FORCE
			//printf("depth=%f\n", depth);
			double f=depth*k + vn*d;
#else

			//double f=(k + vn*d)*depth;

			double f=depth*k ;

			double f2= vn*d*sop::clampMap(depth, 0, _depthMax);

			//if (f2<0) f2=0;

			//if (f2>0)
				f=f+f2;

				if(f<0) f=0;
			//else
			//	f=f+f2*0.5;
				
#endif




			if(f<0.0) continue;



			::vector3 normalForce=-1*normal*f;

			::vector3 fv=relvel-normal*vn;
			
			double tiny = 1e-8;
			double mu=1.0;
			int cqindex=(_boneToCQindex.size())?_boneToCQindex[linkPair.link[0]->treeIndex()]:-1;
			if (cqindex!=-1)
			{
			  mu=_contactQueryBones[cqindex].coef;
			  if (mu<0 ) continue;
			}

			 
			::vector3 force=normalForce;
			if(fv.length()>tiny)	// slip friction force
			{
				// calc frictionForce
				::vector3 dir;
				dir.normalize(fv);

				force-=dir*f*mu;
			}
			else
			{
				//printf("static friction needed\n");
			}
			force/=n_point;

			vector3 torque(0,0,0);
			if(ABS(avn)>tiny)
			{
				// slip friction torque
				double rmu=2.0/3.0*mu;
				double R=0.08; 
				// -> about average of (half of the shoe length) and (foot width) assuming there are only two contact points in a foot. 
				// : current contact model uses bullet collision detection engine for efficiency. 
				// a foot is divided into two parts and each part is modeled as a single convex object, so the above assumption.
				
				//				double R=0;
				if(avn>0)
					torque=R*f*normal*rmu*-1;
				else
					torque=R*f*normal*rmu;
			}

			_addContactForceToLink(linkPair.charIndex[0], linkPair.link[0], frame1.toLocalPos(pos), frame1.toLocalDir(force), frame1.toLocalDir(torque));

			if(cqindex!=-1)
			{
				if(_debug_DynamicsSimulator_penaltyMethod_calcContactForce){
					vectorn temp(16);
					temp.setAllValue(0);
					temp.setVec3(0, pos);
					temp.setVec3(3, force);
					temp.setVec3(6, torque);
					temp.setVec3(9, frame1.toLocalPos(pos));
					temp.setVec3(12, frame1.toLocalDir(force));
					//_debugInfo.add("%s pft lp lf %s\n", linkPair.link[0]->name().ptr(), temp.output().ptr());

				}
			}

			_addContactForceToLink(linkPair.charIndex[1], linkPair.link[1], frame2.toLocalPos(pos), frame2.toLocalDir(force*-1), frame2.toLocalDir(torque*-1));
		}
	}


}

void DynamicsSimulator::_addContactForceToLink(int ichara, VRMLTransform* bone, vector3 const& p, vector3 const& f, vector3 const& tau)
{
	_contactForces.resize(_contactForces.size()+1);
	_contactForces.back().chara=ichara;
	_contactForces.back().bone=bone;
	_contactForces.back().f=f;
	_contactForces.back().p=p;
	_contactForces.back().tau=tau;
}

void DynamicsSimulator::getContactLinkBoneIndex(int ipair, intvectorn & ibone)
{
	int size=collisionDetector->getCollisionPairs().size();
	ibone.resize(size);
	for(int i=0; i<size; i++)
	{
		ibone(i)=collisionDetector->getCollisionPairs()[i].link[ipair]->treeIndex();
	}
}
int DynamicsSimulator::getNumAllLinkPairs() const
{
	return collisionDetector->getCollisionPairs().size();
}
}
