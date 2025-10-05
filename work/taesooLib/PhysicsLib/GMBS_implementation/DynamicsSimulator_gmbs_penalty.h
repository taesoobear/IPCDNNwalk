// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#ifndef DYNAMICSSIMULATOR_gmbs_PENALTY_HEADER
#define DYNAMICSSIMULATOR_gmbs_PENALTY_HEADER



#include "../CollisionDetector.h"
#include "../DynamicsSimulator_penaltyMethod.h"
#include "../Liegroup.h"


class GBodyRigid;
class GJoint;
class GSystem;

struct BodyPart
{
	int jointId;
	int boneId;
	matrix3 Rs;
	int jointType;
	GBodyRigid* body;
	GJoint* joint;
	vector3 centerOfMass;
	TString name;
	BodyPart* parent;
	std::vector<BodyPart*> children;
	BodyPart(const char* name);
	~BodyPart();
	void addChild(BodyPart* link);
};

namespace OpenHRP {

/**
 * DynamicsSimulator_ class
 */
class DynamicsSimulator_gmbs_penalty : public DynamicsSimulator_penaltyMethod
								  
{
	
 public:
	struct System
	{
		System();
		~System();
		GSystem* _system;
		BodyPart* _ground;
		std::vector<BodyPart*> _links;
		intvectorn DOFindexToCoordIndex;
		intvectorn coordIndexToDOFindex;
		intvectorn DQindexToCoordIndex;
		intvectorn coordIndexToDQindex;
		void updateDOFindexMap(GSystem* _system, std::vector<BodyPart*>const & cinfo, VRMLloader const& l );
	};

	double _timeStep;
	double _currTime;			// 
	int _integrateOpt;

	std::vector<System*> _cinfo; // indexed by ichara
	
	DynamicsSimulator_gmbs_penalty();

	virtual ~DynamicsSimulator_gmbs_penalty();


	virtual void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const;
	virtual void getWorldVelocity(int ichara, VRMLTransform* b
								  , ::vector3 const& localpos
								  , ::vector3& velocity) const;

	virtual void getWorldAcceleration(int ichara,VRMLTransform* b
									  , ::vector3 const& localpos
									  , ::vector3& acc) const;

	double calcKineticEnergy() const;

	virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const;
	virtual void getWorldAngAcc(int ichara, VRMLTransform* b, ::vector3& angacc) const;
	void addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force);

	virtual void _registerCharacter(const char *name, CharacterInfo const& cinfo);

	void inverseDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, vectorn& controlforce);
	void hybridDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce);
	void hybridDynamics2(bitvectorn const& isActuated,vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce);

	virtual void init(double timeStep,
					  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);

	double currentTime() const;

	virtual void setTimestep(double timeStep);


	virtual void initSimulation();
		
	virtual bool stepSimulation();

	virtual void setGVector(const ::vector3& wdata);

	virtual void getGVector(::vector3& wdata);

	virtual void setCharacterAllJointModes
		(
		 const char* characterName, 
		 OpenHRP::DynamicsSimulator::JointDriveMode jointMode);

	virtual ::vector3 calculateCOM(int ichara, double& outputTotalMass); 
	virtual ::vector3 calculateCOMvel(int ichara, double& outputTotalMass);
	virtual ::vector3 calculateCOMacc(int ichara);

	void _calcJacobian(int ichar, int ibone, matrixn& jacobian, bool update);
	// dstate to world velocity
	void _calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian, bool update);
	// dstate to body velocity
	void _calcDotBodyJacobian(int ichar, int ibone, matrixn& jacobian, matrixn& dotjacobian, bool update);
	void calcJacobian(int ichar, int ibone, matrixn& jacobian);
	void calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian);
	void calcCOMjacobian(int ichar, matrixn& jacobian);
	void calcCOMdotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);
	// return position jacobian matrix (3 by numDOF)
	void calcBoneDotJacobian(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
	// return full jacobian matrix (6 by numDOF)
	void calcBoneDotJacobian2(int ichar, int ibone, ::vector3 const& localpos,matrixn& jacobian, matrixn& dotjacobian);
	void calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian);
	Liegroup::dse3 calcMomentumCOM(int ichara);
	void calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const;
	void setInertia(int ichara, int ibone, ::vector3 const& localCOM, ::vector3 const& inertia);
Liegroup::dse3 calcMomentumCOMfromPose(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
Liegroup::dse3 calcMomentumCOMfromPoseUpper(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
Liegroup::dse3 calcMomentumCOMfromPoseLower(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
	virtual void calcMassMatrix(int ichara,matrixn& M, vectorn& b );
	// output is compatible to MotionDOF class.
	virtual void getLinkData(int ichara, LinkDataType t, vectorn& out);
	virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in);
	void test(const char* test, matrixn& out);
};

}
#endif
