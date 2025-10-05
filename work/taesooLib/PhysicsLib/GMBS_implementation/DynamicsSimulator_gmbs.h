// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

#ifndef DYNAMICSSIMULATOR_gmbs_HEADER
#define DYNAMICSSIMULATOR_gmbs_HEADER



#include "../CollisionDetector.h"
#include "DynamicsSimulator_gmbs_penalty.h"
#include "../Liegroup.h"
#include "../DynamicsSimulator_QP.h"

namespace OpenHRP {


#ifdef USE_AIST_SIM
class DynamicsSimulator_impl;
#endif
/**
 * DynamicsSimulator_ class
 */
class DynamicsSimulator_gmbs : public DynamicsSimulator, public DynamicsSimulator_QP
								  
{
#ifdef USE_AIST_SIM
	vectorn _tempVel;
#endif
 public:
	TString _debugInfo; // only for debugging

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
#ifdef USE_AIST_SIM
	DynamicsSimulator_impl* AISTsim;
#endif
	



	std::vector<System*> _cinfo; // indexed by ichara
	
	DynamicsSimulator_gmbs(bool useSimpleColdet=false);
	DynamicsSimulator_gmbs(const char* coldettype);

	virtual ~DynamicsSimulator_gmbs();


	virtual void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const;
	virtual void getWorldVelocity(int ichara, VRMLTransform* b
								  , ::vector3 const& localpos
								  , ::vector3& velocity) const;

	virtual void getWorldAcceleration(int ichara,VRMLTransform* b
									  , ::vector3 const& localpos
									  , ::vector3& acc) const;

	double calcKineticEnergy() const;

	void setCollisionMargin(int ilinkpair, double a);
	void setAllCollisionMargin(vectorn const& a);
	void getAllCollisionMargin(vectorn & a);
	void registerCollisionCheckPair
	(
	 const char *charName1,
	 const char *linkName1,
	 const char *charName2,
	 const char *linkName2,
	 vectorn const& param
	 );
	virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const;
	virtual void getWorldAngAcc(int ichara, VRMLTransform* b, ::vector3& angacc) const;
	void addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force);

	virtual void _registerCharacter(const char *name, CharacterInfo const& cinfo);

	void inverseDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, vectorn& controlforce);
	// void hybridDynamics(vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce);
	// void hybridDynamics2(bitvectorn const& isActuated,vectorn const& theta, vectorn const& dtheta, vectorn const& desiredacceleration, std::vector<ContactForce> const& contacts, vectorn& controlforce);

	virtual void init(double timeStep,
					  OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);

	double currentTime() const;
	void setCurrentTime(double t);

	virtual void setTimestep(double timeStep);
	virtual double getTimestep() { return _timeStep;}

	virtual void initSimulation();
		
	virtual bool stepSimulation();

	// ys start
	void stepKinematicMuscle_integrate(vectorn const& ddq, double dt);
	void stepKinematicMuscle_updateother();
	// ys end

	bool stepKinematic(vectorn const& ddq, vectorn const& tau, bool globalACC=false);
	virtual void setGVector(const ::vector3& wdata);

	virtual void getGVector(::vector3& wdata);

	virtual void setCharacterAllJointModes
		(
		 const char* characterName, 
		 OpenHRP::DynamicsSimulator::JointDriveMode jointMode);

	virtual ::vector3 calculateCOM(int ichara, double& outputTotalMass); 
	virtual ::vector3 calculateCOMvel(int ichara, double& outputTotalMass);
	virtual ::vector3 calculateCOMacc(int ichara);
	void getLCPmatrix(matrixn& A, vectorn& b);
	void getLCPsolution(vectorn& out);
	void calcContactJacobianAll(matrixn &J_all, matrixn & dotJ_all, matrixn& V_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef=0.5);

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
	Liegroup::dse3 calcMomentumGlobal(int ichara);
	void calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const;
	void calcInertiaUpper(int ichara,vectorn const& pose, vectorn& inertia) const;
	void calcInertiaLower(int ichara,vectorn const& pose, vectorn& inertia) const;
	void setInertia(int ichara, int ibone, ::vector3 const& localCOM, ::vector3 const& inertia);
Liegroup::dse3 calcMomentumCOMfromPose(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
Liegroup::dse3 calcMomentumCOMfromPoseUpper(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
Liegroup::dse3 calcMomentumCOMfromPoseLower(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo);
	virtual void calcMassMatrix(int ichara,matrixn& );
	void calcMassMatrix2(int ichara,matrixn&, vectorn& );
	void calcMassMatrix3(int ichara,matrixn&, vectorn& );
	void collectExternalForce(int ichara, vectorn & extForce);

	// output is compatible to MotionDOF class.
	virtual void getLinkData(int ichara, LinkDataType t, vectorn& out);
	virtual void setLinkData(int ichara, LinkDataType t, vectorn const& in);
	void test(const char* test, matrixn& out);
};

}
#endif
