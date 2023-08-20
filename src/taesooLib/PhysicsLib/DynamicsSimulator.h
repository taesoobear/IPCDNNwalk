#ifndef DYNAMICS_SIMULATOR_H
#define  DYNAMICS_SIMULATOR_H
#pragma once

#include "OpenHRPcommon.h"
#include "../BaseLib/motion/VRMLloader.h"
#include "../BaseLib/motion/Liegroup.h"
class VRMLloader;
class VRMLTransform;
class vector3;
class vectorn;
class BoneForwardKinematics;
namespace OpenHRP
{
	// taesoo. global parameters.
	namespace globals
	{
		extern bool usePenaltyMethod;
	}

	struct CharacterInfo;
	class CollisionDetector;
	
	class DynamicsSimulator
	{
	protected:
		CollisionDetector* collisionDetector;

		CollisionSequence* collisions;

	public:
		inline CollisionDetector* getCollisionDetector() { return collisionDetector;}
		inline CollisionSequence* getCollisionSequence() { return collisions;}

		struct CQInfo
		{
			VRMLTransform* bone;
			bool contact;
			m_real depth;
			m_real coef;
		};
		std::vector<CQInfo> _contactQueryBones;
		struct ContactForce {
			int chara;
			VRMLTransform* bone;
			vector3 f;
			vector3 p;
			vector3 tau;	// torque
		};
		std::vector<ContactForce> _contactForces;
		void setFrictionCoef(int contactQueryIndex, double coef);
		std::vector<ContactForce> & queryContactAll();
		intvectorn _boneToCQindex;
		virtual void _calcContactForce(CollisionSequence& c);
		// f and p in joint local coordinate.
		void _addContactForceToLink(int ichara, VRMLTransform* bone, vector3 const& p, vector3 const& f, vector3 const& tau);

		virtual void _updateCharacterPose(); // this function is not thread-safe. Implement your own version for thread safety.
		// DynamicSimulator maintains a copy of skeleton (all of local and global coordinates of joints) - These are used for the collision detection, coordinate transformation, and calculations of COM and ZMP in a simulator-independent manner.

		struct Character
		{
			Character(VRMLloader* skel);
			~Character();
			VRMLloader* skeleton;
			BoneForwardKinematics* chain;
			void setChain(const vectorn& poseDOF);
			void getChain(vectorn& poseDOF);
			std::string name;
			private:
			mutable vectorn _tempPose; // last simulated pose
			friend class DynamicsSimulator;
		};
		std::vector<Character*> _characters;
		virtual const vectorn & getLastSimulatedPose(int ichara=0) const;

		DynamicsSimulator(bool useSimpleColdet=true);
		DynamicsSimulator(const char* collisionDetectorType);
		virtual ~DynamicsSimulator();
		enum IntegrateMethod { EULER, RUNGE_KUTTA };

		void registerCharacter(VRMLloader*l);
		void createObstacle(OBJloader::Geometry const& mesh);
		void createFreeBody(OBJloader::Geometry const& mesh);

		enum JointDriveMode {
			HIGH_GAIN_MODE,
			TORQUE_MODE
		};

		virtual void drawDebugInformation() {}

		virtual vector3 calculateCOM(int ichara, double& outputTotalMass);
		virtual vector3 calculateCOMvel(int ichara, double& outputTotalMass);
		virtual vector3 calculateCOMacc(int ichara) { Msg::error("calculateCOMacc not implemented yet"); return vector3(0,0,0);}
		vector3 calculateZMP(int ichara);
		/* dw, dv, tau, fext in world coordinate
 		  |       |   | dw   |   |    |   | tauext    |
		  | out_M | * | dv   | + | b1 | = | fext      |
		  |       |   |ddq   |   |    |   | u         |
		  */
		virtual void calcMassMatrix(int ichara, matrixn&, vectorn& ){ Msg::error("calcMassMatrix not implemented yet");}

		// assumes that dq contains global angular/linear velocity of the root joint (unlike gmbs implementation's calcJacobian)
		virtual void calcJacobianAt(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos){ Msg::error("calcJacobian not implemented yet");}
		virtual void calcDotJacobianAt(int ichar, int ibone, matrixn& dotjacobian, vector3 const& localpos){ Msg::error("calcDotJacobian not implemented yet");}

		virtual void calcDotJacobian(int ichar, int ibone, matrixn& dotjacobian){ Msg::error("calcDotJacobian not implemented yet");}
		void getWorldPosition(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3& globalpos) const;
		
		// local pos: local to joint frame. 
		virtual void getWorldVelocity(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& velocity) const=0;
		virtual void getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const { Msg::msgBox("Error! getWorldAcceleration not implemented.");}
		virtual void getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
			{ Msg::msgBox("Error! getWorldAngVel not implemented.");}
		
		virtual void getWorldAngAcc(int ichara, VRMLTransform* b, ::vector3& angacc) const
			{ Msg::msgBox("Error! getWorldAngAcc not implemented.");}
		::vector3 getWorldPosition(int ichara, VRMLTransform* b, ::vector3 const& localpos) const;
		::vector3 getWorldVelocity(int ichara,VRMLTransform* b, ::vector3 const& localpos) const;
		::vector3 getWorldAcceleration(int ichara,VRMLTransform* b, ::vector3 const& localpos) const;
		::vector3 getWorldAngVel(int ichara, VRMLTransform* b) const;
		::vector3 getWorldAngAcc(int ichara, VRMLTransform* b) const;

		virtual void getBodyVelocity(int chara, VRMLTransform* b, Liegroup::se3& V) const { Msg::msgBox("Error! getBodyVelocity not implemented yet");}

		virtual void addRelativeConstraint(int ichara, Bone& bone1,::vector3 boneVector1,Bone& bone2,::vector3 boneVector2) { Msg::msgBox("Error! addRelativeConstraint not implemented.");}
		virtual void removeRelativeConstraint(int ichara, Bone& bone1, Bone& bone2) { Msg::msgBox("Error! removeRelativeConstraint not implemented.");}

		// both the force and its position are local.
		virtual void addForceToBone(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& localforce);
		// default implementation uses the function above, but you can override this to speedup.
		virtual void addGlobalForceToBone(int ichara, int treeindex, ::vector3 const& globalpos, ::vector3 const& globalforce);


		virtual void _registerCharacter(
				const char *name,
				CharacterInfo const& cinfo)=0;

		// param[0]=staticFriction
		// param[1]=slipFriction
		// param[2]=penaltyForceStiffness
		// param[3]=penaltyForceDamp
		virtual void registerCollisionCheckPair(
				const char* char1, 
				const char* name1, 
				const char* char2,
				const char* name2,
				vectorn const& param)=0;
		virtual void registerCollisionCheckPair( int ichara1, const char* bone1, int ichara2, const char* bone2, vectorn const& param)
		{ registerCollisionCheckPair( name(ichara1).c_str(), bone1, name(ichara2).c_str(), bone2, param); }

		// initialize everything. This function should be called at least once.
		virtual void init(
				double timeStep,
				OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)=0;
		
		virtual double currentTime() const =0;
		virtual void setCurrentTime(double t){ Msg::error("setCurrentTime not implmented yet");}

		virtual void setTimestep(double timeStep)
		{
			throw std::runtime_error("setTimestep not implemented yet");
		}
		virtual double getTimestep()
		{
			throw std::runtime_error("setTimestep not implemented yet");
			return 0;
		}
		// initialize temporary data so that simulation can start 
		// while preserving JOINT_VALUES and JOINT_VELOCITIES 
		virtual void initSimulation()=0;

		virtual void setGVector(const vector3& wdata)=0;

		virtual bool stepSimulation()=0;

		// mostly for debugging purpose. 
		virtual void setSimulatorParam(const char* string, vectorn const& value){}
		
		enum LinkDataType {
			JOINT_VALUE,		// double (joint angle or translation)
			JOINT_VELOCITY,		// double 
			JOINT_ACCELERATION,	// double
			JOINT_TORQUE,		// double (joint torque or force)
		};

		// output is compatible to MotionDOF class.
		// in case of JOINT_VALUE, you can use getWorldState which is more efficient.
		virtual void getLinkData(int i, LinkDataType t, vectorn& out)=0;	
		
		// after chainging JOINT_VALUE, you usually need to call initSimulation().
		virtual void setLinkData(int i, LinkDataType t, vectorn const& in)=0;

		// shortcuts
		inline void setPoseDOF(int ichara, vectorn const& v) { setLinkData(ichara, OpenHRP::DynamicsSimulator::JOINT_VALUE, v);}
		inline void getPoseDOF(int ichara, vectorn & v) const { ((DynamicsSimulator *)this)->getLinkData(ichara, OpenHRP::DynamicsSimulator::JOINT_VALUE, v);}
		inline vectorn getPoseDOF(int ichara) const { vectorn v; getPoseDOF(ichara, v);return v;}
		inline void setDPoseDOF(int ichara, vectorn const& v) { setLinkData(ichara, OpenHRP::DynamicsSimulator::JOINT_VELOCITY, v);}
		inline void getDPoseDOF(int ichara, vectorn & v)  const{ ((DynamicsSimulator *)this)->getLinkData(ichara, OpenHRP::DynamicsSimulator::JOINT_VELOCITY, v);}
		inline vectorn getDPoseDOF(int ichara) const { vectorn v; getDPoseDOF(ichara,  v);return v;}

		// SDFAST style packing (x,y,z,qx,qy,qz,theta1,theta2,...,thetaN,qw), which is different from the MotionDOF/PoseDOF format  (x,y,z,qw,qx,qy,qz, theta1, ..., thetaN)
		virtual void setQ(int ichara, vectorn const& v);
		virtual void getQ(int ichara, vectorn & v) const ;
		inline vectorn getQ(int ichara) const { vectorn v; getQ(ichara, v);return v;}
		//   (wx, wy, wz, vx, vy, vz, dtheta1, dtheta2, ...,dthetaN). w, v is in the global coordinate unlike dpose.
		virtual void setDQ(int ichara, vectorn const& v);
		virtual void getDQ(int ichara, vectorn& v) const;
		inline vectorn getDQ(int ichara) const { vectorn v; getDQ(ichara, v); return v;}
		virtual void setU(int ichara, const vectorn& in);
		///////////////////////////////////////////////
		// utilities
		///////////////////////////////////////////////

		// you can modify WorldState. 
		BoneForwardKinematics& getWorldState(int ichara) ;	
		const BoneForwardKinematics& getWorldState(int ichara) const ;	
		// after modifying WorldState, call this.
		void setWorldState(int ichara);		

		int findCharacter(const char* _name) ;
		VRMLloader & skeleton(int ichara) { RANGE_ASSERT(ichara<_characters.size()); return *_characters[ichara]->skeleton;}
		std::string name(int ichara) const { RANGE_ASSERT(ichara<_characters.size()); return _characters[ichara]->name;}
		const VRMLloader & skeleton(int ichara) const { RANGE_ASSERT(ichara<_characters.size()); return *_characters[ichara]->skeleton;}
		int numSkeleton() const { return _characters.size();}
	
		// num DOFs including redundant coordinates (w)
		inline int rdof(int ichar=0) const { return skeleton(ichar).dofInfo.numDOF();}
		inline int dof(int ichar=0) const { return skeleton(ichar).dofInfo.numActualDOF();}
		inline int numSphericalJoints(int ichara) const { return skeleton(ichara).dofInfo.numSphericalJoint();}

		void registerContactQueryBone(int contactQueryIndex, VRMLTransform* bone);
		bool queryContact(int index);
		vectorn queryContacts();
		vectorn queryContactDepths();
		void _updateContactInfo(CollisionSequence& corbaCollisionSequence);
		inline void _updateContactInfo() { _updateContactInfo(*collisions);}
		std::vector<CQInfo> const&	queryContactInfoAll();
		// for QPservo
		void getContactLinkBoneIndex(int ipair, intvectorn & ibone);
		int getNumAllLinkPairs() const;

		Liegroup::dse3 calcMomentumCOM(int ichara);
		Liegroup::dse3 calcMomentumCOMfromPose(int ichara, double delta_t, vectorn const& poseFrom, vectorn const& poseTo);
		void  calcInertia(int ichara,vectorn const& pose, vectorn& inertia) const;
		virtual void drawLastContactForces(int ichara=0, vector3 const& draw_offset=::vector3(0,0,0)) const {}
		virtual ::vector3 getContactForce(int ichar, int ibone) const {return vector3(-1,-1,-1);}
	protected:
		vectorn& _getLastSimulatedPose(int ichara=0) const { 
			return _characters[ichara]->_tempPose;
		}
	};
}
#endif
