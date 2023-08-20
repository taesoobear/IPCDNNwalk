/** \file
    \author Taesoo  Kwon
*/

#ifndef T_LCP_SOLVER_H_INCLUDED
#define T_LCP_SOLVER_H_INCLUDED


#include "../OpenHRPcommon.h"
//#include "DynamicsSimulator_Trbdl_LCP.h"
#include "../TRL/ContactForceSolver.h"
namespace TRL
{
	
	class MCPsolver;
}
namespace Trbdl
{
	class DynamicsSimulator_Trbdl_LCP;

	class GWorld;
	class GBody;

	// body.calcAccelsWithoutExtForce()
	// link->getAcceleration(point)
	// body.isTestForceBeingApplied=true
	// loop
	// 	  link->addTestForce (f, p)
	// body.calcAccels()
	class GLink {
		// A single rigidBody or a softBody
		//
		public:

			GLink(GBody* _body, int _index) { body=_body; index=_index;}
			virtual ~GLink(){}
			int index;
			GBody* body;

			// reimplement at least the following functions!!!
			virtual void addTestForce(vector3 const& f, vector3 const& contactPoint,int nodeIndex){}
			virtual vector3 getVelocity(vector3 const& contactPoint, int nodeIndex){ return vector3(0,0,0);} // by default, static object is assumed.
			virtual vector3 getAcceleration(vector3 const& contactPoint, int nodeIndex){ return vector3(0,0,0); }
			virtual void addConstraintForce(vector3 const& contactPoint, int nodeIndex, vector3 const& f){}
			virtual bool isStatic(){ return true;}
			virtual bool hasCustomFricCoef() { return false;} // by default, use the friction coef specified in the linkpair
			virtual void getCustomFrictionCoef(vector3 const& contactPoint, int nodeIndex, bool isSlipping, double& mu){}
			
			// optionally reimplement these too.
			virtual void updateSkipInformation(GBody* parentbody, int constraintIndex ){
			}
	};
	
	class GBody {
		protected:
			virtual bool _isStatic() {return true;}

		public:
			class LinkConnection {
			};
			std::vector<GLink*> linksData;
			bool _precomputed;
			GBody(){}
			virtual ~GBody(){}
			virtual GLink* getArticulatedBodyLink(int j) { return linksData[j];}
			
			// the following three variables are used internally. you do not need to manually set them.
			bool isStatic;
			bool hasConstrainedLinks; 
			bool isTestForceBeingApplied;

			virtual void testForceFinished() { isTestForceBeingApplied=false;}
			int numLinks(){return linksData.size();}


			virtual void initialize(){
				hasConstrainedLinks=false;
				isTestForceBeingApplied=false;
				_precomputed=false;
				isStatic=_isStatic();
			}
			// reimplement at least the following functions!!!
			virtual void calcAccelsWithoutExtForce(){}
			virtual void renderme(){}
			// should work with or without testForce being applied
			virtual void calcAccels(){}


			// optionally reimplement these too.
			virtual void clearSkipInformation(){}
	};

	class GWorld {
		protected:
	std::vector<GBody*> _bodies;
		public:
	int numBodies() { return _bodies.size();}
	inline std::vector<GBody*>& bodiesData() {return _bodies;}
	inline GBody* body(int bodyIndex){ return _bodies[bodyIndex];}

			virtual double _getTimestep() const { return 0;}
	};
    class LCPsolver : public TRL::MCPsolver
    {
		std::vector<int> frictionIndexToContactIndex;
    public:
		
		DynamicsSimulator_Trbdl_LCP& world;
        LCPsolver(DynamicsSimulator_Trbdl_LCP& _world);
        ~LCPsolver();
		
        bool addCollisionCheckLinkPair
		(int bodyIndex1, GLink* link1, int bodyIndex2, GLink* link2, double muStatic, double muDynamic, double epsilon);

		void setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError);

		void initialize(void);
        bool solve(OpenHRP::CollisionSequence& corbaCollisionSequence);
		struct ConstraintPoint {
			int nodeIndex[2]; // goes to getVelocity and getAcceleration functions. use as you want.
            int globalIndex;
			::vector3 point;
            ::vector3 _normalTowardInside0;
            inline ::vector3 normalTowardInside(int i) { if(i==0) return _normalTowardInside0; return -_normalTowardInside0; }
			::vector3 defaultAccel[2];
			double normalProjectionOfRelVelocityOn0;
			double depth; // position error in the case of a connection point

			double mu;
			::vector3 relVelocityOn0;
			int globalFrictionIndex;
			int numFrictionVectors;
			::vector3 _frictionVector0[2];
			inline ::vector3 frictionVector(int i, int j) { 
				if(j==0)
					return _frictionVector0[i]; 
				return -_frictionVector0[i]; 
			}
        };
		typedef std::vector<ConstraintPoint> ConstraintPointArray;

		std::vector<GBody*> &bodiesData;

		struct LinkPair {

			int index;
			bool isSameBodyPair;
			int bodyIndex[2];
			GBody* bodyData[2];
			GLink* linkData[2];
			ConstraintPointArray constraintPoints;

			double muStatic;
			double muDynamic;
			double epsilon;

			TRL::Body::LinkConnection* connection;

		};
		typedef std::vector<LinkPair> LinkPairArray;
		const ConstraintPoint* findConstraint(int globalIndex) const;

		LinkPairArray collisionCheckLinkPairs;
		LinkPairArray connectedLinkPairs;

		std::vector<LinkPair*> constrainedLinkPairs;

		double kappa;
		double epsilon;
		double _R;

		// constant acceleration term when no external force is applied
		vectorn an0;
		vectorn at0;


		// contact force solution: normal forces at contact points
		vectorn solution;


		matrixn Mlcp;
		vectorn B;

		void setConstraintPoints(OpenHRP::CollisionSequence& collisions);
		void setContactConstraintPoints(LinkPair& linkPair, OpenHRP::CollisionPointSequence& collisionPoints);
		void setFrictionVectors(ConstraintPoint& constraintPoint);
		bool setConnectionConstraintPoints(LinkPair& linkPair);
        void putContactPoints();
		void initMatrices();
		void setDefaultAccelerationVector();
		void setAccelerationMatrix();
		/*
		void initABMForceElementsWithNoExtForce(BodyData& bodyData);
		void calcABMForceElementsWithTestForce(BodyData& bodyData, Link* linkToApplyForce, const ::vector3& f, const ::vector3& tau);
		void calcAccelsABM(BodyData& bodyData, int constraintIndex);
		*/

		void extractRelAccelsOfConstraintPoints
		(matrixn& Kxn, matrixn& Kxt, int testForceIndex, int constraintIndex);

		void extractRelAccelsFromLinkPairCase1
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);
		void extractRelAccelsFromLinkPairCase2
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int constraintIndex);
		void extractRelAccelsFromLinkPairCase3
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);

		void clearSingularPointConstraintsOfClosedLoopConnections();
		
		void setConstantVectorAndMuBlock();
		void addConstraintForceToLinks();
		void addConstraintForceToLink(LinkPair* linkPair, int ipair);
		
		void setAccelCalcSkipInformation();
		//modify
		//bool addConnectedLinkPair(int bodyIndex1, GLink* gLink1, int bodyIndex2, GLink* gLink2,double muStatic, double muDynamic, double epsilon,TRL::Link* link1, TRL::Link* link2);
	};
};


#endif

