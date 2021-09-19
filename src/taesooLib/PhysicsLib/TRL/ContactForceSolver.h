// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** \file
    \author Shin'ichiro Nakaoka
*/

#ifndef TCONTACT_FORCE_SOLVER_H_INCLUDED
#define TCONTACT_FORCE_SOLVER_H_INCLUDED

#define TWO_VECTORS

#include "../OpenHRPcommon.h"
#include "Body.h"
namespace TRL
{
	
	class Link;
	class WorldBase;

	class MCPsolver
	{
		inline void solveMCPByProjectedGaussSeidelInitial (const int numIteration);
		inline void solveMCPByProjectedGaussSeidelMain (const int numIteration);
		inline double solveMCPByProjectedGaussSeidelErrorCheck ();
		double* _M;
		double* _b;
		double* _x;
		double* _CI2Mu;
		double* _mcpHi;
		int _stride;
		
	protected:
		// for fast indexing (without error-checking at all). 
		inline double& M(int i, int j) { 
#ifdef _DEBUG
			int dimLCP=globalNumConstraintVectors+globalNumFrictionVectors;
			ASSERT(0<=i && i<dimLCP) ;
			ASSERT(0<=j && j<dimLCP) ;
#endif
			return *(_M+_stride*i+j);
		}
		inline double& b(int j) { 
			ASSERT(0<=j && j<globalNumConstraintVectors+globalNumFrictionVectors) ;
			return *(_b+j); 
		}
		inline double& x(int j) { 
			ASSERT(0<=j && j<globalNumConstraintVectors+globalNumFrictionVectors) ;
			return *(_x+j); 
		}
		inline double& mcpHi(int j) { 
			ASSERT(0<=j && j<globalNumConstraintVectors);
			return *(_mcpHi+j); 
		}
		inline double& contactIndexToMu(int j) { 
			ASSERT(0<=j && j<globalNumConstraintVectors);
			return *(_CI2Mu+j); 
		}

		void initWorkspace();
	public:
		int  maxNumGaussSeidelIteration;
		int  numGaussSeidelInitialIteration;
		/**
		   globalNumConstraintVectors = globalNumContactNormalVectors + globalNumConnectionVectors
		*/
		int globalNumConstraintVectors;

		int globalNumContactNormalVectors;
		int globalNumConnectionVectors;
		int globalNumFrictionVectors;

		int prevGlobalNumConstraintVectors;
		int prevGlobalNumFrictionVectors;

        int numUnconverged;
		double gaussSeidelMaxRelError;
		// for special version of gauss sidel iterative solver
		vectorn _mem_contactIndexToMu;
		vectorn _mem_mcpHi;

		MCPsolver();
		// Mlcp * force + b = acceleration
		// Mlcp * solution + b   _|_  solution

		typedef matrixn rmdmatrix;
		

		// constant vector of LCP
		void solveMCPByProjectedGaussSeidel (const rmdmatrix& M, const vectorn& b, vectorn& x);
	};
	
    class ContactForceSolver : public MCPsolver
    {
		std::vector<int> frictionIndexToContactIndex;
		bool USE_SMOOTHED_CONTACT_DYNAMICS;
    public:
		void setUseSmoothedContactDynamics(bool b) {USE_SMOOTHED_CONTACT_DYNAMICS=b;}
		
		WorldBase& world;
        ContactForceSolver(WorldBase& _world);
        ~ContactForceSolver();
		
        bool addCollisionCheckLinkPair
		(int bodyIndex1, Link* link1, int bodyIndex2, Link* link2, double muStatic, double muDynamic, double epsilon);

		void setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError);

		void initialize(void);
        void solve(OpenHRP::CollisionSequence& corbaCollisionSequence);
		void clearExternalForces();
		//struct ConstraintPoint {
		class ConstraintPoint {
			public:
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
#ifdef TWO_VECTORS
			::vector3 _frictionVector0[2];
#else
			::vector3 _frictionVector0;
#endif
			inline ::vector3 frictionVector(int i, int j) { 
#ifndef TWO_VECTORS
				if(j==0) 
				{
					if(i==0)
						return _frictionVector0; 
					return cross(_normalTowardInside0, _frictionVector0); 
				}
				if(i==0)
					return -_frictionVector0; 
				return cross(_frictionVector0, _normalTowardInside0) ; 
#else
				if(j==0)
					return _frictionVector0[i]; 
				return -_frictionVector0[i]; 
#endif
			}
			
        };
		typedef std::vector<ConstraintPoint> ConstraintPointArray;

		struct LinkData
		{
			::vector3	dvo;
			::vector3	dw;
			::vector3	pf0;
			::vector3	ptau0;
			double  uu;
			double  uu0;
			double  ddq;
			int     numberToCheckAccelCalcSkip;
			int     parentIndex;
			Link*   link;
		};
		typedef std::vector<LinkData> LinkDataArray;

		struct BodyData
		{
			Body* body;
			bool isStatic;
			bool hasConstrainedLinks;
			bool isTestForceBeingApplied;
			LinkDataArray linksData;

			::vector3 dpf;
			::vector3 dptau;
		};

		std::vector<BodyData> bodiesData;

		class LinkPair {
			public:

			int index;
			bool isSameBodyPair;
			int bodyIndex[2];
			BodyData* bodyData[2];
			Link* link[2];
			LinkData* linkData[2];
			ConstraintPointArray constraintPoints;

			double muStatic;
			double muDynamic;
			double epsilon;

			Body::LinkConnection* connection;


			public:

			int getBodyIndex(int i){ return bodyIndex[i];}
			ConstraintPointArray getConstraintPointArray(){
				return constraintPoints; 
			}
			
		};
		typedef std::vector<LinkPair> LinkPairArray;

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
		void setAccelCalcSkipInformation();
		void setDefaultAccelerationVector();
		void setAccelerationMatrix();
		void initABMForceElementsWithNoExtForce(BodyData& bodyData);
		void calcABMForceElementsWithTestForce(BodyData& bodyData, Link* linkToApplyForce, const ::vector3& f, const ::vector3& tau);
		void calcAccelsABM(BodyData& bodyData, int constraintIndex);

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

	};
};


#endif

