/** \file
	\brief Implementation of LCPsolver class
	\author Taesoo Kwon
*/
#define ASSERT(x) 
#define RANGE_ASSERT(x) 

#include "../physicsLib.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
#if 1
// to disable profiling, set 1 above.
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#endif
#ifdef __WIN32__
#define NOMINMAX
#endif

#include "../TRL/ContactForceSolver.h"
#include "LCPsolver.h"

#include <limits>

#include "../OpenHRPcommon.h"
#include "../TRL/eigenSupport.h"
//#include "../TRL/Link.h"
//#include "../TRL/Body.h"
#include "DynamicsSimulator_Trbdl_LCP.h"


inline double norm2(const vector3& v) { return v.length();}

// settings

static const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-2;

//static const bool ENABLE_STATIC_FRICTION = true;
//static const bool ONLY_STATIC_FRICTION_FORMULATION = (true );
static const bool STATIC_FRICTION_BY_TWO_CONSTRAINTS = true;
static const bool IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION = false;

static const bool ENABLE_TRUE_FRICTION_CONE =
	(true && STATIC_FRICTION_BY_TWO_CONSTRAINTS);
static const bool SKIP_REDUNDANT_ACCEL_CALC = true; // set same as the one in Trbdl_LCP.cpp
static const bool USE_SMOOTHED_CONTACT_DYNAMICS = true;
static const bool USE_PREVIOUS_LCP_SOLUTION = false;

static const bool ALLOW_SUBTLE_PENETRATION_FOR_STABILITY = true;
static const double ALLOWED_PENETRATION_DEPTH = 1.0e-2;
static const double NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION = 10.0; 


using namespace OpenHRP;
using namespace Trbdl;
using namespace std;

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923
//#define VERBOSE
namespace TRL
{

#ifdef __WIN32__
	const double CFSImpl::PI   = 3.14159265358979323846;
	const double CFSImpl::PI_2 = 1.57079632679489661923;
#endif
};

bool LCPsolver::addCollisionCheckLinkPair
(int bodyIndex1, GLink* link1, int bodyIndex2, GLink* link2, double muStatic, double muDynamic, double epsilon)
{
	int n = collisionCheckLinkPairs.size();
	int index=n;
	collisionCheckLinkPairs.resize(index+1);

	LinkPair& linkPair = collisionCheckLinkPairs[index];

	linkPair.isSameBodyPair = (bodyIndex1 == bodyIndex2);
	linkPair.bodyIndex[0] = bodyIndex1;
	linkPair.linkData[0] = link1;
	linkPair.bodyIndex[1] = bodyIndex2;
	linkPair.linkData[1] = link2;
	linkPair.index = index;
	linkPair.muStatic = muStatic;
	linkPair.muDynamic = muDynamic;
	linkPair.epsilon = epsilon;
	linkPair.connection = 0;

    return (index >= 0 );
}

/*
//modify
bool LCPsolver::addConnectedLinkPair
(int bodyIndex1, GLink* gLink1, int bodyIndex2, GLink* gLink2,double muStatic,double muDynamic,double epsilon, TRL::Link* link1, TRL::Link* link2)
{
	int n = connectedLinkPairs.size();
	int index=n;

	TRL::Body::LinkConnection* linkCon=new TRL::Body::LinkConnection;
	//focus
	linkCon->link[0]=link1;
	linkCon->link[1]=link2;
	linkCon->point[0]=vector3(0,0,0);
	linkCon->point[1]=vector3(0,0,0);
	linkCon->numConstraintAxes=3;
	linkCon->constraintAxes[0]=vector3(1,0,0);
	linkCon->constraintAxes[1]=vector3(0,1,0);
	linkCon->constraintAxes[2]=vector3(0,0,1);

	connectedLinkPairs.push_back(LinkPair());
	LinkPair& linkPair = connectedLinkPairs.back();

	linkPair.index=index;
	linkPair.isSameBodyPair=(bodyIndex1==bodyIndex2);
	linkPair.bodyIndex[0]=bodyIndex1;
	linkPair.bodyIndex[1]=bodyIndex2;
	linkPair.linkData[0]=gLink1;
	linkPair.linkData[1]=gLink2;
	//linkPair.linkData[0]=(bodiesData[bodyIndex1]->linksData[gLink1->index]);
	//linkPair.linkData[1]=(bodiesData[bodyIndex2]->linksData[gLink2->index]);
	linkPair.bodyData[0]=bodiesData[bodyIndex1];
	linkPair.bodyData[1]=bodiesData[bodyIndex2];
	linkPair.muStatic=0;
	linkPair.muDynamic=0;
	linkPair.epsilon=epsilon;
	linkPair.connection=linkCon;
	linkPair.constraintPoints.resize(linkCon->numConstraintAxes);

	cout<<bodiesData[bodyIndex1]->numLinks()<<"numLinks"<<endl;
	//bodiesData[bodyIndex1]->linkConnections.push_back(linkCon);
	
	return (index >= 0);
}
*/

void LCPsolver::initialize(void)
{

	int numBodies = world.numBodies();


	connectedLinkPairs.clear();

	for(int bodyIndex=0; bodyIndex < numBodies; ++bodyIndex){

		Trbdl::GBody* body = world.body(bodyIndex);
		
		
		body->initialize();

		//cout <<"body"<<bodyIndex<< ":"<< body->isStatic<<endl;
		//cout <<"body"<<((GArticulatedBody*)body)->_allBodiesFixed<<endl;
		/*

		// initialize link connection
		TRL::BodyPtr bodyptr=world.world.body(bodyIndex);
		TRL::Body::LinkConnectionArray& connections = bodyptr->linkConnections;
		for(size_t j=0; j < connections.size(); ++j){

			connectedLinkPairs.push_back(LinkPair());
			LinkPair& linkPair = connectedLinkPairs.back();

			TRL::Body::LinkConnection& connection = connections[j];
			linkPair.connection = &connection;
			linkPair.isSameBodyPair = true;
			linkPair.constraintPoints.resize(connection.numConstraintAxes);

			for(int k=0; k < connection.numConstraintAxes; ++k){
				ConstraintPoint& constraint = linkPair.constraintPoints[k];
				constraint.numFrictionVectors = 0;
				constraint.globalFrictionIndex = numeric_limits<int>::max();
			}

			for(int k=0; k < 2; ++k){
				linkPair.bodyIndex[k] = bodyIndex;
				linkPair.bodyData[k] = body;

				ASSERT(linkPair.bodyData[k]);
				TRL::Link* link = connection.link[k];
				linkPair.linkData[k] = body->getArticulatedBodyLink(link->index);
			}
		}
		*/
	}

	int numLinkPairs = collisionCheckLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = collisionCheckLinkPairs[i];
		for(int j=0; j < 2; ++j){
			GBody* bodyData = bodiesData[linkPair.bodyIndex[j]];
			linkPair.bodyData[j] = bodyData;
		}
	}


	prevGlobalNumConstraintVectors = 0;
	prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

}




bool LCPsolver::solve(CollisionSequence& corbaCollisionSequence)
{
    for(size_t i=0; i < bodiesData.size(); ++i){
		bodiesData[i]->hasConstrainedLinks = false;
		bodiesData[i]->_precomputed = false;
    }

	globalNumConstraintVectors = 0;
	globalNumFrictionVectors = 0;
	constrainedLinkPairs.clear();

	setConstraintPoints(corbaCollisionSequence);

	if(globalNumConstraintVectors > 0){
#ifdef VERBOSE
	cout <<"solve"<< globalNumConstraintVectors<<endl;
#endif
		const bool constraintsSizeChanged = ((globalNumFrictionVectors   != prevGlobalNumFrictionVectors) ||
											 (globalNumConstraintVectors != prevGlobalNumConstraintVectors));

		if(constraintsSizeChanged){
			initMatrices();
		}


		if(SKIP_REDUNDANT_ACCEL_CALC){
			setAccelCalcSkipInformation();
		}

		BEGIN_TIMER(sam1);
	    setDefaultAccelerationVector();
		END_TIMER2(sam1);
		BEGIN_TIMER(sam2);
	    setAccelerationMatrix();
		END_TIMER2(sam2);
#ifdef VERBOSE
		cout<<"mlcp"<<Mlcp<<endl<<"B"<<B<<endl;
#endif
		if(globalNumConstraintVectors - globalNumContactNormalVectors > 0){
			clearSingularPointConstraintsOfClosedLoopConnections();
		}
		
		setConstantVectorAndMuBlock();


		bool isConverged;
		if(!USE_PREVIOUS_LCP_SOLUTION || constraintsSizeChanged){
			solution.setAllValue(0.0);
		}
		// M ddq + c = Jt f
		// ddq + invM *c =  invM*Jt f
		//  J ddq + J *invM *c = J*invM*Jt   f
		// so 
		//  A == Mlcp = JinvMJ'  
		// Mlcp*f = b                   ----(5)
		
		// smoothed dynamics version:
		// (M+MA) dv + c = J'f + u, 
		// and (A+R) *f = (v_star - v_)/h  --- (6)
		//
		//  then, 
		//  dv = inv(M+MA)*(J'f + u-c)    --- (1)
		//
		//  let v' be v + dv*h. Then,
		//  
		//  v'= v + inv(M+MA)*(J'f + u-c)*h --- (2)
		//   
		//  let v^ be v+inv(M+MA) *(u-c)*h. Then
		//
		//  v' = v^ + inv(M+MA)*J'f*h        --- (3)
		//
		//  By multiplying J on both sides of (3)
		//
		//  v2 = v_ + Af *h,                  --- (4)
		//  where v2=Jv' and v_ = Jv^.
		//
		// from (4), (5) and the fact that v'=0,
		//    b = (-v_)/h
		//
		// to modify (5) to  (6),
		//
		//    b should change to   b - (-v_)/h +(v_star - v_)/h
		//                        = b + v_star/h
		// 
		//
		//  
		//	desired contact velocity v*:
		//		kp =   (1+epsilon) / (kappa*kappa);
		//		kd = 2*(1+epsilon) / kappa;
		//  	v* : (normal*(vn + h*(kp * (-0 + depth) - kd * vn)));
		//
		// A*F = v_star - Jv^ 
		// , where impulse F is f*dt
		//
		// or A * f = (v_star- v_)/dt   --- (2)
		//
		assert(Mlcp.rows()==Mlcp.cols());
		Mlcp.diag()+=_R;

		BEGIN_TIMER(mcp);
		solveMCPByProjectedGaussSeidel(Mlcp, B, solution);
		END_TIMER2(mcp);
		isConverged = true;

#if 1
		// MLCP  *solution +B  == 0 for all non-zero solutions
		// but we added diag(_R) so ...
		//
		// (MLCP+R)*solution +B == 
		// MLCP*(solution+error)+B == 0
		
		//->  MLCP*error= R*solution
		//-> error should be the minimum norm solution 
		
		BEGIN_TIMER(fixForce);

		/*
		matrixn temp;
		temp.setSameSize(Mlcp);
		int c=0;
		vectorn tempR;
		tempR.resize(Mlcp.rows());
		std::vector<int> targetIdx;
		targetIdx.resize(Mlcp.rows());

		for(int i=0; i<solution.size(); i++)
		{
			if(std::abs(solution(i))>10)
			{
				temp.row(c)=Mlcp.row(i);
				tempR(c)=_R*solution(i);
				targetIdx[c]=i;
				c++;
			}
		}
		if(c>0){
			temp.resize(c, Mlcp.cols());
			tempR.resize(c);


			Eigen::VectorXd v= eigenView(temp).colPivHouseholderQr().solve(eigenTView(tempR));
			//cout << "v="<<v<<endl;
			for(int i=0; i<c; i++)
				solution[targetIdx[i]]+=v[i];
		}
		*/
		// preserve the acceleration solution using minimum-norm corrective forces.
		Eigen::VectorXd v= eigenView(Mlcp).colPivHouseholderQr().solve(_R*eigenTView(solution));
		//cout << "v="<<v<<endl;
		for(int i=0, ni=Mlcp.rows(); i<ni; i++)
			solution[i]+=v[i];
		END_TIMER2(fixForce);
#endif
		if(!isConverged){
			++numUnconverged;
				//std::cout << "LCP didn't converge" << numUnconverged << std::endl;
		} else {

			addConstraintForceToLinks();
		}
		
	}

	prevGlobalNumConstraintVectors = globalNumConstraintVectors;
	prevGlobalNumFrictionVectors = globalNumFrictionVectors;
	return globalNumConstraintVectors>0;
}


void LCPsolver::setConstraintPoints(CollisionSequence& collisions)
{
    for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i){

        LinkPair& linkPair = collisionCheckLinkPairs[i];
		CollisionPointSequence& points = collisions[i].points;

		if(points.size() > 0){
			constrainedLinkPairs.push_back(&linkPair);
			setContactConstraintPoints(linkPair, points);
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		}
    }
	globalNumContactNormalVectors = globalNumConstraintVectors;

	//connected check 1
	for(size_t i=0; i < connectedLinkPairs.size(); ++i){
        LinkPair& linkPair = connectedLinkPairs[i];
		constrainedLinkPairs.push_back(&linkPair);
		if(setConnectionConstraintPoints(linkPair)){
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		} else {
			constrainedLinkPairs.pop_back();
		}
    }
	globalNumConnectionVectors = globalNumConstraintVectors - globalNumContactNormalVectors;
}


void LCPsolver::setContactConstraintPoints(LinkPair& linkPair, CollisionPointSequence& collisionPoints)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;
	constraintPoints.clear();
	int numExtractedPoints = 0;
	int numContactsInPair = collisionPoints.size();

	for(int j=0; j < numContactsInPair; ++j){

		CollisionPoint& collision = collisionPoints[j];
		constraintPoints.push_back(ConstraintPoint());
		ConstraintPoint& contact = constraintPoints.back();

		contact.point= collision.position+collision.normal*(collision.idepth*0.5);
		contact._normalTowardInside0=-collision.normal;
		contact.depth = collision.idepth;
		contact.nodeIndex[0]=collision.inode[0];
		contact.nodeIndex[1]=collision.inode[1];

		bool isNeighborhood = false;


		if(isNeighborhood){
			constraintPoints.pop_back();
		} else {
			numExtractedPoints++;
			contact.globalIndex = globalNumConstraintVectors++;

			// check velocities
			vector3 v[2];
			for(int k=0; k < 2; ++k){
				GLink* link = linkPair.linkData[k];
				v[k]=link->getVelocity(contact.point, contact.nodeIndex[k]);
#ifdef VERBOSE
				if (k==0)
				{
					std::cout << "index"<<link->index<<endl;
					std::cout << "Position " << contact.point << std::endl;
					std::cout << "Velocity " << v[k] << std::endl;
				}
#endif
			}
			contact.relVelocityOn0 = v[1] - v[0];
			contact.normalProjectionOfRelVelocityOn0 = dot(contact.normalTowardInside(1), contact.relVelocityOn0);

			vector3 v_tangent(contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside(1));

			contact.globalFrictionIndex = globalNumFrictionVectors;

			double vt_square = dot(v_tangent, v_tangent);
			static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
			bool isSlipping = (vt_square > vsqrthresh);
			contact.mu = isSlipping ? linkPair.muDynamic : linkPair.muStatic;
			contact.numFrictionVectors = (STATIC_FRICTION_BY_TWO_CONSTRAINTS ? 2 : 4);
			setFrictionVectors(contact);
#ifdef VERBOSE
			cout <<"SFV"<< contact.relVelocityOn0 <<","<<v_tangent <<endl;
			cout <<contact.numFrictionVectors <<","<<contact._frictionVector0[0] 
			<<","<<contact._frictionVector0[1] <<endl;
#endif
			globalNumFrictionVectors += contact.numFrictionVectors;
		}
	}
}


void LCPsolver::setFrictionVectors(ConstraintPoint& contact)
{
	vector3 u(0.0);
	int minAxis = 0;
	vector3 normal = contact.normalTowardInside(0);

	for(int i=1; i < 3; i++){
		if(fabs(normal[i]) < fabs(normal[minAxis])){
			minAxis = i;
		}
	}
	u[minAxis] = 1.0;

	vector3 t1(cross(normal, u));
	t1 /= norm2(t1);

#ifdef TWO_VECTORS
	contact._frictionVector0[0] = t1;
	contact._frictionVector0[1] =cross(normal, t1);
#else
	contact._frictionVector0 = t1;
#endif
}


bool LCPsolver::setConnectionConstraintPoints(LinkPair& linkPair)
{
	/*
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	TRL::Body::LinkConnection* connection = linkPair.connection;

	TRL::Link* link0 = connection->link[0];
	TRL::Link* link1 = connection->link[1];

	vector3 point[2];
	point[0] = link0->p + link0->R * connection->point[0];
	point[1] = link1->p + link1->R * connection->point[1];
	vector3 midPoint((point[0] + point[1]) / 2.0);
	vector3 error(midPoint - point[0]);

	if(dot(error, error) > (0.04 * 0.04)){
		//mod
		cout << "error in LCPsolver::setConnectionConstraintPoints....error!" << endl;	
		//return false;
	}

	// check velocities
	vector3 v[2];
	for(int k=0; k < 2; ++k){
		TRL::Link* link = connection->link[k];
		if(link->jointType == TRL::Link::FIXED_JOINT){
			v[k] = 0.0;
		} else {
			v[k] = link->vo + cross(link->w, point[k]);
		}
	}
	vector3 relVelocityOn0(v[1] - v[0]);

	for(int i=0; i < connection->numConstraintAxes; ++i){
		ConstraintPoint& constraint = constraintPoints[i];
		constraint.point = midPoint;
		const vector3 axis(link0->R * connection->constraintAxes[i]);
		constraint._normalTowardInside0 =  axis;
		constraint.depth = dot(axis, error);
		constraint.globalIndex = globalNumConstraintVectors++;
		constraint.normalProjectionOfRelVelocityOn0 = dot(constraint.normalTowardInside(1), relVelocityOn0);
	}
	*/
	return true;
}





void LCPsolver::initMatrices()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;

	const int dimLCP = n + m;

	Mlcp.setSize(dimLCP, dimLCP);
	B.setSize(dimLCP);
	solution.setSize(dimLCP);


	{
		frictionIndexToContactIndex.resize(m);
	}

	an0.setSize(n);
	at0.setSize(m);

	MCPsolver::initWorkspace();
}
const LCPsolver::ConstraintPoint* LCPsolver::findConstraint(int globalIndex) const
{
	// add the number of contact points to skip check numbers of the links from a contact target to the root
	int numLinkPairs = constrainedLinkPairs.size();
	for(int i=0; i < numLinkPairs; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];

		//int constraintIndex = linkPair->constraintPoints.front().globalIndex;
		//mod
		int constraintIndex;
		if(linkPair->constraintPoints.size()!=0)
			constraintIndex=linkPair->constraintPoints.front().globalIndex;

		if(constraintIndex==globalIndex)
			return &linkPair->constraintPoints.front();
	}
	Msg::error("need to perform a full-search here, but this isn't supposed to happen");
	return NULL;
}

void LCPsolver::setAccelCalcSkipInformation()
{
	// clear skip check numbers
	for(size_t i=0; i < bodiesData.size(); ++i){
		GBody* bodyData = bodiesData[i];
		bodyData->clearSkipInformation();
	}

	// add the number of contact points to skip check numbers of the links from a contact target to the root
	int numLinkPairs = constrainedLinkPairs.size();
	for(int i=0; i < numLinkPairs; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];

		//int constraintIndex = linkPair->constraintPoints.front().globalIndex;
		//mod
		int constraintIndex;
		if(linkPair->constraintPoints.size()!=0)
			constraintIndex=linkPair->constraintPoints.front().globalIndex;
		else
			constraintIndex = 0;

		for(int j=0; j < 2; ++j){
			linkPair->linkData[j]->updateSkipInformation(linkPair->bodyData[j], constraintIndex);
		}
	}
}

void LCPsolver::setDefaultAccelerationVector()
{
	//check Point2
	// calculate accelerations with no constraint force
	for(size_t i=0; i < bodiesData.size(); ++i){
		GBody& bodyData = *bodiesData[i];
		if(! bodyData.isStatic){
			bodyData.calcAccelsWithoutExtForce();
		}
	}

	// extract accelerations
	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

		for(size_t j=0; j < constraintPoints.size(); ++j){
			ConstraintPoint& constraint = constraintPoints[j];

			for(int k=0; k < 2; ++k){
				if(linkPair.bodyData[k]->isStatic){
					constraint.defaultAccel[k] = 0.0;
				} else {
					GLink* link = linkPair.linkData[k];
					constraint.defaultAccel[k] =
						link->getAcceleration(constraint.point,constraint.nodeIndex[k]);
#ifdef VERBOSE
				if (k==0)
				std::cout << "DefaultAcc : dv = " << constraint.defaultAccel[k] << std::endl;
#endif
				}
			}

			vector3 relDefaultAccel(constraint.defaultAccel[1] - constraint.defaultAccel[0]);
			an0[constraint.globalIndex] = dot(constraint.normalTowardInside(1), relDefaultAccel);
			
			//mod
			if(at0.size()>0&&constraint.globalFrictionIndex!=numeric_limits<int>::max()){
					for(int k=0; k<constraint.numFrictionVectors; k++){
						at0[constraint.globalFrictionIndex+k]=dot(constraint.frictionVector(k,1), relDefaultAccel);
					}
			}

		}
	}
}


void LCPsolver::setAccelerationMatrix()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;	

	
	//mod
	matrixnView Knn=Mlcp.range(0,n,0,n);
	matrixnView Knt=Mlcp.range(n,n+m,0,n);

	//matrixnView Knn=Mlcp.range( 0,n, 0,n);
	//matrixnView Ktn=Mlcp.range( 0,n, n,n+m);
	//matrixnView Knt=Mlcp.range( n,n+m, 0,n);
	//matrixnView Ktt=Mlcp.range( n,n+m, n,n+m);

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){

			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int constraintIndex = constraint.globalIndex;

			// apply test normal force
			for(int k=0; k < 2; ++k){
				GBody& bodyData = *linkPair.bodyData[k];
				if(!bodyData.isStatic){

					bodyData.isTestForceBeingApplied = true;
					const vector3& f = constraint.normalTowardInside(k);

					{
#ifdef VERBOSE
						cout <<"testForce " << f <<endl;
#endif
						//check Point3
						linkPair.linkData[k]->addTestForce( f, constraint.point, constraint.nodeIndex[k]);
						if(!linkPair.isSameBodyPair || (k > 0)){
							bodyData.calcAccels();
						}
					}
				}
			}
			extractRelAccelsOfConstraintPoints(Knn, Knt, constraintIndex, constraintIndex);


			// apply test friction force(modify)
			if(m>0)
			{
				matrixnView Ktn=Mlcp.range(0,n,n,n+m);
				matrixnView Ktt=Mlcp.range(n,n+m,n,n+m);
				for(int l=0; l < constraint.numFrictionVectors; ++l){
					for(int k=0; k < 2; ++k){
						GBody& bodyData = *linkPair.bodyData[k];
						if(!bodyData.isStatic){
							const vector3& f = constraint.frictionVector(l,k);
							{
#ifdef VERBOSE
								cout <<"testForce " << f <<endl;
#endif
								//check Point4
								linkPair.linkData[k]->addTestForce( f, constraint.point, constraint.nodeIndex[k]);
								if(!linkPair.isSameBodyPair || (k > 0)){
									bodyData.calcAccels();
								}
							}	
						}	
					}

					//extractRelAccelsOfConstraintPoints(Ktn, Ktt, constraint.globalFrictionIndex + l, constraintIndex);
					//mod
					if(constraint.globalFrictionIndex!=numeric_limits<int>::max())
					{
						extractRelAccelsOfConstraintPoints(Ktn,Ktt,constraint.globalFrictionIndex + l, constraintIndex);
					}

				}
			}
			//check Point5
			linkPair.bodyData[0]->testForceFinished();
			linkPair.bodyData[1]->testForceFinished();
		}
	}



}




void LCPsolver::extractRelAccelsOfConstraintPoints
(matrixn& Kxn, matrixn& Kxt, int testForceIndex, int constraintIndex)
{
	int maxConstraintIndexToExtract =  globalNumConstraintVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];

		GBody& bodyData0 = *linkPair.bodyData[0];
		GBody& bodyData1 = *linkPair.bodyData[1];

		if(bodyData0.isTestForceBeingApplied){
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase1(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 0, 1, testForceIndex, maxConstraintIndexToExtract);
			}
		} else {
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 1, 0, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase3(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			}
		}
	}
}


void LCPsolver::extractRelAccelsFromLinkPairCase1
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;

		//check Point6
		//different between linkData & link
		GLink* linkData0 = linkPair.linkData[0];
		GLink* linkData1 = linkPair.linkData[1];

		vector3 dv0(linkData0->getAcceleration(constraint.point, constraint.nodeIndex[0]));
		vector3 dv1(linkData1->getAcceleration(constraint.point, constraint.nodeIndex[1]));

		//printf("\ndv0:%lf,%lf,%lf\n",dv0.x,dv0.y,dv0.z);

		vector3 relAccel(dv1 - dv0);

		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(1), relAccel) - an0(constraintIndex);

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			const int index = constraint.globalFrictionIndex + j;

			//mod
			if(constraint.globalFrictionIndex!=numeric_limits<int>::max())
			{
				if(at0.size()>0){
					Kxt(index, testForceIndex) = dot(constraint.frictionVector(j,1), relAccel) - at0(index);
				}
			}	
			//Kxt(index, testForceIndex) = dot(constraint.frictionVector(j,1), relAccel) - at0(index);
		}
	}
}


void LCPsolver::extractRelAccelsFromLinkPairCase2
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int maxConstraintIndexToExtract)
{
	//check Point7
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;


		GLink* linkData = linkPair.linkData[iTestForce];


		vector3 dv(linkData->getAcceleration(constraint.point, constraint.nodeIndex[iTestForce]));

		vector3 relAccel(constraint.defaultAccel[iDefault] - dv);

		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(iDefault), relAccel) - an0(constraintIndex);

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			const int index = constraint.globalFrictionIndex + j;

			//mod	
			if(at0.size()>0){
				Kxt(index, testForceIndex) =
					dot(constraint.frictionVector(j,iDefault), relAccel) - at0(index);
			}
		}

	}
}


void LCPsolver::extractRelAccelsFromLinkPairCase3
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;

		Kxn(constraintIndex, testForceIndex) = 0.0;

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			Kxt(constraint.globalFrictionIndex + j, testForceIndex) = 0.0;
		}
	}
}



void LCPsolver::clearSingularPointConstraintsOfClosedLoopConnections()
{
	for(int i = globalNumContactNormalVectors; i < globalNumConstraintVectors; ++i){
		if(Mlcp(i, i) < 1.0e-4){
			for(size_t j=0; j < Mlcp.rows(); ++j){
				Mlcp(j, i) = 0.0;
			}
			Mlcp(i, i) = numeric_limits<double>::max();
		}
	}
}


void LCPsolver::setConstantVectorAndMuBlock()
{
	double timestep=world.getTimestep();
	double dtinv = 1.0 / timestep;
	const int block2 = globalNumConstraintVectors;
	const int block3 = globalNumConstraintVectors + globalNumFrictionVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){
			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int globalIndex = constraint.globalIndex;

			// set constant vector of LCP

			// constraints for normal acceleration

			if(linkPair.connection){
				// connection constraint

				const double& error = constraint.depth;
				double v;
				if(error >= 0){
					v = 0.1 * (-1.0 + exp(-error * 20.0));
				} else {
					v = 0.1 * ( 1.0 - exp( error * 20.0));
				}
					
				B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + v) * dtinv;

			} else {
				// contact constraint

				bool bUseSmoothedContact=USE_SMOOTHED_CONTACT_DYNAMICS;

				double v_star=0.0;
				if(bUseSmoothedContact)
				{
					double kp = (1+epsilon) / (kappa*kappa);
					double kd = (1+epsilon) / kappa;
					double h=timestep;
					double vn=constraint.normalProjectionOfRelVelocityOn0;
					double d=constraint.depth;

					/*
					double nc=max(4.0, double(globalNumFrictionVectors/constraint.numFrictionVectors));
					kp= kp*( 4.0/nc); // adhoc heuristic
					*/
					//cout <<"nc"<<nc<<endl;

					//v_star= vn + h*(kp * (-0 +d) - kd * vn); // original
					double vp=std::max(vn,0.0); // 빠져 나오는 방향
					double vneg=std::min(vn,0.0);
					//v_star= vn + (h+0.001)*(kp * (-0 +d*d*1+d*0.1) - kd*16.0 * vp*vp -kd*vneg);  // without _R correction
					v_star= vn + (h+0.001)*(kp * (-0 +d*d*0.1+d*0.1) - kd*16.0 * vp*vp -1*kd*vneg); 

#ifdef VERBOSE
					printf("2: %f %f %f %f\n", vn, constraint.depth, h, v_star);
#endif
				}

				if(ALLOW_SUBTLE_PENETRATION_FOR_STABILITY && constraint.depth < ALLOWED_PENETRATION_DEPTH){

					double extraNegativeVel = (ALLOWED_PENETRATION_DEPTH - constraint.depth) * NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION;
					B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + extraNegativeVel) * dtinv;
				} else {
					B(globalIndex) = an0(globalIndex) + constraint.normalProjectionOfRelVelocityOn0 * dtinv;
				}
				// modify b to a smoothed dynamics version
				B(globalIndex)+= -v_star*dtinv;

#ifdef VERBOSE
				printf("mu %f\n", constraint.mu);
#endif
				contactIndexToMu(globalIndex) = constraint.mu;

				int globalFrictionIndex = constraint.globalFrictionIndex;
				for(int k=0; k < constraint.numFrictionVectors; ++k){

					// constraints for tangent acceleration
					double tangentProjectionOfRelVelocity = dot(constraint.frictionVector(k,1), constraint.relVelocityOn0);

					B(block2 + globalFrictionIndex) = at0(globalFrictionIndex);
					if( !IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION ){
						B(block2 + globalFrictionIndex) += tangentProjectionOfRelVelocity * dtinv;
					}

					// for iterative solver
					frictionIndexToContactIndex[globalFrictionIndex] = globalIndex;

					++globalFrictionIndex;
				}
			}
		}
	}
}


void LCPsolver::addConstraintForceToLinks()
{
    int n = constrainedLinkPairs.size();
    for(int i=0; i < n; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];
		for(int j=0; j < 2; ++j){
			if(!linkPair->linkData[j]->isStatic()){
				addConstraintForceToLink(linkPair, j);
			}
		}
    }
}


void LCPsolver::addConstraintForceToLink(LinkPair* linkPair, int ipair)
{
    //vector3 f_total(0.0);
    //vector3 tau_total(0.0);

	//check Point9

    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();

    GLink* link = linkPair->linkData[ipair];
    for(int i=0; i < numConstraintPoints; ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int globalIndex = constraint.globalIndex;

		vector3 f(solution(globalIndex) * constraint.normalTowardInside(ipair));

		for(int j=0; j < constraint.numFrictionVectors; ++j){
			//mod
			if(constraint.globalFrictionIndex!=numeric_limits<int>::max()){
				f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
			}
			//f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
		}

#ifdef VERBOSE
		std::cout << "Constraint force :  " << f << std::endl;
#endif
		link->addConstraintForce(constraint.point, constraint.nodeIndex[ipair], f); 
		//f_total   += f;
		//tau_total += cross(constraint.point, f);
    }

	//link->addConstraintForce(f_total, tau_total);


#ifdef VERBOSE
		std::cout << "Constraint force to " << link->index << std::endl;
#endif
}





LCPsolver::LCPsolver(DynamicsSimulator_Trbdl_LCP& _world)
	:
	world(_world),
	bodiesData(_world.bodiesData())
{
	// recommended for low-freq control:
	//kappa=0.05; epsilon=0.1;
	// rigid:
	kappa=0.01; epsilon=0.01;
	_R=1.0e-5;

	//setGaussSeidelParameters(500, 0, 1.0e-3);
}


LCPsolver::~LCPsolver()
{
}

















void LCPsolver::setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError)
{
	LCPsolver	*impll=this;
	impll->maxNumGaussSeidelIteration = maxNumIteration;
	impll->numGaussSeidelInitialIteration = numInitialIteration;
	impll->gaussSeidelMaxRelError = maxRelError;
}
