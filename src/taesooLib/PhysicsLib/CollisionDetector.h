/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
#ifndef OPENHRP_COLLISION_DETECTOR_IDL_INCLUDED
#define OPENHRP_COLLISION_DETECTOR_IDL_INCLUDED

#include "OpenHRPcommon.h"
#include "DynamicsSimulator.h"
class VRMLloader;
namespace OpenHRP {

  class CollisionDetector
  {
  protected:
	  std::vector<VRMLloader*> mTrees;
	  std::vector<std::string> mTreeNames;
	  std::vector<LinkPair> mPairs;
  public:
	  CollisionDetector(){}
	  virtual ~CollisionDetector(){}

    virtual int addModel(VRMLloader* loader); // returns characterIndex
	void changeLastModelName(const char* charName) { mTreeNames.back()=charName;} // overwrite name.
	int addObstacle(OBJloader::Geometry const& mesh); // returns characterIndex
	VRMLloader* getModel(int ichar) { return mTrees[ichar];}
	int numModels() const { return (int)mTrees.size();}

	void setMargin(int ilink, double margin);
	void setMarginAll(vectorn const & margin);
	void getMarginAll(vectorn & margin);

  
	std::vector<LinkPair> const& getCollisionPairs() const	{ return mPairs;}

	virtual void _addCollisionPair(int ichar1, int ibone1, int ichar2, int ibone2);
	// used only in CollisionChecker (deprecated)
	virtual void addCollisionPair(VRMLloader* skel1, int ibone1, VRMLloader* skel2, int ibone2);
    virtual void removeCollisionPair(LinkPair   const& colPair)=0;

	// setWorldTransformations of collision shapes
	virtual void setWorldTransformations(int charIndex, BoneForwardKinematics const& fk)=0; 
    virtual bool testIntersectionsForDefinedPairs(CollisionSequence & collisions)=0;
	// only libccd supports this.
	virtual bool getLocalBoundingBoxSize(int charIndex, int ibone, vector3& localSize) { Msg::error("not imple");return false;}


	// only bullet support this.
	virtual bool isSignedDistanceSupported() { return false;}
	virtual double calculateSignedDistance(int iloader, int ibody, vector3 const& position, vector3& normal) { return 0.0;}

	// only libccd_merged support this for now.
	virtual bool isSphereTestSupported() { return false;}
	// returns penetration depth
	virtual double testSphereIntersection(int iloader, int ibody, vector3 const& position, double radius, vector3& contactpos, vector3& normal) { return 0.0;}
	struct RayTestResult
	{
		m_real m_closestHitFraction;
		vector3 m_hitNormalWorld;
		RayTestResult():m_closestHitFraction(1.0){}
		bool hasHit() const { return m_closestHitFraction!=1.0; }
	};

	virtual void rayTest(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result);
	virtual void rayTestBackside(int ichar, int ilink, vector3 const& from, vector3 const& to, RayTestResult& result){Msg::error("not implemented yet");}
	

	////////////////////////////////////////////////////////
	// Functions below are for DynamicsSimulator class.
    /**
     * Clear the Cache
     * @param   url         Data Identifier
     */
	virtual void clearCache(const char* url){}
	
    /**
     * Adding Objects
     * @param   charName    Character Name
     * @param   model	    model information
    */
    virtual void addModel(const char* charName, CharacterInfo const& model);
    /**
     * Adding Polygon Set Pairs
     * @param   colPair       Collision Pair
     * @param   convexsize1   unused
     * @param   convexsize2   unused
    */
    virtual void addCollisionPair(LinkPair const& colPair, bool convexsize1, bool convexsize2);
    
	// update positions of the collisionShapes, and check collisions
	virtual bool queryContactDeterminationForDefinedPairs(
			std::vector<DynamicsSimulator::Character*> const& positions, CollisionSequence & collisions);
  };

#ifdef INCLUDE_OPCODE_DETECTOR
	class CollisionDetectorFactory
	{
	public:
		void * data;
		CollisionDetectorFactory();
		virtual ~CollisionDetectorFactory();
		virtual CollisionDetector* create();
	};
#endif

}

#endif
