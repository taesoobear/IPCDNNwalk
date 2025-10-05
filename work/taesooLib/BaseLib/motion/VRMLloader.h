#ifndef VRMLLOADER_BASELIB_H_
#define VRMLLOADER_BASELIB_H_

//#include "nodestack.h"
#include "MotionLoader.h"
#include "MotionDOF.h"
//#include "Mesh.h"
#include "Geometry.h"
#include "../math/tvector.h"
#include "../math/matrix3.h"
class CTextFile;
namespace OBJloader
{
	class Terrain;
}

namespace TRL
{
	class CollisionDetector_fcl ;
}

namespace HRP_JOINT
{
	enum jointType_T {  // trans, rot channel string:
		FREE,			// "XYZ", "ZXY" (jointAxis will be ignored, ball joint will be used for the rotational channels)
		BALL,			// quaternion. (jointAxis will be ignored)
		ROTATE,			// ""	, jointAxis
		FIXED,			// ""	, ""	(jointAxis will be ignored)
		GENERAL,		// "Z", "XYZ" when jointAxis== "Z_XYZ"
		SLIDE			// jointAxis, ""
	};
}

struct VRML_TRANSFORM;
struct _HRP_JOINT;
struct HRP_SHAPE;
struct HRP_SEGMENT;
class VRMLloader;
/// OgreMesh파일의 skeleton구조의 한 본에 해당한다. 
class VRMLTransform: public Bone
{
public:
	_HRP_JOINT* mJoint;
	HRP_SEGMENT* mSegment;
	HRP_SHAPE* mShape;
	VRML_TRANSFORM* mTransform;

	VRMLTransform();
	virtual ~VRMLTransform();
	TString mVRMLtype;
	void Unpack(VRMLloader& l, CTextFile &file);
	void UnpackChildren(VRMLloader& l, CTextFile& file);
	void pack(FILE* file, int level=0);
	void pack(BinaryFile& bf);
	void unpack(VRMLloader& l, BinaryFile& bf);
	void copyFrom(VRMLTransform const& bone);	
	void setJointAxes(const char* axes);
	virtual void printHierarchy(int depth);

	// shortcuts:
	bool hasShape() const;
	OBJloader::Geometry& getMesh() const;
	void createNewShape() ; // replacing the existing one if any.
	void removeShape() ; 
	int numHRPjoints() const;
	int HRPjointIndex(int i) const;
	int DOFindex(int i) const;
	int DQindex(int i) const;
	TString HRPjointName(int i) const;
	HRP_JOINT::jointType_T HRPjointType(int i) const;
	TString HRPjointAxis(int i) const;
	void setJointRange(int i, double min_deg, double max_deg);

	inline int lastHRPjointIndex() const	{return HRPjointIndex(numHRPjoints()-1);}
	
	vector3 localCOM() const;
	void setLocalCOM(vector3 const& com);
	double mass();
	vector3 inertia() const; // only diagonal.
	matrix3 const& momentsOfInertia() const; // full.
	void setMass(double m);
	void setInertia(double ix, double iy, double iz);
	void scaleMass( m_real scalef);
	void translateMesh( vector3 const& trans);
	// set joint position relative to its parent joint.
	void setJointPosition(vector3 const& trans);
	void translateBone(vector3 const& trans);
	void transformMesh(matrix4 const& m);
	void transformMeshLocal(matrix4 const& m);
	void scaleMesh( vector3 const& scale);

	

	// convert a position in joint local frame to body local frame.
	void jointToBody(vector3& lposInOut) const;
	// convert a position in body local frame to joint local frame.
	void bodyToJoint(vector3& lposInOut) const;

private:
	void initBones();
	friend class VRMLloader;
	
};

// Skeleton 정보를 읽고 tree hierarchy를 만든다. 
// 또한 skinning에 필요한 모든 정보를 읽어온다.
class VRMLloader: public MotionLoader
{
	double _frameRate;

	OBJloader::Terrain* _terrain;// reference
	friend class TRL::CollisionDetector_fcl ;
public:
	class MaterialCreator
	{
		public:
		MaterialCreator(){}
		virtual void createMaterial(const char* id, const vector3 & diffuse, const vector3& specular, const vector3&  emissive, double shininess){}
	};
	static void registerMaterialCreator(MaterialCreator* singletonMatCtr);
	TString url;
	TString name;
	TString assetFolder;
	TString version;
	TStrings info;

	TString getURL() const {return url;}
	void setURL(const char* u) { url=u;}
	void setPosition(const vector3 & pos); // adjust the fixed root joint 
	virtual TString getName() { return name;}

	VRMLloader(VRMLloader const& other, int newRootIndex, bool bFreeRootJoint);
	VRMLloader(OBJloader::Geometry const& mesh, bool useFixedJoint=false);
	VRMLloader(OBJloader::Terrain *terrain);
	VRMLloader(const char* vrmlFile);
	VRMLloader(const std::string & vrmlFile);
	VRMLloader(CTextFile& vrmlFile);
	VRMLloader(VRMLloader const& other); // copy constructor
	VRMLloader(MotionLoader const& skel, double cylinder_radius);
	VRMLloader(); // a sphere-shaped free body.
	virtual ~VRMLloader();
	void operator=(VRMLloader const& other);
	
	virtual void removeBone(Bone& target);
	virtual void removeAllRedundantBones();
	void exportVRML(const char* filename);
	void exportBinary(const char* filename);
	VRMLTransform& VRMLbone(int treeIndex) const;
	int numHRPjoints();
	void changeAll3DOFjointsToSpherical();
	void changeAllMultiDOFjointsToSpherical(); 
	void changeAllJointsToSpherical(); // except the root.
	
	virtual void scale(float fScale, Motion& mot);

	vector3 calcCOM() const;
	void calcZMP(const MotionDOF& motion, matrixn & aZMP, double kernelSize);
	// do not use for the root bone.
	void setChannels(Bone& bone, const char* translation_axis, const char* rotation_axis);
	void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren);
	void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren, vector3 const& offset);
	void setTotalMass( m_real totalMass);
	void printDebugInfo();
	void changeTotalMass( m_real newtotalMass);
	static void projectAngles(vectorn & temp1);


	virtual void loadAnimation(Motion& mot, const char* filename) const;
	virtual void _initDOFinfo();
	void _exportBinary(BinaryFile& bf) const;
	void _importBinary(BinaryFile& bf);
	void _importVRML(CTextFile& tf);

	virtual void setCurPoseAsInitialPose();


	// DynamicsSimulator_TRL_LCP acknowledges this constraint. 
	void addRelativeConstraint(int ibone1, vector3 const& lpos1, int ibone2, vector3 const& lpos2);
	struct Constraint
	{
		int ibone1, ibone2;
		vector3 localpos1, localpos2;
	};
	std::vector<Constraint> constraints;
public:
	void _getAllRelativeConstraints(intvectorn& ibone, vector3N& localpos) const;

protected:
	void _clear();
	void _checkMass();
	friend class VRMLTransform;
};



#endif
