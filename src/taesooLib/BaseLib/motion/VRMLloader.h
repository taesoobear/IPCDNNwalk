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

	// shortcuts:
	bool hasShape() const;
	OBJloader::Geometry& getMesh() const;
	int numHRPjoints() const;
	int HRPjointIndex(int i) const;
	int DOFindex(int i) const;
	int DQindex(int i) const;
	TString HRPjointName(int i) const;
	HRP_JOINT::jointType_T HRPjointType(int i) const;
	TString HRPjointAxis(int i) const;

	inline int lastHRPjointIndex() const	{return HRPjointIndex(numHRPjoints()-1);}
	
	vector3 localCOM() const;
	void setLocalCOM(vector3 const& com);
	double mass();
	vector3 inertia() const; // only diagonal.
	matrix3 const& momentsOfInertia() const; // full.
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
	friend class VRMLloader_subtree;
	
};

class VRMLloader_subtree;
// Skeleton 정보를 읽고 tree hierarchy를 만든다. 
// 또한 skinning에 필요한 모든 정보를 읽어온다.
class VRMLloader: public MotionLoader
{
	double _frameRate;
public:
	TString url;
	TString name;
	TString version;
	TStrings info;
	
	virtual TString getName() { return name;}
	VRMLloader(OBJloader::Geometry const& mesh, bool useFixedJoint=false);
	VRMLloader(const char* vrmlFile);
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
	
	virtual void scale(float fScale, Motion& mot);

	vector3 calcCOM() const;
	void calcZMP(const MotionDOF& motion, matrixn & aZMP, double kernelSize);
	// do not use for the root bone.
	void setChannels(Bone& bone, const char* translation_axis, const char* rotation_axis);
	void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren);
	void insertChildJoint(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren, vector3 const& offset);
	//void setTotalMass( m_real totalMass);
	void printDebugInfo();
	void changeTotalMass( m_real newtotalMass);
	static void projectAngles(vectorn & temp1);

	VRMLloader_subtree* makesubtree(int treeIndex) const;

	virtual void loadAnimation(Motion& mot, const char* filename) const;
	virtual void _initDOFinfo();
	void _exportBinary(BinaryFile& bf) const;
	void _importBinary(BinaryFile& bf);
	void _importVRML(CTextFile& tf);

	virtual void setCurPoseAsInitialPose();
protected:
	void _clear();
	void _checkMass();
	friend class VRMLTransform;
};


class VRMLloader_subtree : public VRMLloader
{
protected:
	vector3 origindofpos;
	quater origindofori;
	friend class VRMLloader;
	int mdof;    
	int mtreeindex;
	VRMLloader_subtree();
public:
	const VRMLloader* fullbody;
	virtual ~VRMLloader_subtree();
	const VRMLloader* getFullbody()  {return fullbody;}
	vector3 originpos() const {return origindofpos;}
	quater originori() const {return origindofori;} 

	int fullDOFindex(int subDOFindex) const { return subDOFindex+mdof+1;}
	int subDOFindex(int fullDOFindex) const { return fullDOFindex-mdof-1;}
	int fullTreeindex(int subDOFindex) const { return subDOFindex+mtreeindex-1;}
	// returns positive integers when found.
	int subTreeindex(int fulltreeindex) const ;

	void subPoseToFullpose(); // modifies fullbody
	void subPoseToFullpose(vectorn const & subpose, vectorn &fullpose) const;
	void FullbodyPoseToSubpose(); // modifies this
	void fullbodyPoseToSubpose(vectorn const & fullpose, vectorn &subpose) const;

};
#endif
