#ifndef VRMLLOADER_internal_BASELIB_H_
#define VRMLLOADER_internal_BASELIB_H_

struct VRML_TRANSFORM
{
	// used only for parsing, and set as identity matrix after initialization.
	VRML_TRANSFORM(); // initialized as identity

	// all these are unused after parsing
	vector3 scale;  
	vector4 rotation;// (axis,angle)
	vector3 translation;
};

struct _HRP_JOINT
{

	_HRP_JOINT();
	HRP_JOINT::jointType_T jointType;
	int jointStartId;
	int jointEndId;
	
private:
	friend class VRMLloader;
	friend class VRMLTransform;
	friend struct VRMLTransformView;
	TString jointAxis;	// use Bone::getRotationalChannels() and getTranslationalChannels()
	vector3* jointAxis2;//두개 중 하나를 사용 jointAxis jointAxis2 
	m_real* jointAxis2Angle;
	int AxisNum;
};


struct HRP_SEGMENT
{
	vector3 centerOfMass;
	m_real mass;
	matrix3 momentsOfInertia;
	// taesoo
	TString material;	
	HRP_SEGMENT();

};

class HRP_CUSTOM_SHAPE
{
	public:
	HRP_CUSTOM_SHAPE(){}
	virtual ~HRP_CUSTOM_SHAPE(){}
};

struct HRP_SHAPE
{
	HRP_SHAPE()		{ customShape=NULL;}
	~HRP_SHAPE()	{ delete  customShape;}
	TString name;
	OBJloader::Geometry mesh;// in parent local coordinates (joint local)
	HRP_CUSTOM_SHAPE* customShape;

	void operator=(HRP_SHAPE& other){ name=other.name; mesh=other.mesh; customShape=NULL;}
};
#endif
