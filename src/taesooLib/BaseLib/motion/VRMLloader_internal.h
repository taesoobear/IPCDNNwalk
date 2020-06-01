#ifndef VRMLLOADER_internal_BASELIB_H_
#define VRMLLOADER_internal_BASELIB_H_



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
