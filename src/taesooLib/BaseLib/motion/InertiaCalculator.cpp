#include "../BaseLib/baselib.h"
#include "../BaseLib/math/matrix3.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/motion/Mesh.h"
#include "../BaseLib/motion/Geometry.h"
#include "../BaseLib/motion/intersectionTest.h"
/*
#include "../MainLib/OgreFltk/Mesh.h"
#include "../MainLib/Ogre/intersectionTest.h"
*/
//#include "../MainLib/OgreFltk/objectList.h"
/*
#ifndef NO_GUI
#include "../MainLib/OgreFltk/pldprimskin.h"
#include "../MainLib/OgreFltk/framemoveobject.h"
#include "../MainLib/OgreFltk/Line3D.h"
#include "../MainLib/OgreFltk/RE.h"
#endif
*/
#include "InertiaCalculator.h"

#include "../BaseLib/motion/gjk/btVector3.h"
#include "../BaseLib/motion/gjk/btGjkEpa2.h"
#include "../BaseLib/motion/gjk/btConvexHullShape.h"

/*
#ifndef NO_GUI
#include <Ogre.h>
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);
#endif

*/

using namespace gjk;
inline gjk::btVector3 ToBullet(vector3 const& v)
{
	return btVector3(v.x, v.y, v.z);
}

inline vector3 ToBase(gjk::btVector3 const& v)
{
	return vector3(v.x(), v.y(), v.z());
}


static double collisionMargin=0.01;
struct 	InertiaCalculator_data
{
	InertiaCalculator_data(){}
	~InertiaCalculator_data()
	{
	}

	OBJloader::Mesh mesh;
	vector3 centerOfMass;
	matrix3 inertia;
	intersectionTest::AABB bound;
	m_real volume;
	
	//ObjectList mVIS;
	bitvectorn mbInside;
	int mPrecision;
	int mPrecisionSQ;
	void _setInside(int i, int j, int k, bool bValue)
	{
		mbInside.set(i*mPrecisionSQ+j*mPrecision+k, bValue);
	}

	bool _isInside(int i, int j, int k)
	{
		return mbInside(i*mPrecisionSQ+j*mPrecision+k);
	}

	void _init(int precision)
	{
		mPrecisionSQ=precision*precision;
		mPrecision=precision;
		mbInside.resize(precision*precision*precision);
	}
	void _calculate()
	{
		mbInside.clearAll();	
		bound.setNull();
		std::vector<btVector3> vertices;
		vertices.resize(mesh.numVertex());

		for(int i=0; i<mesh.numVertex(); i++)
		{
			vector3 const& v=mesh.getVertex(i);
			vertices[i]=ToBullet(v);
			bound.merge(v);
		}

		btConvexHullShape* convexShape = new btConvexHullShape(&vertices[0].getX(), vertices.size());
		convexShape->setMargin(collisionMargin);

		// inside/outside test.
		vector3 pos;
		for(int i=0; i<mPrecision; i++)
		{
			pos.x=sop::map(i, 0, mPrecision-1, bound.getMinimum().x, bound.getMaximum().x);
			for(int j=0; j<mPrecision; j++)
			{
				pos.y=sop::map(j, 0, mPrecision-1, bound.getMinimum().y, bound.getMaximum().y);
				for(int k=0; k<mPrecision; k++)
				{
					pos.z=sop::map(k, 0, mPrecision-1, bound.getMinimum().z, bound.getMaximum().z);

					btTransform	unit;
					unit.setIdentity();
					btGjkEpaSolver2::sResults	res;
					m_real dist=(btGjkEpaSolver2::SignedDistance(ToBullet(pos),0,convexShape,unit,res));

					_setInside(i,j,k, dist<0);
				}
			}
		}

		m_real count=(m_real)mbInside.count();
		// bound.volume() is actually 1-grid smaller than total sampling space.
		volume=bound.volume()*(count/(m_real)((mPrecision-1)*(mPrecision-1)*(mPrecision-1)));
		Msg::print("bounding box volume %f\n", bound.volume());
		Msg::print("volume %f\n", volume);

		m_real mass=volume;

		centerOfMass.setValue(0.0, 0.0, 0.0);

		for(int i=0; i<mPrecision; i++)
		{
			pos.x=sop::map(i, 0, mPrecision-1, bound.getMinimum().x, bound.getMaximum().x);
			for(int j=0; j<mPrecision; j++)
			{
				pos.y=sop::map(j, 0, mPrecision-1, bound.getMinimum().y, bound.getMaximum().y);
				for(int k=0; k<mPrecision; k++)
				{
					pos.z=sop::map(k, 0, mPrecision-1, bound.getMinimum().z, bound.getMaximum().z);
					if(_isInside(i,j,k))
						centerOfMass+=pos;						
				}
			}
		}
		centerOfMass/=count;


		// momentsOfInertia=

		// Ixx Ixy Ixz
		// Iyx Iyy Iyz
		// Izx izy Izz

		inertia.zero();

		m_real dx=(bound.getMaximum().x-bound.getMinimum().x)/mPrecision;
		m_real dy=(bound.getMaximum().y-bound.getMinimum().y)/mPrecision;
		m_real dz=(bound.getMaximum().z-bound.getMinimum().z)/mPrecision;
		m_real dv=dx*dy*dz;
		Msg::print("dx %f %f %f dv %f\n", dx, dy, dz, dv);

		vector3 cp;
		for(int i=0; i<mPrecision; i++)
		{
			pos.x=sop::map(i, 0, mPrecision-1, bound.getMinimum().x, bound.getMaximum().x);
			for(int j=0; j<mPrecision; j++)
			{
				pos.y=sop::map(j, 0, mPrecision-1, bound.getMinimum().y, bound.getMaximum().y);
				for(int k=0; k<mPrecision; k++)
				{
					pos.z=sop::map(k, 0, mPrecision-1, bound.getMinimum().z, bound.getMaximum().z);
					if(_isInside(i,j,k))
					{
						cp.sub(pos, centerOfMass);
						inertia._11+=(SQR(cp.y)+SQR(cp.z))*dv;
						inertia._22+=(SQR(cp.x)+SQR(cp.z))*dv;
						inertia._33+=(SQR(cp.x)+SQR(cp.y))*dv;
						inertia._12-=(cp.x*cp.y)*dv;
						inertia._13-=(cp.x*cp.z)*dv;
						inertia._23-=(cp.y*cp.z)*dv;
					}
				}
			}
		}


		inertia._21=inertia._12;
		inertia._31=inertia._13;
		inertia._32=inertia._23;

		Msg::print("COM %s\n", centerOfMass.output().c_str());
		Msg::print("inertia %f %f %f\n", inertia._11, inertia._12, inertia._13);
		Msg::print("		%f %f %f\n", inertia._21, inertia._22, inertia._23);
		Msg::print("		%f %f %f\n", inertia._31, inertia._32, inertia._33);
	}
	void drawSamplingGrid(m_real radius, vector3 const& translate)
	{
//#ifndef NO_GUI
#if 0
	  int nc=mbInside.count();
	  double scale=100;

		QuadList* box=new QuadList(vector3(0,1,0), radius);
		box->begin(nc);
		int c=0;
		vector3 pos;
		for(int i=0; i<mPrecision; i++)
		{
			pos.x=sop::map(i, 0, mPrecision-1, bound.getMinimum().x, bound.getMaximum().x);
			for(int j=0; j<mPrecision; j++)
			{
				pos.y=sop::map(j, 0, mPrecision-1, bound.getMinimum().y, bound.getMaximum().y);
				for(int k=0; k<mPrecision; k++)
				{
					pos.z=sop::map(k, 0, mPrecision-1, bound.getMinimum().z, bound.getMaximum().z);
					if(_isInside(i,j,k))
						box->quad(c++, pos*scale);
				}
			}
		}
		ASSERT(c==nc);
		box->end();
		_setMaterial(box,"blueCircle");

		mVIS.clear();
		RE::moveEntity(mVIS.registerObject(RE::generateUniqueName(), box), translate);

		mVIS.registerEntity(RE::generateUniqueName(), createMeshEntity(mesh, RE::generateUniqueName()));

		static OBJloader::MeshToEntity* meshToEntity=NULL;
		if (!meshToEntity){
			OBJloader::MeshToEntity::Option o;
			o.useTexCoord=false;
			o.buildEdgeList=true;
			meshToEntity=new OBJloader::MeshToEntity(mesh, RE::generateUniqueName(), o);
			mVIS.registerEntity(RE::generateUniqueName(), meshToEntity->createEntity(RE::generateUniqueName(), "white"))->scale(scale,scale,scale);
		}
#endif		
		
	}

};

#define set_d()	InertiaCalculator_data& d=*((InertiaCalculator_data*)_data)

InertiaCalculator::~InertiaCalculator()
{
	set_d();
	delete &d;
}

InertiaCalculator::InertiaCalculator(int precision)
{
	_data=(void*)new InertiaCalculator_data();
	set_d()	;

	d._init(precision);
}

// assumes unit mass, uniform density.
void InertiaCalculator::calculateFromFile(const char* filename)
{
	set_d()	;
	d.mesh.loadObj(filename);
	d._calculate();
}

static vector3 boxInertia(double mass, vector3 const& size)
{
	double 	Ix=	mass*1.0/12.0*(size.y*size.y+size.z*size.z);
	double 	Iy=	mass*1.0/12.0*(size.x*size.x+size.z*size.z);
	double 	Iz=	mass*1.0/12.0*(size.y*size.y+size.x*size.x);
	return vector3(Ix, Iy, Iz);
}
// for debugging purposes. (We know the right answer);
void InertiaCalculator::calculateFromBox(m_real sizeX, m_real sizeY, m_real sizeZ)
{
	set_d()	;
	OBJloader::createBox(d.mesh, sizeX, sizeY, sizeZ);
	d._calculate();
}

void InertiaCalculator::calculateFromMesh(OBJloader::Mesh const &mesh)
{
	set_d();
	d.mesh.copyFrom(mesh);
	d._calculate();
}

void InertiaCalculator::calculateFromCylinder(m_real radius, m_real height)
{
	set_d()	;
	OBJloader::createCylinder(d.mesh, radius, height, 16);

	m_real M=radius*radius*M_PI*height;
	Msg::print("\ncylinder: exact volume=%f\n", M);
	d._calculate();

	m_real ix=M*height*height/12.0+M*radius*radius/4.0;
	m_real iy=0.5*M*radius*radius;
	Msg::print("exact tensor=%f %f %f\n", ix, iy, ix);
	
}

// com will be at the origin when translate==0
void InertiaCalculator::drawSamplingGrid(m_real radius, vector3 const& translate)
{
	set_d()	;
	d.drawSamplingGrid(radius, translate);
}

const vector3& InertiaCalculator::centerOfMass() const
{
	set_d()	;
	return d.centerOfMass;
}

m_real InertiaCalculator::volume() const
{
	set_d()	;
	return d.volume;
}
const matrix3& InertiaCalculator::inertia() const
{
	set_d()	;
	return d.inertia;
}

///////////////
struct 	InertiaCalculator_dataA
{
	InertiaCalculator_dataA(){}
	~InertiaCalculator_dataA()
	{
	}

	OBJloader::Geometry mesh;
	vector3 centerOfMass;
	matrix3 inertia;
	intersectionTest::AABB bound;
	m_real volume;
	
	void _calculate()
	{
		bound.setNull();
		for(int i=0; i<mesh.numVertex(); i++)
		{
			vector3 const& v=mesh.getVertex(i);
			bound.merge(v);
		}

		volume=bound.volume();
		m_real mass=volume;

		centerOfMass=bound.getCenter();

		// momentsOfInertia=

		// Ixx Ixy Ixz
		// Iyx Iyy Iyz
		// Izx izy Izz

		inertia.zero();

		vector3 ii=boxInertia(mass, bound.getSize());
		inertia._11=ii.x;
		inertia._22=ii.y;
		inertia._33=ii.z;

		Msg::print("COM %s\n", centerOfMass.output().c_str());
		Msg::print("inertia %f %f %f\n", inertia._11, inertia._12, inertia._13);
		Msg::print("		%f %f %f\n", inertia._21, inertia._22, inertia._23);
		Msg::print("		%f %f %f\n", inertia._31, inertia._32, inertia._33);
	}
};

#undef set_d
#define set_d()	InertiaCalculator_dataA& d=*((InertiaCalculator_dataA*)_data)

InertiaCalculatorAnalytic::~InertiaCalculatorAnalytic()
{
	set_d();
	delete &d;
}

InertiaCalculatorAnalytic::InertiaCalculatorAnalytic()
{
	_data=(void*)new InertiaCalculator_dataA();
	set_d()	;
}

// for debugging purposes. (We know the right answer);
void InertiaCalculatorAnalytic::calculateFromBox(m_real sizeX, m_real sizeY, m_real sizeZ)
{
	set_d()	;
	OBJloader::createBox(d.mesh, sizeX, sizeY, sizeZ);
	d._calculate();
}
using namespace OBJloader;
using namespace std;
#include <iostream>

void InertiaCalculatorAnalytic::calculateFromMesh(OBJloader::Geometry const &mesh)
{
	set_d();
	d.mesh.copyFrom(mesh);
	d._calculate();

	matrix3 inertia;
	inertia.zero();
	vector3 localCOM(0,0,0);
	double totalVolume=0.0;
	bool bOkay=true;
	for(int i=0; i<mesh.numElements(); i++)
	{
		double volume=0.0;
		auto& elt=mesh.element(i);
		matrix3 eltInertia;
		eltInertia.zero();
		if(mesh.element(i).elementType==Element::BOX)
		{
			volume=elt.elementSize.x*elt.elementSize.y*elt.elementSize.z;
			vector3 ii=boxInertia(volume, elt.elementSize);
			eltInertia._11=ii.x;
			eltInertia._22=ii.y;
			eltInertia._33=ii.z;
		}
		else if(mesh.element(i).elementType==Element::CAPSULE || Element::CYLINDER)
		{

			double radius=elt.elementSize.x*0.5;
			double height=elt.elementSize.y;
			volume=3.141592*radius*radius*height;
			double M=volume; // will later be scaled.

			double ix=M*height*height/12.0+M*radius*radius/4.0;
			double iy=0.5*M*radius*radius;
			eltInertia._11=ix;
			eltInertia._22=iy;
			eltInertia._33=ix;
		}
		else if(mesh.element(i).elementType==Element::SPHERE || mesh.element(i).elementType==Element::ELLIPSOID)
		{
			vector3 s=elt.elementSize;
			vector3 ii;
			double a,b;
			// actually volume
			volume=4.0*3.141592/3.0/8.0*elt.elementSize.x*elt.elementSize.y*elt.elementSize.z;
			double mass=volume;
			b=s.y; a=s.z;
			ii.x=mass*b*b/5.0*(1.0+a*a/(b*b));
			b=s.z; a=s.x;
			ii.y=mass*b*b/5.0*(1.0+a*a/(b*b));
			b=s.x; a=s.y;
			ii.z=mass*b*b/5.0*(1.0+a*a/(b*b));
			eltInertia._11=ii.x;
			eltInertia._22=ii.y;
			eltInertia._33=ii.z;
		}
		else
			bOkay=false;

		localCOM+=volume*elt.tf.translation;

		auto const&R=elt.tf.rotation;
		eltInertia=matrix3(R)*eltInertia*matrix3(R.inverse());
		inertia+=eltInertia;
		totalVolume+=volume;
	}

	localCOM*=(1.0/totalVolume);


	if(bOkay && localCOM.x==localCOM.x)
	{
		cout << "mass"<<d.volume << " " << totalVolume<< " "<<bOkay<<endl;
		cout<< "COM"<<d.centerOfMass <<" "<<localCOM<<endl;
		cout<< "i"<<d.inertia <<" "<<inertia<<endl;
		d.centerOfMass=localCOM;
		d.inertia=inertia;
		d.volume=totalVolume;
	}
}

void InertiaCalculatorAnalytic::calculateFromCylinder(m_real radius, m_real height)
{
	set_d()	;
	OBJloader::createCylinder(d.mesh, radius, height, 16);

	m_real M=radius*radius*M_PI*height;
	Msg::print("\ncylinder: exact volume=%f\n", M);
	d._calculate();

	m_real ix=M*height*height/12.0+M*radius*radius/4.0;
	m_real iy=0.5*M*radius*radius;
	Msg::print("exact tensor=%f %f %f\n", ix, iy, ix);
	
}

const vector3& InertiaCalculatorAnalytic::centerOfMass() const
{
	set_d()	;
	return d.centerOfMass;
}

m_real InertiaCalculatorAnalytic::volume() const
{
	set_d()	;
	return d.volume;
}
const matrix3& InertiaCalculatorAnalytic::inertia() const
{
	set_d()	;
	return d.inertia;
}

