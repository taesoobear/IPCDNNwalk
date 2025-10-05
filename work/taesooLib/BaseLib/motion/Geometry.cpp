#include "stdafx.h"
#include "Geometry.h"
#include "Terrain.h"
#include "../math/Operator.h"
#include <iostream>
#include <fstream>
#ifdef _MSC_VER
#include <direct.h>
#else
#include <sys/stat.h> 
#include <sys/types.h>
#endif
using namespace OBJloader;

static void initFaceGroups(Geometry& mesh)
{
	mesh.faceGroups.resize(1);
	mesh.faceGroups.start(0)=0;
	mesh.faceGroups.end(0)=mesh.numFace();
	mesh.elements.resize(1);
	mesh.elements[0].elementType=Element::OBJ;
	mesh.elements[0].tf.identity();
}

void Geometry::mergeAllElements()
{
	initFaceGroups(*this);
}

void Geometry::copyFrom(Geometry const& otherMesh)
{
	//printf("copyfrom called\n");
	if (otherMesh.elements.size()!=0)
	{
		Mesh::copyFrom(otherMesh);
		elements=otherMesh.elements;
	}
	else
	{
		// invalid geometry. use only the mesh part (excluding elements).
		assignMesh(otherMesh);
	}
}
void Geometry::assignMesh(OBJloader::Mesh const& otherMesh)
{
	Mesh::copyFrom(otherMesh);
	int nE=faceGroups.size();
	elements.resize(nE);
	for(int i=0; i<nE; i++)
	{
		elements[i].elementType=Element::OBJ;
		elements[i].tf.identity();
	}

}
void Geometry::assignTerrain(OBJloader::Terrain const& otherMesh, vector3 const& center)
{
	Mesh::copyFrom(otherMesh);

	matrix4 temp;
	temp.setTranslation(center, false);
	transform(temp);

	initFaceGroups(*this);
	elements[0].elementType=Element::TRI;
}

bool Geometry::saveObj(const char* filename, bool vn, bool vt)
{
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(filename);
	if ( !fout.is_open() ) return false;

	fout << "# GEOMETRY FILE v1.0"<<std::endl;
	fout << "# "<< elements.size() <<std::endl;

	Msg::verify(elements.size()==faceGroups.size(), "Invalid geometry");

	for (int i=0; i<elements.size(); i++)
	{
		fout << "# "<< elements[i].elementType << " " << elements[i].elementSize << " " << elements[i].tf.translation <<" " <<elements[i].tf.rotation  ;
		fout << "# "<< faceGroups.start(i) << " " << faceGroups.end(i) 	<< std::endl;
	}

	_saveObj(fout, vn, vt);
	fout.close();
	return true;
}
bool Geometry::loadObj(const char* filename)
{
	CTextFile file;
	if(!file.OpenReadFile(filename))
	{
		printf("Error! Faild to open %s\n", filename);
		return false;
	}
	_loadObj(file);
	// todo: loading faceGroups and elements in comments
	initFaceGroups(*this);
	return true;
}

// always make file format backward compatible.
void Geometry::pack(BinaryFile& bf) const
{
	int version=2; // version 1: geometry. 
	bf.packInt(version);
	bf.packInt(elements.size());
	for (int i=0; i<elements.size(); i++)
	{
		bf.packInt(elements[i].elementType);
		bf.pack( elements[i].elementSize );
		bf.pack( elements[i].tf.rotation);
		bf.pack( elements[i].tf.translation);
		bf.pack( elements[i].material);
		bf.packInt( faceGroups.start(i));
		bf.packInt( faceGroups.end(i));
	}
	Mesh::pack(bf);
}

void Geometry::unpack(BinaryFile& bf)
{
	int version=bf.unpackInt();
	if (version>=1){
		int eltSize= bf.unpackInt();
		elements.resize(eltSize);
		intIntervals temp_faceGroups;		// becomes valid after classifyTriangles method is called.
		temp_faceGroups.resize(eltSize);
		for (int i=0; i<elements.size(); i++)
		{
			elements[i].elementType=bf.unpackInt();
			bf.unpack( elements[i].elementSize );
			bf.unpack( elements[i].tf.rotation);
			bf.unpack( elements[i].tf.translation);
			if (version==2)
				bf.unpack( elements[i].material);
			temp_faceGroups.start(i)=bf.unpackInt( );
			temp_faceGroups.end(i)=bf.unpackInt( );
		}
		Mesh::unpack(bf);
		faceGroups=temp_faceGroups;
	}
	else
	{
		Mesh::_unpackMeshVersion(version, bf);
	}
}
void Geometry::initBox(const vector3& size)
{
	OBJloader::createBox(*this, size.x, size.y, size.z);
	elements.resize(1);
	elements[0].elementType=Element::BOX;
	elements[0].elementSize=size;
	elements[0].tf.identity();
}
void Geometry::initCylinder(double radius, double height, int numDivision)
{
	OBJloader::createCylinder(*this, radius, height, numDivision);
	elements.resize(1);
	elements[0].elementType=Element::CYLINDER;
	elements[0].elementSize.x=2*radius;
	elements[0].elementSize.y=height;
	elements[0].elementSize.z=2*radius;
	elements[0].tf.identity();
}

#include "half_sphere_111.obj.h"
void Geometry::initEllipsoid(const vector3& size)
{
	Mesh halfSphere;
	//halfSphere.loadObj("../Resource/mesh/half_sphere_111.obj");
	load_half_sphere_111(halfSphere); // radius==1 == half diameter
	matrix4 m;
	m.setScaling(size.x, size.y, size.z);
	halfSphere.transform(m);
	Mesh halfSphere2=halfSphere;
	m.setRotation(vector3(1,0,0), TO_RADIAN(180));
	halfSphere2.transform(m);
	Mesh::merge(halfSphere, halfSphere2);
	//Mesh::mergeDuplicateVertices(false, 0.001);
	if(1)
	{
		// 6, 7, 14, 15, 12, 13, 10, 11,  8, 9,
		// 33,32,35, 34, 37, 36, 39, 38, 41,40,
		//
		// specific to half_sphere_111.obj 
		std::vector<std::pair<int,int> > mergeVertices;
		mergeVertices.push_back(std::pair<int,int>(6,33));
		mergeVertices.push_back(std::pair<int,int>(7,32));
		mergeVertices.push_back(std::pair<int,int>(8,41));
		mergeVertices.push_back(std::pair<int,int>(9,40));
		mergeVertices.push_back(std::pair<int,int>(10,39));
		mergeVertices.push_back(std::pair<int,int>(11,38));
		mergeVertices.push_back(std::pair<int,int>(12,37));
		mergeVertices.push_back(std::pair<int,int>(13,36));
		mergeVertices.push_back(std::pair<int,int>(14,35));
		mergeVertices.push_back(std::pair<int,int>(15,34));
		Mesh::mergeVertices(mergeVertices,false);
	}
	elements.resize(1);
	elements[0].elementType=Element::ELLIPSOID;
	elements[0].elementSize=size;
	elements[0].tf.identity();
	faceGroups.resize(1);
	faceGroups.start(0)=0;
	faceGroups.end(0)=numFace();
}
static void createPipe(Mesh& mesh, m_real radius, m_real height, int ndiv)
{

	vector3 front(0,0,1);

	mesh.resize(ndiv*2, ndiv,0,0,ndiv*2);

	int nFV=ndiv*2;

	for(int i=0; i<ndiv; i++)
	{
		// assuming ndiv==0
		//         v(ndiv*2)
		//   v0   /_\   v2      F0
		//       |\  |
		//       | \ |         F1\F2
		//   v1  |___|  v3
		//        \ /           F3
		//         v(ndiv*2+1)

		vector3 nf;
		quater q;
		q.setRotation(vector3(0,1,0), sop::map(i, 0, ndiv, 0, M_PI*2.0));
		nf.rotate(q, front);

		mesh.getVertex(i*2)=nf*radius+vector3(0,height/2.0,0);
		mesh.getVertex(i*2+1)=nf*radius+vector3(0,height/-2.0,0);;

		mesh.getNormal(i)=nf;

		int v0=i*2;
		int v1=i*2+1;
		int v2=((i+1)*2)%nFV;
		int v3=((i+1)*2+1)%nFV;

		// set vertex index
		//mesh.getFace(i*4).setIndex(nFV, v0, v2);
		mesh.getFace(i*2+0).setIndex(v0, v1, v3); 
		mesh.getFace(i*2+1).setIndex(v0, v3, v2);
		//mesh.getFace(i*4+3).setIndex(v1, nFV+1, v3);

		int n2=(i+1)%ndiv;

		// set normal index
		//mesh.getFace(i*4).setIndex(ndiv, ndiv, ndiv, OBJloader::Buffer::NORMAL);
		mesh.getFace(i*2+0).setIndex(i, i, n2, OBJloader::Buffer::NORMAL);
		mesh.getFace(i*2+1).setIndex(i, n2, n2, OBJloader::Buffer::NORMAL);
		//mesh.getFace(i*4+3).setIndex(ndiv+1, ndiv+1, ndiv+1, OBJloader::Buffer::NORMAL);
	}

}
void Geometry::initCapsule(double radius, double height)
{
	Mesh halfSphere;
	//halfSphere.loadObj("../Resource/mesh/half_sphere_111.obj");
	load_half_sphere_111(halfSphere);
	matrix4 m;
	m.setScaling(radius, radius, radius);
	m.setTranslation(vector3(0,height/2.0,0));
	halfSphere.transform(m);
	Mesh halfSphere2=halfSphere;
	m.setRotation(vector3(1,0,0), TO_RADIAN(180));
	halfSphere2.transform(m);
	Mesh cylinder;
	createPipe(cylinder, radius, height, 10); 
	halfSphere.merge(halfSphere, halfSphere2);
	Mesh::merge(halfSphere, cylinder);
	if(1)
	{

		int upperHalfSphereBoundary[]= {6, 7, 14, 15, 12, 13, 10, 11,  8, 9};
		int upperCylinderBoundary[]=   {56,58,60, 62, 64, 66, 68, 70, 52,54};
		int lowerCylinderBoundary[]=   {57,59,61, 63, 65, 67, 69, 71, 53,55};
		int lowerHalfSphereBoundary[]= {33,32,35, 34, 37, 36, 39, 38, 41,40};
		std::vector<std::pair<int,int> > mergeVertices;
		for(int i=0; i<10; i++)
		{
			mergeVertices.push_back(std::pair<int,int>(upperHalfSphereBoundary[i], upperCylinderBoundary[i]));
			mergeVertices.push_back(std::pair<int,int>(lowerHalfSphereBoundary[i], lowerCylinderBoundary[i]));
		}
		Mesh::mergeVertices(mergeVertices,false);
	}

	elements.resize(1);
	elements[0].elementType=Element::CAPSULE;
	elements[0].elementSize.x=2*radius;
	elements[0].elementSize.y=height;
	elements[0].elementSize.z=2*radius;
	elements[0].tf.identity();
	faceGroups.resize(1);
	faceGroups.start(0)=0;
	faceGroups.end(0)=numFace();
}
void Geometry::classifyTriangles()
{
	Msg::verify(elements.size()==0 || (elements.size()==1 && elements[0].elementType==Element::OBJ), "Geometry::classifyTriangles");
	OBJloader::classifyTriangles(*this);
	elements.resize(faceGroups.size());
	for (int i=0; i< elements.size(); i++)
	{
		elements[i].elementType=Element::OBJ;
		elements[i].elementSize=vector3(0,0,0);
		elements[0].tf.identity();
	}
}


void Geometry::operator=(Geometry const& otherMesh) { copyFrom(otherMesh); }
void Geometry::operator=(Mesh const& otherMesh) { assignMesh(otherMesh);}
void Geometry::merge(Geometry const& a, Geometry const& b)
{
	// avoid aliasing
	if(&a==this)
	{
		if (a.numVertex()==0)
		{
			copyFrom(b);
			return;
		}
		Geometry aa(a);
		merge(aa,b);
		return;
	}
	if(&b==this)
	{
		if (b.numVertex()==0)
		{
			copyFrom(a);
			return;
		}
		Geometry bb(b);
		merge(a,bb);
		return;
	}
	Mesh::merge(a,b);
	
	//printf("merging %d %d\n", a.elements.size(), b.elements.size());

	elements.resize(a.elements.size()+b.elements.size());
	for(int i=0; i<a.elements.size(); i++)
		elements[i]=a.elements[i];
	for(int i=0; i<b.elements.size(); i++)
		elements[i+a.elements.size()]=b.elements[i];
}
void Geometry::rigidTransform(transf const& b)
{
	matrix4 m(b);
	Mesh::transform(m);
	for (int i=0; i<elements.size(); i++)
		elements[i].tf.leftMult(b);
}
static void geometry_markFaceGroup(Geometry const& g, int iFaceGroup, boolN& cvertices, boolN& cnormals)
{
	cvertices.resize(g.numVertex());
	cvertices.clearAll();
	cnormals.resize(g.numVertex());
	cnormals.clearAll();
	for (int ifc=g.faceGroups.start(iFaceGroup); ifc<g.faceGroups.end(iFaceGroup); ifc++)
	{
		Face const& f=g.getFace(ifc);
		cvertices.setAt(f.vi(0));
		cvertices.setAt(f.vi(1));
		cvertices.setAt(f.vi(2));
		cnormals.setAt(f.normalIndex(0));
		cnormals.setAt(f.normalIndex(1));
		cnormals.setAt(f.normalIndex(2));
	}
}

void Geometry::extractSubMesh(Geometry const& otherMesh, int isubMesh)
{

	boolN cvertices, cnormals;
	geometry_markFaceGroup(otherMesh, isubMesh, cvertices, cnormals);

	intvectorn newVertexIndex(otherMesh.numVertex());
	intvectorn newNormalIndex(otherMesh.numNormal());

	resizeBuffer(Buffer::VERTEX, cvertices.count());
	int newIndex=0;
	for(int i=0; i<otherMesh.numVertex(); i++)
	{
		if(cvertices(i))
		{
			getVertex(newIndex)=otherMesh.getVertex(i);
			newVertexIndex[i]=newIndex;
			newIndex++;
		}
	}
	ASSERT(newIndex==m_array[Buffer::VERTEX].size());

	resizeBuffer(Buffer::NORMAL,cnormals.count());
	newIndex=0;
	for(int i=0; i<otherMesh.numNormal(); i++)
	{
		if(cnormals(i))
		{
			getNormal(newIndex)=otherMesh.getNormal(i);
			newNormalIndex[i]=newIndex;
			newIndex++;
		}
	}
	ASSERT(newIndex==m_array[Buffer::NORMAL].size());

	for(int i=Buffer::TEXCOORD; i<Buffer::NUM_BUFFER; i++)
		m_array[i]=otherMesh.m_array[i];

	int ifs=otherMesh.faceGroups.start(isubMesh);
	int ife=otherMesh.faceGroups.end(isubMesh);
	resizeIndexBuffer(ife-ifs);

	intvectorn const &nvi=newVertexIndex;
	intvectorn const &nni=newNormalIndex;
	for(int i=ifs; i<ife; i++)
	{
		Face const& f=otherMesh.getFace(i);
		getFace(i-ifs)=f;
		getFace(i-ifs).setIndex(nvi(f.vi(0)), nvi(f.vi(1)), nvi(f.vi(2)));
		getFace(i-ifs).setIndex(nni(f.normalIndex(0)), nni(f.normalIndex(1)), nni(f.normalIndex(2)), Buffer::NORMAL);
	}

	initFaceGroups(*this);
	elements[0]=otherMesh.elements[isubMesh];
}

void Geometry::rigidTransform(transf const& b, int iFaceGroup)
{
	boolN cvertices, cnormals;
	geometry_markFaceGroup(*this, iFaceGroup, cvertices, cnormals);

	for(int i=0; i<numVertex(); i++)
		if (cvertices(i))
			getVertex(i).leftMult(b);

	for(int i=0; i<numNormal(); i++)
		if(cnormals(i))
		{
			getNormal(i).rotate(b.rotation);
			getNormal(i).normalize();
		}
	elements[iFaceGroup].tf.leftMult(b);
}
void Geometry::scaleAndRigidTransform(matrix4 const& m)
{
	Mesh::transform(m);

	vector3 c0(m._11, m._21, m._31);
	vector3 c1(m._12, m._22, m._32);
	vector3 c2(m._13, m._23, m._33);

	vector3 s(c0.length(), c1.length(), c2.length());
	c0.normalize();
	c1.normalize();
	c2.normalize();
	matrix4 mm=m;
	mm.setColumn(0, c0);
	mm.setColumn(1, c1);
	mm.setColumn(2, c2);

	transf b(mm);
	b.rotation.normalize();
	for (int i=0; i<elements.size(); i++)
	{
		elements[i].elementSize*=s;
		elements[i].tf.leftMult(b);
	}
}

void Geometry::transform(matrix4 const& b)
{
	printf("Warning! do not directly transform geometry! use scaleAndRigidTransform instead.\n"); 
	scaleAndRigidTransform(b);
}

static void geometry_elt_scale(Element&e, vector3 const& scalef)
{
	  vector3 lscale=scalef;
	  lscale.rotate(e.tf.rotation.inverse());
	  lscale.x=ABS(lscale.x);
	  lscale.y=ABS(lscale.y);
	  lscale.z=ABS(lscale.z);
	  e.elementSize*=lscale;
	  e.tf.translation*=scalef;
}
void Geometry::scale(vector3 const& scalef)
{
  matrix4 scalem;
  scalem.setScaling(scalef.x,scalef.y,scalef.z);
  Mesh::transform(scalem);
  for (int i=0; i<elements.size(); i++)
  {
	  geometry_elt_scale(elements[i], scalef);
  }
}
void Geometry::scale(vector3 const& scalef, int iFaceGroup)
{
	boolN cvertices, cnormals;
	geometry_markFaceGroup(*this, iFaceGroup, cvertices, cnormals);

	for(int i=0; i<numVertex(); i++)
		if (cvertices(i))
		{
			getVertex(i).x*=scalef.x;
			getVertex(i).y*=scalef.y;
			getVertex(i).z*=scalef.z;
		}

	geometry_elt_scale(elements[iFaceGroup], scalef);
}
void Geometry::scaleElements(vector3 const& scalef)
{
	for (int i=0; i<elements.size(); i++)
	{
		int t=elements[i].elementType;
		if(t!=Element::OBJ)
		{
			if(t==Element::SPHERE)
				elements[i].elementSize*=scalef.x;
			else
			{
				elements[i].elementSize.x*=scalef.x;
				elements[i].elementSize.y*=scalef.y;
				elements[i].elementSize.z*=scalef.z;
			}
		}
	}

	for (int i=0; i<elements.size(); i++)
	{
		if(elements[i].elementType!=Element::OBJ)
			elements[i].elementSize*=scalef;
	}
	_updateMeshFromElements();
}

void Geometry::scaleElements(vector3 const& scalef, int i)
{
	if(elements[i].elementType!=Element::OBJ)
	{
		printf("OBJ element scaling has not been implemented yet");
		return;
	}

	if(elements[i].elementType!=Element::OBJ)
		elements[i].elementSize*=scalef;
	_updateMeshFromElements();
}
void Geometry::_updateMeshFromElements()
{
	Geometry out;
	for (int i=0; i<elements.size(); i++)
	{
		Geometry temp;
		vector3 size=elements[i].elementSize;
		switch(elements[i].elementType)
		{
			case Element::BOX:
				temp.initBox(size);
				break;
			case Element::CYLINDER:
				temp.initCylinder(size.x/2, size.y, 10);
				break;
			case Element::CAPSULE:
				temp.initCapsule(size.x/2, size.y);
				break;
			case Element::ELLIPSOID:
				temp.initEllipsoid(size);
				break;
			case Element::SPHERE:
				temp.initEllipsoid(size);
				break;
			case Element::PLANE:
				temp.initPlane(size.x, size.z);
				break;
		}
		temp.rigidTransform(elements[i].tf);
		out.merge(out, temp);
	}
	*this=out;
}

static TString packEltGeometry(const Element& e)
{
	TString out;
	switch(e.elementType)
	{
		case Element::BOX:
			out.add("geometry Box { size %f %f %f }", 
			 e.elementSize.x, 
			 e.elementSize.y,
			 e.elementSize.z);
		 break;
		case Element::ELLIPSOID:
			out.add("geometry Ellipsoid { size %f %f %f }", 
			 e.elementSize.x, 
			 e.elementSize.y,
			 e.elementSize.z);
		 break;
		 case Element::CYLINDER:
			out.add("geometry Cylinder { radius %f height %f }", 
			 e.elementSize.x/2.0, 
			 e.elementSize.y);
			break;
		 case Element::CAPSULE:
			out.add("geometry Capsule { radius %f height %f }", 
			 e.elementSize.x/2.0, 
			 e.elementSize.y);
			break;
		 default:
			ASSERT(0);
	}
	return out;
}
static TString packTransform(transf const& tf)
{
	TString shape;
	vector3 axis;
	m_real angle;
	tf.rotation.toAxisAngle(axis, angle);
	shape.add("Transform { rotation %f %f %f %f translation %f %f %f",
			axis.x, axis.y, axis.z, angle, 
			tf.translation.x, 
			tf.translation.y, 
			tf.translation.z); 
	return shape;
}
static TString packElt(const Element& e)
{
	double thr=1e-20;
	bool isIdentity=
		isSimilar((e.tf.rotation-quater(1,0,0,0)).length(), thr) &&
		isSimilar((e.tf.translation-vector3(0,0,0)).length(), thr);
	//isIdentity=false; // for debugging
	if(isIdentity)
	{
		TString tf;
		tf.add("  Shape {%s}\n", packEltGeometry(e).ptr());
		return tf;
	}
	else
	{
		TString tf;
		tf.add("  %s", packTransform(e.tf).ptr());
		tf.add("  children Shape {%s}}\n", packEltGeometry(e).ptr());
		return  tf;
	}
}

void Geometry::convertToOBJ() // remove geometry information.
{
	for (int i=0; i<elements.size(); i++)
		elements[i].elementType=Element::OBJ;
}

TString Geometry_packShape(OBJloader::Geometry const& _mesh, const char* dir, const char* shapeFn, transf const& meshCoord)
{
	TString shape;
	bool isOBJ=false;

	for (int i=0; i<_mesh.elements.size(); i++)
		if(_mesh.elements[i].elementType==Element::OBJ)
		{
			isOBJ=true;
			break;
		}

	//isOBJ=true; // for debugging. the geometries will be converted to mesh.
	if(!isOBJ)
	{
		if(_mesh.elements.size()==1)
		{
			shape.add("  children %s\n",packElt(_mesh.elements[0]).ptr());
		}
		else
		{
			shape.add("  children [\n");
			for (int i=0; i<_mesh.elements.size(); i++)
				shape.add(packElt(_mesh.elements[i]));
			shape.add("  ]\n");
		}
		return shape;
	}
#ifdef _MSC_VER
	_mkdir(dir);
#else
	mkdir(dir,0755);
#endif
	OBJloader::Geometry mesh=_mesh;
	mesh.rigidTransform(meshCoord);
	mesh.saveObj(shapeFn,true, false);
	transf globalToLocal=meshCoord.inverse();

	shape.add("  children [ ");
	shape.add(packTransform(globalToLocal).ptr());
	shape.add(" children Shape {geometry OBJ \"%s\"}}]\n", shapeFn);
	return shape;
}
void Geometry::initPlane(double size_x, double size_z) // normal : (0,1,0)
{
	OBJloader::createPlane(*this, 1,1,size_x, size_z);
	// center origin
	for(int i=0; i<numVertex(); i++)
		getVertex(i)+=vector3(-size_x/2.0,0,-size_z/2.0);

	elements.resize(1);
	elements[0].elementType=Element::PLANE;
	elements[0].elementSize=vector3(size_x, 0, size_z);
	elements[0].tf.identity();
}
double Geometry::totalVolume()
{
	double volume=0;
	for(int i=0; i<elements.size(); i++)
	{
		int t=element(i).elementType;

		auto& es=element(i).elementSize;
		double size3=es.x*es.y*es.z;
		switch(t)
		{
			case Element::OBJ:
				return 0;
			case Element::BOX:
				volume+=size3;
				break;
			case Element::CAPSULE:
				volume+=0.523*es.x*es.x*es.x ; 
			case Element::CYLINDER:
				volume+=M_PI*0.25*es.x*es.x*es.y; // approximately 0.785 size3
				break;
			case Element::SPHERE:
			case Element::ELLIPSOID:
				volume+=4.0/3.0*M_PI*0.125*size3; // approximately 0.523 size3
				break;

		}
	}
	return volume;
}


void Geometry::_addVertices(const vectorn& vertices)
{
	int nv=vertices.size()/3;
	int prev_nv=numVertex();
	resizeVertexBuffer(numVertex()+nv);
	for(int i=0; i<nv; i++)
		getVertex(prev_nv+i)=vertices.toVector3(i*3);
}
void Geometry::_addNormals(const vectorn& normals)
{
	int nv=normals.size()/3;
	int prev_nv=numNormal();
	resizeNormalBuffer(numNormal()+nv);
	for(int i=0; i<nv; i++)
		getNormal(prev_nv+i)=normals.toVector3(i*3);
}
void Geometry::_addTexCoords(const vectorn& coords)
{
	int nv=coords.size()/2;
	int prev_nv=numTexCoord();
	resizeUVbuffer(numTexCoord()+nv);
	for(int i=0; i<nv; i++)
		getTexCoord(prev_nv+i)=vector2(coords[i*2], coords[i*2+1]);
}
void Geometry::_addSubMesh(int vstart, int nstart, int texstart, int VERTEX_OFFSET, int NORMAL_OFFSET, int TEXCOORD_OFFSET, const intvectorn& all_indices)
{
	int face_offset=9;
	int elt_offset=3;
	int nf=all_indices.size()/face_offset;
	int prev_nf=numFace();
	resizeIndexBuffer(prev_nf+nf);
	for(int i=0; i<nf; i++)
	{
		auto& f=getFace(prev_nf+i);
		f.vertexIndex(0)=all_indices(i*face_offset+VERTEX_OFFSET)+vstart;
		f.vertexIndex(1)=all_indices(i*face_offset+VERTEX_OFFSET+elt_offset)+vstart;
		f.vertexIndex(2)=all_indices(i*face_offset+VERTEX_OFFSET+elt_offset*2)+vstart;
		f.normalIndex(0)=all_indices(i*face_offset+NORMAL_OFFSET)+nstart;
		f.normalIndex(1)=all_indices(i*face_offset+NORMAL_OFFSET+elt_offset)+nstart;
		f.normalIndex(2)=all_indices(i*face_offset+NORMAL_OFFSET+elt_offset*2)+nstart;
		f.texCoordIndex(0)=all_indices(i*face_offset+TEXCOORD_OFFSET)+texstart;
		f.texCoordIndex(1)=all_indices(i*face_offset+TEXCOORD_OFFSET+elt_offset)+texstart;
		f.texCoordIndex(2)=all_indices(i*face_offset+TEXCOORD_OFFSET+elt_offset*2)+texstart;
		Msg::verify(f.vertexIndex(0)<numVertex(), "nv?");
		Msg::verify(f.vertexIndex(1)<numVertex(), "nv?");
		Msg::verify(f.vertexIndex(2)<numVertex(), "nv?");
		Msg::verify(f.normalIndex(0)<numNormal(), "nn?");
		Msg::verify(f.normalIndex(1)<numNormal(), "nn?");
		Msg::verify(f.normalIndex(2)<numNormal(), "nn?");
		Msg::verify(f.texCoordIndex(0)<numTexCoord(), "nt?");
		Msg::verify(f.texCoordIndex(1)<numTexCoord(), "nt?");
		Msg::verify(f.texCoordIndex(2)<numTexCoord(), "nt?");
	}
	int igrp=faceGroups.size();
	faceGroups.resize(igrp+1);
	faceGroups.start(igrp)=prev_nf;
	faceGroups.end(igrp)=prev_nf+nf;
	elements.resize(igrp+1);
	elements[igrp].elementType=Element::TRI;
	elements[igrp].tf.identity();
}
void Geometry::_addSubMeshPosNormal(int vstart, int nstart, int VERTEX_OFFSET, int NORMAL_OFFSET, const intvectorn& all_indices)
{
	int face_offset=6;
	int elt_offset=2;
	int nf=all_indices.size()/face_offset;
	int prev_nf=numFace();
	resizeIndexBuffer(prev_nf+nf);
	for(int i=0; i<nf; i++)
	{
		auto& f=getFace(prev_nf+i);
		f.vertexIndex(0)=all_indices(i*face_offset+VERTEX_OFFSET)+vstart;
		f.vertexIndex(1)=all_indices(i*face_offset+VERTEX_OFFSET+elt_offset)+vstart;
		f.vertexIndex(2)=all_indices(i*face_offset+VERTEX_OFFSET+elt_offset*2)+vstart;
		f.normalIndex(0)=all_indices(i*face_offset+NORMAL_OFFSET)+nstart;
		f.normalIndex(1)=all_indices(i*face_offset+NORMAL_OFFSET+elt_offset)+nstart;
		f.normalIndex(2)=all_indices(i*face_offset+NORMAL_OFFSET+elt_offset*2)+nstart;
		Msg::verify(f.vertexIndex(0)<numVertex(), "nv?");
		Msg::verify(f.vertexIndex(1)<numVertex(), "nv?");
		Msg::verify(f.vertexIndex(2)<numVertex(), "nv?");
		Msg::verify(f.normalIndex(0)<numNormal(), "nn?");
		Msg::verify(f.normalIndex(1)<numNormal(), "nn?");
		Msg::verify(f.normalIndex(2)<numNormal(), "nn?");
	}
	int igrp=faceGroups.size();
	faceGroups.resize(igrp+1);
	faceGroups.start(igrp)=prev_nf;
	faceGroups.end(igrp)=prev_nf+nf;
	elements.resize(igrp+1);
	elements[igrp].elementType=Element::TRI;
	elements[igrp].tf.identity();
}
