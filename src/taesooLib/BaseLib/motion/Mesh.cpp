#include "stdafx.h"
#include "Mesh.h"
#include "../math/Operator.h"
#include "../image/Image.h"
#include "../image/ImagePixel.h"
#include <iostream>
#include <fstream>
#include "../motion/MotionLoader.h"
using namespace OBJloader;
static void initFaceGroups(Mesh& mesh)
{
	mesh.faceGroups.resize(1);
	mesh.faceGroups.start(0)=0;
	mesh.faceGroups.end(0)=mesh.numFace();
}
static void loadOBJ_unpack(TString const& t, intvectorn& typecode, OBJloader::Face& f,int comp)
{
	intvectorn tn;
	for(int i=0; i!=-1; i=t.findChar(i+1,'/'))
		tn.push_back(i);
	tn[0]=-1;
	tn.push_back(t.length());

	//ASSERT(typecode.size()==tn.size()-1);

	bool faceLoadError = false;

	int tnsize=tn.size();
	int typecode_offset=0;
	for(int i=0; i<tnsize-1; i++)
	{
		if (tn[i]+1==tn[i+1])
		{
			// unused index
			typecode_offset--;
		}
		else
		{
			int n=atoi(t.subString(tn[i]+1, tn[i+1]).ptr());
			ASSERT(n!=0);
			if(typecode_offset + i >= typecode.size()){
				faceLoadError = true;
				continue;
			}
			f.indexes[comp][typecode[i+typecode_offset]]=n-1;
			//assert(typecode[i+typecode_offset]!=0 || n-1<1500);
			//printf("%d,%d, %d, %d\n", n-1, i, typecode_offset, typecode[i+typecode_offset]);
		}
	}
//	if(faceLoadError)
//		printf("FACE LOAD ERROR : There is no vn,vt, but they are in face data.\n");
}

void MeshConnectivity::selectVerticesFromFaces( boolN const& selectedFaces, boolN& selectedVertices)
{
	MeshConnectivity & mc=*this;
	selectedVertices.resize(mc.mesh().numVertex());
	selectedVertices.clearAll();

	for(int i=0; i<selectedFaces.size(); i++)
	{
		if(selectedFaces(i))
		{
			selectedVertices.setAt(mc.mesh().getFace(i).vertexIndex(0));
			selectedVertices.setAt(mc.mesh().getFace(i).vertexIndex(1));
			selectedVertices.setAt(mc.mesh().getFace(i).vertexIndex(2));
		}
	}
}

void MeshConnectivity::selectFacesFromVertices( boolN const& selectedVertices, boolN& selectedFaces)
{
	MeshConnectivity & mc=*this;
	selectedFaces.resize(mc.mesh().numFace());
	selectedFaces.clearAll();

	if (!mc.isValid())
	{
		selectedFaces.setAllValue(true);
		return;
	}
	for(int v=0; v<selectedVertices.size(); v++)
	{
		if(selectedVertices(v))
		{
			for(MeshConnectivity::IncidentFaceIterator j=mc.beginAdjFace(v); j!=mc.endAdjFace(v); ++j)
			{
				if (j->faceIndex==-1)
				{
					printf("warning! selectFacesFromVertices!!!\n");
					printf("vertex %d has errorneous adjacent faces\n",v);
					//ASSERT(false);
					//Msg::error("selectFaces");
					return;
				}
				else
					selectedFaces.setAt(j->faceIndex);
			}
		}
	}
}

static void swapFace(OBJloader::Mesh& mesh, int f1, int f2)
{
	OBJloader::Face t;
	t=mesh.getFace(f1);
	mesh.getFace(f1)=mesh.getFace(f2);
	mesh.getFace(f2)=t;
}
void OBJloader::Mesh::init(const vector3N& vertices, const intvectorn& triangles)
{
	int nT=int(triangles.size()/3);
	resize(vertices.size(), nT);
	for(int i=0; i<vertices.size(); i++)
		getVertex(i)=vertices(i);
	for(int i=0; i<nT; i++)
		getFace(i).setIndex(triangles(i*3), triangles(i*3+1),triangles(i*3+2));
}
void OBJloader::Mesh::init(const vector3N& vertices, const vector3N& normals, const intvectorn& triangles)
{
	int nT=int(triangles.size()/3);
	Msg::verify(normals.size()==vertices.size(),"#normals!=#vertices: cannot use shared indices");
	resize(vertices.size(), vertices.size(), 0,0, nT);
	for(int i=0; i<vertices.size(); i++)
		getVertex(i)=vertices(i);
	for(int i=0; i<normals.size(); i++)
		getNormal(i)=normals(i);
	for(int i=0; i<nT; i++)
		getFace(i).setIndex(triangles(i*3), triangles(i*3+1),triangles(i*3+2));
	copyIndex(Buffer::VERTEX, Buffer::NORMAL);
}
void OBJloader::Mesh::removeFaces(intvectorn const& _faceIndices)
{
	intvectorn faceIndices=_faceIndices;
	int swapIndex=numFace()-1;
	for(int i=0; i<faceIndices.size(); i++)
	{
		int f=faceIndices[i];
		swapFace(*this, f, swapIndex);

		int swapped=faceIndices.findFirstIndex(swapIndex);
		if(swapped!=-1 && swapped>i)
			faceIndices[swapped]=f;
		swapIndex--;
	}
	resizeIndexBuffer(numFace()-faceIndices.size());
}
void OBJloader::Mesh::addVertices(vector3N const& vertices)
{
	int startIndex=numVertex();
	resizeBuffer(Buffer::VERTEX, startIndex+vertices.size());
	for(int i=startIndex; i<numVertex(); i++)
		getVertex(i)=vertices(i-startIndex);
}
void OBJloader::Mesh::addNormals(vector3N const& normals)
{
	int startIndex=numNormal();
	resizeBuffer(Buffer::NORMAL, startIndex+normals.size());
	for(int i=startIndex; i<numNormal(); i++)
		getNormal(i)=normals(i-startIndex);
}

void OBJloader::Mesh::addFaces(intmatrixn const& faces)
{
	Msg::verify(numVertex()>0, "cannot add faces to an empty mesh yet.");
	int startIndex=numFace();
	resizeIndexBuffer(numFace()+faces.rows());
	int c=faces.cols();
	int sourceFaceIndex=0;
	for(int i=startIndex; i<numFace(); i++)
	{
		if(c==4)
			sourceFaceIndex=faces(i-startIndex,3);
		getFace(i)=getFace(sourceFaceIndex);
		getFace(i).vertexIndex(0)=faces(i-startIndex,0);
		getFace(i).vertexIndex(1)=faces(i-startIndex,1);
		getFace(i).vertexIndex(2)=faces(i-startIndex,2);
	}
}

static void findConnectedFaces(OBJloader::Mesh const& mesh, int face, bitvectorn& selectedFaces)
{
	MeshConnectivity mc(mesh);

	bitvectorn selectedVertices;
	selectedVertices.resize(mesh.numVertex());

	selectedFaces.resize(mesh.numFace());

	selectedVertices.clearAll();
	selectedFaces.clearAll();
	selectedFaces.setAt(face);

	int prevCount=1;
	int newCount=1;
	do
	{
		prevCount=newCount;
		
		mc.selectVerticesFromFaces( selectedFaces, selectedVertices);
		mc.selectFacesFromVertices( selectedVertices, selectedFaces);

		newCount=selectedFaces.count();
	}
	while (prevCount!=newCount);
}

static void rearrangeFaces(OBJloader::Mesh& mesh, int startFace, bitvectorn & connectedFaces)
{
	for(int i=0; i<startFace; i++)
	{
		if(connectedFaces(i))
		{
			puts("error in rearrange faces 1");
			throw std::runtime_error("rearrange faces 1");
			return;
		}
	}

	int firstFill=startFace;
	int curr;
	while((curr=connectedFaces.find(firstFill))!=connectedFaces.size())
	{
		if(curr!=firstFill)
		{
			swapFace(mesh, curr, firstFill);

			if(connectedFaces(firstFill)!=false) {
				puts("rearrange faces 2");
				ASSERT(false);
				return;
			}
			if(connectedFaces(curr)!=true) {
				puts("rearrange faces 3");
				ASSERT(false);
				return;
			}
			connectedFaces.setAt(firstFill);
			connectedFaces.clearAt(curr);
		}

		firstFill++;

		if(firstFill==connectedFaces.size()) return;
	}
}
void OBJloader::Mesh::pack(BinaryFile& bf) const
{
	int version=2;
	bf.packInt(version);

	if(version==0) return;
	bf.packInt(numVertex());
	bf.packInt(numNormal());
	bf.packInt(numFace());

	int num_verts=numVertex();
	int num_normal=numNormal();
	int num_face=numFace();
	
	for(int i=0; i<num_verts; i++)
		bf.pack(getVertex(i));
	for(int i=0; i<num_normal; i++)
		bf.pack(getNormal(i));
	for(int i=0; i<num_face; i++)
	{
		bf.packInt(getFace(i).vertexIndex(0));
		bf.packInt(getFace(i).vertexIndex(1));
		bf.packInt(getFace(i).vertexIndex(2));
	}
	for(int i=0; i<num_face; i++)
	{
		bf.packInt(getFace(i).normalIndex(0));
		bf.packInt(getFace(i).normalIndex(1));
		bf.packInt(getFace(i).normalIndex(2));
	}
}
void OBJloader::Mesh::unpack(BinaryFile& bf)
{
	int version=bf.unpackInt();
	if (version==0 || (version>=2&& version<100)){
		_unpackMeshVersion(version,bf);
		return;
	}
	else
		Msg::error("unknown Mesh file format");
}
void OBJloader::Mesh::_unpackMeshVersion(int version, BinaryFile& bf)
{
	if (version==0)
		OBJloader::createBox(*this,5, 5, 5);
	else if(version==2)
	{
		int num_verts=bf.unpackInt();
		int num_normal=bf.unpackInt();
		int num_face=bf.unpackInt();
		//printf("%d %d %d\n", num_verts, num_normal, num_face);
		resize(num_verts,num_normal,0,0,num_face);
		for(int i=0; i<num_verts; i++)
			bf.unpack(getVertex(i));
		for(int i=0; i<num_normal; i++)
			bf.unpack(getNormal(i));
		for(int i=0; i<num_face; i++)
		{
			int fi1=bf.unpackInt();
			int fi2=bf.unpackInt();
			int fi3=bf.unpackInt();
			getFace(i).setIndex(fi1,fi2,fi3);
		}
		for(int i=0; i<num_face; i++)
		{
			int fi1=bf.unpackInt();
			int fi2=bf.unpackInt();
			int fi3=bf.unpackInt();
			getFace(i).setIndex(fi1,fi2,fi3, Buffer::NORMAL);
		}
	}
	else
		Msg::error("unknown Mesh file format");
}
bool OBJloader::Mesh::loadObj(const char* filename)
{
	CTextFile file;
	if(!file.OpenReadFile(filename))
	{
		printf("Error! Faild to open %s\n", filename);
		return false;
	}
	_loadObj(file);
	// todo: loading faceGroups
	initFaceGroups(*this);

#ifdef _DEBUG
	// if these asserts fail, the obj file has some problems.
	int numFc=numFace();
	for (int b=0; b<(int)(Buffer::NUM_BUFFER); b++)
	{
		int numElements=m_array[(Buffer::Type)b].size();

		for (int f=0; f<numFc; f++)
		{
			if(
				   	!(getFace(f).indexes[0][(Buffer::Type)b]>=-1 &&
				getFace(f).indexes[0][(Buffer::Type)b] < numElements) ||
			!(getFace(f).indexes[1][(Buffer::Type)b]>=-1 &&
				getFace(f).indexes[1][(Buffer::Type)b] < numElements) ||
			!(getFace(f).indexes[2][(Buffer::Type)b]>=-1 &&
				getFace(f).indexes[2][(Buffer::Type)b] < numElements))
			{
				printf("warning!!! this mesh's face %d (%d,%d,%d) has a problem with buffer %d\n", f, 
				getFace(f).indexes[0][(Buffer::Type)b],		
				getFace(f).indexes[1][(Buffer::Type)b],		
				getFace(f).indexes[2][(Buffer::Type)b],	b);
			}
		}
	}
#endif
	return true;
}

void OBJloader::Mesh::_loadObj(CTextFile& file)
{
	TString token;
	TString currMode="v";
	intvectorn typeCode;

	vector3N pos;

	resize(0,0,0,0,0);

	while(1)
	{
		char * t=file.GetToken();
		if(t==NULL)
			break;
		token=t;
		if(token=="mtllib")
		{
			TString mtlname=file.GetToken();
			continue;
		}
		else if(token=="g")
		{
			TString groupname=file.GetToken();
			continue;
		}
		else if(token=="o")
		{
			TString objectname=file.GetToken();
			continue;
		}
		else if(token=="s")
		{
			TString groupnum=file.GetToken();
			continue;
		}
		else if(token=="usemtl")
		{
			TString mtlname=file.GetToken();
			continue;
		}
		if(currMode!=token)
		{
			if(currMode=="v")
			{
				int nv=numVertex();
				resizeBuffer(Buffer::VERTEX,nv+pos.size());

				for(int i=0; i<pos.size(); i++)
					getVertex(nv+i)=pos[i];

				if(typeCode.findFirstIndex(Buffer::VERTEX)==-1)
					typeCode.push_back(Buffer::VERTEX);
			}
			else if(currMode=="vt")
			{
				int nt=numTexCoord();
				resizeBuffer(Buffer::TEXCOORD, nt+pos.size());

				for(int i=0; i<pos.size(); i++)
					getTexCoord(nt+i)=vector2(pos[i][0], pos[i][1]);

				if(typeCode.findFirstIndex(Buffer::TEXCOORD)==-1)
					typeCode.push_back(Buffer::TEXCOORD);
			}
			else if(currMode=="vn")
			{
				int nn=numNormal();
				resizeBuffer(Buffer::NORMAL, nn+pos.size());

				for(int i=0; i<pos.size(); i++)
					getNormal(nn+i)=pos[i];

				if(typeCode.findFirstIndex(Buffer::NORMAL)==-1)
					typeCode.push_back(Buffer::NORMAL);
			}
			pos.setSize(0);
		}
		if(token=="v" || token=="vn")
		{
			double x=atof(file.GetToken());
			double y=atof(file.GetToken());
			double z=atof(file.GetToken());

			double discard;
			while(true)
			{
				TString token=file.GetToken();
				if (sscanf(token.ptr(), "%lf", &discard)!=1) {
					file.Undo();
					break;
				}
			}
//			ASSERT(x!=0.0);
//			ASSERT(y!=0.0);
//			ASSERT(z!=0.0);
			pos.pushBack(vector3(x,y,z));
		}
		else if(token=="vt")
		{
			double x=atof(file.GetToken());
			double y=atof(file.GetToken());
			pos.pushBack(vector3(x,y,0));
		}
		else if(token=="f")
		{
			OBJloader::Face f;
			TString temp=file.GetToken();
			int left=temp.findChar(0,'/');
			if(left!=-1)
			{
				TString ta,tb,tc;
				ta=temp;
				loadOBJ_unpack(ta,typeCode,f,0);
				tb=file.GetToken();
				loadOBJ_unpack(tb,typeCode,f,1);
				tc=file.GetToken();
				loadOBJ_unpack(tc,typeCode,f,2);

				TString temp=file.GetToken();
				while (temp.length()>0 && temp[0]>='0' && temp[0]<='9')
				{
					//printf("a: %d %d %d//", f.vertexIndex(0), f.vertexIndex(1), f.vertexIndex(2));
					m_arrayFace.push_back(f);
					// quad mesh: triangulate
					loadOBJ_unpack(ta, typeCode, f, 0);
					loadOBJ_unpack(tc, typeCode, f, 1);
					loadOBJ_unpack(temp, typeCode, f, 2);
					tc=temp;
					//printf("b: %d %d %d\n", f.vertexIndex(0), f.vertexIndex(1), f.vertexIndex(2));
					temp=file.GetToken();

				}
				file.Undo();
			}
			else
			{
				int i=atoi(temp);
				int j=atoi(file.GetToken());
				int k=atoi(file.GetToken());
				f.setIndex(i-1,j-1,k-1, Buffer::VERTEX);

				TString temp=file.GetToken();
				if (temp.length()>0 && temp[0]>='0' && temp[0]<='9')
				{
					m_arrayFace.push_back(f);
					// quad mesh: triangulate
					int l=atoi(temp);
					f.setIndex(i-1,k-1, l-1, Buffer::VERTEX);
				}
				else
					file.Undo();
				/*
				ASSERT(numNormal()==0);
				ASSERT(numTexCoord()==0);
				ASSERT(numColor()==0);*/
			}
			m_arrayFace.push_back(f);
		}
		else if(token=="off")
		{
		}
		else
		{
			float f;
			if(sscanf(token.ptr(), "%f", &f)==1 && currMode=="vt")
			{
				printf("ignoring vt %s ", token.ptr());
				token="vt";
				printf("%s\n", token.ptr());
			}
			else
			{
				printf("%s %s\n", currMode.ptr(), token.ptr());
				Msg::msgBox("unknown token %s (quad mesh?)", token.ptr());
			}
		}
		currMode=token;
	}
}

vector3& OBJloader::Mesh::getFaceCorner(int iFace, int icorner)
{
	return getVertex(getFace(iFace).vi(icorner));
}
vector3 OBJloader::Mesh::calcFaceCenter(int i) const
{
	vector3 const& v1=getVertex(getFace(i).vi(0));
	vector3 const& v2=getVertex(getFace(i).vi(1));
	vector3 const& v3=getVertex(getFace(i).vi(2));

	return (v1+v2+v3)/3.0;
}
vector3 OBJloader::Mesh::calcFaceNormal(int i) const
{
	vector3 const& v1=getVertex(getFace(i).vi(0));
	vector3 const& v2=getVertex(getFace(i).vi(1));
	vector3 const& v3=getVertex(getFace(i).vi(2));

	vector3 d1, d2;
	d1.difference(v1,v2);
	d2.difference(v2,v3);
	vector3 n;
	n.cross(d1,d2);
	n.normalize();
	return n;
}

void OBJloader::Mesh::transform(matrix4 const& b)
{
	for(int i=0; i<numVertex(); i++)
		getVertex(i).leftMult(b);

	for(int i=0; i<numNormal(); i++)
	{
		getNormal(i).rotate(b);
		getNormal(i).normalize();
	}
}

bool OBJloader::Mesh::loadMesh(const char* file_name_, bool bInit)
{
	int i, n, m;
	vector3 x_tmp;
	std::ifstream fin;

	TString fn(file_name_);
	if (fn.right(4).toUpper()==".OBJ")
		Msg::error("use loadOBJ instead!!!");

	fin.open(file_name_);

	if ( !fin.is_open() ) return false;

	// get the number of nodes and meshes
	fin >> n >> m;

	if(bInit)
		resize(n,m);
	else
	{
		resizeBuffer(Buffer::VERTEX, n);
		resizeIndexBuffer(m);
	}

	// get initial node positions from file
	for (i=0; i<n; i++) {
		// node position in {mesh file reference frame}
		fin >> x_tmp[0] >> x_tmp[1] >> x_tmp[2];

		// position of nodes in {body}
		getVertex(i)=x_tmp;
	}

	// get mesh indexes from file
	for (i=0; i<m; i++) {
		int mesh_index[3];
		fin >> mesh_index[0] >> mesh_index[1] >> mesh_index[2];
		getFace(i).setIndex(mesh_index[0],mesh_index[1],mesh_index[2]);
	}

	fin.close();
	return true;
}

bool OBJloader::Mesh::saveObj(const char* filename, bool vn, bool vt)
{
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(filename);
	if ( !fout.is_open() ) return false;
	_saveObj(fout, vn, vt);
	fout.close();
	return true;
}

void OBJloader::Mesh::_saveObj(std::ofstream & fout, bool vn, bool vt)
{

	// get the number of nodes and mesh elements
	{
		int n=numVertex();

		// get initial node positions from file
		for (int i=0; i<n; i++) {
			vector3& x_tmp=getVertex(i);

			// node position in {mesh file reference frame}
			fout << "v " <<x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<std::endl;
		}
	}

	if(vt)
	{
		int n=numTexCoord();
		for (int i=0; i<n; i++) {
			vector2& x_tmp=getTexCoord(i);

			// node position in {mesh file reference frame}
			fout << "vt " <<x_tmp(0) <<" "<< x_tmp(1) <<std::endl;
		}
	}

	if(vn)
	{
		int n=numNormal();
		for (int i=0; i<n; i++) {
			vector3& x_tmp=getNormal(i);

			// node position in {mesh file reference frame}
			fout << "vn " <<x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<std::endl;
		}
	}

	int m=m_arrayFace.size();

	// get mesh indexes from file
	if (faceGroups.size()>=1 &&  faceGroups.end(faceGroups.size()-1)==m)
	{
		for(int f=0; f<faceGroups.size(); f++)
		{
			fout << "s "<< f +1<<std::endl;
			for (int i=faceGroups.start(f); i<faceGroups.end(f); i++) {
				fout << "f "<< getFace(i).vi(0) +1;
				if(vt)
					fout << "/"<< getFace(i).texCoordIndex(0) +1;
				if(vn)
					fout << "/"<< getFace(i).normalIndex(0) +1;

				fout << " " << getFace(i).vi(1) +1;
				if(vt)
					fout << "/"<< getFace(i).texCoordIndex(1) +1;
				if(vn)
					fout << "/"<< getFace(i).normalIndex(1) +1;

				fout << " " << getFace(i).vi(2) +1;
				if(vt)
					fout << "/"<< getFace(i).texCoordIndex(2) +1;
				if(vn)
					fout << "/"<< getFace(i).normalIndex(2) +1;
				fout<<std::endl;
			}
		}
		return;
	}

	for (int i=0; i<m; i++) {
		fout << "f "<< getFace(i).vi(0) +1;
		if(vt)
			fout << "/"<< getFace(i).texCoordIndex(0) +1;
		if(vn)
			fout << "/"<< getFace(i).normalIndex(0) +1;

		fout << " " << getFace(i).vi(1) +1;
		if(vt)
			fout << "/"<< getFace(i).texCoordIndex(1) +1;
		if(vn)
			fout << "/"<< getFace(i).normalIndex(1) +1;

		fout << " " << getFace(i).vi(2) +1;
		if(vt)
			fout << "/"<< getFace(i).texCoordIndex(2) +1;
		if(vn)
			fout << "/"<< getFace(i).normalIndex(2) +1;
		fout<<std::endl;
	}

}

bool OBJloader::Mesh::saveMesh(const char *file_name_)
{
	int n, m;
	vector3 x_tmp;
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(file_name_);

	if ( !fout.is_open() ) return false;

	// get the number of nodes and mesh elements

	n=numVertex();
	m=m_arrayFace.size();
	fout << n <<" "<< m<<std::endl;

	// get initial node positions from file
	for (int i=0; i<n; i++) {
		x_tmp=getVertex(i);

		// node position in {mesh file reference frame}
		fout << x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<std::endl;

	}

	// get mesh indexes from file
	for (int i=0; i<m; i++) {
		fout << getFace(i).vi(0) <<" "<< getFace(i).vi(1) <<" "<< getFace(i).vi(2)<<std::endl;
	}

	fout.close();
	return true;
}
void OBJloader::Mesh::copyFrom(Mesh const& otherMesh)
{
	for(int i=0; i<Buffer::NUM_BUFFER; i++)
		m_array[i]=otherMesh.m_array[i];

	resizeIndexBuffer(otherMesh.numFace());

	for(int i=0; i<otherMesh.numFace(); i++)
		getFace(i)=otherMesh.getFace(i);

	if(otherMesh.faceGroups.size()==0)
	{
		initFaceGroups(*this);
	}
	else
		faceGroups=otherMesh.faceGroups;
}

void OBJloader::Mesh::resizeBuffer(Buffer::Type t, int n)
{
	m_array[t].resize(n, Buffer::defaultNumElts(t));
}

void OBJloader::Mesh::copyIndex(Buffer::Type from, Buffer::Type to)
{
	for(int i=0; i<numFace(); i++)
		for(int j=0; j<3; j++)
			getFace(i).indexes[j](to)=getFace(i).indexes[j](from);
}
void OBJloader::Mesh::vertFlipUV()
{
	for(int i=0; i<numTexCoord(); i++)
	{
		getTexCoord(i)(1)=1.0-getTexCoord(i)(1);
	}
}

void OBJloader::Mesh::calculateVertexNormal()
{
	vector3N faceNormal;
	faceNormal.setSize(numFace());

	for(int i=0; i<numFace(); i++)
	{
		faceNormal(i)=calcFaceNormal(i);
	}

	if(!numNormal())
	{
		resizeBuffer(Buffer::NORMAL, numVertex());
		copyIndex(Buffer::VERTEX, Buffer::NORMAL);
	}

	for(int i=0; i<numNormal(); i++)
		getNormal(i).setValue(0,0,0);

	for(int i=0; i<numFace(); i++)
	{
		getNormal(getFace(i).normalIndex(0))+=faceNormal(i);
		getNormal(getFace(i).normalIndex(1))+=faceNormal(i);
		getNormal(getFace(i).normalIndex(2))+=faceNormal(i);
	}

	for(int i=0; i<numNormal(); i++)
		getNormal(i).normalize();
}


void OBJloader::createPlane(Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ)
{
	// z|
	//  + ->x    numSegX=2, numSegZ=2, sizeX=sizeZ=2인경우.

	//  0   1   2
	//  +---+---+  2
	//  | / | / |
	//  +---+---+  1
	//  | / | / |
	//  +---+---+  0
	int numVertex=(numSegX+1)*(numSegZ+1);
	mesh.resize(numVertex, 1,numVertex, 0, numSegX*numSegZ*2);

	for(int i=0; i<=numSegZ; i++)
	{
		for(int j=0; j<=numSegX; j++)
		{
			int index=i*(numSegZ+1)+j;

			m_real x=sop::map((m_real)j, 0, numSegX, 0, sizeX);
			m_real z=sop::map((m_real)i, 0, numSegZ, 0, sizeZ);
			mesh.getVertex(index).setValue(x,0,z);
			m_real tu=sop::map((m_real)j, 0, numSegX, 0, 1);
			m_real tv=sop::map((m_real)i, 0, numSegZ, 1, 0);
			mesh.getTexCoord(index)=vector2(tu, tv);
		}
	}
	mesh.getNormal(0).setValue(0,1,0);

	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			// insert lower triangle
			int faceIndex=i*(numSegZ)+j;
			mesh.getFace(faceIndex).vertexIndex(0)=i*(numSegZ+1)+j;
			mesh.getFace(faceIndex).vertexIndex(2)=i*(numSegZ+1)+j+1;
			mesh.getFace(faceIndex).vertexIndex(1)=(i+1)*(numSegZ+1)+j+1;
			mesh.getFace(faceIndex).texCoordIndex(0)=i*(numSegZ+1)+j;
			mesh.getFace(faceIndex).texCoordIndex(2)=i*(numSegZ+1)+j+1;
			mesh.getFace(faceIndex).texCoordIndex(1)=(i+1)*(numSegZ+1)+j+1;
			mesh.getFace(faceIndex).normalIndex(0)=0;
			mesh.getFace(faceIndex).normalIndex(1)=0;
			mesh.getFace(faceIndex).normalIndex(2)=0;

			// insert upper triangle
			faceIndex+=numSegX*numSegZ;
			mesh.getFace(faceIndex).vertexIndex(0)=i*(numSegZ+1)+j;
			mesh.getFace(faceIndex).vertexIndex(2)=(i+1)*(numSegZ+1)+(j+1);
			mesh.getFace(faceIndex).vertexIndex(1)=(i+1)*(numSegZ+1)+j;
			mesh.getFace(faceIndex).texCoordIndex(0)=i*(numSegZ+1)+j;
			mesh.getFace(faceIndex).texCoordIndex(2)=(i+1)*(numSegZ+1)+(j+1);
			mesh.getFace(faceIndex).texCoordIndex(1)=(i+1)*(numSegZ+1)+j;
			mesh.getFace(faceIndex).normalIndex(0)=0;
			mesh.getFace(faceIndex).normalIndex(1)=0;
			mesh.getFace(faceIndex).normalIndex(2)=0;
		}
	}

	initFaceGroups(mesh);
}
void OBJloader::createBox(Mesh& mesh, m_real sizeX, m_real sizeY, m_real sizeZ)
{
	mesh.resize(8,6,4,0,12);

	// vertex
	mesh.getVertex(0)=vector3(-1, 1, 1);
	mesh.getVertex(1)=vector3( 1, 1, 1);
	mesh.getVertex(2)=vector3( 1, 1,-1);
	mesh.getVertex(3)=vector3(-1, 1,-1);
	mesh.getVertex(4)=vector3(-1,-1, 1);
	mesh.getVertex(5)=vector3( 1,-1, 1);
	mesh.getVertex(6)=vector3( 1,-1,-1);
	mesh.getVertex(7)=vector3(-1,-1,-1);

	for(int i=0; i<8; i++)
	{
		mesh.getVertex(i).x*=sizeX/2.0;
		mesh.getVertex(i).y*=sizeY/2.0;
		mesh.getVertex(i).z*=sizeZ/2.0;
	}
	// normal
	mesh.getNormal(0)=vector3( 1,0,0);
	mesh.getNormal(1)=vector3(-1,0,0);
	mesh.getNormal(2)=vector3(0, 1,0);
	mesh.getNormal(3)=vector3(0,-1,0);
	mesh.getNormal(4)=vector3(0,0, 1);
	mesh.getNormal(5)=vector3(0,0,-1);

	// texcoord
	mesh.getTexCoord(0)=vector2(0,0);
	mesh.getTexCoord(1)=vector2(1,0);
	mesh.getTexCoord(2)=vector2(1,1);
	mesh.getTexCoord(3)=vector2(0,1);

	// top
	mesh.getFace(0).setIndex(0,1,2);
	mesh.getFace(1).setIndex(0,2,3);
	mesh.getFace(0).setIndex(2,2,2, OBJloader::Buffer::NORMAL);
	mesh.getFace(1).setIndex(2,2,2, OBJloader::Buffer::NORMAL);
	mesh.getFace(0).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(1).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	// front
	mesh.getFace(2).setIndex(0,4,5);
	mesh.getFace(3).setIndex(0,5,1);
	mesh.getFace(2).setIndex(4,4,4, OBJloader::Buffer::NORMAL);
	mesh.getFace(3).setIndex(4,4,4, OBJloader::Buffer::NORMAL);
	mesh.getFace(2).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(3).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	// right
	mesh.getFace(4).setIndex(1,5,6);
	mesh.getFace(5).setIndex(1,6,2);
	mesh.getFace(4).setIndex(0,0,0, OBJloader::Buffer::NORMAL);
	mesh.getFace(5).setIndex(0,0,0, OBJloader::Buffer::NORMAL);
	mesh.getFace(4).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(5).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	// left
	mesh.getFace(6).setIndex(3,7,4);
	mesh.getFace(7).setIndex(3,4,0);
	mesh.getFace(6).setIndex(1,1,1, OBJloader::Buffer::NORMAL);
	mesh.getFace(7).setIndex(1,1,1, OBJloader::Buffer::NORMAL);
	mesh.getFace(6).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(7).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	// back
	mesh.getFace(8).setIndex(2,6,7);
	mesh.getFace(9).setIndex(2,7,3);
	mesh.getFace(8).setIndex(5,5,5, OBJloader::Buffer::NORMAL);
	mesh.getFace(9).setIndex(5,5,5, OBJloader::Buffer::NORMAL);
	mesh.getFace(8).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(9).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	// bottom
	mesh.getFace(10).setIndex(6,5,4);
	mesh.getFace(11).setIndex(6,4,7);
	mesh.getFace(10).setIndex(3,3,3, OBJloader::Buffer::NORMAL);
	mesh.getFace(11).setIndex(3,3,3, OBJloader::Buffer::NORMAL);
	mesh.getFace(10).setIndex(0,1,2, OBJloader::Buffer::TEXCOORD);
	mesh.getFace(11).setIndex(0,2,3, OBJloader::Buffer::TEXCOORD);

	initFaceGroups(mesh);
}

void OBJloader::createCylinder(Mesh& mesh, m_real radius, m_real height, int ndiv)
{

	vector3 front(0,0,1);

	mesh.resize(ndiv*2+2, ndiv+2,0,0,ndiv*4);

	int nFV=ndiv*2;

	mesh.getVertex(nFV)=vector3(0,height/2.0,0);
	mesh.getVertex(nFV+1)=vector3(0,height/-2.0,0);
	mesh.getNormal(ndiv)=vector3(0,1,0);
	mesh.getNormal(ndiv+1)=vector3(0,-1,0);

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
		mesh.getFace(i*4).setIndex(nFV, v0, v2);
		mesh.getFace(i*4+1).setIndex(v0, v1, v3);
		mesh.getFace(i*4+2).setIndex(v0, v3, v2);
		mesh.getFace(i*4+3).setIndex(v1, nFV+1, v3);

		int n2=(i+1)%ndiv;
		mesh.getFace(i*4).setIndex(ndiv, ndiv, ndiv, OBJloader::Buffer::NORMAL);
		mesh.getFace(i*4+1).setIndex(i, i, n2, OBJloader::Buffer::NORMAL);
		mesh.getFace(i*4+2).setIndex(i, n2, n2, OBJloader::Buffer::NORMAL);
		mesh.getFace(i*4+3).setIndex(ndiv+1, ndiv+1, ndiv+1, OBJloader::Buffer::NORMAL);
	}

	initFaceGroups(mesh);
}
void OBJloader::createCircle(Mesh& mesh, vector3 const& center, m_real radius, int ndiv)
{
	vector3 front(0,0,1);

	mesh.resize(ndiv+1, 1,0,0,ndiv);

	int nFV=ndiv;

	mesh.getVertex(nFV)=center;
	mesh.getNormal(0)=vector3(0,1,0);

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

		mesh.getVertex(i)=nf*radius+center;

		int v0=i;
		int v2=(i+1)%nFV;
		mesh.getFace(i).setIndex(nFV, v0, v2);
		mesh.getFace(i).setIndex(0, 0, 0, OBJloader::Buffer::NORMAL);
	}

	initFaceGroups(mesh);
}

void OBJloader::convertTerrainToBMP(const char* filename, int sizeX,int sizeY, const char* outfile)
{
	int sx=1;
	int sy=1;
	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	FILE* file=fopen(filename, "rb");
	fread( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);

	CImage temp;

	temp.Create(sizeX, sizeY);//, 24);
	CImagePixel pixel(&temp);

	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			int index=y*sizeX+x;

			int color=m_real(image(y*sy,x*sx))/65536.0*255;
			pixel.SetPixel(x, y, CPixelRGB8(color, color, color));
		}
	}

	temp.Save(outfile);
}
void OBJloader::convertTerrainFromBMP(const char* filename, const char* outfile)
{
	CImage temp;
	int sx=1;
	int sy=1;

	temp.Load(filename);

	int sizeY=temp.GetHeight();
	int sizeX=temp.GetWidth();

	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			int index=y*sizeX+x;

			unsigned char color=temp.GetPixel(x,y)->R;
			image(y*sy, x*sx)=color*256;
		}
	}

	ASSERT(sizeof(short)==2);
	FILE* file=fopen(outfile, "wb");
	fwrite( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);
}

void OBJloader::createTerrain(Mesh& mesh, const char* filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ)
{
	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	FILE* file=fopen(filename, "rb");
	fread( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);

	fclose(file);
	OBJloader::_createTerrain(mesh, image, sizeX, sizeY, width, height, heightMax, ntexSegX, ntexSegZ);
}

void OBJloader::_createTerrain(Mesh& mesh, Raw2Bytes& image, int _sizeX, int _sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ)
{
	int sizeX=_sizeX;
	int sizeY=_sizeY;
	int sx=1;
	int sy=1;


#define BAD_NOTEBOOK
#ifdef BAD_NOTEBOOK
	// for my notebook, where 512 by 512 mesh doesn't load.
	sizeX=MIN(64, _sizeX);
	sx=_sizeX/sizeX;
	sizeY=MIN(64, _sizeY);
	sy=_sizeY/sizeY;
#endif

//#define SAVE_BMP
#ifdef SAVE_BMP
	CImage temp;

	temp.Create(sizeX, sizeY);//, 24);
	CImagePixel pixel(&temp);
#endif

	int numVertex=sizeX*sizeY;
	int numSegX=sizeX-1;
	int numSegZ=sizeY-1;
	int numFace=(numSegX*numSegZ*2);

	mesh.resize(numVertex, numVertex, numVertex, 0, numFace);
	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			int index=y*sizeX+x;

			m_real& tu=mesh.getTexCoord(index)(0);
			m_real& tv=mesh.getTexCoord(index)(1);

			tu=m_real(x)*m_real(ntexSegX)/m_real(sizeX-1);
			tv=m_real(y)*m_real(ntexSegZ)/m_real(sizeY-1);

			vector3& pos=mesh.getVertex(index);
			pos.x=m_real(x)*width/m_real(sizeX-1);
			pos.z=m_real(y)*height/m_real(sizeY-1);
			pos.y=heightMax*m_real(image(y*sy,x*sx))/65536.0;

#ifdef SAVE_BMP
			int color=m_real(image(y*sy,x*sx))/65536.0*255;
			pixel.SetPixel(x, y, CPixelRGB8(color, color, color));
#endif
		}
	}

#ifdef SAVE_BMP
	temp.Save("terrain.bmp");
#endif




	vector3N normalSum(mesh.numVertex());
	normalSum.setAllValue(vector3(0,0,0));

	intvectorn normalCount(mesh.numVertex());
	normalCount.setAllValue(0);

	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			// insert lower triangle
			int faceIndex=i*(numSegX)+j;
			{
				Face& f=mesh.getFace(faceIndex);
				f.vertexIndex(0)=i*(numSegX+1)+j;
				f.vertexIndex(1)=(i+1)*(numSegX+1)+j+1;
				f.vertexIndex(2)=i*(numSegX+1)+j+1;
				f.texCoordIndex(0)=i*(numSegX+1)+j;
				f.texCoordIndex(1)=(i+1)*(numSegX+1)+j+1;
				f.texCoordIndex(2)=i*(numSegX+1)+j+1;
				f.normalIndex(0)=i*(numSegX+1)+j;
				f.normalIndex(1)=(i+1)*(numSegX+1)+j+1;
				f.normalIndex(2)=i*(numSegX+1)+j+1;

				vector3 v0=mesh.getVertex(f.vi(0));
				vector3 v1=mesh.getVertex(f.vi(1));
				vector3 v2=mesh.getVertex(f.vi(2));

				vector3 faceNormal;
				faceNormal.cross(v1-v0, v2-v0);
				faceNormal.normalize();

				for(int vv=0; vv<3; vv++)
				{
					normalSum[f.vi(vv)]+=faceNormal;
					normalCount[f.vi(vv)]++;
				}
			}

			// insert upper triangle
			faceIndex+=numSegX*numSegZ;
			Face& f=mesh.getFace(faceIndex);
			f.vertexIndex(0)=i*(numSegX+1)+j;
			f.vertexIndex(1)=(i+1)*(numSegX+1)+j;
			f.vertexIndex(2)=(i+1)*(numSegX+1)+(j+1);
			f.texCoordIndex(0)=i*(numSegX+1)+j;
			f.texCoordIndex(1)=(i+1)*(numSegX+1)+j;
			f.texCoordIndex(2)=(i+1)*(numSegX+1)+(j+1);
			f.normalIndex(0)=i*(numSegX+1)+j;
			f.normalIndex(1)=(i+1)*(numSegX+1)+j;
			f.normalIndex(2)=(i+1)*(numSegX+1)+(j+1);

			vector3 v0=mesh.getVertex(f.vi(0));
			vector3 v1=mesh.getVertex(f.vi(1));
			vector3 v2=mesh.getVertex(f.vi(2));

			vector3 faceNormal;
			faceNormal.cross(v1-v0, v2-v0);
			faceNormal.normalize();

			for(int vv=0; vv<3; vv++)
			{
				normalSum[f.vi(vv)]+=faceNormal;
				normalCount[f.vi(vv)]++;
			}
		}
	}

	for(int i=0; i<mesh.numVertex(); i++)
	{
		mesh.getNormal(i)=normalSum[i]/m_real(normalCount[i]);
		mesh.getNormal(i).normalize();
	}

	initFaceGroups(mesh);
}
void OBJloader::classifyTriangles(OBJloader::Mesh& mesh)
{
	
	mesh.faceGroups.resize(0);
	try {
		int startFace=0;
		bitvectorn connectedFaces;
		while(1)
		{
			try 
			{
				findConnectedFaces(mesh, startFace, connectedFaces);
				rearrangeFaces(mesh, startFace, connectedFaces);
			}
			catch( std::exception& e)
			{
				connectedFaces.resize(mesh.numFace());
				connectedFaces.setAllValue(true);
			}

			int endFace=startFace+connectedFaces.count();
			mesh.faceGroups.pushBack(startFace, endFace);

			if(endFace==mesh.numFace() ) break;
			if(connectedFaces.count()==0) std::runtime_error("classifyTriangles 1");
			startFace=endFace;
		}
	}
	catch(std::runtime_error& e)
	{
		initFaceGroups(mesh);
	}
}

int& Face::vertexIndex(int i) {return indexes[i](Buffer::VERTEX);}
int const& Face::vertexIndex(int i) const{return indexes[i](Buffer::VERTEX);}
int& Face::normalIndex(int i) {return indexes[i](Buffer::NORMAL);}
int const& Face::normalIndex(int i) const{return indexes[i](Buffer::NORMAL);}
int& Face::texCoordIndex(int i) {return indexes[i](Buffer::TEXCOORD);}
int const& Face::texCoordIndex(int i) const{return indexes[i](Buffer::TEXCOORD);}
int& Face::colorIndex(int i) {return indexes[i](Buffer::COLOR);}
int const& Face::colorIndex(int i) const{return indexes[i](Buffer::COLOR);}

void Face::setIndex(int i, int j, int k, Buffer::Type t)
{
	indexes[0](t)=i;
	indexes[1](t)=j;
	indexes[2](t)=k;
}

void Face::setIndex(index3 const& idx, Buffer::Type t)
{
	indexes[0](t)=idx(0);
	indexes[1](t)=idx(1);
	indexes[2](t)=idx(2);
}
index3 Face::getIndex(Buffer::Type t) const
{
	index3 idx;
	idx(0)=indexes[0](t);
	idx(1)=indexes[1](t);
	idx(2)=indexes[2](t);
	return idx;
}

int Mesh::numVertex() const
{
	return m_array[Buffer::VERTEX].size();
}
int Mesh::numNormal() const
{
	return m_array[Buffer::NORMAL].size();
}
int Mesh::numTexCoord() const
{
	return m_array[Buffer::TEXCOORD].size();
}
int Mesh::numColor() const
{
	return m_array[Buffer::COLOR].size();
}

vector3& Mesh::getVertex(int i)
{
	ASSERT(i>=0 && i<numVertex());
	return m_array[Buffer::VERTEX].ref3(i);
}
vector3 const& Mesh::getVertex(int i) const
{
	ASSERT(i>=0 && i<numVertex());
	return m_array[Buffer::VERTEX].ref3(i);
}
vector3& Mesh::getNormal(int i)
{
	ASSERT(i>=0 && i<numNormal());
	return m_array[Buffer::NORMAL].ref3(i);
}
vector3 const& Mesh::getNormal(int i) const
{
	ASSERT(i>=0 && i<numNormal());
	return m_array[Buffer::NORMAL].ref3(i);
}
vector2& Mesh::getTexCoord(int i)
{
	ASSERT(i>=0 && i<numTexCoord());
	return m_array[Buffer::TEXCOORD].ref2(i);
}
vector2 const& Mesh::getTexCoord(int i) const
{
	ASSERT(i>=0 && i<numTexCoord());
	return m_array[Buffer::TEXCOORD].ref2(i);
}

void Mesh::merge(Mesh const& a, Mesh const& b)
{
	if(&a==this)
	{
		if (a.numVertex()==0)
		{
			copyFrom(b);
			return;
		}
		Mesh aa(a);
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
		Mesh bb(b);
		merge(a,bb);
		return;
	}
	
	ASSERT(a.numFace()&&b.numFace());
	ASSERT(a.numVertex()&& b.numVertex());
	bool useNormal=a.numNormal()!=0 && b.numNormal()!=0;
	bool useTexCoord=a.numTexCoord()!=0 && b.numTexCoord()!=0;
	bool useColor=a.numColor()!=0 && b.numColor()!=0;

	copyFrom(a);
	if(b.faceGroups.size()==0)
	{
		faceGroups.pushBack(a.numFace(), a.numFace()+b.numFace());
	}
	else
	{
		for(int i=0; i<b.faceGroups.size(); i++)
		{
			faceGroups.pushBack(b.faceGroups.start(i)+a.numFace(),
				b.faceGroups.end(i)+a.numFace());
		}
	}

	resize(a.numVertex()+b.numVertex(),
		useNormal?a.numNormal()+b.numNormal():0,
		useTexCoord?a.numTexCoord()+b.numTexCoord():0,
		useColor?a.numColor()+b.numColor():0,
		a.numFace()+b.numFace());

	for(int i=0; i<b.numVertex(); i++)
		getVertex(i+a.numVertex())=b.getVertex(i);

	if(useNormal)
	for(int i=0; i<b.numNormal(); i++)
		getNormal(i+a.numNormal())=b.getNormal(i);

	if(useTexCoord)
	for(int i=0; i<b.numTexCoord(); i++)
		getTexCoord(i+a.numTexCoord())=b.getTexCoord(i);

	if(useColor)
	for(int i=0; i<b.numColor(); i++)
		getColor(i+a.numColor())=b.getColor(i);

	for(int i=0; i<b.numFace(); i++)
	{
		for(int bf=0; bf<OBJloader::Buffer::NUM_BUFFER; bf++)
		{
			OBJloader::Buffer::Type bff=(OBJloader::Buffer::Type )bf;
			int numE=a.buffer(bff).size();
			this->getFace(i+a.numFace()).setIndex(
				b.getFace(i).getIndex(bff)
				+index3(numE, numE, numE), bff);
		}
	}
}


vector4& Mesh::getColor(int i)
{
	return m_array[Buffer::COLOR].ref4(i);
}
vector4 const& Mesh::getColor(int i) const
{
	return m_array[Buffer::COLOR].ref4(i);
}

vector3 Mesh::calcMeshCenter() const
{
	vector3 center(0,0,0);

	for(int i=0; i<numVertex(); i++)
		center+=getVertex(i);

	center/=(double)numVertex();
	return center;
}

void OBJloader::Mesh::resizeIndexBuffer(int numFace)
{
	if(numFace)
		m_arrayFace.resize(numFace);
	else
		m_arrayFace.clear();
}

void OBJloader::Mesh::resize(int numVertex, int numFace)
{
	resize(numVertex,0,0,0,numFace);
}
void OBJloader::Mesh::resize(int numVertex, int numNormal, int numTexCoord, int numColor, int numFace)
{
	m_array[Buffer::VERTEX].resize(numVertex,3);
	m_array[Buffer::NORMAL].resize(numNormal,3);
	m_array[Buffer::TEXCOORD].resize(numTexCoord,2);
	m_array[Buffer::COLOR].resize(numColor,4);
	resizeIndexBuffer(numFace);
}
OBJloader::Mesh::MergeInfo::MergeInfo()
{
	backupMesh=new OBJloader::Mesh();
}
OBJloader::Mesh::MergeInfo::~MergeInfo()
{
	delete backupMesh;
}
OBJloader::Mesh::MergeInfo* OBJloader::Mesh::mergeDuplicateVertices(bool bReturnMergeInfo, double distThr)
{
	OBJloader::Mesh& mesh=*this;

	OBJloader::EdgeConnectivity ec(mesh);
	bitvectorn isBoundaryVertex;
	isBoundaryVertex.resize(mesh.numVertex());
	isBoundaryVertex.clearAll();

	for(int i=0; i<ec.numEdges(); i++)
	{
		if(ec.getEdge(i).numFaceEdge<2)
		{
			isBoundaryVertex.setAt(ec.source(i));
			isBoundaryVertex.setAt(ec.target(i));
		}
	}

	intvectorn boundaryVertexIndex;
	boundaryVertexIndex.findIndex(isBoundaryVertex,true);

	size_t vertex_count=numVertex();

	double MaxDist=0;
	double dist;
	std::vector<std::pair<int,int> > vertexPairs;
	// for each pair of boundary vertices, test proximity.
	for(int ii=0; ii<boundaryVertexIndex.size(); ii++)
	{
		int i=boundaryVertexIndex[ii];

		for(int jj=ii+1; jj<boundaryVertexIndex.size(); jj++)
		{
			int j=boundaryVertexIndex[jj];
			dist=mesh.getVertex(i).distance(mesh.getVertex(j));
			if(dist>MaxDist) MaxDist=dist;
			if(dist<=distThr)
			{
				vertexPairs.push_back(std::pair<int,int>(i,j));
				printf("merging vertices v%d and v%d\n", i, j);
			}
		}
	}

	printf("max Dist=%f\n", MaxDist);
	return mergeVertices(vertexPairs, bReturnMergeInfo);
}
OBJloader::Mesh::MergeInfo* OBJloader::Mesh::mergeVertices(std::vector<std::pair<int, int> > const& vertexPairs, bool bReturnMergeInfo)
{
	OBJloader::Mesh& mesh=*this;
	MergeInfo* uifo=new MergeInfo();
	*(uifo->backupMesh)=mesh;

	intvectorn vertexMerge;
	uifo->newVertexIndex;
	size_t vertex_count=uifo->backupMesh->numVertex();

	vertexMerge.colon(0, vertex_count);
	for (int i=0; i<vertexPairs.size(); i++)
	{
		std::pair<int,int> p=vertexPairs[i];
		vertexMerge[p.second]=vertexMerge[p.first];
	}
	uifo->newVertexIndex.setSize(vertex_count);
	int c=0;
	for(int i=0; i<vertex_count; i++)
	{
		if(vertexMerge[i]==i)
		{
			uifo->newVertexIndex[i]=c;
			c++;
		}
		else
		{
			uifo->newVertexIndex[i]=uifo->newVertexIndex[vertexMerge[i]];
		}
	}

	bool hasNormal=mesh.numNormal() ;

	vector3N normals;
	normals.resize(vertex_count);
	if (hasNormal){
		normals.setAllValue(vector3(0,0,0));
		// all normals will be merged using the same index as vertices.
		for(int i=0; i<numFace(); i++)
		{
			Face const& f=getFace(i);
			normals[f.vertexIndex(0)]+=mesh.getNormal(f.normalIndex(0));
			normals[f.vertexIndex(1)]+=mesh.getNormal(f.normalIndex(1));
			normals[f.vertexIndex(2)]+=mesh.getNormal(f.normalIndex(2));
		}
	}

	mesh.resizeBuffer(OBJloader::Buffer::VERTEX, c);

	if(hasNormal) mesh.resizeBuffer(OBJloader::Buffer::NORMAL, c);

	intvectorn count(c);
	count.setAllValue(0);
	for(int i=0; i<vertex_count; i++)
	{
		mesh.getVertex(uifo->newVertexIndex[i])=uifo->backupMesh->getVertex(i);
		if(hasNormal)
			mesh.getNormal(uifo->newVertexIndex[i])=normals[i];

		// 당연히 texcoord는 merge하지 않는다.
	}

	if(hasNormal )
	{
		for(int i=0; i<c; i++)
			mesh.getNormal(i).normalize();
	}


	int numTris=uifo->backupMesh->numFace();
	mesh.resizeIndexBuffer(numTris);
	for(int i=0; i<numTris; i++)
	{
		mesh.getFace(i).vertexIndex(0)=uifo->newVertexIndex[uifo->backupMesh->getFace(i).vertexIndex(0)];
		mesh.getFace(i).vertexIndex(1)=uifo->newVertexIndex[uifo->backupMesh->getFace(i).vertexIndex(1)];
		mesh.getFace(i).vertexIndex(2)=uifo->newVertexIndex[uifo->backupMesh->getFace(i).vertexIndex(2)];
	}

	mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::NORMAL);

	if(bReturnMergeInfo)
		return uifo;
	delete uifo;
	return NULL;
}

Face::Face()
{
	ASSERT(Buffer::NUM_BUFFER==4);
	indexes[0]=_tvector<int,4>(-1,-1,-1,-1);
	indexes[1]=_tvector<int,4>(-1,-1,-1,-1);
	indexes[2]=_tvector<int,4>(-1,-1,-1,-1);
}

int Buffer::defaultNumElts(Type t)
{
	switch(t)
	{
	case VERTEX:
		return 3;
	case NORMAL:
		return 3;
	case TEXCOORD:
		return 2;
	case COLOR:
		return 4;
	default:
		return 0;
	}
	return 0;
}
int Buffer::size() const { return rows();}
void Buffer::resize(int n, int m) { matrixn::resize(n,m);}
vector3& Buffer::ref3(int n)
{
	return *((vector3*)(&(*this)[n][0]));
}
vector3 const& Buffer::ref3(int n)const
{
	return *((vector3*)(&(*this)[n][0]));
}

vector2& Buffer::ref2(int n)
{
	return *((vector2*)(&(*this)[n][0]));
}
vector2 const& Buffer::ref2(int n) const
{
	return *((vector2*)(&(*this)[n][0]));
}

vector4& Buffer::ref4(int n)
{
	return *((vector4*)(&(*this)[n][0]));
}
vector4 const& Buffer::ref4(int n)const
{
	return *((vector4*)(&(*this)[n][0]));
}



void MeshConnectivity::setAdj(FaceEdge const& fe1, FaceEdge const& fe2)
{
	adjFaceIndex(fe1.faceIndex, fe1.vertexIndex)=fe2.faceIndex;
	adjFaceVertexIndex(fe1.faceIndex, fe1.vertexIndex)=fe2.vertexIndex;

	adjFaceIndex(fe2.faceIndex, fe2.vertexIndex)=fe1.faceIndex;
	adjFaceVertexIndex(fe2.faceIndex, fe2.vertexIndex)=fe1.vertexIndex;
}

FaceEdge MeshConnectivity::prev(FaceEdge const& fe) const
{
	FaceEdge out(adjFace(fe));
	if(!out.isNull())
		out.vertexIndex=(out.vertexIndex+1)%3;

	if(!(out.isNull() || out.source(mMesh)==fe.source(mMesh)))
	{
		puts("MeshConnectivity::prev");
		ASSERT(false);
	}
	return out;
}

FaceEdge MeshConnectivity::next(FaceEdge const& fe) const
{
	FaceEdge out(adjFace(FaceEdge(fe.faceIndex, (fe.vertexIndex+2)%3)));
	if(!(out.isNull() || out.source(mMesh)==fe.source(mMesh)))
	{
		puts("error! MeshConnectivity::next");
		throw std::runtime_error("MeshConnectivity::next");
	}
	return out;
}

MeshConnectivity::MeshConnectivity(Mesh const& mesh):mMesh(mesh){ EdgeConnectivity edges(mesh); _init(mesh, edges);}
MeshConnectivity::MeshConnectivity(Mesh const& mesh, EdgeConnectivity const & edges):mMesh(mesh){	_init(mesh, edges);}


bool MeshConnectivity::isValid() const
{
	return _isValid;
}
void MeshConnectivity::_init(Mesh const& mesh, EdgeConnectivity const& edges)
{
	_isValid=true;

	vertexToFaceEdge.resize(mesh.numVertex());

	adjFaceIndex.setSize(mesh.numFace(), 3);
	adjFaceIndex.setAllValue(-1);
	adjFaceVertexIndex.setSize(mesh.numFace(), 3);
	adjFaceVertexIndex.setAllValue(-1);

	for(int i=0; i<mesh.numFace(); i++)
	{
		vertexToFaceEdge[mesh.getFace(i).vi(0)]=FaceEdge(i,0);
		vertexToFaceEdge[mesh.getFace(i).vi(1)]=FaceEdge(i,1);
		vertexToFaceEdge[mesh.getFace(i).vi(2)]=FaceEdge(i,2);
	}

	mesh.isBoundaryVertex.resize(mesh.numVertex());
	mesh.isBoundaryVertex.clearAll();
	for(int i=0; i<edges.numEdges(); i++)
	{
		EdgeConnectivity::edge& edge=edges.getEdge(i);

		if(edge.numFaceEdge==2)
		{
			setAdj(edge.faceEdge[0],edge.faceEdge[1]);
		}
		else
		{
			mesh.isBoundaryVertex.setAt(edges.source(i));
			mesh.isBoundaryVertex.setAt(edges.target(i));
		}
	}

	if(mesh.isBoundaryVertex.size())
	{
		for(int i=0; i<mesh.numVertex(); i++)
		{
			if(mesh.isBoundaryVertex(i))
			{
				FaceEdge e=vertexToFaceEdge[i];

				int count=0;
				while(!prev(e).isNull())
				{
					e=prev(e);
					count++;
					if (count>100)
					{
						printf("BoundaryVertex??????\n");
						mesh.isBoundaryVertex.resize(0);
						_isValid=false;
						return;
					}
				}
				vertexToFaceEdge[i]=e;
			}
		}
	}
}

EdgeConnectivity::EdgeConnectivity(Mesh const& mesh)
{
	// construct mesh graph
	mMeshGraph.clear();

	for(int i=0; i<mesh.numVertex(); i++)
	{
		nodeT v=mMeshGraph.newNode();
		v->vertexIndex=i;
	}

	int error=0;
	for(int i=0; i<mesh.numFace(); i++)
	{
		for(int fe=0; fe<3; fe++)
		{
			int v1=mesh.getFace(i).vi(fe);
			int v2=mesh.getFace(i).vi((fe+1)%3);

			ASSERT(v1>=0 && v1<mesh.numVertex());
			ASSERT(v2>=0 && v2<mesh.numVertex());

			nodeT s=mMeshGraph.findNode(v1);
			nodeT t=mMeshGraph.findNode(v2);
			edgeT e=mMeshGraph.findEdge(s, t);
			if(!e)
			{
				if(v1==v2) 
					throw std::runtime_error("strange triangle 1");
				e=mMeshGraph.newEdge(s,t);
			}

			int nfe=e.data().numFaceEdge;

			if(nfe<2)
			{
				e.data().faceEdge[nfe].faceIndex=i;
				e.data().faceEdge[nfe].vertexIndex=fe;
				e.data().numFaceEdge++;
			}
			else
			{
				Msg::print("Edge(%d,%d): the number of face edges is larger than 2\n", v1, v2);
				error++;
			}

		}
	}
	if(error)
	{
		//Msg::msgBox("Warning! total %d edges have more than 2 face edges",error);
		Msg::print("Error! total %d edges have more than 2 face edges\n",error);
		throw std::exception();
	}
}

int EdgeConnectivity::numEdges() const	{return mMeshGraph.numEdges();}
EdgeConnectivity::edge& EdgeConnectivity::getEdge(int i) const { return mMeshGraph.findEdge(i).data();}

void EdgeConnectivity::selectBoundaryEdgesFromVertices(boolN const& selectedVertices, boolN& selectedEdges)
{
	selectedEdges.resize(numEdges());
	selectedEdges.setAllValue(false);
	for(int i=0; i<selectedVertices.size(); i++)
	{
		if(selectedVertices(i))
		{
			int vi=i;
			for(int j=0, nj=numAdjEdges(vi); j<nj; j++)
			{
				EdgeConnectivity::edgeT e=getAdjEdgeT(vi,j);
				int s=e.v1().index();
				int t=e.v2().index();

				if(selectedVertices(s)!=selectedVertices(t))
					selectedEdges.set(e.index(), true);
			}
		}
	}
}

int FaceEdge::source(Mesh const& mesh) const
{
	return mesh.getFace(faceIndex).vi(vertexIndex);
}
int FaceEdge::target(Mesh const& mesh) const
{
	return mesh.getFace(faceIndex).vi((vertexIndex+1)%3);
}
int FaceEdge::cross(Mesh const& mesh) const
{
	return mesh.getFace(faceIndex).vi((vertexIndex+2)%3);
}


vector3& FaceEdge::getSource(Mesh & mesh)
{
	return mesh.getVertex(source(mesh));}
vector3& FaceEdge::getTarget(Mesh & mesh)
{
	return mesh.getVertex(target(mesh));}
bool FaceEdge::isNull() const
{
	return faceIndex==-1;}
bool FaceEdge::operator==(const FaceEdge& other)
{
	return faceIndex==other.faceIndex && vertexIndex==other.vertexIndex; }
bool FaceEdge::operator!=(const FaceEdge& other)
{
	return !((*this)==other); }


int EdgeConnectivity::source(int iEdge) const
{
	return mMeshGraph.findEdge(iEdge).v1().index();
}

int EdgeConnectivity::target(int iEdge) const
{
	return mMeshGraph.findEdge(iEdge).v2().index();
}

int EdgeConnectivity::numAdjEdges(int vertexIndex) const
{
	return mMeshGraph.findNode(vertexIndex).degree();
}

int EdgeConnectivity::getAdjVertex(int vertexIndex, int i) const
{
	nodeT source=mMeshGraph.findNode(vertexIndex);
	return source.edge(i).target(source).index();
}

EdgeConnectivity::edge& EdgeConnectivity::getAdjEdge(int vertexIndex, int i) const
{
	TUGL::edge_struct* e=mMeshGraph.findNode(vertexIndex).getEdgePtr(i);
	return *((EdgeConnectivity::edge*)(e->_data));
}

bool EdgeConnectivity::isConnected(int vertex1, int vertex2)
{
	return mMeshGraph.findEdge(mMeshGraph.findNode(vertex1), mMeshGraph.findNode(vertex2))!=NULL;
}


SkinnedMeshFromVertexInfo::SkinnedMeshFromVertexInfo(SkinnedMeshFromVertexInfo const& other)
{
	int nv=other.vertices.size();
	vertices.resize(nv);
	for(int vi=0; vi<vertices.size(); vi++)
	{
		vertices[vi].treeIndices=other.vertices[vi].treeIndices;
		vertices[vi].localpos=other.vertices[vi].localpos;
		vertices[vi].weights=other.vertices[vi].weights;
	}
}
SkinnedMeshFromVertexInfo::SkinnedMeshFromVertexInfo( const char* filename)
{
	BinaryFile file;
	Msg::verify(file.openRead(filename), filename);
	int version=file.unpackInt(); // version
	int nv=file.unpackInt();
	vertices.resize(nv);
	for(int vi=0; vi<vertices.size(); vi++)
	{
		file.unpack(vertices[vi].treeIndices);
		file.unpack(vertices[vi].localpos);
		file.unpack(vertices[vi].weights);
	}
	file.close();
}
void SkinnedMeshFromVertexInfo::exportSkinInfo(const char* filename) const
{
	BinaryFile file;
	Msg::verify(file.openWrite(filename), filename);
	file.packInt(0); // version
	file.packInt(vertices.size());
	for(int vi=0; vi<vertices.size(); vi++)
	{
		file.pack(vertices[vi].treeIndices);
		file.pack(vertices[vi].localpos);
		file.pack(vertices[vi].weights);
	}
	file.close();

}
void SkinnedMeshFromVertexInfo::getVertexInfo(int v1, int v2, int v3, vector3 const& baryCoeffs, intvectorn& treeIndices, vector3N& lpos, vectorn &weights)
{
	VertexInfo& info1=vertices[v1];
	VertexInfo& info2=vertices[v2];
	VertexInfo& info3=vertices[v3];

	// now merge these three infos into one.
	//
	// basic idea
	// (M*lpos+b)*w1+(M*lpos2+b)*w2
	// = M*(lpos*w1+lpos2*w2)+b*(w1+w2)
	// = (M*(lpos*w1+lpos2*w)/(w1+w2) + b)*(w1+w2)
	treeIndices=info1.treeIndices;
	lpos=info1.localpos;
	weights=info1.weights*baryCoeffs.x;

	for (int i=0;i< treeIndices.size(); i++)
		lpos(i)*=weights(i); // lpos*w1

	for(int ii=0; ii<2; ii++)
	{
		VertexInfo* oinfo;
		double b;
		if (ii==0)
		{
			oinfo=&vertices[v2];
			b=baryCoeffs.y;
		}
		else
		{
			oinfo=&vertices[v3];
			b=baryCoeffs.z;
		}
		intvectorn& tio=treeIndices;
		intvectorn& ti=oinfo->treeIndices;
		for (int i=0;i< ti.size(); i++)
		{
			int fi=tio.findFirstIndex(ti(i));
			if (fi==-1 ){
				treeIndices.pushBack(ti(i));
				double w=oinfo->weights(i)*b;
				lpos.pushBack(oinfo->localpos(i)*w);
				weights.pushBack(w);
			}
			else{
				double w=oinfo->weights(i)*b;
				lpos(fi)+=oinfo->localpos(i)*w;
				weights(fi)+=w;
			}
		}
	}
	intvectorn& ti=treeIndices;
	for (int i=0;i< ti.size(); i++)
		lpos(i)*=1.0/weights(i);//  (lpos*w1 +lpos2*w2)/(w1+w2)
}
void SkinnedMeshFromVertexInfo::_calcVertexPosition( MotionLoader const& loader, int vertexIndex, vector3& vpos)
{
	vpos.zero();
	auto& vi=vertices[vertexIndex];
	for (int j=0; j < vi.weights.size(); j++)
		vpos+=(loader.bone(vi.treeIndices(j)).getFrame()*vi.localpos(j))*vi.weights(j);
}
void SkinnedMeshFromVertexInfo::_calcVertexPosition( BoneForwardKinematics const& fksolver, int vertexIndex, vector3& vpos)
{
	vpos.zero();
	auto& vi=vertices[vertexIndex];
	for (int j=0; j < vi.weights.size(); j++)
		vpos+=(fksolver.global(vi.treeIndices(j))*vi.localpos(j))*vi.weights(j);
}
vector3 SkinnedMeshFromVertexInfo::calcSurfacePointPosition( MotionLoader const& loader, intvectorn const& treeIndices, vectorn const& weights, vector3N const& localpos)
{
	vector3 vpos(0,0,0);
	for (int j=0; j < weights.size(); j++)
		vpos+=(loader.bone(treeIndices(j)).getFrame()*localpos(j))*weights(j);
	return vpos;
}
void SkinnedMeshFromVertexInfo::calcVertexPositions(MotionLoader const& loader, OBJloader::Mesh& mesh) const
{
	Msg::verify(mesh.numVertex()==vertices.size(), "#vertex does not match");
	for (int i=0; i<mesh.numVertex(); i++)
		((SkinnedMeshFromVertexInfo&)*this)._calcVertexPosition(loader, i, mesh.getVertex(i));

}
void SkinnedMeshFromVertexInfo::calcVertexNormals(MotionLoader const& loader,quaterN const& bindpose_global, vector3N const& localNormal, OBJloader::Mesh& mesh) const
{
	Msg::verify(mesh.numVertex()==vertices.size(), "#vertex does not match");
	Msg::verify(mesh.numNormal()==mesh.numVertex(), "seperate normal indices are not supported!");
	quaterN delta(bindpose_global.size());
	for(int i=1; i<loader.numBone(); i++)
	{
		quater q;
		q.inverse(bindpose_global(i));
		delta(i)=loader.bone(i).getFrame().rotation*q;
	}

	for (int vertexIndex=0; vertexIndex<mesh.numVertex(); vertexIndex++)
	{
		auto& vi=vertices[vertexIndex];
		auto& normal=mesh.getNormal(vertexIndex);
		normal.zero();
		for (int j=0; j < vi.weights.size(); j++)
			normal+=(delta(vi.treeIndices(j))* (localNormal(vertexIndex))*vi.weights(j));
		normal.normalize();
	}

}
void SkinnedMeshFromVertexInfo::calcLocalVertexPositions(MotionLoader const& loader, OBJloader::Mesh const& mesh)
{
	Msg::verify(mesh.numVertex()==vertices.size(), "#vertex does not match");
	for (int i=0; i<mesh.numVertex(); i++)
	{
		auto& indices=vertices[i].treeIndices;
		auto& weights=vertices[i].weights;
		auto& localpos=vertices[i].localpos;
		localpos.resize(weights.size());
		for (int j=0; j < weights.size(); j++)
		{
			localpos(j)=loader.bone(indices(j)).getFrame().toLocalPos(mesh.getVertex(i));
		}
	}
}
