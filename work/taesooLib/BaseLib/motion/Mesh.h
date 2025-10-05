#ifndef _Baselib_MESH_H_
#define _Baselib_MESH_H_

#include "../math/tvector.h"
#include "../math/intervals.h"
#include "../utility/TGL.h"
#include "../utility/TUGL.h"

class BinaryFile;
class CTextFile;
namespace OBJloader
{
	// no virtual function
	class Buffer: protected matrixn
	{
	public:
		Buffer():matrixn(){}
		Buffer(const Buffer& b):matrixn(){assign(b);}
		void operator=(const Buffer& b) {assign(b);}
		enum Type {VERTEX, NORMAL, TEXCOORD, COLOR, NUM_BUFFER};
		static int defaultNumElts(Type t);
		vector2& ref2(int n);
		vector2 const& ref2(int n)const;
		vector3& ref3(int n);
		vector3 const& ref3(int n)const;
		vector4& ref4(int n);
		vector4 const& ref4(int n)const;
		int size() const;
		void resize(int n, int m);
	};
	class Face
	{
	public:
		Face();
		_tvector<int,Buffer::NUM_BUFFER> indexes[3];

		int& vertexIndex(int i);
		int const& vertexIndex(int i) const;
		int vi(int i) const	{ return vertexIndex(i);}

		int& normalIndex(int i) ;
		int const& normalIndex(int i) const;

		int& texCoordIndex(int i);
		int const& texCoordIndex(int i) const;

		int& colorIndex(int i);
		int const& colorIndex(int i) const;

		void setIndex(int i, int j, int k, Buffer::Type t=Buffer::VERTEX);	// vertexIndex
		void setIndex(index3 const& idx, Buffer::Type t);
		index3 getIndex(Buffer::Type t) const;
		inline void setIndex1(int i, int vi, Buffer::Type t=Buffer::VERTEX) { indexes[i](t)=vi;}

		bool isVertex(int vi) {
			for(int i=0;i<3;++i) {
				if(indexes[i](Buffer::VERTEX) == vi) {
					return true;
				}
			}
			return false;
		}
	};
	class Mesh
	{
	protected:
		Buffer m_array[Buffer::NUM_BUFFER];
		std::vector<Face>   m_arrayFace;
	public:
		void resize(int numVertex, int numFace);
		void resize(int numVertex, int numNormal, int numTexCoord, int numColor, int numFace);
		inline void addFace(OBJloader::Face const& f) { m_arrayFace.push_back(f);}
		void init(const vector3N& vertices, const intvectorn& triangles);
		void init(const vector3N& vertices, const vector3N& normals, const intvectorn& triangles);

		Buffer const& buffer(Buffer::Type type) const	{ return m_array[type];}
		void resizeBuffer(Buffer::Type type, int n);
		void resizeIndexBuffer(int numFace);
		inline void resizeVertexBuffer(int n) { resizeBuffer(Buffer::VERTEX, n);}
		inline void resizeNormalBuffer(int n) { resizeBuffer(Buffer::NORMAL, n);}
		inline void resizeUVbuffer(int n) { resizeBuffer(Buffer::TEXCOORD, n);}

		void copyIndex(Buffer::Type from, Buffer::Type to);

		mutable boolN isBoundaryVertex;	// become valid after MeshConnectivity object is created.
		intIntervals faceGroups;		// becomes valid after classifyTriangles method is called.

		Mesh(void){}
		Mesh(const Mesh& otherMesh){copyFrom(otherMesh);}
		virtual ~Mesh(void){}

		int numFace() const {return m_arrayFace.size();}
		Face const& getFace(int i) const	{return m_arrayFace[i];}
		Face & getFace(int i)				{return m_arrayFace[i];}

		int numVertex() const;
		vector3& getVertex(int i);
		vector3 const& getVertex(int i) const;

		int numNormal() const;
		vector3& getNormal(int i);
		vector3 const& getNormal(int i) const;

		int numTexCoord() const;
		vector2& getTexCoord(int i);
		vector2 const& getTexCoord(int i) const;

		int numColor() const;
		vector4& getColor(int i);
		vector4 const& getColor(int i) const;

		void vertFlipUV();

		vector3& getFaceCorner(int iFace, int icorner) ;
		vector3 calcFaceCenter(int i) const;
		vector3 calcFaceNormal(int i) const;
		vector3 calcMeshCenter() const;
		// face indices will be changed after removal.
		void removeFaces(intvectorn const& faceIndices);
		void addVertices(vector3N const& vertices);
		void addNormals(vector3N const& normals);
		void getVertices(vector3N & vertices);
		// [ vi0, vi1, vi2, fi0; // for face 0
		//   vi0', vi1', vi2', fi0'; // for face 1
		//    ...
		//   vi0'', vi1'',vi2'',fi0'']
		//  fi0 represents a face index to copy from.
		//  if vertexIndices3 is a 3-column matrix, all faces are copied from face 0.
		void addFaces(intmatrixn const& faces);

		//////////////////////////////////////////////
		// tools
		//////////////////////////////////////////////

		// assumes that m_arrayFace and {m_arrayVertex[i].pos} are all valid.
		void calculateVertexNormal();

		void eliminateSharedVertex(Mesh const& otherMesh);
		struct MergeInfo
		{
			MergeInfo();
			~MergeInfo();
			Mesh* backupMesh;
			intvectorn newVertexIndex;
		};
		MergeInfo* mergeDuplicateVertices(bool bReturnMergeInfo=false, double distThr=0.00001);
		MergeInfo* mergeVertices(std::vector<std::pair<int,int> > const& vertexPairs, bool bReturnMergeInfo=false);
		inline intvectorn _mergeDuplicateVertices(double distThr=0.00001)
		{
			MergeInfo* mi=mergeDuplicateVertices(true, distThr);
			intvectorn newvi=mi->newVertexIndex;
			delete mi->backupMesh;
			delete mi;
			return newvi;
		}

		// ex) a->merge(a, b);
		void merge(Mesh const& a, Mesh const& b);
		virtual void transform(matrix4 const& b);

		void copyFrom(Mesh const& otherMesh);
		void operator=(Mesh const& otherMesh) { copyFrom(otherMesh);}

		// export to a text file (.tri) - make all indexes identical.
		bool saveMesh(const char* filename_);
		bool loadMesh(const char* filename_, bool bInit=true);

		// export to a widely used file format (.obj)
		virtual bool saveObj(const char* filename, bool vn, bool vt);
		virtual bool loadObj(const char* filename);

		virtual void pack(BinaryFile& bf) const;
		virtual void unpack(BinaryFile& bf);
		void _unpackMeshVersion(int version, BinaryFile& bf);
		
		// protected:
		void _saveObj(std::ofstream & fout, bool vn, bool vt);
		void _loadObj(CTextFile& file);
	};

	struct FaceEdge
	{
		FaceEdge():faceIndex(-1), vertexIndex(-1){}
		FaceEdge(int fi, int vi):faceIndex(fi), vertexIndex(vi){}

		// faceIndex번째 face의 vertexIndex와 (vertexIndex+1)%3을 연결하는 에지
		int faceIndex;
		int vertexIndex;	// in [0,1,2].

		// retrieve global vertex index
		int source(Mesh const& mesh) const;
		int target(Mesh const& mesh) const;
		int cross(Mesh const& mesh) const;	// 맞은편 vertex

		vector3& getSource(Mesh & mesh);
		vector3& getTarget(Mesh & mesh);
		bool isNull() const;
		bool operator==(const FaceEdge& other);
		bool operator!=(const FaceEdge& other);
	};

	// build edge array and adjecent faces to the edges.
	class EdgeConnectivity
	{
	public:
		struct vertex
		{
			int vertexIndex;
		};

		struct edge
		{
			FaceEdge faceEdge[2];
			int numFaceEdge;
			edge() { numFaceEdge=0;}
		};

		typedef TUGL::edge<vertex, edge> edgeT;
		typedef TUGL::node<vertex, edge> nodeT;
		TUGL::graph<vertex, edge>  mMeshGraph;

		EdgeConnectivity(Mesh const& mesh);

		int numEdges() const;
		int numAdjEdges(int vertexIndex) const;
		bool isConnected(int vertex1, int vertex2);
		edge& getAdjEdge(int vertexIndex, int i) const;
		// auto e=getAdjEdgeT(...);  
		// Then, you can use e.data(), e.index(), e.v1().index() or e.v2().index()
		inline edgeT getAdjEdgeT(int vertexIndex, int i) const { return mMeshGraph.findNode(vertexIndex).edge(i);}
		int getAdjVertex(int vertexIndex, int i) const;
		edge& getEdge(int i) const;
		inline int getNumFaceEdge(int iEdge) const { return getEdge(iEdge).numFaceEdge;}
		inline FaceEdge const& getFaceEdge(int iEdge, int iFaceEdge) const { return getEdge(iEdge).faceEdge[iFaceEdge];}
		int source(int iEdge) const;
		int target(int iEdge) const;
		void selectBoundaryEdgesFromVertices(boolN const& selectedVertices, boolN& selectedEdges);
	};

	// build face adjacencies. O(V)+O(E)
	class MeshConnectivity
	{
		void _init(Mesh const& mesh, EdgeConnectivity const & edges);
	public:
		MeshConnectivity(Mesh const& mesh);
		MeshConnectivity(Mesh const& mesh, EdgeConnectivity const & edges);

		FaceEdge vertexToFace(int vertex)			{ return vertexToFaceEdge[vertex];}
		inline FaceEdge adjFace(FaceEdge const& fe) const { return FaceEdge(adjFaceIndex(fe.faceIndex, fe.vertexIndex), adjFaceVertexIndex(fe.faceIndex, fe.vertexIndex));}
		// fe.source()에 인접한 다음 FaceEdge를 return한다. 다음FaceEdge의 source()는 fe.source()와 동일하다.
		FaceEdge next(FaceEdge const& fe) const;
		FaceEdge prev(FaceEdge const& fe) const;

		struct IncidentFaceIterator
		{
			IncidentFaceIterator(MeshConnectivity* pp, FaceEdge const& f){self=pp; fe=f; bFirst=true;}
			IncidentFaceIterator(MeshConnectivity* pp, FaceEdge const& f, bool bf){self=pp; fe=f; bFirst=bf;}
			MeshConnectivity* self;
			FaceEdge fe;
			bool bFirst;	// 맨처음에 cyclic vertice의 경우 begin()이 end()와 같으므로, 루프가 바로 종료되는것을 막기위한 변수.
			void operator++( )		{ fe=self->next(fe); bFirst=false; }
			void operator--( )		{ fe=self->prev(fe);}

			// 마지막 원소인지 알아볼때 쓸수있음. (if (i.next()==end())
			IncidentFaceIterator next() { return IncidentFaceIterator (self, self->next(fe), false);}
			IncidentFaceIterator prev() { return IncidentFaceIterator (self, self->prev(fe), false);}

			bool operator==(const IncidentFaceIterator& other) { if(bFirst) return false;  return fe==other.fe; }
			bool operator!=(const IncidentFaceIterator& other) { return !((*this)==other); }
			FaceEdge* operator->()	{ return &fe;}
			FaceEdge& operator*()	{ return fe;}
		};

		/////////////////////////////////////
		//
		// For(MeshConnectivity::IncidentFaceIterator i=mc.beginAdjFace(v); i!=mc.endAdjFace(v); ++i)
		//
		IncidentFaceIterator beginAdjFace(int vertexIndex)		{ return IncidentFaceIterator(this, vertexToFaceEdge[vertexIndex]);}
		IncidentFaceIterator endAdjFace(int vertexIndex)		{ if(mMesh.isBoundaryVertex[vertexIndex])	return IncidentFaceIterator(this, FaceEdge()); return beginAdjFace(vertexIndex);}

		Mesh const& mesh() const	{return mMesh;}
		bool isValid() const ;

		void selectVerticesFromFaces(boolN const& selectedFaces, boolN& selectedVertices);
		void selectFacesFromVertices(boolN const& selectedVertices, boolN& selectedFaces);

	private:
		bool _isValid;
		std::vector<FaceEdge> vertexToFaceEdge;	// vertex에 달려있는 첫번째 faceEdge return
		intmatrixn adjFaceIndex;	// adjFaceIndex(faceIndex, vertexIndex)=faceIndex2
		intmatrixn adjFaceVertexIndex;

		void setAdj(FaceEdge const& fe1, FaceEdge const& fe2);

		Mesh const& mMesh;

	};

	void classifyTriangles(Mesh& mesh);

	// file: 16bit per pixel raw file.
	void createTerrain(Mesh& mesh, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ);
	void convertTerrainToBMP(const char* filename, int sizeX,int sizeY, const char* outBMPfile);
	void convertTerrainFromBMP(const char* BMPfilename, const char* outfile);

	void createPlane(Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ);
	void createPlane(Mesh& mesh, const char* imageFile, int numSeg, m_real size);

	void createBox(Mesh& mesh, m_real sizeX, m_real sizeY, m_real sizeZ);

	void createCylinder(Mesh& mesh, m_real radius, m_real height, int ndiv);
	void createCircle(Mesh& mesh, vector3 const& center, m_real radius, int ndiv);

	class Raw2Bytes : public _tmat<unsigned short>
	{
		public:
			Raw2Bytes():_tmat<unsigned short>(){}
			template <class T>
				Raw2Bytes(const T& other):_tmat<unsigned short>(other){}
			virtual ~Raw2Bytes(){}
	};
	void _createTerrain(Mesh& mesh, Raw2Bytes& image, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ);
}

class MotionLoader;
class BoneForwardKinematics;
class ScaledBoneKinematics;
class SkinnedMeshFromVertexInfo
{
	public:
	struct VertexInfo
	{
		intvectorn treeIndices;
		vector3N localpos;
		vectorn weights;
	};
	std::vector<VertexInfo> vertices;
	int numVertex() const { return vertices.size();}
		SkinnedMeshFromVertexInfo(){}
		SkinnedMeshFromVertexInfo(SkinnedMeshFromVertexInfo const& other);
		SkinnedMeshFromVertexInfo(const char* filename);

		void exportSkinInfo(const char* filename) const;
		vector3 calcSurfacePointPosition( MotionLoader const& loader, intvectorn const& treeIndices, vectorn const& weights, vector3N const& localpos);
		void _calcVertexPosition( MotionLoader const& loader, int vertexIndex, vector3& out);
		void _calcVertexPosition( BoneForwardKinematics const& loader, int vertexIndex, vector3& out);
		inline vector3 calcVertexPosition( MotionLoader const& loader, int vertexIndex) { vector3 out; _calcVertexPosition(loader, vertexIndex, out); return out; }
		inline vector3 calcVertexPosition( BoneForwardKinematics const& fkSolver, int vertexIndex) { vector3 out; _calcVertexPosition(fkSolver, vertexIndex, out); return out; }
		// modifies mesh
		void calcVertexPositions(MotionLoader const& loader, OBJloader::Mesh& mesh) const;
		void calcVertexPositions(BoneForwardKinematics const& fkSolver, OBJloader::Mesh& mesh) const;
		void calcVertexPositions(ScaledBoneKinematics const& fkSolver, OBJloader::Mesh& mesh) const;
		// assumes that both meshes has a normal buffer 
		void calcVertexNormals(MotionLoader const& loader, quaterN const& bindpose_global, vector3N const& local_normal, OBJloader::Mesh& mesh) const;
		void calcVertexNormals(BoneForwardKinematics const& fkSolver, quaterN const& bindpose_global, vector3N const& local_normal, OBJloader::Mesh& mesh) const;
		void calcVertexNormals(ScaledBoneKinematics const& fkSolver, quaterN const& bindpose_global, vector3N const& local_normal, OBJloader::Mesh& mesh) const;

		// modifies self.
		void calcLocalVertexPositions(MotionLoader const& loader, OBJloader::Mesh const& mesh);

		void resize(int numVertex) { vertices.resize(numVertex);}
		inline intvectorn& treeIndices(int vertexIndex){ return vertices[vertexIndex].treeIndices;}
		inline vector3N& localPos(int vertexIndex){ return vertices[vertexIndex].localpos;}
		inline vectorn& weights(int vertexIndex){ return vertices[vertexIndex].weights;}
		void getVertexInfo(int v1, int v2, int v3, vector3 const& baryCoeffs, intvectorn& treeIndices, vector3N& localpos, vectorn &weights);
};
#endif
