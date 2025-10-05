#ifndef _OGREFLTK_MESH_H_
#define _OGREFLTK_MESH_H_
#pragma once


#include "../../BaseLib/motion/Mesh.h"
#include "../../BaseLib/motion/Geometry.h"
#ifndef NO_OGRE
#include "Line3D.h"
#endif

namespace Ogre
{
	namespace v1
	{
	class Mesh;
	class Entity;
	}
	class Mesh;
	class Item;
}

namespace OBJloader
{


	// Mesh자료구조를 Ogre::Entity자료구조로 변환한다. 메시가 바뀌면, updatePositions()를 호출하면 Entity도 변경된다.
	class MeshToEntity_DATA;
	class MeshToEntity
	{
	public:
		struct Option
		{
			Option();
			Option( bool useColor, bool useNormal, bool useTexCoord, bool buildEdgeList, bool dynamicUpdate);
			bool useColor;
			bool useNormal;
			bool useTexCoord;
			bool buildEdgeList;
			bool dynamicUpdate;
			TString meshId;
		};
#ifndef NO_OGRE
		void _constructSubMesh(int ielt, int startFace, int endFace, const Mesh& mesh, const char* ogreMeshName, const MeshToEntity::Option & option, Ogre::Aabb & box);
#endif
	public:
		std::vector<MeshToEntity_DATA*> mData; // one for each sub-mesh (or element)
		Ogre::Mesh* mMesh;
		MeshToEntity(const Mesh& mesh, const char* ogreMeshName, MeshToEntity::Option option=MeshToEntity::Option ());
		MeshToEntity(const Mesh& mesh, const char* ogreMeshName, bool buildEdgeList, bool dynamicUpdate, bool useNormal=true, bool useTexCoord=true);
		MeshToEntity(const Mesh& mesh, std::string const& ogreMeshName, bool buildEdgeList=false, bool dynamicUpdate=false, bool useNormal=true, bool useTexCoord=true, bool useColor=false);
		MeshToEntity(const Mesh& mesh, const char* ogreMeshName, bool buildEdgeList, bool dynamicUpdate, bool useNormal, bool useTexCoord, bool useColor);
		~MeshToEntity();
		void updatePositions();
		void updatePositions(const vector3N& vertices);
		void updatePositionsAndNormals();
		void setBuildEdgeList(bool value);

		// created entities will not be removed automatically when destructing a MeshToEntity object.
		Ogre::Item* createEntity(const char* entityName, const char* materialName="white");
		Ogre::Item* getLastCreatedEntity() const	{ return mEntity;}

		bool isDynamic() { return mSavedOption.dynamicUpdate;}
		void removeAllResources(); // entity and mesh will be removed. This is not automatically called in dtor.

	private:
		const OBJloader::Mesh& mInputMesh;
		Ogre::Item* mEntity;
		MeshToEntity::Option mSavedOption;
#ifndef NO_OGRE
		void _updateVertices(MeshToEntity_DATA* data, Ogre::Aabb & box, unsigned char* vertex, const OBJloader::MeshToEntity::Option& option) const;
#endif
	};


	Ogre::Item* createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName="white");
	Ogre::Item* createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName, MeshToEntity::Option & option);

#ifndef NO_OGRE
	class MeshEntity : public Mesh, public SimplerRenderable
	{
	public:
		MeshEntity ():Mesh(), SimplerRenderable(){}
		virtual void firstInit()=0;		// initialize structure.
		virtual void update()=0;		// when only vertex positions are changed, you can call update();
		void setMaterial(const char* name);
	};


	// for rendering of Mesh. Each vertex of a face should have valid positions, normals, and texture coordinates.
	class TriangleMesh : public MeshEntity
	{
	public:
		TriangleMesh ();
		~TriangleMesh ();

		void firstInit();	// construct mesh graph
		void update();		// when only vertex position is changed you can call update();
	};

	// for drawing of Mesh using lines.
	class MeshLineStrip: public MeshEntity
	{
	public:
		MeshLineStrip();
		~MeshLineStrip();

		EdgeConnectivity* edges;
		void _createGraph();
		void firstInit();	// construct mesh graph
		void update();		// when vertex position is changed you can call update();
	};

	// for drawing of Mesh using lines.
	class MeshLineStripReduced: public MeshEntity
	{
	public:
		MeshLineStripReduced():MeshEntity(){}
		void firstInit(){ update();}
		void update();		// when vertex position is changed you can call update();
	};
#endif

}

#endif
