#pragma once
namespace OBJloader
{
	class Mesh;
}
class SkinnedMeshFromVertexInfo ;
class FBXimporter
{
	void* _data;
	public:
		FBXimporter(const char* filename);
		virtual ~FBXimporter();

		int getMeshCount();
		int getMaterialCount(int imesh);
		vector3 getDiffuseColor(int imesh, int imat);
		vector3 getSpecularColor(int imesh, int imat);
		std::string getDiffuseTexture(int imesh, int imat);
		std::string getAllTextureNames();

		std::string getMeshName(int imesh);
		std::string getMesh(int imesh, OBJloader::Mesh & mesh);
		matrix4 getMeshCurrPose(int imesh);
		matrix4 getBindPose(int ilimb);
		bool hasBindPose(int ilimb);

		// see FBXloader.lua for understanding how to use this function.
		void getSkinnedMesh(int imesh, SkinnedMeshFromVertexInfo & skinned_mesh);

		intvectornView parentIndices();
		void getDefaultPose(vector3N& jointpos, quaterN& jointori);
		void getRestPose(matrixn& pose);
		void getRestPose(vector3N& jointpos, quaterN& jointori);
		void getRestPoseScale(vectorn& scale); // only for checking purposes
		void getBoneNames(TStrings& out);
		void getLocalPose(matrixn& localpose);
		// ilimb==ibone-1 (unless you changed the root bone)
		void getAnim(int ilimb, vectorn& keytime, matrixn& traj);

		void saveTextures();

};

class MeshMerger
{
	std::vector<const OBJloader::Mesh*> _inputs;
	std::vector<intvectorn> _vertsUniqueMap;
	intmatrixn _vertsUnique; // (imesh, ivert)
	std::vector< std::vector<int> > invMap;
	public:
	MeshMerger(int ninputMesh);
	virtual ~MeshMerger();

	void setInputMesh(int i, const OBJloader::Mesh & inputmesh){ _inputs[i]=&inputmesh;}
	void mergeMeshes(OBJloader::Mesh& outputmesh);

	// after mergeMeshes, these functions are available
	int get_imesh(int mergedvertex) { return _vertsUnique[mergedvertex][0];}
	int get_ivert(int mergedvertex) { return _vertsUnique[mergedvertex][1];}
	int get_mergedvertex(int imesh, int ivertex) { return invMap[imesh][ivertex];}
};
