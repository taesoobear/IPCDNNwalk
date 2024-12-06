#pragma once
namespace OBJloader
{
	class Mesh;
}
class CImage;
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
		TStrings getMaterialPropertyTypes(int imesh, int imat);
		vectorn getMaterialProperty(int imesh, int imat, const char* property_type);
		std::string getAllTextureNames();

		std::string getMeshName(int imesh);
		std::string getMesh(int imesh, OBJloader::Mesh & mesh);
		matrix4 getMeshCurrPose(int imesh);
		TStrings getModelNames();
		matrix4 getModelCurrPose(const char* model_name);
		int countBindPoses(int imesh); // update the BindPose of all limbnodes  using imesh's bindpose.
		matrix4 getBindPose(int ilimbnode);
		bool hasBindPose(int ilimbnode);
		void clearBindPose(); // after this, hasBindPose(ilimbnode) becomes false for all bones. To re-load bindposes, use countBindPoses(imesh)

		// see FBXloader.lua for understanding how to use this function.
		void getSkinnedMesh(int imesh, SkinnedMeshFromVertexInfo & skinned_mesh);

		intvectornView parentIndices();
		void getDefaultPose(vector3N& jointpos, quaterN& jointori);
		void getRestPose(matrixn& pose);
		void getRestPose(vector3N& jointpos, quaterN& jointori);
		void getRestPoseScale(vectorn& scale); // only for checking purposes
		void getBoneNames(TStrings& out);
		void getLocalPose(matrixn& localpose);
		// ilimbnode==ibone-1 (unless you changed the root bone)
		void getAnim(int ilimbnode, vectorn& keytime, matrixn& traj);
		int getAnimCount();
		std::string getAnimName(int ianim);
		vectorn getAnimInfo(int ianim);
		void getAnim2(int ianim, int g_nFrames, double g_T, int ilimbnode, vectorn& keytime, matrixn& traj);

		/// save textures to files
		void saveTextures();

		/// save textures to self.texture_names and self.textures
		void saveTexturesToMemoryBuffer();
		
		TStrings texture_names;
		std::vector<CImage*> textures;
		int numTexture() { return texture_names.size();}
		CImage& getTexture(int itexture) {return *textures[itexture];}
		TString getTextureFileName(int itexture) { return texture_names[itexture];}
		void packTextures(BinaryFile& bf);
		void unpackTextures(BinaryFile& bf);
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
