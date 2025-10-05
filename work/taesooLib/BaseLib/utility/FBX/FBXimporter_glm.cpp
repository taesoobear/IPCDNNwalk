#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <stdio.h>
#include <cstring>
#include <string>
#include <algorithm>
#include <set>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include "ofbx.h"
#include <math.h>

using namespace std;
using namespace glm;

// https://github.com/nem0/OpenFBX/blob/master/demo/main.cpp
// https://github.com/Larry955/FbxParser/tree/master/FbxParser

class _FBXimport{
	public:
ofbx::IScene* scene;
ofbx::u8* content;
		_FBXimport()
		{
			scene=NULL;
			content=NULL;
		}
string FILENAME;
string TEXTURENAME;

vector<const ofbx::Cluster*> clusters;
vector<int> clusters_index;

vector<const ofbx::Object*> limbVec;
map<const ofbx::Object*, int> limbMap;
vector<int> limbParents;

int key_count_max = -1;

class MyVertex
{
public:
	MyVertex()
	{
		p.x = p.y = p.z = t.x = t.y = n.x = n.y = n.z = 0.0f;
	}
	
		inline static bool vec3Eq (const vec3 &a, const vec3 &b)
		{
			return fabs(a.x - b.x) < 1e-6 && fabs(a.y - b.y) < 1e-6 && fabs(a.z - b.z) < 1e-6;
		};
		inline static bool vec2Eq (const vec2 &a, const vec2 &b)
		{
			return fabs(a.x - b.x) < 1e-6 && fabs(a.y - b.y) < 1e-6;
		};
	bool operator==(const MyVertex &v) const
	{
		if(!vec3Eq(p, v.p)) {
			return false;
		}
		if(!vec2Eq(t, v.t)) {
			return false;
		}
		if(!vec3Eq(n, v.n)) {
			return false;
		}
		return true;
	}
	
	vec3 p;
	vec2 t;
	vec3 n;
	vector<float> w;
	vector<int> i;
};

class MyTriangle
{
public:
	MyTriangle()
	{
		v[0] = v[1] = v[2] = 0;
	}
	int v[3];
};

class MyMesh
{
public:
	string name;
	vector<MyVertex> verts;
	vector<MyTriangle> tris;
	
	// We want to store the indices of unique vertices, since there
	// are many duplicated vertices.
	// Let's say that verts contains duplicate vertices A and D:
	//                      0 1 2 3 4 5 6 7
	//    verts          = [A B a C D a d E] <- duplicates in lower case
	//    vertsUnique    = [0 1 3 4 7]
	//    vertsUniqueMap = [0 1 0 2 3 0 3 4] <- indexes into vertsUnique
	vector<int> vertsUnique;
	vector<int> vertsUniqueMap;
	
	int maxInfluences; // maximum number of bone influences
};

vector<MyMesh> myMeshes;

vector<const ofbx::IElement*> find_element(const ofbx::IElement* parent_element, const string& id) {
	// Find a child element of specific id from the given parent element 
	vector<const ofbx::IElement*> elements;
	char string_id[32];
	const ofbx::IElement* child = parent_element->getFirstChild();
	if (child == nullptr) {
		return elements;
	}
	while (child != nullptr) {
		vector<const ofbx::IElement*> find_results = find_element(child, id);
		if (find_results.size() != 0) {
			elements.insert(elements.begin(), find_results.begin(), find_results.end());
		}
		child->getID().toString(string_id);
		if (string(string_id, id.length()) == id) {
			elements.push_back(child);
		}
		child = child->getSibling();
	}
	return elements;
}

const ofbx::IElement* find_child(const ofbx::IElement& element, const char* id)
{	// Helper function for find property of an element
	const ofbx::IElement* iter = element.getFirstChild();
	while (iter != nullptr)
	{
		if (iter->getID() == id) return iter;
		iter = iter->getSibling();
	}
	return nullptr;
}

ofbx::IElement* find_property(const ofbx::IElement& obj, const char* name)
{
	// Find a specific property of a given element
	const ofbx::IElement* props = find_child(obj, "Properties70");
	if (!props) return nullptr;

	ofbx::IElement* prop = props->getFirstChild();
	while (prop)
	{
		if (prop->getFirstProperty() && prop->getFirstProperty()->getValue() == name)
		{
			return prop;
		}
		prop = prop->getSibling();
	}
	return nullptr;
}

glm::vec3 get_vector(const ofbx::IElement* property) {
	// Get the vaule of sepcific property
	const ofbx::IElementProperty* iter = property->getFirstProperty();
	while (iter != nullptr && iter->getType() != ofbx::IElementProperty::Type::DOUBLE) {
		iter = iter->getNext();
	}
	glm::vec3 vec = glm::vec3(0.0f, 0.0f, 0.0f);
	if (iter == nullptr) {
		return vec;
	}
	for (int i = 0; i < 3; i++) {
		vec[i] = iter->getValue().toDouble();
		iter = iter->getNext();
	}
	return vec;
}

glm::mat4 get_T_matrix(const ofbx::IScene* scene, const char* mesh_name) {
	// Get the transform matrix for meshes from original pose to binding pose
	// Usually it is an identity matrix, but for some fbx files it is necessary to apply
	// this transforms to get correct results.

	glm::mat4 transform_matrix = glm::mat4(1.0);
	const ofbx::IElement* root = scene->getRootElement();
	const ofbx::IElement* child = root->getFirstChild();
	vector<const ofbx::IElement*> models = find_element(root, "Model");
	for (auto model : models) {
		ofbx::DataView texture_name = model->getFirstProperty()->getNext()->getValue();

		string name;
		for (long i = 0; i < texture_name.end - texture_name.begin; i++) {
			char c = static_cast<char>(*(texture_name.begin + i));
			if (c == '\0') {
				break;
			}
			name += c;
		}
		//cout << name << endl;

		if (name == mesh_name) {
			//cout << model->getFirstChild()->getID() << endl;
			const ofbx::IElement* translation = find_property(*model, "Lcl Translation");
			const ofbx::IElement* rotation = find_property(*model, "Lcl Rotation");
			const ofbx::IElement* scaling = find_property(*model, "Lcl Scaling");
			glm::vec3 trans_vec = glm::vec3(0.0f, 0.0f, 0.0f);
			glm::vec3 rotation_vec = glm::vec3(0.0f, 0.0f, 0.0f);
			glm::vec3 scaling_vec = glm::vec3(1.0f, 1.0f, 1.0f);

			if (translation) {
				trans_vec = get_vector(translation);
			}
			if (rotation) {
				rotation_vec = get_vector(rotation);
			}
			if (scaling) {
				scaling_vec = get_vector(scaling);
			}

			mat4 I = mat4(1.0);
			float x = rotation_vec.x * M_PI / 180.0f;
			float y = rotation_vec.y * M_PI / 180.0f;
			float z = rotation_vec.z * M_PI / 180.0f;
			mat4 Rx = rotate(I, x, vec3(1.0, 0.0, 0.0));
			mat4 Ry = rotate(I, y, vec3(0.0, 1.0, 0.0));
			mat4 Rz = rotate(I, z, vec3(0.0, 0.0, 1.0));
			mat4 R = I;
			R = Rz * Ry * Rx;
			transform_matrix = glm::translate(transform_matrix, trans_vec) * R;
			transform_matrix = glm::scale(transform_matrix, scaling_vec);
			/*
			cout << "(" << trans_vec.x  << ", " << trans_vec.y << ", " << trans_vec.z << ")" << endl;
			cout << "(" << rotation_vec.x << ", " << rotation_vec.y << ", " << rotation_vec.z << ")" << endl;
			cout << "(" << scaling_vec.x << ", " << scaling_vec.y << ", " << scaling_vec.z << ")" << endl;
			*/

		}
	}
	return transform_matrix;
}

bool saveGeom(const ofbx::IScene *scene)
{
	cout << "=== geometry ===" << endl;
	int mesh_count = scene->getMeshCount();
	
	myMeshes.resize(mesh_count);
	for (int k = 0; k < mesh_count; ++k) {
		const ofbx::Mesh *mesh = scene->getMesh(k);
		const ofbx::Geometry *geom = mesh->getGeometry();
		int vertex_count = geom->getVertexCount();
		int index_count = geom->getIndexCount();
		cout << mesh->name << ": " << vertex_count << " verts, ";
		myMeshes[k].name = mesh->name;
		replace(myMeshes[k].name.begin(), myMeshes[k].name.end(), ':', '_');
		replace(myMeshes[k].name.begin(), myMeshes[k].name.end(), '/', '_');
		
		//Get the transform matrix for this mesh from original pose to binding pose
		glm::mat4 transform_matrix = get_T_matrix(scene, mesh->name);

		myMeshes[k].verts.resize(vertex_count);
		auto &verts = myMeshes[k].verts;
		const ofbx::Vec3* vertices = geom->getVertices();
		for (int i = 0; i < vertex_count; ++i) {
			const ofbx::Vec3 &v = vertices[i];
			glm::vec4 transformed_vertices = transform_matrix * vec4(v.x, v.y, v.z, 1.0f);
			verts[i].p.x = transformed_vertices.x;
			verts[i].p.y = transformed_vertices.y;
			verts[i].p.z = transformed_vertices.z;
			/*
			verts[i].p.x = v.x;
			verts[i].p.y = v.y;
			verts[i].p.z = v.z;
			*/
		}

		bool has_normals = geom->getNormals() != nullptr;
		if(has_normals) {
			const ofbx::Vec3* normals = geom->getNormals();
			// This will fail if ofbx::LoadFlags::TRIANGULATE is not used
			assert(geom->getIndexCount() == vertex_count);
			for(int i = 0; i < vertex_count; ++i) {
				const ofbx::Vec3 &n = normals[i];
				verts[i].n.x = n.x;
				verts[i].n.y = n.y;
				verts[i].n.z = n.z;
			}
		}

		bool has_uvs = geom->getUVs() != nullptr;
		if(has_uvs) {
			const ofbx::Vec2 *uvs = geom->getUVs();
			// This will fail if ofbx::LoadFlags::TRIANGULATE is not used
			assert(geom->getIndexCount() == vertex_count);
			for (int i = 0; i < vertex_count; ++i) {
				const ofbx::Vec2 &uv = uvs[i];
				verts[i].t.x = uv.x;
				verts[i].t.y = uv.y;
			}
		}

		// Assume that we are working with a triangulated array, so
		// there is no index/element array.
		assert(vertex_count == index_count);
		int tri_count = vertex_count/3;
		myMeshes[k].tris.resize(tri_count);
		auto &tris = myMeshes[k].tris;
		for(int i = 0; i < tri_count; ++i) {
			// This is index-0. Convert to index-1 before exporting to OBJ.
			tris[i].v[0] = 3*i + 0;
			tris[i].v[1] = 3*i + 1;
			tris[i].v[2] = 3*i + 2;
		}

		// Find duplicates (slow O(n^2))
		// E.g.,
		//                      0 1 2 3 4 5 6 7
		//    verts          = [A B a C D a d E] <- duplicates in lower case
		//    vertsUnique    = [0 1 3 4 7]
		//    vertsUniqueMap = [0 1 0 2 3 0 3 4] <- indexes into vertsUnique
		auto &vertsUnique = myMeshes[k].vertsUnique;
		auto &vertsUniqueMap = myMeshes[k].vertsUniqueMap;
		vertsUniqueMap.resize(vertex_count, -1);
		for(int i = 0; i < vertex_count; ++i) {
			int duplj = -1;
			for(int j = 0; j < (int)vertsUnique.size(); ++j) {
				if(verts[i] == verts[vertsUnique[j]]) {
					// Found duplicate in vertsUnique
					duplj = j;
					break;
				}
			}
			if(duplj == -1) {
				// No duplicate was found, so create a new entry
				vertsUniqueMap[i] = vertsUnique.size();
				vertsUnique.push_back(i);
			} else {
				// Duplicate found
				vertsUniqueMap[i] = duplj;
			}
		}
		cout << vertsUnique.size() << " unique" << endl;
	}
	
	// Save
	for(auto &mesh : myMeshes) {
		string path = FILENAME + "_" + mesh.name + ".obj";
		FILE* fp = fopen(path.c_str(), "wb");
		if (!fp) {
			cout << "Could not write to " << path << endl;
			return false;
		}
		cout << "Saving to " << path << endl;
		fprintf(fp, "# %s\n", mesh.name.c_str());
		const auto &verts = mesh.verts;
		const auto &tris = mesh.tris;
		const auto &vertsUnique = mesh.vertsUnique;
		const auto &vertsUniqueMap = mesh.vertsUniqueMap;
		fprintf(fp, "# %lu vertices, %lu triangles\n", vertsUnique.size(), tris.size());
		for(int i : vertsUnique) {
			const MyVertex &v = verts[i];
			fprintf(fp, "v %f %f %f\n", v.p.x, v.p.y, v.p.z);
		}
		for(int i : vertsUnique) {
			const MyVertex &v = verts[i];
			fprintf(fp, "vt %f %f\n", v.t.x, v.t.y);
		}
		for(int i : vertsUnique) {
			const MyVertex &v = verts[i];
			fprintf(fp, "vn %f %f %f\n", v.n.x, v.n.y, v.n.z);
		}
		for(const MyTriangle &t : mesh.tris) {
			// Use vertsUniqueMap to index into the vector of unique verts.
			// Add 1 to convert to index-1 from index-0.
			int v0 = vertsUniqueMap[t.v[0]] + 1;
			int v1 = vertsUniqueMap[t.v[1]] + 1;
			int v2 = vertsUniqueMap[t.v[2]] + 1;
			fprintf(fp, "f %d/%d/%d %d/%d/%d %d/%d/%d\n", v0, v0, v0, v1, v1, v1, v2, v2, v2);
		}
		fclose(fp);
	}
	
	return true;
}

bool saveSkin(const ofbx::IScene *scene)
{
	cout << "=== skin ===" << endl;
	
	// Get sizes
	int mesh_count = scene->getMeshCount();
	assert(mesh_count == myMeshes.size());
	int cluster_count = 0;
	int max_cluster_count = 0;

	// Populate attachment info
	for(int i = 0; i < mesh_count; ++i) {
		const ofbx::Mesh *mesh = scene->getMesh(i);
		const ofbx::Geometry *geom = mesh->getGeometry();
		const ofbx::Skin *skin = geom->getSkin();
		myMeshes[i].maxInfluences = 0;
		auto &verts = myMeshes[i].verts;
		auto& verts_map = myMeshes[i].vertsUniqueMap;
		if (skin == nullptr) {
			continue;
		}
		cluster_count = skin->getClusterCount();
		if (cluster_count > max_cluster_count) {
			max_cluster_count = cluster_count;
		}
		for(int j = 0; j < cluster_count; ++j) {
			const ofbx::Cluster *cluster = skin->getCluster(j);
			int indices_count = cluster->getIndicesCount();
			int weights_count = cluster->getWeightsCount();
			assert(indices_count == weights_count);
			if(weights_count > 0) {
				const int *indices = cluster->getIndices();
				const double *weights = cluster->getWeights();
				const ofbx::Object* limb = cluster->getLink();
				assert(limbMap.find(limb) != limbMap.end());
				// Store the clusters and their corrseonding indices for skeleton extraction later
				clusters_index.push_back(limbMap[limb]);
				clusters.push_back(cluster);
				for(int k = 0; k < weights_count; ++k) {
					int index = indices[k];
					double weight = weights[k];
					assert(index < geom->getVertexCount());

					// Error may exist in float comparison. Assume that all weights are correct
					//assert(0.0 <= weight && weight <= 1.0);
					verts[index].w.push_back(weight);
					verts[index].i.push_back(clusters_index.back());
					if(verts[index].i.size() > myMeshes[i].maxInfluences) {
						myMeshes[i].maxInfluences = (int)verts[index].i.size();
					}
				}
			}			
		}
	}
	
	// Write to attachment file
	for(const auto &mesh : myMeshes) {
		string path = FILENAME + "_" + mesh.name + "_skin.txt";
		FILE* fp = fopen(path.c_str(), "wb");
		if (!fp) {
			cout << "Could not write to " << path << endl;
			return false;
		}
		const auto &verts = mesh.verts;
		const auto &vertsUnique = mesh.vertsUnique;
		int vertex_count = vertsUnique.size();
		cout << "Saving to " << path << endl;
		fprintf(fp, "# 1st line: vertCount boneCount maxInfluences\n");
		fprintf(fp, "# Each subsequent line corresponds to a vertex.\n");
		fprintf(fp, "# In each line, the first number is the number of bone influences for the vertex.\n");
		fprintf(fp, "# The next numbers are \"influence\" pairs of {bone index, bone weight}.\n");
		fprintf(fp, "%d %d %d\n", vertex_count, (int)limbVec.size(), mesh.maxInfluences);
		for(int i : vertsUnique) {
			const MyVertex &v = verts[i];
			int influences = v.w.size();
			assert(influences == v.i.size());
			fprintf(fp, "%d ", influences);
			for(int j = 0; j < influences; ++j) {
				fprintf(fp, "%d %f ", v.i[j], v.w[j]);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}
	
	return true;
}

void traverseAll(const ofbx::Object *object, int depth)
{
	int depthMax = 8; // -1 to disable
	if(depth == depthMax) {
		return;
	}
	string s;
	for(int i = 0; i < depth; ++i) {
		s += "\t";
	}
	const char* typeLabel;
	switch (object->getType()) {
		case ofbx::Object::Type::GEOMETRY: typeLabel = "GEOMETRY"; break;
		case ofbx::Object::Type::MESH: typeLabel = "MESH"; break;
		case ofbx::Object::Type::MATERIAL: typeLabel = "MATERIAL"; break;
		case ofbx::Object::Type::ROOT: typeLabel = "ROOT"; break;
		case ofbx::Object::Type::TEXTURE: typeLabel = "TEXTURE"; break;
		case ofbx::Object::Type::NULL_NODE: typeLabel = "NULL"; break;
		case ofbx::Object::Type::LIMB_NODE: typeLabel = "LIMB"; break;
		case ofbx::Object::Type::NODE_ATTRIBUTE: typeLabel = "ATTRIBUTE"; break;
		case ofbx::Object::Type::CLUSTER: typeLabel = "CLUSTER"; break;
		case ofbx::Object::Type::SKIN: typeLabel = "SKIN"; break;
		case ofbx::Object::Type::ANIMATION_STACK: typeLabel = "ANIM_STACK"; break;
		case ofbx::Object::Type::ANIMATION_LAYER: typeLabel = "ANIM_LAYER"; break;
		case ofbx::Object::Type::ANIMATION_CURVE: typeLabel = "ANIM_CURVE"; break;
		case ofbx::Object::Type::ANIMATION_CURVE_NODE: typeLabel = "ANIM_CURVE_NODE"; break;
		default: assert(false); break;
	}
	
	if(object->getType() != ofbx::Object::Type::NODE_ATTRIBUTE) {
		cout << s << typeLabel << " : " << object->name;
		if(object->getType() == ofbx::Object::Type::GEOMETRY) {
			//cout << " : " << ((ofbx::Geometry*)object)->getVertexCount() << " verts";
		}
		cout << endl;
	}
	
	int i = 0;
	while(const ofbx::Object* child = object->resolveObjectLink(i)) {
		traverseAll(child, depth + 1);
		++i;
	}
}

void traverseLimbs(const ofbx::Object *limb, vector<const ofbx::Object*> &limbVec, map<const ofbx::Object*, int> &limbMap, vector<int> &limbParents)
{
	if(limb->getType() != ofbx::Object::Type::LIMB_NODE) {
		return;
	}
	
	limbVec.push_back(limb);
	limbMap[limb] = limbMap.size();
	const ofbx::Object *parent = limb->getParent();
	if(parent != nullptr && parent->getType() == ofbx::Object::Type::LIMB_NODE) {
		limbParents.push_back(limbMap[parent]);
	} else {
		limbParents.push_back(-1);
	}
	
	int i = 0;
	while(const ofbx::Object* child = limb->resolveObjectLink(i)) {
		traverseLimbs(child, limbVec, limbMap, limbParents);
		++i;
	}
}

bool saveAnim(const ofbx::IScene *scene)
{
	cout << "=== skeleton ===" << endl;
	
	// http://docs.autodesk.com/FBX/2014/ENU/FBX-SDK-Documentation/index.html?url=cpp_ref/class_fbx_pose.html,topicNumber=cpp_ref_class_fbx_pose_html3b4fd8fc-688e-43ec-b1df-56acb1cce550
	// Cluster is a bone
	// Use depthMax = 6 in traverseAll() to see the hierarchy:
	//
	//	MESH : newVegas:Elvis_BodyGeo
	//		GEOMETRY : newVegas:Elvis_BodyGeo
	//			SKIN : Skin newVegas:Elvis_BodyGeo
	//				CLUSTER : Link 0
	//					LIMB : newVegas:Hips
	//					LIMB : newVegas:Hips
	//				CLUSTER : Link 1
	//					LIMB : newVegas:Pelvis
	//					LIMB : newVegas:Pelvis
	//				CLUSTER : Link 2
	//					LIMB : newVegas:LeftUpLeg
	//					LIMB : newVegas:LeftUpLeg
	//				CLUSTER : Link 3
	//					LIMB : newVegas:LeftLeg
	//					LIMB : newVegas:LeftLeg
	//
	// TransformLink is the global transform of the bone(link) at the binding moment
	
	int mesh_count = scene->getMeshCount();
	if(mesh_count == 0) {
		cout << "This file has no mesh" << endl;
		return true;
	}
	
	// Casts double[16] to glm::mat4
	auto toMat4 = [](const double *array) {
		mat4 M;
		for(int row = 0; row < 4; ++row) {
			for(int col = 0; col < 4; ++col) {
				double v = array[4 * col + row];
				M[col][row] = v;
			}
		}
		return M;
	};
	
	// Get an Euler angle rotation matrix
	auto toR = [](const ofbx::Vec3 &v, ofbx::RotationOrder ro) {
		mat4 I = mat4(1.0);
		float x = v.x * M_PI / 180.0f;
		float y = v.y * M_PI / 180.0f;
		float z = v.z * M_PI / 180.0f;
		mat4 Rx = rotate(I, x, vec3(1.0, 0.0, 0.0));
		mat4 Ry = rotate(I, y, vec3(0.0, 1.0, 0.0));
		mat4 Rz = rotate(I, z, vec3(0.0, 0.0, 1.0));
		mat4 R = I;
		switch(ro) {
			case ofbx::RotationOrder::EULER_XYZ:
				R = Rz * Ry * Rx;
				break;
			case ofbx::RotationOrder::EULER_XZY:
				R = Ry * Rz * Rx;
				break;
			case ofbx::RotationOrder::EULER_YZX:
				R = Rx * Rz * Ry;
				break;
			case ofbx::RotationOrder::EULER_YXZ:
				R = Rz * Rx * Ry;
				break;
			case ofbx::RotationOrder::EULER_ZXY:
				R = Ry * Rx * Rz;
				break;
			case ofbx::RotationOrder::EULER_ZYX:
				R = Rx * Ry * Rz;
				break;
			case ofbx::RotationOrder::SPHERIC_XYZ:
				assert(false);
				break;
		}
		return R;
	};
	
	// Get a translation matrix
	auto toT = [](const ofbx::Vec3 &v) {
		mat4 I = mat4(1.0);
		mat4 T = translate(I, vec3(v.x, v.y, v.z));
		return T;
	};
	
	// Get a scale matrix
	auto toS = [](const ofbx::Vec3 &v) {
		mat4 I = mat4(1.0);
		if(v.x == 0 && v.y == 0 && v.z == 0) {
			// Assuming that 0 scale was sent in by mistake
			return I;
		} else {
			mat4 S = scale(I, vec3(v.x, v.y, v.z));
			return S;
		}
	};
	
	// prints glm::mat4
	auto printMat4 = [](const mat4 &M, const char *name = nullptr) {
		if(name != nullptr) {
			cout << name << " = [" << endl;
		}
		for(int row = 0; row < 4; ++row) {
			for(int col = 0; col < 4; ++col) {
				cout << M[col][row] << " ";
			}
			cout << endl;
		}
		if(name != nullptr) {
			cout << "];" << endl;
		}
	};
	

	// Store rest pose
	vector<mat4> pose0;

	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		int p = limbParents[j];
		mat4 P = mat4(1.0);
		if (p != -1) {
			P = pose0[p];
		}
		ofbx::RotationOrder ro = limb->getRotationOrder();
		mat4 R = toR(limb->getLocalRotation(), ro);
		mat4 T = toT(limb->getLocalTranslation());
		mat4 Roff = toR(limb->getRotationOffset(), ro);
		mat4 Rp = toR(limb->getRotationPivot(), ro);
		mat4 Rpre = toR(limb->getPreRotation(), ro);
		mat4 Rpost = toR(limb->getPostRotation(), ro);
		mat4 Soff = toS(limb->getScalingOffset());
		mat4 Sp = toS(limb->getScalingPivot());
		mat4 S = toS(limb->getLocalScaling());

		mat4 E = P * T * Roff * Rp * Rpre * R * inverse(Rpost) * inverse(Rp) * Soff * Sp * S * inverse(Sp);
		pose0.push_back(E);
	}
	

	assert(key_count_max != -1);
	
	// Store the local transform values
	vector< vector<mat4> > Ts(limbVec.size());
	vector< vector<mat4> > Rs(limbVec.size());
	for(int j = 0; j < limbVec.size(); ++j) {
		//const ofbx::Cluster *cluster = clusters[j];
		const ofbx::Object *limb = limbVec[j];
		// limb has one or more children that are curve nodes.
		vector<ofbx::Vec3> rotations;
		vector<ofbx::Vec3> positions;
		vector<ofbx::Vec3> scales; // not sure
		int i = 0;
		int key_count = 0; // key count for this limb
		while(const ofbx::Object* child = limb->resolveObjectLink(i)) {
			if(child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
				const ofbx::AnimationCurveNode* node = (ofbx::AnimationCurveNode*)child;
				const ofbx::AnimationCurve* curveX = node->getCurve(0);
				const ofbx::AnimationCurve* curveY = node->getCurve(1);
				const ofbx::AnimationCurve* curveZ = node->getCurve(2);

				const long long* key_time;
				if (curveX != nullptr) {
					key_count = curveX->getKeyCount();
					key_time = curveX->getKeyTime();
				}
				else if (curveY != nullptr) {
					key_count = curveY->getKeyCount();
					key_time = curveY->getKeyTime();
				}
				else if (curveZ != nullptr) {
					key_count = curveZ->getKeyCount();
					key_time = curveZ->getKeyTime();
				}
				else {
					++i;
					continue;
				}

				std::vector<float> emptyKeyValue(key_count, 0);
				const float* xvals = emptyKeyValue.data();
				const float* yvals = emptyKeyValue.data();
				const float* zvals = emptyKeyValue.data();

				if (curveX != nullptr) 
					xvals = curveX->getKeyValue();
				//else 
				//	xvals = emptyKeyValue.data();

				if (curveY != nullptr)
					yvals = curveY->getKeyValue();
				//else 
				//	yvals = emptyKeyValue.data();
				
				if (curveZ != nullptr)
					zvals = curveZ->getKeyValue();
				else
					zvals = emptyKeyValue.data();
				

				// Use key time to align frames for each node
				int index = 0;
				for(int k = 0; k < key_count_max; k++) {
					double step = 1.0 / (key_count_max - 1);
					double delta = 1e-6;
					if (key_count > 1) {
						while (index < key_count && (key_time[index] - key_time[0]) * 1.0 / (key_time[key_count - 1] - key_time[0]) <= step * k + delta) {
							index++;
						}
					}
					else { index = 1; }
					
					ofbx::Vec3 v;
					v.x = xvals[index - 1];
					v.y = yvals[index - 1];
					v.z = zvals[index - 1];

					if(strcmp(child->name, "R") == 0) {
						rotations.push_back(v);
					} else if(strcmp(child->name, "T") == 0) {
						positions.push_back(v);
					} else if(strcmp(child->name, "S")==0){
						cout << "warning! ignoring S "<< v.x<<","<<v.y<<","<<v.z<<endl;
						scales.push_back(v);
					} else {
						cout << "Unsupported animation curve type" << endl;
						assert(false);
					}
				}
			}
			++i;
		}

		for(int k = 0; k < key_count_max; ++k) {
			// http://docs.autodesk.com/FBX/2014/ENU/FBX-SDK-Documentation/cpp_ref/class_fbx_cluster.html
			if(k < positions.size()) {
				Ts[j].push_back(toT(positions[k]));
			}
			if(k < rotations.size()) {
				Rs[j].push_back(toR(rotations[k], limb->getRotationOrder()));
			}
		}
	}
	
	// These poses are in local coordinates, so transform them to world
	// https://help.autodesk.com/view/FBX/2017/ENU/?guid=__files_GUID_10CDD63C_79C1_4F2D_BB28_AD2BE65A02ED_htm
	// WorldTransform = ParentWorldTransform * T * Roff * Rp * Rpre * R * Rpost^-1 * Rp^-1 * Soff * Sp * S * Sp^-1
	vector< vector<mat4> > Es(limbVec.size());

	for(int k = 0; k < key_count_max; ++k) {
		for(int j = 0; j < limbVec.size(); ++j) {
			const ofbx::Object* limb = limbVec[j];
			int p = limbParents[j];
			mat4 P = mat4(1.0);
			if (p != -1) {
				P = Es[p][k];
			}
			ofbx::RotationOrder ro = limb->getRotationOrder();
			mat4 R = toR(limb->getLocalRotation(), ro);
			mat4 T = toT(limb->getLocalTranslation());
			if(Rs[j].empty()) {
				// No key frames for this limb. Use rest pose.
			} else {
				if(k < Rs[j].size()) {
					// Use this key frame.
					R = Rs[j][k];
				} else {
					// Use the last stored key frame.
					R = Rs[j][Rs[j].size() - 1];
				}
			}
			if(Ts[j].empty()) {
				// No key frames for this limb. Use rest pose.
			} else {
				if(k < Ts[j].size()) {
					// Use this key frame.
					T = Ts[j][k];
				} else {
					// Use the last stored key frame.
					T = Ts[j][Ts[j].size() - 1];
				}
			}
			mat4 Roff = toR(limb->getRotationOffset(), ro);
			mat4 Rp = toR(limb->getRotationPivot(), ro);
			mat4 Rpre = toR(limb->getPreRotation(), ro);
			mat4 Rpost = toR(limb->getPostRotation(), ro);
			mat4 Soff = toS(limb->getScalingOffset());
			mat4 Sp = toS(limb->getScalingPivot());
			mat4 S = toS(limb->getLocalScaling());
			//printMat4(P, "P");
			//printMat4(R, "R");
			//printMat4(T, "T");
			//printMat4(Roff, "Roff");
			//printMat4(Rp, "Rp");
			//printMat4(Rpost, "Rpost");
			//printMat4(Soff, "Soff");
			//printMat4(Sp, "Sp");
			//printMat4(S, "S");
			mat4 E = P * T * Roff * Rp * Rpre * R * inverse(Rpost) * inverse(Rp) * Soff * Sp * S * inverse(Sp);
			Es[j].push_back(E);
			/*
			if (k == 0 && j == 0) {
				cout << "E: " << glm::to_string(Es[0][0]) << endl;
				cout << "R: " << glm::to_string(R) << endl;
				cout << "T: " << glm::to_string(T) << endl;
			}
			*/
		}
	}
	
	// Write to skeleton file
	string path = FILENAME + "_skel.txt";
	FILE* fp = fopen(path.c_str(), "wb");
	if (!fp) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;
	fprintf(fp, "# 1st line: frameCount boneCount\n");
	fprintf(fp, "# Each subsequent line is a frame with \"boneCount\" sets of\n");
	fprintf(fp, "# quaternions (x,y,z,w) and positions (x,y,z).\n");
	fprintf(fp, "%d %d\n", key_count_max, (int)limbVec.size());
	
	
	for (int j = 0; j < limbVec.size(); ++j) {
		const auto& M = pose0[j];
		quat q = quat_cast(M);
		vec3 p(M[3]);
		fprintf(fp, "%f %f %f %f %f %f %f ", q.x, q.y, q.z, q.w, p.x, p.y, p.z);
	}
	fprintf(fp, "\n");
	
	for(int k = 0; k < key_count_max; ++k) {
		for(int j = 0; j < limbVec.size(); ++j) {
			const mat4 &E = Es[j][k];
			quat q = quat_cast(E);
			vec3 p(E[3]);
			fprintf(fp, "%f %f %f %f %f %f %f ", q.x, q.y, q.z, q.w, p.x, p.y, p.z);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
	return true;
}

bool saveInputfile() {
	// Automatically generate the input file for skinning program input
	char sep = '/';
	size_t i = FILENAME.rfind(sep, FILENAME.length());
	string path = FILENAME.substr(0, i + 1) + "input.txt";
	FILE* fp = fopen(path.c_str(), "wb");
	if (!fp) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;

	string file_name;
	if (i != string::npos) {
		file_name = FILENAME.substr(i + 1, FILENAME.length() - i);
	}

	fprintf(fp, "# Each line starts with a keyword:\n");
	fprintf(fp, "# - TEXTURE <texture file>\n");
	fprintf(fp, "# - MESH <obj file> <skin file> <texture file>\n");
	fprintf(fp, "# - SKELETON <skeleton file>\n");
	fprintf(fp, "# Alpha blending is used to render the mouth, eyes, and brows. Since the brows mesh covers the eyes mesh,\n");
	fprintf(fp, "# the brows mesh should be rendered after the eyes mesh.\n");
	fprintf(fp, "# Lines for textures in automatically generated input file could be wrong when there are multiple textures, please modify them manually.\n");
	fprintf(fp, "TEXTURE ");
	fprintf(fp, "%s", TEXTURENAME.c_str());
	fprintf(fp, "\n");
	fprintf(fp, "SKELETON ");
	fprintf(fp, "%s", file_name.c_str());
	fprintf(fp, "_skel.txt\n");
	
	for (const auto& mesh : myMeshes) {
		string path = file_name + "_" + mesh.name;
		string input_line = "MESH " + path + ".obj " + path + "_skin.txt " + TEXTURENAME + " \n";
		fprintf(fp, "%s", input_line.c_str());

	}
	
	fclose(fp);

	return true;

}

bool saveTexture(const ofbx::IScene* scene)
{
	cout << "=== Texture ===" << endl;

	const ofbx::IElement* root = scene->getRootElement();
	const ofbx::IElement* child = root->getFirstChild();
	vector<const ofbx::IElement*> videos = find_element(root, "Video");

	// The texture files are stored in fbx as Video element
	for (auto video : videos) {
		vector<const ofbx::IElement*> contents = find_element(video, "Content");
		vector<const ofbx::IElement*> filenames = find_element(video, "Filename");
		if (contents.size() != 0 && contents[0]->getFirstProperty() != nullptr) {
			ofbx::DataView values = contents[0]->getFirstProperty()->getValue();
			ofbx::DataView texture_name = filenames[0]->getFirstProperty()->getValue();
			string name;
			for (long i=0 ; i < texture_name.end - texture_name.begin; i++) {
				name += static_cast<char>(*(texture_name.begin + i));
			}
			string path = FILENAME + "_texture.png";
			size_t l = name.rfind('/', name.length());
			if (l != string::npos) {
				path = FILENAME + "_" + name.substr(l + 1, name.length() - l);
				cout << "Extracting texture: " << name.substr(l + 1, name.length() - l) << endl;
				if (TEXTURENAME.length() == 0) {
					size_t s = path.rfind('/', path.length());
					TEXTURENAME = path.substr(s + 1, path.length() - s);
				}
			}
			FILE* fp = fopen(path.c_str(), "wb");
			if (!fp) {
				cout << "Could not write to " << path << endl;
				return false;
			}

			long data_length = values.end - values.begin;
			/*
			cout << int((values.begin)[0]) << endl;
			cout << int((values.begin)[1]) << endl;
			cout << int((values.begin)[2]) << endl;
			cout << int((values.begin)[3]) << endl;
			*/

			fwrite(values.begin + 4, sizeof(unsigned char), data_length, fp);
			fclose(fp);
		}
	}


	return true;
}

bool saveLocalTransfomfile() {
	// Get an Euler angle rotation matrix
	auto toR = [](const ofbx::Vec3& v, ofbx::RotationOrder ro) {
		mat4 I = mat4(1.0);
		float x = v.x * M_PI / 180.0f;
		float y = v.y * M_PI / 180.0f;
		float z = v.z * M_PI / 180.0f;
		mat4 Rx = rotate(I, x, vec3(1.0, 0.0, 0.0));
		mat4 Ry = rotate(I, y, vec3(0.0, 1.0, 0.0));
		mat4 Rz = rotate(I, z, vec3(0.0, 0.0, 1.0));
		mat4 R = I;
		switch (ro) {
		case ofbx::RotationOrder::EULER_XYZ:
			R = Rz * Ry * Rx;
			break;
		case ofbx::RotationOrder::EULER_XZY:
			R = Ry * Rz * Rx;
			break;
		case ofbx::RotationOrder::EULER_YZX:
			R = Rx * Rz * Ry;
			break;
		case ofbx::RotationOrder::EULER_YXZ:
			R = Rz * Rx * Ry;
			break;
		case ofbx::RotationOrder::EULER_ZXY:
			R = Ry * Rx * Rz;
			break;
		case ofbx::RotationOrder::EULER_ZYX:
			R = Rx * Ry * Rz;
			break;
		case ofbx::RotationOrder::SPHERIC_XYZ:
			assert(false);
			break;
		}
		return R;
	};

	// Get a translation matrix
	auto toT = [](const ofbx::Vec3& v) {
		mat4 I = mat4(1.0);
		mat4 T = translate(I, vec3(v.x, v.y, v.z));
		return T;
	};

	// Get a scale matrix
	auto toS = [](const ofbx::Vec3& v) {
		mat4 I = mat4(1.0);
		if (v.x == 0 && v.y == 0 && v.z == 0) {
			// Assuming that 0 scale was sent in by mistake
			return I;
		}
		else {
			mat4 S = scale(I, vec3(v.x, v.y, v.z));
			return S;
		}
	};

	// Write to local static transforms files
	string path = FILENAME + "_static_transforms.txt";
	FILE* fp = fopen(path.c_str(), "wb");
	if (!fp) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;
	fprintf(fp, "# https://help.autodesk.com/view/FBX/2017/ENU/?guid=__files_GUID_10CDD63C_79C1_4F2D_BB28_AD2BE65A02ED_htm \n");
	fprintf(fp, "# WorldTransform = ParentWorldTransform * T * Roff * Rp * Rpre * R * Rpost^-1 * Rp^-1 * Soff * Sp * S * Sp^-1\n");
	fprintf(fp, "# Of these transforms, the only time - varying ones are : T for the root jointand R for all the joints.\n");
	fprintf(fp, "# This file contains the static transforms, listed in left - to - right order, one joint per line :\n");
	fprintf(fp, "# Each line contains 8 matrices: T Roff Rp Rpre Rpost Soff Sp S\n");
	fprintf(fp, "# Each matrix is 7 numbers: 4 for quaternion(x, y, z, w) and 3 for position(x, y, z), so that each line has 7 * 8 = 56 numbers.\n");
	fprintf(fp, "# Even though the root joint's T is time-varying, we list it here as an identity matrix to simplify parsing.\n");
	fprintf(fp, "# So, the total number of lines is the same as boneCount, which is specified by the first number in the file.\n");
	fprintf(fp, "%d\n",(int)limbVec.size());

	path = FILENAME + "_skel_local.txt";
	FILE* fp2 = fopen(path.c_str(), "wb");
	if (!fp2) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;
	fprintf(fp2, "# 1st line : frameCount boneCount\n");
	fprintf(fp2, "# Each subsequent line is a frame with \"boneCount\" sets of Euler angles.\n");
	fprintf(fp2, "# The root joint contains Euler angles and positions, while all other joints contain just Euler angles.\n");
	fprintf(fp2, "# So, each line has 3 + 3 * boneCount numbers.\n");
	fprintf(fp2, "%d %d\n", key_count_max, (int)limbVec.size());

	// Store the local transform values
	vector< vector<float> > Ts(limbVec.size());
	vector< vector<float> > Rs(limbVec.size());
	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		// limb has one or more children that are curve nodes.
		vector<float> rotations;
		vector<float> positions;
		vector<float> scales;
		int i = 0;
		int key_count = 0; // key count for this limb
		while (const ofbx::Object* child = limb->resolveObjectLink(i)) {
			if (child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
				const ofbx::AnimationCurveNode* node = (ofbx::AnimationCurveNode*)child;
				const ofbx::AnimationCurve* curveX = node->getCurve(0);
				const ofbx::AnimationCurve* curveY = node->getCurve(1);
				const ofbx::AnimationCurve* curveZ = node->getCurve(2);

				const long long* key_time;
				if (curveX != nullptr) {
					key_count = curveX->getKeyCount();
					key_time = curveX->getKeyTime();
				}
				else if (curveY != nullptr) {
					key_count = curveY->getKeyCount();
					key_time = curveY->getKeyTime();
				}
				else if (curveZ != nullptr) {
					key_count = curveZ->getKeyCount();
					key_time = curveZ->getKeyTime();
				}
				else {
					++i;
					continue;
				}

				std::vector<float> emptyKeyValue(key_count, 0);
				const float* xvals = emptyKeyValue.data();
				const float* yvals = emptyKeyValue.data();
				const float* zvals = emptyKeyValue.data();

				if (curveX != nullptr)
					xvals = curveX->getKeyValue();

				if (curveY != nullptr)
					yvals = curveY->getKeyValue();

				if (curveZ != nullptr)
					zvals = curveZ->getKeyValue();

				// Use key time to align frames for each node
				int index = 0;
				for (int k = 0; k < key_count_max; k++) {
					double step = 1.0 / (key_count_max - 1);
					double delta = 1e-6;
					if (key_count > 1) {
						while (index < key_count && (key_time[index] - key_time[0]) * 1.0 / (key_time[key_count - 1] - key_time[0]) <= step * k + delta) {
							index++;
						}
					}
					else { index = 1; }

					ofbx::Vec3 v;
					v.x = xvals[index - 1];
					v.y = yvals[index - 1];
					v.z = zvals[index - 1];
					if (strcmp(child->name, "R") == 0) {
						rotations.push_back(v.x * M_PI / 180.0);
						rotations.push_back(v.y * M_PI / 180.0);
						rotations.push_back(v.z * M_PI / 180.0);
					}
					else if (strcmp(child->name, "T") == 0) {
						positions.push_back(v.x);
						positions.push_back(v.y);
						positions.push_back(v.z);
					}
					else if (strcmp(child->name, "S") == 0) {
						cout << "warning! ignoring S "<< v.x<<","<<v.y<<","<<v.z<<endl;
						scales.push_back(v.x);
						scales.push_back(v.y);
						scales.push_back(v.z);
					}
					else 
					{
						cout << "Unsupported animation curve type" << endl;
						assert(false);
					}
				}
			}
			++i;
		}
		Ts[j].insert(Ts[j].end(), positions.begin(), positions.end());
		Rs[j].insert(Rs[j].end(), rotations.begin(), rotations.end());
	}

	// Get local transforms
	vector< vector<mat4> > Ls(limbVec.size());
	for (int k = 0; k < key_count_max; ++k) {
		for (int j = 0; j < limbVec.size(); ++j) {
			const ofbx::Object* limb = limbVec[j];

			// Mian difference from the world skeleton, P is always identity
			mat4 P = mat4(1.0);
			ofbx::RotationOrder ro = limb->getRotationOrder();
			ofbx::Vec3 R = limb->getLocalRotation();
			ofbx::Vec3 T = limb->getLocalTranslation();
			if (Rs[j].empty()) {
				// No key frames for this limb. Use rest pose.
			}
			else {
				// Use this key frame.
				R.x = Rs[j][k * 3];
				R.y = Rs[j][k * 3 + 1];
				R.z = Rs[j][k * 3 + 2];
			}
			if (Ts[j].empty()) {
				// No key frames for this limb. Use rest pose.
			}
			else {
				T.x = Ts[j][k * 3];
				T.y = Ts[j][k * 3 + 1];
				T.z = Ts[j][k * 3 + 2];
			}
			
			if (k == 0) {
				vector<mat4> transforms;
				transforms.push_back(toT(T));
				transforms.push_back(toR(limb->getRotationOffset(), ro));
				transforms.push_back(toR(limb->getRotationPivot(), ro));
				transforms.push_back(toR(limb->getPreRotation(), ro));
				transforms.push_back(toR(limb->getPostRotation(), ro));
				transforms.push_back(toS(limb->getScalingOffset()));
				transforms.push_back(toS(limb->getScalingPivot()));
				transforms.push_back(toS(limb->getLocalScaling()));
				if (j == 0)
				{
					transforms[0] = mat4(1.0);
				}
				for (auto matrix : transforms) {
					quat q = quat_cast(matrix);
					vec3 p(matrix[3]);
					fprintf(fp, "%f %f %f %f %f %f %f ", q.x, q.y, q.z, q.w, p.x, p.y, p.z);
				}
				fprintf(fp, "\n");
			}
				
			if (j == 0) {
				//quat q = quat_cast(toR(R, ofbx::RotationOrder::EULER_XYZ));
				fprintf(fp2, "%f %f %f %f %f %f ", R.x, R.y, R.z, T.x, T.y, T.z);
				//cout << glm::to_string(q) << endl;
			}
			else {
				fprintf(fp2, "%f %f %f ", R.x, R.y, R.z);
			}
			
		}
		fprintf(fp2, "\n");
	}
	fclose(fp2);
	fclose(fp);

	// Write to hierarchy
	path = FILENAME + "_hierarchy.txt";
	fp = fopen(path.c_str(), "wb");
	if (!fp) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;
	fprintf(fp, "# The first line is the number of bones/joints.\n");
	fprintf(fp, "# All other files are:\n");
	fprintf(fp, "# <JOINT INDEX> <PARENT INDEX> <ROTATION ORDER> <JOINT NAME>\n");
	fprintf(fp, "%d\n", (int)limbVec.size());
	for (int j = 0; j < limbVec.size(); ++j) {
		int joint_index = j;
		int parent_index = limbParents[joint_index];
		fprintf(fp, "%d %d ", joint_index, parent_index);
		switch (limbVec[0]->getRotationOrder()) {
		case ofbx::RotationOrder::EULER_XYZ: fprintf(fp, "EULER_XYZ "); break;
		case ofbx::RotationOrder::EULER_XZY: fprintf(fp, "EULER_XZY "); break;
		case ofbx::RotationOrder::EULER_YXZ: fprintf(fp, "EULER_YXZ "); break;
		case ofbx::RotationOrder::EULER_YZX: fprintf(fp, "EULER_YZX "); break;
		case ofbx::RotationOrder::EULER_ZXY: fprintf(fp, "EULER_ZXY "); break;
		case ofbx::RotationOrder::EULER_ZYX: fprintf(fp, "EULER_YZX "); break;
		case ofbx::RotationOrder::SPHERIC_XYZ: fprintf(fp, "SPHERIC_XYZ "); break;
		}
		if (limbVec[j]->name) {
			fprintf(fp, "%s", limbVec[j]->name);
		}
		else {
			fprintf(fp, "noname");
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	// Wirte to the local transforms of binding pose 
	path = FILENAME + "_binding_pose_local.txt";
	fp = fopen(path.c_str(), "wb");
	if (!fp) {
		cout << "Could not write to " << path << endl;
		return false;
	}
	cout << "Saving to " << path << endl;
	fprintf(fp, "# 1st line : frameCount boneCount\n");
	fprintf(fp, "# Next line is the \"boneCount\" sets of Euler angles for binding pose.\n");
	fprintf(fp, "# The root joint contains Euler angles and positions, while all other joints contain just Euler angles.\n");
	fprintf(fp, "# So, each line has 3 + 3 * boneCount numbers.\n");
	fprintf(fp, "%d %d\n", key_count_max, (int)limbVec.size());


	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		ofbx::Vec3 R = limb->getLocalRotation();
		ofbx::Vec3 T = limb->getLocalTranslation();
		if (j == 0) {
			//quat q = quat_cast(toR(R, ofbx::RotationOrder::EULER_XYZ));
			fprintf(fp, "%f %f %f %f %f %f ", R.x, R.y, R.z, T.x, T.y, T.z);
			//cout << glm::to_string(q) << endl;
		}
		else {
			fprintf(fp, "%f %f %f ", R.x, R.y, R.z);
		}
	}
	fclose(fp);

	return true;
}

// https://www.oreilly.com/library/view/c-cookbook/0596007612/ch10s15.html
string getFileName(const string& s)
{
	char sep = '/';
	
#ifdef _WIN32
	sep = '\\';
#endif

	size_t i = s.rfind(sep, s.length());
	if(i != string::npos) {
		return(s.substr(i + 1, s.length() - i));
	}
	return("");
}
void resolve_limb_nodes(const ofbx::IScene* scene) {
	// Find the root limb (hips)
	const ofbx::Object* root = nullptr; // root limb
	int i = 0;
	
	const ofbx::Object* nullNode = nullptr; // root limb
											//
	while (const ofbx::Object* child = scene->getRoot()->resolveObjectLink(i)) {

		// Used for extracting fbx file from blender
		/*
		if (child->name[0] == 'A') {
			root = child->resolveObjectLink(0);
			break;
		}
		*/

		if (child->getType() == ofbx::Object::Type::LIMB_NODE) {
			root = child;
			break;
		}

		if(child->getType()==ofbx::Object::Type::NULL_NODE)
			nullNode=child;

		++i;
	}
	
	if(!root)
	{
		if (nullNode)
		{
			cout <<"nullnode:"<<endl;
			i=0;
			while (const ofbx::Object* child = nullNode->resolveObjectLink(i)) {
				if (child->getType() == ofbx::Object::Type::LIMB_NODE) {
					root = child;
					break;
				}
				i++;
			}
		}

		if(!root)
		{
			cout <<"root==nullptr???"<<endl;
			i=0;
			while (const ofbx::Object* child = scene->getRoot()->resolveObjectLink(i)) {
				cout << i <<":" <<(int)child->getType() <<endl;
				i++;
			}
			exit(0);
		}
	}

	// Get key count for root limb
	for (int i = 0; const ofbx::Object * child = root->resolveObjectLink(i); i++) {
		if (child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
			const ofbx::AnimationCurveNode* node = (ofbx::AnimationCurveNode*)child;
			const ofbx::AnimationCurve* curveX = node->getCurve(0);
			if (curveX == nullptr) {
				++i;
				continue;
			}
			key_count_max = curveX->getKeyCount();

		}
	}

	// Traverse hierarchy and store the skeleton structure for later use
	traverseLimbs(root, limbVec, limbMap, limbParents);
	assert(limbVec.size() == limbParents.size() && limbVec.size() == limbMap.size());
}

int FBXmain1(const char *fn)
{
	
	// Get file handle
	string filename = fn;
	FILE* fp = fopen(filename.c_str(), "rb");
	if(!fp) {
		cout << filename << " not found" << endl;
		return -1;
	}

	// Get file size and allocate memory
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	content = new ofbx::u8[file_size];
	
	// Load data into memory
	fread(content, 1, file_size, fp);
	
	// Parse
	cout << "Parsing " << filename << endl;
	scene = ofbx::load((ofbx::u8*)content, file_size, (ofbx::u64)ofbx::LoadFlags::TRIANGULATE);
	if(!scene) {
		cout << ofbx::getError() << endl;
	}
	
	// Extract just the filename
	//FILENAME = getFileName(filename);

	// Store extracted file in the same folder
	FILENAME = filename;
	FILENAME = FILENAME.substr(0, FILENAME.length() - 4);
	
	// DEBUG: Traverse tree
	//traverseAll(scene->getRoot(), 0);

	resolve_limb_nodes(scene);
	
	return 0;
}
void saveAll(){

	// Parse and save
	saveGeom(scene);
	saveSkin(scene);
	saveAnim(scene);
	saveTexture(scene);
	saveInputfile();

	// Save local transfroms for each joint
	saveLocalTransfomfile();

}

void cleanup()
{
	// Delete data
	delete [] content;
	
	cout << "done" << endl;
}
};

#include "FBXimporter.h"
FBXimporter::FBXimporter(const char* filename)
{
	_FBXimport* temp=new _FBXimport();
	temp->FBXmain1(filename);
	_data=(void*)temp;
}
FBXimporter::		~FBXimporter()
{
	_FBXimport* temp=(_FBXimport*)_data;
	temp->cleanup();
	delete temp;
}
void FBXimporter::saveAllInformation()
{
	_FBXimport* temp=(_FBXimport*)_data;
	temp->saveAll();
}
