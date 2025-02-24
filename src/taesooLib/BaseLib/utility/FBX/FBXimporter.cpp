// The original code was from fbx-extract. modified heavily by Taesoo. (the animation loading part is completely rewritten) 
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

class CImage;
class TStrings;

#define KEYTIMECONV(x) ((double)((x)/(double)46186158000)) // I hate autodesk
/*
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>
*/
//#define VERBOSE

#include "ofbx.h"
#include <math.h>

void _saveTextureToMem(const char* path, long data_length, const unsigned char* ptr, TStrings& texture_names, std::vector<CImage*>& textures);
bool saveTexturesToMem(const ofbx::IScene* scene, TStrings& texture_names, std::vector<CImage*>& textures);
#include "../math/mathclass_minimum.h"
#include "../math/matrix4.h"
#include "../math/Operator.h"
#include "../QPerformanceTimer.h"
using namespace std;
//using namespace glm;
#define USE_HASH
inline void saveKey( int key_count, vectorn & vvals, vectorn& keyTime,
		const float* xvals, const long long* key_time)
{
	keyTime.setSize(key_count);
	vvals.setSize(key_count);
	for(int ii=0; ii<key_count; ii++) 
	{
		keyTime[ii]=KEYTIMECONV(key_time[ii]);
		vvals[ii]=xvals[ii];
	}
}
inline unsigned int hash100(vector3 const& p)
{
	unsigned int x=(unsigned int)(p.x);
	unsigned int y=(unsigned int)(p.y);
	unsigned int z=(unsigned int)(p.z);
	return (x ^ y ^ z)%100;
}
inline double ccCardinalSplineAt( double p0, double p1, double p2, double p3, double tension, double t )
{
		double t2 = t * t;
		double t3 = t2 * t;

		double s = (1.0 - tension) / 2.0;

		double b1 = s * ((-t3 + (2.0 * t2)) - t);					// s(-t3 + 2 t2 – t)P1
		double b2 = s * (-t3 + t2) + (2.0 * t3 - 3.0* t2 + 1.0);		// s(-t3 + t2)P2 + (2 t3 – 3 t2 + 1)P2
		double b3 = s * (t3 - 2.0 * t2 + t) + (-2.0 * t3 + 3.0 * t2);	// s(t3 – 2 t2 + t)P3 + (-2 t3 + 3 t2)P3
		double b4 = s * (t3 - t2);									// s(t3 – t2)P4

		return (p0*b1 + p1*b2 + p2*b3 + p3*b4); 
}

inline double getCardinalSplineABS(double t_global, vectorn const& time, vectorn const& control)
{
	double tension=0.33;
	//		tension=0.33 -- tension 0 : catmal-rom
	vectornView control1=control.range(0, control.size()-1);
	vectornView control2=control.range(1, control.size());
	double eps=1e-10;
	RANGE_ASSERT(t_global>=0.0);
	double t=0;
	double w;
	int iseg=-1;
	for (int i=0; i<time.size()-1; i++)
	{
		t=time(i+1);
		double d=t-time(i);
		if (t >= t_global-eps) // -- at junctions, returns previous spline (=)
		{
			iseg=i;
			w=sop::map(t_global, t-d, t, 0, 1);
			break;
		}
	}
	if (iseg==-1)
		return control2(control2.size()-1);

	double pc;
	double nc;
	if (iseg==0 )
	{
		pc=control1(0);
		if (w<0.0) w=0.0;
	}
	else
		pc=control1(iseg-1);

	if (iseg==control2.size()-1 )
	{
		if (w>1.0) w=1.0;
		nc=control2(iseg);
	}
	else
		nc=control2(iseg+1);
	double v= ccCardinalSplineAt( pc, control1(iseg), control2(iseg), nc, tension, w );
#ifdef VERBOSE
	cout<<"iseg"<<iseg<<"/"<<time.size()<<" "<<w<< " "<<pc<<" "<< control1(iseg) << " "<< control2(iseg)<< " "<<nc << " "<<v <<endl;
#endif
	return  v;
}
double sampleCurve1D(const vectorn& kt, const vectorn& kv, double t)
{
#ifdef VERBOSE
	cout <<kt.size()<< kt << kv<<endl;
#endif

	if (kt.size()==0)
	{
		return 0.0;
	}
	else  if(kt.size()==1)
		return kv(0);
	else if(kv.size()==2)
	{
		// linear
		ASSERT(kt.size()==2);
		return sop::map(t, kt(0), kt(1), kv(0), kv(1));
	}
	return getCardinalSplineABS(t, kt, kv);
}
vector3 sampleCurve3D(const vectorn * kt, const vectorn * kv, double t)
{
	vector3 out;
#ifdef VERBOSE
	cout<<"sample3D"<<endl;
#endif
	out.x=sampleCurve1D(kt[0], kv[0], t);
	out.y=sampleCurve1D(kt[1], kv[1], t);
	out.z=sampleCurve1D(kt[2], kv[2], t);
	return out;
}
// https://github.com/nem0/OpenFBX/blob/master/demo/main.cpp
// https://github.com/Larry955/FbxParser/tree/master/FbxParser
inline quater quat_cast(matrix4 const& m)
{
	quater q;
	q.setRotation(m);
	return q;
}

inline matrix4 translate(matrix4 const& m, vector3 const& v)
{
	matrix4 t;
	t.setTranslation(v, false);
	return m* t;
}
inline matrix4 scale(matrix4 const& m, vector3 const& v)
{
	matrix4 t;
	t.setScaling(v.x, v.y, v.z);
	return m* t;
}
inline matrix4 inverse(matrix4 const& m) {return m.inverse();}
inline vector3 get_vector(const ofbx::IElement* property) {
	// Get the vaule of sepcific property
	const ofbx::IElementProperty* iter = property->getFirstProperty();
	while (iter != nullptr && 
			(
			 iter->getType() != ofbx::IElementProperty::Type::DOUBLE
			 && iter->getType() != ofbx::IElementProperty::Type::LONG
			)
		  ) {
		//printf("skipping %c\n", iter->getType());
		iter = iter->getNext();
	}
	vector3 vec (0.0f, 0.0f, 0.0f);
	if (iter == nullptr) {
		return vec;
	}
	for (int i = 0; i < 3; i++) {
		vec[i] = iter->getValue().toDouble();
		iter = iter->getNext();
	}
	assert(iter==nullptr);
	return vec;
}
inline vector<const ofbx::IElement*> find_element(const ofbx::IElement* parent_element, const string& id) {
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

inline const ofbx::IElement* find_child(const ofbx::IElement& element, const char* id)
{	// Helper function for find property of an element
	const ofbx::IElement* iter = element.getFirstChild();
	while (iter != nullptr)
	{
		if (iter->getID() == id) return iter;
		iter = iter->getSibling();
	}
	return nullptr;
}

inline ofbx::IElement* find_property(const ofbx::IElement& obj, const char* name)
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


inline matrix4 get_T_matrix(const ofbx::IScene* scene, const char* mesh_name) {
	// Get the transform matrix for meshes from original pose to binding pose
	// Usually it is an identity matrix, but for some fbx files it is necessary to apply
	// this transforms to get correct results.

	matrix4 transform_matrix;
	transform_matrix.identity();
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
			vector3 trans_vec (0.0f, 0.0f, 0.0f);
			vector3 rotation_vec (0.0f, 0.0f, 0.0f);
			vector3 scaling_vec (1.0f, 1.0f, 1.0f);

			if (translation) {
				trans_vec = get_vector(translation);
			}
			if (rotation) {
				rotation_vec = get_vector(rotation);
			}
			if (scaling) {
				scaling_vec = get_vector(scaling);
			}

			matrix4 R;
			R.identity();
			float x = rotation_vec.x * M_PI / 180.0f;
			float y = rotation_vec.y * M_PI / 180.0f;
			float z = rotation_vec.z * M_PI / 180.0f;
			double avalue[]={double(z), double(y), double(x)};
			R.setRotation("ZYX", avalue);
			transform_matrix = translate(transform_matrix, trans_vec) * R;
			transform_matrix = scale(transform_matrix, scaling_vec);
			/*
			cout << "(" << trans_vec.x  << ", " << trans_vec.y << ", " << trans_vec.z << ")" << endl;
			cout << "(" << rotation_vec.x << ", " << rotation_vec.y << ", " << rotation_vec.z << ")" << endl;
			cout << "(" << scaling_vec.x << ", " << scaling_vec.y << ", " << scaling_vec.z << ")" << endl;
			*/

		}
	}
	return transform_matrix;
}
class _FBXimport{
	public:
ofbx::IScene* scene;
ofbx::u8* content;
int g_nFrames;
float g_T;
float g_frameTime;
		_FBXimport()
		{
			scene=NULL;
			content=NULL;
			g_nFrames=0;
		}
string FILENAME;
string TEXTURENAME;

vector<const ofbx::Cluster*> clusters;
vector<int> clusters_index;

vector<const ofbx::Object*> limbVec;
//map<const ofbx::Object*, int> limbMap; -> doesn't work correctly on VS2022-.-
vector<int> limbParents;

int key_count_max = -1;

typedef vector3 vec3;
typedef vector3 vec2;
typedef matrix4 mat4;
typedef quater quat;

	// Casts double[16] to mat4
	inline static mat4 toMat4 (const double *array) {
		mat4 M;
		for(int row = 0; row < 4; ++row) {
			for(int col = 0; col < 4; ++col) {
				double v = array[4 * col + row];
				M(col, row) = v;
			}
		}
		return M;
	};
	// Get an Euler angle rotation matrix

	inline static mat4 toR (const vector3 &v, ofbx::RotationOrder ro) {
		mat4 I ;
		I.identity();
		double x = v.x * M_PI / 180.0;
		double y = v.y * M_PI / 180.0;
		double z = v.z * M_PI / 180.0;

		mat4 R = I;
#define USE_SIMPLE
#ifdef USE_SIMPLE
		mat4 Rx, Ry, Rz;
		Rx.setRotationX(x);
		Ry.setRotationY(y);
		Rz.setRotationZ(z);
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
				RANGE_ASSERT(false);
				break;
		}
#else
		Msg::verify(ro== ofbx::RotationOrder::EULER_XYZ, "unsupportedyet");
		quater q;
		double a[3]={z,y,x};
		q.setRotation("ZYX", a);
		R.setRotation(q);
#endif
		return R;
	};
	inline static mat4 toR (const ofbx::Vec3 &v, ofbx::RotationOrder ro) {return toR(vec3(v.x, v.y, v.z), ro);}
	// Get a translation matrix
	inline static mat4 toT (const vector3& v) { mat4 T ; T.setTranslation(v, false); return T; };
	inline static mat4 toT (const ofbx::Vec3 &v) { return toT(vec3(v.x, v.y, v.z));}
	
	// Get a scale matrix
	inline static mat4 toS (const ofbx::Vec3 &v) {
		mat4 I ;
		I.identity();
		if(v.x == 0 && v.y == 0 && v.z == 0) {
			// Assuming that 0 scale was sent in by mistake
			return I;
		} else {
			mat4 S = scale(I, vec3(v.x, v.y, v.z));
			return S;
		}
	};

inline void  storeRestPose(vector<matrix4>& pose0)
{
	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		int p = limbParents[j];
		mat4 P ;
		P.identity();
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
}
inline void  storeDefaultPose(vector<matrix4>& pose0)
{
	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		int p = limbParents[j];
		mat4 P ;
		P.identity();
		if (p != -1) {
			P = pose0[p];
		}
		ofbx::RotationOrder ro = limb->getRotationOrder();
		//mat4 R = toR(limb->getLocalRotation(), ro);
		mat4 T = toT(limb->getLocalTranslation());
		mat4 Roff = toR(limb->getRotationOffset(), ro);
		mat4 Rp = toR(limb->getRotationPivot(), ro);
		mat4 Rpre = toR(limb->getPreRotation(), ro);
		mat4 Rpost = toR(limb->getPostRotation(), ro);
		mat4 Soff = toS(limb->getScalingOffset());
		mat4 Sp = toS(limb->getScalingPivot());
		//mat4 S = toS(limb->getLocalScaling());

		mat4 E = P *  T* Roff * Rp * Rpre *  inverse(Rpost) * inverse(Rp) * Soff * Sp *  inverse(Sp);
		pose0.push_back(E);
	}
}

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
	vector<double> w;
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
vector<bool> mesh_prepared;




void prepareMesh(int k)
{
	BEGIN_TIMER(removeDuplicated);
	const ofbx::Mesh *mesh = scene->getMesh(k);
	const ofbx::Geometry *geom = mesh->getGeometry();
	int vertex_count = geom->getVertexCount();

	myMeshes[k].verts.resize(vertex_count); // will be overwritten in saveGeom()/saveAllInformation()
	auto &verts = myMeshes[k].verts;
	const ofbx::Vec3* vertices = geom->getVertices();
	bool has_normals = geom->getNormals() != nullptr;
	bool has_uvs = geom->getUVs() != nullptr;
	for (int i = 0; i < vertex_count; ++i) {
		// uses hash. (pos+normal+texcoord) 
		const ofbx::Vec3 &v = vertices[i];
		verts[i].p.x = v.x ;
		verts[i].p.y = v.y;
		verts[i].p.z = v.z;
		if (has_normals)
		{
			const ofbx::Vec3* normals = geom->getNormals();
			const auto& n=normals[i];
			verts[i].p+=vector3(n.x, n.y, n.z)*1e6;
		}
		if (has_uvs)
		{
			const ofbx::Vec2* uvs = geom->getUVs();
			const auto& n=uvs[i];
			verts[i].p+=vector3(n.x, n.y, n.x*7.0-n.y)*1e5;
		}
	}
#ifdef USE_HASH
	std::vector< std::vector<int> > hash;
	hash.resize(100);
#endif
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

#ifdef USE_HASH
		int hh=hash100(verts[i].p);
		auto& hash_hh=hash[hh];
		//printf("v %d hh %d %d\n", i, hh, hash_hh.size());
		for(int jj = 0; jj < hash_hh.size(); ++jj) {
			int j=hash_hh[jj];
			//printf("%d %d :", jj, j);
			if(verts[i] == verts[vertsUnique[j]]) {
				// Found duplicate in vertsUnique
				duplj = j;
				break;
			}
		}
		if(duplj == -1) {
			// No duplicate was found, so create a new entry
			vertsUniqueMap[i] = vertsUnique.size();
			hash_hh.push_back(vertsUnique.size());
			//printf("hash %d %d %d\n", hh, vertsUnique.size(), hash_hh.size());
			vertsUnique.push_back(i);
		} else {
			// Duplicate found
			vertsUniqueMap[i] = duplj;
		}
#else
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
#endif
	}
	printf("%d, vertex count: %d -> %d\n", k, vertex_count, (int)vertsUnique.size());

	END_TIMER2(removeDuplicated);
	BEGIN_TIMER(skin);
	{
	// Populate attachment info
		int cluster_count = 0;
		int max_cluster_count = 0;
		int i=k;
		const ofbx::Mesh *mesh = scene->getMesh(i);
		const ofbx::Geometry *geom = mesh->getGeometry();
		const ofbx::Skin *skin = geom->getSkin();
		myMeshes[i].maxInfluences = 0;
		auto &verts = myMeshes[i].verts;
		auto& verts_map = myMeshes[i].vertsUniqueMap;
		if (skin == nullptr) {
			mesh_prepared[k]=true;
			return;
		}
		cluster_count = skin->getClusterCount();
		if (cluster_count > max_cluster_count) {
			max_cluster_count = cluster_count;
		}
		for(int j = 0; j < cluster_count; ++j) {
			const ofbx::Cluster *cluster = skin->getCluster(j);
			int indices_count = cluster->getIndicesCount();
			int weights_count = cluster->getWeightsCount();
			RANGE_ASSERT(indices_count == weights_count);
			if(weights_count > 0) {
				const int *indices = cluster->getIndices();
				const double *weights = cluster->getWeights();
				const ofbx::Object* limb = cluster->getLink();
				//RANGE_ASSERT(limbMap.find(limb) != limbMap.end());
				RANGE_ASSERT(limb->limbIndex!=-1);
				// Store the clusters and their corrseonding indices for skeleton extraction later
				//clusters_index.push_back(limbMap[limb]);
				clusters_index.push_back(limb->limbIndex);
				clusters.push_back(cluster);
				for(int k = 0; k < weights_count; ++k) {
					int index = indices[k];
					double weight = weights[k];
					RANGE_ASSERT(index < geom->getVertexCount());

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

	mesh_prepared[k]=true;
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
		default: RANGE_ASSERT(false); break;
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

//#define FBX_DEBUG
//void traverseLimbs(const ofbx::Object *limb, vector<const ofbx::Object*> &limbVec, map<const ofbx::Object*, int> &limbMap, vector<int> &limbParents)
void traverseLimbs(const ofbx::Object *limb, vector<const ofbx::Object*> &limbVec, vector<int> &limbParents)
{
	if(limb->getType() != ofbx::Object::Type::LIMB_NODE) {
		return;
	}
	
	limbVec.push_back(limb);
#ifdef FBX_DEBUG
	printf("limbVec %s :  %d %d\n",limb->name, limbVec.size()-1, limbMap.size()); 
#endif
	limb->limbIndex=limbVec.size()-1;
	//limbMap[limb] = limbMap.size();
	const ofbx::Object *parent = limb->getParent();
	if(parent != nullptr && parent->getType() == ofbx::Object::Type::LIMB_NODE) {
#ifdef FBX_DEBUG
		printf("parents %d %s -> %d %s\n", limbParents.size(), limb->name, limbMap[parent], parent->name);
#endif
		//limbParents.push_back(limbMap[parent]);
		limbParents.push_back(parent->limbIndex);
	} else {
		limbParents.push_back(-1);
	}
	
	int i = 0;
	while(const ofbx::Object* child = limb->resolveObjectLink(i)) {
		traverseLimbs(child, limbVec, limbParents);
		++i;
	}
}

bool getAnim(const ofbx::IScene *scene, int ilimb, vectorn& keyTime,  matrixn& traj)
{
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
	if(g_nFrames==0)
	{
		keyTime.setSize(0);
		return false;
	}
	
	
	vectorn rotKeyTime[3];
	vectorn transKeyTime[3];
	vectorn rotations[3];
	vectorn positions[3];
	{
		int j=ilimb;
		Msg::verify(j<limbVec.size(), "getAnim1");
		//const ofbx::Cluster *cluster = clusters[j];
		const ofbx::Object *limb = limbVec[j];
		// limb has one or more children that are curve nodes.
		vector3N scales; // not sure
		int i = 0;
		int key_count = 0; // key count for this limb
		while(const ofbx::Object* child = limb->resolveObjectLink(i)) {
			if(child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
				// ignoring scale node
				if(strcmp(child->name, "S") == 0) 
				{
					++i;
					continue;
				}

				const ofbx::AnimationCurveNode* node = dynamic_cast<const ofbx::AnimationCurveNode*>(child);
				if(!node)
				{
					++i;
					continue;
				}
				const ofbx::AnimationCurve* curveX = node->getCurve(0);
				const ofbx::AnimationCurve* curveY = node->getCurve(1);
				const ofbx::AnimationCurve* curveZ = node->getCurve(2);

				if (!curveX)
				{
					++i;
					continue;
				}
				ASSERT(curveX);
				ASSERT(curveY);
				ASSERT(curveZ);
				const long long* key_time;
				if (curveX != nullptr) {
					key_count = curveX->getKeyCount();
					key_time = curveX->getKeyTime();
					const float* xvals = xvals = curveX->getKeyValue();
					if(strcmp(child->name,"R")==0)
						saveKey(key_count,  rotations[0], rotKeyTime[0], xvals, key_time);
					else if(strcmp(child->name,"T")==0)
						saveKey(key_count,  positions[0], transKeyTime[0], xvals, key_time);
					else {
						ASSERT(false);
					}
				}
				if (curveY != nullptr) {
					key_count = curveY->getKeyCount();
					key_time = curveY->getKeyTime();
					const float* yvals = yvals = curveY->getKeyValue();
					if(strcmp(child->name,"R")==0)
						saveKey(key_count,  rotations[1], rotKeyTime[1], yvals, key_time);
					else if (strcmp(child->name,"T")==0)
						saveKey(key_count,  positions[1], transKeyTime[1], yvals, key_time);
					else {
						ASSERT(false);
					}
				}
				if (curveZ != nullptr) {
					key_count = curveZ->getKeyCount();
					key_time = curveZ->getKeyTime();
					const float* zvals = curveZ->getKeyValue();
					if(strcmp(child->name,"R")==0)
						saveKey(key_count, rotations[2], rotKeyTime[2], zvals, key_time);
					else if(strcmp(child->name,"T")==0)
						saveKey(key_count,  positions[2], transKeyTime[2], zvals, key_time);
					else {
						ASSERT(false);
					}
				}
			}
			++i;
		}
		//	Rs[j].push_back(toR(rotations[k], limb->getRotationOrder()));
	
		// https://help.autodesk.com/view/FBX/2017/ENU/?guid=__files_GUID_10CDD63C_79C1_4F2D_BB28_AD2BE65A02ED_htm
		// WorldTransform = ParentWorldTransform * T * Roff * Rp * Rpre * R * Rpost^-1 * Rp^-1 * Soff * Sp * S * Sp^-1
		key_count=g_nFrames;
		keyTime.setSize(key_count);

		bool hasT=transKeyTime[0].size()>2 ;
		if (transKeyTime[0].size()==2 && 
				!MyVertex::vec3Eq( sampleCurve3D(transKeyTime, positions, 0), sampleCurve3D(transKeyTime, positions, g_T)))
			hasT=true;

		bool useFreeJoint=hasT || ilimb==0; // free joints
#ifdef VERBOSE
		printf("limb%d: %d %d  :  %d %d %d     %d %d %d ",
				ilimb, 
				hasT,
				rotKeyTime[0].size(), 
				rotKeyTime[1].size(), 
				rotKeyTime[2].size(), 
				transKeyTime[0].size(), 
				transKeyTime[1].size(), 
				transKeyTime[2].size()); 
		cout <<sampleCurve3D(transKeyTime, positions, g_T) <<endl;
#endif

		if (useFreeJoint)
			traj.resize(key_count, 7);
		else 
			traj.resize(key_count, 4);

		//printf("%d frames\n", g_nFrames);
		for(int k = 0; k < key_count; ++k) {

			double t=sop::map(k, 0, key_count-1, 0, g_T);
			keyTime[k]=t;

			ofbx::RotationOrder ro = limb->getRotationOrder();
			mat4 R = toR(sampleCurve3D(rotKeyTime, rotations, t), ro);
			mat4 T = toT(sampleCurve3D(transKeyTime, positions, t));
#ifdef VERBOSE
			if (rotKeyTime[0].size()>0 && transKeyTime[0].size()>0 )
			{

			printf("sampleRot %f %f %f\n", t, rotKeyTime[0][0], rotKeyTime[0][rotKeyTime[0].size()-1]);
			printf("sampleTrans %f %f %f\n", t, transKeyTime[0][0], transKeyTime[0][transKeyTime[0].size()-1]);
			}else
			{
				printf("sampleRot 0\n");
			}

#endif

			mat4 Roff = toR(limb->getRotationOffset(), ro);
			mat4 Rp = toR(limb->getRotationPivot(), ro);
			mat4 Rpre = toR(limb->getPreRotation(), ro);
			mat4 Rpost = toR(limb->getPostRotation(), ro);
			mat4 Soff = toS(limb->getScalingOffset());
			mat4 Sp = toS(limb->getScalingPivot());
			mat4 S = toS(limb->getLocalScaling());

			mat4 E =  T * Roff * Rp * Rpre * R * inverse(Rpost) * inverse(Rp) * Soff * Sp * S * inverse(Sp);

			transf tf=E;
#ifdef VERBOSE
			cout <<t<<"/"<<g_T<<" "<<tf<<endl;
#endif
			if(useFreeJoint)
				traj.row(k).setTransf(0, tf);
			else 
				traj.row(k).setQuater(0, tf.rotation);
		}
	}
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

bool saveTexturesToMem(const ofbx::IScene* scene, TStrings& texture_names, std::vector<CImage*>& textures)
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
			long data_length = values.end - values.begin;

			_saveTextureToMem(path.c_str(), data_length, values.begin + 4, texture_names, textures);
		}
	}

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

const ofbx::Object* findLimbNode( const ofbx::Object* startNode,std::list<const ofbx::Object*>& nullNodes )
{
	const ofbx::Object* root = nullptr; // root limb
	int i=0;
	while (const ofbx::Object* child = startNode->resolveObjectLink(i)) {

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
			nullNodes.push_back(child);

		++i;
	}
	return root;
}
void resolve_limb_nodes(const ofbx::IScene* scene) {
	// Find the root limb (hips)
	const ofbx::Object* root = nullptr; // root limb
	int i = 0;
	
	std::list<const ofbx::Object*> nullNodes; 
	Msg::verify(scene, "FBX error!");
	const ofbx::Object* startNode=scene->getRoot();									//
	
	while(!root)
	{
		root=findLimbNode(startNode, nullNodes);

		if(!root)
		{
			if(nullNodes.size()>0)
			{
				startNode=nullNodes.front();
				nullNodes.pop_front();
				printf("startnode %p %s\n", startNode, startNode->name);
			}
			else
				break;
		}
	}
	
	if(!root)
	{
		if (nullNodes.size())
		{
			cout<<"???????????????"<<endl;
			exit(0);
		}

		if(!root)
		{
			cout <<"root==nullptr???"<<endl;
			traverseAll(scene->getRoot(),0);
			i=0;
			while (const ofbx::Object* child = scene->getRoot()->resolveObjectLink(i)) {
				cout << i <<":" <<(int)child->getType() <<endl;
				i++;
			}
			return;
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
	traverseLimbs(root, limbVec, limbParents);
	RANGE_ASSERT(limbVec.size() == limbParents.size() );
//	&& limbVec.size() == limbMap.size());
}

void getModelNames(const ofbx::IScene* scene, std::vector<string>& names){
	std::list<const ofbx::Object*> nullNodes; 
	const ofbx::Object* startNode=scene->getRoot();									//
	
	const ofbx::Object* root = nullptr; // root limb
	while(!root)
	{
		root=findLimbNode(startNode, nullNodes);
		if(!root)
		{
			if(nullNodes.size()>0)
			{
				for(auto i=nullNodes.begin();i!=nullNodes.end(); i++)
					names.push_back((*i)->name);

				startNode=nullNodes.front();
				nullNodes.pop_front();
			}
			else
				break;
		}
	}

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
	
	int mesh_count = scene->getMeshCount();
	
	cout << "=== remove duplicated vertices ===" << endl;

	// annotate duplicated vertices
	myMeshes.resize(mesh_count);
	mesh_prepared.resize(mesh_count);
	for (int k = 0; k < mesh_count; ++k) {
		mesh_prepared[k]=false;
		//prepareMesh(k);
	}




	BEGIN_TIMER(skin);


	// Get sizes
	RANGE_ASSERT(mesh_count == myMeshes.size());


	// prepare anim
	BEGIN_TIMER(prepAnim);
	{
		for(int j=0; j<limbVec.size(); j++)
		{
			//const ofbx::Cluster *cluster = clusters[j];
			const ofbx::Object *limb = limbVec[j];
			int i = 0;
			int key_count = 0; // key count for this limb
			while(const ofbx::Object* child = limb->resolveObjectLink(i)) {
				if(child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
					// ignoring scale node
					if(strcmp(child->name, "S") == 0) 
					{
						++i;
						continue;
					}

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
					if(key_count>g_nFrames)
					{
						g_T=KEYTIMECONV(key_time[key_count-1]);
						g_frameTime=KEYTIMECONV(key_time[key_count-1]-key_time[key_count-2]);
						g_nFrames=key_count;
					}
						//printf("gft: %d %f %d\n",i,  g_frameTime, g_nFrames);					}
				}
				++i;
			}
		}
	}
	END_TIMER2(prepAnim);
	return 0;
}
void saveAll(){

	// Parse and save
	saveTexture(scene);

}

void cleanup()
{
	// Delete data
	delete [] content;
	
	cout << "done" << endl;
}
void saveAll(TStrings& texture_names, std::vector<CImage*>& textures){

	// Parse and save
	saveTexturesToMem(scene, texture_names, textures);

}
};

#include "FBXimporter.h"


int FBXimporter::getMaterialCount(int imesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	return mesh->getMaterialCount();
}
vector3 FBXimporter::getDiffuseColor(int imesh, int imat)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	auto c=mesh->getMaterial(imat)->getDiffuseColor();
	return vector3(c.r, c.g, c.b);
}
TStrings FBXimporter::getMaterialPropertyTypes(int imesh, int imat)
{
	TStrings out;
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	const ofbx::Material* mat=mesh->getMaterial(imat);
	const ofbx::IElement* props = find_child(mat->element, "Properties70");
	const ofbx::IElement* prop = props->getFirstChild();
	while (prop )
	{
		if (!prop->getFirstProperty() 
				|| prop->getFirstProperty()->getType() != ofbx::IElementProperty::Type::STRING) 
		{
			prop=prop->getSibling();
			continue;
		}

		auto* pp=prop->getFirstProperty();
		char tmp[32];
		pp->getValue().toString(tmp);
		if(tmp[0]!=0) out.pushBack(TString(tmp));
		prop=prop->getSibling();
	}
	return out;
}
vectorn FBXimporter::getMaterialProperty(int imesh, int imat, const char* property_type)
{
	vectorn out;
	out.reserve(4);
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	const ofbx::Material* mat=mesh->getMaterial(imat);
	ofbx::IElement* prop=find_property(mat->element, property_type);
	if(!prop) return out;
	auto* pp=prop->getFirstProperty();
	while(pp)
	{
		if(pp->getType()==ofbx::IElementProperty::Type::DOUBLE) 
			out.pushBack(pp->getValue().toDouble());
		pp=pp->getNext();
	}

	return out;
}

vector3 FBXimporter::getSpecularColor(int imesh, int imat)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	auto c=mesh->getMaterial(imat)->getSpecularColor();
	return vector3(c.r, c.g, c.b);
}
std::string FBXimporter::getDiffuseTexture(int imesh, int imat)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	char c[500];
	c[0]=0;
	auto* t=mesh->getMaterial(imat)->getTexture(ofbx::Texture::DIFFUSE);
	if(t)
		t->getFileName().toString<500>(c);
	return c;
}
std::string FBXimporter::getAllTextureNames()
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	return fbx_info.TEXTURENAME;
}

FBXimporter::FBXimporter(const char* filename)
{
	_FBXimport* temp=new _FBXimport();
	BEGIN_TIMER(FBXtotal);
	temp->FBXmain1(filename);
	END_TIMER2(FBXtotal);
	_data=(void*)temp;
}
void FBXimporter::saveTextures()
{
	_FBXimport* temp=(_FBXimport*)_data;
	temp->saveAll();
}
void FBXimporter::saveTexturesToMemoryBuffer()
{
	BEGIN_TIMER(FBXtexture);
	_FBXimport* temp=(_FBXimport*)_data;
	temp->saveAll(texture_names, textures);
	END_TIMER2(FBXtexture);
}


int FBXimporter::getMeshCount()
{
	_FBXimport* fbx_info=(_FBXimport*)_data;
	if(!fbx_info->scene) return 0;
	return fbx_info->scene->getMeshCount();
}

matrix4 FBXimporter::getMeshCurrPose(int imesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	matrix4 transform_matrix = get_T_matrix(scene, mesh->name);
	return transform_matrix;
}
TStrings FBXimporter::getModelNames()
{

	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	std::vector<string> names;
	fbx_info.getModelNames( scene,  names);

	TStrings out;
	out.resize(names.size());
	for (int i=0; i<out.size(); i++)
		out.set(i, names[i].c_str());
	return out;
}

matrix4 FBXimporter::getModelCurrPose(const char* node_name)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	matrix4 transform_matrix = get_T_matrix(scene, node_name);
	return transform_matrix;
}
void FBXimporter::clearBindPose()
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;

	for(int ilimb=0; ilimb<fbx_info.limbParents.size(); ilimb++)
	{
		auto*limb = (ofbx::LimbNodeImpl*) fbx_info.limbVec[ilimb];
		limb->has_bindpose=false;
	}

}
int FBXimporter::countBindPoses(int imesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	ofbx::Pose*  pose=(ofbx::Pose*)mesh->getPose();
	if (!pose)
		return 0;

	return pose->updateLimbBindPoses();
}
bool FBXimporter::hasBindPose(int ilimb)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
		RANGE_ASSERT(ilimb<fbx_info.limbVec.size());
	const auto*limb = (const ofbx::LimbNodeImpl*) fbx_info.limbVec[ilimb];
	matrix4 m;
	if(limb->has_bindpose)
		return true;
	return false;
}
matrix4 FBXimporter::getBindPose(int ilimb)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
		RANGE_ASSERT(ilimb<fbx_info.limbVec.size());
	const auto*limb = (const ofbx::LimbNodeImpl*) fbx_info.limbVec[ilimb];
	matrix4 m;
	if(limb->has_bindpose)
	{
		ofbx::Matrix mm=limb->_bindpose;
		m.transpose(*((matrix4*)(&mm)));
	}
	else
		m.identity();
	return m;
}
std::string FBXimporter::getMeshName(int imesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	return mesh->name;
}

#include "../../motion/Mesh.h"
std::string FBXimporter::getMesh(int imesh, OBJloader::Mesh & mymesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);


	if(!fbx_info.mesh_prepared[imesh])
		fbx_info.prepareMesh(imesh);

	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	const ofbx::Geometry *geom = mesh->getGeometry();
	int vertex_count = geom->getVertexCount();
	int index_count = geom->getIndexCount();
	
	//Get the transform matrix for this mesh from original pose to binding pose
	//matrix4 transform_matrix = get_T_matrix(scene, mesh->name);

	const ofbx::Vec3* vertices = geom->getVertices();

	const auto &vertsUnique = fbx_info.myMeshes[imesh].vertsUnique;
	const auto &vertsUniqueMap = fbx_info.myMeshes[imesh].vertsUniqueMap;

	mymesh.resize(vertsUnique.size(), index_count/3);
	int ni=vertsUnique.size();
	for (int i = 0; i < ni; ++i) {
		const ofbx::Vec3 &v = vertices[vertsUnique[i]];
		//mymesh.getVertex(i)=transform_matrix * vector3(v.x, v.y, v.z);
		mymesh.getVertex(i)= vector3(v.x, v.y, v.z);
	}

	// Assume that we are working with a triangulated array, so
	// there is no index/element array.
	RANGE_ASSERT(vertex_count == index_count);
	int tri_count = vertex_count/3;
	bool has_normals = geom->getNormals() != nullptr;
	bool has_uvs = geom->getUVs() != nullptr;
	for(int i = 0; i < tri_count; ++i) {
		// This is index-0. Convert to index-1 before exporting to OBJ.
		auto& f=mymesh.getFace(i);
		int v1=vertsUniqueMap[3*i];
		int v2=vertsUniqueMap[3*i+1];
		int v3=vertsUniqueMap[3*i+2];
		f.setIndex(v1, v2,v3);
		if(has_normals) f.setIndex(v1, v2, v3,OBJloader::Buffer::NORMAL);
		if(has_uvs) f.setIndex(v1, v2, v3,OBJloader::Buffer::TEXCOORD);
	}

	if(has_normals) {
		const ofbx::Vec3* normals = geom->getNormals();
		// This will fail if ofbx::LoadFlags::TRIANGULATE is not used
		RANGE_ASSERT(geom->getIndexCount() == vertex_count);
		mymesh.resizeNormalBuffer(ni);
		for(int i = 0; i < ni; ++i) {
			const ofbx::Vec3 &n = normals[vertsUnique[i]];
			auto& nn=mymesh.getNormal(i);
			nn.x = n.x;
			nn.y = n.y;
			nn.z = n.z;
		}
	}

	if(has_uvs) {
		const ofbx::Vec2 *uvs = geom->getUVs();
		// This will fail if ofbx::LoadFlags::TRIANGULATE is not used
		RANGE_ASSERT(geom->getIndexCount() == vertex_count);
		mymesh.resizeUVbuffer(ni);
		for (int i = 0; i < ni; ++i) {
			const ofbx::Vec2 &uv = uvs[vertsUnique[i]];
			auto& t=mymesh.getTexCoord(i);
			t(0) = uv.x;
			t(1)= 1.0-uv.y; // flip y for consistency with ogre-created materials
		}
	}
	return mesh->name;
}

void FBXimporter::getSkinnedMesh(int imesh, SkinnedMeshFromVertexInfo & skinned_mesh)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::Mesh *mesh = scene->getMesh(imesh);
	const ofbx::Geometry *geom = mesh->getGeometry();
	int vertex_count = geom->getVertexCount();
	int index_count = geom->getIndexCount();

	const ofbx::Vec3* vertices = geom->getVertices();


	auto& mymesh=fbx_info.myMeshes[imesh];
	const auto &verts = mymesh.verts;
	const auto &vertsUnique = mymesh.vertsUnique;
	const auto &vertsUniqueMap = mymesh.vertsUniqueMap;

	// numVertex after removing redundant vertices
	skinned_mesh.resize(vertsUnique.size());

	for(int vi=0; vi<vertsUnique.size(); vi++)
	{
		int i=vertsUnique[vi];
		const _FBXimport::MyVertex &v = verts[i];
		int influences = v.w.size();
		RANGE_ASSERT(influences == v.i.size());

		auto& sv=skinned_mesh.vertices[vi];
		sv.treeIndices=intvectornView(&(v.i[0]), v.i.size())+1;
		sv.weights=vectornView(&v.w[0], v.w.size());
	}
}
intvectornView FBXimporter::parentIndices()
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	return intvectornView(&fbx_info.limbParents[0], fbx_info.limbParents.size(), 1);
}
void FBXimporter::getRestPose(matrixn& pose)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	pose.resize(fbx_info.limbParents.size(), 7);

	vector<matrix4> pose0;
	fbx_info.storeRestPose(pose0);

	transf tf;
	for (int i=0; i<pose0.size(); i++)
	{
		tf=pose0[i];
		pose.row(i).setTransf(0, tf);
	}
}
void FBXimporter::getRestPose(vector3N& jointpos, quaterN& jointori)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	jointpos.resize(fbx_info.limbParents.size());
	jointori.resize(fbx_info.limbParents.size());

	vector<matrix4> pose0;
	fbx_info.storeRestPose(pose0);

	transf tf;
	for (int i=0; i<pose0.size(); i++)
	{
		tf=pose0[i];
		jointpos[i]=tf.translation;
		jointori[i]=tf.rotation;
	}
}
void FBXimporter::getDefaultPose(vector3N& jointpos, quaterN& jointori)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	jointpos.resize(fbx_info.limbParents.size());
	jointori.resize(fbx_info.limbParents.size());

	vector<matrix4> pose0;
	fbx_info.storeDefaultPose(pose0);

	transf tf;
	for (int i=0; i<pose0.size(); i++)
	{
		tf=pose0[i];
		jointpos[i]=tf.translation;
		jointori[i]=tf.rotation;
	}
}


void FBXimporter::getLocalPose(matrixn& localpose)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	transf tf;
	auto& limbVec=fbx_info.limbVec;
	localpose.resize(fbx_info.limbParents.size(), 7);
	for (int j = 0; j < limbVec.size(); ++j) {
		const ofbx::Object* limb = limbVec[j];
		ofbx::RotationOrder ro = limb->getRotationOrder();
		//ofbx::Vec3 R = limb->getLocalRotation();
		//ofbx::Vec3 T = limb->getLocalTranslation();
		matrix4 Rpost = _FBXimport::toR(limb->getPostRotation(), ro);
		matrix4 Rp = _FBXimport::toR(limb->getRotationPivot(), ro);
		matrix4 Soff = _FBXimport::toS(limb->getScalingOffset());
		matrix4 Sp = _FBXimport::toS(limb->getScalingPivot());
		matrix4 S = _FBXimport::toS(limb->getLocalScaling());
		matrix4 E = inverse(Rpost) * inverse(Rp) * Soff * Sp * S * inverse(Sp);
		transf T=E;
		//matrix4 mR = _FBXimport::toR(limb->getLocalRotation(), ro);
		localpose.row(j).setVec3(0, T.translation);
		localpose.row(j).setQuater(3, T.rotation);
	}
}
void FBXimporter::getRestPoseScale(vectorn& scale) // only for checking purposes
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);

	vector<matrix4> pose0;
	fbx_info.storeRestPose(pose0);
	scale.resize(pose0.size());
	for (int i=0; i<pose0.size(); i++)
	{
		scale[i]=
			pose0[i].getColumn(0).length()/3.0 +
			pose0[i].getColumn(1).length()/3.0 +
			pose0[i].getColumn(2).length()/3.0 ;
	}
}

void FBXimporter::getBoneNames(TStrings& out)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	out.resize(fbx_info.limbParents.size());
	RANGE_ASSERT(out.size()==fbx_info.limbVec.size());
	for (int i=0; i<out.size(); i++)
	{
		out.set(i, fbx_info.limbVec[i]->name);
	}
}

void FBXimporter::getAnim(int ilimb, vectorn& keytime, matrixn& traj)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	fbx_info.getAnim(scene, ilimb, keytime,  traj);
}

MeshMerger::MeshMerger(int ninputMesh)
{
	_inputs.resize(ninputMesh, NULL);
	_vertsUniqueMap.resize(ninputMesh);
}
MeshMerger::~MeshMerger()
{
}

void MeshMerger::mergeMeshes(OBJloader::Mesh& outputmesh)
{
	_vertsUnique.setSize(0,0);
	std::vector< std::vector<int> > hash;
	hash.resize(100);
	int tot_vertex_count=0;
	int numFace=0;
	vector<_FBXimport::MyVertex> vertsUnique;
	for(int imesh=0; imesh<_inputs.size(); imesh++)
	{
		const auto& mesh=*_inputs[imesh];
		int vertex_count=mesh.numVertex();
		tot_vertex_count+=vertex_count;
		bool has_normals=mesh.numNormal() && mesh.numNormal()==mesh.numVertex(); 
		auto &vertsUniqueMap=_vertsUniqueMap[imesh];
		vertsUniqueMap.resize(vertex_count);
		vertsUniqueMap.setAllValue(-1);
		vector<_FBXimport::MyVertex> verts;
		verts.resize(vertex_count);
		for (int i = 0; i < vertex_count; ++i) {
			// uses hash. (pos+normal) 
			auto v=mesh.getVertex(i);
			verts[i].p.x = v.x ;
			verts[i].p.y = v.y;
			verts[i].p.z = v.z;
			if (has_normals)
				verts[i].p+=mesh.getNormal(i)*1e6;
		}
#if 0
		// Find duplicates (slow O(n^2))
		// E.g.,
		//                      0 1 2 3 4 5 6 7
		//    verts          = [A B a C D a d E] <- duplicates in lower case
		//    vertsUnique    = [0 1 3 4 7]
		//    vertsUniqueMap = [0 1 0 2 3 0 3 4] <- indexes into vertsUnique
		for(int i = 0; i < vertex_count; ++i) {
			int duplj = -1;

			int hh=hash100(verts[i].p);
			auto& hash_hh=hash[hh];
			//printf("v %d hh %d %d\n", i, hh, hash_hh.size());
			for(int jj = 0; jj < hash_hh.size(); ++jj) {
				int j=hash_hh[jj];
				//printf("%d %d :", jj, j);
				if(verts[i] == vertsUnique[j]) {
					// Found duplicate in vertsUnique
					duplj = j;
					break;
				}
			}
			if(duplj == -1) {
				// No duplicate was found, so create a new entry
				vertsUniqueMap[i] = vertsUnique.size();
				hash_hh.push_back(vertsUnique.size());
				//printf("hash %d %d %d\n", hh, vertsUnique.size(), hash_hh.size());
				vertsUnique.push_back(verts[i]);
				_vertsUnique.pushBack(intvectorn(2, imesh, i));
			} else {
				// Duplicate found
				vertsUniqueMap[i] = duplj;
			}
		}
#else
		int starti=_vertsUnique.rows();
		//vertsUnique.resize(vertsUnique.size()+vertex_count);
		_vertsUnique.resize(_vertsUnique.rows()+vertex_count,2);
		for(int i=0; i<vertex_count; i++)
		{
			//vertsUnique[starti+i]=verts[i];
			_vertsUnique.row(starti+i)=intvectorn(2, imesh, i);
			vertsUniqueMap[i]=starti+i;
		}
#endif
		numFace+=mesh.numFace();
	}

	if (_inputs[0]->numNormal())
		outputmesh.resize(_vertsUnique.rows(), _vertsUnique.rows(), 0,0, numFace);
	else
		outputmesh.resize(_vertsUnique.rows(), numFace);


	invMap.resize(_inputs.size());
	for(int i=0; i<_inputs.size(); i++)
		invMap[i].resize(_inputs[i]->numVertex());

	for (int i=0; i<outputmesh.numVertex(); i++)
	{
		int imesh=_vertsUnique(i,0);
		int ivertex=_vertsUnique(i,1);
		invMap[imesh][ivertex]=i;

		outputmesh.getVertex(i)=_inputs[imesh]->getVertex(ivertex);
		outputmesh.getNormal(i)=_inputs[imesh]->getNormal(ivertex);
	}
	int cface=0;
	for(int imesh=0; imesh<_inputs.size(); imesh++)
	{
		const auto& meshi=*_inputs[imesh];
		for(int f=0; f<meshi.numFace(); f++)
		{
			OBJloader::Face& outf=outputmesh.getFace(cface+f);
			auto& inf=meshi.getFace(f);
			outf.setIndex(
					invMap[imesh][inf.vertexIndex(0)],
					invMap[imesh][inf.vertexIndex(1)],
					invMap[imesh][inf.vertexIndex(2)],
					OBJloader::Buffer::VERTEX);
			outf.setIndex(                    
					invMap[imesh][inf.normalIndex(0)],
					invMap[imesh][inf.normalIndex(1)],
					invMap[imesh][inf.normalIndex(2)],
					OBJloader::Buffer::NORMAL);
		}
		cface+=meshi.numFace();
	}
	Msg::verify(cface==outputmesh.numFace(), "#face error");
	printf("vertex count: %d -> %d\n", tot_vertex_count, (int)_vertsUnique.rows());
}

#include "../../image/Image.h"
FBXimporter::		~FBXimporter()
{
	_FBXimport* temp=(_FBXimport*)_data;
	temp->cleanup();
	delete temp;
	for(int i=0; i<textures.size(); i++)
		delete textures[i];
}
void _saveTextureToMem(const char* path, long data_length, const unsigned char* ptr, TStrings& texture_names, std::vector<CImage*>& textures)
{
	TString fn=path;
	TString r5=fn.left(-4).right(6).toUpper();
	if(r5=="NORMAL" || r5=="_GLOSS" || r5=="ECULAR")
	{
		printf("skipping loading %s\n", fn.ptr());
		return;
	}
	texture_names.pushBack(path);
	CImage* image=new CImage();
	image->loadFromMemory(path, data_length, ptr);
	//printf("res!!! %d %d\n", image->GetWidth(), image->GetHeight());
	textures.push_back(image);
}


void FBXimporter::packTextures(BinaryFile& bf)
{
	int version=-2; // use a negative number. later versions: -2, -3, ...
	bf.pack(texture_names);
	for(int i=0; i<texture_names.size(); i++)
	{
		CImage& im=*textures[i];
		bf.packInt(version);
		bf.packInt(im.GetWidth());
		bf.packInt(im.GetHeight());

		for(int j=0; j<im.GetHeight(); j++)
			bf.packArray((void*)(im.GetData()+j*im._stride),im._stride, 1);

		if(im.getOpacityMap())
		{
			bf.packInt(1);
			for(int j=0; j<im.GetHeight(); j++)
				bf.packArray((void*)(im.getOpacityMap()+j*im.GetWidth()),im.GetWidth(), 1);

		}
		else
			bf.packInt(0);
	}
	bf.packInt(0);
}
void FBXimporter::unpackTextures(BinaryFile& bf)
{
	bf.unpack(texture_names);
	textures.resize(texture_names.size());
	for(int i=0; i<texture_names.size(); i++)
	{
		int version=bf.unpackInt();
		int w, h;
		//printf("version %d\n", version);
		if (version>=0)
		{
			w=version;
			version=0;
		}
		else
			w=bf.unpackInt();

		h=bf.unpackInt();
		//printf("res %d %d\n", w, h);

		unsigned char *buffer=new unsigned char[w*h*3];
		int stride=w*3;

		for(int j=0; j<h; j++)
			bf.unpackArray((void*)(buffer+j*stride),stride, 1);

		CImage* im=new CImage();
		if(w>0)
			im->_adoptRawData(w,h,buffer,stride);
		textures[i]=im;
		if(version<0)
		{
			int hasOpacity=bf.unpackInt();
			if(hasOpacity)
			{
				unsigned char *buffer=new unsigned char[w*h];
				int stride=w;

				for(int j=0;j<h; j++)
					bf.unpackArray((void*)(buffer+j*stride),stride, 1);
				if(w>0)
					im->_opacityMap=buffer;
			}
		}
		if(version==-1)
			im->flipY();
	}
	Msg::verify(bf.unpackInt()==0," unpack texture error");
}

int FBXimporter::getAnimCount()
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	return scene->getAnimationStackCount();
}
std::string FBXimporter::getAnimName(int ianim)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::AnimationStack* stack=scene->getAnimationStack(ianim);
	return std::string(stack->name);
}
vectorn FBXimporter::getAnimInfo(int ianim)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::AnimationStack* stack=scene->getAnimationStack(ianim);
	vectorn out(3);
	auto* layer=stack->resolveObjectLink(0);
	int i = 0;
	int key_count = 0; // key count for this limb
	const ofbx::Object* argNode=NULL; 
	while(const ofbx::Object* child = layer->resolveObjectLink(i)) {
		if(child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
			// ignoring scale node
			if(strcmp(child->name, "S") == 0) { ++i; continue; }

			const ofbx::AnimationCurveNode* node = dynamic_cast<const ofbx::AnimationCurveNode*>(child);
			if(!node) { ++i; continue; }
			const ofbx::AnimationCurve* curveX = node->getCurve(0);
			if (!curveX) { ++i; continue; }
			if (curveX->getKeyCount()>key_count)
			{
				key_count = curveX->getKeyCount();
				argNode=child;
			}
		}
		++i;
	}
	out[0]=key_count;
	if(key_count>1)
	{
		const ofbx::AnimationCurveNode* node = dynamic_cast<const ofbx::AnimationCurveNode*>(argNode);
		const long long* key_time;
		const ofbx::AnimationCurve* curveX = node->getCurve(0);
		key_time = curveX->getKeyTime();
		out[1]=KEYTIMECONV(key_time[key_count-1]-key_time[key_count-2]);
		out[2]=KEYTIMECONV(key_time[key_count-1]);
	}
	return out;
}

void FBXimporter::getAnim2(int ianim, int g_nFrames, double g_T, int ilimbnode, vectorn& keyTime, matrixn& traj)
{
	_FBXimport& fbx_info=*((_FBXimport*)_data);
	auto* scene=fbx_info.scene;
	const ofbx::AnimationStack* stack=scene->getAnimationStack(ianim);
	auto* layer=stack->resolveObjectLink(ofbx::Object::Type::ANIMATION_LAYER, NULL, 0);
	//printf("%s %d\n", layer->name, layer->getType());
	int i = 0;
	int key_count = 0; // key count for this limb
	vectorn rotKeyTime[3];
	vectorn transKeyTime[3];
	vectorn rotations[3];
	vectorn positions[3];
	while(const ofbx::Object* child = layer->resolveObjectLink(i)) {
		if(child->getType() == ofbx::Object::Type::ANIMATION_CURVE_NODE) {
			auto* limb=child->getParent();
			if(!limb) { ++i; continue; }
			if(limb->getType()!=ofbx::Object::Type::LIMB_NODE) {++i;continue; }
			if(limb!=fbx_info.limbVec[ilimbnode]) { ++i; continue; }
			// ignoring scale node
			if(strcmp(child->name, "S") == 0) { ++i; continue; }

			const ofbx::AnimationCurveNode* node = dynamic_cast<const ofbx::AnimationCurveNode*>(child);

			if(!node) { ++i; continue; }

			const ofbx::AnimationCurve* curveX = node->getCurve(0);
			const ofbx::AnimationCurve* curveY = node->getCurve(1);
			const ofbx::AnimationCurve* curveZ = node->getCurve(2);

			if (!curveX) { ++i; continue; }

			ASSERT(curveX);
			ASSERT(curveY);
			ASSERT(curveZ);
			const long long* key_time;
			if (curveX != nullptr) {
				key_count = curveX->getKeyCount();
				key_time = curveX->getKeyTime();
				const float* xvals = xvals = curveX->getKeyValue();
				if(strcmp(child->name,"R")==0)
					saveKey(key_count,  rotations[0], rotKeyTime[0], xvals, key_time);
				else if(strcmp(child->name,"T")==0)
					saveKey(key_count,  positions[0], transKeyTime[0], xvals, key_time);
				else {
					ASSERT(false);
				}
			}
			if (curveY != nullptr) {
				key_count = curveY->getKeyCount();
				key_time = curveY->getKeyTime();
				const float* yvals = yvals = curveY->getKeyValue();
				if(strcmp(child->name,"R")==0)
					saveKey(key_count,  rotations[1], rotKeyTime[1], yvals, key_time);
				else if (strcmp(child->name,"T")==0)
					saveKey(key_count,  positions[1], transKeyTime[1], yvals, key_time);
				else {
					ASSERT(false);
				}
			}
			if (curveZ != nullptr) {
				key_count = curveZ->getKeyCount();
				key_time = curveZ->getKeyTime();
				const float* zvals = curveZ->getKeyValue();
				if(strcmp(child->name,"R")==0)
					saveKey(key_count, rotations[2], rotKeyTime[2], zvals, key_time);
				else if(strcmp(child->name,"T")==0)
					saveKey(key_count,  positions[2], transKeyTime[2], zvals, key_time);
				else {
					ASSERT(false);
				}
			}
		}
		++i;
	}

	key_count=g_nFrames;
	double g_frameTime=g_T/double(g_nFrames-1);
	auto& limb=fbx_info.limbVec[ilimbnode];
		keyTime.setSize(key_count);

		bool hasT=transKeyTime[0].size()>2 ;
		if (transKeyTime[0].size()==2 && 
				!_FBXimport::MyVertex::vec3Eq( sampleCurve3D(transKeyTime, positions, 0), sampleCurve3D(transKeyTime, positions, keyTime.back())))
			hasT=true;

		bool useFreeJoint=hasT || ilimbnode==0; // free joints

		if (useFreeJoint)
			traj.resize(key_count, 7);
		else 
			traj.resize(key_count, 4);

		//printf("%d frames\n", g_nFrames);
		for(int k = 0; k < key_count; ++k) {

			double t=sop::map(k, 0, key_count-1, 0, g_T);
			keyTime[k]=t;

			ofbx::RotationOrder ro = limb->getRotationOrder();
			matrix4 R = _FBXimport::toR(sampleCurve3D(rotKeyTime, rotations, t), ro);
			matrix4 T = _FBXimport::toT(sampleCurve3D(transKeyTime, positions, t));

			matrix4 Roff = _FBXimport::toR(limb->getRotationOffset(), ro);
			matrix4 Rp = _FBXimport::toR(limb->getRotationPivot(), ro);
			matrix4 Rpre = _FBXimport::toR(limb->getPreRotation(), ro);
			matrix4 Rpost = _FBXimport::toR(limb->getPostRotation(), ro);
			matrix4 Soff = _FBXimport::toS(limb->getScalingOffset());
			matrix4 Sp = _FBXimport::toS(limb->getScalingPivot());
			matrix4 S = _FBXimport::toS(limb->getLocalScaling());

			matrix4 E =  T * Roff * Rp * Rpre * R * inverse(Rpost) * inverse(Rp) * Soff * Sp * S * inverse(Sp);

			transf tf=E;
			if(useFreeJoint)
				traj.row(k).setTransf(0, tf);
			else 
				traj.row(k).setQuater(0, tf.rotation);
		}

}
