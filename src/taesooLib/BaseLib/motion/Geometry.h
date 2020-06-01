#ifndef _Baselib_GEOMETRY_H_
#define _Baselib_GEOMETRY_H_

#include "Mesh.h"
namespace OBJloader
{
	class Element
	{
	public:
		enum {
			// all elements are centered at vector3(0,0,0)
			// do not change the order. you can add more element only at the end of the enum.
			BOX=0,  // uses tf, size.x, y, z
			CYLINDER,  // uses tf, size.x (2*radius), y (height)
			CAPSULE, // uses tf, size.x (2*radius), y (height). total height=height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
			OBJ, // does not use tf, size currently.
			ELLIPSOID, // size.x, y, z is the radius.
			PLANE, // size.x and z are used. the normal is (0,1,0)
			SPHERE, // same as the ELLIPSOID except that size.x,y,and z are identical.
		};

		int elementType;
		vector3 elementSize;
		transf tf; // actual vertices have already this tf applied, so do not double-multiply.
	};

	class Geometry : public OBJloader::Mesh
	{
		public:
		std::vector<Element> elements;
		inline int numElements() const {return elements.size();}
		inline Element const& element(int i) const {return elements[i];}

		// each element corresponds to each faceGroup.
		Geometry(void){}
		Geometry(const Geometry& otherMesh){copyFrom(otherMesh);}

		void convertToOBJ(); // remove geometry information.

		void extractSubMesh(Geometry const& otherMesh, int isubMesh);
		void copyFrom(Geometry const& otherMesh);
		void operator=(Geometry const& otherMesh);
		void operator=(Mesh const& otherMesh);

		void merge(Geometry const& a, Geometry const& b);
		void rigidTransform(transf const& b);
		void rigidTransform(transf const& b, int iFaceGroup);
		void scale(vector3 const& scalef);
		void scale(vector3 const& scalef, int iFaceGroup);
		void scaleElements(vector3 const& scalef);
		void scaleElements(vector3 const& scalef, int iFaceGroup);
		// uses only scale, rotation, and translation components. Shearing components of m are discarded.
		void scaleAndRigidTransform(matrix4 const& m);
		virtual void transform(matrix4 const& b); // never use this. It will produce an error message.
		// export to a widely used file format (.obj)
		// geometry information is stored into the first few commented lines.
		virtual bool saveObj(const char* filename, bool vn, bool vt);
		virtual bool loadObj(const char* filename);

		virtual void pack(BinaryFile& bf);
		virtual void unpack(BinaryFile& bf);

		void initBox(const vector3& size);
		void initCylinder(double radius, double height, int numDivision);
		void initCapsule(double radius, double height);
		void initEllipsoid(const vector3& size); 
		void initSphere(double size) { initEllipsoid(vector3(size,size,size)); elements[0].elementType=Element::SPHERE;}
		void initPlane(double size_x, double size_z); // normal : (0,1,0)
		void classifyTriangles();
		void _updateMeshFromElements();
	};
}
#endif
