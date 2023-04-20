#ifndef TERRAIN_H
#define TERRAIN_H
#pragma once

#include "../math/tvector.h"
#include "Mesh.h"

class Ray;
struct Region2D
{
    m_real left, top, right, bottom;

    Region2D()
    {
    }
    Region2D( long l, long t, long r, long b )
    {
        left = l;
        top = t;   
        right = r;
        bottom = b;                
    }

	m_real width() const				{ return right-left;}
	m_real height() const				{ return bottom-top;}
	bool isInside(vector2 pt)	const	{ if(pt.x()>=left && pt.y()>=top && pt.x()<right && pt.y()<bottom) return true; return false;}
};

class Raw2Bytes: public _tmat<unsigned short>
{
public:
	Raw2Bytes():_tmat<unsigned short>(){}
	template <class T>
	Raw2Bytes(const T& other):_tmat<unsigned short>(other){}
};
namespace OBJloader
{
	class Terrain : public Mesh
	{
		matrixn mHeight;
		matrixn mMaxHeight;
		vector3 mSize;
		bool _tileAlongZ;
		void _init(Raw2Bytes& image, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ);
	public :
		Terrain(const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ=false);
		Terrain(vectorn const& image1d, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ=false);

		~Terrain(){}

		const matrixn & getHeightMap() const { return mHeight;}
		vector3 getSize() const { return mSize;}
		index2 _lowindex(vector2 x) const;
		index2 _highindex(vector2 x) const;
		m_real maxHeight(Region2D const & r) const;
		m_real maxHeight(vector2 x) const;
		m_real height(vector2 x, vector3& normal) const;
		m_real height(vector2 x) const;
		bool isInsideTerrain(vector2 x) const;
		bool findClosestSurfacePoint(vector3 const& x, vector3& normal, vector3& surfacePoint) const;

		vector3 pick(Ray const& ray, vector3& normal) const;
	};
}




#endif
