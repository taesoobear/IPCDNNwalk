#include "stdafx.h"
#include "Terrain.h"
#include "../math/Operator.h"
#include "../math/OperatorStitch.h"

index2 OBJloader::Terrain::_lowindex(vector2 x) const
{
	index2 out;
	out(0)=int(floor((x(0)/mSize.x)*(mHeight.cols()-1)));
	out(1)=int(floor((x(1)/mSize.z)*(mHeight.rows()-1)));
	return out;
}

index2 OBJloader::Terrain::_highindex(vector2 x) const
{
	index2 out=_lowindex(x);
	out(0)++;
	out(1)++;
	return out;
}
m_real OBJloader::Terrain::maxHeight(Region2D const & r) const
{
	index2 low=_lowindex(vector2(r.left, r.top));
	index2 high=_highindex(vector2(r.bottom, r.right));

	if(high(0)<0) return -FLT_MAX;
	if(high(1)<0) return -FLT_MAX;
	if(low(0)>=mHeight.cols()) return -FLT_MAX;
	if(low(1)>=mHeight.rows()) return -FLT_MAX;

	low(0)=MIN(low(0), 0);
	low(1)=MIN(low(1), 0);

	high(0)=MAX(high(0), mHeight.cols()-1);
	high(1)=MAX(high(1), mHeight.rows()-1);

	m_real maxHeight=-FLT_MAX;
	for(int j=low(1); j<=high(1); j++)
	{
		for(int i=low(0); i<=high(0); i++)
		{
			if(mHeight(j,i)>maxHeight)
				maxHeight=mHeight(j,i);						
		}
	}

	return maxHeight;
}

m_real OBJloader::Terrain::maxHeight(vector2 x) const
{
	index2 low=_lowindex(x);
	low(0)=MAX(0, low(0));
	low(1)=MAX(0, low(1));
	low(0)=MIN(low(0), mMaxHeight.cols()-1);
	low(1)=MIN(low(1), mMaxHeight.rows()-1);
	
	return mMaxHeight(low(1), low(0));
}

#include "intersectionTest.h"

m_real OBJloader::Terrain::height(vector2 x, vector3& normal) const
{
	int numSegX=mHeight.cols()-1;
	int numSegZ=mHeight.rows()-1;
	
	double nx=(x(0)/mSize.x)*double(numSegX);
	double nz=(x(1)/mSize.z)*double(numSegZ);

	int j=int(floor(nx));   // x coord (left)
							// y coord (vertical)
	int i=int(floor(nz));	// z coord (down)

	double dx=nx-floor(nx);
	double dz=nz-floor(nz);

	if(j<0) 
	{
		j=0; dx=0;
	}
	if(j>=numSegX)
	{
		j=numSegX-1; dx=1;
	}

	if(_tileAlongZ) {
		if (i<0){
			while(i<0)
				i+=numSegZ;
		}
		else
		{
			while(i>=numSegZ)
				i-=numSegZ;
		}
		nz=double(i)+dz;
		x(1)=nz/double(numSegZ)*(double)mSize.z;
	}
	else
	{
		if(i<0)
		{
			i=0; dz=0;
		}
		if(i>=numSegZ)
		{
			i=numSegZ-1; dz=1;
		}
	}


	int faceIndex;
	vector3 v0, v1, v2;

	/*printf("%f, %f : %f %f %f\n", x(0), x(1), 
			i*m_real(mSize.z)/m_real(numSegZ), 
			mHeight(i,j),
			j*m_real(mSize.x)/m_real(numSegX)); 
			*/
	if(dx>dz)
	{
		// upper triangle ( (j,i), (j+1, i+1), (j+1, i) )
		faceIndex=i*numSegX+j;
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i, j);
		v1.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v1.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v1.y=mHeight(i+1, j+1);
		v2.x=v1.x;
		v2.z=v0.z;
		v2.y=mHeight(i, j+1);
	}
	else
	{
		// lower triangle ( (j,i), (j, i+1), (j+1, i+1) )
		faceIndex=i*numSegX+j+numSegX*numSegZ;
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i,j);
		v2.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v2.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v2.y=mHeight(i+1, j+1);
		v1.x=v0.x;
		v1.z=v2.z;
		v1.y=mHeight(i+1,j);
	}

	//const Face& f=getFace(faceIndex);

	//Plane p(getVertex(f.vi(0)), getVertex(f.vi(1)), getVertex(f.vi(2)));
	//return v0.y;
	Plane p(v0, v1, v2);

	Ray r(vector3(x(0), 1000, x(1)), vector3(0,-1,0));
	std::pair<bool, m_real> res=r.intersects(p);

	ASSERT(res.first);
	normal=p.normal;
	return r.getPoint(res.second).y;
}
bool OBJloader::Terrain::findClosestSurfacePoint(vector3 const& x3, vector3& normal, vector3& surfacePoint) const
{
	vector2 x(x3.x, x3.z);

	int numSegX=mHeight.cols()-1;
	int numSegZ=mHeight.rows()-1;
	
	double nx=(x(0)/mSize.x)*double(numSegX);
	double nz=(x(1)/mSize.z)*double(numSegZ);

	int j=int(floor(nx));   // x coord (left)
							// y coord (vertical)
	int i=int(floor(nz));	// z coord (down)

	double dx=nx-floor(nx);
	double dz=nz-floor(nz);

	if(j<0) 
	{
		j=0; dx=0;
	}
	if(j>=numSegX)
	{
		j=numSegX-1; dx=1;
	}

	if(_tileAlongZ) {
		if (i<0){
			while(i<0)
				i+=numSegZ;
		}
		else
		{
			while(i>=numSegZ)
				i-=numSegZ;
		}
		nz=double(i)+dz;
		x(1)=nz/double(numSegZ)*(double)mSize.z;
	}
	else
	{
		if(i<0)
		{
			i=0; dz=0;
		}
		if(i>=numSegZ)
		{
			i=numSegZ-1; dz=1;
		}
	}


	int faceIndex;
	vector3 v0, v1, v2;

	/*printf("%f, %f : %f %f %f\n", x(0), x(1), 
			i*m_real(mSize.z)/m_real(numSegZ), 
			mHeight(i,j),
			j*m_real(mSize.x)/m_real(numSegX)); 
			*/
	if(dx>dz)
	{
		// upper triangle ( (j,i), (j+1, i+1), (j+1, i) )
		faceIndex=i*numSegX+j;
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i, j);
		v1.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v1.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v1.y=mHeight(i+1, j+1);
		v2.x=v1.x;
		v2.z=v0.z;
		v2.y=mHeight(i, j+1);
	}
	else
	{
		// lower triangle ( (j,i), (j, i+1), (j+1, i+1) )
		faceIndex=i*numSegX+j+numSegX*numSegZ;
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i,j);
		v2.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v2.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v2.y=mHeight(i+1, j+1);
		v1.x=v0.x;
		v1.z=v2.z;
		v1.y=mHeight(i+1,j);
	}

	//const Face& f=getFace(faceIndex);

	//Plane p(getVertex(f.vi(0)), getVertex(f.vi(1)), getVertex(f.vi(2)));
	//return v0.y;
	Plane p(v0, v1, v2);

	Ray r(vector3(x(0), 1000, x(1)), vector3(0,-1,0));
	std::pair<bool, m_real> res=r.intersects(p);

	ASSERT(res.first);
	normal=p.normal;

	Ray r2(vector3(x(0), x3.y, x(1)), normal);
	std::pair<bool, m_real> res2=r2.intersects(p);
	surfacePoint=r2.getPoint(res2.second);
	surfacePoint+=vector3(x3.x-x(0), 0, x3.z-x(1));
	
	return r.getPoint(res.second).y> x3.y;
}
bool OBJloader::Terrain::isInsideTerrain(vector2 x) const
{
	int numSegX=mHeight.cols()-1;
	int numSegZ=mHeight.rows()-1;
	
	double nx=(x(0)/mSize.x)*double(numSegX);
	double nz=(x(1)/mSize.z)*double(numSegZ);

	int j=int(floor(nx));   // x coord (left)
							// y coord (vertical)
	int i=int(floor(nz));	// z coord (down)

	double dx=nx-floor(nx);
	double dz=nz-floor(nz);

	if(j<0) 
	{
		return false;
	}
	if(i<0)
	{
		return false;
	}
	if(j>=numSegX)
	{
		return false;
	}

	if(i>=numSegZ)
	{
		return false;
	}
	return true;
}
m_real OBJloader::Terrain::height(vector2 x) const
{
	int numSegX=mHeight.cols()-1;
	int numSegZ=mHeight.rows()-1;
	
	double nx=(x(0)/mSize.x)*double(numSegX);
	double nz=(x(1)/mSize.z)*double(numSegZ);

	int j=int(floor(nx));   // x coord (left)
							// y coord (vertical)
	int i=int(floor(nz));	// z coord (down)

	double dx=nx-floor(nx);
	double dz=nz-floor(nz);

	if(j<0) 
	{
		return 0.0;
	}
	if(j>=numSegX)
	{
		return 0.0;
	}
	if(_tileAlongZ) {
		while(i<0)
			i+=numSegZ;
		while(i>=numSegZ)
			i-=numSegZ;
		nz=double(i)+dz;
		x(1)=nz/double(numSegZ)*(double)mSize.z;
	}
	else
	{
		if(i<0)
		{
			return 0.0;
		}

		if(i>=numSegZ)
		{
			return 0.0;
		}
	}

	vector3 v0, v1, v2;

	/*printf("%f, %f : %f %f %f\n", x(0), x(1), 
			i*m_real(mSize.z)/m_real(numSegZ), 
			mHeight(i,j),
			j*m_real(mSize.x)/m_real(numSegX)); 
			*/
	if(dx>dz)
	{
		// upper triangle ( (j,i), (j+1, i+1), (j+1, i) )
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i, j);
		v1.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v1.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v1.y=mHeight(i+1, j+1);
		v2.x=v1.x;
		v2.z=v0.z;
		v2.y=mHeight(i, j+1);
	}
	else
	{
		// lower triangle ( (j,i), (j, i+1), (j+1, i+1) )
		v0.x=m_real(j)*m_real(mSize.x)/m_real(numSegX);
		v0.z=m_real(i)*m_real(mSize.z)/m_real(numSegZ);
		v0.y=mHeight(i,j);
		v2.x=m_real(j+1)*m_real(mSize.x)/m_real(numSegX);
		v2.z=m_real(i+1)*m_real(mSize.z)/m_real(numSegZ);
		v2.y=mHeight(i+1, j+1);
		v1.x=v0.x;
		v1.z=v2.z;
		v1.y=mHeight(i+1,j);
	}


	//Plane p(getVertex(f.vi(0)), getVertex(f.vi(1)), getVertex(f.vi(2)));
	//return v0.y;
	Plane p(v0, v1, v2);

	Ray r(vector3(x(0), 1000, x(1)), vector3(0,-1,0));
	std::pair<bool, m_real> res=r.intersects(p);

	ASSERT(res.first);
	return r.getPoint(res.second).y;
}

void OBJloader::Terrain::_init(Raw2Bytes& image, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ)
{
	_tileAlongZ=tileAlongZ;
	if(tileAlongZ)
	{
		matrixn signals1, signals2, signals; 
		int nw=sizeY/8;
		signals1.setSize(nw, sizeX);
		signals2.setSize(nw, sizeX);
		for(int y=0; y<nw; y++)
		{
			for(int x=0; x<sizeX; x++)
			{
				signals1(y,x)=(double)image(y,x);
				signals2(y,x)=(double)image(sizeY-nw+y,x);
				signals1(y,x)/=65535.0;
				signals2(y,x)/=65535.0;
			}
		}
		m::c1stitch op;
		op.calc(signals, signals2, signals1);

		for(int y=0; y<nw; y++)
		{
			for(int x=0; x<sizeX; x++)
			{
				image(sizeY-nw+y,x)=ROUND(sop::clampMap(signals(y,x),0,1,0, 65535));
				image(y,x)=ROUND(sop::clampMap(signals(nw-2+y,x),0,1,0,65535));
			}
		}
		for(int x=0; x<sizeX; x++)
			image(sizeY-1,x)=image(0,x);
	}


	mHeight.setSize(sizeY, sizeX);

	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			mHeight(y,x)=heightMax*m_real(image(y,x))/65536.0;
		}
	}



	int numSegX=sizeX-1;
	int numSegZ=sizeY-1;

	mMaxHeight.setSize(numSegZ, numSegX);
	
	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			mMaxHeight[i][j]=std::max(
				std::max(mHeight[i][j], mHeight[i][j+1]), 
				std::max(mHeight[i+1][j],mHeight[i+1][j+1]));
		}
	}

	OBJloader::_createTerrain(*this, image, sizeX, sizeY, width, height, heightMax, ntexSegX, ntexSegZ);
}
OBJloader::Terrain::Terrain(const char *filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ)
:Mesh()
{
	mSize.x=width;
	mSize.z=height;
	mSize.y=heightMax;

	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	FILE* file=fopen(filename, "rb");
	fread( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);
	
	fclose(file);

#define BAD_NOTEBOOK
#ifdef BAD_NOTEBOOK
	// for my notebook, where 512 by 512 mesh doesn't load.
	{
		int _sizeX=sizeX;
		int _sizeY=sizeY;
		sizeX=MIN(64, _sizeX);
		int sx=_sizeX/sizeX;
		sizeY=MIN(64, _sizeY);
		int sy=_sizeY/sizeY;
		Raw2Bytes image2;
		image2.setSize(sizeY, sizeX);
		for(int y=0; y<sizeY; y++)
			for(int x=0; x<sizeX; x++)
				image2(y,x)=image(y*sy, x*sx);
		image.assign(image2);
	}
#endif
	_init(image, sizeX, sizeY, width, height, heightMax, ntexSegX, ntexSegZ, tileAlongZ);

}
OBJloader::Terrain::Terrain(vectorn const& image1d, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ, bool tileAlongZ)
:Mesh()
{
	int sizeX=2;
	int sizeY=image1d.size();
	mSize.x=width;
	mSize.z=height;
	mSize.y=heightMax;

	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	for(int y=0; y<sizeY; y++)
	{
		image(y,0)=int(image1d(y)*65535.0);
		image(y,1)=int(image1d(y)*65535.0);
	}
	_init(image, sizeX, sizeY, width, height, heightMax, ntexSegX, ntexSegZ, tileAlongZ);
}



vector3 OBJloader::Terrain::pick(Ray const& ray, vector3& normal) const
{

	double rayParam=DBL_MAX;
	for(int i=0; i<numFace(); i++)
	{
		Face const& f=getFace(i);
		vector3 a=getVertex(f.vi(0));
		vector3 b=getVertex(f.vi(1));
		vector3 c=getVertex(f.vi(2));
		normal=calcFaceNormal(i);

		auto res=ray.intersects(a,b,c,normal,true,false);
		if(res.first) 
			rayParam=MIN(rayParam, res.second);
	}
	if (rayParam!=DBL_MAX)
			return ray.getPoint(rayParam);
	return vector3(-10000,-10000,-10000);
}
