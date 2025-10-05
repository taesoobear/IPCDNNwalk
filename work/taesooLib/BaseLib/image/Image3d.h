#pragma once

template <class T>
class TImage3D
{
	std::vector<T*> pixels;

	int _width;
	int _height;
	int _depth;
	public:
	TImage3D(int x, int y, int z, T t)
	{
		_width=x;
		_height=y;
		_depth=z;
		pixels.resize(z);
		for(int i=0; i<z; i++)
		{
			pixels[i]=new T[x*y];

			for(int k=0; k<x*y; k++)
				pixels[i][k]=t;
		}
	}
	~TImage3D()
	{
		for(int i=0; i<pixels.size(); i++)
			delete pixels[i];
	}
	inline T* getPixels(int z) const { return pixels[z];} // 0-indexing
	inline int getSize() const { return pixels.size();}
	inline void setPixel(int x, int y, int z, T value) { getPixels(z)[x+y*_width]=value; }
	inline T getPixel(int x, int y, int z) const { return getPixels(z)[x+y*_width]; }

	inline bool findNeighbor6(int x, int y, int z, T value) const
	{
		if(x<_width-1 && getPixel(x+1, y,z)==value ) return true;
		if(x>0 && getPixel(x-1, y,z)==value ) return true;
		if(y<_height-1 && getPixel(x, y+1,z)==value ) return true;
		if(y>0 && getPixel(x, y-1,z)==value ) return true;
		if(z<_depth-1 && getPixel(x, y,z+1)==value ) return true;
		if(z>0 && getPixel(x, y,z-1)==value ) return true;
		return false;
	}
	inline bool findNonNeighbor6(int x, int y, int z, T value) const
	{
		if(x<_width-1 && getPixel(x+1, y,z)!=value ) return true;
		if(x>0 && getPixel(x-1, y,z)!=value ) return true;
		if(y<_height-1 && getPixel(x, y+1,z)!=value ) return true;
		if(y>0 && getPixel(x, y-1,z)!=value ) return true;
		if(z<_depth-1 && getPixel(x, y,z+1)!=value ) return true;
		if(z>0 && getPixel(x, y,z-1)!=value ) return true;
		return false;
	}
};

