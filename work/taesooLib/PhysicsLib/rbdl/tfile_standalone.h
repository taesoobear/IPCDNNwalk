#ifndef _TFILE_H_
#define _TFILE_H_
// tfile.h: interface for the TFile class.
//
//////////////////////////////////////////////////////////////////////

#pragma once

#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>
using namespace RigidBodyDynamics::Math;

/// Type을 저장한다.
class BinaryFile
{
public:
	BinaryFile(bool bReadToMemory=false);
	BinaryFile(bool bWrite, const char* filename);
	virtual ~BinaryFile();
	bool openWrite(const char *fileName, bool singlePrecisionMode=false);
	bool openRead(const char *fileName);
	virtual void close();

	void packInt(int num);
	void packFloat(double num);
	void packArray(void *buffer, int count, size_t size);
	void pack(const char *str);
	void pack(const VectorNd& vec);

	void unpackInt(int& num);
	void unpackFloat(double& num);
	int	unpackInt()					{ int num; unpackInt(num); return num; }
	double unpackFloat()				{ double num; unpackFloat(num); return num; }
	void unpackStr(char *str);	//!< 할당 되어있음을 가정한다.
	void unpackArray(void *buffer, int count, size_t size);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void unpackArrayMalloc(void **pbuffer, int count, size_t size);
	void unpack(VectorNd& vec);


	// without type checking
	void _packInt(int num);
	void _unpackInt(int& num);
	void _packFloat(double num);
	void _unpackFloat(double& num);
	void _packSPFloat(float num);
	void _unpackSPFloat(float & num);
	
	int _unpackInt()	{ int num; _unpackInt(num); return num;}
	double _unpackFloat()	{ double num; _unpackFloat(num); return num;}
	float _unpackSPFloat()	{ float num; _unpackSPFloat(num); return num;}
	void _unpackVec(VectorNd& vec);
	virtual void _packArray(void *buffer, int count, size_t size);
	virtual void _unpackArray(void *buffer, int count, size_t size);

	enum { TYPE_INT, TYPE_FLOAT, TYPE_FLOATN, TYPE_INTN, TYPE_BITN, TYPE_FLOATMN, TYPE_INTMN, TYPE_BITMN, TYPE_STRING, TYPE_STRINGN, TYPE_ARRAY , TYPE_EOF, TYPE_SPFLOAT, TYPE_SPFLOATN, TYPE_SPFLOATMN };

	inline FILE*& _getFilePtr() { return m_pFile;}
	int getFrameNum(int numOfData);
protected:

	FILE *m_pFile;
	char *m_pBuffer;
	bool m_bReadToMemory;
	bool m_bSinglePrecisionWriteMode;
	char* m_pBufferPointer;
};



#endif
