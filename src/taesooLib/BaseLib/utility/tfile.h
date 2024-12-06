#ifndef _TFILE_H_
#define _TFILE_H_
// tfile.h: interface for the TFile class.
//
//////////////////////////////////////////////////////////////////////

#pragma once

#include "TextFile.h"
#include "TypeString.h"
#include "TArray.h"
class vectorn;
class intvectorn;
class matrixn;
class intmatrixn;
class boolN;
class vector3N;
class quaterN;
class matrix4;
class vector3;
class quater;
namespace BaseLib {
class BitArray;
}
class hypermatrixn;
class floatvec;

class Tensor;
class floatTensor;
/// Type을 저장한다.
class BinaryFile
{
public:
	BinaryFile(bool bReadToMemory=false);
	BinaryFile(bool bWrite, const char* filename);
	BinaryFile(bool bWrite, const std::string & filename);
	virtual ~BinaryFile();
	bool openWrite(const char *fileName, bool singlePrecisionMode=false);
	bool openRead(const char *fileName);
	virtual void close();

	void packInt(int num);
	void packFloat(double num);
	void packArray(void *buffer, int count, size_t size);
	void pack(const char *str);
	void pack(const TString& str) { pack(str.ptr());}
	void pack(const vectorn& vec);
	void pack(const floatvec& vec);
	void pack(const vector3& vec);
	void pack(const quater& vec);
	void pack(const intvectorn& vec);
	void pack(const matrixn& mat);
	void pack(const intmatrixn& mat);
	void pack(const vector3N& mat);
	void pack(const quaterN& mat);
	void pack(const TArray<TString>& aSz);
	void pack(const TStrings& aSz);
	void pack(const boolN& vec);
	void pack(const matrix4& mat);
	void pack(const BaseLib::BitArray& bits);
	void pack(const hypermatrixn& mat3d);
	void pack(const Tensor& matnd);
	void pack(const floatTensor& matnd);


	void unpack(BaseLib::BitArray& bits);
	void unpackInt(int& num);
	void unpackFloat(double& num);
	int	unpackInt()					{ int num; unpackInt(num); return num; }
	double unpackFloat()				{ double num; unpackFloat(num); return num; }
	void unpackStr(char *str);	//!< 할당 되어있음을 가정한다.
	TString unpackStr();
	void unpackArray(void *buffer, int count, size_t size);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void unpackArrayMalloc(void **pbuffer, int count, size_t size);
	void unpack(TString& str);
	void unpack(vectorn& vec);
	void unpack(vector3& vec);
	void unpack(quater& vec);
	void unpack(intvectorn& vec);
	void unpack(matrixn& mat);
	void unpack(intmatrixn& mat);
	void unpack(TArray<TString>& aSz);
	void unpack(TStrings& aSz);
	void unpack(boolN& vec);
	void unpack(quaterN& mat);
	void unpack(vector3N& mat);
	void unpack(matrix4& mat);
	void unpack(hypermatrixn& mat3d);
	void unpack(Tensor& matnd);
	void unpack(floatTensor& matnd);


	// without type checking
	void _packInt(int num);
	void _unpackInt(int& num);
	void _packFloat(double num);
	void _unpackFloat(double& num);
	void _packSPFloat(float num);
	void _unpackSPFloat(float & num);
	
	// used in lua (when implementing unpackAuto()
	int _unpackInt()	{ int num; _unpackInt(num); return num;}
	double _unpackFloat()	{ double num; _unpackFloat(num); return num;}
	float _unpackSPFloat()	{ float num; _unpackSPFloat(num); return num;}
	TString _unpackStr();
	void _unpackVec(vectorn& vec);
	void _unpackVec(intvectorn& vec);
	void _unpackSPVec(vectorn& vec);
	void _unpackMat(matrixn& mat);
	void _unpackSPMat(matrixn& mat);
	void _unpackTensor(floatTensor& mat);
	void _unpackTensor(Tensor& mat);
	void _unpackBit(boolN& vec);
	virtual void _packArray(void *buffer, int count, size_t size);
	virtual void _unpackArray(void *buffer, int count, size_t size);

	enum { TYPE_INT, TYPE_FLOAT, TYPE_FLOATN, TYPE_INTN, TYPE_BITN, TYPE_FLOATMN, TYPE_INTMN, TYPE_BITMN, TYPE_STRING, TYPE_STRINGN, TYPE_ARRAY , TYPE_EOF, TYPE_SPFLOAT, TYPE_SPFLOATN, TYPE_SPFLOATMN , TYPE_FLOATND, TYPE_SPFLOATND};

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
