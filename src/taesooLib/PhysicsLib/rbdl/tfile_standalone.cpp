// TFile.cpp: implementation of the TFile class.
//
//////////////////////////////////////////////////////////////////////

#include "tfile_standalone.h"
#include <stdexcept>
#include <memory.h>

#ifndef ASSERT
#ifdef _DEBUG
#define ASSERT(x) assert(x)
#define RANGE_ASSERT(x) assert(x)
#define VERIFY(x) assert(x)
#define TRACE	Msg::print
#else
#define ASSERT(x) 
#define RANGE_ASSERT(x) do {if(!(x)) throw std::runtime_error("range_error");} while(false)
//#define RANGE_ASSERT(x) assert(x)
//#define RANGE_ASSERT(x) 
#define VERIFY(x)	(x)
#define TRACE	__noop
#endif
#endif

namespace Msg
{
	inline void verify(bool check, const char* str, ...)
	{
		if(not check)
		{
			printf("Error!!! %s\n", str);
			exit(0);
		}
	}
	inline void error(const char* str, ...)
	{
		printf("Error!!! %s\n", str);
		exit(0);
	}
}
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


BinaryFile::BinaryFile(bool bReadToMemory)
{
	m_pFile=NULL;
	m_pBuffer=NULL;

	m_bReadToMemory=bReadToMemory;
}

BinaryFile::BinaryFile(bool bWrite, const char* filename)
{
	m_pFile=NULL;
	m_pBuffer=NULL;
	m_bReadToMemory=false;
	if(bWrite) openWrite(filename);
	else openRead(filename);
}

BinaryFile::~BinaryFile()
{
	if(m_pFile || m_pBuffer)
		close();
}

bool BinaryFile::openWrite(const char *fileName, bool singlePrecisionMode)
{
	m_bSinglePrecisionWriteMode=singlePrecisionMode;
	if(m_pFile) close();
	m_pFile=fopen(fileName,"wb");

	if(m_pFile==NULL)
	{
		printf("Open error %s\n", fileName);
		return false;
	}
	return true;
}
bool BinaryFile::openRead(const char *fileName)
{
	if(m_pFile) close();
	m_pFile=fopen(fileName,"rb");

	if(m_pFile==NULL)
	{
		printf("Open error %s\n", fileName);
		return false;
	}

	if(m_bReadToMemory)
	{
		// obtain file size.
		fseek (m_pFile , 0 , SEEK_END);
		long lSize = ftell (m_pFile);
		rewind (m_pFile);

		// allocate memory to contain the whole file.
		m_pBuffer = (char*) malloc (lSize);
		if (m_pBuffer == NULL) exit (2);

		// copy the file into the buffer.
		fread (m_pBuffer ,1,lSize,m_pFile);

		/*** the whole file is loaded in the buffer. ***/

		m_pBufferPointer=m_pBuffer;
		// terminate
		fclose(m_pFile);
		m_pFile=NULL;
	}

	return true;
}

void BinaryFile::unpackStr(char *str)
{
	int typeCode=_unpackInt();
	Msg::verify(typeCode==TYPE_STRING,"unpackStr failed %d", typeCode);

	int len=_unpackInt();
	if(len==0)
	{
		str[0]='\0';
	}
	else
	{
		_unpackArray((void*)str,len,sizeof(char));
		str[len]='\0';
	}
}

void BinaryFile::close()
{
	ASSERT(m_pFile || m_pBuffer);
	if(m_pFile)
	{
		fclose(m_pFile);
		m_pFile=NULL;
	}

	if(m_pBuffer)
	{
		free(m_pBuffer);
		m_pBuffer=NULL;
	}
}

void BinaryFile::_packInt(int num)
{
	_packArray((void*)&num,1,sizeof(int));
}

void BinaryFile::_unpackInt(int& num)
{
	_unpackArray((void*)&num, 1, sizeof(int));
	if(m_pFile==NULL && m_pBuffer==NULL) num=TYPE_EOF;
}

void BinaryFile::_packFloat(double num)
{
	_packArray((void*)&num,1,sizeof(double));
}

void BinaryFile::_unpackFloat(double& num)
{
	_unpackArray((void*)&num,1,sizeof(double));
}

void BinaryFile::_packSPFloat(float num2)
{
	_packArray((void*)&num2,1,sizeof(float));
}
void BinaryFile::_unpackSPFloat(float& num)
{
	_unpackArray((void*)&num,1,sizeof(float));
}

void BinaryFile::_packArray(void *buffer, int count, size_t size)
{
	fwrite(buffer, size, count, m_pFile);
}

void BinaryFile::_unpackArray(void *buffer, int count, size_t size)
{
	if(m_bReadToMemory)
	{
		size_t sizeT=size*count;
		memcpy(buffer, m_pBufferPointer, sizeT);
		m_pBufferPointer+=sizeT;
	}
	else
	{
		//fread(buffer,size,count,m_pFile);

		if (fread(buffer,size, count,m_pFile) != count)
		{
			
			if (ferror (m_pFile))
			{
				printf("ferror\n");
				perror(NULL);
				Msg::verify(false, "Error Reading from file\n");;
			}

			m_pFile=NULL;
		}
	}
}

void BinaryFile::packInt(int num)
{
	_packInt(TYPE_INT);
	_packInt(num);
}

void BinaryFile::unpackInt(int& num)
{
	int tt=_unpackInt();
	Msg::verify(tt==TYPE_INT,"unpackInt failed %d", tt);
	_unpackInt(num);
}

void BinaryFile::packFloat(double num)
{
	_packInt(TYPE_FLOAT);
	_packFloat(num);
}

void BinaryFile::unpackFloat(double& num)
{
	int tt=_unpackInt();
	Msg::verify(tt==TYPE_FLOAT,"unpackFloat failed %d", tt);
	_unpackFloat(num);
}

void BinaryFile::packArray(void *buffer, int count, size_t size)
{
	_packInt(TYPE_ARRAY);	// checksum
	_packInt(count);
	_packInt(size);
	_packArray(buffer, count, size);
}


void BinaryFile::unpackArray(void *buffer, int count, size_t size)
{
	Msg::verify(_unpackInt()==TYPE_ARRAY,"unpackArray failed1");	// checksum
	Msg::verify(_unpackInt()==count,"unpackArray failed2");
	Msg::verify(_unpackInt()==size,"unpackArray failed3");
	_unpackArray(buffer, count, size);
}

void BinaryFile::unpackArrayMalloc(void **pbuffer, int count, size_t size)
{
	Msg::verify(_unpackInt()==TYPE_ARRAY,"unpackArrayM failed1");	// checksum
	Msg::verify(_unpackInt()==count,"unpackArrayM failed2");
	Msg::verify(_unpackInt()==size,"unpackArrayM failed3");
	*pbuffer=(void**) malloc (size*count);
	_unpackArray(*pbuffer,count,size);
}

void BinaryFile::pack(const char *str)
{
	_packInt(TYPE_STRING);
	if(str==NULL )
	{
		_packInt(0);
		return;
	}

	int len;
	len=strlen(str);
	_packInt(len);
	if(len)
		_packArray((void*)str,len,sizeof(char));
}

void BinaryFile::pack(const VectorNd& vec)
{
	if(m_bSinglePrecisionWriteMode)
	{
		_packInt(TYPE_SPFLOATN);
		_packInt(vec.size());
		if(vec.size()>0)
		{
			for(int i=0; i<vec.size(); i++) _packSPFloat((float)vec[i]);
		}
	}
	else
	{
		_packInt(TYPE_FLOATN);
		_packInt(vec.size());
		//for(int i=0; i<vec.size(); i++) _packFloat((double)vec[i]);
		if(vec.size()>0)
			_packArray((void*)&vec[0], vec.size(), sizeof(double));
	}
}

void BinaryFile::unpack(VectorNd& vec)
{
	int typeCode=_unpackInt();
	if(typeCode==TYPE_FLOATN)
	{
		vec.resize(_unpackInt());
		/*
		   for(int i=0; i<vec.size(); i++)
		   {
		   double f;
		   _unpackFloat(f);
		   vec[i]=f;
		   }*/
		if(vec.size()>0)
			_unpackArray((void*)&vec[0], vec.size(), sizeof(double));
	}
	else if(typeCode==TYPE_SPFLOATN)
	{
		vec.resize(_unpackInt());
		for(int i=0; i<vec.size(); i++)
		{
			float f;
			_unpackSPFloat(f);
		   vec[i]=(double)f;
		}
	}
	else Msg::error("unpackvectorn failed1");
}



