// TFile.cpp: implementation of the TFile class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "tfile.h"
#include "../math/mathclass.h"
#include "../math/hyperMatrixN.h"
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
TString BinaryFile::_unpackStr()
{
	int len=_unpackInt();
	TString str;
	str.reserve(len+1);

	if(len==0)
	{
		str[0]='\0';
	}
	else
	{
		_unpackArray((void*)str.ptr_ref(),len,sizeof(char));
		str[len]='\0';
	}

	str.updateLength();
	return str;

}
TString BinaryFile::unpackStr()
{
	int typeCode=_unpackInt();
	Msg::verify(typeCode==TYPE_STRING,"unpackStr failed %d", typeCode);
	return _unpackStr();
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

void BinaryFile::pack(const vectorn& vec)
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

void BinaryFile::pack(const vector3& vec)
{
	_packInt(TYPE_FLOATN);
	_packInt(3);
	_packFloat((double)vec.x);
	_packFloat((double)vec.y);
	_packFloat((double)vec.z);
}


void BinaryFile::pack(const quater& vec)
{
	_packInt(TYPE_FLOATN);
	_packInt(4);
	_packFloat((double)vec.x);
	_packFloat((double)vec.y);
	_packFloat((double)vec.z);
	_packFloat((double)vec.w);
}

void BinaryFile::pack(const intvectorn& vec)
{
	_packInt(TYPE_INTN);
	_packInt(vec.size());
	for(int i=0; i<vec.size(); i++) _packInt(vec[i]);
}

void BinaryFile::pack(const matrixn& mat)
{
	if(m_bSinglePrecisionWriteMode)
	{
		_packInt(TYPE_SPFLOATMN);
		_packInt(mat.rows());
		_packInt(mat.cols());
		for(int i=0; i<mat.rows(); i++)
			for(int j=0; j<mat.cols(); j++)
				_packSPFloat((float)mat[i][j]);
	}
	else
	{
		_packInt(TYPE_FLOATMN);
		_packInt(mat.rows());
		_packInt(mat.cols());
		for(int i=0; i<mat.rows(); i++)
			for(int j=0; j<mat.cols(); j++)
				_packFloat((double)mat[i][j]);
	}
}

void BinaryFile::pack(const hypermatrixn& mat3d)
{
	_packInt(-1*((int)TYPE_FLOATMN));
	_packInt(mat3d.page());
	_packInt(mat3d.rows());
	_packInt(mat3d.cols());

	for(int i=0; i<mat3d.page(); i++)
		pack(mat3d.page(i));
}

void BinaryFile::unpack(hypermatrixn& mat3d)
{
	int p=_unpackInt();
	if(p<0) 
	{
		ASSERT(p==-1*((int)TYPE_FLOATMN));
		p=_unpackInt();
	}
	int q=_unpackInt();
	int r=_unpackInt();
	mat3d.setSize(p, q, r);

	for(int i=0; i<mat3d.page(); i++)
		unpack(mat3d.page(i));
}

void BinaryFile::pack(const vector3N& vec)
{
	_packInt(TYPE_FLOATMN);
	_packInt(vec.rows());
	_packInt(3);
	for(int i=0; i<vec.rows(); i++)
	{
		_packFloat((double)vec[i].x);
		_packFloat((double)vec[i].y);
		_packFloat((double)vec[i].z);
	}
}

void BinaryFile::pack(const quaterN& vec)
{
	_packInt(TYPE_FLOATMN);
	_packInt(vec.rows());
	_packInt(4);
	for(int i=0; i<vec.rows(); i++)
	{
		_packFloat((double)vec[i].x);
		_packFloat((double)vec[i].y);
		_packFloat((double)vec[i].z);
		_packFloat((double)vec[i].w);
	}
}

void BinaryFile::pack(const matrix4& mat)
{
	_packInt(TYPE_FLOATMN);
	_packInt(-4);	// negative means square

	for(int i=0; i<4; i++)
	{
		_packFloat((double)mat.m[i][0]);
		_packFloat((double)mat.m[i][1]);
		_packFloat((double)mat.m[i][2]);
		_packFloat((double)mat.m[i][3]);
	}
}

void BinaryFile::unpack(matrixn& mat)
{
	int typeCode=_unpackInt();
	if (typeCode==TYPE_FLOATMN)
		_unpackMat(mat);
	else if(typeCode==TYPE_SPFLOATMN)
		_unpackSPMat(mat);
	else
		Msg::error("unpackmatrixn failed");
}

void BinaryFile::_unpackMat(matrixn& mat)
{
	int row=_unpackInt();
	int col;

	if(row==-4)
	{
		row=4;
		col=4;
	}
	else
		col=_unpackInt();

	mat.setSize(row, col);
	for(int i=0; i<mat.rows(); i++)
		/*
		for(int j=0; j<mat.cols(); j++)
		{
			double f;
			_unpackFloat(f);
			mat[i][j]=f;
		}*/
		_unpackArray((void*)&mat(i,0), mat.cols(), sizeof(double));

}
void BinaryFile::_unpackSPMat(matrixn& mat)
{
	int row=_unpackInt();
	int col;

	if(row==-4)
	{
		row=4;
		col=4;
	}
	else
		col=_unpackInt();

	mat.setSize(row, col);
	for(int i=0; i<mat.rows(); i++)
		for(int j=0; j<mat.cols(); j++)
		{
			float f;
			_unpackSPFloat(f);
			mat[i][j]=(double)f;
		}
}
void BinaryFile::unpack(matrix4& mat)
{
	int typeCode=_unpackInt();
	if (typeCode== TYPE_FLOATMN)
	{
		Msg::verify(_unpackInt()==-4,"unpack matrix4 failed");

		for(int i=0; i<4; i++)
		{
			mat.m[i][0]=_unpackFloat();
			mat.m[i][1]=_unpackFloat();
			mat.m[i][2]=_unpackFloat();
			mat.m[i][3]=_unpackFloat();
		}
	}
	else if(typeCode==TYPE_SPFLOATMN)
	{
		Msg::verify(_unpackInt()==-4,"unpack matrix4 failed");

		for(int i=0; i<4; i++)
		{
			mat.m[i][0]=_unpackSPFloat();
			mat.m[i][1]=_unpackSPFloat();
			mat.m[i][2]=_unpackSPFloat();
			mat.m[i][3]=_unpackSPFloat();
		}
	}
	else Msg::error("unpack matrix4 failed");
}


void BinaryFile::pack(const intmatrixn& mat)
{
	_packInt(TYPE_INTMN);
	_packInt(mat.rows());
	_packInt(mat.cols());
	for(int i=0; i<mat.rows(); i++)
		for(int j=0; j<mat.cols(); j++)
			_packInt(mat[i][j]);
}

void BinaryFile::pack(const TArray<TString>& aSz)
{
	_packInt(TYPE_STRINGN);
	_packInt(aSz.size());
	for(int i=0; i<aSz.size(); i++)
		pack((const char*)aSz[i]);
}

void BinaryFile::pack(const TStrings& aSz)
{
	_packInt(TYPE_STRINGN);
	_packInt(aSz.size());
	for(int i=0; i<aSz.size(); i++)
		pack((const char*)aSz[i]);
}

void BinaryFile::pack(const bitvectorn& vec)
{
	_packInt(TYPE_BITN);
	_packInt(vec.size());

	if(vec.size())
	{
		intvectorn temp;
		vec.getRawData(temp);

		int nBitArray=temp.size();
		for(int i=0; i<nBitArray; i++)
			_packInt(temp[i]);
	}
}

void BinaryFile::unpack(bitvectorn& vec)
{
	int tt=_unpackInt();
	Msg::verify(tt==TYPE_BITN,"unpackBitvectorn failed %d", tt);
	_unpackBit(vec);
}

void BinaryFile::_unpackBit(bitvectorn& vec)
{
	vec.resize(_unpackInt());

	if(vec.size())
	{

		int nBitArray=vec.calcRawDataSize();
		intvectorn t(nBitArray);

		for(int i=0; i<nBitArray; i++)
			t[i]=(unsigned int)(_unpackInt());
		vec.setRawData(t, vec.size());
	}
}
void BinaryFile::pack(const BitArray& bits)
{
	_packInt(TYPE_BITN);
	_packInt(32);
	_packInt(int(bits.m_Bits));
}

void BinaryFile::unpack(BitArray& bits)
{
	Msg::verify(_unpackInt()==TYPE_BITN, "unpack bitArray failed");
	Msg::verify(_unpackInt()==32, "unpack bitArray failed");
	bits.m_Bits=(unsigned int)_unpackInt();
}

void BinaryFile::unpack(TString& str)
{
	Msg::verify(_unpackInt()==TYPE_STRING,"unpack string failed");

	int len=_unpackInt();
	if(len==0)
	{
		str.format("");
	}
	else
	{
		str.reserve(len+1);
		_unpackArray((void*)(str.ptr_ref()),len,sizeof(char));
		str.ptr_ref()[len]='\0';
		str.updateLength();
	}
}

void BinaryFile::unpack(vectorn& vec)
{
	int typeCode=_unpackInt();
	if(typeCode==TYPE_FLOATN)
	{
		vec.setSize(_unpackInt());
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
		vec.setSize(_unpackInt());
		for(int i=0; i<vec.size(); i++)
		{
			float f;
			_unpackSPFloat(f);
		   vec[i]=(double)f;
		}
	}
	else Msg::error("unpackvectorn failed1");
}

void BinaryFile::unpack(vector3& vec)
{
	Msg::verify(_unpackInt()==TYPE_FLOATN,"unpackVector3 failed1");
	Msg::verify(_unpackInt()==3,"unpackVector3 failed2");

	double f;
	_unpackFloat(f);
	vec.x=f;
	_unpackFloat(f);
	vec.y=f;
	_unpackFloat(f);
	vec.z=f;
}

void BinaryFile::unpack(quater& vec)
{
	Msg::verify(_unpackInt()==TYPE_FLOATN,"unpack quater failed1");
	Msg::verify(_unpackInt()==4,"unpack quater failed2");

	double f;
	_unpackFloat(f);
	vec.x=f;
	_unpackFloat(f);
	vec.y=f;
	_unpackFloat(f);
	vec.z=f;
	_unpackFloat(f);
	vec.w=f;
}


void BinaryFile::unpack(intvectorn& vec)
{
	Msg::verify(_unpackInt()==TYPE_INTN,"unpackIntvectorn failed");
	vec.setSize(_unpackInt());
	for(int i=0; i<vec.size(); i++) _unpackInt(vec[i]);
}

void BinaryFile::_unpackVec(vectorn& vec)
{
	vec.setSize(_unpackInt());
	for(int i=0; i<vec.size(); i++) _unpackFloat(vec[i]);	
}
void BinaryFile::_unpackVec(intvectorn& vec)
{
	vec.setSize(_unpackInt());
	for(int i=0; i<vec.size(); i++) _unpackInt(vec[i]);	
}
void BinaryFile::_unpackSPVec(vectorn& vec)
{
	vec.setSize(_unpackInt());
	for(int i=0; i<vec.size(); i++)
	{
		float f;
		_unpackSPFloat(f);
		vec[i]=(double)f;
	}
}



void BinaryFile::unpack(vector3N& vec)
{
	Msg::verify(_unpackInt()==TYPE_FLOATMN,"unpackVector3N failed");

	vec.setSize(_unpackInt());
	Msg::verify(_unpackInt()==3,"unpackVector3N failed2");

	for(int i=0; i<vec.rows(); i++)
	{
		double f;
		_unpackFloat(f);
		vec[i].x=f;
		_unpackFloat(f);
		vec[i].y=f;
		_unpackFloat(f);
		vec[i].z=f;
	}
}

void BinaryFile::unpack(quaterN& vec)
{
	Msg::verify(_unpackInt()==TYPE_FLOATMN,"unpackQuaterN failed");

	vec.setSize(_unpackInt());
	Msg::verify(_unpackInt()==4,"unpackQuaterN failed2");

	for(int i=0; i<vec.rows(); i++)
	{
		double f;
		_unpackFloat(f);
		vec[i].x=f;
		_unpackFloat(f);
		vec[i].y=f;
		_unpackFloat(f);
		vec[i].z=f;
		_unpackFloat(f);
		vec[i].w=f;
	}
}


void BinaryFile::unpack(intmatrixn& mat)
{
	Msg::verify(_unpackInt()==TYPE_INTMN,"unpackIntmatrixn failed");

	int row=_unpackInt();
	int col=_unpackInt();
	mat.setSize(row, col);

	for(int i=0; i<mat.rows(); i++)
		for(int j=0; j<mat.cols(); j++)
			_unpackInt(mat[i][j]);
}

void BinaryFile::unpack(TArray<TString>& aSz)
{
	Msg::verify(_unpackInt()==TYPE_STRINGN,"unpackTArray<TString> failed");
	aSz.init(_unpackInt());
	for(int i=0; i<aSz.size(); i++)
		unpack(aSz[i]);
}

void BinaryFile::unpack(TStrings& aSz)
{
	Msg::verify(_unpackInt()==TYPE_STRINGN,"unpackTArray<TString> failed");
	aSz.init(_unpackInt());
	for(int i=0; i<aSz.size(); i++)
		unpack(aSz[i]);
}

int BinaryFile::getFrameNum(int numOfData)
{
  fseek(this->m_pFile, 0, SEEK_END);
  size_t srcFileSize = ftell(this->m_pFile);
  fseek(this->m_pFile, 0, SEEK_SET);
  size_t frameSize=(sizeof(double)*numOfData);
  return srcFileSize/frameSize;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/// type checking을 하지 않는다. 디버그가 어려워서 internal 용도로만 쓴다.
class TFile
{
public:
	/// bTextMode인 경우, text file형태로 저장한다.
	TFile(bool bTextMode=false);
	bool OpenWriteFile(const char *fileName);
	bool OpenReadFile(const char *fileName);
	void PackInt(int num);
	void PackFloat(float num);
	void PackStr(const char *str);
	void CloseFile();
	int UnpackInt();
	float UnpackFloat();
	void UnpackStr(char *str);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void UnpackStrMalloc(char **pstr);
	void PackArray(void *buffer, size_t count, size_t size);
	void UnpackArray(void *buffer, size_t count, size_t size);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void UnpackArrayMalloc(void **pbuffer, size_t count, size_t size);

	virtual ~TFile();
protected:
	FILE *m_pFile;
	char* buffer;
	bool m_bTextMode;
};
TFile::TFile(bool bTextMode)
{
	m_bTextMode=bTextMode;
	if(m_bTextMode) buffer=new char[1000];
	m_pFile=NULL;
}
TFile::~TFile()
{
	if(m_bTextMode) delete[] buffer;
	ASSERT(m_pFile==NULL);
}
bool TFile::OpenWriteFile(const char *fileName)
{
	ASSERT(m_pFile==NULL);
	if(m_bTextMode)
		m_pFile=fopen(fileName,"w");
	else
		m_pFile=fopen(fileName,"wb");

	if(m_pFile==NULL) return false;
	return true;
}
bool TFile::OpenReadFile(const char *fileName)
{
	ASSERT(m_pFile==NULL);
	if(m_bTextMode)
		m_pFile=fopen(fileName,"r");
	else
		m_pFile=fopen(fileName,"rb");

	if(m_pFile==NULL) return false;
	return true;
}
void TFile::PackStr(const char *str)
{
	if(m_bTextMode)
	{
		fprintf(m_pFile,"%s\n", str);
	}
	else
	{
		if(str==NULL)
		{
			PackInt(0);
			return;
		}
		int len;
		len=strlen(str);
		PackInt(len);
		PackArray((void*)str,len,sizeof(char));
	}
}
void TFile::UnpackStr(char *str)
{
	if(m_bTextMode)
	{
		fgets(buffer,1000,m_pFile);
		strncpy(str,buffer,1000);
		if(str[strlen(buffer)-1]=='\n')
			str[strlen(buffer)-1]=0;
	}
	else
	{
		int len=UnpackInt();
		if(len==0) 
		{
			ASSERT(0);
		}
		UnpackArray((void*)str,len,sizeof(char));
		str[len]='\0';
	}
}
void TFile::UnpackStrMalloc(char **pstr)
{
	if(m_bTextMode)
	{
		ASSERT(0);
	}
	else
	{
		int len=UnpackInt();
		if(len==0)
		{
			*pstr=NULL;
			return;
		}
		*pstr=new char[len+1];
		UnpackArray((void*)(*pstr),len,sizeof(char));
		(*pstr)[len]='\0';
	}
}
void TFile::CloseFile()
{
	ASSERT(m_pFile);
	fclose(m_pFile);
	m_pFile=NULL;
}
void TFile::PackInt(int num)
{
	if(m_bTextMode)
	{
		fprintf(m_pFile,"%d\n", num);
	}
	else
		PackArray((void*)&num,1,sizeof(int));
}

int TFile::UnpackInt()
{
	int num;
	if(m_bTextMode)
	{
		fgets(buffer,1000,m_pFile);
		if(buffer[strlen(buffer)-1]=='\n')
			buffer[strlen(buffer)-1]=0;

		num=atoi(buffer);
	}
	else
	{
		UnpackArray((void*)&num, 1, sizeof(int));
	#ifdef TFILE_DEBUG
		FILE *temp;
		temp=fopen("a.txt","a");
		fprintf(temp,"UI:%d\n",num);
		fclose(temp);
	#endif
	}
	return num;
}
void TFile::PackFloat(float num)
{
	ASSERT(!m_bTextMode);
	PackArray((void*)&num,1,sizeof(float));
}

float TFile::UnpackFloat()
{
	ASSERT(!m_bTextMode);
	float num;
	UnpackArray((void*)&num,1,sizeof(float));
#ifdef TFILE_DEBUG
	FILE *temp;
	temp=fopen("a.txt","a");
	fprintf(temp,"UF:%f\n",num);
	fclose(temp);
#endif

	return num;
}
void TFile::PackArray(void *buffer, size_t count, size_t size)
{
	ASSERT(!m_bTextMode);
	fwrite(buffer, size, count, m_pFile);
}

void TFile::UnpackArray(void *buffer, size_t count, size_t size)
{
	ASSERT(!m_bTextMode);
	fread( buffer, size, count, m_pFile);
}

void TFile::UnpackArrayMalloc(void **pbuffer, size_t count, size_t size)
{
	ASSERT(!m_bTextMode);
	*pbuffer=(void**) malloc (size*count);
	UnpackArray(*pbuffer,count,size);
}
