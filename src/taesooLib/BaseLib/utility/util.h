#ifndef _UTIL_H_
#define _UTIL_H_
// util.h: interface for the util class.
//
//////////////////////////////////////////////////////////////////////

#if _MSC_VER >1000
#pragma once
#endif


int fast_strcmp(const char *a, const char *b);
int fast_strcmp_upper(const char *a, const char *b);

//!엽기 함수 , 아마 쓸일 없을꺼다.
int calcMaxPrefix(char* a, char*b);
int calcMaxSubstring(char *a, char*b);	//!< a위에서 b를 움직이면서 가장 많이 겹칠때 겹치는 글자수 return

void FindAndSubstitute(char *source, char *pattern, char *output);
char * GetToken(FILE *file);		//!< #으로 시작하는 줄을 주석으로 취급한다. token은 space또는 ,로 구분된다.
char * GetTokenCLang(FILE *file);	//!< //와 /* */를 주석으로 취급한다. token은 C의 문법을 기준으로 분리되어 나온다. 한줄은 4000자를 넘지 않아야 한다.

bool IsFileExist(const char* filename);
bool IsFileWritable(const char*szFileName);
//!만약 Token이 NULL이 될때까지 GetToken을 하지 않고 다른 파일을 GetToken해야 하는 경우 그전에 FileCloseForGetToken을 call해주어야 한다.
void FileCloseForGetToken();
void FileCloseForGetTokenCLang();

void ParseCommandLineString(const char* input,int& argc, char**& argv);
void FreeCommandLineString(int argc, char** argv);

bool ConfirmWritable(const char* strFilePath);
bool createDirectory(const char *PathToCreate);
void deleteFile( const char* filename);

char* CopyStr(const char* str);

template <class T> T** AllocMatrix(int height, int width)	//!< 행열
{
	T **aaType;
	aaType=new T*[height];
	for(int i=0; i< height; i++)
		aaType[i]=new T[width];
	return aaType;
}

template <class T> void FreeMatrix(int height, T** array)
{
	for( int i=0; i<height; i++)
		delete[] array[i];	// delete columns
	delete[] array;
}


#ifdef DIRECT3D_VERSION
inline float DWToFloat(DWORD dw)	{ return *((float*)(&dw));};
inline DWORD FloatToDW(float f)		{ return *((DWORD*)(&f));};
void GetRotationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource);
void GetTranslationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource);
void GetAxisRotationMatrix(D3DXMATRIX& matRot, const D3DXVECTOR3& vecAxis, const D3DXVECTOR3& front, const D3DXVECTOR3& vecTarget);
#endif

template <class T> T* ToPtr(int i)		{ return *((T**)(&i));};
template <class T> int ToInt(T* ptr)	{ return *((int*)(&ptr));};


//! MOutputToFile("a.txt", ("(%d,%d)",a,b) );	이런 식으로 사용하세요.
#define MOutputToFile(filename, arg) { TString str;str.format arg ; OutputToFile(filename, (const char *)str); }

void OutputToFile(const char* filename, const char* string);
int Hash(const char* string);
namespace RE {
	TString generateUniqueName();
}
#endif
