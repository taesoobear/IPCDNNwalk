// util.cpp: implementation of the util class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "util.h"
#include <stdlib.h>
#include <ctype.h>
#include <vector>
#include <stdexcept>
#include "TypeString.h"

#ifdef _MSC_VER
#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#endif

Msg::Base g_cMsgUtil;
Msg::Base* Msg::g_pMsgUtil=&g_cMsgUtil;

void Msg::verify(bool bExpression, const char* pszFormat, ...)
{
	// release에서도 없어지지 않는 verify를 하고 싶은경우 사용.
	if(!bExpression)
	{
		TString temp;
		va_list argptr ;
		va_start(argptr, pszFormat);
		temp._format(pszFormat, argptr);
		g_pMsgUtil->error(temp);
	}
}
void Msg::print(const char* pszFormat,...)
{
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	g_pMsgUtil->print(temp);
}
void Msg::print2(const char* pszFormat,...)
{
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	g_pMsgUtil->print2(temp);
}

void Msg::error(const char* pszFormat,...)
{
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	printf("%s\n",temp.ptr());
	g_pMsgUtil->error(temp);
}
void Msg::msgBox(const char* pszFormat,...)
{
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	g_pMsgUtil->msgBox(temp);
}
bool Msg::confirm(const char* pszFormat,...)
{
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	return g_pMsgUtil->confirm(temp);
}
void Msg::flush()
{
	g_pMsgUtil->flush();
}

void Msg::outputState(bool bOutput)
{
	g_pMsgUtil->m_bOutput=bOutput;
}

void Msg::output(const char* key, const char* pszFormat, ...)
{
	if(!g_pMsgUtil->m_bOutput) return;
	TString temp;
	va_list argptr ;
	va_start(argptr, pszFormat);
	temp._format(pszFormat, argptr);
	g_pMsgUtil->output(key, temp);
}
		// inherit and implement followings
void Msg::Base::print(const char* msg)	{ printf("%s", msg);}
void Msg::Base::print2(const char* msg)	{ printf("                                       \r");printf("%s\r", msg); }
void Msg::Base::flush()				{ fflush(stdout);}
void Msg::Base::error(const char* msg) { msgBox(msg); ASSERT(0);throw(std::runtime_error(msg));}
void Msg::Base::msgBox(const char* msg){ printf("%s\n", msg);fflush(stdout); }
		// 아직 구현안됨. 사용자에게 yes or no물어보는 기능.
bool Msg::Base::confirm(const char* msg) { ASSERT(0); return true;}
void Msg::Base::output(const char* key, const char* msg){printf("%s: %s\n", key, msg); }
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

int fast_strcmp(const char *a, const char *b)
{
	while(*a!='\0')
	{
		if(*a!=*b) return -1;
		a++;
		b++;
	}
	if(*b=='\0') return 0;
	return -1;
}

int fast_strcmp_upper(const char *a, const char *b)
{
	while(*a!='\0')
	{
		if(toupper(*a)!=toupper(*b)) return -1;
		a++;
		b++;
	}
	if(*b=='\0') return 0;
	return -1;
}

static FILE *g_fFile = NULL;
static char * token = NULL;

void FileCloseForGetToken()
{
	g_fFile = NULL;
	token = NULL;
}

char * GetToken(FILE *file)
{
	static char buff[4096];
	char seps[]   = " ,\t\n\r";

	if(g_fFile){
		ASSERT(g_fFile == file);
	}
	else{
		g_fFile = file;
	}

	/*
	Warning : Each of these functions uses a static variable for parsing the string into tokens.
	If multiple or simultaneous calls are made to the same function, a high potential for data
	corruption and inaccurate results exists.
	Therefore, do not attempt to call the same function simultaneously for different strings
	and be aware of calling one of these functions from within a loop where another routine
	may be called that uses the same function.
	However, calling this function simultaneously from multiple threads does not have
	undesirable effects.
	*/

	// 위의 정보로 추측해 볼때 Print 찾는 부분에서 strtok를 사용하기 때문에 생기는 에러라고 생각됨.
	// 임시 방편 해결책을 사용함...  나중에 Print찾는 부분을 수정하든지 해야겠다.. - akasha
	// 임시 해결책 : token은 NULL이 아닌 이상, 항상 buff부터 buff + 4096까지 address안에 존재해야 함.
	if(token)
	{
		if(token < buff || token > buff + (4096 * sizeof(char)))
		{
			Msg::error("??? gettoken");
			token = NULL;
		}
	}

	if(token){
		token = strtok(NULL, seps);
	}

	while(!token)
		if(fgets(buff, 4096, file)){
			if(buff[0] != '#')
				token = strtok(buff, seps);
		}
		else {
			FileCloseForGetToken();
			return NULL;
		}

#ifdef ALL_CHAR_TOUPPER

	for(int i = 0; i < strlen(token); i++)
		token[i] = toupper(token[i]);

#endif

	return token;
}

static FILE *g_fFileCLang = NULL;
static int g_nCurPos=-1;
void FileCloseForGetTokenCLang()
{
	g_fFileCLang = NULL;
	g_nCurPos=-1;
}

char * GetTokenCLang(FILE *file)
{

	// deprecated.
	static char buffCLang[4096];
	static char buffToken[1000];
	char *token=NULL;
	char seps[]   = " \t\n";


	if(g_fFileCLang){
		ASSERT(g_fFileCLang == file);
	}
	else{
		g_fFileCLang = file;
	}

	bool bComment=false;
NEWLINE:
	if(g_nCurPos==-1)
	{
		if(fgets(buffCLang, 4096, file))
		{
			g_nCurPos=0;
		}
		else
		{
			FileCloseForGetTokenCLang();
			return NULL;
		}

	}
NEWTOKEN:
	int startPos=g_nCurPos;
	for(int i=g_nCurPos; i<4096; i++)
	{
		if(buffCLang[i]=='/')
		{
			if(buffCLang[i+1]=='/') {g_nCurPos=-1; goto NEWLINE;}
			else if(buffCLang[i+1]=='*' && !bComment) { bComment=true; }	// start comment
			else if(bComment && i>1 && buffCLang[i-1]=='*') { bComment=false; startPos=i+1; }	//end comment
		}
		if(bComment) continue;
		else if(buffCLang[i]==';' || buffCLang[i]=='(' || buffCLang[i]==')' || buffCLang[i]=='}' || buffCLang[i]=='{' || buffCLang[i]=='*'
			|| buffCLang[i]=='+' || buffCLang[i]=='<' || buffCLang[i]=='>' || buffCLang[i]==',' || buffCLang[i]=='!'
			|| buffCLang[i]=='[' || buffCLang[i]==']' || buffCLang[i]=='=')
		{
			if(startPos<i)
			{
				strncpy(buffToken, &buffCLang[startPos], i-startPos);
				buffToken[i-startPos]='\0';
				g_nCurPos=i;
				return buffToken;
			}
			else
			{
				buffToken[0]=buffCLang[i];
				buffToken[1]='\0';
				g_nCurPos=i+1;
				return buffToken;
			}
		}
		else if(buffCLang[i]==' ' || buffCLang[i]=='\t' || buffCLang[i]=='\n')
		{
			strncpy(buffToken, &buffCLang[startPos], i-startPos);
			buffToken[i-startPos]='\0';
			g_nCurPos=i;
			while(buffCLang[g_nCurPos]==' ' || buffCLang[g_nCurPos]=='\t') g_nCurPos++;
			if(buffCLang[i]=='\n') { g_nCurPos=-1; if(i-startPos==0) goto NEWLINE; }
			if(i==startPos)
				goto NEWTOKEN;
			else return buffToken;
		}
	}
	if(bComment) { g_nCurPos=-1; goto NEWLINE;	}
	FileCloseForGetTokenCLang();
	return token;
}

void FindAndSubstitute(char *source, char *pattern, char *output)
{
	int i=0;
	int pi=0;
	int oi=0;
	int patLen=strlen(pattern);
	while(source[i]!=0)
	{
		if(source[i]==pattern[pi])
		{
			// if end
			if(pi==patLen-1)
			{
				for(int temp=0;temp<patLen;temp++)
				{
					output[oi]=pattern[temp];
					oi++;
				}
				pi=0;
			}
			else pi++;
			i++;
		}
		else
		{
			for(int temp=0;temp<pi;i++)
			{
				output[oi]=source[i-pi+temp];
			}
			pi=0;
			i++;
		}
	}
}

int calcMaxSubstring(char *a, char*b)
{
	int lena=strlen(a);
	int lenb=strlen(b);
	ASSERT(lenb>lena);
	// a가 b보다 길거나 같다.
	int max=0;
	int len;
	for(int i=0; i<lena-lenb+1; i++)
	{
		if((len=calcMaxPrefix(&(a[i]),b))>max) max=len;
	}
	return max;
}

int calcMaxPrefix(char* a, char*b)
{
	int i=0;
	int count=0;
	while(1)
	{
		if(a[i]==0) break;
		if(b[i]==0) break;
		if(a[i]==b[i]) count++;
	}
	return count;
}
#ifdef DIRECT3D_VERSION
void GetRotationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource)
{
	*pTarget=*pSource;
	pTarget->_41=0;
	pTarget->_42=0;
	pTarget->_43=0;
}

void GetTranslationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource)
{
	D3DXMatrixIdentity(pTarget);
	pTarget->_41=pSource->_41;
	pTarget->_42=pSource->_42;
	pTarget->_43=pSource->_43;
}

void GetAxisRotationMatrix(D3DXMATRIX& matRot, const D3DXVECTOR3& vecAxis, const D3DXVECTOR3& front, const D3DXVECTOR3& vecTarget)
{
	// front벡터를 vecAxis를 중심으로 회전해서 vecTarget과 vecAxis가 이루는 평면에 놓이도록 만드는 Matrix를 구한다.

	// Axis와 vecTarget이 이루는 평면의 normal을 계산한다.
	D3DXVECTOR3 vecNormal, nVecNormal, vecProjVecTarget;
	D3DXVec3Cross(&vecNormal, &vecAxis, &vecTarget);
	D3DXVec3Normalize(&nVecNormal,&vecNormal);
	D3DXVec3Cross(&vecProjVecTarget, &nVecNormal, &vecAxis);

	// Axis와 front가 이루는 평면의 normal을 계산한다.
	D3DXVECTOR3 vecNormal2,nVecNormal2, vecProjFront;
	D3DXVec3Cross(&vecNormal2, &vecAxis, &front);
	D3DXVec3Normalize(&nVecNormal2,&vecNormal2);
	D3DXVec3Cross(&vecProjFront, &nVecNormal2, &vecAxis);

	// 이제 아래가 성립하는 mtrxAxisAlignedBillboard을 구한다.
	// vecAxis									 vecAxis
	// nVecNormal2 * mtrxAxisAlignedBillboard =  nVecNormal
	// vecProjFront								 vecProjVecTarget

	// 따라서								vecAxis    T     vecAxis
	//		mtrxAxisAlignedBillboard =  (	nVecNormal2)  * (nVecNormal)
	//										vecProjFront     vecProjVecTarget

	D3DXMATRIX mat1, mat2;
	mat1._11=vecAxis.x;	mat1._21=vecAxis.y;	mat1._31=vecAxis.z;	mat1._41=0;
	mat1._12=nVecNormal2.x; mat1._22=nVecNormal2.y; mat1._32=nVecNormal2.z; mat1._42=0;
	mat1._13=vecProjFront.x; mat1._23=vecProjFront.y; mat1._33=vecProjFront.z; mat1._43=0;
	mat1._14=0; mat1._24=0; mat1._34=0; mat1._44=1;

	mat2._11=vecAxis.x;	mat2._12=vecAxis.y;	mat2._13=vecAxis.z;	mat2._14=0;
	mat2._21=nVecNormal.x; mat2._22=nVecNormal.y; mat2._23=nVecNormal.z; mat2._24=0;
	mat2._31=vecProjVecTarget.x; mat2._32=vecProjVecTarget.y; mat2._33=vecProjVecTarget.z; mat2._34=0;
	mat2._41=0; mat2._42=0; mat2._43=0; mat2._44=1;

	matRot=mat1*mat2;
}

#endif

void ParseCommandLineString(const char* inputt,int& argc, char**& argv)
{
	char seps[]   = " \n";
	char input[1000];
	strcpy(input, inputt);

	char* token= strtok(input, seps);
	std::vector<char*> argvList;
	while(token)
	{
		char* temp=new char[strlen(token)+1];
		strcpy(temp, token);
		argvList.push_back(temp);
		token=strtok(NULL, seps);
	}

	argv=new char*[argvList.size()];
	argc=argvList.size();
	for(int i=0; i<argc; i++)
	{
		argv[i]=argvList[i];
	}
}

void FreeCommandLineString(int argc, char** argv)
{
	for(int i=0; i<argc; i++)
		delete[] argv[i];
	delete[] argv;
}

void OutputToFile(const char* filename, const char* string)
{
	FILE* pFile;
	pFile = fopen(filename,"a");
	fprintf(pFile,"%s\n",string);
	fclose(pFile);
}
void OutputToFile2(const char* filename, const char* string)
{
	FILE* pFile;
	pFile = fopen(filename,"a");
	fprintf(pFile,"%s",string);
	fclose(pFile);
}

bool IsFileExist(const char* filename)
{
	FILE* temp;
	temp=fopen(filename,"r");
	if(!temp) return false;
	fclose(temp);
	return true;
}

bool IsFileWritable(const char* szFileName)
{
	FILE *fp=fopen(szFileName, "w");
	if(fp == NULL) return 0;
	fclose(fp);
	return true;
}


bool ConfirmWritable(const char* strFilePath)
{
	bool ret=true;
	while(!IsFileWritable(strFilePath))
	{
		TString str;
		str.format("%s 파일에 저장할 수 없습니다. 다시 시도 하시겠습니까?",(strFilePath));
		if(Msg::confirm(str))
		{
			ret=false; break;
		}
	}
	return ret;
}


char* CopyStr(const char* str)
{
	if(str==NULL) return NULL;
	int len=strlen(str);
	char* temp=new char[len+1];
	strcpy(temp, str);
	return temp;
}

bool createDirectory(const char *PathToCreate)
{
#ifdef _WIN32
	static char Path[1000];
	strcpy(Path,PathToCreate);

	// First add an ending slash
	char* Pos = strrchr(Path,'\\');
	if (!Pos || Pos[1]!=0)
	strcat(Path,"\\");
	// From the first slash to the last check and create path
	// Exception : Jump initial duble slashed UNC host name and include first directory
	if (Path[0]=='\\' && Path[1]=='\\')
	{
		Pos = strchr(Path+2,'\\');
		if (Pos)
			Pos = strchr(Pos+1,'\\');
	}
	else
		Pos = strchr(Path,'\\');
	bool AllOk = true;
	int ret;
	while (Pos && AllOk)
	{
		*Pos = 0;
		ret=GetFileAttributes(Path);
		if (ret==-1)
		{
			if (GetLastError()==ERROR_FILE_NOT_FOUND)
				AllOk = CreateDirectory(Path,NULL)==TRUE;
			else
				AllOk = false;
		}
		else if ((ret|FILE_ATTRIBUTE_DIRECTORY)!=0)
		{
			if(strchr(Pos+1,'\\')==0)
				AllOk=false;
		}
		else
		{
			LPVOID lpMsgBuf;
			FormatMessage(
				FORMAT_MESSAGE_ALLOCATE_BUFFER |
				FORMAT_MESSAGE_FROM_SYSTEM |
				FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL,
				GetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
				(LPTSTR) &lpMsgBuf,
				0,
				NULL
			);

			AllOk = false;
		}
		*Pos = '\\';
		Pos = strchr(Pos+1,'\\');
	}
	return AllOk;
#else
	static char Path[1000];
	sprintf(Path, "mkdir -p '%s'", PathToCreate);
	system(Path);
	return true;
#endif
}

void deleteFile( const char* filename)
{
#ifdef _WIN32
	::DeleteFile(filename);
#else
	remove(filename);
#endif
}
TString RE::generateUniqueName()
{
	TString name;
	static int nId=0;
	name.format("___uniq%d", nId);
	nId++;

	return name;
}
static int g_located=0;
static std::string g_taesooLibPath("work/taesooLib/");
void RE::setTaesooLibPath(const char* path)
{
	g_taesooLibPath=path;
	g_located=1;
}
std::string RE::taesooLibPath()
{
	if (!g_located)
	{
		if( !IsFileExist("plugins.cfg"))
			g_located=1;
		else
			g_located=2;
	}
	if(g_located==1)
		return g_taesooLibPath;
	return std::string("../");
}
