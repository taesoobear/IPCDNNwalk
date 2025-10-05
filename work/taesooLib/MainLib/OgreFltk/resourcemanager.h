#ifndef _RESOURCEMANAGER_H_
#define _RESOURCEMANAGER_H_

#if _MSC_VER>1000
#pragma once
#endif

#include "../../BaseLib/utility/namedmapsupport.h"
#include "../../BaseLib/utility/util.h"

//! Motion등의 Resource관리하는 class의 부모 클래스
/*!
Motion등의 리소스를 구분할때 외부적으로는 파일이름 등 string을 사용한다. 이를 identifier라 하고 줄여서 ID라한다.
리소스 구분시 프로그램 내부적으로는 integer를 사용한다. 이를 ResourceHandle이라 하고 약어로 h를 사용한다.
모든 리소스의 ResourceHandle은 OneTimeSceneInit()에서 program전체에 걸쳐 유일하게 결정된다.
리소스는 필요에 따라 Load된다. (On demand lazy loading with explicit loading,unloading function)
일반적인 파일구조 - resource/motion.lua
*/
#pragma warning (disable: 4786)
template <class T>
class ResourceManager
{
public:
	//! 동적을 생성 권장한다. 실제 리소스는 생성하지 않는다. 리소스 array는 NULL로 초기화 된다.
	ResourceManager(const char *szListFileName);
	virtual ~ResourceManager();

	//! ResourceID가 존재하는지 알아본다.
	inline bool Find(std::string &name) { if(GetNumResource()==0) return false; int handle; handle=m_namedmapResourceHandle[name]; return (std::string(m_aResourceFileName[handle])==name);};

	//! ResourceID(string)로 부터 ResourceHandle을 얻는다.
	inline ResourceHandle GetHandle(std::string const&name)
	{
		namedmapInt::iterator i;
		i=m_namedmapResourceHandle.find(name);
		if(i==m_namedmapResourceHandle.end())
			return HANDLE_NONE;
		return i->second;
	};
	inline ResourceHandle GetHandle(const char *name)		{	return GetHandle(std::string(name)); };

	ResourceHandle addResource(const char* szMotion);


	//! ResourceHandle로 부터 ResourceID(string)을 얻는다.
	inline const char* GetName(ResourceHandle handle)		{ASSERT(handle>=0 && handle<m_aResourcePtr.size());return m_aResourceFileName[handle];};
	inline const char* GetFullName(ResourceHandle handle)	{ASSERT(handle>=0 && handle<m_aResourcePtr.size());return m_aResourceFileFullName[handle];};
	inline const char* GetDefaultPath()						{ return m_szDefaultPath;};
	TString const& getDefaultPath()							{ return m_szDefaultPath;}
	inline int GetNumResource(){return m_aResourcePtr.size();};

protected:

	//! ResourceHandle로 부터 Resource*를 얻는다.
	inline T* GetResourcePtr(ResourceHandle handle){ return m_aResourcePtr[handle];};
	//! Handle에 해당하는 Resource*를 setting한다. 일반적으로 Resource를 새로 생성한후 call한다.
	void SetResourcePtr(ResourceHandle handle, T* pResource) { m_aResourcePtr[handle]=pResource;};

	void SetName(int i, const char* name);
	void Release();

	std::vector<T*> m_aResourcePtr;
	//! GetHandle에 사용됨
	namedmapInt m_namedmapResourceHandle;
	//! GetName에 사용됨
	TStrings m_aResourceFileName;
	TStrings m_aResourceFileFullName;
	std::vector<TStrings> m_aOptions;

	TString m_szListFileName;
	TString m_szDefaultPath;
};

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T> ResourceManager<T>::ResourceManager(const char *szListFileName)
{
	m_szListFileName= szListFileName;
}
template <class T>
ResourceManager<T>::~ResourceManager()
{
	Release();
}
template <class T>
ResourceHandle ResourceManager<T>::addResource(const char* szMotion)
{
	int newResource=m_aResourcePtr.size();
	m_aResourcePtr.resize(m_aResourcePtr.size()+1);
	m_aResourcePtr[newResource]=NULL;

	m_aResourceFileName.resize(newResource+1);
	m_aResourceFileFullName.resize(newResource+1);
	m_aOptions.resize(newResource+1);

	TString szMot=szMotion;
#ifdef _MSC_VER
	SetName(newResource, szMot.toUpper());
#else
	SetName(newResource, szMot);
#endif

	return (ResourceHandle)newResource;
}



template <class T>
void ResourceManager<T>::SetName(int i, const char* token)
{
	m_aResourceFileName[i]=token;
	if(token[0]=='.' || token[0]=='/' || token[1]==':')	// full path given
		m_aResourceFileFullName[i]=token;
	else	// use default path
		m_aResourceFileFullName[i].format("%s%s",m_szDefaultPath.ptr(), token);
	m_namedmapResourceHandle[std::string(m_aResourceFileName[i])]=i;
}

template <class T>
void ResourceManager<T>::Release()
{
#ifdef _DEBUG
	// 생성을 여기서 하지 않았으므로 삭제도 여기서 하지 않는다. 상속한 클래스에서 삭제바람
	for(int i=0;i<m_aResourcePtr.size(); i++)
	{
		ASSERT(m_aResourcePtr[i]==NULL);
	}
#endif
	m_aResourcePtr.clear();
	m_namedmapResourceHandle.clear();
}

#endif
