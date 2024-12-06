// MotionManager.cpp: implementation of the MotionManager class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionManager.h"
#include "../BaseLib/motion/MotionLoader.h"
#include "../BaseLib/motion/BVHLoader.h"
#include "../BaseLib/motion/ASFLoader.h"
//#include "../BaseLib/motion/TRCLoader.h"
#ifdef INCLUDE_VLOADER
#include "../BaseLib/motion/Vloader/VLoader.h"
#endif
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../WrapperLua/LUAwrapper.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MotionManager::MotionManager(const char *szMotionFileName)
: ResourceManager<MotionLoader>(szMotionFileName)
{

	// 실제 로딩은 처음으로 GetMotionLoaderPtr을 했을때 이루어진다.
	Init();

}
MotionManager::MotionManager()
: ResourceManager<MotionLoader>("")
{

	// 실제 로딩은 처음으로 GetMotionLoaderPtr을 했을때 이루어진다.
	Init();

}

MotionManager::~MotionManager()
{
	MotionLoader* pLoader;
	for(int i=0; i<m_aResourcePtr.size(); i++)
	{
		if(pLoader=GetResourcePtr(i))
		{
			delete pLoader;
		}
		SetResourcePtr(i,NULL);
	}
}

MotionLoader* MotionManager::GetMotionLoaderPtr(const char* szMotion) 
{
	ResourceHandle handle;
#ifdef _MSC_VER
	TString upr=szMotion;
	handle=GetHandle(upr.toUpper());
#else
	handle=GetHandle(szMotion);
#endif
	if(handle==HANDLE_NONE)
	{
		handle=addResource(szMotion);
	}
	
	return GetMotionLoaderPtr(handle);
}


MotionLoader* MotionManager::GetMotionLoaderPtr(ResourceHandle hMotion) 
{
	if(!GetResourcePtr(hMotion))
	{
		const char* name=GetName(hMotion);
		TRACE("Loading skeleton %s\n", name);
			
		SetResourcePtr(hMotion, createMotionLoader(hMotion, GetFullName(hMotion)));

		GetResourcePtr(hMotion)->m_handle=(int)hMotion;

	}

	return GetResourcePtr(hMotion);
}


ResourceHandle MotionManager::createMotionLoaderExt(const char* key, MotionLoader* input)
{
	// input will be adopted and deleted later.
	ResourceHandle hMotion=addResource(key);

	SetResourcePtr(hMotion, input)	;

	return hMotion;
}

static void loadEE(MotionLoader* loader, TString filename)
{
	// translation file
	filename=filename.left(filename.length()-3);
	filename+="EE";
	if(IsFileExist(filename))
	{
		printf("Reading %s\n", filename.ptr());
		loader->readJointIndex(filename);
	}

	TRACE("done\n");
}
MotionLoader* MotionManager::createMotionLoader(ResourceHandle hMotion, const TString& _filename)
{
	MotionLoader* ret=NULL;
	if(m_aOptions[hMotion].size())
	{
		TString filename=m_aOptions[hMotion][1];
		m_aResourceFileFullName[hMotion].format("%s%s",m_szDefaultPath.ptr(), filename.ptr());

		//if(filename.right(3).toUpper()=="TRC")
		//	ret= new TRCLoader(m_szDefaultPath+filename, m_aOptions[hMotion][2]);
		if(filename.right(3).toUpper()=="BVH")
			ret= new BVHLoader(m_szDefaultPath+filename, m_aOptions[hMotion][2]);
		//else if(filename.right(3).toUpper()=="ASF")
		//	ret= new ASFLoader(m_szDefaultPath+filename, m_aOptions[hMotion][2]);
		else if(filename.right(3).toUpper()=="MOT")
			ret= new MotionLoader(m_szDefaultPath+filename, m_aOptions[hMotion][2]);
		else if(filename.right(3).toUpper()=="WRL"||
				filename.right(7).toUpper()=="WRL.BIN")
			ret= new VRMLloader(m_szDefaultPath+filename);
#ifdef INCLUDE_VLOADER
		else if(filename.right(3).toUpper()=="VSK")
			ret= new VLoader(m_szDefaultPath+filename);
#endif
		else 
		{
			printf("failed to create %s\n", filename.ptr());
			ASSERT(0);
		}

		loadEE(ret, m_aResourceFileFullName[hMotion]);
	}
	else
		ret= createMotionLoader(_filename);

	return ret;
}

MotionLoader* MotionManager::createMotionLoader(const TString& filename)
{
	MotionLoader* ret=NULL;
	if(filename.right(3).toUpper()=="BVH")
		ret= new BVHLoader(filename);
	else if(filename.right(3).toUpper()=="ASF")
		ret= new ASFLoader(filename);
	else if(filename.right(3).toUpper()=="MOT" || filename.right(4).toUpper()=="SKEL")
		ret= new MotionLoader(filename);
	else if(filename.right(3).toUpper()=="WRL"||
				filename.right(7).toUpper()=="WRL.BIN")
		ret= new VRMLloader(filename);
#ifdef INCLUDE_VLOADER
	else if(filename.right(3).toUpper()=="VSK")
		ret= new VLoader(filename);
#endif
	else
	{
		printf("failed to create %s\n", filename.ptr());
		ASSERT(0);
	}

	if(ret)
		loadEE(ret, filename);
	return ret;
}

Motion* MotionManager::createMotion(const char* identifier)
{
	Motion* newMot=new Motion();
	namedmapMotion[TString(identifier)]=newMot;
	newMot->SetIdentifier(identifier);
	return newMot;
}

Motion* MotionManager::getMotion(const char* identifier)
{
	return namedmapMotion[TString(identifier)];
}


void MotionManager::Init()
{
	if(m_szListFileName.length()==0)
	{
	}
	else if(IsFileExist(m_szListFileName)){
		LUAwrapper L;
		L.dofile(m_szListFileName);

		//L.printStack();
	//	m_szDefaultPath=L.getValue<const char*>("default_path");
		L.getglobal("default_path");
		//L.printStack();
		L>>m_szDefaultPath;
		//printf("pop\n");
		//L.printStack();

#ifdef _MSC_VER
		m_szDefaultPath.makeUpper();
#endif
	
		lunaStack l(L.L);
		l.getglobal("resource");
		assert(lua_istable(L.L, l.gettop()));
		int resource=l.gettop();
		int numResource=l.arraySize(resource);
		
		m_aResourcePtr.resize(numResource);

		for(int i=0; i<numResource; i++)
			m_aResourcePtr[i]=NULL;

		m_aResourceFileName.resize(numResource);
		m_aResourceFileFullName.resize(numResource);
		m_aOptions.resize(numResource);

		TString token;
		for(int i=1; i<=numResource; i++)
			{
				L.gettable(resource, i);
				if (lua_istable(L.L, -1))
				{
					int options_tbl=L.gettop();
					TStrings options;
					int nopt=L.arraySize(-1);
					options.resize(nopt);
					for(int j=1; j<=nopt; j++)
					{
						L.gettable(options_tbl, j);
						L>>options[j-1];
					}
					m_aOptions[i-1]=options;
					L.pop();
					token=options[0];
				}
				else
				{
					L>> token;
				}
		
#ifdef _MSC_VER
				token.makeUpper();
#endif
				SetName(i-1, token);
			}		

		//L.printStack();
	}
	else{
		printf("Warning! Motion List File %s not found\n",m_szListFileName.ptr());
	}
}
