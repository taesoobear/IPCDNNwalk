// MotionManager.h: interface for the MotionManager class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTIONMANAGER_H__E5E62DED_0E49_41A1_966D_EF2ABCDCE0D5__INCLUDED_)
#define AFX_MOTIONMANAGER_H__E5E62DED_0E49_41A1_966D_EF2ABCDCE0D5__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "resourcemanager.h"
#include "../../BaseLib/motion/Motion.h"
class MotionLoader;
class MotionManager : public ResourceManager<MotionLoader>
{
public:
	MotionManager(const char *szMotionFileName);
	MotionManager();
	virtual ~MotionManager();

	void Init();

	// Skeleton 관리
	MotionLoader* GetMotionLoaderPtr(const char* szMotion);
	MotionLoader* GetMotionLoaderPtr(ResourceHandle hMotion);

	// 아래 함수들은 왠만하면 사용하지 말것. 
	// 대신 RE::createMotionLoader 함수를 사용할 것.
	// 본 프로그램은 skeleton이 항상 reference를 가정하기 때문에, 
	// skeleton을 지울때 주의가 필요하다. 일반적인 경우 Motion.lua를 보고 자동으로 loading과 delete가 수행된다.
	static MotionLoader* createMotionLoader(const TString& filename);
	MotionLoader* createMotionLoader(ResourceHandle hMotion, const TString& filename);
	ResourceHandle createMotionLoaderExt(const char* key, MotionLoader* input);// input will be adopted and deleted later.


	// Motion관리.
	Motion* createMotion(const char* identifier);
	Motion* getMotion(const char* identifier);

private:

	std::map<TString, Motion*, cmpTString> namedmapMotion;

};

#endif // !defined(AFX_MOTIONMANAGER_H__E5E62DED_0E49_41A1_966D_EF2ABCDCE0D5__INCLUDED_)
