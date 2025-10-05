
#include "stdafx.h"
#include "../BaseLib/image/Image.h"
#include "../BaseLib/image/ImageProcessor.h"
#include "../BaseLib/motion/MotionRetarget.h"
#include "../BaseLib/motion/MotionUtil.h"
#include "../BaseLib/motion/viewpoint.h"
#include "../BaseLib/motion/ConstraintMarking.h"
#include "../BaseLib/motion/postureip.h"
#include "../BaseLib/math/Operator.h"
#include "../BaseLib/math/conversion.h"
#include "../BaseLib/math/Filter.h"
#include "../BaseLib/math/matrix3.h"
#include "../BaseLib/motion/Liegroup.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/utility/QPerformanceTimer.h"
#include "../BaseLib/utility/tfile.h"
#include "../OgreFltk/MotionPanel.h"
#include "../OgreFltk/renderer.h"
#include "../OgreFltk/FltkRenderer.h"

#include "luna.h"


#ifdef _MSC_VER
#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#endif

TString getCurrentDirectory();

namespace BaselibLUA {
	// String manipulation 함수들.
	int str_left(lua_State* L) // str_left("asdfasdf", 4)=="asdf", str_left("afffasdf",-3)=="afffa"
	{
		lunaStack lua(L);
		TString in;
		int n;
		lua>>in>>n;
		lua<<in.left(n);
		return 1;
	}

	int str_right(lua_State* L) // str_right("asdfasdf", 4)=="asdf", str_right("asdfasdf",-2)=="dfasdf"
	{
		lunaStack lua(L);
		TString in;
		int n;
		lua>>in>>n;
		lua<<in.right(n);
		return 1;
	}

	int str_include(lua_State* L)	// str_include("asdfasdff", "asdf")=true
	{
		lunaStack lua(L);
		TString in, sub;
		lua>>in>>sub;
		if(in.findStr(0, sub)!=-1)
			lua<<true;
		else
			lua<<false;

		return 1;

	}

	int isFileExist(lua_State* L)
	{
		lunaStack lua(L);
		TString fn;
		lua>>fn;
		if(::IsFileExist(fn))
			lua<<true;
		else
			lua<<false;
		return 1;
	}

#define method(name) {#name, &name}

	static luaL_reg PostProcessGlue[] =
	{
		method(str_left),
		method(str_right),
		method(str_include),
		method(isFileExist),
		{NULL, NULL}
	};

}

#ifndef NO_OGRE
#include <Ogre.h>

#define OGRE_VOID(x) x
#define OGRE_PTR(x) x
#else
#define OGRE_VOID(x)
#define OGRE_PTR(x) return NULL
#endif
//#include <OgreViewport.h>
//#include <OgreSceneNode.h>
//#include <OgreSceneManager.h>
//#include <OgreEntity.h>
//#include <OgreOverlayManager.h>
#ifndef NO_OGRE
#define BEGIN_OGRE_CHECK try {
#define END_OGRE_CHECK	} catch ( Ogre::Exception& e ) {Msg::msgBox(e.getFullDescription().c_str());}

#include "OgreOverlayManager.h"
#include "OgreOverlayContainer.h"
#include "OgreOverlayElement.h"

namespace Ogre
{

	Ogre::v1::OverlayContainer* createContainer(int x, int y, int w, int h, const char* name) ;
	Ogre::v1::OverlayElement* createTextArea(const String& name, Real width, Real height, Real top, Real left, uint fontSize, const String& caption, bool show) ;
}

Ogre::v1::Overlay* createOverlay_(const char* name)
{
	return Ogre::v1::OverlayManager::getSingleton().create(name);
}

void destroyOverlay_(const char* name)
{
	Ogre::v1::OverlayManager::getSingleton().destroy(name);
}
void destroyOverlayElement_(const char* name)
{
	Ogre::v1::OverlayManager::getSingleton().destroyOverlayElement(name);
}
void destroyAllOverlayElements_()
{
	Ogre::v1::OverlayManager::getSingleton().destroyAllOverlayElements();
}
Ogre::v1::OverlayElement* createTextArea_(const char* name, double width, double height, double top, double left, int fontSize, const char* caption, bool show)
{
	return Ogre::createTextArea(name, width, height, top, left, fontSize, caption, show);
}

#endif
#include "mainliblua_wrap.h"
Viewpoint* RE_::getViewpoint()
{
  return RE::renderer().viewport().m_pViewpoint;
}
Viewpoint* RE_::getViewpoint(int n)
{
  return RE::renderer().viewport(n).m_pViewpoint;
}

std::string RE_::generateUniqueName()
{
  return std::string(RE::generateUniqueName().ptr());
}

Ogre::Item* RE_::createPlane2(const char* id, m_real width, m_real height, int xsegment, int ysegment, int texSegx, int texSegy)
{
  return RE::createPlane(id, width, height, xsegment, ysegment, texSegx, texSegy);
}

Ogre::Item* RE_::createPlane(const char* id, m_real width, m_real height, int xsegment, int ysegment)
{
  return createPlane2(id, width, height, xsegment, ysegment, 1, 1);
}

void RE_::setBackgroundColour(m_real r, m_real g, m_real b)
{
#ifndef NO_OGRE

  RE::renderer().setBackgroundColour((float)r,(float)g,(float)b);
#endif
}
void RE_::remove(PLDPrimSkin* p)
{
  printf("remove is deprecated. (Calling this is no longer needed as the object is owned by LUA).\n");
  //RE::remove(p);
}
PLDPrimSkin* RE_::createSkin2(const MotionLoader& skel, int typet)
{
  return RE::createSkin(skel, (RE::PLDPrimSkinType)typet);
}

PLDPrimSkin* RE_::createSkin3(const Motion& mot, int typet)
{
  return RE::createSkin(mot, (RE::PLDPrimSkinType)typet);
}

int FlGenShortcut(const char* s)
{
#ifdef NO_GUI
	return 0;
#else
	TString ss(s);

	TString token;
	int shortc=0;
	for(int start=0, end=ss.length(); start!=end; )
		{
			ss.token(start, "+", token) ;

			if(token=="FL_ALT")
				shortc+=FL_ALT;
			else if(token=="FL_CTRL")
				shortc+=FL_CTRL;
			else
				shortc+=token[0];
		}
	return shortc;
#endif
}
matrixn MotionDOF_calcDerivative(MotionDOF const& motionDOF, double frameRate )
{
	Msg::verify( motionDOF.mInfo.numSphericalJoint()==1,"numSphericalJoint!=-1");

	return MotionDOF_calcDerivative((const_cast<MotionDOF&>(motionDOF))._matView(), frameRate);
}

matrixn MotionDOF_calcDerivative(matrixn const& motionDOF, double frameRate )
{
	matrixn dmotionDOF;
	dmotionDOF.setSize(motionDOF.rows(), motionDOF.cols());
	for(int  i=0; i<motionDOF.rows()-1; i++)
	{
		//-- 아래 코드는 MotionDOF.derivPose(mot:row(i), mot:row(i+1))과 동일
		auto dmotionDOF_i=dmotionDOF.row(i);
		dmotionDOF_i.sub(motionDOF.row(i+1), motionDOF.row(i)); //-- forward difference
		dmotionDOF_i*=frameRate;

		auto T=motionDOF.row(i).toTransf(0);
		//-- body velocity
		auto V=Liegroup::twist( T, motionDOF.row(i+1).toTransf(0), 1/frameRate);
		dmotionDOF_i.setVec3(0, V.V());
		dmotionDOF_i.set(3,0); //-- unused
		dmotionDOF_i.setVec3(4, V.W());
	}
	dmotionDOF.row(dmotionDOF.rows()-1).assign(dmotionDOF.row(dmotionDOF.rows()-2));
	return dmotionDOF;
}
