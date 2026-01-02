// MainlibPython.cpp : Defines the entry point for the DLL application.
//

#include "stdafx.h"
#include "../../BaseLib/math/tvector.h"
#include "../../BaseLib/math/vector3.h"
#include "../../BaseLib/math/quater.h"
#include "../../BaseLib/math/quaterN.h"
#include "../../BaseLib/math/hyperMatrixN.h"
#include "../../BaseLib/math/conversion.h"
#include "../../BaseLib/math/BSpline.h"
#include "../../BaseLib/image/ImagePixel.h"
#include "../../BaseLib/motion/VRMLloader.h"
#include "../../BaseLib/motion/viewpoint.h"
#include "../../BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "../../BaseLib/motion/MotionRetarget.h"
#include "../../BaseLib/utility/QPerformanceTimer.h"
#include "../../BaseLib/utility/BinaryFile.h"
#include "../../MainLib/OgreFltk/MotionManager.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/Mesh.h"
#include "../../MainLib/OgreFltk/FltkAddon.h"
#include "../../MainLib/OgreFltk/FltkScrollPanel.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#include "../../MainLib/WrapperLua/luna.h"
#include "../../MainLib/WrapperLua/LUAwrapper.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "../../MainLib/WrapperLua/ThreadedScript.h"
#include "../../PhysicsLib/luna_physics.h"
#include "../../MainLib/WrapperLua/mainliblua_wrap.h"
#include "../../BaseLib/motion/Terrain.h"
#include "../../MainLib/Ogre/intersectionTest.h"
#include "../../BaseLib/motion/MotionUtil.h"
#include "../../BaseLib/motion/LimbIKsolver.h"
#include "../../BaseLib/motion/LimbIKsolver2.h"
#include "../../BaseLib/motion/HandIKsolver.h"
#include "../../BaseLib/motion/COM_IKsolver.h"
#include "../../BaseLib/motion/IK_sdls/NodeWrap.h"
#include "../../BaseLib/motion/Liegroup.h"
#include "../../BaseLib/math/Operator.h"
#include "../../PhysicsLib/physicsLib.h"
#include "../../PhysicsLib/physicsLib.h"
#include "../../ClassificationLib/motion/Locomotion/vehicle.h"
#define EXCLUDE_UT_SIM
#define EXCLUDE_AIST_SIM
#ifndef EXCLUDE_UT_SIM
#include "../../PhysicsLib/DynamicsSimulator_UT.h"
#endif
#ifndef EXCLUDE_AIST_SIM
#include "../../PhysicsLib/AIST_implementation/DynamicsSimulator_impl.h"
#include "../../PhysicsLib/AIST_implementation/DynamicsSimulator_AIST_penalty.h"
#endif
#include "../../PhysicsLib/TRL/DynamicsSimulator_TRL_penalty.h"
#include "../../PhysicsLib/TRL/DynamicsSimulator_TRL_LCP.h"
//#include "../../PhysicsLib/TRL/DynamicsSimulator_TRL_softbody.h"
#include "../../PhysicsLib/TRL/DynamicsSimulator_TRL_QP.h"
#ifndef EXCLUDE_RBDL_SIMULATOR
#ifdef Success
#undef Success
#endif
#include "../../PhysicsLib/rbdl/DynamicsSimulator_Trbdl_penalty.h"
#include "../../PhysicsLib/rbdl/DynamicsSimulator_Trbdl_LCP.h"
#include "../../PhysicsLib/rbdl/DynamicsSimulator_Trbdl_impulse.h"
#include "../../PhysicsLib/rbdl/DynamicsSimulator_Trbdl_QP.h"
#endif
#include "../../PhysicsLib/CollisionDetector.h"
//#include "RMatrixLUA.h"

#include "../../PhysicsLib/DynamicsSimulator_penaltyMethod.h"
#include "../../BaseLib/motion/InertiaCalculator.h"
#include "../../BaseLib/math/OperatorStitch.h"
#include "../../PhysicsLib/clapack_wrap.h"
#include "../../PhysicsLib/SDRE.h"
#ifndef EXCLUDE_GMBS_SIM
#include "../../PhysicsLib/GMBS_implementation/rmatrix3j.h"
#endif
class GrahamScan;
#include "../../PhysicsLib/CollisionDetector/CollisionDetector_libccd.h"

namespace OpenHRP {
	class DynamicsSimulator_impl;
	class DynamicsSimulator_UT;
	class DynamicsSimulator_AIST_penalty;
	class DynamicsSimulator_TRL;
	class DynamicsSimulator_TRL_LCP;
	class DynamicsSimulator_TRL_QP;
	class DynamicsSimulator_TRL_penalty;
	class CollisionDetector_libccd;
	OpenHRP::CollisionDetector* createCollisionDetector_bullet();
	OpenHRP::CollisionDetector* createCollisionDetector_fcl();
	OpenHRP::CollisionDetector* createCollisionDetector_gjk();
	OpenHRP::CollisionDetector* createCollisionDetector_libccd();
	//OpenHRP::CollisionDetector* createCollisionDetector_libccd_LBS();
	//OpenHRP::CollisionDetector* createCollisionDetector_libccd_merged();
	bool CollisionCheck(OpenHRP::CollisionDetector &s, OpenHRP::CollisionSequence & collisions, std::string chekmesh, std::string skipmesh);
}
namespace RE_ {
	bool renderOneFrame(bool check);
}
#include "../../PhysicsLib/convexhull/graham.h"
void VRMLloader_checkMass(VRMLloader& l);
#include "Wrapper.hpp"
#include "_PythonExtendWin.h"
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>

using namespace pybind11;

#if PY_MAJOR_VERSION >= 3
#define IS_PY3K
#define PyString_Check PyBytes_Check
#define PyString_AsString PyBytes_AsString
#endif

#ifdef _MSC_VER
#ifdef _MANAGED
#pragma managed(push, off)
#endif
 
BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
    return TRUE;
}

#ifdef _MANAGED
#pragma managed(pop)
#endif

#endif

#include "_MainlibPython.h"

#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"

#ifndef NO_OGRE
#ifdef None
#undef None
#endif
#include <Ogre.h>

#define OGRE_VOID(x) x
#define OGRE_PTR(x) x
#else
#define OGRE_VOID(x)
#define OGRE_PTR(x) return NULL
#endif
 // defined in main.cpp in the latest sample_python
void _createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, OgreRenderer* _renderer);
void createMainWin(int, int, int ,int, float);
void releaseMainWin();
void _createInvisibleMainWin();
void createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, const char* configFileName, const char* plugins_file, const char* ogre_config);
void showMainWin();
void startMainLoop();
PythonExtendWin* getPythonWin();
bool hasPythonWin();
#ifdef NO_GUI
// for multi-threading. use ThreadedScript instread.
//PythonExtendWin* getAdditionalPythonWin(PythonExtendWin* parent, int worker_id);
#endif


void RE_outputRaw(const char* key, const char* output, int i);
void RE_dumpOutput(TStrings& output, int i);
void RE_outputEraseAll(int i);
#ifndef NO_OGRE
#define BEGIN_OGRE_CHECK try {
#define END_OGRE_CHECK	} catch ( Ogre::Exception& e ) {Msg::msgBox(e.getFullDescription().c_str());}

#include "OgreOverlay.h"
#include "OgreOverlayManager.h"
#include "OgreOverlayContainer.h"
#include "OgreOverlayElement.h"
#include <OgreItem.h>
#include <OgreEntity.h>
#include <OgreAxisAlignedBox.h>

namespace Ogre
{

	Ogre::v1::OverlayContainer* createContainer(int x, int y, int w, int h, const char* name) ;
	Ogre::v1::OverlayElement* createTextArea(const String& name, Ogre::Real width, Ogre::Real height, Ogre::Real top, Ogre::Real left, uint fontSize, const String& caption, bool show) ;
}

Ogre::v1::Overlay* createOverlay_(const char* name);
void destroyOverlay_(const char* name);
void destroyOverlayElement_(const char* name);
void destroyAllOverlayElements_();
Ogre::v1::OverlayElement* createTextArea_(const char* name, double width, double height, double top, double left, int fontSize, const char* caption, bool show);
#endif


void viewLock()
{
	((FltkToolkitRenderer&)RE::FltkRenderer()).m_bLockViewpoint.value(true);
	((FltkToolkitRenderer&)RE::FltkRenderer()).redraw();
	((FltkToolkitRenderer&)RE::FltkRenderer()).onCallback(&((FltkToolkitRenderer&)RE::FltkRenderer()).m_bLockViewpoint,0);		
}
void viewUnlock()
{
	((FltkToolkitRenderer&)RE::FltkRenderer()).m_bLockViewpoint.value(false);
	((FltkToolkitRenderer&)RE::FltkRenderer()).redraw();
	((FltkToolkitRenderer&)RE::FltkRenderer()).onCallback(&((FltkToolkitRenderer&)RE::FltkRenderer()).m_bLockViewpoint,0);
}


#include "../../BaseLib/motion/ASFLoader.h"
#include "../../BaseLib/motion/BVHLoader.h"

std::string tail(std::string const& source, size_t const length) {
  if (length >= source.size()) { return source; }
  return source.substr(source.size() - length);
} 
#include <algorithm>
#include <string>

std::string toUpper(std::string const& in){
	std::string str=in;
	std::transform(str.begin(), str.end(),str.begin(), ::toupper);
	return str;
}
		struct matrixn_
		{
			// does not copy memory.
			static PyObject* ref(matrixn const& v)
			{
				npy_intp dims[2];
				dims[0]=v.rows();
				dims[1]=v.cols();
				npy_intp strides[2];
				strides[0]=sizeof(double)*v._getStride();
				strides[1]=sizeof(double);
				double* vv=&v[0][0];
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 2, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};


#define RETURN_REFERENCE return_value_policy::reference
#define TAKE_OWNERSHIP return_value_policy::take_ownership
class impl_luna__interface_PLDPrimSkin
{
	public:
			static void startAnim(PLDPrimSkin& s)
			{
				s.m_pTimer->StartAnim();
			}
			static void initAnim(PLDPrimSkin& s, float curframe, float endframe, float returnframe)
			{
				s.m_pTimer->InitAnim(curframe, endframe, returnframe);
			}
			static void stopAnim(PLDPrimSkin& s)
			{
				s.m_pTimer->StopAnim();
			}
			static void loop(PLDPrimSkin& s, bool b)
			{
				s.m_pTimer->loop(b);
			}
			static bool isPlaying(PLDPrimSkin& s)
			{
				return s.m_pTimer->IsPlaying();
			}
			static float calcCurFrameFromInterpolator(PLDPrimSkin& s, int iframe)
			{
				return s.m_pTimer->calcCurFrameFromInterpolator(iframe);
			}
			static int numFrames(PLDPrimSkin& s)
			{
				return s.m_pTimer->getNumFrameFromInterpolator();
			}
			static void setFrameTime(PLDPrimSkin& s, float fFrameTime)
			{
				InterpolatorLinear* apip=((InterpolatorLinear*)(s.m_pTimer->GetFirstInterpolator()));
				apip->init(fFrameTime);
			}
			static float totalTime(PLDPrimSkin& s)
			{
				return s.m_pTimer->getTotalTimeFromInterpolator();
			}
			static void setRotation(PLDPrimSkin& s,quater const& q)
			{
				#ifndef NO_OGRE
				s.m_pSceneNode->setOrientation(ToOgre(q));
				#endif
			}
			static quater getRotation(PLDPrimSkin& s)
			{
				#ifndef NO_OGRE
				return ToBase(s.m_pSceneNode->getOrientation());
				#else
				return quater(1,0,0,0);
				#endif
			}
};
class impl_luna__interface_Viewpoint 
{
	public:
			inline static void update(Viewpoint & view)
			{
				view.CalcHAngle();
				view.CalcVAngle();
				view.CalcDepth();
			}

			inline static void setClipDistances(Viewpoint& view, m_real fnear, m_real ffar)
			{
#ifndef NO_OGRE

				double nn=fnear;
				double ff=ffar;
				RE::renderer().viewport().mCam->setNearClipDistance(Ogre::Real(fnear));
				RE::renderer().viewport().mCam->setFarClipDistance(Ogre::Real(ffar));
#endif
			}
			inline static void setOrthographicMode(Viewpoint& view, bool isOrtho)
			{
#ifndef NO_OGRE
				RE::renderer().viewport().setOrthographicMode(isOrtho);
#endif
			}
			inline static void setFOVy(Viewpoint& view, m_real degree)
			{
#ifndef NO_OGRE

//#ifdef OCULUS // do this outside of this function. This won't work anymore.
//				for (int i=0,n=RE::renderer().numViewport();i<n; i++)
//					{
//						printf("%f\n", degree);
//				RE::renderer().viewport(i).mCam->setFOVy(Ogre::Radian(Ogre::Degree(degree)));
//			}
//#else
				RE::renderer().viewport().mCam->setFOVy(Ogre::Radian(Ogre::Degree(degree)));
//#endif
#endif
			}
			inline static void setNearClipDistance(Viewpoint& view, m_real dist)
			{
#ifndef NO_OGRE
//#ifdef OCULUS // do this outside of this function. This won't work anymore.
//
//				for (int i=0,n=RE::renderer().numViewport();i<n; i++)
//				{
//					RE::renderer().viewport(i).mCam->setNearClipDistance(Ogre::Real(dist));
//				}
//#else
				RE::renderer().viewport().mCam->setNearClipDistance(dist);
//#endif
#endif
			}
};
class impl_luna__interface_SceneManager 
{
	public:
            static void setFog(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
            {
#ifndef NO_OGRE
                pmgr->setFog(Ogre::FOG_LINEAR,Ogre::ColourValue(r,g,b), a, min, max);
#endif
            }
            static void setFogExponential(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max)
            {
#ifndef NO_OGRE
                pmgr->setFog(Ogre::FOG_EXP2,Ogre::ColourValue(r,g,b), a, min, max);
#endif
            }
            static void setFogNone(Ogre::SceneManager* pmgr)
            {
#ifndef NO_OGRE
                pmgr->setFog(Ogre::FOG_NONE,Ogre::ColourValue(0,0,0), 0, 0, 0);
#endif
            }
            static Ogre::Item* createEntity(Ogre::SceneManager* pmgr, const char* id, const char* mesh)
            {
#ifndef NO_OGRE
                BEGIN_OGRE_CHECK
                    return RE::_createEntity(mesh);
                END_OGRE_CHECK
#else 
                    return NULL;
#endif
            }

            static Ogre::SceneNode* getSceneNode(Ogre::SceneManager* pmgr, const char* id)
            {
#ifndef NO_OGRE
                BEGIN_OGRE_CHECK
                    return RE::getSceneNode(id);
                END_OGRE_CHECK
#else 
                    return NULL;
#endif

            }

            static Ogre::Light* createLight(Ogre::SceneManager* pmgr, const char* id)
            {
#ifndef NO_OGRE
                BEGIN_OGRE_CHECK
                    return pmgr->createLight();

                END_OGRE_CHECK
#else 
                    return NULL;
#endif


            }


            static void setSkyBox(Ogre::SceneManager* pmgr, bool enable, const char* materialName)
            {printf("warning! setSkyBox not ported to ogre2 yet\n");}
            static bool hasSceneNode(Ogre::SceneManager* pmgr, const char * name) 
            {
#if !defined (NO_GUI)                                         
                return (bool)RE::getSceneNode(name);
#else
                return true;
#endif
            }
            static int getShadowTechnique(Ogre::SceneManager* pmgr)
            {
                return 18;
            }
            static void setShadowTechnique(Ogre::SceneManager* pmgr, int i)
            {
            }
};

// wrapper functions..

void vector3_assign(vector3& l, WRAP_PY::list ll) 
{
	if(len(ll)!=3) throw std::range_error("vector3_assign");
	l.x=ll[0].cast<double>();
	l.y=ll[1].cast<double>();
	l.z=ll[2].cast<double>();
}

void quater_assign(quater& l, WRAP_PY::list ll) 
{
	if(len(ll)!=4) throw std::range_error("quater_assign");
	l.x=ll[0].cast<double>();
	l.y=ll[1].cast<double>();
	l.z=ll[2].cast<double>();
	l.w=ll[3].cast<double>();
}

static std::string quater_output(quater& q)
{
	return q.output();
}

static std::string vector3_output(vector3& q)
{
	return q.output();
}



std::string get_Widget_mId(FlLayout::Widget const& w)
{
	return w.mId.ptr();
}

std::string get_Widget_mType(FlLayout::Widget const& w)
{
	return w.mType.ptr();
}
/*
#if PY_VERSION_HEX >= 0x03000000
void *
#else
void
#endif
initialize()
{
  import_array();
#if PY_VERSION_HEX >= 0x03000000
  return NULL;
#endif
}
*/

PYBIND11_MODULE(libmainlib, mainlib)
{
	if (_import_array() < 0) {
        throw std::runtime_error("NumPy initialization failed");
    }
	mainlib.doc()="taesoolib: libmainlib";
	void (*createMainWin1)(int, int,int, int, float)=&createMainWin;
	void (*createMainWin2)(int, int,int, int, float, const char*, const char*, const char*)=&createMainWin;
	//initialize(); // import_array
	//boost::python::numpy::array::set_module_and_type("numpy", "ndarray");
	mainlib.def("showMainWin", showMainWin)
#ifndef NO_GUI
		.def ("motionPanel", &RE::motionPanel, RETURN_REFERENCE)
#endif
		.def("viewLock", viewLock)
		.def("viewUnlock", viewUnlock)
		.def("getOgreVersionMinor", &RE::getOgreVersionMinor)
		.def("buildEdgeList", &RE::buildEdgeList) // 1458
		.def("_createMainWin", _createMainWin)
		.def("createMainWin", createMainWin1)
		.def("createMainWin", createMainWin2)
		.def("releaseMainWin", releaseMainWin)
		.def("_createInvisibleMainWin", _createInvisibleMainWin)
		.def("ThreadScriptPoolWithPhysicsLib", &ThreadScriptPoolWithPhysicsLib)
		.def("ThreadedScriptWithPhysicsLib", &ThreadedScriptWithPhysicsLib)
		//.def("startMainLoop", startMainLoop)
		.def("getPythonWin", getPythonWin, RETURN_REFERENCE)
		.def("hasPythonWin", hasPythonWin)
#ifdef NO_GUI
		//.def("getAdditionalPythonWin", getAdditionalPythonWin, RETURN_REFERENCE)
#endif
		.def("map", &sop::map)
		.def("clamp", [](double i, double a, double b){ return MIN(MAX(i,a), b);})
		.def("clampMap", (m_real (*)(m_real t, m_real min, m_real max, m_real v1, m_real v2))&sop::clampMap) // 1458
		.def("sigmoid", (m_real (*)(m_real x))&sop::sigmoid)          // 1458
		.def("LUsolve", &m::LUsolve)
		.def("ogreRootSceneNode", (Ogre ::SceneNode * (*)())&RE::ogreRootSceneNode, RETURN_REFERENCE) // 1446
		.def("createFullbodyIk_LimbIK", (MotionUtil ::FullbodyIK * (*)(MotionLoader& skeleton, std::vector<MotionUtil::Effector>& effectors))&MotionUtil::createFullbodyIk_LimbIK, return_value_policy::reference) // 1463
		.def("createFullbodyIk_MotionDOF_MultiTarget", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors))&MotionUtil::createFullbodyIk_MotionDOF_MultiTarget, return_value_policy::reference) // 1463
		.def("createFullbodyIk_MotionDOF_MultiTarget_lbfgs", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, std::vector<MotionUtil::RelativeConstraint> &con))&MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs, return_value_policy::reference) // 1463
		.def("createFullbodyIk_MotionDOF_MultiTarget_lbfgs", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info))&MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs, return_value_policy::reference) // 1463
		.def("createFullbodyIkDOF_limbIK", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee))&MotionUtil::createFullbodyIkDOF_limbIK, return_value_policy::reference) // 1463
		.def("createFullbodyIkDOF_limbIK", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee, bool bReversed))&MotionUtil::createFullbodyIkDOF_limbIK, return_value_policy::reference) // 1463
		.def("createFullbodyIkDOF_limbIK_straight", (MotionUtil ::FullbodyIK_MotionDOF * (*)(MotionDOFinfo const& info, std::vector<MotionUtil::Effector>& effectors, Bone const& left_knee, Bone const& right_knee))&MotionUtil::createFullbodyIkDOF_limbIK_straight, return_value_policy::reference) // 1463
		.def("setLimbIKParam_straight", (void (*)(MotionUtil::FullbodyIK_MotionDOF* ik, bool bStraight))&MotionUtil::setLimbIKParam_straight) // 1458
		.def("usleep", (void (*)(int usec))&RE::usleep)              // 1446
		.def("ogreSceneManager", (Ogre ::SceneManager * (*)())&RE::ogreSceneManager, RETURN_REFERENCE) // 1446
		.def("_output", (void (*)(const char* key, const char* output, int i))&RE_outputRaw) // 1458
		.def("useSeperateOgreWindow", (bool (*)())&RE::useSeperateOgreWindow) // 1446
		.def("numOutputManager", (int (*)())&RE::numOutputManager)   // 1446
		.def("motionLoader", (MotionLoader * (*)(const char* name))&RE::motionLoader, RETURN_REFERENCE) // 1446
		.def("createMotionLoader", (MotionLoader * (*)(const char* name, const char* key))&RE::createMotionLoader, RETURN_REFERENCE) // 1446
		.def("createMotionLoaderExt_cpp", [](const char* name)->MotionLoader*{ return MotionManager::createMotionLoader(name);}, RETURN_REFERENCE) // 1446
		.def("createChildSceneNode", (Ogre ::SceneNode * (*)(Ogre::SceneNode* parent, const char* child_name))&RE::createChildSceneNode, RETURN_REFERENCE) // 1446
		.def("createEntity", (Ogre ::SceneNode * (*)(const char* id, const char* filename))&RE::createEntity, RETURN_REFERENCE) // 1446
		.def("createEntity", (Ogre ::SceneNode * (*)(const char* id, const char* filename, const char* materialName))&RE::createEntity, RETURN_REFERENCE) // 1446
		.def("createEntity", (Ogre ::SceneNode * (*)(Ogre::SceneNode*, const char* id, const char* filename))&RE::createEntity, RETURN_REFERENCE) // 1446
		.def("removeEntity", (void (*)(Ogre::SceneNode*))&RE::removeEntity) // 1446
		.def("removeEntity", (void (*)(const char*))&RE::removeEntity) // 1446
		.def("setMaterialName", (void (*)(Ogre::SceneNode* pNode, const char* mat))&RE::setMaterialName) // 1446
		.def("moveEntity", (void (*)(Ogre::SceneNode*, quater const&, vector3 const&))&RE::moveEntity) // 1446
		.def("setBackgroundColour", (void (*)(m_real r, m_real g, m_real b))&RE_::setBackgroundColour) // 1446
		.def("viewpoint", (Viewpoint * (*)())&RE_::getViewpoint, return_value_policy::reference) // 1465
		.def("viewpoint", (Viewpoint * (*)(int))&RE_::getViewpoint, return_value_policy::reference) // 1465
		.def("rendererValid", (bool (*)())&RE::rendererValid)        // 1446
		.def("motionPanelValid", (bool (*)())&RE::motionPanelValid)  // 1446
		.def("renderer", (OgreRenderer & (*)())&RE::renderer, return_value_policy::reference) // 1469
		.def("_createRenderer", &RE::_createRenderer, return_value_policy::reference) // 1469
		.def("FltkRenderer", (FltkRenderer & (*)())&RE::FltkRenderer, return_value_policy::reference) // 1469
		.def("renderOneFrame", (bool (*)(bool check))&RE_::renderOneFrame) // 1446
		.def("loadPose", (void (*)(Posture& pose, const char* fn))&::loadPose) // 1446
		.def("savePose", (void (*)(Posture& pose, const char* fn))&::savePose) // 1446
		.def("getSceneNode", (Ogre ::SceneNode * (*)(PLDPrimSkin* skin))&RE::getSceneNode, return_value_policy::reference) // 1446
		.def("getSceneNode", (Ogre ::SceneNode * (*)(const char* id))&RE::getSceneNode, return_value_policy::reference) // 1446
		.def("createSceneNode", (Ogre ::SceneNode * (*)(const char* node_name))&RE::createSceneNode, return_value_policy::reference) // 1446
		.def("createChildSceneNode", (Ogre ::SceneNode * (*)(Ogre::SceneNode* parent, const char* child_name))&RE::createChildSceneNode, return_value_policy::reference) // 1446
		.def("createSkin", (PLDPrimSkin * (*)(const Motion&))&RE::createSkin) // 1446
		.def("createSkin", (PLDPrimSkin * (*)(const MotionLoader&))&RE::createSkin) // 1446
		.def("createVRMLskin", (PLDPrimVRML * (*)(VRMLloader*pTgtSkel, bool bDrawSkeleton))&RE::createVRMLskin, TAKE_OWNERSHIP) // 1446
		.def("generateUniqueName", []()->std::string{
				return std::string(RE::generateUniqueName().ptr());})
		.def("taesooLibPath", &RE::taesooLibPath)
		.def("setTaesooLibPath", &RE::setTaesooLibPath)
		;

	/////////////////////////////////////////////////////////////////
	// Baselib
	/////////////////////////////////////////////////////////////////

    
	// vector3

	void (vector3::*add1)(const vector3&, const vector3&) =&vector3::add;

	void (vector3::*sub1)(const vector3&, const vector3&) =&vector3::sub;


	class_<vector3>(mainlib, "vector3")
		.def(init<>())
		.def(init<m_real, m_real, m_real>())
		.def(init<m_real>())
		.def_readwrite("x", &vector3::x)
		.def_readwrite("y", &vector3::y)
		.def_readwrite("z", &vector3::z)
		.def_property_readonly("array",[](vector3 const& v){ npy_intp dims[1]; dims[0]=3; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.x); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
		.def("ref",  [](vector3 const& v){ npy_intp dims[1]; dims[0]=3; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.x); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
		.def("copy", [](vector3 const&v )->vector3 *{ return new vector3(v);}, TAKE_OWNERSHIP )
		.def("add", (void (vector3::*)(const vector3&, const vector3&) )&vector3::add)
		.def("__repr__", &vector3_output)
		.def("sub", sub1)
		.def("multadd", &vector3::multadd)
		.def("output", &vector3::output)
		.def("length", &vector3::length)
		.def("assign", &vector3_assign)
		.def("interpolate", &vector3::interpolate)
		.def("difference", &vector3::difference)
		.def("rotationVector", &vector3::rotationVector)
		.def("zero", (void (vector3::*)())&vector3::zero)             // 1447
		.def("rmult", (void (vector3::*)(double))&vector3::operator*=) // 1447
		.def("scale", (void (vector3::*)(double))&vector3::operator*=) // 1447
		.def("leftMult", (void (vector3::*)(const matrix4& mat))&vector3::leftMult) // 1447
		.def("cross", (void (vector3::*)(const vector3&, const vector3&))&vector3::cross) // 1447
		.def("cross", (vector3 (vector3::*)(const vector3&) const)&vector3::cross) // 1447
		.def("distance", (m_real (vector3::*)(const vector3& other))&vector3::distance) // 1447
		.def("normalize", (void (vector3::*)())&vector3::normalize)   // 1447
		.def("unitVector", (vector3 (vector3::*)())&vector3::dir)     // 1447
		.def("normalized", (vector3 (vector3::*)())&vector3::dir)     // 1447
		.def("multadd", (void (vector3::*)(const vector3&, m_real))&vector3::multadd) // 1447
		.def("length", (m_real (vector3::*)())&vector3::length)       // 1447
		.def("ln", (void (vector3::*)( const quater& q))&vector3::ln) // 1447
		.def("exp", (quater (vector3::*)())&vector3::exp)             // 1447
		.def("interpolate", (void (vector3::*)( m_real, vector3 const&, vector3 const& ))&vector3::interpolate) // 1447
		.def("difference", (void (vector3::*)(vector3 const& v1, vector3 const& v2))&vector3::difference) // 1447
		.def("rotate", (void (vector3::*)( const quater& q))&vector3::rotate) // 1447
		.def("rotate", (void (vector3::*)( const quater& q, vector3 const& in))&vector3::rotate) // 1447
		.def("rotationVector", (void (vector3::*)(const quater& in))&vector3::rotationVector) // 1447
		.def("angularVelocity", (void (vector3::*)( quater const& q1, quater const& q2))&vector3::angularVelocity) // 1447
		.def("linearVelocity", (void (vector3::*)(vector3 const& v1, vector3 const& v2))&vector3::linearVelocity) // 1447
		.def("translation", (void (vector3::*)(const matrix4& other))&vector3::translation) // 1447
		.def("quaternion", (quater (vector3::*)())&vector3::quaternion) // 1447
		.def("radd", (void (vector3::*)(const vector3&))&vector3::add) // 1447
		.def("rsub", (void (vector3::*)(const vector3&))&vector3::sub) // 1447
		.def("assign", (void (vector3::*)(const vector3&))&vector3::operator=) // 1447
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self - self) // add (homogeneous)
		.def(self * self) // mul
		.def(self * double()) // mul
		.def(self / double()) // div
		.def(self * matrix3()) // mul
		.def(matrix3() * self) // mul
		.def(double()*self) // mul
		.def(quater()*self)
		.def("dotProduct", (double (vector3::*)(vector3 const& b))&vector3::operator%) // 1460
		.def("set", (void (vector3::*)(double ,double, double))&vector3::setValue) // 1460
		.def("set", [](vector3& v, int i, double vv){ v[i]=vv;})
		.def("__call__", [](vector3& v, int i)->double { return v[i];})
	;

	
	{
		struct vector2_wrap
		{
			static double getX(vector2 const& v){ return v.x();}
			static double getY(vector2 const& v){ return v.y();}
			static void setX(vector2 & v, double x){ v.x()=x;}
			static void setY(vector2 & v, double y){ v.y()=y;}
		};
		class_<vector2>(mainlib, "vector2")
			.def(init<>())
			.def(init<m_real, m_real>())
			.def_property("x", &vector2_wrap::getX, &vector2_wrap::setX)
			.def_property("y", &vector2_wrap::getY, &vector2_wrap::setY)
			.def_property_readonly("array",[](vector2 const& v){ npy_intp dims[1]; dims[0]=2; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.x()); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](vector2 const& v){ npy_intp dims[1]; dims[0]=2; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.x()); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("copy", [](vector2 const&v )->vector2 *{ return new vector2(v);}, TAKE_OWNERSHIP )
			.def("assign", [](vector2& l, WRAP_PY::list ll){
					if(len(ll)!=2) throw std::range_error("vector2_assign");
					l.x()=ll[0].cast<double>();
					l.y()=ll[1].cast<double>();
			})
			.def("__neg__", [](vector2 const& v)->vector2 { return -v;}) // neg (unary minus)
			.def("__add__", [](vector2 const& a, vector2 const& b)->vector2 { return a+b;}) 
			.def("__sub__", [](vector2 const& a, vector2 const& b)->vector2 { return a-b;}) 
			.def("__mul__", [](vector2 const& a, double b)->vector2 { return a*b;}) 
			.def("__mul__", [](double b, vector2 const& a)->vector2 { return a*(double)b;}) 
			.def("__mul__", [](vector2 const& a, double b)->vector2 { return a/b;}) 
			.def("set", [](vector2& v, double x,double y ){
				v.x()=x; v.y()=y;}) // 1460
			;
	}

	{
		struct __pybindgen___vector4_wrapper
		{                                                             // 1382
			inline static double getX(vector4 const& a) { return a.x();}
			inline static double getY(vector4 const& a) { return a.y();}
			inline static double getZ(vector4 const& a) { return a.z();}
			inline static double getW(vector4 const& a) { return a.w();}
			inline static void setX(vector4 & a, m_real b) { a.x()=b;}
			inline static void setY(vector4 & a, m_real b) { a.y()=b;}
			inline static void setZ(vector4 & a, m_real b) { a.z()=b;}
			inline static void setW(vector4 & a, m_real b) { a.w()=b;}
			inline static void set(vector4 & a, m_real x, m_real y, m_real z, m_real w) {a.x()=x; a.y()=y; a.z()=z; a.w()=w;}
			// 1383
		};                                                            // 1384
		class_<vector4 > (mainlib, "vector4")                           // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<>())                                                // 1426
			.def(init<double,double,double,double>())                     // 1426
			.def("toVector3", (vector3 (vector4::*)())&vector4::toVector3) 
			.def(matrix4()*self)
			.def_property("x", &__pybindgen___vector4_wrapper::getX, &__pybindgen___vector4_wrapper::setX)
			.def_property("y", &__pybindgen___vector4_wrapper::getY, &__pybindgen___vector4_wrapper::setY)
			.def_property("z", &__pybindgen___vector4_wrapper::getZ, &__pybindgen___vector4_wrapper::setZ)
			.def_property("w", &__pybindgen___vector4_wrapper::getW, &__pybindgen___vector4_wrapper::setW)
			.def("set", (void (*)(vector4 & a, m_real x, m_real y, m_real z, m_real w))&__pybindgen___vector4_wrapper::set) // 1458
			; // end of class impl___pybindgen___vector4                  // 1505
	}

	void (quater::*setRotation1)(const vector3& axis, m_real angle)=&quater::setRotation;
	void (quater::*setRotation2)(const vector3& rotationVector)=&quater::setRotation;
	m_real (quater::*rotationAngle1)(void) const=&quater::rotationAngle;
	void (quater::*normalize1)()=&quater::normalize;

	struct __pybindgen___quater_wrapper
	{                                                             // 1382
		static vector3 rotate(quater const& q, vector3 const& v)
		{
			vector3 out;
			out.rotate(q,v);
			return out;
		}
		static void setRotation(quater &q, const char* aChannel, vector3 & euler)
		{
			q.setRotation(aChannel, euler);
		}
		static void setRotation2(quater &q, const char* aChannel, vector3 & euler, bool bRightToLeft)
		{
			q.setRotation(aChannel, euler, bRightToLeft);
		}
		static void getRotation(quater const&q, const char* aChannel, vector3 & euler)
		{
			q.getRotation(aChannel, euler);
		}
		static vector3 getRotation(quater const&q, const char* aChannel)
		{
			vector3 euler;
			q.getRotation(aChannel, euler);
			return euler;
		}
		static m_real toAxisAngle(quater const& q, vector3& axis)
		{
			m_real angle;
			q.toAxisAngle(axis, angle);
			return angle;
		}
		inline static double dotProduct(quater const& a, quater const& b)
		{
			return a%b;
		}
		inline static double _property_get_x(quater const& a) { return a.x; }inline static void _property_set_x(quater & a, double b){ a.x=b;}
		inline static double _property_get_y(quater const& a) { return a.y; }inline static void _property_set_y(quater & a, double b){ a.y=b;}
		inline static double _property_get_z(quater const& a) { return a.z; }inline static void _property_set_z(quater & a, double b){ a.z=b;}
		inline static double _property_get_w(quater const& a) { return a.w; }inline static void _property_set_w(quater & a, double b){ a.w=b;}
		// 1383
	};                                                            // 1384
	class_<quater>(mainlib, "quater")
		.def(init<>())
		.def(init<m_real, m_real, m_real, m_real>())
		.def(init<m_real, const vector3&>())
		.def_readwrite("x", &quater::x)
		.def_readwrite("y", &quater::y)
		.def_readwrite("z", &quater::z)
		.def_readwrite("w", &quater::w)
		.def("__call__", [](quater& v, int i)->double { return v[i];})
		.def("set", [](quater& v, int i, double vv){ v[i]=vv;})
		.def_property_readonly("array",  [](quater const& v){ npy_intp dims[1]; dims[0]=4; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.w); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
		.def("ref",  [](quater const& v){ npy_intp dims[1]; dims[0]=4; npy_intp strides[1]; strides[0]=sizeof(double); double* vv=const_cast<double*>(&v.w); PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE , NULL); return WRAP_PY::reinterpret_steal<WRAP_PY::object>(o); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

		.def("copy", [](quater const&v )->quater *{ return new quater(v);}, TAKE_OWNERSHIP )
		.def("getFrameAxis", &quater::getFrameAxis) // 1445
		.def("setFrameAxesYZ", &quater::setFrameAxesYZ) // 1445
		.def("slerp", &quater::slerp)
		.def("safeSlerp", &quater::safeSlerp)
		.def("interpolate", &quater::interpolate)
		.def("setAxisRotation", &quater::setAxisRotation)
		.def("identity", &quater::identity)
		.def("inverse", (quater (quater::*)() const)&quater::inverse)       // 1445
		.def("rotationY", (quater (quater::*)())&quater::rotationY)   // 1445
		.def("decompose", &quater::decompose) // 1445
		.def("decomposeTwistTimesNoTwist", &quater::decomposeTwistTimesNoTwist)
		.def("decomposeNoTwistTimesTwist", &quater::decomposeNoTwistTimesTwist)
		.def("difference", &quater::difference) // 1445
		.def("toLocal", &quater::toLocal) // 1445
		.def("scale", &quater::scale)
		.def("mult", (void (quater::*)(quater const& a, quater const& b))&quater::mult) // 1445
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self - self) // add (homogeneous)
		.def(self * self) // mul
		.def("output", &quater_output)
		.def("__repr__", &quater_output)
		.def("length", &quater::length)
		.def("rotationAngle", rotationAngle1)
		.def("rotationAngleAboutAxis", &quater::rotationAngleAboutAxis)
		.def("rotationVector", (vector3 (quater::*)() const)&quater::rotationVector) // 1445
		.def("assign", &quater_assign)
		.def("assign", (&quater::operator=))
		.def("set", (void (quater::*)(quater const& a))&quater::operator=) // 1445
		.def("setValue", (void (quater::*)( m_real ww,m_real xx, m_real yy, m_real zz ))&quater::setValue) // 1445
		.def("axisToAxis", (void (quater::*)( const vector3& vFrom, const vector3& vTo))&quater::axisToAxis) // 1445
		.def("leftMult", &quater::leftMult)
		.def("rightMult", &quater::rightMult)
		.def("setRotation", setRotation1)
		.def("setRotation", setRotation2)
		.def("setRotation", (void (quater::*)(const matrix4& a))&quater::setRotation) // 1445
		.def("setRotation", (void (quater::*)(const matrix3& a))&quater::setRotation) // 1445
		.def("normalize", normalize1)
		.def("normalized", &quater::normalized)
		.def("align", &quater::align)
		.def("blend", (void (quater::*)(const vectorn& weight, matrixn& aInputQuater))&quater::blend) // 1445
		.def("rotate", (vector3 (*)(quater const& q, vector3 const& v))&__pybindgen___quater_wrapper
::rotate) // 1458
		.def("setRotation", (void (*)(quater &q, const char* aChannel, vector3 & euler))&__pybindgen___quater_wrapper
::setRotation) // 1458
		.def("setRotation", &__pybindgen___quater_wrapper ::setRotation2) // 1458
		.def("getRotation", (void (*)(quater const&q, const char* aChannel, vector3 & euler))&__pybindgen___quater_wrapper
::getRotation) // 1458
		.def("getRotation", (vector3 (*)(quater const&q, const char* aChannel))&__pybindgen___quater_wrapper
::getRotation) // 1458
		.def("toAxisAngle", (m_real (*)(quater const& q, vector3& axis))&__pybindgen___quater_wrapper
::toAxisAngle) // 1458
		.def("dotProduct", (double (*)(quater const& a, quater const& b))&__pybindgen___quater_wrapper
::dotProduct) // 1458
	;
	
	{
		// transf
		class_<transf>(mainlib, "transf")
			.def(init<quater const&, vector3 const&>())
			.def(init<quater const&>())
			.def(init<vector3 const&>())
			.def(init<matrix4 const&>())
			.def(init<>())
			.def("log", [](transf const&v )->Liegroup::se3{ Liegroup::se3 out; out.log(v); return out;})
			.def("copy", [](transf const&v )->transf *{ return new transf(v);}, TAKE_OWNERSHIP )
			.def("__repr__", [](transf & v)->std::string {
					return std::string("R:")+v.rotation.output()+" T:"+v.translation.output();})
			.def_readwrite("rotation", &transf::rotation)
			.def("project2D", &transf::project2D)
			.def_readwrite("translation", &transf::translation)
			.def("inverse", &transf::inverse)
			.def("toLocal", &transf::toLocal)
			.def("toGlobal", &transf::toGlobal)
			.def("toLocalRot", &transf::toLocalRot)
			.def("toGlobalRot", &transf::toGlobalRot)
			.def("toLocalDRot", &transf::toLocalDRot)
			.def("toGlobalDRot", &transf::toGlobalDRot)
			.def("toLocalPos", &transf::toLocalPos)
			.def("toGlobalPos", &transf::toGlobalPos)
			.def("toLocalDir", &transf::toLocalDir)
			.def("toGlobalDir", &transf::toGlobalDir)
			.def("difference", &transf::difference)
			.def("identity", &transf::identity)
			.def("encode2D", &transf::encode2D)
			.def("decode2D", &transf::decode2D)
			.def("align2D", &transf::align2D)
			.def("leftMult", &transf::leftMult)
			.def("rightMult", &transf::operator*=)
			.def("slerp", &transf::slerp)
			.def("mult", &transf::mult)
			.def("leftMultRotation", [](transf& tf, quater const& q){ tf=transf(q, vector3(0.0,0.0,0.0))*tf;})
			.def("leftMultTranslation", [](transf& tf, vector3 const& v){ tf=transf(quater(1.0,0.0,0.0,0.0), v)*tf;})
			.def("integrateBodyVel", &transf::integrateBodyVel)
			.def("assign", (void (transf::*)(const transf& other))(&transf::operator=))
			.def("assign", (void (transf::*)(const matrix4& other))(&transf::operator=))
			.def(self * self) // mul
			.def(self * vector3()) // mul
			.def("twist",&Liegroup ::twist)
			.def("twist_nonlinear",&Liegroup ::twist_nonlinear)
			;
	}
	{
		struct matrix3_
		{
			// does not copy memory.
			static PyObject* ref(matrix3 const& v)
			{
				npy_intp dims[2];
				dims[0]=3;
				dims[1]=3;
				npy_intp strides[2];
				strides[0]=sizeof(double)*3;
				strides[1]=sizeof(double);
				double* vv=(double*)&v._11;
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 2, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
			// does not copy memory.
			static PyObject* ref1D(matrix3 const& v)
			{
				npy_intp dims[1];
				dims[0]=9;
				npy_intp strides[1];
				strides[0]=sizeof(double);
				double* vv=(double*)&v._11;
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		
		class_<matrix3 > (mainlib, "matrix3")                     // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<>())                                                // 1426
			.def(init<matrix3 const &>())                                 // 1426
			.def(init<quater const &>())                                  // 1426
			.def("setValue", (void (matrix3::*)( m_real a00, m_real a01, m_real a02, m_real a10, m_real a11, m_real a12, m_real a20, m_real a21, m_real a22 ))&matrix3::setValue) // 1446
			.def("setValue", (void (matrix3::*)( vector3 const&row1, vector3 const&row2, vector3 const&row3 ))&matrix3::setValue) // 1446
			.def("zero", &matrix3::zero)                                  // 1445
			.def("identity", &matrix3::identity)                          // 1445
			.def("transpose", (void (matrix3::*)( void ))&matrix3::transpose) // 1446
			.def("negate", &matrix3::negate)                              // 1445
			.def("inverse", (bool (matrix3::*)(matrix3 const& a))&matrix3::inverse) // 1446
			.def("setTilde", (void (matrix3::*)( vector3 const &v ))&matrix3::setTilde) // 1446
			.def("setFromQuaternion", &matrix3::setFromQuaternion)        // 1445
			.def("mult", (void (matrix3::*)(matrix3 const& a,matrix3 const& b))&matrix3::mult) // 1446
			.def("mult", (void (matrix3::*)(matrix3 const& a, m_real b))&matrix3::mult) // 1446
			.def("toQuater", &matrix3::toQuater)																						// .def
			.def(self-self)
			.def(self+self)
			.def(self*vector3())
			.def(self*self)
			.def_property_readonly("array",  [](matrix3 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix3_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](matrix3 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix3_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("ref1D",  [](matrix3 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix3_::ref1D(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			;
	}
	// matrix4
	{
		struct wrap_matrix4
		{
			static void matrix4_assign(matrix4& l, WRAP_PY::list ll) 
			{
				if(len(ll)!=4) throw std::range_error("matrix4_assign");
				l._11=ll[0].cast<double>();
				l._12=ll[1].cast<double>();
				l._13=ll[2].cast<double>();
				l._14=ll[3].cast<double>();
				l._21=ll[4].cast<double>();
				l._22=ll[5].cast<double>();
				l._23=ll[6].cast<double>();
				l._24=ll[7].cast<double>();
				l._31=ll[8].cast<double>();
				l._32=ll[9].cast<double>();
				l._33=ll[10].cast<double>();
				l._34=ll[11].cast<double>();
				l._41=ll[12].cast<double>();
				l._42=ll[13].cast<double>();
				l._43=ll[14].cast<double>();
				l._44=ll[15].cast<double>();
			}

			//a.assign([0 1 2 3 4 4 5 5 6 6 7 ])

		};
		struct matrix4_
		{
			// does not copy memory.
			static PyObject* ref(matrix4 const& v)
			{
				npy_intp dims[2];
				dims[0]=4;
				dims[1]=4;
				npy_intp strides[2];
				strides[0]=sizeof(double)*4;
				strides[1]=sizeof(double);
				double* vv=(double*) &v._11;
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 2, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
			// does not copy memory.
			static PyObject* ref1D(matrix4 const& v)
			{
				npy_intp dims[1];
				dims[0]=16;
				npy_intp strides[1];
				strides[0]=sizeof(double);
				double* vv=(double*)&v._11;
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		
		class_<matrix4>(mainlib, "matrix4")
			.def(init<>())                                                // 1426
			.def(init<const quater &,const vector3 &>())                  // 1426
			.def(init<const transf &>())                                  // 1426
			.def_property_readonly("array", [](matrix4 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix4_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](matrix4 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix4_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("ref1D",  [](matrix4 const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrix4_::ref1D(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def_readwrite("_11", &matrix4::_11)
			.def_readwrite("_12", &matrix4::_12)
			.def_readwrite("_13", &matrix4::_13)
			.def_readwrite("_14", &matrix4::_14)
			.def_readwrite("_21", &matrix4::_21)
			.def_readwrite("_22", &matrix4::_22)
			.def_readwrite("_23", &matrix4::_23)
			.def_readwrite("_24", &matrix4::_24)
			.def_readwrite("_31", &matrix4::_31)
			.def_readwrite("_32", &matrix4::_32)
			.def_readwrite("_33", &matrix4::_33)
			.def_readwrite("_34", &matrix4::_34)
			.def_readwrite("_41", &matrix4::_41)
			.def_readwrite("_42", &matrix4::_42)
			.def_readwrite("_43", &matrix4::_43)
			.def_readwrite("_44", &matrix4::_44)
			.def("assign", &wrap_matrix4::matrix4_assign)
			.def("copy", [](matrix4 const&v )->matrix4 *{ return new matrix4(v);}, TAKE_OWNERSHIP )
			.def("determinant", (m_real (matrix4::*)())&matrix4::determinant) // 1445
			.def("identity", (void (matrix4::*)())&matrix4::identity)     // 1445
			//.def("setValue", [](matrix4&m, m_real x00, m_real x01, m_real x02, m_real x03, m_real x10, m_real x11, m_real x12, m_real x13, m_real x20, m_real x21, m_real x22, m_real x23, m_real x30, m_real x31, m_real x32, m_real x33) { m.setValue( x00,  x01,  x02,  x03,  x10,  x11,  x12,  x13,  x20,  x21,  x22,  x23,  x30,  x31,  x32,  x33); })
			.def("setRotation", [](matrix4&m, const quater& q){ m.setRotation(q);})
			.def("setRotation", [](matrix4&m, const matrix3& q){ m.setRotation(q);})
			.def("setRotation", [](matrix4&m, const vector3& a, double angle){ m.setRotation(a, angle);})
			.def("setTranslation", (void (matrix4::*)(const vector3& vec, bool bPreserveCurrentRotation))&matrix4::setTranslation) // 1445
			.def("setTransform", (void (matrix4::*)(const vector3& position,const vector3& scale,const quater& orientation))&matrix4::setTransform) // 1445
			.def("setScaling", (void (matrix4::*)(m_real sx, m_real sy, m_real sz))&matrix4::setScaling) // 1445
			.def("setProjection", (void (matrix4::*)(m_real fovx, m_real fovy, m_real Z_near, m_real Z_far))&matrix4::setProjection) // 1445
			.def("leftMultRotation", (void (matrix4::*)(const quater& b))&matrix4::leftMultRotation) // 1445
			.def("leftMultRotation", (void (matrix4::*)(const vector3& axis, m_real angle))&matrix4::leftMultRotation) // 1445
			.def("leftMultTranslation", (void (matrix4::*)(const vector3& vec))&matrix4::leftMultTranslation) // 1445
			.def("leftMultScaling", (void (matrix4::*)(m_real sx, m_real sy, m_real sz))&matrix4::leftMultScaling) // 1445
			.def("leftMultScale", [](matrix4&m, m_real s){ m.leftMultScaling(s,s,s);})
			.def("inverse", (void (matrix4::*)(const matrix4& a))&matrix4::inverse) // 1445
			.def("transpose", (void (matrix4::*)())&matrix4::transpose)   // 1445
			.def("T", (matrix4 (matrix4::*)())&matrix4::T)                // 1445
			.def("inverse", (matrix4 (matrix4::*)() const)&matrix4::inverse)    // 1445
			.def(self*self)
			.def(self*vector3())
			.def(self*vector4())
			.def("rotate", (vector3 (matrix4::*)(vector3 const& a))&matrix4::rotate) // 1445
			.def(self+self)
			.def(self-self)
			.def("assign", (matrix4 (matrix4::*)(matrix4 const& b))&matrix4::operator=) // 1445
			.def("rmult", (void (matrix4::*)(double b))&matrix4::operator*=) // 1445
			.def("rightMult", (void (matrix4::*)(matrix4 const& b))&matrix4::rightMult) // 1445
			.def("getColumn", (vector3 (matrix4::*)(int i))&matrix4::getColumn) // 1445
			.def("setColumn", (void (matrix4::*)(int i, vector3 const& v))&matrix4::setColumn) // 1445
			.def("assign", [](matrix4 &m, const matrix4& o){ m=o;})
			//.def("__repr__", [](matrix4 const& in)->std::string{ return in.output().tostring();})
			.def("inverse", [](matrix4 &m)->matrix4{ return m.inverse();})
			.def("translation", [](matrix4 &m)->vector3{ return m.translation();})
			;
	}


	{
		void	(matrixn::*setAllValue)(m_real d)=&matrixn::setAllValue;
		matrixnView (matrixn::*range)(int startr, int endr, int startc, int endc)=&matrixn::range;

		class_<matrixn>(mainlib, "matrixn")
			.def(init<>())
			.def(init<int,int>())
			.def("quatViewCol", &quatViewCol)
			.def("vec3ViewCol", &vec3ViewCol)
			.def("copy", [](matrixn const&v )->matrixn *{ return new matrixn(v);}, TAKE_OWNERSHIP )
			.def_property_readonly("array", [](matrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](matrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("__repr__", [](matrixn const& in)->std::string{ return std::string("matrixn")+in.shortOutput().tostring();})
			// concat
			.def("__or__",[](const matrixn& self, const matrixn& b)->matrixn{
					if(self.rows()!=b.rows()) throw std::range_error("matrixn column concatenation (|)");
					matrixn out(self.rows(), self.cols()+b.cols());
					out.range(0, self.rows(), 0, self.cols())=self;
					out.range(0, self.rows(), self.cols(), out.cols())=b;
					return out;
					})
			// slicing
			.def("zero",  [](matrixn & v){ v.setAllValue(0.0);})
			.def("sub",[](matrixn & inout, int srow, int erow, int scol, int ecol)->matrixnView{
					if (srow<0 ) srow=inout.rows()+srow;
					if (erow<=0 ) erow=inout.rows()+erow;
					if (scol<0 ) scol=inout.cols()+scol;
					if (ecol<=0 ) ecol=inout.cols()+ecol;
					return inout.range(srow, erow, scol, ecol);
					}, "srow"_a, "erow"_a, "scol"_a=0, "ecol"_a=0)
			.def(WRAP_PY::pickle(
						[](const matrixn &p) { // __getstate__
						auto my_tuple = WRAP_PY::tuple(p.rows()*p.cols()+2);
						my_tuple[0]=p.rows();
						my_tuple[1]=p.cols();
						int c=2;
						for(int i=0; i<p.rows(); i++)
						for(int j=0; j<p.cols(); j++)
						my_tuple[c++]=p(i,j);
						/* Return a tuple that fully encodes the state of the object */
						return my_tuple;
						},
						[](WRAP_PY::tuple t) { // __setstate__
						matrixn out;
						out.setSize(t[0].cast<int>(), t[1].cast<int>());
						int c=2;
						for(int i=0; i<out.rows(); i++)
							for(int j=0; j<out.cols(); j++)
							out(i,j)=t[c++].cast<double>();
						return out;
						}))
			.def("derivative",&matrixn::derivative)
			.def("derivative_forward",&matrixn::derivative_forward)
			.def("MotionDOF_calcDerivative", (matrixn (*)(matrixn const& dof, double frameRate))&MotionDOF_calcDerivative)																																		//
			.def("assign", (matrixn& (matrixn::*)(const matrixn&))(&matrixn::assign), RETURN_REFERENCE)
			.def("row", &matrixn::row)
			.def(self + self) // add (homogeneous)
			.def(self - self) // add (homogeneous)
			.def(self * self) // mul
			.def(self * double()) // mul
			.def("__getitem__",
			   [](matrixn const&v, int i) -> vectornView {
					if(i<0) i+=v.rows();
				   if (i >= v.rows())
					   throw index_error();
					return v.row(i);
				 })
			.def("isnan", &matrixn::isnan)
			.def("extractRows", (void (matrixn::*)(matrixn const& mat, intvectorn const& rows))&matrixn::extractRows) // 1445
			.def("extractColumns", (void (matrixn::*)(matrixn const& mat, intvectorn const& columns))&matrixn::extractColumns) // 1445
			.def("assignRows", (void (matrixn::*)(matrixn const& mat, intvectorn const& rows))&matrixn::assignRows) // 1445
			.def("assignColumns", (void (matrixn::*)(matrixn const& mat, intvectorn const& columns))&matrixn::assignColumns) // 1445
			.def("get", &matrixn::getValue)
			.def("set", &matrixn::set)
			.def("rows", &matrixn::rows)
			.def("cols", &matrixn::cols)
			.def("column", &matrixn::column)
			.def("diag", (vectornView (matrixn::*)())&matrixn::diag)      // 1445
			.def("transpose", (void (matrixn::*)(matrixn const& o))&matrixn::transpose) // 1445
			.def("setSize", &matrixn::setSize)
			.def("resize", &matrixn::resize)
			.def("setAllValue", setAllValue)
			.def("__call__", &matrixn::getValue)
			.def("range", static_cast<matrixnView (matrixn::*)(int, int, int, int)>(&matrixn::range), WRAP_PY::keep_alive<0,1>(), "startRow"_a, "endRow"_a,"startColumn"_a=0,"endColumn"_a=INT_MAX)
			.def("minimum", (double (matrixn::*)())&matrixn::minimum)     // 1445
			.def("maximum", (double (matrixn::*)())&matrixn::maximum)     // 1445
			.def("mean",[](matrixn const& a)->vectorn{vectorn v; v.mean(a); return v;})
			.def("sum", (double (matrixn::*)())&matrixn::sum)             // 1445
			.def("pushBack", (void (matrixn::*)(vectorn const& o))&matrixn::pushBack) // 1445
			.def("mult", [](matrixn &m, matrixn const& a, matrixn const&b ){ m.mult(a,b);})
			.def("multABt", [](matrixn &m, matrixn const& a, matrixn const&b ){ m.multABt(a,b);})
			.def("multAtB", [](matrixn &m, matrixn const& a, matrixn const&b ){ m.multAtB(a,b);})
			.def("multAtBt", [](matrixn &m, matrixn const& a, matrixn const&b ){ m.multAtBt(a,b);})
			.def("resample", (void (matrixn::*)(matrixn const& mat, int numSample))&matrixn::resample) // 1445
			.def("toVector", (vectorn (matrixn::*)() const)&matrixn::toVector)  // 1445
			.def("fromVector", (void (matrixn::*)(const vectorn& vec, int column))&matrixn::fromVector) // 1445
			.def("sampleRow",&matrixn::sampleRow)
		;
		class_<matrixnView, matrixn >(mainlib, "matrixnView")
			.def(init<const matrixn &>())
			.def(init<const matrixnView &>())
			;
	}
	{
		struct wrap_hyper
		{
			static matrixnView page(hypermatrixn const& h, int index)
			{
				return h.page(index);
			}
			// does not copy memory.
			static PyObject* ref(hypermatrixn const& v)
			{
				npy_intp dims[3];
				dims[0]=v.pages();
				dims[1]=v.rows();
				dims[2]=v.cols();
				npy_intp strides[3];
				strides[0]=sizeof(double)*v.cols()*v.rows();
				strides[1]=sizeof(double)*v.cols();
				strides[2]=sizeof(double);
				double* vv=&v[0][0][0];
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 3, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		int (hypermatrixn::*page1)() const=&hypermatrixn::page;
		class_<hypermatrixn>(mainlib, "hypermatrixn")
			.def(init<>())
			.def(init<int, int, int>())
			.def("pages", page1)
			.def("rows",&hypermatrixn::rows)
			.def("cols",&hypermatrixn::cols)
			.def("setSize",&hypermatrixn::setSize)
			.def("setSameSize",&hypermatrixn::setSameSize)
			.def("page",&wrap_hyper::page)
			.def_property_readonly("array",  [](hypermatrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(wrap_hyper::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](hypermatrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(wrap_hyper::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("resize", &hypermatrixn::resize)                         // 1445
			.def("pushBack", &hypermatrixn::pushBack)                     // 1445
			.def("column", &hypermatrixn::column)                         // 1445
			.def("setColumn", &hypermatrixn::setColumn)                   // 1445
			.def("row", &hypermatrixn::row)                               // 1445
			.def("setRow", &hypermatrixn::setRow)                         // 1445
			.def("weightedAverage", &hypermatrixn::weightedAverage)       // 1445
			.def("tensorView", (TensorView (*)(const hypermatrixn& other))&tensorView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1459
			.def("assign", &hypermatrixn::operator=)                      // 1445
			.def("__call__", (double & (hypermatrixn::*)(int i, int j, int k))&hypermatrixn::operator(),return_value_policy::reference ) // 1451
		;
	}
	{
		struct wrap_tensor
		{
			// does not copy memory.
			static PyObject* ref(Tensor const& v)
			{
				npy_intp dims[TENSOR_MAX_DIMS];
				npy_intp strides[TENSOR_MAX_DIMS];
				int ndim=v.shape().size();
				for(int i=0; i<ndim; i++)
				{
					dims[i]=v.shape(i);
					strides[i]=sizeof(double)*v.strides()[i];
				}
				double* vv=(double*)v.dataPointer();
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), ndim, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		class_<Tensor>(mainlib, "Tensor")
			.def(init<>())
			.def(init<int, int, int>())
			.def(init<int, int, int, int>())
			.def(init<int, int, int, int, int>())
			.def("slice_1d", &Tensor::slice_1d, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def_property_readonly("array", [](Tensor const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(wrap_tensor::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](Tensor const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(wrap_tensor::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("shape", (intvectornView (Tensor::*)() const)&Tensor::shape)                                 // 1445
			.def("pages", &Tensor::pages)                                 // 1445
			.def("rows", &Tensor::rows)                                   // 1445
			.def("cols", &Tensor::cols)                                   // 1445
			.def("setAllValue", &Tensor::setAllValue)																		  //
			.def("__call__", (double (Tensor::*)(int i, int j, int k))&Tensor::get) // 1446
			.def("set", (void (Tensor::*)(int i, int j, int k, double f))&Tensor::set) // 1446
			.def("__call__", (double (Tensor::*)(int i, int j, int k, int l))&Tensor::get) // 1446
			.def("set", (void (Tensor::*)(int i, int j, int k, int l, double f))&Tensor::set) // 1446
			.def("__call__", (double (Tensor::*)(const intvectorn& indices))&Tensor::get_ref) // 1446
			.def("set", (void (Tensor::*)(const intvectorn& indices, double f))&Tensor::set) // 1446
			.def("__call__", &Tensor::page)//1446
			.def("slice", (TensorView (Tensor::*)(const intvectorn& _indices))&Tensor::slice, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1446
			.def("slice", [](Tensor const& in, int i, int j, int k)->TensorView { 
					return in.slice(intvectorn(3, i,j,k));
					}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458
			.def("slice", [](Tensor const& in, int i, int j, int k, int l)->TensorView { 
					return in.slice(intvectorn(4, i,j,k,l));
					}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458
			.def("assign", (void (Tensor::*)(floatTensor const& other))&Tensor::assign) // 1446
			.def("assign", (void (Tensor::*)(Tensor const& other))&Tensor::assign) // 1446
			.def("assign", (void (Tensor::*)(floatvec const& other))&Tensor::assign) // 1446
			.def("assign", (void (Tensor::*)(vectorn const& other))&Tensor::assign) // 1446
			.def("assign", (void (Tensor::*)(matrixn const& other))&Tensor::assign) // 1446
			.def("assign", (void (Tensor::*)(hypermatrixn const& other))&Tensor::assign) // 1446
			.def("toMat", &Tensor::toMat)                                 // 1445
			.def("__repr__", [](Tensor const& in)->std::string{ return in.shortOutput().tostring();})
		;
		class_<TensorView ,Tensor> (mainlib, "TensorView")               
			; // end of class impl___pybindgen___TensorView               // 1506
	}
	// quaterN
	{
		void (quaterN::*assign1)(const quaterN& other)=&quaterN::assign;
		class_<quaterN>(mainlib, "quaterN")
			.def(init<>())                                                // 1426
			.def(init<int>())                                             // 1426
			.def("copy", [](quaterN const&v )->quaterN *{ return new quaterN(v);}, TAKE_OWNERSHIP )
			.def("value", &quaterN::value, RETURN_REFERENCE)
			.def_property_readonly("array",  [](quaterN const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(matView(v))); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](quaterN const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(matView(v))); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("row", &quaterN::row, RETURN_REFERENCE)
			.def("row", &quaterN::rows)
			.def("__repr__", [](quaterN const& in)->std::string{ return std::string("quaterN")+matView(in).shortOutput().tostring();})
			.def("__call__", &quaterN::row, RETURN_REFERENCE)
			.def("__getitem__", 
			   [](quaterN const&v, int i) -> quater& {
					if(i<0) i+=v.size();
				   if (i >= v.size())
					   throw index_error();
					return v(i);
				 }, WRAP_PY::return_value_policy::reference_internal)
			.def("__len__", &quaterN::size)
			.def("range", static_cast<quaterNView (quaterN::*)(int, int, int)>(&quaterN::range),WRAP_PY::keep_alive<0,1>(), "start"_a, "end"_a,"step"_a=1 ) // return value(0) depends on self(1).
			.def("assign", assign1)
			.def("align", (void (quaterN::*)())&quaterN::align)           // 1445
			.def("hermite", [](quaterN& qq,const quater& a, const quater& b, int duration, const quater& c, const quater& d){ qq.hermite(a,b,duration, c, d);})
			.def("hermite0", [](quaterN& qq,const quater& a, const quater& b, int duration, const quater& c, const quater& d){ qq.hermite0(a,b,duration, c, d);})
			.def("hermite_mid", [](quaterN& qq,const quater& a, const quater& b, int duration, const quater& c, const quater& d,quater const& mid){ qq.hermite_mid(a,b,duration, c, d, mid);})
			.def("bubbleOut", (void (quaterN::*)(int start, int end))&quaterN::bubbleOut) // 1445
			.def("rows", (int (quaterN::*)())&quaterN::rows)              // 1445
			.def("size", &quaterN::size)
			.def("setSize", (void (quaterN::*)(int))&quaterN::setSize)    // 1445
			.def("resize", (void (quaterN::*)(int))&quaterN::resize)      // 1445
			.def("reserve", (void (quaterN::*)(int))&quaterN::reserve)    // 1445
			.def("range", (quaterNView (quaterN::*)(int,int))&quaterN::range, WRAP_PY::keep_alive<0,1>()) // 1445
			.def("range", (quaterNView (quaterN::*)(int,int,int))&quaterN::range, WRAP_PY::keep_alive<0,1>()) // 1445
			.def("assign", (void (quaterN::*)(quaterN const&))&quaterN::assign) // 1445
			.def("row", (quater & (quaterN::*)(int))&quaterN::at,return_value_policy::reference ) // 1450
			.def("at", (quater & (quaterN::*)(int))&quaterN::at,return_value_policy::reference ) // 1450
			.def("transition", (void (quaterN::*)(quater const&, quater const&, int duration))&quaterN::transition) // 1445
			.def("setAllValue", (void (quaterN::*)(quater))&quaterN::setAllValue) // 1445
			.def("pushBack", (void (quaterN::*)(quater const& o))&quaterN::pushBack) // 1445
			.def("pushFront", (void (quaterN::*)(quater const& o))&quaterN::pushFront) // 1445
			.def("sampleRow", [](quaterN const& in, m_real criticalTime)->quater { return in.sampleRow(criticalTime);}) // 1458
			.def("matView", (matrixnView (*)(quaterN const& a, int start, int end))&matView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458
			.def("matView", [](quaterN const& a)->matrixnView { return matView(a);}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("__repr__", [](quaterN const& in)->std::string{ return std::string("quaterN")+matView(in).output().tostring();})
		;

		class_<quaterNView, quaterN >(mainlib, "quaterNView");
	}

	// vector3N
	{
		void (vector3N::*assign1)(const vector3N& other)=&vector3N::assign;
		class_<vector3N>(mainlib, "vector3N")
			.def(init<>())
			.def(init<int>())
			.def(init<matrixn>())
			.def(init<matrixnView>())
			.def("copy", [](vector3N const&v )->vector3N *{ return new vector3N(v);}, TAKE_OWNERSHIP )
			.def_property_readonly("array",  [](vector3N const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(matView(v))); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](vector3N const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(matView(v))); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("value", &vector3N::value, RETURN_REFERENCE)
			.def("row", &vector3N::row, RETURN_REFERENCE)
			.def("__repr__", [](vector3N const& in)->std::string{ return std::string("vector3N")+matView(in).shortOutput().tostring();})
			.def("__call__", &vector3N::row, RETURN_REFERENCE)
			.def("__getitem__", 
			   [](vector3N const&v, int i) -> vector3& {
					if(i<0) i+=v.size();
				   if (i >= v.size())
					   throw index_error();
					return v(i);
				 }, WRAP_PY::return_value_policy::reference_internal)
			.def("__len__", &vector3N::size)
			.def("row", &vector3N::rows)
			.def("rows", &vector3N::rows)
			.def("size", &vector3N::size)
			.def("setSize", &vector3N::setSize)
			.def("reserve", &vector3N::reserve)
			.def("translate", &vector3N::translate)
			.def("rotate", (void (vector3N::*)(const quater& q))&vector3N::rotate)
			.def("rotate", (void (vector3N::*)(const vector3& center, const quater& q))&vector3N::rotate)
			.def("range", static_cast<vector3NView (vector3N::*)(int, int, int)>(&vector3N::range), WRAP_PY::keep_alive<0,1>(), "start"_a, "end"_a,"step"_a=1)
			.def("assign", assign1)
			.def("setAllValue",(void (vector3N::*)(vector3))(&vector3N::setAllValue))
			.def("pushBack",(void (vector3N::*)(vector3))(&vector3N::pushBack))
			.def("x",&vector3N::x)
			.def("y",&vector3N::y)
			.def("z",&vector3N::z)
			.def("sampleRow", [](vector3N const& in, m_real criticalTime)->vector3 { return in.sampleRow(criticalTime);}) // 1458
			.def("matView", (matrixnView (*)(vector3N const&))(&matView), WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("matView", (matrixnView (*)(vector3N const& a, int start, int end))&matView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458

			.def("__repr__", [](vector3N const& in)->std::string{ return std::string("vector3N")+matView(in).output().tostring();})
			.def(self*double())
			.def(self+self)
			.def(self-self)
			.def(self+vector3())
			.def(self-vector3())
		;

		class_<vector3NView, vector3N >(mainlib, "vector3NView");
	}

	{
		class_<intIntervals > (mainlib, "intIntervals")           // 1393
			.def(init<>())                                                // 1431
			.def("copy", [](intIntervals const&v )->intIntervals *{ return new intIntervals(v);}, TAKE_OWNERSHIP )
			.def("numInterval", &intIntervals::numInterval)               // 1450
			.def("size", &intIntervals::size)                             // 1450
			.def("setSize", &intIntervals::setSize)                       // 1450
			.def("resize", &intIntervals::resize)                         // 1450
			.def("removeInterval", &intIntervals::removeInterval)         // 1450
			.def("startI", &intIntervals::start)                          // 1450
			.def("endI", &intIntervals::end)                              // 1450
			.def("load", &intIntervals::load)                             // 1450
			.def("pushBack", &intIntervals::pushBack)                     // 1450
			.def("findOverlap", (int (intIntervals ::*)(int start, int end, int startInterval))&intIntervals::findOverlap) // 1451
			.def("findOverlap", [](intIntervals & v,int start, int end)->int{return v.findOverlap(start, end);}) // 1451
			.def("runLengthEncode", [](intIntervals& a, const boolN& source) { a.runLengthEncode(source); })
			.def("runLengthEncode", (void (intIntervals::*)(const boolN& source , int start, int end))&intIntervals::runLengthEncode) // 1451
			.def("findConsecutiveIntervals", &intIntervals::findConsecutiveIntervals) // 1450
			.def("runLengthEncodeCut", &intIntervals::runLengthEncodeCut) // 1450
			.def("encodeIntoVector", &intIntervals::encodeIntoVector)     // 1450
			.def("decodeFromVector", &intIntervals::decodeFromVector)     // 1450
			.def("offset", &intIntervals::offset)                         // 1450
			.def("toBitvector", &intIntervals::toBitvector)               // 1450
			.def("set", [](intIntervals& v, int iint, int s, int e){
					v.start(iint)=s;
					v.end(iint)=e;
					})
			; // end of class impl___pybindgen___intIntervals             // 1511
	}
	// BinaryFile, intvectorn 
	{
		struct __pybindgen__util_BinaryFile_wrapper
		{                                                             // 1382
			inline static void _pack(BinaryFile& bf, MotionLoader* pLoader)
			{
				pLoader->pack(bf, MOT_RECENT_VERSION);
			}
			inline static void _unpack(BinaryFile& bf, MotionLoader* pLoader)
			{
				pLoader->unpack(bf);
			}
			inline static void _packVRMLloader(BinaryFile& bf, VRMLloader* pLoader)
			{
				pLoader->_exportBinary(bf);
			}
			inline static void _unpackVRMLloader(BinaryFile& bf, VRMLloader* pLoader)
			{
				pLoader->_importBinary(bf);
			}
			inline static void _pack(BinaryFile& bf, Posture& pose)
			{
				bf.pack(MOT_VERSION_STRING[MOT_RECENT_VERSION]);
				bf.packInt(MOT_RECENT_VERSION);    
				pose.pack(bf, MOT_RECENT_VERSION);
			}
			inline static void _unpack(BinaryFile& bf, Posture& pose)
			{
				bf.unpackStr();
				int version=bf.unpackInt();    
				pose.unpack(bf,version);
			}
			// 1383
		};                                                            // 1384
		class_<BinaryFile>(mainlib, "BinaryFile")
			.def(init<bool, const std::string&>())
			.def(init<>())
			.def(init<bool>())
			.def("openWrite", (bool (BinaryFile::*)(const char *fileName))&BinaryFile::openWrite) // 1445
			.def("openWrite", (bool (BinaryFile::*)(const char *fileName, bool))&BinaryFile::openWrite) // 1445
			.def("openRead", (bool (BinaryFile::*)(const char *fileName))&BinaryFile::openRead) // 1445
			.def("close", (void (BinaryFile::*)())&BinaryFile::close)     // 1445
			.def("packInt", (void (BinaryFile::*)(int num))&BinaryFile::packInt) // 1445
			.def("packFloat", (void (BinaryFile::*)(double num))&BinaryFile::packFloat) // 1445
			.def("pack", (void (BinaryFile::*)(const char *str))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const vectorn& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const floatvec& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const vector3& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const quater& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const intvectorn& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const matrixn& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const intmatrixn& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const vector3N& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const quaterN& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const TStrings& aSz))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const boolN& vec))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const matrix4& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const hypermatrixn& mat))&BinaryFile::pack) // 1445
			.def("pack", (void (BinaryFile::*)(const Tensor& mat))&BinaryFile::pack) // 1445
			.def("unpackInt", (int (BinaryFile::*)())&BinaryFile::unpackInt) // 1445
			.def("unpackFloat", (double (BinaryFile::*)())&BinaryFile::unpackFloat) // 1445
			.def("unpackStr", [](BinaryFile & self)->std::string{ return std::string(self.unpackStr().ptr()); })
			.def("unpack", (void (BinaryFile::*)(vectorn& vec))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(vector3& vec))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(quater& vec))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(intvectorn& vec))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(matrixn& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(intmatrixn& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(TStrings& aSz))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(boolN& vec))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(quaterN& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(vector3N& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(matrix4& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(hypermatrixn& mat))&BinaryFile::unpack) // 1445
			.def("unpack", (void (BinaryFile::*)(Tensor& mat))&BinaryFile::unpack) // 1445
			.def("_unpackInt", (int (BinaryFile::*)())&BinaryFile::_unpackInt) // 1445
			.def("_unpackFloat", (double (BinaryFile::*)())&BinaryFile::_unpackFloat) // 1445
			.def("_unpackStr", [](BinaryFile & self)->std::string{ return std::string(self._unpackStr().ptr()); })
			.def("_unpackVec", (void (BinaryFile::*)(vectorn& vec))&BinaryFile::_unpackVec) // 1445
			.def("_unpackVec", (void (BinaryFile::*)(intvectorn& vec))&BinaryFile::_unpackVec) // 1445
			.def("_unpackSPVec", (void (BinaryFile::*)(vectorn& vec))&BinaryFile::_unpackSPVec) // 1445
			.def("_unpackMat", (void (BinaryFile::*)(matrixn& mat))&BinaryFile::_unpackMat) // 1445
			.def("_unpackSPMat", (void (BinaryFile::*)(matrixn& mat))&BinaryFile::_unpackSPMat) // 1445
			.def("_unpackBit", (void (BinaryFile::*)(boolN& vec))&BinaryFile::_unpackBit) // 1445
			.def("getFrameNum", (int (BinaryFile::*)(int numOfData))&BinaryFile::getFrameNum) // 1445
			.def("_pack", (void (*)(BinaryFile& bf, MotionLoader* pLoader))&__pybindgen__util_BinaryFile_wrapper::_pack) // 1458
			.def("_unpack", (void (*)(BinaryFile& bf, MotionLoader* pLoader))&__pybindgen__util_BinaryFile_wrapper::_unpack) // 1458
			.def("_packVRMLloader", (void (*)(BinaryFile& bf, VRMLloader* pLoader))&__pybindgen__util_BinaryFile_wrapper::_packVRMLloader) // 1458
			.def("_unpackVRMLloader", (void (*)(BinaryFile& bf, VRMLloader* pLoader))&__pybindgen__util_BinaryFile_wrapper::_unpackVRMLloader) // 1458
			.def("_pack", (void (*)(BinaryFile& bf, Posture& pose))&__pybindgen__util_BinaryFile_wrapper::_pack) // 1458
			.def("_unpack", (void (*)(BinaryFile& bf, Posture& pose))&__pybindgen__util_BinaryFile_wrapper::_unpack) // 1458
			; // end of class impl___pybindgen__util_BinaryFile           // 1505
		class_<MemoryFile ,BinaryFile> (mainlib, "MemoryFile")                     // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<>())                                                // 1426
		   ;

		struct __pybindgen___boolN_wrapper
		{                                                             // 1382
			static boolNView _range(boolN const& a, int start, int end)
			{
				return boolNView (a._vec, a._start+start, a._start+end);
			}
			// 1383
		};                                                            // 1384
		class_<boolN > (mainlib, "boolN")                               // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<>())                                                // 1426
			.def(init<int>())                                             // 1426
			.def("copy", [](boolN const&v )->boolN *{ return new boolN(v);}, TAKE_OWNERSHIP )
			.def("count", &boolN::count, "bVal"_a=true) // 1445
			.def("assign", &boolN::assign) // 1445
			.def("assign", [](boolN & v, WRAP_PY::array_t<bool> const&array) {
					v.resize(array.size());
					//ssize_t stride=array.strides()[0];
					for(int i=0; i<v.size(); i++)
						v.set(i, *array.data(i));
					})
			.def(-self) // neg (unary minus)
			.def("setAt", (void (boolN::*)( const intvectorn&))&boolN::setAt)
			.def("clearAt", (void (boolN::*)( const intvectorn&))&boolN::clearAt)
			.def("numpy", [](boolN const& v) {
					// Allocate and initialize some data; make this big so
					// we can see the impact on the process memory use:
					size_t size = v.size();
					bool *foo = new bool[size];
					for (size_t i = 0; i < size; i++) foo[i] = v(i);

					// Create a Python object that will free the allocated memory when destroyed:
					WRAP_PY::capsule free_when_done(foo, [](void *f) { double *foo = reinterpret_cast<double *>(f); delete[] foo; });

					return WRAP_PY::array_t<bool>(
							{v.size()}, // shape
							{sizeof(bool)}, // C-style contiguous strides for double
							foo, // the data pointer
							free_when_done); // numpy array references this parent
			})
			.def("set", &boolN::set)      // 1445
			.def("setAllValue", &boolN::setAllValue) // 1445
			.def("resize", &boolN::resize)        // 1445
			.def("size", &boolN::size)                  // 1445
			.def("__repr__", [](boolN const& in)->std::string{ return in.shortOutput().tostring();})
			.def("output", [](boolN const& in)->std::string{ return in.output().tostring();})
			.def("__call__", (bool (boolN::*)(int i))&boolN::operator[])    // 1445
			.def("findZeroCrossing", &boolN::findZeroCrossing) // 1445
			.def("findLocalOptimum", &boolN::findLocalOptimum) // 1445
			.def("save", &boolN::save) // 1445
			.def("load", &boolN::load) // 1445
			.def("findNearest", &boolN::findNearest) // 1445
			.def("find", (int (boolN::*)(int start, bool bValue)const) &boolN::find, "start"_a, "bValue"_a=true) // 1445
			.def("findPrev", (int (boolN::*)(int i, bool bValue))&boolN::findPrev) // 1445
			.def("_or", (void (boolN::*)(const boolN& a, const boolN& b))&boolN::_or) // 1445
			.def("_and", (void (boolN::*)(const boolN& a, const boolN& b))&boolN::_and) // 1445
			.def("__or__",[](const boolN& a, const boolN& b)->boolN{ boolN c; c._or(a,b); return c;})
			.def("__and__",[](const boolN& a, const boolN& b)->boolN{ boolN c; c._and(a,b); return c;})
			.def("__invert__",[](const boolN& a)->boolN{ boolN c; c.negate(a); return c;})
			// concat
			.def("__add__",[](const boolN& self, const boolN& b)->boolN{
					boolN c(self.size()+b.size());
					c.range(0, self.size()).assign(self);
					c.range(self.size(), c.size()).assign(b);
					return c;
					})
			.def("range", (boolNView (*)(boolN const& a, int start, int end))&__pybindgen___boolN_wrapper::_range, WRAP_PY::keep_alive<0,1>()) // 1458
			.def("slice", [](boolN const& self, int scol, int ecol)->boolNView{
					if (scol<0 )
					scol=self.size()+scol;
					if (ecol<=0 )
					ecol=self.size()+ecol;
					return self.range(scol, ecol);
					}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("__getitem__",
			   [](boolN const&v, int i) -> bool {
					if(i<0) i+=v.size();
				   if (i >= v.size())
					   throw index_error();
					return v[i];
				 })
			; // end of class impl___pybindgen___boolN                    // 1505
		class_<boolNView , boolN> (mainlib, "boolNView")                       // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			; // end of class impl___pybindgen___boolNView                // 1505
		class_<CPixelRGB8 > (mainlib, "CPixelRGB8")                     // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<uchar,uchar,uchar>())                               // 1426
			.def_readwrite("R", &CPixelRGB8::R)
			.def_readwrite("G", &CPixelRGB8::G)
			.def_readwrite("B", &CPixelRGB8::B)
			; // end of class impl___pybindgen___CPixelRGB8               // 1505

		int (intvectorn::*getValue1)( int i ) const=&intvectorn::getValue;
		intvectornView (intvectorn::*range)(int start, int end, int step)=&intvectorn::range;

		struct intvectorn_
		{
			static void assign(intvectorn & l, WRAP_PY::list ll) 
			{
				l.setSize(len(ll));

				for(int i=0,ni=len(ll); i<ni; i++)
					l[i]=ll[i].cast<int>();
			}
			static int & value(intvectorn& v, int i)
			{
				return v[i];
			}

			static std::string output(intvectorn & q)
			{
				return q.output().ptr();
			}
			// this function does not copy memory
			static PyObject* ref(intvectorn const& v)
			{
				npy_intp dims[1];
				dims[0]=v.size();
				npy_intp strides[1];
				strides[0]=sizeof(int)*v._getStride();
				int* vv=&v[0];
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_INT), 1, dims, strides, vv, 
						NPY_ARRAY_C_CONTIGUOUS 
						| NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		class_<intvectorn>(mainlib, "intvectorn")
			.def(init<>())
			.def(init<int>())                                             
			// concat
			.def("__or__",[](const intvectorn& self, const intvectorn& b)->intvectorn{
					intvectorn c(self.size()+b.size());
					c.range(0, self.size()).assign(self);
					c.range(self.size(), c.size()).assign(b);
					return c;
					})
			.def("copy", [](intvectorn const&v )->intvectorn *{ return new intvectorn(v);}, TAKE_OWNERSHIP )
			.def("assign", &intvectorn_::assign)
			.def("value", getValue1)
			.def("__call__", getValue1)
			.def("size", &intvectorn::size)
			.def("setSize", &intvectorn::setSize)
			.def("resize", &intvectorn::resize)
			.def("find", [](intvectorn const& self, int v)->intvectorn{
					intvectorn out;
					out.findIndex(self, v);
					return out;
					})
			.def("__setitem__",
				[](intvectorn &v, int i, int t) {
					while(i<0) i+=v.size();
					if (i >= v.size())
						 throw index_error();
					 v[i] = t;
				 }) 
			.def("__getitem__",
			   [](intvectorn const&v, int i) -> int & {
					if(i<0) i+=v.size();
				   if (i >= v.size())
					   throw index_error();
					return v[i];
				 })
			.def("__call__", getValue1)
			.def("range", static_cast<intvectornView (intvectorn::*)(int, int, int)>(&intvectorn::range), WRAP_PY::keep_alive<0,1>(), "start"_a, "end"_a,"step"_a=1)
			.def_property_readonly("array", [](intvectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(intvectorn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](intvectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(intvectorn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("setAt", [](intvectorn &v, intvectorn const& columnIndex, intvectorn const& value){v.setAt(columnIndex, value);}) // 1445
			.def("findIndex", [](intvectorn & v,intvectorn const& source, int value){ v.findIndex(source, value);})
			.def("findIndex", [](intvectorn & v,boolN const& source, bool value){ v.findIndex(source, value);})
			.def("sortedOrder", (void (intvectorn::*)(vectorn const & input))&intvectorn::sortedOrder) // 1445
			.def("makeSamplingIndex", (void (intvectorn::*)(int nLen, int numSample))&intvectorn::makeSamplingIndex) // 1445
			.def("makeSamplingIndex2", (void (intvectorn::*)(int nLen, int numSample))&intvectorn::makeSamplingIndex2) // 1445
			.def("bubbleOut", (void (intvectorn::*)(int start, int end))&intvectorn::bubbleOut) // 1445
			.def("__repr__", [](intvectorn const& in)->std::string{ return in.output().tostring();})
			.def("maximum", (int (intvectorn::*)())&intvectorn::maximum)  // 1445
			.def("minimum", (int (intvectorn::*)())&intvectorn::minimum)  // 1445
			.def("sum", (int (intvectorn::*)())&intvectorn::sum)          // 1445
			.def("toVectorn", (vectorn (intvectorn::*)())&intvectorn::toVectorn) // 1445
			.def("findFirstIndex", (int (intvectorn::*)(int value))&intvectorn::findFirstIndex) // 1445
			.def("pushBack", (void (intvectorn::*)(int x))&intvectorn::pushBack) // 1445
			.def("pushFront", (void (intvectorn::*)(int x))&intvectorn::pushFront) // 1445
			.def("size", (int (intvectorn::*)())&intvectorn::size)        // 1445
			.def("setSize", (void (intvectorn::*)(int))&intvectorn::setSize) // 1445
			.def("resize", (void (intvectorn::*)(int))&intvectorn::resize) // 1445
			.def("set", (void (intvectorn::*)(int i, int d))&intvectorn::setValue) // 1445
			.def("colon", (void (intvectorn::*)(int start, int endf, int stepSize))&intvectorn::colon) // 1445
			.def("setAllValue", (void (intvectorn::*)(int v))&intvectorn::setAllValue) // 1445
			.def("assign", (void (intvectorn::*)(const intvectorn &other))&intvectorn::assign) // 1445
			.def("get", [](intvectorn const& a, int i) -> int{ return a[i];})
			.def("set", [](intvectorn & a, int i, int d){ a[i]=d;})
			.def("set", [](intvectorn & a, int i, double d){ a[i]=(int)d;})
			.def("radd", [](intvectorn& a, int v){ a+=v;})
			.def("radd", [](intvectorn& a, const intvectorn& v){ a+=v;})
			.def("rsub", [](intvectorn& a, int v){ a-=v;})
			.def("rsub", [](intvectorn& a, const intvectorn& v){ a-=v;})
			.def("rdiv", [](intvectorn& a, int v){ a/=v;})
			.def("rmult", [](intvectorn& a, int v){ a*=v;})
			.def("rmult", [](intvectorn& a, const intvectorn& v){ a*=v;})
			.def("count", [](intvectorn & a, int b)->int{
					int count=0;
					for(int i=0; i<a.size(); i++)
					if(a[i]==b) count++;
					return count;
					})
			.def(self+int())
			.def(self-int())
		;
		class_<intvectornView, intvectorn >(mainlib, "intvectornView")
			.def(init<const int*, int, int>())
			.def(init<const intvectorn &>())
			.def(init<const intvectornView &>())
			;
	}

	// vectorn
	{
		struct vectorn_
		{
			static void vectorn_assign(vectorn & l, WRAP_PY::list ll) 
			{
				l.setSize(len(ll));

				for(int i=0,ni=len(ll); i<ni; i++)
					l[i]=ll[i].cast<double>();
			}
			static m_real & vectorn_value(vectorn& v, int i)
			{
				return v[i];
			}

			static std::string vectorn_output(vectorn const& q)
			{
				return q.shortOutput().ptr();
			}
			// this function does not copy memory
			static PyObject* ref(vectorn const& v)
			{
				npy_intp dims[1];
				dims[0]=v.size();
				npy_intp strides[1];
				strides[0]=sizeof(double)*v._getStride();
				double* vv=&v[0];
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_DOUBLE), 1, dims, strides, vv, 
						NPY_ARRAY_C_CONTIGUOUS 
						| NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		vectorn& (vectorn::*assignv)(const vector3& other)=&vectorn::assign;
		vectorn& (vectorn::*assignq)(const quater& other)=&vectorn::assign;
		void (vectorn::*setValue1)( int i, m_real d )=&vectorn::setValue;
		void	(vectorn::*setAllValue)(m_real d)=&vectorn::setAllValue;
		m_real (vectorn::*getValue1)( int i ) const=&vectorn::getValue;
		vectornView (vectorn::*range)(int start, int end, int step)=&vectorn::range;

		struct __pybindgen___vectorn_wrapper
		{                                                             // 1382
			inline static m_real get(vectorn const& a, int i)
			{
				return a[i];
			}
			inline static void set(vectorn & a, int i, m_real d)
			{
				a[i]=d;
			}
			inline static void assign(vectorn & a, intvectorn const& b)
			{
				a.setSize(b.size());
				for(int i=0; i<a.size(); i++) a[i]=(double)b[i];
			}
			inline static void assign(vectorn & a, floatvec const& b)
			{
				a.setSize(b.size());
				for(int i=0; i<a.size(); i++) a[i]=(double)b[i];
			}
			inline static void radd(vectorn & a, m_real b)    {a+=b;}
			inline static void rsub(vectorn & a, m_real b)    {a-=b;}
			inline static void rdiv(vectorn & a, m_real b) {a/=b;}
			inline static void rmult(vectorn & a, m_real b) {a*=b;}
			inline static void rmult(vectorn & a, vectorn const&b) {Msg::verify(a.size()==b.size(), "size err");for(int i=0; i<a.size(); i++) a[i]*=b[i];}
			inline static void setAllValue(vectorn & a, m_real b) {a.setAllValue(b);}
			inline static void radd(vectorn & a, vectorn const& b)    {Msg::verify(a.size()==b.size(), "size err");a+=b;}
			inline static void rsub(vectorn & a, vectorn const& b)    {Msg::verify(a.size()==b.size(), "size err");a-=b;}
			inline static void smoothTransition(vectorn &c, m_real s, m_real e, int size)
			{
				c.setSize(size);
				for(int i=0; i<size; i++)
				{
					m_real sv=sop::map(i, 0, size-1, 0,1);
					c[i]=sop::map(sop::smoothTransition(sv), 0,1, s, e);
				}
			}
			inline static void clamp(vectorn &cc, double a, double b){
				for (int i=0; i<cc.size(); i++)
				{
					double c=cc[i];
					c=(c<a)?a:c;
					c=(c>b)?b:c;
					cc[i]=c;
				}
			}
			inline static void clamp(vectorn &cc, vectorn const& aa, vectorn const&bb){
				assert(aa.size()==bb.size());
				assert(aa.size()==cc.size());
				for (int i=0; i<cc.size(); i++)
				{
					double c=cc[i];
					double a=aa[i];
					double b=bb[i];
					c=(c<a)?a:c;
					c=(c>b)?b:c;
					cc[i]=c;
				}
			}
			// 1383
		};                                                            // 1384
		class_<vectorn>(mainlib, "vectorn")
			.def(init<>())
			.def(init<int>())
			.def("reserve", &vectorn::reserve)
			.def("to_se3",[](vectorn const&v)->Liegroup::se3{ Liegroup::se3 out; out.W()=v.toVector3(0); out.V()=v.toVector3(3); return out;})
			.def("copy", [](vectorn const&v )->vectorn *{ return new vectorn(v);}, TAKE_OWNERSHIP )
			.def("assign", &vectorn_::vectorn_assign)
			.def("assign", assignv, RETURN_REFERENCE)
			.def("assign", (vectorn& (vectorn::*)(const vectorn&))(&vectorn::assign), RETURN_REFERENCE)
			.def("assign", assignq, RETURN_REFERENCE)
			.def("assign",  [](vectorn & v, transf const& t){ v.setSize(7); v.setTransf(0, t);})
			.def("zero",  [](vectorn & v){ v.setAllValue(0.0);})
			.def_property_readonly("array", [](vectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(vectorn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](vectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(vectorn_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			// concat
			.def("__or__",[](const vectorn& self, const vectorn& b)->vectorn{
					vectorn c(self.size()+b.size());
					c.range(0, self.size()).assign(self);
					c.range(self.size(), c.size()).assign(b);
					return c;
					})
			.def(WRAP_PY::pickle(
						[](const vectorn &p) { // __getstate__
						auto my_tuple = WRAP_PY::tuple(p.size());
						for(int i=0; i<p.size(); i++)
						my_tuple[i]=p[i];
						/* Return a tuple that fully encodes the state of the object */
						return my_tuple;
						},
						[](WRAP_PY::tuple t) { // __setstate__
						vectorn out;
						out.setSize(t.size());
						for(int i=0; i<t.size(); i++)
						out[i]=t[i].cast<double>();
						return out;
						}))
			//member functions
			//.def("tolist", &vectorn_::tolist) 
			.def("isnan", &vectorn::isnan)
			.def("__call__", [](vectorn& v, int i)->double{ return v(i);})
			.def("resample", (void (vectorn::*)(vectorn const& vec, int numSample))&vectorn::resample) // 1445
			.def("bubbleOut", (void (vectorn::*)(int start, int end))&vectorn::bubbleOut) // 1445
			.def("sub", [](vectorn&v,vectorn const& a, vectorn const& b){v.sub(a,b);})
			.def("add", [](vectorn&v,vectorn const& a, vectorn const& b){v.add(a,b);})
			.def("_extract", (void (vectorn::*)(vectorn const& source, intvectorn const& index))&vectorn::extract) // 1445
			.def("assignSelective", [](vectorn&v ,intvectorn const& index, vectorn const& value){ v.assignSelective(index, value);})
			.def("value", getValue1)
			.def("get", getValue1)
			.def("set", setValue1)
			.def("getStride", &vectorn::_getStride)
			.def("setAllValue", setAllValue)
			.def("output", &vectorn_::vectorn_output)
			.def("__repr__", &vectorn_::vectorn_output)
			.def("toVector3", &vectorn::toVector3, "startIndex"_a=0) // defaultArgument
			.def("toQuater", &vectorn::toQuater, "startIndex"_a=0)
			.def("toQuater6", &vectorn::toQuater6, "startIndex"_a=0)
			.def("toTransf", &vectorn::toTransf, "startIndex"_a=0)
			.def("toTransf9", &vectorn::toTransf9, "startIndex"_a=0)
			.def("setVec3", &vectorn::setVec3)
			.def("setQuater", &vectorn::setQuater)
			.def("setQuater6", &vectorn::setQuater6)
			.def("setTransf", &vectorn::setTransf)
			.def("setTransf9", &vectorn::setTransf9)
			.def("convertAxesYZtoTorch6D", &vectorn::convertAxesYZtoTorch6D)
			.def("convertTorch6DtoAxesYZ", &vectorn::convertTorch6DtoAxesYZ)
			.def("pushBack", [](vectorn &v, double x){v.pushBack(x);})
			.def("size", &vectorn::size)
			.def("setSize", &vectorn::setSize)
			.def("resize", &vectorn::resize)
			.def("set", (void (*)(vectorn & a, int i, m_real d))&__pybindgen___vectorn_wrapper::set)     // 1458
			.def("range", static_cast<vectornView (vectorn::*)(int, int, int)>(&vectorn::range), WRAP_PY::keep_alive<0,1>(), "start"_a, "end"_a,"step"_a=1)
			.def("slice", [](vectorn const& self, int scol, int ecol)->vectornView{
					if (scol<0 )
					scol=self.size()+scol;
					if (ecol<=0 )
					ecol=self.size()+ecol;
					return self.range(scol, ecol);
					}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

			.def("length", &vectorn::length)
			.def("minimum", (m_real (vectorn::*)() const)&vectorn::minimum)     // 1445
			.def("maximum", (m_real (vectorn::*)() const)&vectorn::maximum)     // 1446
			.def("sum", &vectorn::sum)
			.def("squareSum", &vectorn::squareSum)
			.def("avg", &vectorn::avg)
			.def("minimum", (void (vectorn::*)(const matrixn& other))&vectorn::minimum) // 1445
			.def("maximum", (void (vectorn::*)(const matrixn& other))&vectorn::maximum) // 1445
			.def("mean", (void (vectorn::*)(const matrixn& other))&vectorn::mean) // 1445
			.def("lengths", (void (vectorn::*)(matrixn const& in))&vectorn::lengths) // 1445
			.def("argMin", &vectorn::argMin)
			.def("argMax", &vectorn::argMax)
			.def("argNearest", &vectorn::argNearest)
			//.def("normalize", &vectorn::normalize, RETURN_REFERENCE)
			.def("__setitem__",
				[](vectorn &v, int i, double t) {
					if(i<0) i+=v.size();
					if (i >= v.size())
						 throw index_error();
					 v[i] = t;
				 }) 
			.def("__getitem__",
			   [](vectorn const&v, int i) -> double & {
					if(i<0) i+=v.size();
				   if (i >= v.size())
					   throw index_error();
					return v[i];
				 })
			.def("colon", (void (vectorn::*)(m_real start, m_real stepSize,int nSize))&vectorn::colon) // 1445
			.def("colon", (void (vectorn::*)(m_real start, m_real stepSize))&vectorn::colon) // 1445
			.def("colon2", (void (vectorn::*)(double start, double end, double stepSize))&vectorn::colon2) // 1445
			.def("linspace", [](vectorn&v, m_real x1, m_real x2){ v.linspace(x1,x2);})
			.def("linspace", [](vectorn&v, m_real x1, m_real x2, int nSize){ v.linspace(x1,x2, nSize);})
			.def("uniform", [](vectorn&v,m_real x1, m_real x2){ v.uniform(x1, x2);})
			.def("uniform", [](vectorn&v,m_real x1, m_real x2, int nSize){ v.uniform(x1, x2,nSize);})
			.def("column", (matrixnView (vectorn::*)())&vectorn::column)  // 1445
			.def("row", (matrixnView (vectorn::*)())&vectorn::row)        // 1445
			.def("fromMatrix", (void (vectorn::*)(matrixn const& in))&vectorn::fromMatrix) // 1445
			// static member
			.def("radd", (void (*)(vectorn & a, m_real b))&__pybindgen___vectorn_wrapper::radd)          // 1458
			.def("assign", (void (*)(vectorn & a, intvectorn const& d))&__pybindgen___vectorn_wrapper::assign) // 1458
			.def("assign", (void (*)(vectorn & a, floatvec const& b))&__pybindgen___vectorn_wrapper::assign) // 1458
			.def("rsub", (void (*)(vectorn & a, m_real b))&__pybindgen___vectorn_wrapper::rsub)          // 1458
			.def("rdiv", (void (*)(vectorn & a, m_real b))&__pybindgen___vectorn_wrapper::rdiv)          // 1458
			.def("rmult", (void (*)(vectorn & a, m_real b))&__pybindgen___vectorn_wrapper::rmult)        // 1458
			.def("rmult", (void (*)(vectorn & a, vectorn const&b))&__pybindgen___vectorn_wrapper::rmult) // 1458
			.def("clamp", (void (*)(vectorn &cc, double a, double b))&__pybindgen___vectorn_wrapper::clamp) // 1458
			.def("clamp", (void (*)(vectorn &cc, vectorn const& a, vectorn const& b))&__pybindgen___vectorn_wrapper::clamp) // 1458
			.def("setAllValue", (void (*)(vectorn & a, m_real b))&__pybindgen___vectorn_wrapper::setAllValue) // 1458
			.def("radd", (void (*)(vectorn & a, vectorn const& b))&__pybindgen___vectorn_wrapper::radd)  // 1458
			.def("rsub", (void (*)(vectorn & a, vectorn const& b))&__pybindgen___vectorn_wrapper::rsub)  // 1458
			.def("matView", (matrixnView (*)(vectorn const& a, int col))&matView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458
			.def("vec3View", (vector3NView (*)(vectorn const&))&vec3View, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1458
			.def("quatView", (quaterNView (*)(vectorn const&))&quatView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
  // 1458
			.def("smoothTransition", (void (*)(vectorn &c, m_real s, m_real e, int size))&__pybindgen___vectorn_wrapper::smoothTransition) // 1458
			.def("sample", (m_real (*)(vectorn const& in, m_real criticalTime))&v::sample) // 1458
			.def("interpolate", (void (*)(vectorn & out, m_real t, vectorn const& a, vectorn const& b))&v::interpolate) // 1458
			.def("hermite", (void (*)(vectorn& out, double a, double b, int duration, double c, double d))&v::hermite) // 1458
			.def("hermite", (void (*)(vectorn& out, double t, double T, const vectorn& a, const vectorn va, const vectorn& b,  const vectorn& vb))&v::hermite) // 1458
			.def("quintic", (void (*)(vectorn& out, double x0, double v0, double a0, double x1, double v1, double a1, double T))&v::quintic) // 1458
			.def("dotProduct", [](vectorn const& a, vectorn const& b)->double{ return a%b;})
			.def(-self) // neg (unary minus)
			.def(self + self) // add (homogeneous)
			.def(self - self) // add (homogeneous)
			.def(self * self) // mul
			.def(self / self) // mul
			.def(self + double()) // mul
			.def(self - double()) // mul
			.def(self * double()) // mul
			.def(self / double()) // mul
		;
		class_<vectornView, vectorn >(mainlib, "vectornView")
			.def(init<double*, int, int>())
			.def(init<const vectorn &>())
			.def(init<const vectornView &>())
			;
	}

	struct __pybindgen__math_KovarMetric_wrapper
	{                                                             // 1382
		static matrix4& _property_get_m_transfB(KovarMetric const& a) { return (matrix4 &) a.m_transfB; }
		static void _property_set_m_transfB(KovarMetric & a, const matrix4 &m) { a.m_transfB=m; }
		static matrixn& _property_get_m_transformedB(KovarMetric const& a) { return (matrixn &) a.m_transformedB; }
		static void _property_set_m_transformedB(KovarMetric & a, const matrixn &m) { a.m_transformedB=m; }
	};                                                            // 1384
	class_<KovarMetric > (mainlib, "KovarMetric")             // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<bool>())                                            // 1426
		.def("calcDistance", &KovarMetric::CalcDistance)              // 1445
		.def_property("m_transfB", (matrix4 & (*)(KovarMetric const& a))&__pybindgen__math_KovarMetric_wrapper::_property_get_m_transfB, 
				&__pybindgen__math_KovarMetric_wrapper::_property_set_m_transfB)
		.def_property("m_transformedB", (matrixn & (*)(KovarMetric const& a))&__pybindgen__math_KovarMetric_wrapper::_property_get_m_transformedB, 
				&__pybindgen__math_KovarMetric_wrapper::_property_set_m_transformedB)
		; // end of class impl___pybindgen__math_KovarMetric          // 1506
	/////////////////////////////////////////////////////////////////
	// Mainlib
	/////////////////////////////////////////////////////////////////


	// functions in namespace RE
	{
		class_<Ogre::MovableObject>(mainlib, "MovableObject")
			.def("setCastShadows",[](Ogre::MovableObject* o, bool enabled){
					OGRE_VOID(o->setCastShadows(enabled);)
					})
			;
	}
	{
		class_<Ogre::SceneNode>(mainlib, "SceneNode")
		.def("attachObject", [](Ogre::SceneNode* o, Ogre::MovableObject* e){
				OGRE_VOID( o->attachObject(e));
				})
#ifndef NO_OGRE
		.def("numAttachedObjects", (int (Ogre ::SceneNode::*)())&Ogre ::SceneNode::numAttachedObjects) // 1443
		.def("setVisible", (void (Ogre ::SceneNode::*)(bool visible))&Ogre ::SceneNode::setVisible) // 1443
		.def("flipVisibility", (void (Ogre ::SceneNode::*)())&Ogre ::SceneNode::flipVisibility) // 1443
#endif
		.def("resetToInitialState", [](Ogre::SceneNode* pNode){
#ifndef NO_OGRE
				RE::resetToInitialState(pNode);
#endif
				})
		.def("createChildSceneNode", [](Ogre::SceneNode* pNode){
			OGRE_PTR(return pNode->createChildSceneNode());
				}, RETURN_REFERENCE)
		.def("createChildSceneNode", [](Ogre::SceneNode* pNode, const char* name){
			OGRE_PTR(return RE::createChildSceneNode(pNode, name));
				}, RETURN_REFERENCE)
		.def("translate", [](Ogre::SceneNode* pNode, vector3 const& t){
				OGRE_VOID(pNode->translate(t.x, t.y, t.z));
				})
		.def("translate", [](Ogre::SceneNode* pNode, double x, double y, double z){
				OGRE_VOID(pNode->translate(x, y, z));
				})
		.def("rotate", [](Ogre::SceneNode* pNode, quater const& t){
				OGRE_VOID(pNode->rotate(ToOgre(t)));
				})
		.def("setDirection", [](Ogre::SceneNode* pNode, vector3 const& t){
				OGRE_VOID( pNode->setDirection(t.x,t.y,t.z));
				}) 
		.def("getPosition", [](Ogre::SceneNode* pNode)->vector3{
				vector3 t;
#ifndef NO_OGRE
				t=ToBase(pNode->getPosition());
#endif
				return t;
				}) 
		.def("getScale", [](Ogre::SceneNode* pNode)->vector3{
				vector3 t;
#ifndef NO_OGRE
				t=ToBase(pNode->getScale());
#endif
				return t;
				}) 
		.def("getOrientation", [](Ogre::SceneNode* pNode)->quater{
				quater t;
#ifndef NO_OGRE
				t=ToBase(pNode->getOrientation());
#endif
				return t;
				}) 
		.def("scale", [](Ogre::SceneNode* pNode, vector3 const& t){
				OGRE_VOID(pNode->scale(t.x, t.y, t.z));
				})
		.def("scale", [](Ogre::SceneNode* pNode, m_real x, m_real y, m_real z){
				OGRE_VOID(pNode->scale(x, y, z));
				})
		.def("scale", [](Ogre::SceneNode* pNode, m_real x){
				OGRE_VOID(pNode->scale(x, x, x));
				})
		.def("setPosition", [](Ogre::SceneNode* pNode, m_real x, m_real y, m_real z){
				OGRE_VOID(pNode->setPosition(x,y,z));
				})
		.def("setPosition", [](Ogre::SceneNode* pNode, vector3 const& t){
				OGRE_VOID(pNode->setPosition(t.x,t.y,t.z));
				})
		.def("setScale", [](Ogre::SceneNode* pNode, m_real x, m_real y, m_real z){
				OGRE_VOID(pNode->setScale(x,y,z));
				})
		.def("setScale", [](Ogre::SceneNode* pNode, vector3 const& t){
				OGRE_VOID(pNode->setScale(t.x,t.y,t.z));
				})
		.def("setOrientation", [](Ogre::SceneNode* pNode, m_real w, m_real x, m_real y, m_real z){
				OGRE_VOID( pNode->setOrientation(w, x, y, z));
				})
		.def("setOrientation", [](Ogre::SceneNode* pNode, quater const& q){
				OGRE_VOID( pNode->setOrientation(q.w, q.x, q.y, q.z));
				})
		.def("getEntity", (Ogre ::Item * (*)(Ogre::SceneNode* node))&RE::getItem,RETURN_REFERENCE ) // 1446
			;
	}
	{
		class_<Ogre ::SceneManager > (mainlib, "SceneManager")          // 1388
			.def("setFog", (void (*)(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max))&impl_luna__interface_SceneManager::setFog) // 1446
			.def("setFogExponential", (void (*)(Ogre::SceneManager* pmgr, double r, double g, double b, double a, double min, double max))&impl_luna__interface_SceneManager::setFogExponential) // 1446
			.def("setFogNone", (void (*)(Ogre::SceneManager* pmgr))&impl_luna__interface_SceneManager::setFogNone) // 1446
			.def("createEntity", (Ogre ::Item * (*)(Ogre::SceneManager* pmgr, const char* id, const char* mesh))&impl_luna__interface_SceneManager::createEntity, RETURN_REFERENCE) // 1446
			.def("getSceneNode", (Ogre ::SceneNode * (*)(Ogre::SceneManager* pmgr, const char* id))&impl_luna__interface_SceneManager::getSceneNode, RETURN_REFERENCE) // 1446
		/*
		class SceneNode_Wrapper
		{
		public:
			Ogre::SceneNode* ptrj
			SceneNode_Wrapper(){}
			SceneNode_Wrapper(Ogre::SceneNode* ptr){this->ptr=ptr;}
			void move(quater const& q, vector3 const& v){ RE::moveEntity(ptr, q, v);}
			static SceneNode_Wrapper createEntity(const char* id, const char* filename)			
			{
				return SceneNode_Wrapper(RE::createEntity(id, filename));
			}

			static void removeEntity(SceneNode_Wrapper & a)
			{	
				RE::removeEntity(a.ptr);
				a.ptr=NULL;
			}
		};
		
		class_<SceneNode_Wrapper>("SceneNode")
			.def("move", &SceneNode_Wrapper::move)
		;

		PLDPrimSkin* (*createSkin1)(const Motion& mot)=&RE::createSkin; 		
		def("createSkin", createSkin1, RETURN_REFERENCE);
		def("createEntity", SceneNode_Wrapper::createEntity);
		def("removeEntity", SceneNode_Wrapper::removeEntity);		
		def("generateUniqueName", RE::generateUniqueName);
		*/
			.def("createLight", (Ogre ::Light * (*)(Ogre::SceneManager* pmgr, const char* id))&impl_luna__interface_SceneManager::createLight, RETURN_REFERENCE) // 1446
			.def("setSkyBox", (void (*)(Ogre::SceneManager* pmgr, bool enable, const char* materialName))&impl_luna__interface_SceneManager::setSkyBox) // 1446
			.def("hasSceneNode", (bool (*)(Ogre::SceneManager* pmgr, const char * name))&impl_luna__interface_SceneManager::hasSceneNode) // 1446
			; // end of class impl_LunaTraits<Ogre ::SceneManager >       // 1562
	}
	{
		class_<OBJloader ::Face > (mainlib, "Face")               // 1393
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
			.def("setIndex", [](OBJloader ::Face& self, int i, int j, int k){ self.setIndex(i,j,k);})                 // 1450
			.def("vertexIndex", &OBJloader ::Face::vi)           // 1450
			.def("normalIndex", [](OBJloader ::Face& self, int i)->int{ return self.normalIndex(i);})
			.def("texCoordIndex", [](OBJloader ::Face& self, int i)->int{ return self.texCoordIndex(i);})       // 1450
			.def("colorIndex", [](OBJloader ::Face& self, int i)->int{ return self.colorIndex(i);})       // 1450
			; // end of class impl___pybindgen__OBJloader_Face            // 1511
	}
	{
		class_<OBJloader::Mesh>(mainlib, "Mesh")
			.def(init<>())
			.def("copy", [](OBJloader::Mesh const&v )->OBJloader::Mesh *{ return new OBJloader::Mesh(v);}, TAKE_OWNERSHIP )
			.def_readwrite("isBoundaryVertex", &OBJloader::Mesh::isBoundaryVertex)
			.def_readwrite("faceGroups", &OBJloader::Mesh::faceGroups)
			.def("init", (void (OBJloader::Mesh::*)(const vector3N& vertices, const intvectorn& triangles))&OBJloader::Mesh::init)
			.def("init", (void (OBJloader::Mesh::*)(const vector3N& vertices, const vector3N& normals, const intvectorn& triangles))&OBJloader::Mesh::init)
			.def("_mergeDuplicateVertices",  (intvectorn (OBJloader::Mesh::*)(double distThr))&OBJloader::Mesh::_mergeDuplicateVertices)
			.def("_mergeDuplicateVertices",  (intvectorn (OBJloader::Mesh::*)())&OBJloader::Mesh::_mergeDuplicateVertices)
			.def("numFace", &OBJloader::Mesh::numFace)
			.def("numVertex", &OBJloader::Mesh::numVertex)
			.def("numNormal", &OBJloader::Mesh::numNormal)
			.def("numTexCoord", &OBJloader::Mesh::numTexCoord)
			.def("numColor", &OBJloader::Mesh::numColor)
			.def("getVertex", (vector3& (OBJloader::Mesh::*)(int i))&OBJloader::Mesh::getVertex, RETURN_REFERENCE)
			.def("getNormal", (vector3& (OBJloader::Mesh::*)(int i))&OBJloader::Mesh::getNormal, RETURN_REFERENCE)
			.def("getTexCoord", (vector2& (OBJloader::Mesh::*)(int i))&OBJloader::Mesh::getTexCoord, RETURN_REFERENCE)
			.def("getColor", (vector4& (OBJloader::Mesh::*)(int i))&OBJloader::Mesh::getColor, RETURN_REFERENCE)
			.def("saveMesh", (bool (OBJloader::Mesh::*)(const char* filename_)) &OBJloader::Mesh::saveMesh)
			.def("loadMesh", (bool (OBJloader::Mesh::*)(const char* filename_))&OBJloader::Mesh::loadMesh)
			.def("loadMesh", (bool (OBJloader::Mesh::*)(const char* filename_, bool bInit))&OBJloader::Mesh::loadMesh)
			.def("saveOBJ", (bool (OBJloader::Mesh::*)(const char* filename_, bool vn, bool vt))&OBJloader::Mesh::saveObj)
			.def("getFace", (OBJloader::Face& (OBJloader::Mesh::*)(int i))&OBJloader::Mesh::getFace)
			.def("loadOBJ", &OBJloader::Mesh::loadObj)
			.def("assignMesh", &OBJloader::Mesh::copyFrom)
			.def("mergeMesh", &OBJloader::Mesh::merge)
			.def("transform", &OBJloader::Mesh::transform)
			.def("resize", (void (OBJloader::Mesh::*)(int numVertex, int numFace))&OBJloader::Mesh::resize)
			.def("resize", (void (OBJloader::Mesh::*)(int numVertex, int numNormal, int numTexCoord, int numColor, int numFace))&OBJloader::Mesh::resize)
			.def("pack", &OBJloader::Mesh::pack)
			.def("unpack", &OBJloader::Mesh::unpack)
			.def("calculateVertexNormal", &OBJloader::Mesh::calculateVertexNormal)
			.def("removeFaces", &OBJloader::Mesh::removeFaces)
			.def("addVertices", &OBJloader::Mesh::addVertices)
			.def("addNormals", &OBJloader::Mesh::addNormals)
			.def("addFaces", &OBJloader::Mesh::addFaces)
			.def("resizeIndexBuffer", &OBJloader::Mesh::resizeIndexBuffer)
			.def("resizeVertexBuffer", &OBJloader::Mesh::resizeVertexBuffer)
			.def("calcFaceCenter", &OBJloader::Mesh::calcFaceCenter)
			.def("calcFaceNormal", &OBJloader::Mesh::calcFaceNormal)
			.def("calcMeshCenter", &OBJloader::Mesh::calcMeshCenter)
			.def("_initBox", &OBJloader::createBox) // 1458
			.def("_initCylinder", &OBJloader::createCylinder) // 1458
			.def("createCircle", &OBJloader::createCircle) // 1458
			.def("_initPlane", (void (*)(OBJloader::Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ))&OBJloader::createPlane) // 1458
			;
		class_<OBJloader::Terrain, OBJloader::Mesh>(mainlib, "Terrain")
			.def(init<const std::string&, int , int , double , double , double , int , int , bool >())
			.def(init<const std::string&, int , int , double , double , double , int , int >())
			.def(init<vectorn const &,m_real,m_real,m_real,int,int,bool>()) // 1425
			.def("getHeightMap", (const matrixn & (OBJloader ::Terrain::*)())&OBJloader ::Terrain::getHeightMap) // 1443
			.def("getSize", (vector3 (OBJloader ::Terrain::*)())&OBJloader ::Terrain::getSize) // 1443
			.def("height", (m_real (OBJloader ::Terrain::*)(vector2 x, vector3& normal) const)&OBJloader ::Terrain::height) // 1443
			.def("height", (m_real (OBJloader ::Terrain::*)(vector2 x) const)&OBJloader ::Terrain::height) // 1443
			.def("pick", (vector3 (OBJloader ::Terrain::*)(Ray const& ray, vector3& normal))&OBJloader ::Terrain::pick) // 1443
			.def("isInsideTerrain", (bool (OBJloader ::Terrain::*)(vector2 x))&OBJloader ::Terrain::isInsideTerrain) // 1443
			;
		class_<OBJloader ::Element > (mainlib, "Element")         // 1393
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
			.def_readwrite("elementType", &OBJloader::Element::elementType)
			.def_readwrite("elementSize", &OBJloader::Element::elementSize)
			.def_readwrite("tf", &OBJloader::Element::tf)
			.def_readwrite("material", &OBJloader::Element::material)
			; // end of class impl___pybindgen__OBJloader_Element         // 1511
		class_<OBJloader::Geometry, OBJloader::Mesh>(mainlib, "Geometry")
			.def(init<>())
			.def("copy", [](OBJloader::Geometry const&v )->OBJloader::Geometry *{ return new OBJloader::Geometry(v);}, TAKE_OWNERSHIP )
			.def_readwrite("faceGroups", &OBJloader::Mesh::faceGroups)
			.def("numElements", (int (OBJloader ::Geometry::*)())&OBJloader ::Geometry::numElements) // 1443
			.def("element", (OBJloader ::Element const & (OBJloader ::Geometry::*)(int i))&OBJloader ::Geometry::element, RETURN_REFERENCE) // 1443
			.def("mergeAllElements", (void (OBJloader ::Geometry::*)())&OBJloader ::Geometry::mergeAllElements) // 1443
			.def("scale", (void (OBJloader ::Geometry::*)(vector3 const& scalef))&OBJloader ::Geometry::scale) // 1443
			.def("scale", (void (OBJloader ::Geometry::*)(vector3 const& scalef, int ifacegroup))&OBJloader ::Geometry::scale) // 1443
			.def("scaleElements", (void (OBJloader ::Geometry::*)(vector3 const& scalef))&OBJloader ::Geometry::scaleElements) // 1443
			.def("scaleElements", (void (OBJloader ::Geometry::*)(vector3 const& scalef, int ifacegroup))&OBJloader ::Geometry::scaleElements) // 1443
			.def("rigidTransform", (void (OBJloader ::Geometry::*)(transf const& b))&OBJloader ::Geometry::rigidTransform) // 1443
			.def("rigidTransform", (void (OBJloader ::Geometry::*)(transf const& b, int ifacegroup))&OBJloader ::Geometry::rigidTransform) // 1443
			.def("scaleAndRigidTransform", (void (OBJloader ::Geometry::*)(matrix4 const& m))&OBJloader ::Geometry::scaleAndRigidTransform) // 1443
			.def("initBox", (void (OBJloader ::Geometry::*)(const vector3& size))&OBJloader ::Geometry::initBox) // 1443
			.def("initCylinder", (void (OBJloader ::Geometry::*)(double radius, double height, int numDivision))&OBJloader ::Geometry::initCylinder) // 1443
			.def("initCapsule", (void (OBJloader ::Geometry::*)(double radius, double height))&OBJloader ::Geometry::initCapsule) // 1443
			.def("initEllipsoid", (void (OBJloader ::Geometry::*)(const vector3& size))&OBJloader ::Geometry::initEllipsoid) // 1443
			.def("initPlane", (void (OBJloader ::Geometry::*)(double size_x, double size_z))&OBJloader ::Geometry::initPlane) // 1443
			.def("extractSubMesh", (void (OBJloader ::Geometry::*)(OBJloader::Geometry const& otherMesh, int isubMesh))&OBJloader ::Geometry::extractSubMesh) // 1443
			.def("merge", (void (OBJloader ::Geometry::*)(OBJloader::Geometry const& a, OBJloader::Geometry const& b))&OBJloader ::Geometry::merge) // 1443
			.def("convertToOBJ", (void (OBJloader ::Geometry::*)())&OBJloader ::Geometry::convertToOBJ) // 1443
			.def("assign", (void (OBJloader ::Geometry::*)(OBJloader::Geometry const& otherMesh))&OBJloader ::Geometry::copyFrom) // 1443
			.def("_updateMeshFromElements", (void (OBJloader ::Geometry::*)())&OBJloader ::Geometry::_updateMeshFromElements) // 1443
			.def("totalVolume", (double (OBJloader ::Geometry::*)())&OBJloader ::Geometry::totalVolume) // 1443
			.def("assignMesh", (void (OBJloader ::Geometry::*)(OBJloader::Mesh const& otherMesh))&OBJloader ::Geometry::assignMesh) // 1443
			.def("assignTerrain", (void (OBJloader ::Geometry::*)(OBJloader::Terrain const& otherMesh, vector3 const& trans))&OBJloader ::Geometry::assignTerrain) // 1443
			;
	}
	{
		class_<OBJloader ::MeshToEntity > (mainlib, "MeshToEntity")     // 1388
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1391
			.def(init<const OBJloader ::Mesh & ,const std::string >())
			.def(init<const OBJloader ::Mesh &,const std::string,bool,bool>()) // 1426
			.def(init<const OBJloader ::Mesh &,const std::string,bool,bool,bool,bool>()) // 1426
			.def(init<const OBJloader ::Mesh &,const std::string,bool,bool,bool,bool,bool>()) // 1426
			.def("updatePositions", (void (OBJloader ::MeshToEntity::*)())&OBJloader ::MeshToEntity::updatePositions) // 1443
			.def("updatePositionsAndNormals", (void (OBJloader ::MeshToEntity::*)())&OBJloader ::MeshToEntity::updatePositionsAndNormals) // 1443
			.def("createEntity", [](OBJloader::MeshToEntity& self, const std::string & entityName)->Ogre::Item* {
					return self.createEntity(entityName.c_str());}, RETURN_REFERENCE) // 1443
			.def("getLastCreatedEntity", (Ogre ::Item * (OBJloader ::MeshToEntity::*)())&OBJloader ::MeshToEntity::getLastCreatedEntity, RETURN_REFERENCE) // 1443
			; // end of class impl_LunaTraits<OBJloader ::MeshToEntity > // 1605
		class_<Ogre::Item, Ogre::MovableObject> (mainlib, "Entity")
#ifndef NO_GUI
			.def("setMaterialName", [](Ogre::Item& entity, const std::string name){ entity.setDatablockOrMaterialName(name);})
#endif
			;
		class_<Viewpoint > (mainlib, "Viewpoint")                       // 1388
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1391
			.def("GetViewMatrix", (int (Viewpoint::*)(matrix4& matView))&Viewpoint::GetViewMatrix) // 1447
			.def("SetViewMatrix", (int (Viewpoint::*)(matrix4 const& matView))&Viewpoint::SetViewMatrix) // 1447
			.def("setYUp", (void (Viewpoint::*)())&Viewpoint::setYUp)     // 1447
			.def("setZUp", (void (Viewpoint::*)())&Viewpoint::setZUp)     // 1447
			.def("setScale", (void (Viewpoint::*)(m_real f))&Viewpoint::setScale) // 1447
			.def("updateVPosFromVHD", (int (Viewpoint::*)())&Viewpoint::UpdateVPosFromVHD) // 1447
			.def("TurnRight", (int (Viewpoint::*)(m_real radian))&Viewpoint::TurnRight) // 1447
			.def("TurnLeft", (int (Viewpoint::*)(m_real radian))&Viewpoint::TurnLeft) // 1447
			.def("TurnUp", (int (Viewpoint::*)(m_real radian))&Viewpoint::TurnUp) // 1447
			.def("TurnDown", (int (Viewpoint::*)(m_real radian))&Viewpoint::TurnDown) // 1447
			.def("ZoomIn", (int (Viewpoint::*)(m_real ZoomAmount))&Viewpoint::ZoomIn) // 1447
			.def("ZoomOut", (int (Viewpoint::*)(m_real ZoomAmount))&Viewpoint::ZoomOut) // 1447
			.def("getZoom", (m_real (Viewpoint::*)(void))&Viewpoint::getZoom) // 1447
			.def("setZoom", (void (Viewpoint::*)(m_real))&Viewpoint::setZoom) // 1447
			.def("assign", (Viewpoint & (Viewpoint::*)(const Viewpoint& other))&Viewpoint::operator=,return_value_policy::reference ) // 1452
			.def("update", (void (*)(Viewpoint & view))&impl_luna__interface_Viewpoint::update)           // 1460
			.def("setClipDistances", (void (*)(Viewpoint& view, m_real fnear, m_real ffar))&impl_luna__interface_Viewpoint:: setClipDistances) // 1460
			.def("setFOVy", (void (*)(Viewpoint& view, m_real degree))&impl_luna__interface_Viewpoint:: setFOVy) // 1460
			.def("setNearClipDistance", (void (*)(Viewpoint& view, m_real dist))&impl_luna__interface_Viewpoint:: setNearClipDistance) // 1460
			.def("setOrthographicMode", (void (*)(Viewpoint& view, bool isOrtho))&impl_luna__interface_Viewpoint:: setOrthographicMode) // 1460
			.def_readwrite("vpos", &Viewpoint ::m_vecVPos)
			.def_readwrite("vat", &Viewpoint ::m_vecVAt)
			.def_readwrite("vup", &Viewpoint ::m_vecVUp)
			;
	}

	mainlib.def("transitionCost", MotionUtil::transitionCost);
	

#ifndef NO_GUI
	class_<FltkMotionWindow >(mainlib, "FltkMotionWindow")
		.def(init<int, int, int>())
		.def("addSkin",&FltkMotionWindow::addSkin)
		.def("releaseAllSkin",&FltkMotionWindow::releaseAllSkin)
		.def("releaseSkin",&FltkMotionWindow::releaseSkin)
		.def("detachAllSkin",&FltkMotionWindow::detachAllSkin)
		.def("detachSkin",&FltkMotionWindow::detachSkin)
		.def("getCurrFrame", &FltkMotionWindow::getCurrFrame)
		.def("getNumFrame", &FltkMotionWindow::getNumFrame)
		.def("getNumSkin", &FltkMotionWindow::getNumSkin)
		.def("getSkin", &FltkMotionWindow::getSkin, RETURN_REFERENCE)
		.def("changeCurrFrame",&FltkMotionWindow::changeCurrFrame)
		.def("playUntil",&FltkMotionWindow::playUntil)
		.def("playFrom",&FltkMotionWindow::playFrom)
	;
    class_<FltkScrollPanel > (mainlib, "FltkScrollPanel")     // 1393
		.def("addPanel", (void (FltkScrollPanel::*)(const char* filename))&FltkScrollPanel::addPanel) // 1451
		.def("addPanel", (void (FltkScrollPanel::*)(CImage* pSource))&FltkScrollPanel::addPanel) // 1451
		.def("createPanel", (CImage * (FltkScrollPanel::*)())&FltkScrollPanel::createPanel,return_value_policy::reference ) // 1456
		.def("addPanel", (void (FltkScrollPanel::*)(const boolN& bits, CPixelRGB8 color))&FltkScrollPanel::addPanel) // 1451
		.def("addPanel", (void (FltkScrollPanel::*)(const boolN& bits, CPixelRGB8 color, int startFrame))&FltkScrollPanel::addPanel) // 1451
		.def("addPanel", [](FltkScrollPanel& self, const intvectorn& indexes){ self.addPanel(indexes);})
		.def("addPanel", (void (FltkScrollPanel::*)(const vectorn& signal))&FltkScrollPanel::addPanel) // 1451
		.def("addPanel", (void (FltkScrollPanel::*)(const vectorn& input, double fmin, double fmax))&FltkScrollPanel::addPanel) // 1451
		.def("addPanel", (void (FltkScrollPanel::*)(const matrixn& signal))&FltkScrollPanel::addPanel) // 1451
		.def("setLabel", (void (FltkScrollPanel::*)(const char* label))&FltkScrollPanel::setLabel) // 1451
		.def("changeLabel", (void (FltkScrollPanel::*)(const char* prevLabel, const char* newLabel))&FltkScrollPanel::changeLabel) // 1451
		.def("selectedPanel", [](FltkScrollPanel& self)->std::string{ return std::string(self.selectedPanel());})
		.def("removeAllPanel", &FltkScrollPanel::removeAllPanel)      // 1450
		.def("sortPanels", &FltkScrollPanel::sortPanels)              // 1450
		.def("setLastPanelXOffset", &FltkScrollPanel::setLastPanelXOffset) // 1450
		.def("removePanel", &FltkScrollPanel::removePanel)            // 1450
		.def("changeXpos", &FltkScrollPanel::changeXpos)              // 1450
		.def("setCutState", &FltkScrollPanel::setCutState)            // 1450
		.def("cutState", (const boolN & (FltkScrollPanel::*)())&FltkScrollPanel::cutState,return_value_policy::reference ) // 1456
		.def("redraw", [](FltkScrollPanel& self){ self.redraw();})                      // 1450
		.def("currFrame", &FltkScrollPanel::currFrame)                // 1450
		; // end of class impl___pybindgen___FltkScrollPanel          // 1511
#endif

	class_<MotionPanel >(mainlib, "MotionPanel")
		.def( init<int,int,int,int>())
#ifndef NO_GUI
		.def("motionWin", &MotionPanel::motionWin, RETURN_REFERENCE)
		.def("currMotion", &MotionPanel::currMotion, RETURN_REFERENCE)
		.def("hasPairMotion", &MotionPanel::hasPairMotion)
		.def("currPairMotion", &MotionPanel::currPairMotion, RETURN_REFERENCE)
		.def("numMotion", &MotionPanel::numMotion)
		.def("motion", &MotionPanel::motion, RETURN_REFERENCE)
		.def ("scrollPanel", &MotionPanel::scrollPanel, RETURN_REFERENCE)
#endif
	;

	void (Motion::*initEmpty1)(const Motion&, int) =&Motion::InitEmpty;
	void (Motion::*init1)(const Motion&, int, int) =&Motion::Init;
	int (Motion::*numFrames1) () const=&Motion::numFrames;
	void (Motion::*setDiscontinuity1)(int, bool) =&Motion::setDiscontinuity;
	
	struct __pybindgen___Motion_wrapper
	{                                                             // 1382
		static void initFromFile(Motion& motion, const char* fn)
		{
			motion.Init(RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn));
		}

		static void initSkeletonFromFile(Motion& motion, const char* fn)
		{
			motion.InitSkeleton(RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn));
		}

		static void concatFromFile(Motion& motion, const char* fn)
		{
			RE::motion::concatFromFile(motion, fn);
		}
		static void scale(Motion& motion, m_real fScale)
		{
			if (&motion.skeleton().m_cPostureIP==&motion)
				motion.skeleton().Scale(fScale);
			else
				motion.skeleton().scale(fScale, motion);
		}
		static void calcInterFrameDifference(Motion& motion)
		{
			motion.CalcInterFrameDifference(0);
		}

		static void translate(Motion& motion, vector3 const& t)
		{
			MotionUtil::translate(motion, t);
		}

		static void smooth(Motion& motion, float kernelRoot, float kernelJoint)
		{
			Motion copy=motion;
			MotionUtil::smooth(motion, copy, kernelRoot, kernelJoint);
		}
		static double transitionCost(const Motion& motion, int from, int to,int windowsize )
		{
			return MotionUtil::transitionCost(motion, from, to, windowsize);
		}
		// 1383
	};                                                            // 1384
	struct __pybindgen___Ray_wrapper
	{                                                             // 1382
		inline static vectorn intersects_plane(Ray & r, const Plane& pl) 
		{
			vectorn v(2);
			std::pair<bool,m_real> p=r.intersects(pl);
			v(0)=(p.first)?1:0;
			v(1)=p.second;
			return v;
		}
		inline static vectorn intersects(Ray & r, const std::vector<Plane>& pl) 
		{
			vectorn v(2);
			std::pair<bool,m_real> p=r.intersects(pl);
			v(0)=(p.first)?1:0;
			v(1)=p.second;
			return v;
		}
		inline static vectorn intersects_sphere(Ray & r, const Sphere& pl) 
		{
			vectorn v(2);
			std::pair<bool,m_real> p=r.intersects(pl);
			v(0)=(p.first)?1:0;
			v(1)=p.second;
			return v;
		}
		// 1383
	};                                                            // 1384
	class_<Ray > (mainlib, "Ray")                             // 1393
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
		.def(init<>())                                                // 1431
		.def(init<const vector3 &,const vector3 &>())                 // 1431
		.def("origin", (vector3& (Ray::*)())&Ray::origin) // 1451
		.def("direction", (vector3& (Ray::*)())&Ray::direction) // 1451
		.def("getPoint", (vector3 (Ray::*)(m_real t))&Ray::getPoint) // 1451
		.def("scale", (void (Ray::*)(double s))&Ray::scale) // 1451
		.def("translate", (void (Ray::*)(vector3 const& t))&Ray::translate) // 1451
		.def("pickBarycentric", (int (Ray::*)(const OBJloader::Mesh& mesh, vector3 & baryCoeffs, vector3 & pickPos))&Ray::pickBarycentric) // 1451
		.def("pickBarycentric", (int (Ray::*)(const OBJloader::Mesh& mesh, const vector3N& vertexPositions, vector3 & baryCoeffs, vector3 & pickPos))&Ray::pickBarycentric) // 1451
		.def("intersects", &__pybindgen___Ray_wrapper::intersects_plane)
		.def("intersects", &__pybindgen___Ray_wrapper::intersects_sphere)
		; // end of class impl___pybindgen___Ray                      // 1511
	struct __pybindgen___Box2D_wrapper
	{                                                             // 1382
		inline static vector2& _property_get_min(Box2D const& a) { return (vector2 &) a.min; }
		inline static vector2& _property_get_max(Box2D const& a) { return (vector2 &) a.max; }
		// 1383
	};                                                            // 1384
	class_<intersectionTest::LineSegment>(mainlib, "LineSegment")
		.def(init<>())
		.def(init<const vector3& , const vector3& >())
		.def("minDist", &intersectionTest::LineSegment::minDist)
		.def("minDistTime", &intersectionTest::LineSegment::minDistTime)
		.def("pos", &intersectionTest::LineSegment::pos)
		;

	class_<Box2D > (mainlib, "Box2D")                         // 1393
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
		.def(init<>())                                                // 1431
		.def(init<vector2 const &,vector2 const &>())                 // 1431
		.def("distance", &Box2D::distance)                            // 1450
																	  //when necessary, check c++ header: .def("distance", (double (Box2D::*)(vector2 const& p))&Box2D::distance) // 1451
		.def("contains", &Box2D::contains)                            // 1450
																	  //when necessary, check c++ header: .def("contains", (bool (Box2D::*)(vector2 const& pt, double margin))&Box2D::contains) // 1451
		//.def("_property_get_min", (vector2 & (*)(Box2D const& a))&_property_get_min, return_value_policy::reference) // 1469
		//.def("_property_get_max", (vector2 & (*)(Box2D const& a))&_property_get_max, return_value_policy::reference) // 1469
		; // end of class impl___pybindgen___Box2D                    // 1511
	struct __pybindgen___Plane_wrapper
	{                                                             // 1382
		inline static vector3& _property_get_normal(Plane const& a) { return (vector3 &) a.normal; }
		inline static double _property_get_d(Plane const& a) { return a.d; }inline static void _property_set_d(Plane & a, double b){ a.d=b;}
		// 1383
	};                                                            // 1384
	class_<Plane > (mainlib, "Plane")                         // 1393
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
		.def(init<>())                                                // 1431
		.def(init<const vector3 &,m_real>())                          // 1431
		.def(init<const vector3 &,const vector3 &>())                 // 1431
		.def(init<const vector3 &,const vector3 &,const vector3 &>()) // 1431
		.def("distance", (m_real (Plane::*)(const vector3& point))&Plane::distance) // 1451
		.def("setPlane", (void (Plane::*)(const vector3& vPoint0, const vector3& vPoint1, const vector3& vPoint2))&Plane::setPlane) // 1451
		.def("setPlane", (void (Plane::*)(const vector3& vNormal, const vector3& vPoint))&Plane::setPlane) // 1451
		.def_readwrite("normal", &Plane::normal)
		.def_readwrite("d", &Plane::d)
		; // end of class impl___pybindgen___Plane                    // 1511
	struct __pybindgen___Sphere_wrapper
	{                                                             // 1382
		inline static vector3& _property_get_center(Sphere const& a) { return (vector3 &) a.center; }
		inline static double _property_get_radius(Sphere const& a) { return a.radius; }inline static void _property_set_radius(Sphere & a, double b){ a.radius=b;}
		// 1383
	};                                                            // 1384
	class_<Sphere > (mainlib, "Sphere")                       // 1393
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
		.def(init<vector3,m_real>())                                  // 1431
		.def_readwrite("center", &Sphere::center)
		.def_readwrite("radius", &Sphere::radius)
		; // end of class impl___pybindgen___Sphere                   // 1511
	class_<Motion>(mainlib, "Motion")
		.def(init<>())                                                // 1426
		.def(init<MotionLoader*>())
		.def(init<const Motion&, int, int>())
		.def(init<Motion const&>())
		.def(init<const MotionDOF &>())                               // 1426
		.def(init<const MotionDOF &,int,int>())                       // 1426
		.def("length",&Motion::length)	
		.def("copy", [](Motion const&v )->Motion *{ return new Motion(v);}, TAKE_OWNERSHIP )
		.def("print", [](MotionLoader&l){ l.printHierarchy();})
		.def("changeLength", &Motion::changeLength)
		.def("resize", &Motion::Resize)
		.def("empty", &Motion::empty)
		.def("assign", &Motion::assign)
		.def("setPose", &Motion::setPose)
		.def("setSkeleton", &Motion::setSkeleton) // mot.setSkeleton(12) mot.skeleton().
		.def("numFrames", numFrames1)  
		.def("skeleton", &Motion::skeleton, RETURN_REFERENCE) // MotionLoader????
		.def("numRotJoint", &Motion::numRotJoints)
		.def("numTransJoint", &Motion::numTransJoints)
		.def("numFrames", (int (Motion::*)() const)&Motion::numFrames)      // 1445
		.def("size", (int (Motion::*)())&Motion::size)                // 1445
		.def("changeCoord", (void (Motion::*)(int eCoord))&Motion::ChangeCoord) // 1445
		.def("range", [] (Motion& m,int start, int end)->MotionView{ return m.range(start, end);}, WRAP_PY::keep_alive<0,1>())
		.def("init", init1)
		.def("init", (void (Motion::*)(MotionLoader* pSource))&Motion::Init) // 1445
		.def("initEmpty", [](Motion&m , MotionLoader* pSource, int numFrames){ m.InitEmpty(pSource, numFrames);})
		.def("initEmpty", [](Motion&m , MotionLoader* pSource, int numFrames, float f){ m.InitEmpty(pSource, numFrames, f);})
		.def("initEmpty", (void (Motion::*)(const Motion& source, int numFrames))&Motion::InitEmpty) // 1445
		.def("initSkeleton", (void (Motion::*)(MotionLoader* pSource))&Motion::InitSkeleton) // 1445
		.def("isConstraint", (bool (Motion::*)(int fr, int eConstraint) const)&Motion::isConstraint) // 1445
		.def("setConstraint", (void (Motion::*)(int fr, int con, bool bSet))&Motion::setConstraint) // 1445
		.def("getConstraint", (bitvectorn (Motion::*)(int econ) const)&Motion::getConstraint)
		.def("setConstraint", (void (Motion::*)(int econ, bitvectorn const& bit) )&Motion::setConstraint)
		.def_property("identifier", &Motion::GetIdentifier, &Motion::SetIdentifier)
		.def("concat", &Motion::Concat, "pAdd"_a, "startFrame"_a=0, "endFrame"_a=INT_MAX, "bTypeCheck"_a=true)
		.def("totalTime", &Motion::totalTime)
		.def("frameRate", &Motion::frameRate)
		.def("setFrameTime", (void (Motion::*)(float ftime))&Motion::frameTime) // 1445
		.def("isDiscontinuous", &Motion::isDiscontinuous)
		.def("setDiscontinuity", setDiscontinuity1)
		.def("getDiscontinuity", &Motion::getDiscontinuity)
		.def("row", &Motion::row)
		.def("setDiscontinuity", (void (Motion::*)(boolN const& bit))&Motion::setDiscontinuity) // 1445
		.def("pose", &Motion::pose, RETURN_REFERENCE)
		.def("calcInterFrameDifference",&Motion::CalcInterFrameDifference)
		.def("reconstructFromInterFrameDifference", &Motion::ReconstructDataByDifference)
		.def("exportMot", &Motion::exportMOT)
		.def("exportBinary", (void (Motion::*)(const char* filename))&Motion::exportBinary) // 1445
		.def("importBinary", (void (Motion::*)(const char* filename))&Motion::importBinary) // 1445
		.def("samplePose",&Motion::samplePose)
		.def("initSkeletonFromFile", (void (*)(Motion& motion, const char* fn))&__pybindgen___Motion_wrapper::
initSkeletonFromFile) // 1458
		.def("initFromFile", (void (*)(Motion& motion, const char* fn))&__pybindgen___Motion_wrapper::initFromFile) // 1458
		.def("concatFromFile", (void (*)(Motion& motion, const char* fn))&__pybindgen___Motion_wrapper::concatFromFile) // 1458
		.def("scale", (void (*)(Motion& motion, m_real fScale))&__pybindgen___Motion_wrapper::scale) // 1458
		.def("calcInterFrameDifference", (void (*)(Motion& motion))&__pybindgen___Motion_wrapper::calcInterFrameDifference) // 1458
		.def("translate", (void (*)(Motion& motion, vector3 const& t))&__pybindgen___Motion_wrapper::translate) // 1458
		.def("smooth", (void (*)(Motion& motion, float kernelRoot, float kernelJoint))&__pybindgen___Motion_wrapper::smooth) // 1458
		.def("transitionCost", (double (*)(const Motion& motion, int from, int to, int windowsize))&__pybindgen___Motion_wrapper::transitionCost) // 1458
		.def("mirrorMotion", (void (*)(Motion& out, const Motion& in, intvectorn const& LrootIndices, intvectorn const& RrootIndices))&MotionUtil::mirrorMotion) // 1458
		; // end of class impl___pybindgen___Motion                   // 1505
	class_<MotionView ,Motion> (mainlib, "MotionView")                     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		; // end of class impl___pybindgen___MotionView               // 1505

		mainlib.attr("DOFtype") = WRAP_PY::module::import("enum").attr("IntEnum")
        ("DOFtype", WRAP_PY::dict(
						"ROTATE"_a=(int)MotionDOFinfo::ROTATE, 
						"SLIDE"_a=(int)MotionDOFinfo::SLIDE,
						"QUATERNION_W"_a=(int)MotionDOFinfo::QUATERNION_W, 
						"QUATERNION_X"_a=(int)MotionDOFinfo::QUATERNION_X, 
						"QUATERNION_Y"_a=(int)MotionDOFinfo::QUATERNION_Y, 
						"QUATERNION_Z"_a=(int)MotionDOFinfo::QUATERNION_Z
						));
    class_<MotionDOFinfo > (mainlib, "MotionDOFinfo")               // 1388
		.def(init<>())                                                // 1425
		.def("skeleton", (MotionLoader & (MotionDOFinfo::*)())&MotionDOFinfo::skeleton,return_value_policy::reference ) // 1452
		.def("numDOF", (int (MotionDOFinfo::*)() const)&MotionDOFinfo::numDOF) // 1447
		.def("numActualDOF", (int (MotionDOFinfo::*)() const)&MotionDOFinfo::numActualDOF) // 1447
		.def("numBone", (int (MotionDOFinfo::*)() const)&MotionDOFinfo::numBone) // 1447
		.def("numDOF", (int (MotionDOFinfo::*)(int ibone) const)&MotionDOFinfo::numDOF) // 1447
		.def("DOFtype", (int (MotionDOFinfo::*)(int ibone, int offset) const)&MotionDOFinfo::DOFtype) // 1447
		.def("DOFindex", (int (MotionDOFinfo::*)(int ibone, int offset) const)&MotionDOFinfo::DOFindex) // 1447
		.def("sphericalDOFindex", (int (MotionDOFinfo::*)(int isphericalJoint) const)&MotionDOFinfo::sphericalDOFindex) // 1447
		.def("numSphericalJoint", (int (MotionDOFinfo::*)() const)&MotionDOFinfo::numSphericalJoint) // 1447
		.def("frameRate", (double (MotionDOFinfo::*)() const)&MotionDOFinfo::frameRate) // 1447
		.def("setFrameRate", (void (MotionDOFinfo::*)(double f))&MotionDOFinfo::setFrameRate) // 1447
		.def("getDOF", (void (MotionDOFinfo::*)(Posture const& p, vectorn& dof) const)&MotionDOFinfo::getDOF) // 1447
		.def("setDOF", (void (MotionDOFinfo::*)(vectorn const& dof, Posture& p) const)&MotionDOFinfo::setDOF) // 1447
		.def("setDOF", (Posture const & (MotionDOFinfo::*)(vectorn const& dof) const)&MotionDOFinfo::setDOF,return_value_policy::reference ) // 1452
		.def("hasTranslation", (bool (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::hasTranslation) // 1447
		.def("hasQuaternion", (bool (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::hasQuaternion) // 1447
		.def("hasAngles", (bool (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::hasAngles) // 1447
		.def("startT", (int (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::startT) // 1447
		.def("startR", (int (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::startR) // 1447
		.def("endR", (int (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::endR) // 1447
		.def("startDQ", (int (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::startDQ) // 1447
		.def("endDQ", (int (MotionDOFinfo::*)(int iBone) const)&MotionDOFinfo::endDQ) // 1447
		.def("DQtoBone", (int (MotionDOFinfo::*)(int DQindex) const)&MotionDOFinfo::DQtoBone) // 1447
		.def("DOFtoBone", (int (MotionDOFinfo::*)(int DOFindex) const)&MotionDOFinfo::DOFtoBone) // 1447
		.def("DOFtoDQ", (int (MotionDOFinfo::*)(int DOFindex) const)&MotionDOFinfo::DOFtoDQ) // 1447
		.def("DQtoDOF", (int (MotionDOFinfo::*)(int DOFindex) const)&MotionDOFinfo::DQtoDOF) // 1447
		.def("DOFtoDQ", (void (MotionDOFinfo::*)(vectorn const& dtheta, vectorn & dq) )&MotionDOFinfo::DOFtoDQ) // 1447
		.def("DQtoDOF", (void (MotionDOFinfo::*)(vectorn const& dq, vectorn & dtheta) )&MotionDOFinfo::DQtoDOF) // 1447
		.def("blend", (void (MotionDOFinfo::*)(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const)&MotionDOFinfo::blend) // 1447
		.def("blendDelta", (void (MotionDOFinfo::*)(vectorn & out, vectorn const& p1, vectorn const& p2, m_real t) const)&MotionDOFinfo::blendDelta) // 1447
		.def("blendBone", (void (MotionDOFinfo::*)(int ibone, vectorn & c, vectorn const& a, vectorn const& b, m_real t) const)&MotionDOFinfo::blendBone) // 1447
		; // end of class impl_LunaTraits<MotionDOFinfo >             // 1583
    class_<MotionDOF , matrixn> (mainlib, "MotionDOF")                       // 1388
		.def(init<const MotionDOFinfo &>())                           // 1425
		.def(init<const MotionDOF &>())                               // 1425
		.def(init<const MotionDOFinfo &,const Motion &>())            // 1425
		.def("slice", [](MotionDOF const& self, int scol, int ecol)->MotionDOFview{
				if (scol<0 )
				scol=self.rows()+scol;
				if (ecol<=0 )
				ecol=self.rows()+ecol;
				return self.range(scol, ecol);
				}, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).

		.def("copy", [](MotionDOF const&v )->MotionDOF *{ return new MotionDOF(v);}, TAKE_OWNERSHIP )
		.def("assign", (void (MotionDOF::*)(const MotionDOF& other))&MotionDOF::operator=) // 1447
		.def("copyFrom", (void (MotionDOF::*)(const MotionDOF& other))&MotionDOF::operator=) // 1447
		.def("assign", (void (MotionDOF::*)(const MotionDOF& other))&MotionDOF::operator=) // 1447
		.def("matView", (matrixnView (MotionDOF::*)())&MotionDOF::_matView, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
 // 1447
		.def("numFrames", (int (MotionDOF::*)())&MotionDOF::numFrames) // 1447
		.def("numDOF", (int (MotionDOF::*)())&MotionDOF::numDOF)      // 1447
		.def("resize", (void (MotionDOF::*)(int numFrames))&MotionDOF::resize) // 1447
		.def("changeLength", (void (MotionDOF::*)(int length))&MotionDOF::changeLength) // 1447
		.def("setDOFinfo", (void (MotionDOF::*)(MotionDOFinfo const& info))&MotionDOF::setDOFinfo) // 1447
		.def("get", (void (MotionDOF::*)(Motion& tgtMotion))&MotionDOF::get) // 1447
		.def("set", (void (MotionDOF::*)(const Motion& srcMotion))&MotionDOF::set) // 1447
		.def("set", (void (MotionDOF::*)(const Motion& srcMotion, intvectorn const& shoulder_tree_indices, intvectorn const& elbow_tree_indices, intvectorn const& wrist_tree_indices))&MotionDOF::set) // 1447
		.def("range", (MotionDOFview (MotionDOF::*)(int start, int end))&MotionDOF::range, WRAP_PY::keep_alive<0,1>()) // 1447
		.def("range_c", (MotionDOFview (MotionDOF::*)(int first, int last))&MotionDOF::range_c, WRAP_PY::keep_alive<0,1>()) // 1447
		.def("samplePose", (void (MotionDOF::*)(m_real criticalTime, vectorn& out))&MotionDOF::samplePose) // 1447
		.def("stitch", (void (MotionDOF::*)(MotionDOF const& motA, MotionDOF const& motB))&MotionDOF::stitch) // 1447
		.def("align", (void (MotionDOF::*)(MotionDOF const& motA, MotionDOF const& motB))&MotionDOF::align) // 1447
		.def("alignSimple", (void (MotionDOF::*)(MotionDOF const& motA, MotionDOF const& motB))&MotionDOF::alignSimple) // 1447
		.def("stitchDeltaRep", (void (MotionDOF::*)(MotionDOF const& motA, MotionDOF const& motB))&MotionDOF::stitchDeltaRep) // 1447
		.def("__call__", (vectornView (MotionDOF::*)(int i))&MotionDOF::row) // 1447
		.def("convertToDeltaRep", (vector3 (MotionDOF::*)())&MotionDOF::convertToDeltaRep) // 1447
		.def("generateID", (void (MotionDOF::*)(vector3 const& start_transf, InterframeDifference& out))&MotionDOF::generateID) // 1447
		.def("reconstructData", (void (MotionDOF::*)(vector3 const & startTransf))&MotionDOF::reconstructData) // 1447
		.def("reconstructData", (void (MotionDOF::*)(vector3 const& startTransf, matrixn& out) const)&MotionDOF::reconstructData) // 1447
		.def("reconstructData", (void (MotionDOF::*)(transf const& startTransf, matrixn& out) const)&MotionDOF::reconstructData) // 1447
		.def("reconstructOneFrame", (void (MotionDOF::*)(vector3 const& startTransf, vectorn const& deltaPose, vectorn & outpose) const)&MotionDOF::reconstructOneFrame) // 1447
		.def("dv_x", (vectornView (MotionDOF::*)())&MotionDOF::dv_x)  // 1447
		.def("dv_z", (vectornView (MotionDOF::*)())&MotionDOF::dv_z)  // 1447
		.def("dq_y", (vectornView (MotionDOF::*)())&MotionDOF::dq_y)  // 1447
		.def("offset_y", (vectornView (MotionDOF::*)())&MotionDOF::offset_y) // 1447
		.def("length", (int (MotionDOF::*)())&MotionDOF::length)      // 1447
		.def("transform", (void (MotionDOF::*)(transf const& t))&MotionDOF::transform) // 1447
		.def("scale", (void (MotionDOF::*)(double t))&MotionDOF::scale) // 1447
		.def("calcForwardDerivative", (void (MotionDOF::*)(int i, vectorn & dpose, double frameRate))&MotionDOF::calcForwardDerivative) // 1447
		.def("calcDerivative", (matrixn (*)(MotionDOF const& dof, double frameRate))&MotionDOF_calcDerivative)																																		//
		.def("rootTransformation", (transf (*)(vectorn const& pose))&MotionDOF::rootTransformation) // 1460
		.def("setRootTransformation", (void (*)(vectorn & pose, transf const& t))&MotionDOF::setRootTransformation) // 1460
		.def_readwrite("dofInfo", &MotionDOF::mInfo)
		;

    class_<MotionDOFview , MotionDOF> (mainlib, "MotionDOFview")               // 1388
		; // end of class impl_LunaTraits<MotionDOFview >             // 1583
	{
		struct __pybindgen___BoneForwardKinematics_wrapper
		{                                                             // 1382
			static transf& localFrame(BoneForwardKinematics& fk, int i){ return fk._local(i);}
			static transf& localFrame(BoneForwardKinematics& fk, Bone& bone){ return fk._local(bone);}
			static transf& globalFrame(BoneForwardKinematics& fk, int i){ return fk._global(i);}
			static transf& globalFrame(BoneForwardKinematics& fk, Bone& bone){return fk._global(bone);}
			// 1383
		};                                                            // 1384
		class_<BoneForwardKinematics > (mainlib, "BoneForwardKinematics") // 1389
			.def(init<MotionLoader *>())                                  // 1426
			.def("init", (void (BoneForwardKinematics::*)())&BoneForwardKinematics::init) // 1445
			.def("numBone", (int (BoneForwardKinematics::*)())&BoneForwardKinematics::numBone) // 1445
			.def("forwardKinematics", (void (BoneForwardKinematics::*)())&BoneForwardKinematics::forwardKinematics) // 1445
			.def("inverseKinematics", (void (BoneForwardKinematics::*)())&BoneForwardKinematics::inverseKinematics) // 1445
			.def("inverseKinematicsExact", (void (BoneForwardKinematics::*)())&BoneForwardKinematics::inverseKinematicsExact) // 1445
			.def("updateBoneLength", (void (BoneForwardKinematics::*)(MotionLoader const& loader))&BoneForwardKinematics::updateBoneLength) // 1445
			.def("assign", (void (BoneForwardKinematics::*)(BoneForwardKinematics const& other))&BoneForwardKinematics::operator=) // 1445
			.def("setPose", (void (BoneForwardKinematics::*)(const Posture& pose))&BoneForwardKinematics::setPose) // 1445
			.def("setPoseDOF", (void (BoneForwardKinematics::*)(const vectorn& poseDOF))&BoneForwardKinematics::setPoseDOF) // 1445
			.def("setPoseDOFusingCompatibleDOFinfo", (void (BoneForwardKinematics::*)(MotionDOFinfo const& dofInfo, const vectorn& poseDOF))&BoneForwardKinematics::setPoseDOFusingCompatibleDOFinfo) // 1445
			.def("setSphericalQ", (void (BoneForwardKinematics::*)(const vectorn& q))&BoneForwardKinematics::setSphericalQ) // 1445
			.def("setChain", (void (BoneForwardKinematics::*)(const Posture& pose, const Bone& bone))&BoneForwardKinematics::setChain) // 1445
			.def("setChain", (void (BoneForwardKinematics::*)(const Bone& bone))&BoneForwardKinematics::setChain) // 1445
			.def("getPose", &BoneForwardKinematics::getPose) 
			.def("getPoseDOF", &BoneForwardKinematics::getPoseDOF) 
			.def("getPoseFromGlobal", (void (BoneForwardKinematics::*)(Posture& pose))&BoneForwardKinematics::getPoseFromGlobal) // 1445
			.def("getPoseDOFfromGlobal", (void (BoneForwardKinematics::*)(vectorn& poseDOF))&BoneForwardKinematics::getPoseDOFfromGlobal) // 1445
			.def("getPoseFromLocal", (void (BoneForwardKinematics::*)(Posture& pose))&BoneForwardKinematics::getPoseFromLocal) // 1445
			.def("getPoseDOFfromLocal", (void (BoneForwardKinematics::*)(vectorn& poseDOF))&BoneForwardKinematics::getPoseDOFfromLocal) // 1445
			.def("getSkeleton", (MotionLoader const & (BoneForwardKinematics::*)())&BoneForwardKinematics::getSkeleton,return_value_policy::reference ) // 1450
			.def("localFrame", (transf & (*)(BoneForwardKinematics& fk, int i))&__pybindgen___BoneForwardKinematics_wrapper::localFrame, return_value_policy::reference) // 1463
			.def("localFrame", (transf & (*)(BoneForwardKinematics& fk, Bone& bone))&__pybindgen___BoneForwardKinematics_wrapper::localFrame, return_value_policy::reference) // 1463
			.def("globalFrame", (transf & (*)(BoneForwardKinematics& fk, int i))&__pybindgen___BoneForwardKinematics_wrapper::globalFrame, return_value_policy::reference) // 1463
			.def("globalFrame", (transf & (*)(BoneForwardKinematics& fk, Bone& bone))&__pybindgen___BoneForwardKinematics_wrapper::globalFrame, return_value_policy::reference) // 1463
			; // end of class impl___pybindgen___BoneForwardKinematics    // 1505
	}
	{
		class_<ScaledBoneKinematics > (mainlib, "ScaledBoneKinematics") // 1393
																		// : number denotes the line number of luna_gen.lua that generated the sentence // 1397
			.def(init<MotionLoader *>())                                  // 1431
			.def("init", &ScaledBoneKinematics::init)                     // 1450
			.def("numBone", &ScaledBoneKinematics::numBone)               // 1450
			.def("localRot", (quater & (ScaledBoneKinematics::*)(int i))&ScaledBoneKinematics::localRot,return_value_policy::reference ) // 1456
			.def("localScale", (const matrix4 & (ScaledBoneKinematics::*)(int i))&ScaledBoneKinematics::localScale,return_value_policy::reference ) // 1456
			.def("globalRot", (quater & (ScaledBoneKinematics::*)(int i))&ScaledBoneKinematics::globalRot,return_value_policy::reference ) // 1456
			.def("forwardKinematics", &ScaledBoneKinematics::forwardKinematics) // 1450
			.def("updateBoneLength", &ScaledBoneKinematics::updateBoneLength) // 1450
			.def("setScale", &ScaledBoneKinematics::setScale)             // 1450
			.def("setLengthScale", &ScaledBoneKinematics::setLengthScale) // 1450
			.def("setPose", &ScaledBoneKinematics::setPose)               // 1450
			.def("setPoseDOF", &ScaledBoneKinematics::setPoseDOF)         // 1450
			.def("setPoseDOFusingCompatibleDOFinfo", &ScaledBoneKinematics::setPoseDOFusingCompatibleDOFinfo) // 1450
			.def("getSkeleton", (MotionLoader const & (ScaledBoneKinematics::*)())&ScaledBoneKinematics::getSkeleton,return_value_policy::reference ) // 1456
			.def("getPose", [](ScaledBoneKinematics& self)->Posture { return self.getPose();}) 
			; // end of class impl___pybindgen__util_ScaledBoneKinematics // 1511
	}
    class_<MotionDOFcontainer > (mainlib, "MotionDOFcontainer")    // 1388
		.def(init<MotionDOFinfo const &,const std::string&>())              // 1425
		.def(init<MotionDOFinfo const &>())                           // 1425
		.def(init<MotionDOF const &>())                               // 1425
		.def("copy", [](MotionDOFcontainer const&v )->MotionDOFcontainer *{ return new MotionDOFcontainer(v);}, TAKE_OWNERSHIP )
		.def("loadMotion", (void (MotionDOFcontainer::*)(const char* fn))&MotionDOFcontainer::loadMotion) // 1447
		.def("resize", (void (MotionDOFcontainer::*)(int nframes))&MotionDOFcontainer::resize) // 1447
		.def("concat", (void (MotionDOFcontainer::*)(MotionDOF const& mot))&MotionDOFcontainer::concat) // 1447
		.def("row", (vectornView (MotionDOFcontainer::*)(int i))&MotionDOFcontainer::row) // 1447
		.def("numFrames", (int (MotionDOFcontainer::*)())&MotionDOFcontainer::numFrames) // 1447
		.def("isConstraint", (bool (MotionDOFcontainer::*)(int iframe, int con))&MotionDOFcontainer::isConstraint) // 1447
		.def("setConstraint", (void (MotionDOFcontainer::*)(int iframe, int con))&MotionDOFcontainer::setConstraint) // 1447
		.def("setConstraint", (void (MotionDOFcontainer::*)(int iframe, int con, bool bSet))&MotionDOFcontainer::setConstraint) // 1447
		.def("isContinuous", (bool (MotionDOFcontainer::*)(int startTime))&MotionDOFcontainer::isContinuous) // 1447
		.def("isValid", (bool (MotionDOFcontainer::*)(int startTime, int endTime) const)&MotionDOFcontainer::isValid) // 1447
		.def("isValid", (bool (MotionDOFcontainer::*)(int startTime) const)&MotionDOFcontainer::isValid) // 1447
		.def_readwrite("mot", &MotionDOFcontainer::mot)
		.def_readwrite("discontinuity", &MotionDOFcontainer::discontinuity)
		.def_readwrite("conL", &MotionDOFcontainer::conL)
		.def_readwrite("conR",  &MotionDOFcontainer::conR)
		;

		struct pybind_effector_wrap
		{
					static void init(MotionUtil::Effector& e, Bone* bone, vector3 const& l)
					{
						e.bone=bone;
						e.localpos=l;
					}
					static void initCOM(MotionUtil::Effector& e, vector3 const& l)
					{
						e.bone=NULL;
						e.localpos=l;
					}
		};

	class_<MotionUtil ::Effector > (mainlib, "Effector")            // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("init", (void (*)(MotionUtil::Effector& e, Bone* bone, vector3 const& l))&pybind_effector_wrap::init) // 1458
		.def("initCOM", (void (*)(MotionUtil::Effector& e, vector3 const& l))&pybind_effector_wrap::initCOM) // 1458
		.def_readwrite("bone", &MotionUtil::Effector::bone)
		.def_readwrite("localpos",&MotionUtil::Effector::localpos)
		; // end of class impl___pybindgen__MotionUtil_Effector       // 1505
	struct __pybindgen__MotionUtil_RelativeConstraint_wrapper
	{                                                             // 1382
		static void init(MotionUtil::RelativeConstraint& e, Bone* bone1, Bone* bone2, vector3 const& l)
		{
			e.bone1=bone1;
			e.bone2=bone2;
			e.localpos1=l;
		}
		// 1383
	};                                                            // 1384
	class_<MotionUtil ::RelativeConstraint > (mainlib, "RelativeConstraint") // 1389
																	   // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("init", (void (*)(MotionUtil::RelativeConstraint& e, Bone* bone1, Bone* bone2, vector3 const& l))&__pybindgen__MotionUtil_RelativeConstraint_wrapper::init) // 1458
		.def_readwrite("bone1", &MotionUtil ::RelativeConstraint::bone1)
		.def_readwrite("bone2", &MotionUtil ::RelativeConstraint ::bone2)
		.def_readwrite("localpos1", &MotionUtil ::RelativeConstraint ::localpos1)
		; // end of class impl___pybindgen__MotionUtil_RelativeConstraint // 1505
	class_<std ::vector <MotionUtil ::Effector > > (mainlib, "Effectors") // 1389
																	// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("resize",[](std::vector< MotionUtil ::Effector>& e, int i){ e.resize(i);}) // 1445
		.def("at", [](std::vector< MotionUtil ::Effector>& e, int i)->MotionUtil::Effector&{ return e[i];},return_value_policy::reference ) // 1450
		.def("__call__", [](std::vector< MotionUtil ::Effector>& e, int i)->MotionUtil::Effector&{ return e[i];},return_value_policy::reference ) // 1450
		.def("size", [](std::vector< MotionUtil ::Effector>& e)->int{ return e.size();})
		; // end of class impl___pybindgen__MotionUtil_Effectors      // 1505
	class_<std ::vector <MotionUtil ::RelativeConstraint > > (mainlib, "Constraints") // 1389
																				// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("resize",[](std::vector< MotionUtil ::RelativeConstraint>& e, int i){ e.resize(i);}) // 1445
		.def("at", [](std::vector< MotionUtil ::RelativeConstraint>& e, int i)->MotionUtil::RelativeConstraint&{ return e[i];},return_value_policy::reference ) // 1450
		.def("__call__", [](std::vector< MotionUtil ::RelativeConstraint>& e, int i)->MotionUtil::RelativeConstraint&{ return e[i];},return_value_policy::reference ) // 1450
		.def("size", [](std::vector< MotionUtil ::RelativeConstraint>& e)->int{ return e.size();})
		; // end of class impl___pybindgen__MotionUtil_Constraints    // 1505
	{
		/// IKsolvers
		class_<MotionUtil ::LimbIKsolver > (mainlib, "LimbIKsolver")    // 1389
																 // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,intvectorn const &,vectorn const &>()) // 1426
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,vectorn const &>()) // 1426
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,const vector3N &>()) // 1426
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,Bone const &,Bone const &>()) // 1426
			.def("IKsolve3", (void (MotionUtil ::LimbIKsolver::*)(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance))&MotionUtil ::LimbIKsolver::IKsolve3) // 1445
			.def("IKsolve", (void (MotionUtil ::LimbIKsolver::*)(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con))&MotionUtil ::LimbIKsolver::IKsolve) // 1445
			.def("IKsolve2", (void (MotionUtil ::LimbIKsolver::*)(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con))&MotionUtil ::LimbIKsolver::IKsolve2) // 1445
			.def("setOption", (void (MotionUtil ::LimbIKsolver::*)(const char* id, double val))&MotionUtil ::LimbIKsolver::setOption) // 1445
			; // end of class impl___pybindgen___LimbIKsolver             // 1505
		class_<MotionUtil ::LimbIKsolver2 > (mainlib, "LimbIKsolver2")  // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,vectorn const &>()) // 1426
			.def("IKsolve3", (void (MotionUtil ::LimbIKsolver2::*)(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance))&MotionUtil ::LimbIKsolver2::IKsolve3) // 1445
			.def("IKsolve", (void (MotionUtil ::LimbIKsolver2::*)(vectorn & temp, quater const& currRotY, transf const& newRootTF, vector3N const& con))&MotionUtil ::LimbIKsolver2::IKsolve) // 1445
			.def("IKsolve2", (void (MotionUtil ::LimbIKsolver2::*)(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con))&MotionUtil ::LimbIKsolver2::IKsolve2) // 1445
			.def("setOption", (void (MotionUtil ::LimbIKsolver2::*)(int option))&MotionUtil ::LimbIKsolver2::setOption) // 1445
			.def("setOption", (void (MotionUtil ::LimbIKsolver2::*)(const char* id, double val))&MotionUtil ::LimbIKsolver2::setOption) // 1445
			.def("setValue", (void (MotionUtil ::LimbIKsolver2::*)(double ValL, double ValM, double ValN,int IterNum))&MotionUtil ::LimbIKsolver2::setValue) // 1445
			; // end of class impl___pybindgen___LimbIKsolver2            // 1505
		class_<MotionUtil ::HandIKsolver > (mainlib, "HandIKsolver")    // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<MotionDOFinfo const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,intvectorn const &,vectorn const &>()) // 1426
			.def("IKsolve", (void (MotionUtil ::HandIKsolver::*)(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance))&MotionUtil ::HandIKsolver::IKsolve) // 1445
			.def("IKsolve", (void (MotionUtil ::HandIKsolver::*)(vectorn & poseInout, transf const& newRootTF, vector3N const& conpos, quaterN const& conori, vectorn const& importance, vectorn const& importance_wrist))&MotionUtil ::HandIKsolver::IKsolve) // 1445
			.def("setOption", (void (MotionUtil ::HandIKsolver::*)(const char* id, double val))&MotionUtil ::HandIKsolver::setOption) // 1445
			; // end of class impl___pybindgen___HandIKsolver             // 1505
		class_<MotionUtil ::COM_IKsolver > (mainlib, "COM_IKsolver")    // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<VRMLloader const &,std ::vector <MotionUtil ::Effector > &,intvectorn const &,vectorn const &>()) // 1426
			.def("IKsolve", (void (MotionUtil ::COM_IKsolver::*)(vectorn& origRootTF_, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& _con, vectorn const& _importance, vector3 const& _desiredCOM))&MotionUtil ::COM_IKsolver::IKsolve) // 1445
			.def("IKsolve2", (void (MotionUtil ::COM_IKsolver::*)(vectorn & temp, quater const& currRotY, transf const& newRootTF, quaterN const& delta_foot, vector3N const& con))&MotionUtil ::COM_IKsolver::IKsolve2) // 1445
			; // end of class impl___pybindgen___COM_IKsolver             // 1505
		class_<MotionUtil ::FullbodyIK > (mainlib, "FullbodyIK")        // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def("IKsolve", (void (MotionUtil ::FullbodyIK::*)(Posture& , vector3N const& ))&MotionUtil ::FullbodyIK::IKsolve) // 1445
			.def("IKsolve", (void (MotionUtil ::FullbodyIK::*)(Posture const&, Posture&, vector3N const& ))&MotionUtil ::FullbodyIK::IKsolve) // 1445
			.def("IKsolve", (void (MotionUtil ::FullbodyIK::*)(Posture const& input_pose, vector3N const& cPositions, intvectorn & rot_joint_index, quaterN& delta_rot))&MotionUtil ::FullbodyIK::IKsolve) // 1445
			; // end of class impl___pybindgen__MotionUtil_FullbodyIK     // 1505
		class_<MotionUtil ::FullbodyIK_MotionDOF > (mainlib, "FullbodyIK_MotionDOF") // 1389
																			   // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def("setParam", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(const char* type, double value))&MotionUtil ::FullbodyIK_MotionDOF::setParam) // 1445
			.def("setParam", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(const char* type, double value, double value2))&MotionUtil ::FullbodyIK_MotionDOF::setParam) // 1445
			.def("setParam", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(const char* type, vectorn const& ))&MotionUtil ::FullbodyIK_MotionDOF::setParam) // 1445
			.def("IKsolve", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(vectorn const& , vectorn& , vector3N const& ))&MotionUtil ::FullbodyIK_MotionDOF::IKsolve) // 1445
			.def("IKsolve", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(vectorn& , vector3N const& ))&MotionUtil ::FullbodyIK_MotionDOF::IKsolve) // 1445
			.def("_updateBoneLength", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(MotionLoader const& loader))&MotionUtil ::FullbodyIK_MotionDOF::_updateBoneLength) // 1445
			.def("_changeNumEffectors", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int n))&MotionUtil ::FullbodyIK_MotionDOF::_changeNumEffectors) // 1445
			.def("_numConstraints", (int (MotionUtil ::FullbodyIK_MotionDOF::*)())&MotionUtil ::FullbodyIK_MotionDOF::_numConstraints) // 1445
			.def("_changeNumConstraints", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int n))&MotionUtil ::FullbodyIK_MotionDOF::_changeNumConstraints) // 1445
			.def("_setEffector", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, vector3 const& lpos))&MotionUtil ::FullbodyIK_MotionDOF::_setEffector) // 1445
			.def("_setRelativeConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeConstraint) // 1445
			.def("_setRelativeConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeConstraint) // 1445
			.def("_setRelativeConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& delta, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeConstraint) // 1445
			.def("_setRelativeDistanceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, double thr, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeDistanceConstraint) // 1445
			.def("_setRelativeDistanceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& delta, double targetDist, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeDistanceConstraint) // 1445
			.def("_setPlaneDistanceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth))&MotionUtil ::FullbodyIK_MotionDOF::_setPlaneDistanceConstraint) // 1445
			.def("_setDistanceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, vector3 const& lpos, vector3 const& gpos, float targetDist))&MotionUtil ::FullbodyIK_MotionDOF::_setDistanceConstraint) // 1445
			.def("_setRelativeHalfSpaceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone1, vector3 const& lpos1, Bone* bone2, vector3 const& lpos2, vector3 const& global_normal, float idepth, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setRelativeHalfSpaceConstraint) // 1445
			.def("_setHalfSpaceConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, vector3 const& lpos, vector3 const& global_normal, float idepth))&MotionUtil ::FullbodyIK_MotionDOF::_setHalfSpaceConstraint) // 1445
			.def("_setSkinningConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, intvectorn const& treeIndices, vector3N const& localpos, vectorn  const&weights, vector3 const& desired_pos))&MotionUtil ::FullbodyIK_MotionDOF::_setSkinningConstraint) // 1445
			.def("_setFastSkinningConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, int numMarkers))&MotionUtil ::FullbodyIK_MotionDOF::_setFastSkinningConstraint) // 1445
			.def("_setFastSkinningConstraintParam", (void (MotionUtil ::FullbodyIK_MotionDOF::*)(int imarker, intvectorn const& treeIndices, vector3N const& localpos, vectorn  const&weights, vector3 const& desired_pos))&MotionUtil ::FullbodyIK_MotionDOF::_setFastSkinningConstraintParam) // 1445
			.def("_setCOMConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vector3 const& com))&MotionUtil ::FullbodyIK_MotionDOF::_setCOMConstraint) // 1445
			.def("_setCOMConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vector3 const& com, double wx, double wy, double wz))&MotionUtil ::FullbodyIK_MotionDOF::_setCOMConstraint) // 1445
			.def("_setPositionConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, vector3 const&lpos, vector3 const& desired_pos, double wx, double wy, double wz))&MotionUtil ::FullbodyIK_MotionDOF::_setPositionConstraint) // 1445
			.def("_setOrientationConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, quater const& desired_ori))&MotionUtil ::FullbodyIK_MotionDOF::_setOrientationConstraint) // 1445
			.def("_setOrientationConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, Bone* bone, quater const& desired_ori, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setOrientationConstraint) // 1445
			.def("_setMomentumConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vector3 const& ang, vector3 const& lin))&MotionUtil ::FullbodyIK_MotionDOF::_setMomentumConstraint) // 1445
			.def("_setMomentumConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vector3 const& ang, vector3 const& lin, double w))&MotionUtil ::FullbodyIK_MotionDOF::_setMomentumConstraint) // 1445
			.def("_setPoseConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vectorn const& pose, double weight, int startBoneIndex, int endBoneIndex))&MotionUtil ::FullbodyIK_MotionDOF::_setPoseConstraint) // 1445
			.def("_setPoseConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vectorn const& pose, double weight, int startBoneIndex))&MotionUtil ::FullbodyIK_MotionDOF::_setPoseConstraint) // 1445
			.def("_setPoseConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, vectorn const& pose, double weight))&MotionUtil ::FullbodyIK_MotionDOF::_setPoseConstraint) // 1445
			.def("_setEffectorWeight", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, double w))&MotionUtil ::FullbodyIK_MotionDOF::_setEffectorWeight) // 1445
			.def("_setConstraintWeight", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, double w))&MotionUtil ::FullbodyIK_MotionDOF::_setConstraintWeight) // 1445
			.def("_setEffectorYConstraint", (bool (MotionUtil ::FullbodyIK_MotionDOF::*)(int i, double weight, const vectorn& ew))&MotionUtil ::FullbodyIK_MotionDOF::_setEffectorYConstraint) // 1445
			.def("_effectorUpdated", &MotionUtil ::FullbodyIK_MotionDOF::_effectorUpdated) // 1445
			;
	}
	struct __pybindgen__MotionUtil_Node_wrapper
	{                                                             // 1382
		inline static transf& _property_get__global(IK_sdls ::Node const& a) { return (transf &) a._global; }
		// 1383
	};                                                            // 1384
	class_<IK_sdls ::Node > (mainlib, "Node")                       // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def("globalFrame", (const transf & (IK_sdls ::Node::*)())&IK_sdls ::Node::globalFrame,return_value_policy::reference ) // 1450
		.def("SetTheta", (void (IK_sdls ::Node::*)(double newTheta))&IK_sdls ::Node::SetTheta) // 1445
		.def("GetTheta", (double (IK_sdls ::Node::*)())&IK_sdls ::Node::GetTheta) // 1445
		.def("IsEffector", (bool (IK_sdls ::Node::*)())&IK_sdls ::Node::IsEffector) // 1445
		.def("IsJoint", (bool (IK_sdls ::Node::*)())&IK_sdls ::Node::IsJoint) // 1445
		.def("IsHingeJoint", (bool (IK_sdls ::Node::*)())&IK_sdls ::Node::IsHingeJoint) // 1445
		.def("IsSlideJoint", (bool (IK_sdls ::Node::*)())&IK_sdls ::Node::IsSlideJoint) // 1445
		.def("IsFreeJoint", (bool (IK_sdls ::Node::*)())&IK_sdls ::Node::IsFreeJoint) // 1445
		.def("GetEffectorNum", (int (IK_sdls ::Node::*)())&IK_sdls ::Node::GetEffectorNum) // 1445
		.def("GetJointNum", (int (IK_sdls ::Node::*)())&IK_sdls ::Node::GetJointNum) // 1445
		.def("bodyLinVel", (vector3 const & (IK_sdls ::Node::*)())&IK_sdls ::Node::bodyLinVel,return_value_policy::reference ) // 1450
		.def("bodyAngVel", (vector3 const & (IK_sdls ::Node::*)())&IK_sdls ::Node::bodyAngVel,return_value_policy::reference ) // 1450
		.def("GetJointVel", (void (IK_sdls ::Node::*)(vector3 & lin_vel, vector3 & ang_vel))&IK_sdls ::Node::GetJointVel) // 1445
		.def_readwrite("global", &IK_sdls ::Node ::_global)
		; // end of class impl___pybindgen__MotionUtil_Node           // 1505
	class_<Liegroup ::Inertia > (mainlib, "Inertia")                // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<double>())                                          // 1426
		.def(init<double,double,double,double>())                     // 1426
		.def(init<const Liegroup ::Inertia &>())                      // 1426
		.def("zero", (void (Liegroup ::Inertia::*)(void))&Liegroup ::Inertia::zero) // 1445
		.def("setMass", (void (Liegroup ::Inertia::*)(double mass))&Liegroup ::Inertia::setMass) // 1445
		.def("mass", (double (Liegroup ::Inertia::*)(void))&Liegroup ::Inertia::mass) // 1445
		.def("getOffDiag", (vector3 (Liegroup ::Inertia::*)())&Liegroup ::Inertia::getOffDiag) // 1445
		.def("getDiag", (vector3 (Liegroup ::Inertia::*)())&Liegroup ::Inertia::getDiag) // 1445
		.def("getSymm", (vector3 (Liegroup ::Inertia::*)())&Liegroup ::Inertia::getSymm) // 1445
		.def("setInertia", (void (Liegroup ::Inertia::*)(double Ixx, double Iyy, double Izz, double Ixy, double Ixz, double Iyz))&Liegroup ::Inertia::setInertia) // 1445
		.def("setOffDiag", (void (Liegroup ::Inertia::*)(const vector3& r))&Liegroup ::Inertia::setOffDiag) // 1445
		.def("transform", (Liegroup ::Inertia (Liegroup ::Inertia::*)(const transf &T))&Liegroup ::Inertia::transform) // 1445
		; // end of class impl___pybindgen__Liegroup_Inertia          // 1505
	class_<Liegroup ::se3 > (mainlib, "se3")                        // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<double>())                                          // 1426
		.def(init<double,double,double,double,double,double>())       // 1426
		.def(init<const Liegroup ::se3 &>())                          // 1426
		.def(init<const vector3 &,const vector3 &>())                 // 1426
		.def("copy", [](Liegroup::se3 const&v )->Liegroup::se3 *{ return new Liegroup::se3(v);}, TAKE_OWNERSHIP )
		.def("W", (vector3 & (Liegroup ::se3::*)())&Liegroup ::se3::W,return_value_policy::reference ) // 1450
		.def("V", (vector3 & (Liegroup ::se3::*)())&Liegroup ::se3::V,return_value_policy::reference ) // 1450
		.def_property("v",
				[](Liegroup::se3 const& v)->vector3{ return v.V();},
				[](Liegroup::se3& v, vector3 const& in){ v.V()=in;})
		.def_property("w",
				[](Liegroup::se3 const& v)->vector3{ return v.W();},
				[](Liegroup::se3& v, vector3 const& in){ v.W()=in;})
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self + vectorn()) // add (homogeneous)
		.def(self - self) // add (homogeneous)
		.def(self - vectorn()) // add (homogeneous)
		.def(self * double()) // add (homogeneous)
		.def(double() * self) // add (homogeneous)
		.def("assign", (Liegroup ::se3 & (Liegroup ::se3::*)(const Liegroup::se3 &t))&Liegroup ::se3::operator =,return_value_policy::reference ) // 1450
		.def("assign", (Liegroup ::se3 & (Liegroup ::se3::*)(double d))&Liegroup ::se3::operator =,return_value_policy::reference ) // 1450
		.def("radd", (Liegroup ::se3 & (Liegroup ::se3::*)(const Liegroup::se3 &t))&Liegroup ::se3::operator +=,return_value_policy::reference ) // 1450
		.def("rsub", (Liegroup ::se3 & (Liegroup ::se3::*)(const Liegroup::se3 &t))&Liegroup ::se3::operator -=,return_value_policy::reference ) // 1450
		.def("rmult", (Liegroup ::se3 & (Liegroup ::se3::*)(double d))&Liegroup ::se3::operator *=,return_value_policy::reference ) // 1450
		.def("rdiv", (Liegroup ::se3 & (Liegroup ::se3::*)(double d))&Liegroup ::se3::operator /=,return_value_policy::reference ) // 1450
		.def("__call__", (double & (Liegroup ::se3::*)(int i))&Liegroup ::se3::operator [],return_value_policy::reference ) // 1450
		.def("zero", (void (Liegroup ::se3::*)(void))&Liegroup ::se3::zero) // 1445
		.def("innerProduct", (double (Liegroup ::se3::*)(const Liegroup::se3& s))&Liegroup ::se3::innerProduct) // 1445
		.def("squaredLen", (double (Liegroup ::se3::*)())&Liegroup ::se3::squaredLen) // 1445
		.def("exp", (transf (Liegroup ::se3::*)())&Liegroup ::se3::exp) // 1445
		.def("log", (void (Liegroup ::se3::*)(transf const& o))&Liegroup ::se3::log) // 1445
		.def("vec", (vectornView (Liegroup ::se3::*)())&Liegroup ::se3::vec) // 1445
		; // end of class impl___pybindgen__Liegroup_se3              // 1505
	class_<Liegroup ::dse3 > (mainlib, "dse3")                      // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<double>())                                          // 1426
		.def(init<double,double,double,double,double,double>())       // 1426
		.def(init<const Liegroup ::dse3 &>())                         // 1426
		.def(init<const vector3 &,const vector3 &>())                 // 1426
		.def("copy", [](Liegroup::dse3 const&v )->Liegroup::dse3 *{ return new Liegroup::dse3(v);}, TAKE_OWNERSHIP )
		.def("M", (vector3 & (Liegroup ::dse3::*)())&Liegroup ::dse3::M,return_value_policy::reference ) // 1450
		.def("F", (vector3 & (Liegroup ::dse3::*)())&Liegroup ::dse3::F,return_value_policy::reference ) // 1450
		.def_property("m",
				[](Liegroup::dse3 const& v)->vector3{ return v.M();},
				[](Liegroup::dse3& v, vector3 const& in){ v.M()=in;})
		.def_property("f",
				[](Liegroup::dse3 const& v)->vector3{ return v.F();},
				[](Liegroup::dse3& v, vector3 const& in){ v.F()=in;})
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self - self) // add (homogeneous)
		.def(self * double()) 
		.def(double() * self) 
		.def("assign", (Liegroup ::dse3 & (Liegroup ::dse3::*)(const Liegroup::dse3 &t))&Liegroup ::dse3::operator =,return_value_policy::reference ) // 1450
		.def("assign", (Liegroup ::dse3 & (Liegroup ::dse3::*)(double d))&Liegroup ::dse3::operator =,return_value_policy::reference ) // 1450
		.def("radd", (Liegroup ::dse3 & (Liegroup ::dse3::*)(const Liegroup::dse3 &t))&Liegroup ::dse3::operator +=,return_value_policy::reference ) // 1450
		.def("rsub", (Liegroup ::dse3 & (Liegroup ::dse3::*)(const Liegroup::dse3 &t))&Liegroup ::dse3::operator -=,return_value_policy::reference ) // 1450
		.def("rmult", (Liegroup ::dse3 & (Liegroup ::dse3::*)(double d))&Liegroup ::dse3::operator *=,return_value_policy::reference ) // 1450
		.def("rdiv", (Liegroup ::dse3 & (Liegroup ::dse3::*)(double d))&Liegroup ::dse3::operator /=,return_value_policy::reference ) // 1450
		.def("__call__", (double & (Liegroup ::dse3::*)(int i))&Liegroup ::dse3::operator [],return_value_policy::reference ) // 1450
		.def("zero", (void (Liegroup ::dse3::*)(void))&Liegroup ::dse3::zero) // 1445
		.def("innerProduct", (double (Liegroup ::dse3::*)(const Liegroup::dse3& s))&Liegroup ::dse3::innerProduct) // 1445
		.def("squaredLen", (double (Liegroup ::dse3::*)())&Liegroup ::dse3::squaredLen) // 1445
		.def("getM",[] (const Liegroup::dse3 &t)->vector3 const&{ return getM(t);}, return_value_policy::reference) // 1463
		.def("getF",[] (const Liegroup::dse3 &t)->vector3 const&{ return getF(t);}, return_value_policy::reference) // 1463
		; // end of class impl___pybindgen__Liegroup_dse3             // 1505
	struct __pybindgen__MotionUtil_LoaderToTree_wrapper
	{                                                             // 1382
		static void Print(IK_sdls::LoaderToTree& self)
		{
			self.mTree.Print();
		}
		// 1383
	};                                                            // 1384
	class_<IK_sdls ::LoaderToTree > (mainlib, "LoaderToTree")       // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<MotionLoader &,std ::vector <MotionUtil ::Effector > &,std ::vector <MotionUtil ::RelativeConstraint > &,bool,bool>()) // 1426
		.def(init<MotionLoader &,bool,bool>())                        // 1426
		.def(init<>())                                                // 1426
		.def("computeTree", (void (IK_sdls ::LoaderToTree::*)())&IK_sdls ::LoaderToTree::computeTree) // 1445
		.def("nDOF", (int (IK_sdls ::LoaderToTree::*)())&IK_sdls ::LoaderToTree::nDOF) // 1445
		.def("nJoints", (int (IK_sdls ::LoaderToTree::*)())&IK_sdls ::LoaderToTree::nJoints) // 1445
		.def("calcInertia", (double (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader, vectorn& inertia))&IK_sdls ::LoaderToTree::calcInertia) // 1445
		.def("getDQindex", (intvectorn const & (IK_sdls ::LoaderToTree::*)())&IK_sdls ::LoaderToTree::getDQindex,return_value_policy::reference ) // 1450
		.def("getJoint", (IK_sdls ::Node * (IK_sdls ::LoaderToTree::*)(int jointindex))&IK_sdls ::LoaderToTree::getJoint,return_value_policy::reference ) // 1450
		.def("getNode", (IK_sdls ::Node * (IK_sdls ::LoaderToTree::*)(int treeIndex, int dofIndex))&IK_sdls ::LoaderToTree::getNode,return_value_policy::reference ) // 1450
		.def("getLastNode", (IK_sdls ::Node * (IK_sdls ::LoaderToTree::*)(int treeIndex))&IK_sdls ::LoaderToTree::getLastNode,return_value_policy::reference ) // 1450
		.def("getVarIndex", (int (IK_sdls ::LoaderToTree::*)(int treeIndex, int dofIndex))&IK_sdls ::LoaderToTree::getVarIndex) // 1445
		.def("getVarIndexByAxis", (int (IK_sdls ::LoaderToTree::*)(int treeIndex, const char *axis))&IK_sdls ::LoaderToTree::getVarIndexByAxis) // 1445
		.def("globalFrame", (const transf & (IK_sdls ::LoaderToTree::*)(int ibone))&IK_sdls ::LoaderToTree::globalFrame,return_value_policy::reference ) // 1450
		.def("setPoseDOF", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, vectorn const& pose))&IK_sdls ::LoaderToTree::setPoseDOF) // 1445
		.def("getPoseDOF", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, vectorn& pose))&IK_sdls ::LoaderToTree::getPoseDOF) // 1445
		.def("getPoseDOF", (vectorn (IK_sdls ::LoaderToTree::*)())&IK_sdls ::LoaderToTree::getPoseDOF) // 1445
		.def("setVelocity", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, vectorn const& pose))&IK_sdls ::LoaderToTree::setVelocity) // 1445
		.def("getVelocity", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, vectorn & pose))&IK_sdls ::LoaderToTree::getVelocity) // 1445
		.def("getWorldVelocity", (vector3 (IK_sdls ::LoaderToTree::*)(int ibone))&IK_sdls ::LoaderToTree::getWorldVelocity) // 1445
		.def("getWorldAngVel", (vector3 (IK_sdls ::LoaderToTree::*)(int ibone))&IK_sdls ::LoaderToTree::getWorldAngVel) // 1445
		.def("integrate", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, vectorn const& dtheta, double timeStep))&IK_sdls ::LoaderToTree::integrate) // 1445
		.def("integrate", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& mDofInfo, double timeStep))&IK_sdls ::LoaderToTree::integrate) // 1445
		.def("calcEffectorJacobianTranspose", (void (IK_sdls ::LoaderToTree::*)(matrixn& J))&IK_sdls ::LoaderToTree::calcEffectorJacobianTranspose) // 1445
		.def("calcJacobianTransposeAt", (void (IK_sdls ::LoaderToTree::*)(matrixn& J, int ibone, vector3 const& localpos))&IK_sdls ::LoaderToTree::calcJacobianTransposeAt) // 1445
		.def("getJacobianSparsity", (void (IK_sdls ::LoaderToTree::*)(boolN& hasValue, int ibone))&IK_sdls ::LoaderToTree::getJacobianSparsity) // 1445
		.def("findAxes", (void (IK_sdls ::LoaderToTree::*)(boolN& hasValue, vector3 const& axis))&IK_sdls ::LoaderToTree::findAxes) // 1445
		.def("calcJacobianTransposeAt", (void (IK_sdls ::LoaderToTree::*)(matrixn& J, int ibone, int ibone2, vector3 const& localpos))&IK_sdls ::LoaderToTree::calcJacobianTransposeAt) // 1445
		.def("calcGlobalJacobianTransposeAt", (void (IK_sdls ::LoaderToTree::*)(matrixn& J, int ibone, vector3 const& localpos))&IK_sdls ::LoaderToTree::calcGlobalJacobianTransposeAt) // 1445
		.def("calcCOM", (vector3 (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader))&IK_sdls ::LoaderToTree::calcCOM) // 1445
		.def("calcCOMjacobianTranspose", (void (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader, matrixn& JT, int chainRootBone, int chainTailBone))&IK_sdls ::LoaderToTree::calcCOMjacobianTranspose) // 1445
		.def("calcCOMjacobianTranspose", (void (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader, matrixn& JT))&IK_sdls ::LoaderToTree::calcCOMjacobianTranspose) // 1445
		.def("updateGrad_S_JT", (void (IK_sdls ::LoaderToTree::*)(vectorn& g,vector3 const& deltaS, int ibone, vector3 const& localpos))&IK_sdls ::LoaderToTree::updateGrad_S_JT) // 1445
		.def("updateCOM_grad_S_JT", (void (IK_sdls ::LoaderToTree::*)(vectorn & grad, const VRMLloader& loader, vector3 const& deltaS, double wx, double wy, double wz))&IK_sdls ::LoaderToTree::updateCOM_grad_S_JT) // 1445
		.def("calcRotJacobianTranspose", (void (IK_sdls ::LoaderToTree::*)(matrixn& J, int ibone))&IK_sdls ::LoaderToTree::calcRotJacobianTranspose) // 1445
		.def("calcMomentumJacobianTranspose", (void (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader, matrixn& JT))&IK_sdls ::LoaderToTree::calcMomentumJacobianTranspose) // 1445
		.def("calcMomentumCOM", (Liegroup ::dse3 (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader))&IK_sdls ::LoaderToTree::calcMomentumCOM) // 1445
		.def("calcInverseInertiaTimesMomentumCOM", (Liegroup ::se3 (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader))&IK_sdls ::LoaderToTree::calcInverseInertiaTimesMomentumCOM) // 1445
		.def("calcMomentumCOMfromPose", (Liegroup ::dse3 (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain1))&IK_sdls ::LoaderToTree::calcMomentumCOMfromPose) // 1445
		.def("calcMomentumCOMtoPose", (Liegroup ::dse3 (IK_sdls ::LoaderToTree::*)(const VRMLloader& loader,double delta_t, BoneForwardKinematics &chain2))&IK_sdls ::LoaderToTree::calcMomentumCOMtoPose) // 1445
		.def("getDTheta", (void (IK_sdls ::LoaderToTree::*)( vectorn& dq))&IK_sdls ::LoaderToTree::getDTheta) // 1445
		.def("getTheta", (void (IK_sdls ::LoaderToTree::*)( vectorn& q))&IK_sdls ::LoaderToTree::getTheta) // 1445
		.def("setTheta", (void (IK_sdls ::LoaderToTree::*)( vectorn const& q))&IK_sdls ::LoaderToTree::setTheta) // 1445
		.def("getSphericalState", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& spherical_dofInfo, vectorn& q, vectorn& dq))&IK_sdls ::LoaderToTree::getSphericalState) // 1445
		.def("setSphericalState", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& spherical_dofInfo, const vectorn& q, const vectorn& dq))&IK_sdls ::LoaderToTree::setSphericalState) // 1445
		.def("getSphericalQ", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& spherical_dofInfo, vectorn& q))&IK_sdls ::LoaderToTree::getSphericalQ) // 1445
		.def("setSphericalQ", (void (IK_sdls ::LoaderToTree::*)(MotionDOFinfo const& spherical_dofInfo, const vectorn& q))&IK_sdls ::LoaderToTree::setSphericalQ) // 1445
		.def("getDQ", (void (IK_sdls ::LoaderToTree::*)( vectorn& dq))&IK_sdls ::LoaderToTree::getDQ) // 1445
		.def("getQ", (void (IK_sdls ::LoaderToTree::*)( vectorn& q))&IK_sdls ::LoaderToTree::getQ) // 1445
		.def("setQ", (void (IK_sdls ::LoaderToTree::*)( vectorn const& q))&IK_sdls ::LoaderToTree::setQ) // 1445
		.def("setDQ", (void (IK_sdls ::LoaderToTree::*)( vectorn const& q))&IK_sdls ::LoaderToTree::setDQ) // 1445
		.def("setLinkData", (void (IK_sdls ::LoaderToTree::*)(vectorn const& pose, vectorn const& dpose))&IK_sdls ::LoaderToTree::setLinkData) // 1445
		.def("Print", (void (*)(IK_sdls::LoaderToTree& self))&__pybindgen__MotionUtil_LoaderToTree_wrapper::Print)  // 1458
		; // end of class impl___pybindgen__MotionUtil_LoaderToTree   // 1505
	void (Posture::*blend1)(const Posture&, const Posture&,m_real)=&Posture::Blend;

	class_<Posture>(mainlib, "Pose")
		.def(init<>())
		.def(init<const Posture&>())
		.def(init<const MotionLoader&>())
		.def("copy", [](Posture const&v )->Posture *{ return new Posture(v);}, TAKE_OWNERSHIP )
		.def("init", &Posture::Init)
		.def("identity", &Posture::identity)
		.def("assign", (&Posture::operator=))
		.def("numRotJoint", &Posture::numRotJoint)
		.def("numTransJont",&Posture::numTransJoint)
		.def("clone", &Posture::clone, RETURN_REFERENCE)
		.def("blend", blend1)
		.def("blend", (void (Posture::*)(const Posture& b, m_real t))&Posture::Blend)
		.def("align", &Posture::Align)
		.def("front",&Posture::front)
		.def("decomposeRot",&Posture::decomposeRot)
		.def_readwrite("translations", &Posture::m_aTranslations)
		.def_readwrite("rotations", &Posture::m_aRotations)
		.def_readwrite("constraint", &Posture::constraint)
		.def_readwrite("m_dv", &Posture::m_dv)
		.def_readwrite("m_dq", &Posture::m_dq)
		.def_readwrite("m_offset_y", &Posture::m_offset_y)
		.def_readwrite("m_offset_q", &Posture::m_offset_q)
		.def_readwrite("m_rotAxis_y", &Posture::m_rotAxis_y)
	;
	{
		struct __pybindgen___TRect_wrapper
		{                                                             // 1382
			inline static int get_left(TRect const& a) { return a.left; }inline static void set_left(TRect & a, int b){ a.left=b;}
			inline static int get_top(TRect const& a) { return a.top; }inline static void set_top(TRect & a, int b){ a.top=b;}
			inline static int get_right(TRect const& a) { return a.right; }inline static void set_right(TRect & a, int b){ a.right=b;}
			inline static int get_bottom(TRect const& a) { return a.bottom; }inline static void set_bottom(TRect & a, int b){ a.bottom=b;}
			// 1383
		};                                                            // 1384
		class_<TRect > (mainlib, "TRect")                               // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<>())                                                // 1426
			.def(init<int,int,int,int>())                                 // 1426
			.def_property("left", &__pybindgen___TRect_wrapper::get_left, &__pybindgen___TRect_wrapper::set_left) 
			.def_property("top", &__pybindgen___TRect_wrapper::get_top, &__pybindgen___TRect_wrapper::set_top) 
			.def_property("right", &__pybindgen___TRect_wrapper::get_right, &__pybindgen___TRect_wrapper::set_right) 
			.def_property("bottom", &__pybindgen___TRect_wrapper::get_bottom, &__pybindgen___TRect_wrapper::set_bottom) 
			; // end of class impl___pybindgen___TRect                    // 1505
		struct CImage_
		{
			// does not copy memory.
			static PyObject* ref(CImage const& v)
			{
				npy_intp dims[3];
				dims[0]=v.GetHeight();
				dims[1]=v.GetWidth();
				dims[2]=3; // RGB
				npy_intp strides[3];
				strides[0]=sizeof(uchar)*v._stride;
				strides[1]=sizeof(uchar)*3;
				strides[2]=sizeof(uchar);
				uchar* vv=v._dataPtr;
				PyObject* o=PyArray_NewFromDescr(&PyArray_Type, PyArray_DescrFromType (NPY_UINT8), 3, dims, strides, vv, 
						//NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_WRITEABLE
						NPY_ARRAY_CARRAY | NPY_ARRAY_WRITEABLE
						, NULL);
				return o;
			}
		};
		class_<CImage > (mainlib, "CImage")                       // 1393
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
			.def(init<>())                                                // 1431
			.def_property_readonly("array", [](CImage const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(CImage_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("ref",  [](CImage const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(CImage_::ref(v)); }, WRAP_PY::keep_alive<0,1>()) // return value(0) depends on self(1).
			.def("GetWidth", &CImage::GetWidth)                           // 1450
																		  //when necessary, check c++ header: .def("GetWidth", (int (CImage::*)())&CImage::GetWidth) // 1451
			.def("GetHeight", &CImage::GetHeight)                         // 1450
																		  //when necessary, check c++ header: .def("GetHeight", (int (CImage::*)())&CImage::GetHeight) // 1451
			.def("Load", &CImage::Load)                                   // 1450
																		  //when necessary, check c++ header: .def("Load", (bool (CImage::*)(const char* filename))&CImage::Load) // 1451
			.def("Save", &CImage::Save)                                   // 1450
																		  //when necessary, check c++ header: .def("Save", (bool (CImage::*)(const char* filename))&CImage::Save) // 1451
			.def("save", &CImage::save)                                   // 1450
																		  //when necessary, check c++ header: .def("save", (bool (CImage::*)(const char* filename, int BPP))&CImage::save) // 1451
			.def("create", &CImage::Create)                               // 1450
																		  //when necessary, check c++ header: .def("create", (bool (CImage::*)(int width, int height))&CImage::Create) // 1451
			.def("CopyFrom", &CImage::CopyFrom)                           // 1450
																		  //when necessary, check c++ header: .def("CopyFrom", (void (CImage::*)(CImage const& other))&CImage::CopyFrom) // 1451
			.def("GetPixel", (CPixelRGB8 * (CImage::*)(int i, int j))&CImage::GetPixel,return_value_policy::reference ) // 1456
			.def("drawBox", (void (*)(CImage& inout, TRect const& t, int R, int G, int B))&Imp::drawBox) // 1464
			.def("sharpen", (void (*)(CImage& inout, double factor, int iterations))&Imp::sharpen) // 1464
			.def("contrast", (void (*)(CImage& inout, double factor))&Imp::contrast) // 1464
			.def("gammaCorrect", (void (*)(CImage& inout, double factor))&Imp::gammaCorrect) // 1464
			.def("dither", (void (*)(CImage& inout, int levels))&Imp::dither) // 1464
			.def("resize", (void (*)(CImage& inout, int width, int height))&Imp::resize) // 1464
			.def("blit", (void (*)(CImage& out, CImage const& in, TRect const& rect_in, int x, int y))&Imp::blit) // 1464
			.def("concatVertical", (void (*)(CImage& out, CImage const& a, CImage const& b))&Imp::concatVertical) // 1464
			.def("crop", (void (*)(CImage& out, CImage const& in, int left, int top, int right, int bottom))&Imp::crop) // 1464
			.def("rotateRight", (void (*)(CImage& other))&Imp::rotateRight) // 1464
			.def("rotateLeft", (void (*)(CImage& other))&Imp::rotateLeft) // 1464
			.def("hsv2rgb", (vector3 (*)(vector3 const& in))&hsv2rgb)     // 1464
			.def("rgb2hsv", (vector3 (*)(vector3 const& in))&rgb2hsv)     // 1464
			; // end of class impl___pybindgen___CImage                   // 1511
		class_<CImagePixel > (mainlib, "Pixels")                  // 1393
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1397
			.def(init<>())                                                // 1431
			.def(init<CImage *>())                                        // 1431
			.def("SetPixel", (void (CImagePixel::*)(int x, int y, CPixelRGB8 color))&CImagePixel::SetPixel) // 1451
			.def("GetPixel", [](CImagePixel*p ,int x, int y)->CPixelRGB8&{ return p->GetPixel(x,y);},return_value_policy::reference ) // 1456
			.def("Pixel", [](CImagePixel* p,int x, int y)->CPixelRGB8&{ return p->GetPixel(x,y);},return_value_policy::reference ) // 1456
			.def("SetPixel", (void (CImagePixel::*)(float x, float y, CPixelRGB8 color))&CImagePixel::SetPixel) // 1451
			.def("DrawHorizLine", &CImagePixel::DrawHorizLine)            // 1450
																		  //when necessary, check c++ header: .def("DrawHorizLine", (void (CImagePixel::*)(int x, int y, int width, CPixelRGB8 color))&CImagePixel::DrawHorizLine) // 1451
			.def("DrawVertLine", &CImagePixel::DrawVertLine)              // 1450
																		  //when necessary, check c++ header: .def("DrawVertLine", (void (CImagePixel::*)(int x, int y, int height, CPixelRGB8 color))&CImagePixel::DrawVertLine) // 1451
			.def("DrawVertLine", &CImagePixel::DrawVertLine)              // 1450
																		  //when necessary, check c++ header: .def("DrawVertLine", (void (CImagePixel::*)(int x, int y, int height, CPixelRGB8 color,bool bDotted))&CImagePixel::DrawVertLine) // 1451
			.def("DrawLine", &CImagePixel::DrawLine)                      // 1450
																		  //when necessary, check c++ header: .def("DrawLine", (void (CImagePixel::*)(int x1, int y1, int x2, int y2, CPixelRGB8 color))&CImagePixel::DrawLine) // 1451
			.def("DrawBox", &CImagePixel::DrawBox)                        // 1450
																		  //when necessary, check c++ header: .def("DrawBox", (void (CImagePixel::*)(const TRect& rect, CPixelRGB8 color))&CImagePixel::DrawBox) // 1451
			.def("DrawLineBox", &CImagePixel::DrawLineBox)                // 1450
																		  //when necessary, check c++ header: .def("DrawLineBox", (void (CImagePixel::*)(const TRect& rect, CPixelRGB8 color))&CImagePixel::DrawLineBox) // 1451
			.def("DrawPattern", [](CImagePixel* p,int x, int y, const CImagePixel& patternPixel){
					p->DrawPattern(x,y, patternPixel);}) // 1451
			.def("DrawPattern", [](CImagePixel*p,int x, int y, CImage* pPattern, bool bUseColorKey, CPixelRGB8 colorkey){
					p->DrawPattern(x,y,pPattern, bUseColorKey, colorkey);}) // 1451
			.def("DrawSubPattern", [](CImagePixel*p,int x, int y, const CImagePixel& patternPixel, const TRect& patternRect){
					p->DrawSubPattern(x,y, patternPixel, patternRect);}) // 1451
			.def("Clear", &CImagePixel::Clear)                            // 1450
			.def("DrawText", [](CImagePixel*p,int x, int y, const char* str){
					p->drawText(x,y,str);}) // 1451
			.def("DrawText", &CImagePixel::drawText)                      // 1450
			.def("Width", &CImagePixel::Width)                            // 1450
			.def("Height", &CImagePixel::Height)                          // 1450
			.def("GetPixel", [](CImagePixel& pixel, float x, float y)->CPixelRGB8{
					int count;
					return pixel.GetPixel(x,y, count);} ) // 1464
			; // end of class impl___pybindgen__CImage_Pixels             // 1511
	}
	{
		void (TStrings::*set)(int i, const char* v)=&TStrings::set;
		void (TStrings::*pushback)(const char* v)=&TStrings::pushBack;
		class_<TStrings>(mainlib, "TStrings")
			.def(init<>())
			.def(init<int>())
			.def("set", set)
			.def("get", &TStrings::get)
			.def("pushBack",pushback)
			.def("find", &TStrings::find)
			.def("size", &TStrings::size)
			.def("resize", &TStrings::resize)
			;
	}

	{
		// pose transfer
		class_<PoseTransfer > (mainlib, "PoseTransfer")                 // 1389
			.def(init<MotionLoader *,MotionLoader *>())                   // 1426
			.def(init<MotionLoader *,MotionLoader *,bool>())              // 1426
			.def(init<MotionLoader *,MotionLoader *,TStrings const &,TStrings const &,bool>()) // 1426
			.def("setTargetSkeleton", (void (PoseTransfer::*)(const Posture & srcposture))&PoseTransfer::setTargetSkeleton) // 1445
			.def("setTargetSkeleton", (void (PoseTransfer::*)(const vectorn & srcposture))&PoseTransfer::setTargetSkeleton) // 1445
			.def("setTargetSkeletonBothRotAndTrans", (void (PoseTransfer::*)(const Posture& srcposture))&PoseTransfer::setTargetSkeletonBothRotAndTrans) // 1445
			.def("source", &PoseTransfer::source,return_value_policy::reference ) // 1450
			.def("target", &PoseTransfer::target,return_value_policy::reference ) // 1450
			; // end of class impl___pybindgen__MotionUtil_PoseTransfer   // 1505
		class_<PoseTransfer2 > (mainlib, "PoseTransfer2")               // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def(init<MotionLoader *,MotionLoader *,TStrings const &,TStrings const &,double>()) // 1426
			.def(init<MotionLoader *,MotionLoader *>())                   // 1426
			.def("_setTargetSkeleton", (void (PoseTransfer2::*)())&PoseTransfer2::_setTargetSkeleton) // 1445
			.def("setTargetSkeleton", (void (PoseTransfer2::*)(const Posture & srcposture))&PoseTransfer2::setTargetSkeleton) // 1445
			.def("setTargetSkeleton", (void (PoseTransfer2::*)(const vectorn & srcposture))&PoseTransfer2::setTargetSkeleton) // 1445
			.def("setTargetSkeleton", (void (PoseTransfer2::*)(const BoneForwardKinematics & srcposture))&PoseTransfer2::setTargetSkeleton) // 1445
			.def("source", (MotionLoader * (PoseTransfer2::*)())&PoseTransfer2::source,return_value_policy::reference ) // 1450
			.def("target", (MotionLoader * (PoseTransfer2::*)())&PoseTransfer2::target,return_value_policy::reference ) // 1450
			.def_readwrite("rAtoB_additionalAindices", &PoseTransfer2::rAtoB_additionalAindices)
			.def_readwrite("rAtoB_additionalBindices", &PoseTransfer2::rAtoB_additionalBindices)
			.def_readwrite("targetIndexAtoB", &PoseTransfer2::targetIndexAtoB)
			.def_readwrite("BtoA", &PoseTransfer2::BtoA)
			.def_readwrite("parentIdx",&PoseTransfer2::parentIdx)
			; // end of class impl___pybindgen__MotionUtil_PoseTransfer2  // 1505
	}
	// motionloader
	{

		struct MotionLoaderWrapper
		{
			static void getLinks(MotionLoader& loader, intvectorn& from, intvectorn & to )
			{
				NodeStack & stack=loader.m_TreeStack;

				stack.Initiate();

				Node *src=loader.m_pTreeRoot->m_pChildHead;	// dummy?????? ????????.
				while(TRUE)
				{
					for(;src; src=src->m_pChildHead)
					{
						Node* child;
						for(child=src->m_pChildHead; child; child=child->m_pSibling)
						{
							//printf("%s -> %s\n", src->NameId, child->NameId);
							from.pushBack(loader.GetIndex(src));
							to.pushBack(loader.GetIndex(child));

						}
						stack.Push(src);
					}
					stack.Pop(&src);
					if(!src) break;
					src=src->m_pSibling;
				}
			}
			static MotionLoader* getMotionLoader(const char* fn)
			{
				return RE::renderer().m_pMotionManager->GetMotionLoaderPtr(fn);
			}
			static MotionLoader* _create(const char* filename)
			{
				MotionLoader* l;

				TString fn(filename);
				TString ext=fn.right(3).toUpper();
				if(ext=="ASF")
				{
					l=new ASFLoader(filename);
				}
				else if (ext=="WRL")
				{
					l=new VRMLloader(filename);
				}
				else if(ext=="BVH")
				{
					l=new BVHLoader(filename);
				}
				else if(ext=="SKL"){
					l=new BVHLoader(fn.left(-3)+"BVH","loadSkeletonOnly");
				}
				else
				{
					l=new MotionLoader(filename);
				}
				return l;
			}
			static void _append(MotionLoader* l, const char* motFile)
			{
				Motion& mot=l->m_cPostureIP;

				if(mot.numFrames()==0)
					l->loadAnimation(mot, motFile);
				else
					RE::motion::concatFromFile(mot, motFile);
			}
			static void loadAnimation(MotionLoader& skel, Motion& mot, const char* fn)
			{
				RE::motion::loadAnimation(skel, mot, fn);
			}
		};
		void (MotionLoader::*setPose1)(const Posture& pose) const=&MotionLoader::setPose;
		void (MotionLoader::*setChain2)(const Posture& pose, Bone& bone) const=&MotionLoader::setChain;

		mainlib.attr("Voca") = WRAP_PY::module::import("enum").attr("IntEnum")
        ("Voca", WRAP_PY::dict(
				"CONSTRAINT_LEFT_FOOT"_a= (int)CONSTRAINT_LEFT_FOOT,
				"CONSTRAINT_RIGHT_FOOT"_a= (int)CONSTRAINT_RIGHT_FOOT,
				"CONSTRAINT_LEFT_TOE"_a= (int)CONSTRAINT_LEFT_TOE,
				"CONSTRAINT_RIGHT_TOE"_a= (int)CONSTRAINT_RIGHT_TOE,
				"CONSTRAINT_LEFT_HEEL"_a= (int)CONSTRAINT_LEFT_HEEL,
				"CONSTRAINT_RIGHT_HEEL"_a= (int)CONSTRAINT_RIGHT_HEEL,
				"HIPS"_a= (int)MotionLoader::HIPS,
				"LEFTHIP"_a=(int)MotionLoader::LEFTHIP,
				"LEFTKNEE"_a= (int)MotionLoader::LEFTKNEE,
				"LEFTANKLE"_a= (int)MotionLoader::LEFTANKLE,
				"LEFTTOES"_a= (int)MotionLoader::LEFTTOES,
				"RIGHTHIP"_a= (int)MotionLoader::RIGHTHIP,
				"RIGHTKNEE"_a= (int)MotionLoader::RIGHTKNEE,
				"RIGHTANKLE"_a= (int)MotionLoader::RIGHTANKLE,
				"RIGHTTOES"_a= (int)MotionLoader::RIGHTTOES,
				"CHEST"_a= (int)MotionLoader::CHEST,
				"CHEST2"_a= (int)MotionLoader::CHEST2,
				"LEFTCOLLAR"_a= (int)MotionLoader::LEFTCOLLAR,
				"LEFTSHOULDER"_a= (int)MotionLoader::LEFTSHOULDER,
				"LEFTELBOW"_a= (int)MotionLoader::LEFTELBOW,
				"LEFTWRIST"_a= (int)MotionLoader::LEFTWRIST,
				"RIGHTCOLLAR"_a= (int)MotionLoader::RIGHTCOLLAR,
				"RIGHTSHOULDER"_a= (int)MotionLoader::RIGHTSHOULDER,
				"RIGHTELBOW"_a= (int)MotionLoader::RIGHTELBOW,
				"RIGHTWRIST"_a= (int)MotionLoader::RIGHTWRIST,
				"NECK"_a= (int)MotionLoader::NECK,
				"HEAD"_a= (int)MotionLoader::HEAD,
				"LThWrist"_a=(int)MotionLoader::LThWrist,
				"LThMetac"_a=(int)MotionLoader::LThMetac,
				"LThIntra1"_a=(int)MotionLoader::LThIntra1,
				"LF1Metac"_a=(int)MotionLoader::LF1Metac,
				"LF1Intra1"_a=(int)MotionLoader::LF1Intra1,
				"LF1Intra2"_a=(int)MotionLoader::LF1Intra2,
				"LF2Metac"_a=(int)MotionLoader::LF2Metac,
				"LF2Intra1"_a=(int)MotionLoader::LF2Intra1,
				"LF2Intra2"_a=(int)MotionLoader::LF2Intra2,
				"LF3Metac"_a=(int)MotionLoader::LF3Metac,
				"LF3Intra1"_a=(int)MotionLoader::LF3Intra1,
				"LF3Intra2"_a=(int)MotionLoader::LF3Intra2,
				"LF4Metac"_a=(int)MotionLoader::LF4Metac,
				"LF4Intra1"_a=(int)MotionLoader::LF4Intra1,
				"LF4Intra2"_a=(int)MotionLoader::LF4Intra2,
				"RThWrist"_a=(int)MotionLoader::RThWrist,
				"RThMetac"_a=(int)MotionLoader::RThMetac,
				"RThIntra1"_a=(int)MotionLoader::RThIntra1,
				"RF1Metac"_a=(int)MotionLoader::RF1Metac,
				"RF1Intra1"_a=(int)MotionLoader::RF1Intra1,
				"RF1Intra2"_a=(int)MotionLoader::RF1Intra2,
				"RF2Metac"_a=(int)MotionLoader::RF2Metac,
				"RF2Intra1"_a=(int)MotionLoader::RF2Intra1,
				"RF2Intra2"_a=(int)MotionLoader::RF2Intra2,
				"RF3Metac"_a=(int)MotionLoader::RF3Metac,
				"RF3Intra1"_a=(int)MotionLoader::RF3Intra1,
				"RF3Intra2"_a=(int)MotionLoader::RF3Intra2,
				"RF4Metac"_a=(int)MotionLoader::RF4Metac,
				"RF4Intra1"_a=(int)MotionLoader::RF4Intra1,
				"RF4Intra2"_a=(int)MotionLoader::RF4Intra2
					));

		class_<MotionLoader >(mainlib, "MotionLoader")
			.def(init<>())
			.def(init<const std::string&>())
			.def_readwrite("mMotion", &MotionLoader::m_cPostureIP)
			.def_readwrite("dofInfo", &MotionLoader::dofInfo)
			.def("pose",&MotionLoader::pose)
			.def("loadAnimationExt", (void (MotionLoader::*)(Motion& mot, const char* fn))&MotionLoader::loadAnimation) // 1447
			.def("insertSiteBones", (void (MotionLoader::*)())&MotionLoader::insertSiteBones) // 1447
			.def("fkSolver", (BoneForwardKinematics & (MotionLoader::*)())&MotionLoader::fkSolver,return_value_policy::reference ) // 1452
			.def("readJointIndex", (void (MotionLoader::*)(const char* filename))&MotionLoader::readJointIndex) // 1447
			.def("numRotJoint", (int (MotionLoader::*)())&MotionLoader::numRotJoint) // 1447
			.def("numTransJoint", (int (MotionLoader::*)())&MotionLoader::numTransJoint) // 1447
			.def("numBone", (int (MotionLoader::*)())&MotionLoader::numBone) // 1447
			.def("numEndBone", (int (MotionLoader::*)())&MotionLoader::numEndBone) // 1447
			.def("setCurPoseAsInitialPose", (void (MotionLoader::*)())&MotionLoader::setCurPoseAsInitialPose) // 1447
			.def("updateInitialBone", (void (MotionLoader::*)())&MotionLoader::UpdateInitialBone) // 1447
			.def("updateBone", (void (MotionLoader::*)())&MotionLoader::UpdateBone) // 1447
			.def("getPose", (void (MotionLoader::*)(Posture& pose))&MotionLoader::getPose) // 1447
			.def("Scale", (void (MotionLoader::*)(float fScale))&MotionLoader::Scale) // 1447
			.def("scale", (void (MotionLoader::*)(float fScale, Motion& mot))&MotionLoader::scale) // 1447
			.def("updateBoneLengthFromGlobal", (void (MotionLoader::*)())&MotionLoader::updateBoneLengthFromGlobal) // 1447
			.def("createDummyRootBone", (void (MotionLoader::*)())&MotionLoader::createDummyRootBone) // 1447
			.def("setChain", (void (MotionLoader::*)(const Posture&, int) const)&MotionLoader::setChain) // 1447
			.def("setChain", (void (MotionLoader::*)(const Posture&, Bone&) const)&MotionLoader::setChain) // 1447
			.def("setNewRoot", (void (MotionLoader::*)(Bone& newRoot))&MotionLoader::setNewRoot) // 1447
			.def("insertJoint", (void (MotionLoader::*)(Bone&, const char* type))&MotionLoader::insertJoint) // 1447
			.def("insertChildBone", (void (MotionLoader::*)(Bone& parent, const char* nameId, bool bMoveChildren))&MotionLoader::insertChildBone) // 1447
			.def("insertChildBone", (void (MotionLoader::*)(Bone& parent, const char* nameId))&MotionLoader::insertChildBone) // 1447
			.def("exportSkeleton", (void (MotionLoader::*)( const char* filename))&MotionLoader::exportSkeleton) // 1447
			.def("removeBone", (void (MotionLoader::*)(Bone& target))&MotionLoader::removeBone) // 1447
			.def("removeAllRedundantBones", (void (MotionLoader::*)())&MotionLoader::removeAllRedundantBones) // 1447
			.def("setPose", (void (MotionLoader::*)(const Posture& pose) const)&MotionLoader::setPose) // 1447
			.def("getPose", (void (MotionLoader::*)(Posture& pose) const)&MotionLoader::getPose) // 1447
			.def("setSphericalQ", (void (MotionLoader::*)(const vectorn& q) const)&MotionLoader::setSphericalQ) // 1447
			.def("setPoseDOF", (void (MotionLoader::*)(const vectorn& poseDOF) const)&MotionLoader::setPoseDOF) // 1447
			.def("getPoseDOF", (void (MotionLoader::*)(vectorn& poseDOF) const)&MotionLoader::getPoseDOF) // 1447
			.def("getPoseDOF", (vectorn (MotionLoader::*)() const)&MotionLoader::getPoseDOF) // 1447
			.def("bone", (Bone & (MotionLoader::*)(int index) const)&MotionLoader::bone,return_value_policy::reference ) // 1452
			.def("getBoneByTreeIndex", (Bone & (MotionLoader::*)(int index) const)&MotionLoader::getBoneByTreeIndex,return_value_policy::reference ) // 1452
			.def("getBoneByRotJointIndex", (Bone & (MotionLoader::*)(int iRotJoint) const)&MotionLoader::getBoneByRotJointIndex,return_value_policy::reference ) // 1452
			.def("getBoneByTransJointIndex", (Bone & (MotionLoader::*)(int iTransJoint) const)&MotionLoader::getBoneByTransJointIndex,return_value_policy::reference ) // 1452
			.def("getBoneByVoca", (Bone & (MotionLoader::*)(int jointVoca) const)&MotionLoader::getBoneByVoca,return_value_policy::reference ) // 1452
			.def("getBoneByName", (Bone & (MotionLoader::*)(const char*) const)&MotionLoader::getBoneByName,return_value_policy::reference ) // 1452
			.def("getTreeIndexByName", (int (MotionLoader::*)(const char* name) const)&MotionLoader::getTreeIndexByName) // 1447
			.def("getTreeIndexByRotJointIndex", (int (MotionLoader::*)(int rotjointIndex) const)&MotionLoader::getTreeIndexByRotJointIndex) // 1447
			.def("getTreeIndexByTransJointIndex", (int (MotionLoader::*)(int transjointIndex) const)&MotionLoader::getTreeIndexByTransJointIndex) // 1447
			.def("getTreeIndexByVoca", (int (MotionLoader::*)(int jointVoca) const)&MotionLoader::getTreeIndexByVoca) // 1447
			.def("getRotJointIndexByName", (int (MotionLoader::*)(const char* nameID) const)&MotionLoader::getRotJointIndexByName) // 1447
			.def("getRotJointIndexByTreeIndex", (int (MotionLoader::*)(int treeIndex) const)&MotionLoader::getRotJointIndexByTreeIndex) // 1447
			.def("getRotJointIndexByVoca", (int (MotionLoader::*)(int jointVoca) const)&MotionLoader::getRotJointIndexByVoca) // 1447
			.def("getTransJointIndexByName", (int (MotionLoader::*)(const char* nameID) const)&MotionLoader::getTransJointIndexByName) // 1447
			.def("getTransJointIndexByTreeIndex", (int (MotionLoader::*)(int treeIndex) const)&MotionLoader::getTransJointIndexByTreeIndex) // 1447
			.def("getVocaByTreeIndex", (int (MotionLoader::*)(int treeIndex) const)&MotionLoader::getVocaByTreeIndex) // 1447
			.def("getVocaByRotJointIndex", (int (MotionLoader::*)(int rotjointIndex) const)&MotionLoader::getVocaByRotJointIndex) // 1447
			.def("_changeVoca", (void (MotionLoader::*)(int jointVoca, Bone & bone) )&MotionLoader::_changeVoca) // 1447
			.def("_updateTreeIndex", (void (MotionLoader::*)())&MotionLoader::_updateTreeIndex) // 1447
			.def("_initDOFinfo", (void (MotionLoader::*)())&MotionLoader::_initDOFinfo) // 1447
			.def("printHierarchy", (void (MotionLoader::*)())&MotionLoader::printHierarchy) // 1447
			.def("print", (void (MotionLoader::*)() const)&MotionLoader::printHierarchy) // 1447
			.def("loadAnimation", (void (*)(MotionLoader& skel, Motion& mot, const char* fn))&MotionLoaderWrapper::loadAnimation) // 1460
			.def("_create", (MotionLoader * (*)(const char* filename))&MotionLoaderWrapper::_create, return_value_policy::take_ownership) // 1463
			.def("_append", (void (*)(MotionLoader* l, const char* motFile))&MotionLoaderWrapper::_append) // 1460
			.def("getMotionLoader", (MotionLoader * (*)(const char* fn))&MotionLoaderWrapper::getMotionLoader, return_value_policy::reference) // 1465
			.def("getLinks", &MotionLoaderWrapper::getLinks)
		;	
		class_<VRMLloader, MotionLoader >(mainlib, "VRMLloader")
			.def(init<>())                                                // 1425
			.def(init<const std::string&>())                                    // 1425
			.def(init<VRMLloader const &>())                              // 1425
			.def(init<MotionLoader const &,double>())                     // 1425
			.def(init<OBJloader ::Geometry const &,bool>())               // 1425
			.def(init<OBJloader ::Terrain *>())                           // 1425
			.def("copy", [](VRMLloader const&v )->VRMLloader *{ return new VRMLloader(v);}, TAKE_OWNERSHIP )
			.def("getURL", [](VRMLloader& l)->std::string{ return l.getURL().tostring();}) // 1447
			.def("setURL", (void (VRMLloader::*)(const char* u))&VRMLloader::setURL) // 1447
			.def("setPosition", &VRMLloader::setPosition)
			.def("setTotalMass", (void (VRMLloader::*)( m_real totalMass))&VRMLloader::setTotalMass) // 1447
			.def("changeAll3DOFjointsToSpherical", (void (VRMLloader::*)())&VRMLloader::changeAll3DOFjointsToSpherical) // 1447
			.def("changeAllMultiDOFjointsToSpherical", (void (VRMLloader::*)())&VRMLloader::changeAllMultiDOFjointsToSpherical) // 1447
			.def("changeAllJointsToSpherical", (void (VRMLloader::*)())&VRMLloader::changeAllJointsToSpherical) // 1447
			.def("assign", (void (VRMLloader::*)(VRMLloader const&))&VRMLloader::operator=) // 1447
			.def("_initDOFinfo", (void (VRMLloader::*)())&VRMLloader::_initDOFinfo) // 1447
			.def("printDebugInfo", (void (VRMLloader::*)())&VRMLloader::printDebugInfo) // 1447
			.def("changeTotalMass", (void (VRMLloader::*)( m_real totalMass))&VRMLloader::changeTotalMass) // 1447
			.def("VRMLbone", (VRMLTransform & (VRMLloader::*)(int treeIndex))&VRMLloader::VRMLbone,return_value_policy::reference ) // 1452
			.def("setChannels", (void (VRMLloader::*)(Bone& bone, const char* translation_axis, const char* rotation_axis))&VRMLloader::setChannels) // 1447
			.def("insertChildJoint", (void (VRMLloader::*)(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren))&VRMLloader::insertChildJoint) // 1447
			.def("insertChildJoint", (void (VRMLloader::*)(Bone& parent, const char* tchannels, const char* rchannels, const char* nameId, bool bMoveChildren, vector3 const& offset))&VRMLloader::insertChildJoint) // 1447
			.def("calcCOM", (vector3 (VRMLloader::*)())&VRMLloader::calcCOM) // 1447
			.def("calcZMP", (void (VRMLloader::*)(const MotionDOF& motion, matrixn & aZMP, double kernelSize))&VRMLloader::calcZMP) // 1447
			.def("export", (void (VRMLloader::*)(const char* filename))&VRMLloader::exportVRML) // 1447
			.def("exportBinary", (void (VRMLloader::*)(const char* filename))&VRMLloader::exportBinary) // 1447
			.def("numHRPjoints", (int (VRMLloader::*)())&VRMLloader::numHRPjoints) // 1447
			.def("scale", (void (VRMLloader::*)(float fScale,Motion& mot))&VRMLloader::scale) // 1447
			.def("addRelativeConstraint", (void (VRMLloader::*)(int ibone1, vector3 const& lpos1, int ibone2, vector3 const& lpos2))&VRMLloader::addRelativeConstraint) // 1447
			.def("_getAllRelativeConstraints", (void (VRMLloader::*)(intvectorn& ibone, vector3N& localpos))&VRMLloader::_getAllRelativeConstraints) // 1447
			.def("projectAngles", (void (*)(vectorn & temp1))&VRMLloader::projectAngles) // 1460
			.def("name", [](VRMLloader& l)->std::string{ return std::string(l.name.ptr());}) // 1465
			.def("setName",[]( VRMLloader& l, const char* name) { l.name=name; })
			;
	}

	// bone
	{	

		vector3 const& (Bone::*getTranslation1)() const=&Bone::getTranslation;
		quater const& (Bone::*getRotation1)() const=&Bone::getRotation;
		struct __pybindgen__Bone_wrapper
		{
            static bool isChildHeadValid(Bone& bone)   { return bone.m_pChildHead!=NULL;}
            static bool isChildTailValid(Bone& bone)   { return bone.m_pChildTail!=NULL;}
            static bool isSiblingValid(Bone& bone)   { return bone.m_pSibling!=NULL;}

            static Bone* childHead(Bone& bone)    { return ((Bone*)bone.m_pChildHead);}
            static Bone* childTail(Bone& bone)    { return ((Bone*)bone.m_pChildTail);}
            static Bone* sibling(Bone& bone)    { return ((Bone*)bone.m_pSibling);}

            static void getTranslation(Bone& bone, vector3& trans)
            {
                bone.getTranslation(trans);
            }
            static vector3 getTranslation(Bone& bone)
            {
                return bone.getTranslation();
            }
            static vector3 getOffset(Bone& bone)
            {
                vector3 v;
                bone.getOffset(v);
                return v;
            }

            static void getRotation(Bone& bone, quater& q)
            {
                bone.getRotation(q);
            }
            static bool eq(Bone& bone1, Bone& bone2)
            {
                return &bone1==&bone2;
            }

            static std::string name(Bone& bone)
            {
                return std::string(bone.NameId);
            }
                                                              // 1387
};                                                            // 1388
		class_<Bone > (mainlib,"Bone")
			.def("numChildren", (int (Bone::*)())&Bone::numChildren)      // 1447
			.def("setName", (void (Bone::*)(const char*))&Bone::SetNameId) // 1447
			.def("getOffsetTransform", (transf & (Bone::*)())&Bone::_getOffsetTransform,return_value_policy::reference ) // 1452
			.def("getFrame", (transf & (Bone::*)())&Bone::_getFrame,return_value_policy::reference ) // 1452
			.def("getLocalFrame", (transf & (Bone::*)())&Bone::_getLocalFrame,return_value_policy::reference ) // 1452
			.def("length", (m_real (Bone::*)())&Bone::length)             // 1447
			.def("getOffset", (void (Bone::*)(vector3& offset))&Bone::getOffset) // 1447
			.def("axis", (vector3 (Bone::*)(int ichannel))&Bone::axis)    // 1447
			.def("parent", (Bone * (Bone::*)())&Bone::parent,return_value_policy::reference ) // 1452
			.def("sibling", (Bone * (Bone::*)())&Bone::sibling,return_value_policy::reference ) // 1452
			.def("isDescendent", (bool (Bone::*)(const Bone * parent))&Bone::isDescendent) // 1447
			.def("getRotationalChannels", [](Bone const& bone)->std::string{ return bone.getRotationalChannels().ptr();})
			.def("getTranslationalChannels", [](Bone const& bone)->std::string{ return bone.getTranslationalChannels().ptr();})
			.def("setChannels", (void (Bone::*)(const char* translation_axis, const char* rotation_axis))&Bone::setChannels) // 1447
			.def("numChannels", (int (Bone::*)())&Bone::numChannels)      // 1447
			.def("parent", (Bone * (Bone::*)())&Bone::parent,return_value_policy::reference ) // 1452
			.def("voca", (int (Bone::*)())&Bone::voca)                    // 1447
			.def("getSkeleton", (MotionLoader const & (Bone::*)())&Bone::getSkeleton,return_value_policy::reference ) // 1452
			.def("treeIndex", (int (Bone::*)())&Bone::treeIndex)          // 1447
			.def("rotJointIndex", (int (Bone::*)())&Bone::rotJointIndex)  // 1447
			.def("transJointIndex", (int (Bone::*)())&Bone::transJointIndex) // 1447
			.def("isChildHeadValid", (bool (*)(Bone& bone))&__pybindgen__Bone_wrapper::isChildHeadValid) // 1460
			.def("isChildTailValid", (bool (*)(Bone& bone))&__pybindgen__Bone_wrapper::isChildTailValid) // 1460
			.def("isSiblingValid", (bool (*)(Bone& bone))&__pybindgen__Bone_wrapper::isSiblingValid) // 1460
			.def("childHead", (Bone * (*)(Bone& bone))&__pybindgen__Bone_wrapper::childHead, return_value_policy::reference) // 1465
			.def("childTail", (Bone * (*)(Bone& bone))&__pybindgen__Bone_wrapper::childTail, return_value_policy::reference) // 1465
			.def("sibling", (Bone * (*)(Bone& bone))&__pybindgen__Bone_wrapper::sibling, return_value_policy::reference) // 1465
			.def("getTranslation", (void (*)(Bone& bone, vector3& trans))&__pybindgen__Bone_wrapper::getTranslation) // 1460
			.def("getTranslation", (vector3 (*)(Bone& bone))&__pybindgen__Bone_wrapper::getTranslation) // 1460
			.def("getOffset", (vector3 (*)(Bone& bone))&__pybindgen__Bone_wrapper::getOffset)        // 1460
			.def("getRotation", (void (*)(Bone& bone, quater& q))&__pybindgen__Bone_wrapper::getRotation) // 1460
			.def("__eq__", (bool (*)(Bone& bone1, Bone& bone2))&__pybindgen__Bone_wrapper::eq)         // 1460
			.def("name", (std ::string (*)(Bone& bone))&__pybindgen__Bone_wrapper::name)             // 1460
			.def("__repr__", (std ::string (*)(Bone& bone))&__pybindgen__Bone_wrapper::name)       // 1460
			.def("getJointAxis", &Bone::getJointAxis)
		;
		class_<VRMLTransform ,Bone > (mainlib, "VRMLTransform")               // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
			.def("createNewShape", (void (VRMLTransform::*)())&VRMLTransform::createNewShape) // 1445
			.def("removeShape", (void (VRMLTransform::*)())&VRMLTransform::removeShape) // 1445
			.def("translateMesh", (void (VRMLTransform::*)( vector3 const& trans))&VRMLTransform::translateMesh) // 1445
			.def("setJointRange", (void (VRMLTransform::*)(int i, double min_deg, double max_deg))&VRMLTransform::setJointRange) // 1445
			.def("setJointPosition", (void (VRMLTransform::*)(vector3 const& trans))&VRMLTransform::setJointPosition) // 1445
			.def("setJointAxes", (void (VRMLTransform::*)(const char* axes))&VRMLTransform::setJointAxes) // 1445
			.def("translateBone", (void (VRMLTransform::*)(vector3 const& trans))&VRMLTransform::translateBone) // 1445
			.def("transformMesh", (void (VRMLTransform::*)(matrix4 const& m))&VRMLTransform::transformMesh) // 1445
			.def("transformMeshLocal", (void (VRMLTransform::*)(matrix4 const& m))&VRMLTransform::transformMeshLocal) // 1445
			.def("scaleMesh", (void (VRMLTransform::*)( vector3 const& scale))&VRMLTransform::scaleMesh) // 1445
			.def("scaleMass", (void (VRMLTransform::*)( m_real scalef))&VRMLTransform::scaleMass) // 1445
			.def("copyFrom", (void (VRMLTransform::*)(VRMLTransform const& bone))&VRMLTransform::copyFrom) // 1445
			.def("hasShape", (bool (VRMLTransform::*)())&VRMLTransform::hasShape) // 1445
			.def("getMesh", (OBJloader ::Geometry & (VRMLTransform::*)())&VRMLTransform::getMesh,return_value_policy::reference ) // 1450
			.def("numHRPjoints", (int (VRMLTransform::*)())&VRMLTransform::numHRPjoints) // 1445
			.def("HRPjointIndex", (int (VRMLTransform::*)(int i))&VRMLTransform::HRPjointIndex) // 1445
			.def("DOFindex", (int (VRMLTransform::*)(int i))&VRMLTransform::DOFindex) // 1445
			.def("HRPjointName", [](VRMLTransform& t,int i)->std::string{ return t.HRPjointName(i).tostring();}) // 1445
			.def("HRPjointType", (int (VRMLTransform::*)(int i))&VRMLTransform::HRPjointType) // 1445
			.def("HRPjointAxis", [](VRMLTransform& t,int i)->std::string{ return t.HRPjointAxis(i).tostring();}) // 1445
			.def("lastHRPjointIndex", (int (VRMLTransform::*)())&VRMLTransform::lastHRPjointIndex) // 1445
			.def("localCOM", (vector3 (VRMLTransform::*)())&VRMLTransform::localCOM) // 1445
			.def("setLocalCOM", (void (VRMLTransform::*)(vector3 const& com))&VRMLTransform::setLocalCOM) // 1445
			.def("mass", (double (VRMLTransform::*)())&VRMLTransform::mass) // 1445
			.def("inertia", (vector3 (VRMLTransform::*)())&VRMLTransform::inertia) // 1445
			.def("setInertia", (void (VRMLTransform::*)(double ix, double iy, double iz))&VRMLTransform::setInertia) // 1445
			.def("setMass", (void (VRMLTransform::*)(double m))&VRMLTransform::setMass) // 1445
			.def("jointToBody", (void (VRMLTransform::*)(vector3& lposInOut))&VRMLTransform::jointToBody) // 1445
			.def("bodyToJoint", (void (VRMLTransform::*)(vector3& lposInOut))&VRMLTransform::bodyToJoint) // 1445
			; // end of class impl___pybindgen__MainLib_VRMLTransform     // 1581
	}

	// PLDPrimSkin
	{		
		class_<AnimationObject>(mainlib, "AnimationObject")
			;

		class_<ObjectList > (mainlib, "ObjectList")                     // 1389
			.def(init<>())                                                // 1426
			.def("clear", &ObjectList::clear)     // 1445
			.def("setVisible", &ObjectList::setVisible) // 1445
			.def("drawSphere", &ObjectList::drawSphere) // 1445
			.def("drawSphere", [](ObjectList& o,vector3 const& pos, const char* nameid){ o.drawSphere(pos, nameid);})
			.def("drawAxes", &ObjectList::drawAxes) // 1445
			.def("drawAxes", [](ObjectList& o,transf const& tf, const char* nameid){ o.drawAxes(tf, nameid);})
			.def("registerEntity", (Ogre ::SceneNode * (ObjectList::*)(const char* node_name, const char* filename))&ObjectList::registerEntity,return_value_policy::reference ) // 1450
			.def("registerEntity", (Ogre ::SceneNode * (ObjectList::*)(const char* node_name, const char* filename, const char* materialName))&ObjectList::registerEntity,return_value_policy::reference ) // 1450
			.def("registerEntity", (Ogre ::SceneNode * (ObjectList::*)(const char* node_name, Ogre::Item* pObject))&ObjectList::registerEntity,return_value_policy::reference ) // 1450
			.def("registerObject", (Ogre ::SceneNode * (ObjectList::*)(const char* node_name, Ogre::MovableObject* pObject))&ObjectList::registerObject,return_value_policy::reference ) // 1450
			.def("registerObject", (Ogre ::SceneNode * (ObjectList::*)(const char* node_name, const char* typeName, const char* materialName, matrixn const& data, m_real thickness))&ObjectList::registerObject,return_value_policy::reference ) // 1450
			.def("registerObject", [](ObjectList& s, const char* node_name, const char* typeName, const char* materialName, matrixn const& data){ s.registerObject( node_name, typeName, materialName, data);},return_value_policy::reference ) // 1450
			.def("registerEntityScheduled", &ObjectList::registerEntityScheduled,return_value_policy::reference ) // 1450
			.def("registerObjectScheduled", (Ogre ::SceneNode * (ObjectList::*)(Ogre::MovableObject* pObject, m_real destroyTime))&ObjectList::registerObjectScheduled,return_value_policy::reference ) // 1450
			.def("registerObjectScheduled", (Ogre ::SceneNode * (ObjectList::*)(m_real destroyTime, const char* typeName, const char* materialName, matrixn const& data, m_real thickness))&ObjectList::registerObjectScheduled,return_value_policy::reference ) // 1450
			.def("findNode", &ObjectList::findNode,return_value_policy::reference ) // 1450
			.def("erase", &ObjectList::erase) // 1445
			.def("eraseAllScheduled", &ObjectList::eraseAllScheduled) // 1445
			.def("createSceneNode", &ObjectList::createSceneNode,return_value_policy::reference ) // 1450
			.def("getCurrRootSceneNode", &ObjectList::getCurrRootSceneNode,return_value_policy::reference ) // 1450
			; // end of class impl___pybindgen__Ogre_ObjectList           // 1505
		class_<PLDPrimSkin , AnimationObject>(mainlib, "PLDPrimSkin")
			.def_property("visible", &PLDPrimSkin::GetVisible, &PLDPrimSkin::SetVisible)
			.def("setTranslation",&PLDPrimSkin::SetTranslation)
			.def("setPose", &PLDPrimSkin::SetPose)
			.def("applyAnim", &PLDPrimSkin::ApplyAnim)
			.def("getState", (const BoneForwardKinematics & (PLDPrimSkin::*)())&PLDPrimSkin::getState,return_value_policy::reference ) // 1451
			.def("updateBoneLength", (void (PLDPrimSkin::*)(MotionLoader const& loader))&PLDPrimSkin::updateBoneLength) // 1446
			.def("setPose", (void (PLDPrimSkin::*)(const Posture & posture))&PLDPrimSkin::setPose)  // 1446
			.def("setPoseDOF", (void (PLDPrimSkin::*)(const vectorn& poseDOF))&PLDPrimSkin::setPoseDOF) // 1446
			.def("setPose", (void (PLDPrimSkin::*)(const Posture & posture, const MotionLoader& skeleton))&PLDPrimSkin::SetPose) // 1447
			.def("_setPose", (void (PLDPrimSkin::*)(const Posture & posture, const MotionLoader& skeleton))&PLDPrimSkin::SetPose) // 1447
			.def("_setPoseDOF", (void (PLDPrimSkin::*)(const vectorn& poseDOF, MotionDOFinfo const& info))&PLDPrimSkin::setPoseDOF) // 1447
			.def("setSamePose", (void (PLDPrimSkin::*)(BoneForwardKinematics  const& in))&PLDPrimSkin::setSamePose) // 1447
			.def("getVisible", (bool (PLDPrimSkin::*)())&PLDPrimSkin::GetVisible) // 1447
			.def("setVisible", (void (PLDPrimSkin::*)(bool bVisible))&PLDPrimSkin::SetVisible) // 1447
			.def("applyAnim", (void (PLDPrimSkin::*)(const Motion& mot))&PLDPrimSkin::ApplyAnim) // 1447
			.def("applyMotionDOF", (void (PLDPrimSkin::*)(const MotionDOF& motion))&PLDPrimSkin::applyAnim) // 1447
			.def("setThickness", (void (PLDPrimSkin::*)(float thick))&PLDPrimSkin::setThickness) // 1447
			.def("scale", (void (PLDPrimSkin::*)(double x, double y, double z))&PLDPrimSkin::scale) // 1447
			.def("setScale", (void (PLDPrimSkin::*)(double s))&PLDPrimSkin::setScale) // 1447
			.def("setScale", (void (PLDPrimSkin::*)(double x, double y, double z))&PLDPrimSkin::setScale) // 1447
			.def("setScale", (void (PLDPrimSkin::*)(vector3 const& x))&PLDPrimSkin::setScale) // 1447
			.def("setTranslation", (void (PLDPrimSkin::*)(float x, float y, float z))&PLDPrimSkin::setTranslation) // 1447
			.def("setTranslation", (void (PLDPrimSkin::*)(vector3 const& x))&PLDPrimSkin::setTranslation) // 1447
			.def("getTranslation", (vector3 const & (PLDPrimSkin::*)())&PLDPrimSkin::getTranslation,return_value_policy::reference ) // 1452
			.def("setMaterial", (void (PLDPrimSkin::*)(const char* mat))&PLDPrimSkin::setMaterial) // 1447
			.def("loop", (void (*)(PLDPrimSkin& s, bool b))&impl_luna__interface_PLDPrimSkin
::loop)         // 1460
			.def("startAnim", (void (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::startAnim)       // 1460
			.def("initAnim", (void (*)(PLDPrimSkin& s, float curframe, float endframe, float returnframe))&impl_luna__interface_PLDPrimSkin::initAnim) // 1460
			.def("stopAnim", (void (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::stopAnim)         // 1460
			.def("setRotation", (void (*)(PLDPrimSkin& s,quater const& q))&impl_luna__interface_PLDPrimSkin::setRotation) // 1460
			.def("getRotation", (quater (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::getRotation) // 1460
			.def("isPlaying", (bool (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::isPlaying)       // 1460
			.def("calcCurFrameFromInterpolator", (float (*)(PLDPrimSkin& s, int iframe))&impl_luna__interface_PLDPrimSkin::calcCurFrameFromInterpolator) // 1460
			.def("numFrames", (int (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::numFrames)        // 1460
			.def("setFrameTime", (void (*)(PLDPrimSkin& s, float fFrameTime))&impl_luna__interface_PLDPrimSkin::setFrameTime) // 1460
			.def("totalTime", (float (*)(PLDPrimSkin& s))&impl_luna__interface_PLDPrimSkin::totalTime)      // 1460 ;
		;
		struct __pybind_gen_PLDPrimVRML_wrapper{
		static void setPose(PLDPrimVRML& prim, OpenHRP::DynamicsSimulator& s, int ichara)
		{
			/*
			   static Posture pose;
			   OpenHRP::DynamicsSimulator::Character* c=s._characters[ichara];
			   c->chain->getPoseFromLocal(pose);
			//            printf("y=%s \n", c->chain->local(5).translation.output().ptr());
			//            printf("y=%s \n", c->chain->global(5).translation.output().ptr());
			prim.SetPose(pose, *c->skeleton);
			*/
			prim.setSamePose(s.getWorldState(ichara));
		}
		static void setPose2(PLDPrimVRML& prim, Posture const& pose)
		{
			prim.setPose(pose);
		}
		};
		class_<PLDPrimVRML, PLDPrimSkin>(mainlib, "PLDPrimVRML")
			.def("setPose", &__pybind_gen_PLDPrimVRML_wrapper::setPose)
			.def("setPose", &__pybind_gen_PLDPrimVRML_wrapper::setPose2)
			.def("setBoneMaterial", [](PLDPrimVRML& self, int i, const char* name){ self.setMaterial(i, name);})
			;
	}



	
	// FlLayout
	{
		Fl_Widget* (FlLayout::*create1)(const char* type, const char* id, const char* title)=&FlLayout::create;
		Fl_Widget* (FlLayout::*create2)(const char* type, const char* id, const char* title, int startSlot, int endSlot, int height)=&FlLayout::create;
		class_<FlLayout >(mainlib, "FlLayout")
			.def("create", create1,RETURN_REFERENCE)
			.def("create", create2,RETURN_REFERENCE)
			.def("create", [](FlLayout&self, const char* type, const char* id){
					self.create(type, id);
					},RETURN_REFERENCE)
			.def("create", [](FlLayout&self, const char* type, const char* id, const char* title, int startSlot){
					self.create(type, id, title, startSlot);
					},RETURN_REFERENCE)
			.def("create", [](FlLayout&self, const char* type, const char* id, const char* title, int startSlot, int endSlot){
					self.create(type, id, title, startSlot, endSlot);
					},RETURN_REFERENCE)
			.def("newLine", &FlLayout::newLine)
			.def("setLineSpace", &FlLayout::setWidgetHeight)
			.def("setWidgetPos", &FlLayout::setWidgetPos)
			.def("setUniformGuidelines", &FlLayout::setUniformGuidelines)
			.def("setWidgetHeight", &FlLayout::setWidgetHeight)
			.def("addButton",[](FlLayout& o, const char* title){ o.create("Button", title, title); })
			.def("addButton",[](FlLayout& o, const char* title, const char* on_screen_title){ o.create("Button", title, on_screen_title); })
			.def("addCheckButton",[](FlLayout& o, const char* title, bool value){ 
					o.create("Check_Button", title, title); 
					o.widgetRaw(0).checkButton()->value(value);

					})
			.def("updateLayout", &FlLayout::updateLayout)
			.def("resetToDefault", (void (FlLayout::*)())&FlLayout::resetToDefault) // 1448
			.def("redraw", &FlLayout::redraw)
			.def("minimumHeight", &FlLayout::minimumHeight)
			.def("widget", &FlLayout::widgetRaw,RETURN_REFERENCE)
			.def("widgetIndex", &FlLayout::widgetIndex)
			.def("findWidget", &FlLayout::findWidget,RETURN_REFERENCE)	
			.def("removeWidgets", &FlLayout::removeWidgets)
		;
		class_<Fl_Widget >(mainlib, "Fl_Widget") ;


		class_<FlLayout::Widget>(mainlib, "Widget")
			.def("progressValue", (float (FlLayout::Widget::*)() const)&FlLayout::Widget::progressValue)
			.def("progressValue", (void (FlLayout::Widget::*)(float))&FlLayout::Widget::progressValue)
			.def("checkButtonValue", &WidgetWrapper::checkButtonValue)
			.def("checkButtonValue", &WidgetWrapper::checkButtonValue2)
			.def("checkButtonValue", &WidgetWrapper::checkButtonValue3)
			.def("menuSize", &WidgetWrapper::menuSize)
			.def("menuItem", &WidgetWrapper::menuItem)
			.def("menuItem", &WidgetWrapper::menuItem2)
			.def("menuValue", &WidgetWrapper::menuValue)
			.def("menuText",&WidgetWrapper::menuText)
			.def("menuText",&WidgetWrapper::menuText2)
			.def("menuValue",&WidgetWrapper::menuValue2)
			.def("sliderValue",&WidgetWrapper::sliderValue)
			.def("sliderStep",&WidgetWrapper::sliderStep)
			.def("sliderRange",&WidgetWrapper::sliderRange)
			.def("sliderValue",&WidgetWrapper::sliderValue2)
			.def("buttonShortcut",&WidgetWrapper::buttonShortcut)
			.def("buttonTooltip",&WidgetWrapper::buttonTooltip)
			.def("buttonSetLabel",&WidgetWrapper::buttonSetLabel)
			.def("copyLabel",&WidgetWrapper::buttonSetLabel)
			.def("buttonLabel",&WidgetWrapper::buttonLabel)
			.def("label",&WidgetWrapper::buttonLabel)
			.def("redraw",&WidgetWrapper::redraw)
			.def("setVisible",&WidgetWrapper::setVisible)
			.def("userData",&WidgetWrapper::userData)
			.def("userData",&WidgetWrapper::userData2)
			.def("clearVisible",&WidgetWrapper::clearVisible)
			.def("deactivate",&WidgetWrapper::deactivate)
			.def("activate",&WidgetWrapper::activate)
			.def("id",&WidgetWrapper::id)
			.def("browserSize",&WidgetWrapper::browserSize)
			.def("browserSelected",&WidgetWrapper::browserSelected)
			.def("browserDeselect",&WidgetWrapper::browserDeselect)
			.def("browserSelect",&WidgetWrapper::browserSelect)
			.def("browserAdd",&WidgetWrapper::browserAdd)
			.def("browserRemove",&WidgetWrapper::browserRemove)
			.def("browserClear",&WidgetWrapper::browserClear)
			.def("browserValue",&WidgetWrapper::browserValue)
			.def("browserText",&WidgetWrapper::browserText)
			.def("inputValue",&WidgetWrapper::inputValue1)
			.def("inputValue",&WidgetWrapper::inputValue2)
			.def("inputType",&WidgetWrapper::inputType)
			
		;

	}
	enum_<NonuniformSpline::boundaryCondition::bcT>(mainlib, "boundaryCondition")
		.value("ZERO_VEL",NonuniformSpline::boundaryCondition::ZERO_VEL)
		.value("ZERO_ACC",NonuniformSpline::boundaryCondition::ZERO_ACC)
		.value("VEL",NonuniformSpline::boundaryCondition::VEL)
		;
	class_<NonuniformSpline > (mainlib, "NonuniformSpline")         // 1389
	.def(init<vectorn const &,const matrixn &>())                 // 1426
.def(init<vectorn const &,const matrixn &,NonuniformSpline ::boundaryCondition ::bcT>()) // 1426
	.def("getCurve", (void (NonuniformSpline::*)(vectorn const& time, matrixn& points))&NonuniformSpline::getCurve) // 1445
	.def("getFirstDeriv", (void (NonuniformSpline::*)(vectorn const& time, matrixn& points))&NonuniformSpline::getFirstDeriv) // 1445
	.def("getSecondDeriv", (void (NonuniformSpline::*)(vectorn const& time, matrixn& points))&NonuniformSpline::getSecondDeriv) // 1445
	; // end of class impl___pybindgen__math_NonuniformSpline     // 1505
	class_<SkinnedMeshFromVertexInfo > (mainlib, "SkinnedMeshFromVertexInfo") // 1389
																		// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<>())                                                // 1426
	.def(init<const char *>())                                    // 1426
.def(init<SkinnedMeshFromVertexInfo const &>())               // 1426
	.def("exportSkinInfo", (void (SkinnedMeshFromVertexInfo::*)(const char* filename))&SkinnedMeshFromVertexInfo::exportSkinInfo) // 1445
	.def("calcSurfacePointPosition", (vector3 (SkinnedMeshFromVertexInfo::*)( MotionLoader const& loader, intvectorn const& treeIndices, vectorn const& weights, vector3N const& localpos))&SkinnedMeshFromVertexInfo::calcSurfacePointPosition) // 1445
	.def("calcVertexPosition", (vector3 (SkinnedMeshFromVertexInfo::*)( MotionLoader const& loader, int vertexIndex))&SkinnedMeshFromVertexInfo::calcVertexPosition) // 1445
	.def("_calcVertexPosition", (void (SkinnedMeshFromVertexInfo::*)( MotionLoader const& loader, int vertexIndex, vector3& out))&SkinnedMeshFromVertexInfo::_calcVertexPosition) // 1445
	.def("calcVertexPositions", (void (SkinnedMeshFromVertexInfo::*)(MotionLoader const& loader, OBJloader::Mesh& mesh)const)&SkinnedMeshFromVertexInfo::calcVertexPositions) // 1445
	.def("calcVertexPositions", (void (SkinnedMeshFromVertexInfo::*)(BoneForwardKinematics const& fkSolver, OBJloader::Mesh& mesh)const)&SkinnedMeshFromVertexInfo::calcVertexPositions) // 1445
	.def("calcVertexNormals", (void (SkinnedMeshFromVertexInfo::*)(MotionLoader const& loader,quaterN const& bindpose_global, vector3N const& localNormal, OBJloader::Mesh& mesh) const)&SkinnedMeshFromVertexInfo::calcVertexNormals) // 1445
	.def("calcVertexNormals", (void (SkinnedMeshFromVertexInfo::*)(BoneForwardKinematics const& fkSolver, quaterN const& bindpose_global, vector3N const& local_normal, OBJloader::Mesh& mesh) const)&SkinnedMeshFromVertexInfo::calcVertexNormals) // 1445
	.def("calcLocalVertexPositions", (void (SkinnedMeshFromVertexInfo::*)(MotionLoader const& loader, OBJloader::Mesh const& mesh))&SkinnedMeshFromVertexInfo::calcLocalVertexPositions) // 1445
	.def("resize", (void (SkinnedMeshFromVertexInfo::*)(int numVertex))&SkinnedMeshFromVertexInfo::resize) // 1445
	.def("treeIndices", (intvectorn & (SkinnedMeshFromVertexInfo::*)(int vertexIndex))&SkinnedMeshFromVertexInfo::treeIndices,return_value_policy::reference ) // 1450
	.def("localPos", (vector3N & (SkinnedMeshFromVertexInfo::*)(int vertexIndex))&SkinnedMeshFromVertexInfo::localPos,return_value_policy::reference ) // 1450
	.def("weights", (vectorn & (SkinnedMeshFromVertexInfo::*)(int vertexIndex))&SkinnedMeshFromVertexInfo::weights,return_value_policy::reference ) // 1450
	.def("getVertexInfo", (void (SkinnedMeshFromVertexInfo::*)(int v1, int v2, int v3, vector3 const& baryCoeffs, intvectorn& treeIndices, vector3N& localpos, vectorn &weights))&SkinnedMeshFromVertexInfo::getVertexInfo) // 1445
	.def("numVertex", (int (SkinnedMeshFromVertexInfo::*)())&SkinnedMeshFromVertexInfo::numVertex) // 1445
	; // end of class impl___pybindgen___SkinnedMeshFromVertexInfo // 1505

	enum_<handle_message>(mainlib, "handle")
		.value("FRAME_MOVE", M_FRAME_MOVE)
		.value("TRIGGERED", M_TRIGGERED)
		.value("ON_DRAW",M_ON_DRAW)
		.value("CALLBACK", M_CALLBACK)
		.value("HANDLE", M_HANDLE)
	;

	enum_<Fl_Event>(mainlib, "event")
		.value("FL_NO_EVENT",FL_NO_EVENT)
		.value("FL_PUSH",FL_PUSH)
  		.value("FL_RELEASE",FL_RELEASE		)
  		.value("FL_ENTER",FL_ENTER		)
  		.value("FL_LEAVE",FL_LEAVE		)
  		.value("FL_DRAG",FL_DRAG		)
  		.value("FL_FOCUS",FL_FOCUS	)	
  		.value("FL_UNFOCUS",FL_UNFOCUS)		
  		.value("FL_KEYDOWN",FL_KEYDOWN)		
  		.value("FL_KEYUP",FL_KEYUP	)	
  		.value("FL_CLOSE",FL_CLOSE	)	
  		.value("FL_MOVE",FL_MOVE		)
  		.value("FL_SHORTCUT",FL_SHORTCUT)		
  		.value("FL_DEACTIVATE",FL_DEACTIVATE		)
  		.value("FL_ACTIVATE",FL_ACTIVATE		)
  		.value("FL_HIDE",FL_HIDE		)
  		.value("FL_SHOW",FL_SHOW		)
  		.value("FL_PASTE",FL_PASTE	)	
  		.value("FL_SELECTIONCLEAR",FL_SELECTIONCLEAR	)
  		.value("FL_MOUSEWHEEL",FL_MOUSEWHEEL		)
  		.value("FL_DND_ENTER",FL_DND_ENTER	)	
  		.value("FL_DND_DRAG",FL_DND_DRAG		)
  		.value("FL_DND_LEAVE",FL_DND_LEAVE	)	
  		.value("FL_DND_RELEASE",FL_DND_RELEASE	)
	;
	{
		class_<lunaStack >(mainlib, "lunaStack")
			;
	}
{
	class_<LuaScript > (mainlib, "LuaScript")                 // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("luaType", &LuaScript::luaType)                          // 1445
																	  //when necessary, check c++ header: .def("luaType", (int (LuaScript::*)(int i))&LuaScript::luaType) // 1446
		.def("lunaType", &LuaScript::lunaType)                        // 1445
																	  //when necessary, check c++ header: .def("lunaType", (std ::string (LuaScript::*)(int i))&LuaScript::lunaType) // 1446
		.def("next", &LuaScript::next)                                // 1445
																	  //when necessary, check c++ header: .def("next", (bool (LuaScript::*)( int index))&LuaScript::next) // 1446
		.def("pushvalue", &LuaScript::pushvalue)                      // 1445
																	  //when necessary, check c++ header: .def("pushvalue", (void (LuaScript::*)(int index))&LuaScript::pushvalue) // 1446
		.def("pushnil", &LuaScript::pushnil)                          // 1445
																	  //when necessary, check c++ header: .def("pushnil", (void (LuaScript::*)())&LuaScript::pushnil) // 1446
		.def("newtable", &LuaScript::newtable)                        // 1445
																	  //when necessary, check c++ header: .def("newtable", (void (LuaScript::*)())&LuaScript::newtable) // 1446
		.def("settable", &LuaScript::settable)                        // 1445
																	  //when necessary, check c++ header: .def("settable", (void (LuaScript::*)(int index))&LuaScript::settable) // 1446
		.def("getglobal", (void (LuaScript::*)(const char* key))&LuaScript::getglobal) // 1446
		.def("_setglobal", &LuaScript::_setglobal)                    // 1445
																	  //when necessary, check c++ header: .def("_setglobal", (void (LuaScript::*)(const char* key))&LuaScript::_setglobal) // 1446
		.def("getglobalNoCheck", (void (LuaScript::*)(const char* key))&LuaScript::getglobalNoCheck) // 1446
		.def("replaceTop", (void (LuaScript::*)(const char* key))&LuaScript::replaceTop) // 1446
		.def("insert", &LuaScript::insert)                            // 1445
																	  //when necessary, check c++ header: .def("insert", (void (LuaScript::*)(int index))&LuaScript::insert) // 1446
		.def("replaceTop", (void (LuaScript::*)(int index))&LuaScript::replaceTop) // 1446
		.def("getglobal", (void (LuaScript::*)(const char* key1, const char* key2))&LuaScript::getglobal) // 1446
		.def("getglobalNoCheck", (void (LuaScript::*)(const char* key1, const char* key2))&LuaScript::getglobalNoCheck) // 1446
		.def("getMemberFunc", &LuaScript::getMemberFunc)              // 1445
																	  //when necessary, check c++ header: .def("getMemberFunc", (void (LuaScript::*)( const char* name))&LuaScript::getMemberFunc) // 1446
		.def("releaseScript", &LuaScript::releaseScript)              // 1445
																	  //when necessary, check c++ header: .def("releaseScript", (void (LuaScript::*)())&LuaScript::releaseScript) // 1446
		.def("initLuaEnvironment", &LuaScript::initLuaEnvironment)    // 1445
																	  //when necessary, check c++ header: .def("initLuaEnvironment", (void (LuaScript::*)())&LuaScript::initLuaEnvironment) // 1446
		.def("loadScript", &LuaScript::loadScript)                    // 1445
																	  //when necessary, check c++ header: .def("loadScript", (void (LuaScript::*)(const char* script))&LuaScript::loadScript) // 1446
		.def("loadScript", &LuaScript::loadScript)                    // 1445
																	  //when necessary, check c++ header: .def("loadScript", (void (LuaScript::*)(const char* script, const char* scriptstring))&LuaScript::loadScript) // 1446
		.def("dostring", &LuaScript::dostring)                        // 1445
																	  //when necessary, check c++ header: .def("dostring", (void (LuaScript::*)(const char* str))&LuaScript::dostring) // 1446
		.def("dofile", &LuaScript::dofile)                            // 1445
																	  //when necessary, check c++ header: .def("dofile", (void (LuaScript::*)(const char* str))&LuaScript::dofile) // 1446
		.def("call", (void (LuaScript::*)(int numIn, int numOut))&LuaScript::call) // 1446
		.def("call", (void (LuaScript::*)(int numIn))&LuaScript::call) // 1446
		.def("popmatrixn", (matrixn * (LuaScript::*)())&LuaScript::popmatrixn,return_value_policy::reference ) // 1451
		.def("popMotionDOF", (MotionDOF * (LuaScript::*)())&LuaScript::popMotionDOF,return_value_policy::reference ) // 1451
		.def("popVRMLloader", (VRMLloader * (LuaScript::*)())&LuaScript::popVRMLloader,return_value_policy::reference ) // 1451
		.def("pophypermatrixn", (hypermatrixn * (LuaScript::*)())&LuaScript::pophypermatrixn,return_value_policy::reference ) // 1451
		.def("popTensor", (Tensor * (LuaScript::*)())&LuaScript::popTensor,return_value_policy::reference ) // 1451
		.def("popvectorn", (vectorn * (LuaScript::*)())&LuaScript::popvectorn,return_value_policy::reference ) // 1451
		.def("popintvectorn", (intvectorn * (LuaScript::*)())&LuaScript::popintvectorn,return_value_policy::reference ) // 1451
		.def("checkmatrixn", (matrixn * (LuaScript::*)())&LuaScript::checkmatrixn,return_value_policy::reference ) // 1451
		.def("checkhypermatrixn", (hypermatrixn * (LuaScript::*)())&LuaScript::checkhypermatrixn,return_value_policy::reference ) // 1451
		.def("checkTensor", (Tensor * (LuaScript::*)())&LuaScript::checkTensor,return_value_policy::reference ) // 1451
		.def("checkvectorn", (vectorn * (LuaScript::*)())&LuaScript::checkvectorn,return_value_policy::reference ) // 1451
		.def("checkintvectorn", (intvectorn * (LuaScript::*)())&LuaScript::checkintvectorn,return_value_policy::reference ) // 1451
		.def("popnumber", &LuaScript::popnumber)                      // 1445
																	  //when necessary, check c++ header: .def("popnumber", (double (LuaScript::*)())&LuaScript::popnumber) // 1446
		.def("popstring", &LuaScript::popstring)                      // 1445
																	  //when necessary, check c++ header: .def("popstring", (std ::string (LuaScript::*)())&LuaScript::popstring) // 1446
		.def("popboolean", &LuaScript::popboolean)                    // 1445
																	  //when necessary, check c++ header: .def("popboolean", (bool (LuaScript::*)())&LuaScript::popboolean) // 1446
		.def("popint", &LuaScript::popint)                            // 1445
																	  //when necessary, check c++ header: .def("popint", (int (LuaScript::*)())&LuaScript::popint) // 1446
		.def("isnil", &LuaScript::isnil)                              // 1445
																	  //when necessary, check c++ header: .def("isnil", (bool (LuaScript::*)(int i))&LuaScript::isnil) // 1446
		.def("isLuaReady", &LuaScript::isLuaReady)                    // 1445
																	  //when necessary, check c++ header: .def("isLuaReady", (bool (LuaScript::*)())&LuaScript::isLuaReady) // 1446
		.def("gettop", &LuaScript::gettop)                            // 1445
																	  //when necessary, check c++ header: .def("gettop", (int (LuaScript::*)())&LuaScript::gettop) // 1446
		.def("getPreviousTop", &LuaScript::getPreviousTop)            // 1445
																	  //when necessary, check c++ header: .def("getPreviousTop", (int (LuaScript::*)())&LuaScript::getPreviousTop) // 1446
		.def("saveCurrentTop", &LuaScript::saveCurrentTop)            // 1445
																	  //when necessary, check c++ header: .def("saveCurrentTop", (void (LuaScript::*)())&LuaScript::saveCurrentTop) // 1446
		.def("pop", &LuaScript::pop)                                  // 1445
																	  //when necessary, check c++ header: .def("pop", (void (LuaScript::*)())&LuaScript::pop) // 1446
		.def("set", &LuaScript::set)                                  // 1445
																	  //when necessary, check c++ header: .def("set", (void (LuaScript::*)(std::string const& key))&LuaScript::set) // 1446
		.def("push", (void (LuaScript::*)(FlLayout::Widget & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(boolN & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)( double a))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)( bool a))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)( std::string const &a))&LuaScript::push) // 1446
		.def("printStack", &LuaScript::printStack)                    // 1445
																	  //when necessary, check c++ header: .def("printStack", (void (LuaScript::*)())&LuaScript::printStack) // 1446
		.def("push", (void (LuaScript::*)(vector3 & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(quater & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(transf & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(matrix4 & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(quaterN & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(vector3N & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(intvectorn & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(vectorn & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(VRMLloader & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(MotionDOF & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(MotionLoader & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(Bone & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(PLDPrimSkin & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(FlLayout & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(matrixn & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(hypermatrixn & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(Tensor & w))&LuaScript::push) // 1446
		.def("push", (void (LuaScript::*)(Posture & w))&LuaScript::push) // 1446
		; // end of class impl___pybindgen___LuaScript                // 1506
	class_<ThreadedScript, LuaScript > (mainlib, "ThreadedScript")       // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("threadedCall", &ThreadedScript::threadedCall)           // 1445
																	  //when necessary, check c++ header: .def("threadedCall", (void (ThreadedScript::*)(int numIn))&ThreadedScript::threadedCall) // 1446
		.def("waitUntilFinished", &ThreadedScript::waitUntilFinished) // 1445
		.def("stop", &ThreadedScript::stop)
																	  //
																	  //when necessary, check c++ header: .def("waitUntilFinished", (void (ThreadedScript::*)())&ThreadedScript::waitUntilFinished) // 1446
		; // end of class impl___pybindgen___ThreadedScript           // 1506
	class_<ThreadScriptPool > (mainlib, "ThreadScriptPool")   // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<int>())                                             // 1426
		.def("queueJob", &ThreadScriptPool::queueJob)                 // 1445
																	  //when necessary, check c++ header: .def("queueJob", (void (ThreadScriptPool::*)(const char* job))&ThreadScriptPool::queueJob) // 1446
		.def("start", &ThreadScriptPool::start)                       // 1445
																	  //when necessary, check c++ header: .def("start", (void (ThreadScriptPool::*)())&ThreadScriptPool::start) // 1446
		.def("stop", &ThreadScriptPool::stop)                         // 1445
																	  //when necessary, check c++ header: .def("stop", (void (ThreadScriptPool::*)())&ThreadScriptPool::stop) // 1446
		.def("busy", &ThreadScriptPool::busy)                         // 1445
																	  //when necessary, check c++ header: .def("busy", (bool (ThreadScriptPool::*)())&ThreadScriptPool::busy) // 1446
		.def("numThreads", &ThreadScriptPool::numThreads)             // 1445
																	  //when necessary, check c++ header: .def("numThreads", (int (ThreadScriptPool::*)())&ThreadScriptPool::numThreads) // 1446
		.def("env", (LuaScript * (ThreadScriptPool::*)(int i))&ThreadScriptPool::env,return_value_policy::reference ) // 1451
		; // end of class impl___pybindgen___ThreadScriptPool         // 1506
	}
	{
		struct PythonExtendWin_wrapper
		{
			static void getglobal(PythonExtendWin& l, const char* key){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				lua_pushstring(l.L, key);
				lua_gettable(l.L,LUA_GLOBALSINDEX); // stack top becomes _G[key] 
				if (lua_isnil(l.L, -1)) throw std::runtime_error("missing global: "+std::string(key));
			}
			static void getglobalNoCheck(PythonExtendWin& l, const char* key){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				lua_pushstring(l.L, key);
				lua_gettable(l.L,LUA_GLOBALSINDEX); // stack top becomes _G[key] 
			}
			static void replaceTop(PythonExtendWin& l, const char* key){
				lua_State *L=l.L;
				if (!lua_istable(L,-1)) throw std::runtime_error("replaceTop: non-table access: "+std::string(key));
				lua_pushstring(L, key);
				lua_gettable(L, -2);
				lua_insert(L, -2);  // swap table and value 
				lua_pop(L,1); // pop-out prev table
			}
			static void insert(PythonExtendWin& l, int index)
			{
				lua_State *L=l.L;
				lua_insert(L,index);
			}
			static void replaceTop(PythonExtendWin& l, int index){
				lua_State *L=l.L;
				if (!lua_istable(L,-1)) throw std::runtime_error("replaceTop: non-table indexing ");
				lua_pushnumber(L, index);
				lua_gettable(L, -2);
				lua_insert(L, -2);  // swap table and value 
				lua_pop(L,1); // pop-out prev table
			}
			static void getglobal2(PythonExtendWin& l, const char* key1, const char* key2){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				getglobal(l, key1);
				replaceTop(l, key2);
				if (lua_isnil(l.L, -1)) throw std::runtime_error("missing global: "+std::string(key1)+std::string(key2));
			}
			static void getglobal2NoCheck(PythonExtendWin& l, const char* key1, const char* key2){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				getglobal(l, key1);
				replaceTop(l, key2);
			}
			static void getMemberFunc(PythonExtendWin&l, const char* name){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				lua_State *L=l.L;
				Msg::verify(L, "PythonExtendWin::L is NULL");
				if (!lua_isuserdata(L,-1)) luaL_error(l.L,"stack top is not a userdata");
				lua_getmetatable(L, -1);
				replaceTop(l, name);
				lua_insert(L,-2); // swap the member function and the self object.
			}
			// done: reference로 받음. see RETURN_REFERENCE		  
		 static matrixn* popmatrixn(PythonExtendWin& l)
		  {
		    matrixn* result= (matrixn*)Luna<typename LunaTraits<matrixn>::base_t>::check(l.L,-1);
		    lua_pop(l.L,1);
		    return result;
		  }
		  static hypermatrixn* pophypermatrixn(PythonExtendWin& l)
		  {
		    hypermatrixn* result= (hypermatrixn*)Luna<typename LunaTraits<hypermatrixn>::base_t>::check(l.L,-1);
		    lua_pop(l.L,1);
		    return result;
		  }
		  static Tensor* popTensor(PythonExtendWin& l)
		  {
		    Tensor* result= (Tensor*)Luna<typename LunaTraits<Tensor>::base_t>::check(l.L,-1);
		    lua_pop(l.L,1);
		    return result;
		  }
		  static vectorn* popvectorn(PythonExtendWin& l)
		  {
		    vectorn* result= (vectorn*)Luna<typename LunaTraits<vectorn>::base_t>::check(l.L,-1);
		    lua_pop(l.L,1);
		    return result;
		  }
		  static intvectorn* popintvectorn(PythonExtendWin& l)
		  {
		    intvectorn* result= (intvectorn*)Luna<typename LunaTraits<intvectorn>::base_t>::check(l.L,-1);
		    lua_pop(l.L,1);
		    return result;
		  }
		 static matrixn* checkmatrixn(PythonExtendWin& l)
		  {
		    matrixn* result= (matrixn*)Luna<typename LunaTraits<matrixn>::base_t>::check(l.L,-1);
		    return result;
		  }
		  static hypermatrixn* checkhypermatrixn(PythonExtendWin& l)
		  {
		    hypermatrixn* result= (hypermatrixn*)Luna<typename LunaTraits<hypermatrixn>::base_t>::check(l.L,-1);
		    return result;
		  }
		  static Tensor* checkTensor(PythonExtendWin& l)
		  {
		    Tensor* result= (Tensor*)Luna<typename LunaTraits<Tensor>::base_t>::check(l.L,-1);
		    return result;
		  }
		  static vectorn* checkvectorn(PythonExtendWin& l)
		  {
		    vectorn* result= (vectorn*)Luna<typename LunaTraits<vectorn>::base_t>::check(l.L,-1);
		    return result;
		  }
		  static intvectorn* checkintvectorn(PythonExtendWin& l)
		  {
		    intvectorn* result= (intvectorn*)Luna<typename LunaTraits<intvectorn>::base_t>::check(l.L,-1);
		    return result;
		  }
		  // has to be double.  for int, use popint
		  static double popnumber(PythonExtendWin& l)
		  {
		    double i=lua_tonumber(l.L,-1);
		    lua_pop(l.L,1);
		    return i;
		  }
		  static std::string popstring(PythonExtendWin& l)
		  {
			  std::string i=lua_tostring(l.L,-1);
			  lua_pop(l.L,1);
			  return i;
		  }
		  static bool popboolean(PythonExtendWin& l)
		  {
		    bool i=lua_toboolean(l.L,-1);
		    lua_pop(l.L,1);
		    return i;
		  }
		  static int popint(PythonExtendWin& l)
		  {
		    int i=lua_tonumber(l.L,-1);
		    lua_pop(l.L,1);
		    return i;
		  }
		  static bool isnil(PythonExtendWin& l,int i) { return lua_isnil(l.L, i); }
			static bool isLuaReady(PythonExtendWin& l) { return l.L;}
			static int gettop(PythonExtendWin& l)
			{
				return lua_gettop(l.L);
			}
			static void pop(PythonExtendWin& l)
			{
			  lua_pop(l.L,1);
		  }
			static void set(PythonExtendWin& l,std::string const& key) 
			{
				// after push
				lua_pushstring(l.L, key.c_str());
				lua_insert(l.L, -2);  // swap value and key
				lua_settable(l.L, LUA_GLOBALSINDEX);
			}
			static void push1(PythonExtendWin& l,FlLayout::Widget & w) { luna_push<FlLayout::Widget>(l.L, &w); }
			static void push2( PythonExtendWin& os, double a)		    	{ lua_pushnumber(os.L, a); }
			static void push22( PythonExtendWin& os, int a)		    	{ lua_pushnumber(os.L, (double)a); }
			static void push3( PythonExtendWin& os, bool a)		    	{ lua_pushboolean(os.L, a); }
			static void push4( PythonExtendWin& os, std::string const &a)	{ lua_pushstring(os.L,a.c_str()); }
			static lunaStack* call(PythonExtendWin& os, int numIn, int numOut)
			{
				lunaStack* pp =new lunaStack(os.L);
				os.luna_call(*pp, numIn, numOut);
				return pp;
			}
			static lunaStack* call2(PythonExtendWin& os, int numIn)
			{
				lunaStack* pp =new lunaStack(os.L);
				os.luna_call(*pp, numIn, LUA_MULTRET);
				return pp;
			}
			
			static void printStack(PythonExtendWin& os)
			{
				luna_printStack(os.L,false);
			}
			static void push_vector3(PythonExtendWin& l,vector3 & w) { luna_push<vector3>(l.L, &w); }
			static void push_quater(PythonExtendWin& l,quater & w) { luna_push<quater>(l.L, &w); }
			static void push_transf(PythonExtendWin& l,transf & w) { luna_push<transf>(l.L, &w); }
			static void push_matrix4(PythonExtendWin& l,matrix4 & w) { luna_push<matrix4>(l.L, &w); }
			static void push_quaterN(PythonExtendWin& l,quaterN & w) { luna_push<quaterN>(l.L, &w); }
			static void push_vector3N(PythonExtendWin& l,vector3N & w) { luna_push<vector3N>(l.L, &w); }
			static void push_intvectorn(PythonExtendWin& l,intvectorn & w) { luna_push<intvectorn>(l.L, &w); }
			static void push_vectorn(PythonExtendWin& l,vectorn & w) { luna_push<vectorn>(l.L, &w); }
			static void push_motionloader(PythonExtendWin& l,MotionLoader & w) { luna_push<MotionLoader>(l.L, &w); }
			static void push_bone(PythonExtendWin& l,Bone & w) { luna_push<Bone>(l.L, &w); }
			static void push_PLDPrimSkin(PythonExtendWin& l,PLDPrimSkin & w) { luna_push<PLDPrimSkin>(l.L, &w); }
			static void push_FlLayout(PythonExtendWin& l,FlLayout & w) { luna_push<FlLayout>(l.L, &w); }
			static void push_matrixn(PythonExtendWin& l,matrixn & w) { luna_push<matrixn>(l.L, &w); }
			static void push_hypermatrixn(PythonExtendWin& l,hypermatrixn & w) { luna_push<hypermatrixn>(l.L, &w); }
			static void push_Tensor(PythonExtendWin& l,Tensor & w) { luna_push<Tensor>(l.L, &w); }
			static void push_posture(PythonExtendWin& l,Posture & w) { luna_push<Posture>(l.L, &w); }
		};
#ifndef NO_GUI
		class_<FltkRenderer > (mainlib, "_FltkRenderer")           // 1393
			.def("screenToWorldXZPlane", (vector3 (FltkRenderer::*)(float x, float y, float height))&FltkRenderer::screenToWorldXZPlane,"x"_a, "y"_a,"height"_a=0.f) // 1451
			//.def("worldToScreen", (vector2 (FltkRenderer::*)(vector3 const& w))&FltkRenderer::worldToScreen) // 1451
			.def("screenToWorldRay", (void (FltkRenderer::*)(float x, float y, Ray& ray))&FltkRenderer::screenToWorldRay) // 1451
			.def("volumeQuery", (void (FltkRenderer::*)(TStrings& nodeNames, float left, float top, float right, float bottom))&FltkRenderer::volumeQuery) // 1451
			.def("rayQuery", (void (FltkRenderer::*)(TStrings& nodeNames, float x, float y))&FltkRenderer::rayQuery) // 1451
			.def("saveView", (void (FltkRenderer::*)(int slot))&FltkRenderer::saveView) // 145j
			;
		class_<OgreRenderer > (mainlib, "OgreRenderer")           // 1393
			.def("getConfig", &OgreRenderer::getConfig)
			.def("getConfigFloat", &OgreRenderer::getConfigFloat)
			.def("screenshot", &OgreRenderer::screenshot)                 // 1450
			.def("setScreenshotMotionBlur", &OgreRenderer::setScreenshotMotionBlur) // 1450
			.def("setScreenshotPrefix", &OgreRenderer::setScreenshotPrefix) // 1450
			.def("fixedTimeStep", &OgreRenderer::fixedTimeStep)           // 1450
			.def("setCaptureFPS", &OgreRenderer::setCaptureFPS)           // 1450
			.def("addNewViewport", &OgreRenderer::addNewViewport)         // 1450
			.def("numViewport", &OgreRenderer::numViewport)               // 1450
			.def("viewport", (OgreRenderer ::Viewport & (OgreRenderer::*)())&OgreRenderer::viewport,return_value_policy::reference ) // 1456
			.def("viewport", (OgreRenderer ::Viewport & (OgreRenderer::*)(int viewport))&OgreRenderer::viewport,return_value_policy::reference ) // 1456
			.def("addFrameMoveObject", &OgreRenderer::addFrameMoveObject) // 1450
			.def("removeFrameMoveObject", &OgreRenderer::removeFrameMoveObject) // 1450
			.def("addAfterFrameMoveObject", &OgreRenderer::addAfterFrameMoveObject) // 1450
			.def("removeAfterFrameMoveObject", &OgreRenderer::removeAfterFrameMoveObject) // 1450
			.def("_getOgreTextureWidth", &OgreRenderer::_getOgreTextureWidth) // 1450
			.def("_updateDynamicTexture", &OgreRenderer::_updateDynamicTexture) // 1450
			.def("_linkMaterialAndTexture", &OgreRenderer::_linkMaterialAndTexture) // 1450
			.def("createDynamicTexture", (void (OgreRenderer::*)(const char* name, CImage const& image))&OgreRenderer::createDynamicTexture) // 1451
			.def("createDynamicTexture", (void (OgreRenderer::*)(const char* name, CImage const& image, vector3 const& diffuseColor, vector3 const& specular_color, double shininess))&OgreRenderer::createDynamicTexture) // 1451
			.def("createMaterial", &OgreRenderer::createMaterial)         // 1450
			.def("createRenderTexture", &OgreRenderer::createRenderTexture) // 1450
			.def("updateRenderTexture", &OgreRenderer::updateRenderTexture) // 1450
			.def("setMaterialParam", &OgreRenderer::setMaterialParam)     // 1450
			.def("cloneMaterial", &OgreRenderer::cloneMaterial)           // 1450
			; // end of class impl___pybindgen___OgreRenderer             // 1511
#endif
																						;
		class_<PythonExtendWin, FlLayout >(mainlib, "PythonExtendWin")
			.def( init<int, int, int, int, MotionPanel&, FltkRenderer&>())
			.def("loadScript", &PythonExtendWin::__loadScript)
			.def("loadEmptyScript", &PythonExtendWin::__loadEmptyScript)
			.def("releaseScript", &ScriptWin::releaseScript)
			.def("dofile", &PythonExtendWin::dofile)
			.def("dostring", &PythonExtendWin::dostring)
			.def("luaType", [](PythonExtendWin&l , int i)->int{ return (int)::lua_type(l.L,i);})
			.def("lunaType", [](PythonExtendWin&l , int i)->std::string{ return ::lunaType(l.L,i);})
			.def("isLuaReady", &PythonExtendWin_wrapper::isLuaReady)
			// derived class should appear first
			.def("push", [](PythonExtendWin& l,Motion & w) { luna_push<Motion>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,MotionDOF & w) { luna_push<MotionDOF>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,VRMLloader & w) { luna_push<VRMLloader>(l.L, &w); })
			.def("push", &PythonExtendWin_wrapper::push1) // FlLayout::Widget
			.def("push", &PythonExtendWin_wrapper::push2) // double
			.def("push", &PythonExtendWin_wrapper::push22) // int
			.def("pushBoolean", &PythonExtendWin_wrapper::push3) // bool
			.def("push", &PythonExtendWin_wrapper::push4) // std::string
			.def("push", &PythonExtendWin_wrapper::push_vector3)
			.def("push", &PythonExtendWin_wrapper::push_quater)
			.def("push", &PythonExtendWin_wrapper::push_transf)
			.def("push", &PythonExtendWin_wrapper::push_matrix4)
			.def("push", &PythonExtendWin_wrapper::push_quaterN)
			.def("push", &PythonExtendWin_wrapper::push_vector3N)
			.def("push", &PythonExtendWin_wrapper::push_intvectorn)
			.def("push", &PythonExtendWin_wrapper::push_vectorn)
			.def("push", &PythonExtendWin_wrapper::push_motionloader)
			.def("push", &PythonExtendWin_wrapper::push_bone)
			.def("push", &PythonExtendWin_wrapper::push_PLDPrimSkin)
			.def("push", &PythonExtendWin_wrapper::push_FlLayout)
			.def("push", &PythonExtendWin_wrapper::push_matrixn)
			.def("push", &PythonExtendWin_wrapper::push_hypermatrixn)
			.def("push", &PythonExtendWin_wrapper::push_Tensor)
			.def("push", &PythonExtendWin_wrapper::push_posture)
			.def("push", [](PythonExtendWin& l,OBJloader::Geometry & w) { luna_push<OBJloader::Geometry>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,OBJloader::Mesh & w) { luna_push<OBJloader::Mesh>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,boolN & w) { luna_push<boolN>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,PoseTransfer & w) { luna_push<PoseTransfer>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,Viewpoint & w) { luna_push<Viewpoint>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,Ray & w) { luna_push<Ray>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,LuaScript & w) { luna_push<LuaScript>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,ThreadScriptPool & w) { luna_push<ThreadScriptPool>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,TStrings & w) { luna_push<TStrings>(l.L, &w); })
			.def("push", [](PythonExtendWin& l,MotionDOFcontainer & w) { luna_push<MotionDOFcontainer>(l.L, &w); })
			.def("pushnil", [](PythonExtendWin&l){lua_pushnil(l.L);})
			.def("newtable", [](PythonExtendWin&l){lua_newtable(l.L);})
			.def("settable", [](PythonExtendWin&l, int index){lua_settable(l.L, index);})
			.def("call", &PythonExtendWin_wrapper::call, TAKE_OWNERSHIP)
			.def("call", &PythonExtendWin_wrapper::call2, TAKE_OWNERSHIP)
			.def("getglobal", &PythonExtendWin_wrapper::getglobal)
			.def("getglobal", &PythonExtendWin_wrapper::getglobal2)
			.def("getglobal", []( PythonExtendWin& l, const char* key1, const char* key2, int key3){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				PythonExtendWin_wrapper::getglobal(l, key1);
				PythonExtendWin_wrapper::replaceTop(l, key2);
				PythonExtendWin_wrapper::replaceTop(l, key3);
				if (lua_isnil(l.L, -1)) luaL_error(l.L, "missing global: %s.%s[%d]", key1, key2, key3);
			})
			.def("getglobal", []( PythonExtendWin& l, const char* key1, int key2){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				PythonExtendWin_wrapper::getglobal(l, key1);
				PythonExtendWin_wrapper::replaceTop(l, key2);
				if (lua_isnil(l.L, -1)) luaL_error(l.L, "missing global: %s[%d]", key1, key2);
			})
			.def("getglobal", []( PythonExtendWin& l, const char* key1, const char* key2, const char* key3){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				PythonExtendWin_wrapper::getglobal(l, key1);
				PythonExtendWin_wrapper::replaceTop(l, key2);
				PythonExtendWin_wrapper::replaceTop(l, key3);
				if (lua_isnil(l.L, -1)) luaL_error(l.L, "missing global: %s.%s.%d", key1, key2, key3);
			})
			.def("getglobalNoCheck", &PythonExtendWin_wrapper::getglobalNoCheck)
			.def("getglobalNoCheck", &PythonExtendWin_wrapper::getglobal2NoCheck)
			.def("pushvalue",[](PythonExtendWin& l, int index){ lua_pushvalue(l.L, index);})
			.def("pushnil",[](PythonExtendWin& l){ lua_pushnil(l.L);})
			.def("next",[](PythonExtendWin& l, int index)->bool{ return lua_next(l.L, index);})
			.def("getMemberFunc", &PythonExtendWin_wrapper::getMemberFunc)
			.def("insert", &PythonExtendWin_wrapper::insert)
			.def("replaceTop", (void (*)(PythonExtendWin& l, const char* key))&PythonExtendWin_wrapper::replaceTop)
			.def("replaceTop", (void (*)(PythonExtendWin& l, int key))&PythonExtendWin_wrapper::replaceTop)
			.def("printStack", &PythonExtendWin_wrapper::printStack)
  			.def("popmatrixn", &PythonExtendWin_wrapper::popmatrixn, RETURN_REFERENCE)
  			.def("pophypermatrixn", &PythonExtendWin_wrapper::pophypermatrixn, RETURN_REFERENCE)
  			.def("popTensor", &PythonExtendWin_wrapper::popTensor, RETURN_REFERENCE)
  			.def("popvectorn", &PythonExtendWin_wrapper::popvectorn, RETURN_REFERENCE)
  			.def("popintvectorn", &PythonExtendWin_wrapper::popintvectorn, RETURN_REFERENCE)
  			.def("poploader", [](PythonExtendWin& l)->MotionLoader*{
					MotionLoader* result= (MotionLoader*)Luna<typename LunaTraits<MotionLoader>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popMotion", [](PythonExtendWin& l)->Motion*{
					Motion* result= (Motion*)Luna<typename LunaTraits<Motion>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popPose", [](PythonExtendWin& l)->Posture*{
					Posture* result= (Posture*)Luna<typename LunaTraits<Posture>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popMotionDOF", [](PythonExtendWin& l)->MotionDOF*{
					MotionDOF* result= (MotionDOF*)Luna<typename LunaTraits<MotionDOF>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popVRMLloader", [](PythonExtendWin& l)->VRMLloader*{
					VRMLloader* result= (VRMLloader*)Luna<typename LunaTraits<VRMLloader>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popvector3", [](PythonExtendWin& l)->vector3*{
					vector3* result= (vector3*)Luna<typename LunaTraits<vector3>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popvector2", [](PythonExtendWin& l)->vector2*{
					vector2* result= (vector2*)Luna<typename LunaTraits<vector3>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("poptransf", [](PythonExtendWin& l)->transf*{
					transf* result= (transf*)Luna<typename LunaTraits<transf>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popquater", [](PythonExtendWin& l)->quater*{
					quater* result= (quater*)Luna<typename LunaTraits<quater>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popmatrix4", [](PythonExtendWin& l)->matrix4*{
					matrix4* result= (matrix4*)Luna<typename LunaTraits<matrix4>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popVector3N", [](PythonExtendWin& l)->vector3N*{
					vector3N* result= (vector3N*)Luna<typename LunaTraits<vector3N>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popQuaterN", [](PythonExtendWin& l)->quaterN*{
					quaterN* result= (quaterN*)Luna<typename LunaTraits<quaterN>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popTStrings", [](PythonExtendWin& l)->TStrings*{
					TStrings* result= (TStrings*)Luna<typename LunaTraits<TStrings>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popboolN", [](PythonExtendWin& l)->boolN*{
					boolN* result= (boolN*)Luna<typename LunaTraits<boolN>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popCollisionDetector", [](PythonExtendWin& l)->OpenHRP::CollisionDetector*{
					OpenHRP::CollisionDetector* result= (OpenHRP::CollisionDetector*)Luna<typename LunaTraits<OpenHRP::CollisionDetector>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popLoaderToTree", [](PythonExtendWin& l)->IK_sdls::LoaderToTree*{
					IK_sdls::LoaderToTree* result= (IK_sdls::LoaderToTree*)Luna<typename LunaTraits<IK_sdls::LoaderToTree>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
			.def("popboolean", &PythonExtendWin_wrapper::popboolean)
  			.def("checkmatrixn", &PythonExtendWin_wrapper::checkmatrixn, RETURN_REFERENCE)
  			.def("checkhypermatrixn", &PythonExtendWin_wrapper::checkhypermatrixn, RETURN_REFERENCE)
  			.def("checkTensor", &PythonExtendWin_wrapper::checkTensor, RETURN_REFERENCE)
  			.def("checkvectorn", &PythonExtendWin_wrapper::checkvectorn, RETURN_REFERENCE)
  			.def("checkintvectorn", &PythonExtendWin_wrapper::checkintvectorn, RETURN_REFERENCE)
		    .def("popnumber", &PythonExtendWin_wrapper::popnumber)
		    .def("popstring", &PythonExtendWin_wrapper::popstring)
		    .def("popint", &PythonExtendWin_wrapper::popint)
			.def("set", &PythonExtendWin_wrapper::set)
  			.def("isnil", &PythonExtendWin_wrapper::isnil)
			.def("gettop",&PythonExtendWin_wrapper::gettop)
			.def("pop",&PythonExtendWin_wrapper::pop)
  			.def("popIntIntervals", [](PythonExtendWin& l)->intIntervals*{
					intIntervals* result= (intIntervals*)Luna<typename LunaTraits<intIntervals>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popScaledBoneKinematics", [](PythonExtendWin& l)->ScaledBoneKinematics*{
					ScaledBoneKinematics* result= (ScaledBoneKinematics*)Luna<typename LunaTraits<ScaledBoneKinematics>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
  			.def("popBoneForwardKinematics", [](PythonExtendWin& l)->BoneForwardKinematics*{
					BoneForwardKinematics* result= (BoneForwardKinematics*)Luna<typename LunaTraits<BoneForwardKinematics>::base_t>::check(l.L,-1);
					lua_pop(l.L,1);
					return result;
					},RETURN_REFERENCE)
		;
	}


class_<interval > (mainlib, "interval")                         // 1389
														  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<>())                                                // 1426
	.def(init<m_real>())                                          // 1426
.def(init<m_real,m_real>())                                   // 1426
	.def("setValue", (void (interval::*)(m_real a, m_real b))&interval::setValue) // 1445
	.def("start_pt", (m_real (interval::*)())&interval::start_pt) // 1445
	.def("end_pt", (m_real (interval::*)())&interval::end_pt)     // 1445
	.def("len", (m_real (interval::*)())&interval::len)           // 1445
	.def("intersect", [] (interval const& a,interval const& b)->interval{return a&b;})
	; // end of class impl___pybindgen__math_interval             // 1505
	class_<m ::stitchOp > (mainlib, "stitchOp")                     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def("calc", (void (m ::stitchOp::*)(matrixn& c, const matrixn& a, const matrixn& b))&m ::stitchOp::calc) // 1445
	; // end of class impl___pybindgen__math_stitchOp             // 1505
	class_<MotionUtil ::RetargetOnline2D > (mainlib, "RetargetOnline2D") // 1389
																   // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<Motion &,int>())                                    // 1426
.def(init<MotionDOF &,int>())                                 // 1426
	.def("adjust", (void (MotionUtil ::RetargetOnline2D::*)(int time, quater const& oriY, vector3 const& pos2D))&MotionUtil ::RetargetOnline2D::adjust) // 1445
	.def("adjust", (void (MotionUtil ::RetargetOnline2D::*)(int time, quater const& oriY))&MotionUtil ::RetargetOnline2D::adjust) // 1445
	.def("adjust", (void (MotionUtil ::RetargetOnline2D::*)(int time, vector3 const& pos2D))&MotionUtil ::RetargetOnline2D::adjust) // 1445
	.def("adjust", (void (MotionUtil ::RetargetOnline2D::*)(int time, m_real deltarot))&MotionUtil ::RetargetOnline2D::adjust) // 1445
	.def("adjustSafe", (void (MotionUtil ::RetargetOnline2D::*)(int time, m_real deltarot))&MotionUtil ::RetargetOnline2D::adjustSafe) // 1445
	.def("adjust", (void (MotionUtil ::RetargetOnline2D::*)(int time, int time2, intvectorn& times))&MotionUtil ::RetargetOnline2D::adjust) // 1445
	; // end of class impl___pybindgen__MotionUtil_RetargetOnline2D // 1505
	class_<m ::c0stitch > (mainlib, "c0stitch")                     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
.def(init<>())                                                // 1426
	; // end of class impl___pybindgen__math_c0stitch             // 1505
	class_<m ::c0concat > (mainlib, "c0concat")                     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
.def(init<>())                                                // 1426
	; // end of class impl___pybindgen__math_c0concat             // 1505
	class_<m ::linstitch > (mainlib, "linstitch")                   // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<>())                                                // 1426
.def(init<m_real>())                                          // 1426
	; // end of class impl___pybindgen__math_linstitch            // 1505
	class_<m ::linstitchOnline > (mainlib, "linstitchOnline")       // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<>())                                                // 1426
.def(init<m_real>())                                          // 1426
	; // end of class impl___pybindgen__math_linstitchOnline      // 1505
	class_<m ::linstitchForward > (mainlib, "linstitchForward")     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
.def(init<>())                                                // 1426
	; // end of class impl___pybindgen__math_linstitchForward     // 1505
	class_<m ::linstitchMulti > (mainlib, "linstitchMulti")         // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<>())                                                // 1426
.def(init<m_real>())                                          // 1426
	; // end of class impl___pybindgen__math_linstitchMulti       // 1505
	class_<m ::c1stitchPreprocess > (mainlib, "c1stitchPreprocess") // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<int,int,m_real,bool>())                             // 1426
.def(init<int,int>())                                         // 1426
	; // end of class impl___pybindgen__math_c1stitchPreprocess   // 1505
	class_<m ::c1stitchPreprocessOnline > (mainlib, "c1stitchPreprocessOnline") // 1389
																		  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<int,int,m_real>())                                  // 1426
.def(init<int,int>())                                         // 1426
	; // end of class impl___pybindgen__math_c1stitchPreprocessOnline // 1505
	class_<BSpline > (mainlib, "BSpline")                           // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
	.def(init<const matrixn &,int,bool,bool>())                   // 1426
.def(init<const matrixn &,int>())                             // 1426
	.def("GetNumCtrlPoints", (int (BSpline::*)())&BSpline::GetNumCtrlPoints) // 1445
	.def("GetDegree", (int (BSpline::*)())&BSpline::GetDegree)    // 1445
	.def("IsOpen", (bool (BSpline::*)())&BSpline::IsOpen)         // 1445
	.def("IsUniform", (bool (BSpline::*)())&BSpline::IsUniform)   // 1445
	.def("IsLoop", (bool (BSpline::*)())&BSpline::IsLoop)         // 1445
	.def("SetControlPoint", (void (BSpline::*)(int i, const vectorn& rkCtrl))&BSpline::SetControlPoint) // 1445
	.def("GetControlPoint", (vectornView (BSpline::*)(int i))&BSpline::GetControlPoint) // 1445
	.def("Knot", (m_real (BSpline::*)(int i))&BSpline::Knot)      // 1445
	.def("GetPosition", (void (BSpline::*)(m_real fTime, vectorn& kPos))&BSpline::GetPosition) // 1445
	.def("GetFirstDerivative", (void (BSpline::*)(m_real fTime, vectorn& kDer1))&BSpline::GetFirstDerivative) // 1445
	.def("GetSecondDerivative", (void (BSpline::*)(m_real fTime, vectorn& kDer2))&BSpline::GetSecondDerivative) // 1445
	.def("GetThirdDerivative", (void (BSpline::*)(m_real fTime, vectorn& kDer3))&BSpline::GetThirdDerivative) // 1445
	; // end of class impl___pybindgen__math_BSpline              // 1505
	class_<QPerformanceTimerCount2 > (mainlib, "PerfTimer2")        // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
.def(init<>())                                                // 1426
	.def("reset", (void (QPerformanceTimerCount2::*)())&QPerformanceTimerCount2::reset) // 1445
	.def("start", (void (QPerformanceTimerCount2::*)())&QPerformanceTimerCount2::start) // 1445
	.def("pause", (void (QPerformanceTimerCount2::*)())&QPerformanceTimerCount2::pause) // 1445
	.def("stop", (long (QPerformanceTimerCount2::*)())&QPerformanceTimerCount2::stop) // 1445
	; // end of class impl___pybindgen__util_PerfTimer2           // 1505
	class_<QPerformanceTimer2 > (mainlib, "Timer")                  // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
.def(init<>())                                                // 1426
	.def("start", (void (QPerformanceTimer2::*)())&QPerformanceTimer2::start) // 1445
	.def("stop2", (long (QPerformanceTimer2::*)())&QPerformanceTimer2::stop2) // 1445
	; // end of class impl___pybindgen__util_Timer                // 1505

{
	// luna_physics
	class_<InertiaCalculator > (mainlib, "InertiaCalculator")       // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<int>())                                             // 1426
		.def("calculateFromFile", (void (InertiaCalculator::*)(const char* objfilename))&InertiaCalculator::calculateFromFile) // 1445
		.def("calculateFromMesh", (void (InertiaCalculator::*)(OBJloader::Mesh const &mesh))&InertiaCalculator::calculateFromMesh) // 1445
		.def("calculateFromBox", (void (InertiaCalculator::*)(m_real sizeX, m_real sizeY, m_real sizeZ))&InertiaCalculator::calculateFromBox) // 1445
		.def("calculateFromCylinder", (void (InertiaCalculator::*)(m_real radius, m_real height))&InertiaCalculator::calculateFromCylinder) // 1445
		.def("drawSamplingGrid", (void (InertiaCalculator::*)(m_real radius, vector3 const& translate))&InertiaCalculator::drawSamplingGrid) // 1445
		.def("centerOfMass", (const vector3 & (InertiaCalculator::*)())&InertiaCalculator::centerOfMass,return_value_policy::reference ) // 1450
		.def("inertia", (const matrix3 & (InertiaCalculator::*)())&InertiaCalculator::inertia,return_value_policy::reference ) // 1450
		.def("volume", (m_real (InertiaCalculator::*)())&InertiaCalculator::volume) // 1445
		; // end of class impl___pybindgen__Physics_InertiaCalculator // 1505

	class_<OpenHRP ::DynamicsSimulator_QP ::ContactBasis > (mainlib, "ContactBasis") // 1389
																			   // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def_readwrite("ibody", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::ibody)
		.def_readwrite("ibone", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::ibone)
		.def_readwrite("globalpos", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::globalpos)
		.def_readwrite("relvel", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::relvel)
		.def_readwrite("normal", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::normal)
		.def_readwrite("frictionNormal", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::frictionNormal)
		.def_readwrite("depth", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::depth)
		.def_readwrite("globalIndex", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::globalIndex)
		.def_readwrite("globalFrictionIndex", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::globalFrictionIndex)
		.def_readwrite("ilinkpair", &OpenHRP ::DynamicsSimulator_QP ::ContactBasis ::ilinkpair)
		; // end of class impl___pybindgen__Physics_ContactBasis      // 1505
	struct __pybindgen__Physics_Vec_ContactBasis_wrapper
	{                                                             // 1382
		static void assign(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> & out, std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> const& in) 
		{
			out.resize(in.size());
			for(int i=0; i<in.size(); i++){ out[i]=in[i];}
		}
		// 1383
	};                                                            // 1384
	class_<std ::vector <OpenHRP ::DynamicsSimulator_QP ::ContactBasis > > (mainlib, "Vec_ContactBasis") // 1389
																								   // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("size", (int (std ::vector <OpenHRP ::DynamicsSimulator_QP ::ContactBasis >::*)())&std ::vector <OpenHRP ::DynamicsSimulator_QP ::ContactBasis >::size) // 1445
		.def("__call__", [](std ::vector <OpenHRP ::DynamicsSimulator_QP ::ContactBasis >& vec, int i)->OpenHRP ::DynamicsSimulator_QP ::ContactBasis &{ return vec[i];},return_value_policy::reference ) // 1450
		.def("assign", &__pybindgen__Physics_Vec_ContactBasis_wrapper:: assign) // 1458
		; // end of class impl___pybindgen__Physics_Vec_ContactBasis  // 1505
	class_<GrahamScan > (mainlib, "GrahamScan")                     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("add_point", (void (GrahamScan::*)(std::pair<double, double> const& point))&GrahamScan::add_point) // 1445
		.def("partition_points", (void (GrahamScan::*)())&GrahamScan::partition_points) // 1445
		.def("build_hull", (void (GrahamScan::*)())&GrahamScan::build_hull) // 1445
		.def("print_raw_points", (void (GrahamScan::*)())&GrahamScan::print_raw_points) // 1445
		.def("print_hull", (void (GrahamScan::*)())&GrahamScan::print_hull) // 1445
		.def("get_hull", (void (GrahamScan::*)(matrixn& out))&GrahamScan::get_hull) // 1445
		.def("direction", (double (*)( std::pair<double,double> p0, std::pair<double,double> p1, std::pair<double,double> p2 ))&GrahamScan::direction) // 1458
		; // end of class impl___pybindgen__math_GrahamScan           // 1505
	struct __pybindgen__Physics_ContactForce_wrapper
	{                                                             // 1382
		inline static int _property_get_chara(OpenHRP ::DynamicsSimulator ::ContactForce const& a) { return a.chara; }inline static void _property_set_chara(OpenHRP ::DynamicsSimulator ::ContactForce & a, int b){ a.chara=b;}
		inline static VRMLTransform *& _property_get_bone(OpenHRP ::DynamicsSimulator ::ContactForce const& a) { return (VRMLTransform * &) a.bone; }
		inline static vector3& _property_get_f(OpenHRP ::DynamicsSimulator ::ContactForce const& a) { return (vector3 &) a.f; }
		inline static vector3& _property_get_p(OpenHRP ::DynamicsSimulator ::ContactForce const& a) { return (vector3 &) a.p; }
		inline static vector3& _property_get_tau(OpenHRP ::DynamicsSimulator ::ContactForce const& a) { return (vector3 &) a.tau; }
		// 1383
	};                                                            // 1384
	class_<OpenHRP ::DynamicsSimulator ::ContactForce > (mainlib, "ContactForce") // 1389
																			// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def_readwrite("chara", &OpenHRP ::DynamicsSimulator ::ContactForce ::chara)
		.def_readwrite("bone", &OpenHRP ::DynamicsSimulator ::ContactForce ::bone)
		.def_readwrite("f", &OpenHRP ::DynamicsSimulator ::ContactForce ::f)
		.def_readwrite("p", &OpenHRP ::DynamicsSimulator ::ContactForce ::p)
		.def_readwrite("tau", &OpenHRP ::DynamicsSimulator ::ContactForce ::tau)
		; // end of class impl___pybindgen__Physics_ContactForce      // 1505
	class_<HessianQuadratic > (mainlib, "HessianQuadratic")         // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<int>())                                             // 1426
		.def("addSquared", (void (HessianQuadratic::*)(intvectorn const& , vectorn const& ))&HessianQuadratic::addSquared) // 1445
		.def_readwrite("H", &HessianQuadratic ::H)
		.def_readwrite("R", &HessianQuadratic ::R)
		; // end of class impl___pybindgen___HessianQuadratic         // 1505
	class_<std ::vector <float > > (mainlib, "CartPoleBallCpp")     // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		; // end of class impl___pybindgen___CartPoleBallCpp          // 1505
	struct __pybindgen__Physics_Vec_CFinfo_wrapper
	{                                                             // 1382
		static OpenHRP::DynamicsSimulator_QP::ContactBasis& __call2(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis> const& in, int index)
		{
			return (OpenHRP::DynamicsSimulator_QP::ContactBasis&)(in[index]);
		}
		static OpenHRP::DynamicsSimulator::ContactForce & __call(std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in, int index)
		{
			return (OpenHRP::DynamicsSimulator::ContactForce &)(in[index]);
		}

		static void assign(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in) 
		{
			out.resize(in.size());
			for(int i=0; i<in.size(); i++){ out[i]=in[i];}
		}

		static void normalize(OpenHRP::DynamicsSimulator::ContactForce & cf2,
				OpenHRP::DynamicsSimulator::ContactForce const& cf,
				OpenHRP::DynamicsSimulator & sim)
		{
			Bone* b= cf.bone;
			vector3 gf=sim.getWorldState(cf.chara)._global(*b).toGlobalDir(cf.f);
			vector3 gp=sim.getWorldState(cf.chara)._global(*b).toGlobalPos(cf.p);
			vector3 gtau=gp.cross(gf)+sim.getWorldState(cf.chara)._global(*b).toGlobalDir(cf.tau);

			cf2.bone=cf.bone;
			cf2.chara=cf.chara;                
			cf2.f=sim.getWorldState(cf.chara)._global(*b).toLocalDir(gf);
			cf2.p=sim.getWorldState(cf.chara)._global(*b).toLocalPos(vector3(0,0,0));
			cf2.tau=sim.getWorldState(cf.chara)._global(*b).toLocalDir(gtau);
		}

		static void scale(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, 
				double s)
		{
			// assumes that cf has been normalized.
			for (int i=0; i<out.size(); i++){
				out[i].f*=s;
				out[i].tau*=s;
			}                
		}

		static void compaction(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1, OpenHRP::DynamicsSimulator & sim)
		{
			out.reserve(in1.size());
			out.resize(0);

			for(int i=0; i<in1.size(); i++){

				if (in1[i].chara==0){// ignore chara2 assuming it's static object.
					if(out.size()==0 || out.back().bone!=in1[i].bone){
						OpenHRP::DynamicsSimulator::ContactForce cf;
						normalize(cf, in1[i], sim);
						out.push_back(cf);
					}
					else {
						OpenHRP::DynamicsSimulator::ContactForce cf;
						normalize(cf, in1[i], sim);

						OpenHRP::DynamicsSimulator::ContactForce& cf2=out.back();
						cf2.f+=cf.f;
						cf2.tau+=cf.tau;
					}
				}
			}
		}

		static void merge(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out, 
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1,
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2)
		{
			// assumes that both in1 and in2 are normalized
			intvectorn indexes;

			for (int i=0; i<in1.size(); i++){
				Msg::verify(indexes.findFirstIndex(in1[i].bone->treeIndex())==-1, "index==-1");
				indexes.pushBack(in1[i].bone->treeIndex());
			}
			for (int i=0; i<in2.size(); i++){
				int idx=in2[i].bone->treeIndex();
				if(indexes.findFirstIndex(idx)==-1)
					indexes.pushBack(idx);
			}

			out.resize(indexes.size());
			for(int i=0; i<indexes.size(); i++){
				out[i].f.setValue(0,0,0);
				out[i].tau.setValue(0,0,0);
			}

			for (int i=0; i<in1.size(); i++){
				int idx=indexes.findFirstIndex(in1[i].bone->treeIndex());
				Msg::verify(idx!=-1, "idx ==-1");
				out[i].bone=in1[i].bone;
				out[i].chara=in1[i].chara;
				out[i].p=in1[i].p;
				out[i].f+=in1[i].f;
				out[i].tau+=in1[i].tau;
			}
			for (int i=0; i<in2.size(); i++){
				int idx=indexes.findFirstIndex(in2[i].bone->treeIndex());
				out[i].bone=in2[i].bone;
				out[i].chara=in2[i].chara;
				out[i].p=in2[i].p;
				out[i].f+=in2[i].f;
				out[i].tau+=in2[i].tau;
			}



		}

		static void interpolate(std::vector<OpenHRP::DynamicsSimulator::ContactForce> & out,                     
				m_real t,
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in1,
				std::vector<OpenHRP::DynamicsSimulator::ContactForce> const& in2,
				OpenHRP::DynamicsSimulator & sim)
		{
			std::vector<OpenHRP::DynamicsSimulator::ContactForce> t1, t2;

			compaction(t1, in1, sim);
			scale(t1, 1-t);

			compaction(t2, in2, sim);
			scale(t2, t);

			printf("here\n");
			merge(out, t1, t2);
		}
		// 1383
	};                                                            // 1384
	class_<std ::vector <OpenHRP ::DynamicsSimulator ::ContactForce > > (mainlib, "Vec_CFinfo") // 1389
																						  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("size", (int (std ::vector <OpenHRP ::DynamicsSimulator ::ContactForce >::*)())&std ::vector <OpenHRP ::DynamicsSimulator ::ContactForce >::size) // 1445
		.def("__call__", &__pybindgen__Physics_Vec_CFinfo_wrapper::__call2, return_value_policy::reference) // 1463
		.def("__call__", &__pybindgen__Physics_Vec_CFinfo_wrapper::__call, return_value_policy::reference) // 1463
		.def("assign", &__pybindgen__Physics_Vec_CFinfo_wrapper::assign, return_value_policy::reference) // 1463
		.def("normalize", &__pybindgen__Physics_Vec_CFinfo_wrapper::normalize, return_value_policy::reference) // 1463
		.def("scale", &__pybindgen__Physics_Vec_CFinfo_wrapper::scale, return_value_policy::reference) // 1463
		.def("compaction", &__pybindgen__Physics_Vec_CFinfo_wrapper::compaction, return_value_policy::reference) // 1463
		.def("merge", &__pybindgen__Physics_Vec_CFinfo_wrapper::merge, return_value_policy::reference) // 1463
		.def("interpolate", &__pybindgen__Physics_Vec_CFinfo_wrapper::interpolate, return_value_policy::reference) // 1463
		; // end of class impl___pybindgen__Physics_Vec_CFinfo        // 1505
	class_<OpenHRP ::CollisionDetector ::RayTestResult > (mainlib, "RayTestResult") // 1389
																			  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("hasHit", (bool (OpenHRP ::CollisionDetector ::RayTestResult::*)())&OpenHRP ::CollisionDetector ::RayTestResult::hasHit) // 1445
		.def_readwrite("m_closestHitFraction", &OpenHRP ::CollisionDetector ::RayTestResult ::m_closestHitFraction)
		.def_readwrite("m_hitNormalWorld", &OpenHRP ::CollisionDetector ::RayTestResult ::m_hitNormalWorld)
		; // end of clasdgen__Physics_RayTestResult     // 1505
	class_<OpenHRP ::CollisionPoint > (mainlib, "CollisionPoint")   // 1389
															  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def_readwrite("position", &OpenHRP ::CollisionPoint ::position)
		.def_readwrite("normal", &OpenHRP ::CollisionPoint ::normal)
		.def_readwrite("idepth", &OpenHRP ::CollisionPoint ::idepth)
		; // end of class impl___pybindgen__Physics_CollisionPoint    // 1505
	struct __pybindgen__Physics_CollisionPointSequence_wrapper
	{                                                             // 1382
		static void erase(OpenHRP::CollisionPointSequence& self, int i)
		{ self.erase(self.begin()+i); }
		// 1383
	};                                                            // 1384
	class_<OpenHRP ::CollisionPointSequence > (mainlib, "CollisionPointSequence") // 1389
																			// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def("__call__", [](OpenHRP ::CollisionPointSequence& v,int i)->OpenHRP::CollisionPoint&{ return v[i];},return_value_policy::reference ) // 1450
		.def("size", (int (OpenHRP ::CollisionPointSequence::*)())&OpenHRP ::CollisionPointSequence::size) // 1445
		.def("resize", [](OpenHRP ::CollisionPointSequence&v, int n){ v.resize(n);})
		.def("erase", [](OpenHRP ::CollisionPointSequence&v, int n){ v.erase(v.begin()+n);})
		; // end of class impl___pybindgen__Physics_CollisionPointSequence // 1505
	class_<OpenHRP ::CollisionSequence > (mainlib, "CollisionSequence") // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("getCollisionPoints", (OpenHRP ::CollisionPointSequence & (OpenHRP ::CollisionSequence::*)(int ilinkpair))&OpenHRP ::CollisionSequence::getCollisionPoints,return_value_policy::reference ) // 1450
		.def("getNumLinkPairs", (int (OpenHRP ::CollisionSequence::*)())&OpenHRP ::CollisionSequence::getNumLinkPairs) // 1445
		.def("getCollisionLinkPairs", (intvectorn (OpenHRP ::CollisionSequence::*)())&OpenHRP ::CollisionSequence::getCollisionLinkPairs) // 1445
		.def("getCharacterIndex1", (int (OpenHRP ::CollisionSequence::*)(int ilinkpair))&OpenHRP ::CollisionSequence::getCharacterIndex1) // 1445
		.def("getBone1", (VRMLTransform * (OpenHRP ::CollisionSequence::*)(int ilinkpair))&OpenHRP ::CollisionSequence::getBone1,return_value_policy::reference ) // 1450
		.def("getCharacterIndex2", (int (OpenHRP ::CollisionSequence::*)(int ilinkpair))&OpenHRP ::CollisionSequence::getCharacterIndex2) // 1445
		.def("getBone2", (VRMLTransform * (OpenHRP ::CollisionSequence::*)(int ilinkpair))&OpenHRP ::CollisionSequence::getBone2,return_value_policy::reference ) // 1450
		.def("createCollisionDetector_bullet", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_bullet, return_value_policy::reference) // 1463
		.def("createCollisionDetector_gjk", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_gjk, return_value_policy::reference) // 1463
		.def("createCollisionDetector_libccd", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_libccd, return_value_policy::reference) // 1463
		//.def("createCollisionDetector_libccd_LBS", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_libccd_LBS, return_value_policy::reference) // 1463
		//.def("createCollisionDetector_libccd_merged", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_libccd_merged, return_value_policy::reference) // 1463
		.def("createCollisionDetector_fcl", (OpenHRP ::CollisionDetector * (*)())&OpenHRP::createCollisionDetector_fcl, return_value_policy::reference) // 1463
		; // end of class impl___pybindgen__Physics_CollisionSequence // 1505
	class_<OpenHRP ::CollisionDetector > (mainlib, "CollisionDetector") // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def("addModel", (int (OpenHRP ::CollisionDetector::*)(VRMLloader* loader))&OpenHRP ::CollisionDetector::addModel) // 1445
		.def("addObstacle", (int (OpenHRP ::CollisionDetector::*)(OBJloader::Geometry const& mesh))&OpenHRP ::CollisionDetector::addObstacle) // 1445
		.def("getModel", (VRMLloader * (OpenHRP ::CollisionDetector::*)(int ichar))&OpenHRP ::CollisionDetector::getModel,return_value_policy::reference ) // 1450
		.def("setMargin", (void (OpenHRP ::CollisionDetector::*)(int ilink, double margin))&OpenHRP ::CollisionDetector::setMargin) // 1445
		.def("setMarginAll", (void (OpenHRP ::CollisionDetector::*)(vectorn const & margin))&OpenHRP ::CollisionDetector::setMarginAll) // 1445
		.def("getMarginAll", (void (OpenHRP ::CollisionDetector::*)(vectorn & margin))&OpenHRP ::CollisionDetector::getMarginAll) // 1445
		.def("addCollisionPair", (void (OpenHRP ::CollisionDetector::*)(VRMLloader* skel1, int ibone1, VRMLloader* skel2, int ibone2))&OpenHRP ::CollisionDetector::addCollisionPair) // 1445
		.def("setWorldTransformations", (void (OpenHRP ::CollisionDetector::*)(int charIndex, BoneForwardKinematics const& fk))&OpenHRP ::CollisionDetector::setWorldTransformations) // 1445
		.def("rayTest", (void (OpenHRP ::CollisionDetector::*)(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result))&OpenHRP ::CollisionDetector::rayTest) // 1445
		.def("rayTestBackside", (void (OpenHRP ::CollisionDetector::*)(int ichar, int ilink, vector3 const& from, vector3 const& to, OpenHRP::CollisionDetector::RayTestResult& result))&OpenHRP ::CollisionDetector::rayTestBackside) // 1445
		.def("testIntersectionsForDefinedPairs", (bool (OpenHRP ::CollisionDetector::*)(OpenHRP::CollisionSequence & collisions))&OpenHRP ::CollisionDetector::testIntersectionsForDefinedPairs) // 1445
		.def("getLocalBoundingBoxSize", (bool (OpenHRP ::CollisionDetector::*)(int charIndex, int ibone, vector3& localSize))&OpenHRP ::CollisionDetector::getLocalBoundingBoxSize) // 1445
		.def("isSignedDistanceSupported", (bool (OpenHRP ::CollisionDetector::*)())&OpenHRP ::CollisionDetector::isSignedDistanceSupported) // 1445
		.def("calculateSignedDistance", (double (OpenHRP ::CollisionDetector::*)(int iloader, int ibody, vector3 const& position, vector3& normal))&OpenHRP ::CollisionDetector::calculateSignedDistance) // 1445
		.def("isSphereTestSupported", (bool (OpenHRP ::CollisionDetector::*)())&OpenHRP ::CollisionDetector::isSphereTestSupported) // 1445
		.def("testSphereIntersection", (double (OpenHRP ::CollisionDetector::*)(int iloader, int ibody, vector3 const& position, double radius, vector3& contactpos, vector3& normal))&OpenHRP ::CollisionDetector::testSphereIntersection) // 1445
		; // end of class impl___pybindgen__Physics_CollisionDetector // 1505
	class_<OpenHRP ::CollisionDetector_libccd , OpenHRP ::CollisionDetector> (mainlib, "CollisionDetector_libccd") // 1389
																				// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def("CollisionCheck", (bool (*)(OpenHRP::CollisionDetector &s,OpenHRP::CollisionSequence & collisions, std::string chekmesh, std::string skipmesh))&OpenHRP::CollisionCheck) // 1458
		.def("testSphereBox", (vector2 (*)(vector3 const& center, double r, transf const& box, vector3 const& box_size, vector3& pos, vector3& normal))&OpenHRP::testSphereBox) // 1458
		; // end of class impl___pybindgen__Physics_CollisionDetector_libccd // 1505
	struct __pybindgen__Physics_DynamicsSimulator_wrapper
	{                                                             // 1382

		static void setParameter2(const char* _what, bool value)
		{
			TString what(_what);

			if(what=="usePenaltyMethod")
				OpenHRP::globals::usePenaltyMethod=value;
			else
				Msg::error("unknown parameter %s", _what);
		}

		static void setParameter(const char* _what, double value)
		{
			TString what(_what);

			Msg::error("unknown parameter %s", _what);
		}

		static vector3 calcCOM(OpenHRP::DynamicsSimulator&s, int ichara)
		{
			double totalMass;
			return s.calculateCOM(ichara,totalMass);
		}
		static double calcTotalMass(OpenHRP::DynamicsSimulator&s, int ichara)
		{
			double totalMass;
			s.calculateCOM(ichara,totalMass);
			return totalMass;
		}
		static vector3 calcCOMvel(OpenHRP::DynamicsSimulator&s, int ichara)
		{
			double totalMass;
			return s.calculateCOMvel(ichara,totalMass);
		}
		// 1383
	};                                                            // 1384
	enum_<OpenHRP::DynamicsSimulator::LinkDataType>(mainlib, "Physics")
		.value("JOINT_VALUE", OpenHRP::DynamicsSimulator::JOINT_VALUE)
		.value("JOINT_VELOCITY", OpenHRP::DynamicsSimulator::JOINT_VELOCITY)
		.value("JOINT_ACCELERATION",OpenHRP::DynamicsSimulator::JOINT_ACCELERATION)
		.value("JOINT_TORQUE", OpenHRP::DynamicsSimulator::JOINT_TORQUE)
	;
	class_<OpenHRP ::DynamicsSimulator > (mainlib, "DynamicsSimulator") // 1389
																  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def("drawLastContactForces", [](OpenHRP ::DynamicsSimulator&s ){ s.drawLastContactForces();}) // 1445
		.def("drawLastContactForces", [](OpenHRP ::DynamicsSimulator&s ,int ichara){ s.drawLastContactForces(ichara);}) // 1445
		.def("drawLastContactForces", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vector3 const& draw_offset))&OpenHRP ::DynamicsSimulator::drawLastContactForces) // 1445
		.def("getContactForce", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichar, int ibone))&OpenHRP ::DynamicsSimulator::getContactForce) // 1445
		.def("calcInertia", (void (OpenHRP ::DynamicsSimulator::*)(int ichara,vectorn const& pose, vectorn& inertia))&OpenHRP ::DynamicsSimulator::calcInertia) // 1445
		.def("calcMomentumCOMfromPose", (Liegroup ::dse3 (OpenHRP ::DynamicsSimulator::*)(int ichara, double t, vectorn const& poseFrom, vectorn const& poseTo))&OpenHRP ::DynamicsSimulator::calcMomentumCOMfromPose) // 1445
		.def("calcMomentumCOM", (Liegroup ::dse3 (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::calcMomentumCOM) // 1445
		.def("setPoseDOF", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator::setPoseDOF) // 1445
		.def("setDPoseDOF", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator::setDPoseDOF) // 1445
		.def("getPoseDOF", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn & v)const)&OpenHRP ::DynamicsSimulator::getPoseDOF) // 1445
		.def("getDPoseDOF", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn & v)const)&OpenHRP ::DynamicsSimulator::getDPoseDOF) // 1445
		.def("getPoseDOF", (vectorn (OpenHRP ::DynamicsSimulator::*)(int ichara)const)&OpenHRP ::DynamicsSimulator::getPoseDOF) // 1445
		.def("getDPoseDOF", (vectorn (OpenHRP ::DynamicsSimulator::*)(int ichara)const)&OpenHRP ::DynamicsSimulator::getDPoseDOF) // 1445
		.def("getCollisionDetector", (OpenHRP ::CollisionDetector * (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::getCollisionDetector,return_value_policy::reference ) // 1450
		.def("getCollisionSequence", (OpenHRP ::CollisionSequence * (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::getCollisionSequence,return_value_policy::reference ) // 1450
		.def("drawDebugInformation", (void (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::drawDebugInformation) // 1445
		.def("registerCharacter", (void (OpenHRP ::DynamicsSimulator::*)(VRMLloader*l))&OpenHRP ::DynamicsSimulator::registerCharacter) // 1445
		.def("createObstacle", (void (OpenHRP ::DynamicsSimulator::*)(OBJloader::Geometry const& mesh))&OpenHRP ::DynamicsSimulator::createObstacle) // 1445
		.def("createFreeBody", (void (OpenHRP ::DynamicsSimulator::*)(OBJloader::Geometry const& mesh))&OpenHRP ::DynamicsSimulator::createFreeBody) // 1445
		.def("registerCollisionCheckPair", (void (OpenHRP ::DynamicsSimulator::*)( int ichara1, const char* name1, int ichara2, const char* name2, vectorn const& param))&OpenHRP ::DynamicsSimulator::registerCollisionCheckPair) // 1445
		.def("registerCollisionCheckPair", (void (OpenHRP ::DynamicsSimulator::*)( const char* char1, const char* name1, const char* char2, const char* name2, vectorn const& param))&OpenHRP ::DynamicsSimulator::registerCollisionCheckPair) // 1445
		.def("init", [](OpenHRP ::DynamicsSimulator& s, double timestep){ s.init(timestep,OpenHRP::DynamicsSimulator::EULER );}) // 1445
		.def("setTimestep", (void (OpenHRP ::DynamicsSimulator::*)(double timeStep))&OpenHRP ::DynamicsSimulator::setTimestep) // 1445
		.def("getTimestep", (double (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::getTimestep) // 1445
		.def("setGVector", (void (OpenHRP ::DynamicsSimulator::*)(const vector3& wdata))&OpenHRP ::DynamicsSimulator::setGVector) // 1445
		.def("initSimulation", (void (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::initSimulation) // 1445
		.def("rdof", (int (OpenHRP ::DynamicsSimulator::*)(int ichar))&OpenHRP ::DynamicsSimulator::rdof) // 1445
		.def("dof", (int (OpenHRP ::DynamicsSimulator::*)(int ichar))&OpenHRP ::DynamicsSimulator::dof) // 1445
		.def("numSphericalJoints", (int (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::numSphericalJoints) // 1445
		.def("getWorldPosition", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, VRMLTransform* b, vector3 const& localpos, vector3& globalpos)const)&OpenHRP ::DynamicsSimulator::getWorldPosition) // 1445
		.def("getWorldPosition", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara, VRMLTransform* b, vector3 const& localpos)const)&OpenHRP ::DynamicsSimulator::getWorldPosition) // 1445
		.def("getWorldVelocity", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara,VRMLTransform* b, vector3 const& localpos)const)&OpenHRP ::DynamicsSimulator::getWorldVelocity) // 1445
		.def("getWorldAcceleration", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara,VRMLTransform* b, vector3 const& localpos)const)&OpenHRP ::DynamicsSimulator::getWorldAcceleration) // 1445
		.def("getWorldAngVel", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara, VRMLTransform* b)const)&OpenHRP ::DynamicsSimulator::getWorldAngVel) // 1445
		.def("getWorldAngAcc", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara, VRMLTransform* b)const)&OpenHRP ::DynamicsSimulator::getWorldAngAcc) // 1445
		.def("getWorldState", (BoneForwardKinematics & (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::getWorldState,return_value_policy::reference ) // 1450
		.def("skeleton", (VRMLloader & (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::skeleton,return_value_policy::reference ) // 1450
		.def("name", (std ::string (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::name) // 1445
		.def("findCharacter", (int (OpenHRP ::DynamicsSimulator::*)(const char* _name))&OpenHRP ::DynamicsSimulator::findCharacter) // 1445
		.def("numSkeleton", (int (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::numSkeleton) // 1445
		.def("setWorldState", (void (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::setWorldState) // 1445
		.def("calcJacobianAt", (void (OpenHRP ::DynamicsSimulator::*)(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos))&OpenHRP ::DynamicsSimulator::calcJacobianAt) // 1445
		.def("calcDotJacobianAt", (void (OpenHRP ::DynamicsSimulator::*)(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos))&OpenHRP ::DynamicsSimulator::calcDotJacobianAt) // 1445
		.def("calcDotJacobian", (void (OpenHRP ::DynamicsSimulator::*)(int ichar, int ibone, matrixn& dotjacobian))&OpenHRP ::DynamicsSimulator::calcDotJacobian) // 1445
		.def("setLinkData", (void (OpenHRP ::DynamicsSimulator::*)(int i, OpenHRP::DynamicsSimulator::LinkDataType t, vectorn const& in))&OpenHRP ::DynamicsSimulator::setLinkData) // 1445
		.def("getLinkData", (void (OpenHRP ::DynamicsSimulator::*)(int i, OpenHRP::DynamicsSimulator::LinkDataType t, vectorn& out))&OpenHRP ::DynamicsSimulator::getLinkData) // 1445
		.def("stepSimulation", (bool (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::stepSimulation) // 1445
		.def("currentTime", (double (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::currentTime) // 1445
		.def("setCurrentTime", (void (OpenHRP ::DynamicsSimulator::*)(double t))&OpenHRP ::DynamicsSimulator::setCurrentTime) // 1445
		.def("calculateZMP", (vector3 (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::calculateZMP) // 1445
		.def("registerContactQueryBone", (void (OpenHRP ::DynamicsSimulator::*)(int contactQueryIndex, VRMLTransform* bone))&OpenHRP ::DynamicsSimulator::registerContactQueryBone) // 1445
		.def("queryContact", (bool (OpenHRP ::DynamicsSimulator::*)(int index))&OpenHRP ::DynamicsSimulator::queryContact) // 1445
		.def("queryContacts", (vectorn (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::queryContacts) // 1445
		.def("queryContactDepths", (vectorn (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::queryContactDepths) // 1445
		.def("_updateContactInfo", (void (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::_updateContactInfo) // 1445
		.def("getLastSimulatedPose", (vectorn const & (OpenHRP ::DynamicsSimulator::*)(int ichara))&OpenHRP ::DynamicsSimulator::getLastSimulatedPose,return_value_policy::reference ) // 1450
		.def("queryContactAll", (std ::vector <OpenHRP ::DynamicsSimulator ::ContactForce > & (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::queryContactAll,return_value_policy::reference ) // 1450
		.def("addForceToBone", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, VRMLTransform* b, vector3 const& localpos, vector3 const& localforce))&OpenHRP ::DynamicsSimulator::addForceToBone) // 1445
		.def("addGlobalForceToBone", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, int ibone, vector3 const& globalpos, vector3 const& globalforce))&OpenHRP ::DynamicsSimulator::addGlobalForceToBone) // 1445
		.def("calcMassMatrix", (void (OpenHRP ::DynamicsSimulator::*)(int ichara,matrixn& ,vectorn&))&OpenHRP ::DynamicsSimulator::calcMassMatrix) // 1445
		.def("getNumAllLinkPairs", (int (OpenHRP ::DynamicsSimulator::*)())&OpenHRP ::DynamicsSimulator::getNumAllLinkPairs) // 1445
		.def("getContactLinkBoneIndex", (void (OpenHRP ::DynamicsSimulator::*)(int ipair, intvectorn & ibone))&OpenHRP ::DynamicsSimulator::getContactLinkBoneIndex) // 1445
		.def("addRelativeConstraint", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2))&OpenHRP ::DynamicsSimulator::addRelativeConstraint) // 1445
		.def("removeRelativeConstraint", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, Bone& bone1, Bone& bone2))&OpenHRP ::DynamicsSimulator::removeRelativeConstraint) // 1445
		.def("setQ", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator::setQ) // 1445
		.def("getQ", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn & v)const)&OpenHRP ::DynamicsSimulator::getQ) // 1445
		.def("getQ", (vectorn (OpenHRP ::DynamicsSimulator::*)(int ichara)const)&OpenHRP ::DynamicsSimulator::getQ) // 1445
		.def("setDQ", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator::setDQ) // 1445
		.def("getDQ", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, vectorn& v)const)&OpenHRP ::DynamicsSimulator::getDQ) // 1445
		.def("getDQ", (vectorn (OpenHRP ::DynamicsSimulator::*)(int ichara)const)&OpenHRP ::DynamicsSimulator::getDQ) // 1445
		.def("setU", (void (OpenHRP ::DynamicsSimulator::*)(int ichara, const vectorn& in)const)&OpenHRP ::DynamicsSimulator::setU) // 1445
		.def("setParameter", (void (*)(const char* _what, bool value))&__pybindgen__Physics_DynamicsSimulator_wrapper::setParameter2) // 1458
		.def("setParameter", (void (*)(const char* _what, double value))&__pybindgen__Physics_DynamicsSimulator_wrapper::setParameter) // 1458
		.def("calculateCOM", (vector3 (*)(OpenHRP::DynamicsSimulator&s, int ichara))&__pybindgen__Physics_DynamicsSimulator_wrapper::calcCOM) // 1458
		.def("calculateCOMvel", (vector3 (*)(OpenHRP::DynamicsSimulator&s, int ichara))&__pybindgen__Physics_DynamicsSimulator_wrapper::calcCOMvel) // 1458
		.def("calcTotalMass", (double (*)(OpenHRP::DynamicsSimulator&s, int ichara))&__pybindgen__Physics_DynamicsSimulator_wrapper::calcTotalMass) // 1458
		; // end of class impl___pybindgen__Physics_DynamicsSimulator // 1505
	class_<OpenHRP ::DynamicsSimulator_penaltyMethod , OpenHRP::DynamicsSimulator> (mainlib, "DynamicsSimulator_penaltyMethod") // 1389
																							  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_penaltyMethod // 1505
#if !defined (EXCLUDE_AIST_SIM)                               // 1361
	class_<OpenHRP ::DynamicsSimulator_AIST_penalty , OpenHRP::DynamicsSimulator_penaltyMethod > (mainlib, "DynamicsSimulator_AIST_penalty") // 1389
																							// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_AIST_penalty // 1505
#endif //!defined (EXCLUDE_AIST_SIM)                          // 1536
#if !defined (EXCLUDE_AIST_SIM)                               // 1361
	class_<OpenHRP ::DynamicsSimulator_impl , OpenHRP::DynamicsSimulator > (mainlib, "DynamicsSimulator_AIST") // 1389
																			// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<const char *>())                                    // 1426
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_AIST // 1505
#endif //!defined (EXCLUDE_AIST_SIM)                          // 1536
#if !defined (EXCLUDE_UT_SIM)                                 // 1361
	class_<OpenHRP ::DynamicsSimulator_UT , OpenHRP::DynamicsSimulator > (mainlib, "DynamicsSimulator_UT") // 1389
																		// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<const char *>())                                    // 1426
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_UT // 1505
#endif //!defined (EXCLUDE_UT_SIM)                            // 1536
	class_<OpenHRP ::DynamicsSimulator_TRL_penalty , OpenHRP::DynamicsSimulator_penaltyMethod > (mainlib, "DynamicsSimulator_TRL_penalty") // 1389
																						  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<const char *>())                                    // 1426
		.def("addWorldTorqueToBone", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, VRMLTransform* b, vector3 const& world_torque))&OpenHRP ::DynamicsSimulator_TRL_penalty::addWorldTorqueToBone) // 1445
		.def("calcBodyJacobianAt", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos))&OpenHRP ::DynamicsSimulator_TRL_penalty::calcBodyJacobianAt) // 1445
		.def("calcDotBodyJacobianAt", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichar, int ibone, matrixn& jacobian, matrixn& dotjacobian, vector3 const& localpos))&OpenHRP ::DynamicsSimulator_TRL_penalty::calcDotBodyJacobianAt) // 1445
		.def("calcMomentumDotJacobian", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichar, matrixn& jacobian, matrixn& dotjacobian))&OpenHRP ::DynamicsSimulator_TRL_penalty::calcMomentumDotJacobian) // 1445
		.def("poseToQ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& v, vectorn& out))&OpenHRP ::DynamicsSimulator_TRL_penalty::poseToQ) // 1445
		.def("dposeToDQ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(quater const& rootOri, vectorn const& v, vectorn& out))&OpenHRP ::DynamicsSimulator_TRL_penalty::dposeToDQ) // 1445
		.def("torqueToU", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(const vectorn& cf, vectorn& U))&OpenHRP ::DynamicsSimulator_TRL_penalty::torqueToU) // 1445
		.def("QToPose", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& v, vectorn& out))&OpenHRP ::DynamicsSimulator_TRL_penalty::QToPose) // 1445
		.def("setNonStatePoseDOF", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator_TRL_penalty::setNonStatePoseDOF) // 1445
		.def("setNonStateDQ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn const& dq))&OpenHRP ::DynamicsSimulator_TRL_penalty::setNonStateDQ) // 1445
		.def("setNonStateDDQ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn const& ddq))&OpenHRP ::DynamicsSimulator_TRL_penalty::setNonStateDDQ) // 1445
		.def("getSphericalState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn & q, vectorn& dq))&OpenHRP ::DynamicsSimulator_TRL_penalty::getSphericalState) // 1445
		.def("setSphericalState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, const vectorn& q, const vectorn& dq))&OpenHRP ::DynamicsSimulator_TRL_penalty::setSphericalState) // 1445
		.def("setTau", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, const vectorn& tau))&OpenHRP ::DynamicsSimulator_TRL_penalty::setTau) // 1445
		.def("getNonStateRootQ", (transf (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara))&OpenHRP ::DynamicsSimulator_TRL_penalty::getNonStateRootQ) // 1445
		.def("setState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator_TRL_penalty::setState) // 1445
		.def("getState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn & v))&OpenHRP ::DynamicsSimulator_TRL_penalty::getState) // 1445
		.def("setDDQ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn const& v))&OpenHRP ::DynamicsSimulator_TRL_penalty::setDDQ) // 1445
		.def("getU", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, vectorn& out)const)&OpenHRP ::DynamicsSimulator_TRL_penalty::getU) // 1445
		.def("getU", (vectorn (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara)const)&OpenHRP ::DynamicsSimulator_TRL_penalty::getU) // 1445
		.def("calcS", (int (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, int ibone, matrixn& S))&OpenHRP ::DynamicsSimulator_TRL_penalty::calcS) // 1445
		.def("stateToEulerZYX", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& q, vectorn const& dq, vectorn& eulerState))&OpenHRP ::DynamicsSimulator_TRL_penalty::stateToEulerZYX) // 1445
		.def("stateToEulerYXZ", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& q, vectorn const& dq, vectorn& eulerState))&OpenHRP ::DynamicsSimulator_TRL_penalty::stateToEulerYXZ) // 1445
		.def("eulerZYXtoState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& eulerState, vectorn& state))&OpenHRP ::DynamicsSimulator_TRL_penalty::eulerZYXtoState) // 1445
		.def("eulerYXZtoState", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(vectorn const& eulerState, vectorn& state))&OpenHRP ::DynamicsSimulator_TRL_penalty::eulerYXZtoState) // 1445
		.def("setStablePDparam", (void (OpenHRP ::DynamicsSimulator_TRL_penalty::*)(int ichara, const vectorn& kp, const vectorn& kd))&OpenHRP ::DynamicsSimulator_TRL_penalty::setStablePDparam) // 1445
		.def("calculateStablePDForces", [](OpenHRP ::DynamicsSimulator_TRL_penalty& sim,int ichara, const vectorn& desired_q, vectorn& tau){ sim.calculateStablePDForces(ichara, desired_q, tau);})
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_TRL_penalty // 1505
#if !defined (EXCLUDE_RBDL_SIMULATOR)                         // 1361
	class_<Trbdl ::DynamicsSimulator_Trbdl_penalty , OpenHRP::DynamicsSimulator_penaltyMethod> (mainlib, "DynamicsSimulator_Trbdl_penalty") // 1389
																							// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<const char *>())                                    // 1426
		.def("setNonStatePoseDOF", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, vectorn const& v))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setNonStatePoseDOF) // 1445
		.def("setNonStateDQ", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, vectorn const& dq))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setNonStateDQ) // 1445
		.def("setNonStateDDQ", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, vectorn const& ddq))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setNonStateDDQ) // 1445
		.def("getNonStateRootQ", (transf (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara))&Trbdl ::DynamicsSimulator_Trbdl_penalty::getNonStateRootQ) // 1445
		.def("getSphericalState", (int (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, vectorn& q, vectorn& dq))&Trbdl ::DynamicsSimulator_Trbdl_penalty::getSphericalState) // 1445
		.def("setSphericalState", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, const vectorn& q, const vectorn& dq))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setSphericalState) // 1445
		.def("setTau", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, const vectorn& tau))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setTau) // 1445
		.def("_calcMassMatrix", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, matrixn& out, vectorn & b))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_calcMassMatrix) // 1445
		.def("_calcJacobianAt", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_calcJacobianAt) // 1445
		.def("_enableDotJocobianComputation", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_enableDotJocobianComputation) // 1445
		.def("_calcDotJacobianAt", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichar, int ibone, matrixn& jacobian, vector3 const& localpos))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_calcDotJacobianAt) // 1445
		.def("_Q", (vectornView (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_Q) // 1445
		.def("_QDot", (vectornView (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_QDot) // 1445
		.def("_bodyW", (vector3 (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, int treeIndex))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_bodyW) // 1445
		.def("_bodyV", (vector3 (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, int treeIndex))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_bodyV) // 1445
		.def("_stepKinematic", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, vectorn const& QDDot))&Trbdl ::DynamicsSimulator_Trbdl_penalty::_stepKinematic) // 1445
		.def("setStablePDparam", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, const vectorn& kp, const vectorn& kd))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setStablePDparam) // 1445
		.def("calculateStablePDForces", [](Trbdl ::DynamicsSimulator_Trbdl_penalty& sim,int ichara, const vectorn& desired_q, vectorn& tau){ sim.calculateStablePDForces(ichara, desired_q, tau);})
		.def("setStablePDparam_dof", (void (Trbdl ::DynamicsSimulator_Trbdl_penalty::*)(int ichara, const vectorn& kp, const vectorn& kd))&Trbdl ::DynamicsSimulator_Trbdl_penalty::setStablePDparam_dof) // 1445
		.def("calculateStablePDForces_dof", [](Trbdl ::DynamicsSimulator_Trbdl_penalty& sim,int ichara, const vectorn& desired_pose, const vectorn& desired_dpose, vectorn& tau){ sim.calculateStablePDForces_dof(ichara, desired_pose, desired_dpose, tau);})
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_Trbdl_penalty // 1505
#endif //!defined (EXCLUDE_RBDL_SIMULATOR)                    // 1536
#if !defined (EXCLUDE_RBDL_SIMULATOR)                         // 1361
	class_<Trbdl ::DynamicsSimulator_Trbdl_LCP ,Trbdl::DynamicsSimulator_Trbdl_penalty> (mainlib, "DynamicsSimulator_Trbdl_LCP") // 1389
																					// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<const char *>())                                    // 1426
		.def("getMLCP", (const matrixn & (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)())&Trbdl ::DynamicsSimulator_Trbdl_LCP::getMLCP,return_value_policy::reference ) // 1450
		.def("getMLCP_B", (const vectorn & (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)())&Trbdl ::DynamicsSimulator_Trbdl_LCP::getMLCP_B,return_value_policy::reference ) // 1450
		.def("getMLCP_X", (const vectorn & (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)())&Trbdl ::DynamicsSimulator_Trbdl_LCP::getMLCP_X,return_value_policy::reference ) // 1450
		.def("setParam_Epsilon_Kappa", (void (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)(double eps, double kap))&Trbdl ::DynamicsSimulator_Trbdl_LCP::setParam_Epsilon_Kappa) // 1445
		.def("setParam_R_B_MA", (void (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)(double r, double b, double ma))&Trbdl ::DynamicsSimulator_Trbdl_LCP::setParam_R_B_MA) // 1445
		.def("stepSimulation", (void (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)())&Trbdl ::DynamicsSimulator_Trbdl_LCP::stepSimulation) // 1445
		.def("addRelativeConstraint", (void (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2))&Trbdl ::DynamicsSimulator_Trbdl_LCP::addRelativeConstraint) // 1445
		.def("removeRelativeConstraint", (void (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)(int ichara, Bone& bone1, Bone& bone2))&Trbdl ::DynamicsSimulator_Trbdl_LCP::removeRelativeConstraint) // 1445
		.def("getCOMbasedContactForce", (Liegroup ::dse3 (Trbdl ::DynamicsSimulator_Trbdl_LCP::*)(int ichar, int ibone))&Trbdl ::DynamicsSimulator_Trbdl_LCP::getCOMbasedContactForce) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_Trbdl_LCP // 1505
#endif //!defined (EXCLUDE_RBDL_SIMULATOR)                    // 1536
#if !defined (EXCLUDE_RBDL_SIMULATOR)                         // 1361
	class_<Trbdl ::DynamicsSimulator_Trbdl_impulse ,Trbdl::DynamicsSimulator_Trbdl_penalty > (mainlib, "DynamicsSimulator_Trbdl_impulse") // 1389
																							// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<const char *>())                                    // 1426
		.def("setParam_restitution_MA", (void (Trbdl ::DynamicsSimulator_Trbdl_impulse::*)(double r, double ma))&Trbdl ::DynamicsSimulator_Trbdl_impulse::setParam_restitution_MA) // 1445
		.def("stepSimulation", (void (Trbdl ::DynamicsSimulator_Trbdl_impulse::*)())&Trbdl ::DynamicsSimulator_Trbdl_impulse::stepSimulation) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_Trbdl_impulse // 1505
#endif //!defined (EXCLUDE_RBDL_SIMULATOR)                    // 1536
#if !defined (EXCLUDE_RBDL_SIMULATOR)                         // 1361
	class_<Trbdl ::DynamicsSimulator_Trbdl_QP , Trbdl::DynamicsSimulator_Trbdl_penalty> (mainlib, "DynamicsSimulator_Trbdl_QP") // 1389
																				  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<const char *>())                                    // 1426
		.def("getContactBases", (void (Trbdl ::DynamicsSimulator_Trbdl_QP::*)(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>& basis, double invfrictionCoef) const)&Trbdl ::DynamicsSimulator_Trbdl_QP::getContactBases) // 1445
		.def("calcContactBasisAll", (void (Trbdl ::DynamicsSimulator_Trbdl_QP::*)(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef))&Trbdl ::DynamicsSimulator_Trbdl_QP::calcContactBasisAll) // 1445
		.def("calcContactBoneIndex", (void (Trbdl ::DynamicsSimulator_Trbdl_QP::*)(int link_pair_count, intvectorn& boneIndex))&Trbdl ::DynamicsSimulator_Trbdl_QP::calcContactBoneIndex) // 1445
		.def("getNumContactLinkPairs", (int (Trbdl ::DynamicsSimulator_Trbdl_QP::*)() const)&Trbdl ::DynamicsSimulator_Trbdl_QP::getNumContactLinkPairs) // 1445
		.def("getLinkPairBodiesBones", (void (Trbdl ::DynamicsSimulator_Trbdl_QP::*)(intvectorn& ibodies, intvectorn& ibones) const)&Trbdl ::DynamicsSimulator_Trbdl_QP::getLinkPairBodiesBones) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_Trbdl_QP // 1505
#endif //!defined (EXCLUDE_RBDL_SIMULATOR)                    // 1536
	class_<OpenHRP ::DynamicsSimulator_TRL_LCP , OpenHRP ::DynamicsSimulator_TRL_penalty > (mainlib, "DynamicsSimulator_TRL_LCP") // 1389
																				  // : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def(init<bool>())                                            // 1426
		.def(init<const char *>())                                    // 1426
		.def("stepKinematic", (void (OpenHRP ::DynamicsSimulator_TRL_LCP::*)(int ichar, vectorn const& dq))&OpenHRP ::DynamicsSimulator_TRL_LCP::stepKinematic) // 1445
		.def("setParam_Epsilon_Kappa", (void (OpenHRP ::DynamicsSimulator_TRL_LCP::*)(double eps, double kap))&OpenHRP ::DynamicsSimulator_TRL_LCP::setParam_Epsilon_Kappa) // 1445
		.def("setParam_R_B_MA", (void (OpenHRP ::DynamicsSimulator_TRL_LCP::*)(double r, double b, double ma))&OpenHRP ::DynamicsSimulator_TRL_LCP::setParam_R_B_MA) // 1445
		.def("setDamping", (void (OpenHRP ::DynamicsSimulator_TRL_LCP::*)(double linearDamping, double angularDamping))&OpenHRP ::DynamicsSimulator_TRL_LCP::setDamping) // 1445
		.def("setDamping", (void (OpenHRP ::DynamicsSimulator_TRL_LCP::*)())&OpenHRP ::DynamicsSimulator_TRL_LCP::setDamping) // 1445
		.def("getCOMbasedContactForce", (Liegroup ::dse3 (OpenHRP ::DynamicsSimulator_TRL_LCP::*)(int ichar, int ibone))&OpenHRP ::DynamicsSimulator_TRL_LCP::getCOMbasedContactForce) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_TRL_LCP // 1505
#ifdef USE_SOFTBODY
	class_<TRL ::DynamicsSimulator_TRL_softbody > (mainlib, "DynamicsSimulator_TRL_softbody") // 1389
																						// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<>())                                                // 1426
		.def("registerLBScharacter", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(VRMLloader* l, SkinnedMeshFromVertexInfo * info))&TRL ::DynamicsSimulator_TRL_softbody::registerLBScharacter) // 1445
		.def("registerCollisionCheckPair_Rigid_vs_LBS", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(int bodyIndex1, int treeIndex1, int bodyIndex2, vectorn const& param))&TRL ::DynamicsSimulator_TRL_softbody::registerCollisionCheckPair_Rigid_vs_LBS) // 1445
		.def("registerCollisionCheckPair_LBS_vs_LBS", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(int bodyIndex1, int bodyIndex2, vectorn const& param))&TRL ::DynamicsSimulator_TRL_softbody::registerCollisionCheckPair_LBS_vs_LBS) // 1445
		.def("init", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt))&TRL ::DynamicsSimulator_TRL_softbody::init) // 1445
		.def("setParam_Epsilon_Kappa", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(double eps, double kap))&TRL ::DynamicsSimulator_TRL_softbody::setParam_Epsilon_Kappa) // 1445
		.def("setParam_R_B_MA", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(double r, double b, double ma))&TRL ::DynamicsSimulator_TRL_softbody::setParam_R_B_MA) // 1445
		.def("stepSimulation", (void (TRL ::DynamicsSimulator_TRL_softbody::*)())&TRL ::DynamicsSimulator_TRL_softbody::stepSimulation) // 1445
		.def("addRelativeConstraint", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(int ichara, Bone& bone1,vector3 boneVector1,Bone& bone2, vector3 boneVector2))&TRL ::DynamicsSimulator_TRL_softbody::addRelativeConstraint) // 1445
		.def("removeRelativeConstraint", (void (TRL ::DynamicsSimulator_TRL_softbody::*)(int ichara, Bone& bone1, Bone& bone2))&TRL ::DynamicsSimulator_TRL_softbody::removeRelativeConstraint) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_TRL_softbody // 1505
#endif
	class_<OpenHRP ::DynamicsSimulator_TRL_QP > (mainlib, "DynamicsSimulator_TRL_QP") // 1389
																				// : number denotes the line number of luna_gen.lua that generated the sentence // 1392
		.def(init<const char *>())                                    // 1426
		.def("skipIntegration", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)())&OpenHRP ::DynamicsSimulator_TRL_QP::skipIntegration) // 1445
		.def("stepKinematic", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(int ichar, vectorn const& dq))&OpenHRP ::DynamicsSimulator_TRL_QP::stepKinematic) // 1445
		.def("stepKinematic2", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(int ichar, vectorn const& ddq))&OpenHRP ::DynamicsSimulator_TRL_QP::stepKinematic2) // 1445
		.def("setParam_Epsilon_Kappa", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(double eps, double kap))&OpenHRP ::DynamicsSimulator_TRL_QP::setParam_Epsilon_Kappa) // 1445
		.def("setParam_R_B_MA", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(double r, double b, double ma))&OpenHRP ::DynamicsSimulator_TRL_QP::setParam_R_B_MA) // 1445
		.def("getNumContactLinkPairs", (int (OpenHRP ::DynamicsSimulator_TRL_QP::*)() const)&OpenHRP ::DynamicsSimulator_TRL_QP::getNumContactLinkPairs) // 1445
		.def("getNumAllLinkPairs", (int (OpenHRP ::DynamicsSimulator_TRL_QP::*)())&OpenHRP ::DynamicsSimulator_TRL_QP::getNumAllLinkPairs) // 1445
		.def("getContactBases", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(std::vector<OpenHRP::DynamicsSimulator_QP::ContactBasis>& basis, double invfrictionCoef) const)&OpenHRP ::DynamicsSimulator_TRL_QP::getContactBases) // 1445
		.def("calcContactBasisAll", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(matrixn& v_all, matrixn & dot_v_all, int link_pair_count, double invfrictionCoef))&OpenHRP ::DynamicsSimulator_TRL_QP::calcContactBasisAll) // 1445
		.def("calcContactBoneIndex", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(int link_pair_count, intvectorn& boneIndex))&OpenHRP ::DynamicsSimulator_TRL_QP::calcContactBoneIndex) // 1445
		.def("getNumContactLinkPairs", (int (OpenHRP ::DynamicsSimulator_TRL_QP::*)() const)&OpenHRP ::DynamicsSimulator_TRL_QP::getNumContactLinkPairs) // 1445
		.def("getLinkPairBodiesBones", (void (OpenHRP ::DynamicsSimulator_TRL_QP::*)(intvectorn& ibodies, intvectorn& ibones) const)&OpenHRP ::DynamicsSimulator_TRL_QP::getLinkPairBodiesBones) // 1445
		; // end of class impl___pybindgen__Physics_DynamicsSimulator_TRL_QP // 1505
}

}

