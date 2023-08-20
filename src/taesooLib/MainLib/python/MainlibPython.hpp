// MainlibPython.cpp : Defines the entry point for the DLL application.
//

#include "stdafx.h"
#include "../../BaseLib/math/vector3.h"
#include "../../BaseLib/math/quater.h"
#include "../../BaseLib/math/quaterN.h"
#include "../../BaseLib/math/hyperMatrixN.h"
#include "../../BaseLib/motion/VRMLloader.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/FltkAddon.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/WrapperLua/luna.h"
#include "../../MainLib/WrapperLua/LUAwrapper.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "../../BaseLib/motion/MotionUtil.h"
#include "PythonExtendWin.h"
#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <numpy/ndarrayobject.h>

#ifdef USE_BOOST_PYTHON
#include <boost/python/numpy.hpp>
#include <boost/python/numpy/ndarray.hpp>
#else
using namespace pybind11;
#include <pybind11/operators.h>
#endif

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

#include "MainlibPython.h"

#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"

void createMainWin(int, int, int ,int, float);
void createMainWin(int w, int h, int rw, int rh, float UIscaleFactor, const char* configFileName, const char* plugins_file, const char* ogre_config);
void showMainWin();
void startMainLoop();
PythonExtendWin* getPythonWin();
MotionPanel* g_motionPanel=NULL;
FrameSensor::Trigger* g_trigger=NULL;
FlLayout* g_layout=NULL;
int g_retVal=0;
namespace RE
{
	extern Globals* g_pGlobals;
}
void MainlibPythonSetGlobal(RE::Globals* g, MotionPanel* w, FrameSensor::Trigger* t, FlLayout* l)
{
	RE::g_pGlobals=g;
	g_motionPanel=w;
	g_trigger=t;
	g_layout=l;
}
int getOgreVersionMinor()
{
#ifdef NO_GUI
	return 7;
#else
	return OGRE_VERSION_MINOR;
#endif
}

MotionPanel* motionPanel()
{
	return g_motionPanel;
}

FlLayout* layout()
{
	return g_layout;
}

void setIntegerReturnValue(int i)
{
	g_retVal=i;
}

int getIntegerReturnValue()
{
	return g_retVal;
}

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

void screenToWorldXZPlane(vectorn const& cursorPos, vector3& worldPos)
{
	worldPos=RE::g_pGlobals->pFltkRenderer->screenToWorldXZPlane(cursorPos[0], cursorPos[1]);
}

void screenToWorldLine(vectorn const& cursorPos, vector3& lineStart, vector3& lineEnd)
{
	RE::g_pGlobals->pFltkRenderer->screenToWorldLine(cursorPos[0], cursorPos[1], lineStart, lineEnd);
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

MotionLoader* createMotionLoader(std::string const& filename)
{
	std::string ext=toUpper(tail(filename,3));
	if(ext=="BVH")
		return new BVHLoader(filename.c_str());
	else if(ext=="ASF")
		return new ASFLoader(filename.c_str());
	else if(ext=="MOT")
		return new MotionLoader(filename.c_str());		
	else ASSERT(0);

	return NULL;
}


#ifdef USE_BOOST_PYTHON
#include <boost/python.hpp>
using namespace boost::python;



// to handle default arguments
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(concat_member_overloads, Concat, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(range_member_overloads, range, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(range_member_overloads2, range, 2, 4)
#define RETURN_REFERENCE return_value_policy<reference_existing_object>()
#define TAKE_OWNERSHIP return_value_policy<manage_new_object>()
#else
#define RETURN_REFERENCE return_value_policy::reference
#define TAKE_OWNERSHIP return_value_policy::take_ownership
#endif
// wrapper functions..
void FrameSensor_triggerSet(FrameSensor& fs, float fTriggerTime)
{
	fs.triggerSet(g_trigger,fTriggerTime);
}

void vector3_assign(vector3& l, WRAP_PY::list ll) 
{
	if(len(ll)!=3) throw std::range_error("vector3_assign");
#ifdef USE_BOOST_PYTHON
	l.x=extract<double>(ll[0]);
	l.y=extract<double>(ll[1]);
	l.z=extract<double>(ll[2]);
#else
	l.x=ll[0].cast<double>();
	l.y=ll[1].cast<double>();
	l.z=ll[2].cast<double>();
#endif
}

void quater_assign(quater& l, WRAP_PY::list ll) 
{
	if(len(ll)!=4) throw std::range_error("quater_assign");
#ifdef USE_BOOST_PYTHON
	l.x=extract<double>(ll[0]);
	l.y=extract<double>(ll[1]);
	l.z=extract<double>(ll[2]);
	l.w=extract<double>(ll[3]);
#else
	l.x=ll[0].cast<double>();
	l.y=ll[1].cast<double>();
	l.z=ll[2].cast<double>();
	l.w=ll[3].cast<double>();
#endif
}

std::string quater_output(quater& q)
{
	return std::string(q.output().ptr());
}



std::string get_Widget_mId(FlLayout::Widget const& w)
{
	return w.mId.ptr();
}

std::string get_Widget_mType(FlLayout::Widget const& w)
{
	return w.mType.ptr();
}
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

#ifdef USE_BOOST_PYTHON
#define MDEF def
#define MCOMMA
#define BASES(x) bases<x>
#define NONCOPYABLE ,boost::noncopyable
BOOST_PYTHON_MODULE(libmainlib)
#else
#define MDEF m.def
#define MCOMMA m,
#define BASES(x) x
#define NONCOPYABLE
#define add_property def_property
PYBIND11_MODULE(libmainlib, m)
#endif
{
	void (*createMainWin1)(int, int,int, int, float)=&createMainWin;
	void (*createMainWin2)(int, int,int, int, float, const char*, const char*, const char*)=&createMainWin;
	initialize(); // import_array
	//boost::python::numpy::array::set_module_and_type("numpy", "ndarray");
	MDEF("motionPanel", motionPanel, RETURN_REFERENCE);
	MDEF("layout", layout, RETURN_REFERENCE);
	MDEF("setReturnValue", setIntegerReturnValue);
	MDEF("viewLock", viewLock);
	MDEF("viewUnlock", viewUnlock);
	MDEF("getOgreVersionMinor", getOgreVersionMinor);
	MDEF("screenToWorldLine", screenToWorldLine);
	MDEF("createMotionLoader", createMotionLoader, TAKE_OWNERSHIP );
	MDEF("createMainWin", createMainWin1);
	MDEF("createMainWin", createMainWin2);
	MDEF("showMainWin", showMainWin);
	MDEF("startMainLoop", startMainLoop);
	MDEF("getPythonWin", getPythonWin, RETURN_REFERENCE);
	/////////////////////////////////////////////////////////////////
	// Baselib
	/////////////////////////////////////////////////////////////////

    
	// vector3

	void (vector3::*add1)(const vector3&, const vector3&) =&vector3::add;
	void (vector3::*add2)(const vector3&) =&vector3::add;

	void (vector3::*sub1)(const vector3&, const vector3&) =&vector3::sub;
	void (vector3::*cross1)(const vector3&, const vector3&) =&vector3::cross;
	void (vector3::*sub2)(const vector3&) =&vector3::sub;


	class_<vector3>(MCOMMA"vector3")
		.def(init<>())
		.def(init<m_real, m_real, m_real>())
		.def_readwrite("x", &vector3::x)
		.def_readwrite("y", &vector3::y)
		.def_readwrite("z", &vector3::z)

/*		.def("add", add1)
		.def("add", add2)*/


		.def("add", (void (vector3::*)(const vector3&, const vector3&) )&vector3::add)

		.def("sub", sub1)
		.def("sub", sub2)
		.def("cross", cross1)
		.def("multadd", &vector3::multadd)
		.def("output", &vector3::output)
		.def("length", &vector3::length)
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self * self) // mul
		.def(self * double()) // mul
		.def(quater()*self)
		.def("assign", &vector3_assign)
		.def("interpolate", &vector3::interpolate)
		.def("difference", &vector3::difference)
		.def("rotationVector", &vector3::rotationVector)
	;

	void (quater::*setRotation1)(const vector3& axis, m_real angle)=&quater::setRotation;
	void (quater::*setRotation2)(const vector3& rotationVector)=&quater::setRotation;
	m_real (quater::*rotationAngle1)(void) const=&quater::rotationAngle;
	void (quater::*normalize1)()=&quater::normalize;

	class_<quater>(MCOMMA"quater")
		.def(init<m_real, m_real, m_real, m_real>())
		.def(init<m_real, const vector3&>())
		.def_readwrite("x", &quater::x)
		.def_readwrite("y", &quater::y)
		.def_readwrite("z", &quater::z)
		.def_readwrite("w", &quater::w)
		.def("slerp", &quater::slerp)
		.def("safeSlerp", &quater::safeSlerp)
		.def("decomposeTwistTimesNoTwist", &quater::decomposeTwistTimesNoTwist)
		.def("decomposeNoTwistTimesTwist", &quater::decomposeNoTwistTimesTwist)
		.def("scale", &quater::scale)
		.def(-self) // neg (unary minus)
		.def(self + self) // add (homogeneous)
		.def(self * self) // mul
		.def("output", &quater_output)
		.def("length", &quater::length)
		.def("rotationAngle", rotationAngle1)
		.def("rotationAngleAboutAxis", &quater::rotationAngleAboutAxis)
		.def("assign", &quater_assign)
		.def("leftMult", &quater::leftMult)
		.def("setRotation", setRotation1)
		.def("setRotation", setRotation2)
		.def("normalize", normalize1)
		.def("align", &quater::align)
		.def("difference", &quater::difference)
	;
	
	{
		// transf
		class_<transf>(MCOMMA"transf")
			.def(init<quater const&, vector3 const&>())
			.def_readwrite("rotation", &transf::rotation)
			.def_readwrite("translation", &transf::translation)
			;
	}
	// matrix4
	{
		struct wrap_matrix4
		{
			static void matrix4_assign(matrix4& l, WRAP_PY::list ll) 
			{
				if(len(ll)!=4) throw std::range_error("matrix4_assign");
#ifdef USE_BOOST_PYTHON
				l._11=extract<double>(ll[0]);
				l._12=extract<double>(ll[1]);
				l._13=extract<double>(ll[2]);
				l._14=extract<double>(ll[3]);
				l._21=extract<double>(ll[4]);
				l._22=extract<double>(ll[5]);
				l._23=extract<double>(ll[6]);
				l._24=extract<double>(ll[7]);
				l._31=extract<double>(ll[8]);
				l._32=extract<double>(ll[9]);
				l._33=extract<double>(ll[10]);
				l._34=extract<double>(ll[11]);
				l._41=extract<double>(ll[12]);
				l._42=extract<double>(ll[13]);
				l._43=extract<double>(ll[14]);
				l._44=extract<double>(ll[15]);
#else
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
#endif
			}

			//a.assign([0 1 2 3 4 4 5 5 6 6 7 ])

		};
		
		class_<matrix4>(MCOMMA"matrix4")
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
		;
	}


	{
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
		void	(matrixn::*setAllValue)(m_real d)=&matrixn::setAllValue;
		matrixnView (matrixn::*range)(int startr, int endr, int startc, int endc)=&matrixn::range;
		class_<matrixn>(MCOMMA"matrixn")
			.def(init<>())
			.def(init<int,int>())
#ifdef USE_BOOST_PYTHON
			.def("ref", &matrixn_::ref)
#else
			.def("ref",  [](matrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(matrixn_::ref(v)); })
#endif
			.def("row", &matrixn::row)
			.def("column", &matrixn::column)
			.def("rows", &matrixn::rows)
			.def("cols", &matrixn::cols)
			.def("set", &matrixn::set)
			.def("setSize", &matrixn::setSize)
			.def("setAllValue", setAllValue)
			.def("get", &matrixn::getValue)
#ifdef USE_BOOST_PYTHON
			.def("range", range, range_member_overloads2())
#else
			.def("range", static_cast<matrixnView (matrixn::*)(int, int, int, int)>(&matrixn::range), "startRow"_a, "endRow"_a,"startColumn"_a=0,"endColumn"_a=INT_MAX)
#endif
		;
#ifdef USE_BOOST_PYTHON
		class_<matrixnView, BASES(matrixn) >(MCOMMA"matrixnView", init<double*, int,int,int>())
#else
		class_<matrixnView, BASES(matrixn) >(MCOMMA"matrixnView")
#endif
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
		class_<hypermatrixn>(MCOMMA"hypermatrixn")
			.def(init<>())
			.def(init<int, int, int>())
			.def("pages", page1)
			.def("rows",&hypermatrixn::rows)
			.def("cols",&hypermatrixn::cols)
			.def("setSize",&hypermatrixn::setSize)
			.def("setSameSize",&hypermatrixn::setSameSize)
			.def("page",&wrap_hyper::page)
#ifdef USE_BOOST_PYTHON
			.def("ref", &wrap_hyper::ref)
#else
			.def("ref",  [](hypermatrixn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(wrap_hyper::ref(v)); })
#endif
		;
	}
	// quaterN
	{
		void (quaterN::*assign1)(const quaterN& other)=&quaterN::assign;
		class_<quaterN>(MCOMMA"quaterN")
			.def("value", &quaterN::value, RETURN_REFERENCE)
			.def("row", &quaterN::row, RETURN_REFERENCE)
			.def("row", &quaterN::rows)
			.def("size", &quaterN::size)
#ifdef USE_BOOST_PYTHON
			.def("range", &quaterN::range, range_member_overloads())
#else
			.def("range", static_cast<quaterNView (quaterN::*)(int, int, int)>(&quaterN::range), "start"_a, "end"_a,"step"_a=1)
#endif
			.def("assign", assign1)
		;

		class_<quaterNView, BASES(quaterN) >(MCOMMA"quaterNView");
	}

	// vector3N
	{
		void (vector3N::*assign1)(const vector3N& other)=&vector3N::assign;
		class_<vector3N>(MCOMMA"vector3N")
			.def("value", &vector3N::value, RETURN_REFERENCE)
			.def("row", &vector3N::row, RETURN_REFERENCE)
			.def("row", &vector3N::rows)
			.def("size", &vector3N::size)
#ifdef USE_BOOST_PYTHON
			.def("range", &vector3N::range, range_member_overloads())
#else
			.def("range", static_cast<vector3NView (vector3N::*)(int, int, int)>(&vector3N::range), "start"_a, "end"_a,"step"_a=1)
#endif
			.def("assign", assign1)
		;

		class_<vector3NView, BASES(vector3N) >(MCOMMA"vector3NView");
	}

	// intvectorn 
	{
		struct intvectorn_
		{
			static void assign(intvectorn & l, WRAP_PY::list ll) 
			{
#ifdef USE_BOOST_PYTHON
				l.setSize(len(ll));

				for(int i=0; i<len(ll); i++)
					l[i]=extract<int>(ll[i]);
#endif
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
		int	(BinaryFile::*unpackInt1)()=&BinaryFile::unpackInt;
		double	(BinaryFile::*unpackFloat1)()=&BinaryFile::unpackFloat;
		void	(BinaryFile::*pack1)(const vectorn& )=&BinaryFile::pack;
		void	(BinaryFile::*pack2)(const char*)=&BinaryFile::pack;
		void	(BinaryFile::*pack3)(const matrixn&)=&BinaryFile::pack;
		void	(BinaryFile::*pack4)(const hypermatrixn&)=&BinaryFile::pack;
		void	(BinaryFile::*unpack1)(vectorn& )=&BinaryFile::unpack;
		void	(BinaryFile::*unpack3)(matrixn&)=&BinaryFile::unpack;
		void	(BinaryFile::*unpack4)(hypermatrixn&)=&BinaryFile::unpack;
		class_<BinaryFile>(MCOMMA"BinaryFile")
			.def(init<bool, const char*>())
			.def(init<>())
		   .def("openRead", &BinaryFile::openRead)
		   .def("openWrite", &BinaryFile::openWrite)
		   .def("close", &BinaryFile::close)
		   .def("packInt", &BinaryFile::packInt)
		   .def("packFloat", &BinaryFile::packFloat)
		   .def("pack", pack1)
		   .def("pack", pack2)
		   .def("pack", pack3)
		   .def("pack", pack4)
		   .def("unpack", unpack1)
		   .def("unpack", unpack3)
		   .def("unpack", unpack4)
		   .def("unpackInt", unpackInt1)
		   .def("unpackFloat", unpackFloat1)
		   ;
		void (intvectorn::*setValue1)( int i, int d )=&intvectorn::setValue;
		int (intvectorn::*getValue1)( int i ) const=&intvectorn::getValue;
		intvectornView (intvectorn::*range)(int start, int end, int step)=&intvectorn::range;
		class_<intvectorn>(MCOMMA"intvectorn")
			.def(init<>())
			.def("assign", &intvectorn_::assign)
			.def("value", getValue1)
			.def("get", getValue1)
			.def("set", setValue1)
			.def("size", &intvectorn::size)
			.def("setSize", &intvectorn::setSize)
			.def("resize", &intvectorn::resize)
#ifdef USE_BOOST_PYTHON
			.def("range", range, range_member_overloads())
#else
			.def("range", static_cast<intvectornView (intvectorn::*)(int, int, int)>(&intvectorn::range), "start"_a, "end"_a,"step"_a=1)
#endif
#ifdef USE_BOOST_PYTHON
			.def("ref", &intvectorn_::ref) 
#else
			.def("ref",  [](intvectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(intvectorn_::ref(v)); })
#endif
			.def("output", &intvectorn_::output)
			.def("colon", &intvectorn::colon, RETURN_REFERENCE)
		;
	}

	// vectorn
	{
		struct vectorn_
		{
			static void vectorn_assign(vectorn & l, WRAP_PY::list ll) 
			{
#ifdef USE_BOOST_PYTHON
				l.setSize(len(ll));

				for(int i=0,ni=len(ll); i<ni; i++)
					l[i]=extract<double>(ll[i]);
#else
				l.setSize(len(ll));

				for(int i=0,ni=len(ll); i<ni; i++)
					l[i]=ll[i].cast<double>();
#endif
			}
			static m_real & vectorn_value(vectorn& v, int i)
			{
				return v[i];
			}

			static std::string vectorn_output(vectorn const& q)
			{
				return q.output().ptr();
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
		m_real (vectorn::*minimum1)() const =&vectorn::minimum;
		m_real (vectorn::*maximum1)() const =&vectorn::maximum;
		void (vectorn::*setValue1)( int i, m_real d )=&vectorn::setValue;
		void	(vectorn::*setAllValue)(m_real d)=&vectorn::setAllValue;
		m_real (vectorn::*getValue1)( int i ) const=&vectorn::getValue;
		vectornView (vectorn::*range)(int start, int end, int step)=&vectorn::range;
		class_<vectorn>(MCOMMA"vectorn")
			.def(init<>())
			.def(init<int>())
			.def("assign", &vectorn_::vectorn_assign)
			.def("assign", assignv, RETURN_REFERENCE)
			.def("assign", assignq, RETURN_REFERENCE)
#ifdef USE_BOOST_PYTHON
			.def("ref", &vectorn_::ref) 
#else
			.def("ref",  [](vectorn const& v){ return WRAP_PY::reinterpret_steal<WRAP_PY::object>(vectorn_::ref(v)); })
#endif
			//.def("tolist", &vectorn_::tolist) 
			.def("minimum", minimum1)
			.def("maximum", maximum1)
			.def("value", getValue1)
			.def("get", getValue1)
			.def("set", setValue1)
			.def("getStride", &vectorn::_getStride)
			.def("setAllValue", setAllValue)
			.def("size", &vectorn::size)
			.def("setSize", &vectorn::setSize)
			.def("resize", &vectorn::resize)
#ifdef USE_BOOST_PYTHON
			.def("range", range, range_member_overloads())
#else
			.def("range", static_cast<vectornView (vectorn::*)(int, int, int)>(&vectorn::range), "start"_a, "end"_a,"step"_a=1)
#endif
			.def("length", &vectorn::length)
			.def("sum", &vectorn::sum)
			.def("squareSum", &vectorn::squareSum)
			.def("avg", &vectorn::avg)
			.def("argMin", &vectorn::argMin)
			.def("argMax", &vectorn::argMax)
			.def("output", &vectorn_::vectorn_output)
			.def("argNearest", &vectorn::argNearest)
			.def("colon", &vectorn::colon, RETURN_REFERENCE)
			.def("uniform", &vectorn::uniform, RETURN_REFERENCE)
			.def("linspace", &vectorn::linspace, RETURN_REFERENCE)
			//.def("normalize", &vectorn::normalize, RETURN_REFERENCE)
			.def("negate", &vectorn::negate, RETURN_REFERENCE)
			.def(-self) // neg (unary minus)
			.def(self + self) // add (homogeneous)
			.def(self * self) // mul
		;
#ifdef USE_BOOST_PYTHON
		class_<vectornView, BASES(vectorn) >(MCOMMA"vectornView", init<double*, int,int>())
#else
		class_<vectornView, BASES(vectorn) >(MCOMMA"vectornView")
			.def(init<double*, int, int>())
#endif
			.def(init<const vectorn &>())
			.def(init<const vectornView &>())
			;
	}

	/////////////////////////////////////////////////////////////////
	// Mainlib
	/////////////////////////////////////////////////////////////////

	MDEF("motionPanel", motionPanel, RETURN_REFERENCE);

	// functions in namespace RE
	{
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
		def("createFrameSensor", RE::createFrameSensor, RETURN_REFERENCE);
		def("createEntity", SceneNode_Wrapper::createEntity);
		def("removeEntity", SceneNode_Wrapper::removeEntity);		
		def("generateUniqueName", RE::generateUniqueName);
		*/
	}

	MDEF("transitionCost", MotionUtil::transitionCost);
	

#ifndef NO_GUI
#ifdef USE_BOOST_PYTHON
	class_<FltkMotionWindow NONCOPYABLE>("FltkMotionWindow", init<int, int, int>())
#else
	class_<FltkMotionWindow NONCOPYABLE>(MCOMMA"FltkMotionWindow")
		.def(init<int, int, int>())
#endif
		.def("addSkin",&FltkMotionWindow::addSkin)
		.def("releaseAllSkin",&FltkMotionWindow::releaseAllSkin)
		.def("getCurrFrame", &FltkMotionWindow::getCurrFrame)
		.def("getNumFrame", &FltkMotionWindow::getNumFrame)
		.def("getNumSkin", &FltkMotionWindow::getNumSkin)
		.def("getSkin", &FltkMotionWindow::getSkin, RETURN_REFERENCE)
		.def("changeCurrFrame",&FltkMotionWindow::changeCurrFrame)
		.def("playUntil",&FltkMotionWindow::playUntil)
		.def("playFrom",&FltkMotionWindow::playFrom)
	;
#endif

#ifdef USE_BOOST_PYTHON
	class_<MotionPanel NONCOPYABLE>(MCOMMA"MotionPanel", init<int,int,int,int>())
#else
	class_<MotionPanel NONCOPYABLE>(MCOMMA"MotionPanel")
		.def( init<int,int,int,int>())
#endif
#ifndef NO_GUI
		.def("motionWin", &MotionPanel::motionWin, RETURN_REFERENCE)
		.def("currMotion", &MotionPanel::currMotion, RETURN_REFERENCE)
		.def("hasPairMotion", &MotionPanel::hasPairMotion)
		.def("currPairMotion", &MotionPanel::currPairMotion, RETURN_REFERENCE)
		.def("numMotion", &MotionPanel::numMotion)
		.def("motion", &MotionPanel::motion, RETURN_REFERENCE)
#endif
	;

	void (Motion::*initEmpty1)(const Motion&, int) =&Motion::InitEmpty;
	void (Motion::*init1)(const Motion&, int, int) =&Motion::Init;
	int (Motion::*numFrames1) () const=&Motion::numFrames;
	void (Motion::*setDiscontinuity1)(int, bool) =&Motion::setDiscontinuity;
	
	class_<Motion>(MCOMMA"Motion")
		.def(init<MotionLoader*>())
		.def(init<const Motion&, int, int>())
		.def(init<Motion const&>())
		.def("length",&Motion::length)	
		.def("changeLength", &Motion::changeLength)
		.def("numFrames", numFrames1)  
		.def("resize", &Motion::Resize)
		.def("skeleton", &Motion::skeleton, RETURN_REFERENCE) // MotionLoader????
		.def("setSkeleton", &Motion::setSkeleton) // mot.setSkeleton(12) mot.skeleton().
		.def("setPose", &Motion::setPose)
		.def("empty", &Motion::empty)
		.def("initEmpty", initEmpty1)
		.def("init", init1)
		.add_property("identifier", &Motion::GetIdentifier, &Motion::SetIdentifier)
		.def("exportMot", &Motion::exportMOT)
		.def("samplePose",&Motion::samplePose)
#ifdef USE_BOOST_PYTHON
		.def("concat", &Motion::Concat, concat_member_overloads())	// to handle default arguments
#else
		.def("concat", &Motion::Concat, "pAdd"_a, "startFrame"_a=0, "endFrame"_a=INT_MAX, "bTypeCheck"_a=true)
#endif
		.def("numRotJoint", &Motion::numRotJoints)
		.def("numTransJoint", &Motion::numTransJoints)
		.def("totalTime", &Motion::totalTime)
		.def("frameRate", &Motion::frameRate)
		.def("isDiscontinuous", &Motion::isDiscontinuous)
		.def("setDiscontinuity", setDiscontinuity1)
		.def("pose", &Motion::pose, RETURN_REFERENCE)
		.def("calcInterFrameDifference",&Motion::CalcInterFrameDifference)
		.def("reconstructFromInterFrameDifference", &Motion::ReconstructDataByDifference)
	;

	void (Posture::*blend1)(const Posture&, const Posture&,m_real)=&Posture::Blend;

	class_<Posture>(MCOMMA"Pose")
		.def("init", &Posture::Init)
		.def("numRotJoint", &Posture::numRotJoint)
		.def("numTransJont",&Posture::numTransJoint)
		.def("clone", &Posture::clone, RETURN_REFERENCE)
		.def("blend", blend1)
		.def("align", &Posture::Align)
		.def("front",&Posture::front)
		.def("decomposeRot",&Posture::decomposeRot)
		.def_readwrite("translations", &Posture::m_aTranslations)
		.def_readwrite("rotations", &Posture::m_aRotations)
		.def_readwrite("constraint", &Posture::constraint)
		.def_readwrite("dv", &Posture::m_dv)
		.def_readwrite("dq", &Posture::m_dq)
		.def_readwrite("offset_y", &Posture::m_offset_y)
		.def_readwrite("offset_q", &Posture::m_offset_q)
		.def_readwrite("rotAxis_y", &Posture::m_rotAxis_y)
	;
	{
		void (TStrings::*set)(int i, const char* v)=&TStrings::set;
		void (TStrings::*pushback)(const char* v)=&TStrings::pushBack;
		class_<TStrings>(MCOMMA"TStrings")
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
		};
		void (MotionLoader::*setPose1)(const Posture& pose) const=&MotionLoader::setPose;
		void (MotionLoader::*setChain2)(const Posture& pose, Bone& bone) const=&MotionLoader::setChain;
		class_<MotionLoader NONCOPYABLE>(MCOMMA"MotionLoader")
			.def_readwrite("mMotion", &MotionLoader::m_cPostureIP)
			.def("readJointIndex", &MotionLoader::readJointIndex)
			.def("numRotJoint", &MotionLoader::numRotJoint)
			.def("numTransJoint", &MotionLoader::numTransJoint)
			.def("numBone", &MotionLoader::numBone)
			.def("setPose", setPose1)
			.def("setChain", setChain2)
			.def("getTreeIndexByName", &MotionLoader::getTreeIndexByName)
			.def("getBoneByTreeIndex", &MotionLoader::getBoneByTreeIndex, RETURN_REFERENCE)
			.def("bone", &MotionLoader::getBoneByTreeIndex, RETURN_REFERENCE)
			.def("getTreeIndexFromRotJointIndex", &MotionLoader::getTreeIndexByRotJointIndex)
			.def("getTreeIndexFromTransJointIndex", &MotionLoader::getTreeIndexByTransJointIndex)
			.def("scale", &MotionLoader::Scale)
			.def("getLinks", &MotionLoaderWrapper::getLinks)
		;	
		class_<VRMLloader, BASES(MotionLoader) >(MCOMMA"VRMLloader")
			;
	}

	// bone
	{	
		struct Bone_wrapper
		{
			static bool isChildHeadValid(Bone& bone)   { return bone.m_pChildHead!=NULL;}
			static bool isChildTailValid(Bone& bone)   { return bone.m_pChildTail!=NULL;}
			static bool isSiblingValid(Bone& bone)   { return bone.m_pSibling!=NULL;}

			static Bone& childHead(Bone& bone)	{ return *((Bone*)bone.m_pChildHead);}
			static Bone& childTail(Bone& bone)	{ return *((Bone*)bone.m_pChildTail);}
			static Bone& sibling(Bone& bone)	{ return *((Bone*)bone.m_pSibling);}
			static std::string name(const Bone& bone)	{ return std::string(bone.name().ptr());}
		};

		vector3 const& (Bone::*getTranslation1)() const=&Bone::getTranslation;
		quater const& (Bone::*getRotation1)() const=&Bone::getRotation;
		class_<Bone NONCOPYABLE> (MCOMMA"Bone")
			.def("getFrame", &Bone::_getFrame, RETURN_REFERENCE)
			.def("length", &Bone::length)
			.def("name", &Bone_wrapper::name)
			.def("getOffset", &Bone::getOffset, RETURN_REFERENCE)
			.def("getTranslation", getTranslation1, RETURN_REFERENCE)
			.def("getRotation", getRotation1, RETURN_REFERENCE)
			.def("childHead", &Bone_wrapper::childHead, RETURN_REFERENCE)
			.def("childTail", &Bone_wrapper::childTail, RETURN_REFERENCE)
			.def("sibling", &Bone_wrapper::sibling, RETURN_REFERENCE)
			.def("isChildHeadValid", &Bone_wrapper::isChildHeadValid)
			.def("isChildTailValid", &Bone_wrapper::isChildTailValid)
			.def("isSiblingValid", &Bone_wrapper::isSiblingValid)			
		;
	}

	// PLDPrimSkin
	{		
		class_<PLDPrimSkin NONCOPYABLE>(MCOMMA"PLDPrimSkin")
			.add_property("visible", &PLDPrimSkin::GetVisible, &PLDPrimSkin::SetVisible)
			.def("setTranslation",&PLDPrimSkin::SetTranslation)
			.def("setPose", &PLDPrimSkin::SetPose)
			.def("applyAnim", &PLDPrimSkin::ApplyAnim)
		;
	}

	// FrameSensor
	{
		void (FrameSensor::*connect1) (Motion* pMotion, PLDPrimSkin* pSkin, bool)=&FrameSensor::connect;

		// create FrameSensor using createFrameSensor() instead of FrameSensor()
		class_<FrameSensor NONCOPYABLE>(MCOMMA"FrameSensor")
			.def("connect", connect1)
			//.def("initAnim", &FrameSensor::InitAnim)
			.def("triggerSet", &FrameSensor_triggerSet)
		;
	}
	
	// FlLayout
	{
		Fl_Widget* (FlLayout::*create1)(const char* type, const char* id, const char* title)=&FlLayout::create;
		Fl_Widget* (FlLayout::*create2)(const char* type, const char* id, const char* title, int startSlot, int endSlot, int height)=&FlLayout::create;
		class_<FlLayout NONCOPYABLE>(MCOMMA"FlLayout")
			.def("create", create1,RETURN_REFERENCE)
			.def("create", create2,RETURN_REFERENCE)
			.def("newLine", &FlLayout::newLine)
			.def("setLineSpace", &FlLayout::setWidgetHeight)
			.def("setWidgetPos", &FlLayout::setWidgetPos)
			.def("setUniformGuidelines", &FlLayout::setUniformGuidelines)
			.def("updateLayout", &FlLayout::updateLayout)
			.def("redraw", &FlLayout::redraw)
			.def("minimumHeight", &FlLayout::minimumHeight)
			.def("widget", &FlLayout::widgetRaw,RETURN_REFERENCE)
			.def("widgetIndex", &FlLayout::widgetIndex)
			.def("findWidget", &FlLayout::findWidget,RETURN_REFERENCE)	
		;
	
		class ButtonWrapper
		{
			enum {CHECK_BUTTON, BUTTON};
			int type;
			Fl_Button* ptr;
		public:
			ButtonWrapper(){}
			ButtonWrapper(int type, Fl_Button* ptr){this->type=type; this->ptr=ptr;}
			static ButtonWrapper checkButton(FlLayout::Widget& w)	{ return ButtonWrapper(CHECK_BUTTON, w.checkButton());}
			static ButtonWrapper button(FlLayout::Widget& w)		{ return ButtonWrapper(BUTTON, w.button());}

			void setButtonValue(bool bValue)						{ ((Fl_Button*)(ptr))->value(bValue);}
			bool getButtonValue() const								{ return ((Fl_Button*)(ptr))->value();}
		};

		class ValuatorWrapper
		{
			Fl_Valuator* ptr;
		public:
			ValuatorWrapper(){}
			ValuatorWrapper(Fl_Valuator* ptr){this->ptr=ptr;}
			static ValuatorWrapper valuator(FlLayout::Widget& w)	{ return ValuatorWrapper (w.valuator());}			

			double getvalue() const									{ return ptr->value();}
			void setvalue(double a)									{ ptr->value(a);}
			void range(double a, double b)							{ ptr->range(a,b);}
#ifndef NO_GUI
			void step(double s)										{ ptr->step(s);}
			double getstep() const									{ return ptr->step();}
#endif
  		};

		class MenuWrapper
		{
			FlChoice* ptr;
		public:
			MenuWrapper(){}
			MenuWrapper(FlChoice* ptr){this->ptr=ptr;}
			static MenuWrapper menu(FlLayout::Widget& w)			{ return MenuWrapper(w.menu());}
		};

		class_<FlLayout::Widget>(MCOMMA"Widget")
#ifdef USE_BOOST_PYTHON
			.add_property("mId", &get_Widget_mId)
			.add_property("mType", &get_Widget_mType)
#endif
			//.def("button", &ButtonWrapper::button)
			//.def("checkButton", &ButtonWrapper::checkButton)
			//.def("lightButton", &ButtonWrapper::checkButton)
			//.def("valuator", &ValuatorWrapper::valuator)
			//.def("slider", &ValuatorWrapper::valuator)
			//.def("menu", &MenuWrapper::menu)
		;

		/*
		class_<ButtonWrapper>(MCOMMA"Fl_Button")
			.def("value", &ButtonWrapper::getButtonValue)
			.def("value", &ButtonWrapper::setButtonValue)
		;

		class_<ValuatorWrapper>(MCOMMA"Fl_Valuator")
			.def("value", &ValuatorWrapper::getvalue)
			.def("value", &ValuatorWrapper::setvalue)
			.def("range", &ValuatorWrapper::range)
			.def("step", &ValuatorWrapper::step)
			.def("step", &ValuatorWrapper::getstep)
		;
		*/

	}

#ifndef NO_OGRE
	// LineSegment
	{
		void (LineSegment::*addCtrlPoint2)(const vector3& v)=&LineSegment::addCtrlPoint;
		class_<LineSegment NONCOPYABLE>(MCOMMA"LineSegment")
			.def("begin", &LineSegment::begin)
			.def("row", &LineSegment::row)
			.def("end", &LineSegment::end)
			.def("addCtrlPoint", addCtrlPoint2)
			.def("numCtrlPoint", &LineSegment::numCtrlPoint)
			.def("setDrawRange", &LineSegment::SetDrawRange)
			.def("setThickness", &LineSegment::SetThickness)
			.def("setVisible", &LineSegment::SetVisible)
			.def("remove", &LineSegment::remove)
		;
	}
#endif
	enum_<handle_message>(MCOMMA"handle")
		.value("FRAME_MOVE", M_FRAME_MOVE)
		.value("TRIGGERED", M_TRIGGERED)
		.value("ON_DRAW",M_ON_DRAW)
		.value("CALLBACK", M_CALLBACK)
		.value("HANDLE", M_HANDLE)
	;

	enum_<Fl_Event>(MCOMMA"event")
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
		class_<lunaStack NONCOPYABLE>(MCOMMA"lunaStack")
		;
	}

	{
		struct PythonExtendWin_wrapper
		{
			static void getglobal(PythonExtendWin& l, const char* key){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				lua_pushstring(l.L, key);
				lua_gettable(l.L,LUA_GLOBALSINDEX); // stack top becomes _G[key] 
				if (lua_isnil(l.L, -1)) luaL_error(l.L, "missing global: %s", key);
			}
			static void getglobalNoCheck(PythonExtendWin& l, const char* key){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				lua_pushstring(l.L, key);
				lua_gettable(l.L,LUA_GLOBALSINDEX); // stack top becomes _G[key] 
			}
			static void replaceTop(PythonExtendWin& l, const char* key){
				lua_State *L=l.L;
				if (!lua_istable(L,-1)) luaL_error(L, "Luna<>::replaceTop: non-table object cannot be accessed");
				lua_pushstring(L, key);
				lua_gettable(L, -2);
				lua_insert(L, -2);  // swap table and value 
				lua_pop(L,1); // pop-out prev table
			}
			static void insert(PythonExtendWin& l, int index)
			{
				lua_State *L=l.L;
				if (!lua_istable(L,-1)) luaL_error(L, "Luna<>::insert: non-table object cannot be accessed");
				lua_insert(L,index);
			}
			static void replaceTop2(PythonExtendWin& l, int index){
				lua_State *L=l.L;
				if (!lua_istable(L,-1)) luaL_error(L, "Luna<>::replaceTop: non-table object cannot be accessed");
				lua_pushnumber(L, index);
				lua_gettable(L, -2);
				lua_insert(L, -2);  // swap table and value 
				lua_pop(L,1); // pop-out prev table
			}
			static void getglobal2(PythonExtendWin& l, const char* key1, const char* key2){
				Msg::verify(l.L, "PythonExtendWin::L is NULL");
				getglobal(l, key1);
				replaceTop(l, key2);
				if (lua_isnil(l.L, -1)) luaL_error(l.L, "missing global: %s.%s", key1, key2);
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
		};
#ifdef USE_BOOST_PYTHON
		class_<PythonExtendWin NONCOPYABLE>("PythonExtendWin",  init<int, int, int, int, MotionPanel&, FltkRenderer&>())
#else
		class_<PythonExtendWin NONCOPYABLE>(MCOMMA"PythonExtendWin")
			.def( init<int, int, int, int, MotionPanel&, FltkRenderer&>())
#endif
			.def("loadScript", &PythonExtendWin::__loadScript)
			.def("loadEmptyScript", &PythonExtendWin::__loadEmptyScript)
			.def("releaseScript", &ScriptWin::releaseScript)
			.def("dofile", &PythonExtendWin::dofile)
			.def("dostring", &PythonExtendWin::dostring)
			.def("isLuaReady", &PythonExtendWin_wrapper::isLuaReady)
			.def("push", &PythonExtendWin_wrapper::push1)
			.def("push", &PythonExtendWin_wrapper::push2)
			.def("push", &PythonExtendWin_wrapper::push22)
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
			.def("push", &PythonExtendWin_wrapper::push4)
			.def("push", &PythonExtendWin_wrapper::push_hypermatrixn)
			.def("pushBoolean", &PythonExtendWin_wrapper::push3)
			.def("call", &PythonExtendWin_wrapper::call, TAKE_OWNERSHIP)
			.def("getglobal", &PythonExtendWin_wrapper::getglobal)
			.def("getglobal", &PythonExtendWin_wrapper::getglobal2)
			.def("getglobalNoCheck", &PythonExtendWin_wrapper::getglobalNoCheck)
			.def("getglobalNoCheck", &PythonExtendWin_wrapper::getglobal2NoCheck)
			.def("getMemberFunc", &PythonExtendWin_wrapper::getMemberFunc)
			.def("insert", &PythonExtendWin_wrapper::insert)
			.def("replaceTop", &PythonExtendWin_wrapper::replaceTop)
			.def("replaceTop", &PythonExtendWin_wrapper::replaceTop2)
			.def("printStack", &PythonExtendWin_wrapper::printStack)
  			.def("popmatrixn", &PythonExtendWin_wrapper::popmatrixn, RETURN_REFERENCE)
  			.def("pophypermatrixn", &PythonExtendWin_wrapper::pophypermatrixn, RETURN_REFERENCE)
  			.def("popvectorn", &PythonExtendWin_wrapper::popvectorn, RETURN_REFERENCE)
  			.def("popintvectorn", &PythonExtendWin_wrapper::popintvectorn, RETURN_REFERENCE)
			.def("popboolean", &PythonExtendWin_wrapper::popboolean)
  			.def("checkmatrixn", &PythonExtendWin_wrapper::checkmatrixn, RETURN_REFERENCE)
  			.def("checkhypermatrixn", &PythonExtendWin_wrapper::checkhypermatrixn, RETURN_REFERENCE)
  			.def("checkvectorn", &PythonExtendWin_wrapper::checkvectorn, RETURN_REFERENCE)
  			.def("checkintvectorn", &PythonExtendWin_wrapper::checkintvectorn, RETURN_REFERENCE)
		    .def("popnumber", &PythonExtendWin_wrapper::popnumber)
		    .def("popint", &PythonExtendWin_wrapper::popint)
			.def("set", &PythonExtendWin_wrapper::set)
  			.def("isnil", &PythonExtendWin_wrapper::isnil)
			.def("gettop",&PythonExtendWin_wrapper::gettop)
			.def("pop",&PythonExtendWin_wrapper::pop)
		;
	}

}

