
#include "physicsLib.h"

#include "DynamicsSimulator_penaltyMethod.h"
#include "InertiaCalculator.h"

#include "../MainLib/WrapperLua/luna_mainlib.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "../MainLib/OgreFltk/VRMLloaderView.h"
#include "../BaseLib/motion/VRMLloader_internal.h"
#include "ScriptBaseWin.h"
void Register_physicsbind(lua_State*L);
ScriptBaseWin::ScriptBaseWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder)
	:ScriptWin(x,y,w,h,mp,renderer, defaultScript, _defaultScriptFolder),
	_autoLoaded(false)
{
}

void ScriptBaseWin::createMenu()
{
	setUniformGuidelines(10);
	for(int i=0; i<scripts.size(); i++)
	{
		TString fn=scripts[i];
		if(fn.right(4)==".lua") {
			fn=fn.right(-1*(fn.findCharRight('/')+1));
			create("Button", TString("load:", i), TString("", i)+TString(".  ")+fn, 0, 10);
			Fl_Button* b=button(0);
#ifndef NO_GUI
			b->shortcut('0'+i);
			b->tooltip("shotcut: number");
			b->align(FL_ALIGN_INSIDE | FL_ALIGN_LEFT);
#endif
		}
		else
		{
			fn=fn.left(fn.findCharRight('/'));
			fn=fn.left(fn.findCharRight('/'));
			fn=scripts[i].right(-fn.length()-1);
			create("Button", TString("chos:", i), TString("", i)+TString(".  ")+fn, 0, 10);
			Fl_Button* b=button(0);
#ifndef NO_GUI
			b->shortcut('0'+i);
			b->tooltip("shotcut: number");
			b->align(FL_ALIGN_INSIDE | FL_ALIGN_LEFT);
#endif

		}
	}
}
ScriptBaseWin::ScriptBaseWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, TStrings const& _scripts)
	:ScriptWin(x,y,w,h,mp,renderer, _scripts[0], "")
{
	// remove all widgets ( load, x buttons)
	removeWidgets(-2);
	create("Button", "X", "back", 0,10, 0);
#ifndef NO_GUI
	button(0)->shortcut(FL_BackSpace);
#endif
	
	scripts=_scripts;
	createMenu();
	updateLayout();
}

void ScriptBaseWin::loadDefaultScript(){
	int wi=widgetIndex("scriptfn"); // remote the "load" button.
	removeWidgets(wi+1);
	loadScript( _scriptFn);
	_autoLoaded=true;
}

ScriptBaseWin::~ScriptBaseWin(void)
{
	if(_autoLoaded)
		create("Button", "X", "X", 9); // so that ~ScriptWin does not fail.
}

void ScriptBaseWin::initLuaEnvironment()
{
	ScriptWin::initLuaEnvironment();
	Register_physicsbind(L);
}
	
void VRMLloader_checkMass(VRMLloader& l)
{
	OpenHRP::LinkInfo min;
	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			if (bone.mSegment->mass<min.mass)
				bone.mSegment->mass	=min.mass;

			if (bone.mSegment->momentsOfInertia._11<min.inertia._11)
				bone.mSegment->momentsOfInertia._11=min.inertia._11;
			if (bone.mSegment->momentsOfInertia._22<min.inertia._22)
				bone.mSegment->momentsOfInertia._22=min.inertia._22;
			if (bone.mSegment->momentsOfInertia._33<min.inertia._33)
				bone.mSegment->momentsOfInertia._33=min.inertia._33;
		}
	}
}
void VRMLloader_setTotalMass(VRMLloader & l, m_real totalMass)
{
	vectorn mass;
	std::vector<matrix3> inertia;

	mass.resize(l.numBone());
	inertia.resize(l.numBone());

#ifndef NO_BULLET
	InertiaCalculator ic;
#else
	InertiaCalculatorAnalytic ic;
#endif
	mass[0]=0.0;
	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mShape)
		{
			ic.calculateFromMesh(bone.mShape->mesh);
			mass[b]=ic.volume();
			inertia[b]=ic.inertia();
			bone.mSegment->centerOfMass=ic.centerOfMass();
		}
		else
		{			
			mass[b]=0;
			inertia[b].zero();
		}
	}

	m_real scale=totalMass/mass.sum();
	if (totalMass==0.0) scale=997.0; // water density 997kg/m^3

	VRMLloader_checkMass(l);

	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			bone.mSegment->mass=mass[b]*scale;
			bone.mSegment->momentsOfInertia.mult(inertia[b],scale);
		}
	}
}

// calculate all center of masses and inertia tensors based on geometry.
void VRMLloader_setTotalMass(VRMLloader & l, m_real totalMass);
void VRMLloader_checkMass(VRMLloader& l);


TString FlChooseFile(const char* message, const char* path, const char* Mask, bool bCreate);
void ScriptBaseWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId.length()>5 && w.mId.left(5)=="load:")
	{
		int i=atoi(w.mId.right(-5).ptr());
		releaseScript();
		loadScript(scripts[i]);
	}
	else if(w.mId.length()>5 && w.mId.left(5)=="chos:")
	{
		int i=atoi(w.mId.right(-5).ptr());
		TString fn=scripts[i];
		defaultScriptFolder=fn.left(fn.findCharRight('/')+1);
		TString new_script=FlChooseFile("choose a script", defaultScriptFolder.ptr(), "*.lua");
		if(new_script.length())
		{
			releaseScript();
			loadScript(new_script);
		}
	}
	else if(w.mId=="X")
	{
		releaseScript();
		createMenu();
		updateLayout();
	}
	else
		ScriptWin::onCallback(w, pWidget, userData);
}
