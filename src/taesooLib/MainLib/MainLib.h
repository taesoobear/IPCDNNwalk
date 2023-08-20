#ifndef _MAINLIB_H_
#define _MAINLIB_H_

#ifndef TRACE
#include <assert.h>

#ifdef _DEBUG
#define ASSERT(x) assert(x)
#define VERIFY(x) assert(x)
#define TRACE	Msg::print
#else
#define ASSERT(x)
#define VERIFY(x)	(x)
#define TRACE	__noop
#endif
#endif

#ifndef NO_GUI

#if _MSC_VER >1000
#define FL_DLL   // uncomment if you linked against fltk dll.

#pragma once
// disable boost codepage warning.
#pragma warning( disable : 4819)

#endif // FL_DLL

#endif//ndef NO_GUI

#include <iostream>
//#include <tchar.h>

#ifdef NO_GUI
  
#include "console/dummies.h"
#else //NO_GUI
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Tabs.H>

// following two files include the notorious "windows.h".
// please avoid including these files in header files.
//#include <FL/x.H>
//#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Round_Button.H>
#include <FL/Fl_Value_Input.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Check_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Choice.H>

// following two files also bring horrible dependencies.
//#include <Fl/Fl_PNG_Image.h>
//#include <Fl/Fl_JPEG_Image.h>
#include <FL/Fl_Int_Input.H>

#endif //NO_GUI

#include "../BaseLib/utility/stdtemplate.h"
#include "../BaseLib/utility/configtable.h"
#include "../BaseLib/image/imageclass.h"
#include "../BaseLib/utility/TypeString.h" 
#include "../BaseLib/motion/Motion.h"
#include "../BaseLib/motion/MotionLoader.h"
/*
#include "Ogre.h"
//#include "OgreEventListeners.h"
#include "OgreStringConverter.h"
#include "OgreException.h"
#include "OgreFontManager.h"
*/

#ifndef NO_OGRE
#if OGRE_VERSION_MINOR >= 12 || OGRE_VERSION_MAJOR>=13
#include <Ogre.h>
#else
#include <OgrePrerequisites.h>
#endif
#include <OgreColourValue.h>
// following three files also bring horrible dependencies.
//#include <OgreSceneNode.h>
//#include <OgreSceneManager.h>
//#include <OgreEntity.h>
#endif

#include "OgreFltk/RE.h"
//#include "OgreFltk/FltkRenderer.h"
#include "OgreFltk/AlzzaPostureIP.h"
#include "OgreFltk/LineStrip.h"
#include "OgreFltk/LineSegment.h"
#include "OgreFltk/pldprimskin.h"
#include "OgreFltk/timesensor.h"
#endif
