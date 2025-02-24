#ifndef _STDAFX_HHH_
#define _STDAFX_HHH_

#pragma once

#include <BaseLib/baselib.h>
#include "MLExport.h"
#include <iostream>

#ifndef NO_GUI
#ifdef _MSC_VER
#define FL_DLL // use DLL
#endif

#include <Fl/Fl.H>
#include <Fl/Fl_Window.H>
#include <Fl/Fl_Tile.H>
#include <Fl/Fl_Tabs.H>
#include <Fl/x.H>
#include <Fl/Fl_File_Chooser.H>
#include <Fl/Fl_Value_Slider.H>
#include <Fl/Fl_Round_Button.H>
#include <Fl/Fl_Value_Input.H>
#include <Fl/Fl_Output.H>
#include <Fl/Fl_PNG_Image.H>
#include <Fl/Fl_JPEG_Image.H>
#include <FL/Fl_Int_Input.H>
#endif

#include <BaseLib/utility/stdtemplate.h>
#include <BaseLib/utility/configtable.h>
#include <BaseLib/image/imageclass.h>
#include <BaseLib/utility/TypeString.h>
#include <BaseLib/utility/GArray.h>
#include <BaseLib/motion/Motion.h>
#include <BaseLib/motion/MotionLoader.h>

#ifndef NO_OGRE
#ifdef None
#undef None
#endif
#include "Ogre.h"
#include "OgreStringConverter.h"
#include "OgreException.h"
#else
#include "MainLib/MainLib.h"
#endif


#include <MainLib/OgreFltk/RE.h>
#include <MainLib/OgreFltk/AlzzaPostureIP.h>
#include <MainLib/OgreFltk/LineSegment.h>
#include <MainLib/OgreFltk/FltkRenderer.h>
#include <MainLib/OgreFltk/FltkAddon.h>
#include <MainLib/OgreFltk/FlLayout.h>
#include <MainLib/OgreFltk/pldprimskin.h>
#include <MainLib/OgreFltk/timesensor.h>
#include <MainLib/OgreFltk/MovableText.h>

#undef Success
#endif
