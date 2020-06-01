
#pragma once

#define NOMINMAX

#include "../BaseLib/baselib.h"

#include "MainLib.h"

/*
#include <iostream>
#include <tchar.h>

// TODO: 프로그램에 필요한 추가 헤더는 여기에서 참조합니다.
#define FL_DLL // 다시 DLL로 갈아끼움. (BaseLib의 이미지 클래스들이 얘네 이미지랑 쫑나서리..)
// 다시 static으로갈아끼움. (DLL에 정의안된 함수가 너무 많아서.) 그대신 Fl_Jpeg_Image클래스 사용 불가.
// Jpeg로딩시 CImage 이미지 클래스 사용할것. 그리기 위해서는 fl_draw_CImage함수를 새로 정의하였음.
// 다시 dll로 갈아끼움. (2007. 11. 1) -> mainlibpython이랑 쫑나서 어쩔수 없었음.

#include <FL/FL.h>
#include <FL/FL_window.h>
#include <FL/Fl_Tabs.h>
#include <FL/x.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/Fl_value_slider.H>
#include <FL/fl_draw.H>
#include <Fl/FL_Round_Button.H>
#include <Fl/Fl_Value_Input.h>
#include <Fl/Fl_Output.h>
#include <Fl/Fl_PNG_Image.h>
#include <Fl/Fl_JPEG_Image.h>
#include <FL/Fl_Int_Input.h>

#include "../BaseLib/utility/stdtemplate.h"
#include "../BaseLib/utility/configtable.h"
#include "../BaseLib/image/imageclass.h"
#include "../BaseLib/utility/TypeString.h"
#include "../BaseLib/utility/GArray.h"
#include "../BaseLib/motion/Motion.h"
#include "../BaseLib/motion/MotionLoader.h"
#include "../BaseLib/utility/TWord.h"

#include "Ogre.h"
//#include "OgreEventListeners.h"
#include "OgreStringConverter.h"
#include "OgreException.h"
#include "OgreFontManager.h"


#include "OgreFltk/RE.h"
#include "OgreFltk/AlzzaPostureIP.h"
#include "OgreFltk/LineSegment.h"
#include "OgreFltk/FltkRenderer.h"
#include "OgreFltk/pldprimskin.h"
#include "OgreFltk/timesensor.h"
#include "OgreFltk/MovableText.h"
*/


void dprintf(bool bExpression,const char* pszFormat, ...);