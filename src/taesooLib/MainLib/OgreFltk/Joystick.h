#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#pragma once

#if defined(_MSC_VER) && defined(INCLUDE_DIRECT_INPUT)
//#define STRICT
#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#endif

class Joystick
{
public:
#ifdef DIRECTINPUT_VERSION
	Joystick(HWND hDlg);
#else
	Joystick();
#endif
	virtual ~Joystick();

	void onActivate();
	void update();
	TString state();

#ifdef DIRECTINPUT_VERSION
	DIJOYSTATE2 m_js;           // DInput joystick state 

	BOOL enumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance);
	BOOL enumObjectsCallback( const DIDEVICEOBJECTINSTANCE* pdidoi);
#endif

	// -1 to 1
	float x();
	float y();
	float theta();
	float x_circle();
	float y_circle();
	float speed();
	float rotX();
	float rotZ();
	// 0 to 1
	float slider(int i=0);
	bool button(int i)	;
private:
#ifdef DIRECTINPUT_VERSION
	HRESULT InitDirectInput( HWND hDlg );
	LPDIRECTINPUT8       g_pDI;
	LPDIRECTINPUTDEVICE8 g_pJoystick;
	HRESULT UpdateInputState();
#endif
};

#include "FltkRenderer.h"

class FltkJoystickRenderer  : public FltkToolkitRenderer
{
public:
	FltkJoystickRenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer);
	virtual ~FltkJoystickRenderer();

	virtual void firstInit();
	virtual void loop(Fl_Window& mainWin);
	virtual int handle(int ev);
	Joystick* m_pJoystick;

};

#ifdef DIRECTINPUT_VERSION
void initJoystick(HWND hDlg);
#else
void initJoystick();
#endif
void deinitJoystick();
Joystick* joystick();
#endif
