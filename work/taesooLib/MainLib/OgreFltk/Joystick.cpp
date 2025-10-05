#include "stdafx.h"
#include "Joystick.h"
#ifdef DIRECTINPUT_VERSION

//-----------------------------------------------------------------------------
// File: Joystick.cpp
//
// Desc: Demonstrates an application which receives immediate 
//       joystick data in exclusive mode via a dialog timer.
//
// Copyright (c) 1998-2001 Microsoft Corporation. All rights reserved.
//-----------------------------------------------------------------------------

#include <windows.h>
#include <commctrl.h>
#include <basetsd.h>
//#include "Resource.h"



	void Joystick::onActivate()	{ if( g_pJoystick ) g_pJoystick->Acquire();}
	void Joystick::update()		{ UpdateInputState();}
	float Joystick::x()				{ return ((float)(m_js.lX-32768))/32768.f;}
	float Joystick::y()				{ return ((float)(m_js.lY-32768))/32768.f;}
	float Joystick::speed()			{ return sqrt(SQR(x())+SQR(y()));}
	float Joystick::rotX()			{ return ((float)(m_js.lRx-32768))/32768.f;}
	float Joystick::rotZ()			{ return ((float)(m_js.lRz-32768))/32768.f;}
	float Joystick::slider(int i)	{ return ((float)m_js.rglSlider[i])/65535.f;}
	bool Joystick::button(int i)		{ return m_js.rgbButtons[i] & 0x80;}

//-----------------------------------------------------------------------------
// Function-prototypes
//-----------------------------------------------------------------------------
BOOL CALLBACK    EnumObjectsCallback( const DIDEVICEOBJECTINSTANCE* pdidoi, VOID* pContext );
BOOL CALLBACK    EnumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance, VOID* pContext );

//-----------------------------------------------------------------------------
// Defines, constants, and global variables
//-----------------------------------------------------------------------------
#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }

Joystick::Joystick(HWND hDlg)
{
	g_pDI              = NULL;         
	g_pJoystick        = NULL;     
	InitDirectInput( hDlg );
}

Joystick::~Joystick()
{
    // Unacquire the device one last time just in case 
    // the app tried to exit while the device is still acquired.
    if( g_pJoystick ) 
        g_pJoystick->Unacquire();
    
    // Release any DirectInput objects.
    SAFE_RELEASE( g_pJoystick );
    SAFE_RELEASE( g_pDI );
}


//-----------------------------------------------------------------------------
// Name: InitDirectInput()
// Desc: Initialize the DirectInput variables.
//-----------------------------------------------------------------------------
HRESULT Joystick::InitDirectInput( HWND hDlg )
{
    HRESULT hr;

    // Register with the DirectInput subsystem and get a pointer
    // to a IDirectInput interface we can use.
    // Create a DInput object
    if( FAILED( hr = DirectInput8Create( GetModuleHandle(NULL), DIRECTINPUT_VERSION, 
                                         IID_IDirectInput8, (VOID**)&g_pDI, NULL ) ) )
        return hr;

    // Look for a simple joystick we can use for this sample program.
    if( FAILED( hr = g_pDI->EnumDevices( DI8DEVCLASS_GAMECTRL, 
                                         EnumJoysticksCallback,
                                         (VOID*) this, DIEDFL_ATTACHEDONLY ) ) )
        return hr;

    // Make sure we got a joystick
    if( NULL == g_pJoystick )
    {
		printf("Joystick not found. The sample will now exit.\n");
		return S_OK;
    }

    // Set the data format to "simple joystick" - a predefined data format 
    //
    // A data format specifies which controls on a device we are interested in,
    // and how they should be reported. This tells DInput that we will be
    // passing a DIJOYSTATE2 structure to IDirectInputDevice::GetDeviceState().
    if( FAILED( hr = g_pJoystick->SetDataFormat( &c_dfDIJoystick2 ) ) )
        return hr;

    // Set the cooperative level to let DInput know how this device should
    // interact with the system and with other DInput applications.
    if( FAILED( hr = g_pJoystick->SetCooperativeLevel( hDlg, DISCL_EXCLUSIVE | 
                                                             DISCL_FOREGROUND ) ) )
        return hr;

    // Enumerate the joystick objects. The callback function enabled user
    // interface elements for objects that are found, and sets the min/max
    // values property for discovered axes.
    if( FAILED( hr = g_pJoystick->EnumObjects( EnumObjectsCallback, 
                                                (VOID*)this, DIDFT_ALL ) ) )
        return hr;

    return S_OK;
}




//-----------------------------------------------------------------------------
// Name: EnumJoysticksCallback()
// Desc: Called once for each enumerated joystick. If we find one, create a
//       device interface on it so we can play with it.
//-----------------------------------------------------------------------------
BOOL CALLBACK EnumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance,
                                     VOID* pContext )
{
	return ((Joystick*)pContext)->enumJoysticksCallback(pdidInstance);
}

BOOL Joystick::enumJoysticksCallback( const DIDEVICEINSTANCE* pdidInstance)
{
    HRESULT hr;

    // Obtain an interface to the enumerated joystick.
    hr = g_pDI->CreateDevice( pdidInstance->guidInstance, &g_pJoystick, NULL );

    // If it failed, then we can't use this joystick. (Maybe the user unplugged
    // it while we were in the middle of enumerating it.)
    if( FAILED(hr) ) 
        return DIENUM_CONTINUE;

    // Stop enumeration. Note: we're just taking the first joystick we get. You
    // could store all the enumerated joysticks and let the user pick.
    return DIENUM_STOP;
}




//-----------------------------------------------------------------------------
// Name: EnumObjectsCallback()
// Desc: Callback function for enumerating objects (axes, buttons, POVs) on a 
//       joystick. This function enables user interface elements for objects
//       that are found to exist, and scales axes min/max values.
//-----------------------------------------------------------------------------
BOOL CALLBACK EnumObjectsCallback( const DIDEVICEOBJECTINSTANCE* pdidoi,
                                   VOID* pContext )
{
	return ((Joystick*)pContext)->enumObjectsCallback(pdidoi);
}

BOOL Joystick::enumObjectsCallback( const DIDEVICEOBJECTINSTANCE* pdidoi)
{
    static int nSliderCount = 0;  // Number of returned slider controls
    static int nPOVCount = 0;     // Number of returned POV controls

    // For axes that are returned, set the DIPROP_RANGE property for the
    // enumerated axis in order to scale min/max values.
    if( pdidoi->dwType & DIDFT_AXIS )
    {
        DIPROPRANGE diprg; 
        diprg.diph.dwSize       = sizeof(DIPROPRANGE); 
        diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER); 
        diprg.diph.dwHow        = DIPH_BYID; 
        diprg.diph.dwObj        = pdidoi->dwType; // Specify the enumerated axis
        diprg.lMin              = -1000; 
        diprg.lMax              = +1000; 
    
        // Set the range for the axis
        if( FAILED( g_pJoystick->SetProperty( DIPROP_RANGE, &diprg.diph ) ) ) 
            return DIENUM_STOP;
         
    }

    // Set the UI to reflect what objects the joystick supports
    if (pdidoi->guidType == GUID_XAxis)
    {
		printf("X_AXIS enabled\n");
    }
    if (pdidoi->guidType == GUID_YAxis)
    {
		printf("Y_AXIS enabled\n");
    }
    if (pdidoi->guidType == GUID_ZAxis)
    {
		printf("Z_AXIS enabled\n");
    }
    if (pdidoi->guidType == GUID_RxAxis)
    {
		printf("X_ROT enabled\n");
    }
    if (pdidoi->guidType == GUID_RyAxis)
    {
		printf("Y_ROT enabled\n");
    }
    if (pdidoi->guidType == GUID_RzAxis)
    {
		printf("Z_ROT enabled\n");
    }
    if (pdidoi->guidType == GUID_Slider)
    {
		printf("Slider%d enabled\n", nSliderCount++ );
    }
    if (pdidoi->guidType == GUID_POV)
    {
        printf("Pov%d enabled\n", nPOVCount++ );
    }

    return DIENUM_CONTINUE;
}


//-----------------------------------------------------------------------------
// Name: UpdateInputState()
// Desc: Get the input device's state and display it.
//-----------------------------------------------------------------------------
HRESULT Joystick::UpdateInputState() 
{
    HRESULT     hr;


    if( NULL == g_pJoystick ) 
        return S_OK;

    // Poll the device to read the current state
    hr = g_pJoystick->Poll(); 
    if( FAILED(hr) )  
    {
        // DInput is telling us that the input stream has been
        // interrupted. We aren't tracking any state between polls, so
        // we don't have any special reset that needs to be done. We
        // just re-acquire and try again.
        hr = g_pJoystick->Acquire();
        while( hr == DIERR_INPUTLOST ) 
            hr = g_pJoystick->Acquire();

        // hr may be DIERR_OTHERAPPHASPRIO or other errors.  This
        // may occur when the app is minimized or in the process of 
        // switching, so just try again later 
        return S_OK; 
    }

    // Get the input's device state
	
	DIJOYSTATE2 js;
    if( FAILED( hr = g_pJoystick->GetDeviceState( sizeof(DIJOYSTATE2), &js) ) )
        return hr; // The device should have been acquired during the Poll()

	// thread safe
    m_js =js;
    return S_OK;
}

TString Joystick::state()
{
	TString out;
	// Display joystick state to dialog

    // Axes
	out.format("axis [%f %f %ld]  axisRot [%ld %ld %ld]",x(), y(), m_js.lZ, m_js.lRx, m_js.lRy, m_js.lRz);

    // Slider controls
	out.add("slider %f %f ", slider(0), slider(1));

	// Points of view
    // m_js.rgdwPOV[0][1][2][3]

    // Fill up text with which buttons are pressed
    for( int i = 0; i < 128; i++ )
    	if ( m_js.rgbButtons[i] & 0x80 ) 
			out.add("bt %ld ", i);

	return out;
}

float Joystick::theta()
{
	vector3 angle;
	angle.x=x();
	angle.y=y();
	return vector3(1,0,0).angle2ds(angle);
}

float Joystick::x_circle()
{	
	vector3 pos;
	pos.x=x();
	pos.y=y();
	pos.z=0;

	float cosTheta=pos.cosTheta(vector3(1,0,0));
    return x()*ABS(cosTheta);
}

float Joystick::y_circle()
{
	vector3 pos;
	pos.x=x();
	pos.y=y();
	pos.z=0;

	float sinTheta=pos.sinTheta(vector3(1,0,0));
    return y()*ABS(sinTheta);
}


Joystick* g_pJoystick=NULL;

void initJoystick(HWND hDlg)
{
	g_pJoystick=new Joystick(hDlg);
}

void deinitJoystick()
{
	delete g_pJoystick;
	g_pJoystick=NULL;
}

Joystick* joystick()
{
	if(g_pJoystick)	return g_pJoystick;
	return NULL;
}
#else

Joystick::Joystick()
{
}

Joystick::~Joystick()
{
}


TString Joystick::state()
{
	TString out;

	return out;
}

float Joystick::theta()
{
	vector3 angle;
	angle.x=x();
	angle.y=y();
	return vector3(1,0,0).angle2ds(angle);
}

float Joystick::x_circle()
{	
	vector3 pos;
	pos.x=x();
	pos.y=y();
	pos.z=0;

	float cosTheta=pos.cosTheta(vector3(1,0,0));
    return x()*ABS(cosTheta);
}

float Joystick::y_circle()
{
	vector3 pos;
	pos.x=x();
	pos.y=y();
	pos.z=0;

	float sinTheta=pos.sinTheta(vector3(1,0,0));
    return y()*ABS(sinTheta);
}


	void Joystick::onActivate()	{ }
	void Joystick::update()		{ }
	float Joystick::x()				{ return 0;}
	float Joystick::y()				{ return 0;}
	float Joystick::speed()			{ return 0;}
	float Joystick::rotX()			{ return 0;}
	float Joystick::rotZ()			{ return 0;}
	float Joystick::slider(int i)	{ return 0;}
	bool Joystick::button(int i)		{ return false;}
Joystick* g_pJoystick=NULL;

void initJoystick()
{
	g_pJoystick=new Joystick();
}

void deinitJoystick()
{
	delete g_pJoystick;
	g_pJoystick=NULL;
}

Joystick* joystick()
{
	if(g_pJoystick)	return g_pJoystick;
	return NULL;
}
#endif
