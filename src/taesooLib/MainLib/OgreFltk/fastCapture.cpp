#include "stdafx.h"
#ifndef NO_GUI
#include "fastCapture.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/utility/QPerformanceTimer.h"




// Helper macro to return from function when error occurs:
#define ERROR_BREAK(x) throw (int)(x);

FastCapture::FastCapture(const char* filename)
{
	fn=filename;
	last_x=-1;
	last_y=-1;
}


FastCaptureDirect::FastCaptureDirect(const char* FileName)
:FastCapture(FileName)
{
	createDirectory(fn.left(-4));
	mWidth=0;
	mHeight=0;
	m_nCount=0;
	m_nCurrAccumulation=0;
	m_nTotalAccumulation=1;
	lpvBits=NULL;
}

FastCaptureDirect::~FastCaptureDirect()
{
	if (lpvBits) delete[] lpvBits;
	lpvBits=NULL;
}

#ifdef _MSC_VER
#include <fl/x.h>
// Helper function to retrieve current position of file pointer:
inline int GetFilePointer(HANDLE FileHandle)
{
	return SetFilePointer(FileHandle, 0, 0, FILE_CURRENT);
}
#endif
bool FastCaptureDirect::Screenshot(Fl_Window* win, int Width, int Height)
{
	win->make_current();
#ifdef _MSC_VER
	HDC SurfDC=fl_gc;
#endif
	if(mWidth==0)
	{
		mWidth=Width;
		mHeight=Height;
		image.Create(mWidth, mHeight);
		tempPixel.Init(&image);
	}
	else
		Msg::verify(mWidth==Width && mHeight==Height, "Error! size changed");

	if(last_x!=-1)
	{
		cursorPos.resize(m_nCount*2+2);
		cursorPos[m_nCount*2]=last_x;
		cursorPos[m_nCount*2+1]=last_y;
	}

#ifdef _MSC_VER
	HBITMAP OffscrBmp=NULL; // bitmap that is converted to a DIB
	HDC OffscrDC=NULL;      // offscreen DC that we can select OffscrBmp into
	LPBITMAPINFO lpbi=NULL; // bitmap format info; used by GetDIBits
#endif
	unsigned int bitmapSize;




	CImage temp;
	CImagePixel cursor;

	if(cursorPos.size())
	{
		temp.Load("../Resource/default/cursor2.bmp");
		cursor.Init(&temp);
	}

	try
	{
#ifdef _MSC_VER
		{
			BEGIN_TIMER(screenshot);

			// We need an HBITMAP to convert it to a DIB:
			if ((OffscrBmp = CreateCompatibleBitmap(SurfDC, Width, Height)) == NULL)
				ERROR_BREAK(2);


			// The bitmap is empty, so let's copy the contents of the surface to it.
			// For that we need to select it into a device context. We create one.
			if ((OffscrDC = CreateCompatibleDC(SurfDC)) == NULL) ERROR_BREAK(3);


			//END_TIMER(create);

			// bitblt와 fwrite가 느림.
			//BEGIN_TIMER(bitblt);

			// Select OffscrBmp into OffscrDC:
			HBITMAP OldBmp = (HBITMAP)SelectObject(OffscrDC, OffscrBmp);
			// Now we can copy the contents of the surface to the offscreen bitmap:
			BitBlt(OffscrDC, 0, 0, Width, Height, SurfDC, 0, 0, SRCCOPY);

			//END_TIMER(bitblt);

			// We don't need SurfDC anymore. Free it:
			SurfDC = NULL;

			//BEGIN_TIMER(getdib);

			// GetDIBits requires format info about the bitmap. We can have GetDIBits
			// fill a structure with that info if we pass a NULL pointer for lpvBits:
			// Reserve memory for bitmap info (BITMAPINFOHEADER + largest possible
			// palette):
			if ((lpbi = (LPBITMAPINFO)(new char[sizeof(BITMAPINFOHEADER) +
							256 * sizeof(RGBQUAD)])) == NULL) ERROR_BREAK(4);
			ZeroMemory(&lpbi->bmiHeader, sizeof(BITMAPINFOHEADER));
			lpbi->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
			// Get info but first de-select OffscrBmp because GetDIBits requires it:
			SelectObject(OffscrDC, OldBmp);
			if (!GetDIBits(OffscrDC, OffscrBmp, 0, Height, NULL, lpbi, DIB_RGB_COLORS))
				ERROR_BREAK(5);

			//END_TIMER(getdib);
			// Reserve memory for bitmap bits:

			if(!lpvBits)
			{
				if ((lpvBits = new unsigned char[lpbi->bmiHeader.biSizeImage]) == NULL)
					ERROR_BREAK(6);
			}

			// Have GetDIBits convert OffscrBmp to a DIB (device-independent bitmap):
			if (!GetDIBits(OffscrDC, OffscrBmp, 0, Height, lpvBits, lpbi,
						DIB_RGB_COLORS)) ERROR_BREAK(7);

			bitmapSize=lpbi->bmiHeader.biSizeImage;
			// Write bitmap bits to the file:

			//BEGIN_TIMER(screenshot);
		}
#else


		if(!lpvBits)
			lpvBits=new unsigned char[mWidth*mHeight*3];
		fl_read_image(lpvBits, 0,0,mWidth, mHeight);

		if (m_nTotalAccumulation==1)
		{
			for(int i=0; i<mHeight; i++)
			{
				CPixelRGB8* yy=tempPixel[mHeight-1-i];
				uchar* pp=lpvBits+i*mWidth*3;
				for(int j=0; j<mWidth; j++)
				{
					yy[j].R=*(pp++);
					yy[j].G=*(pp++);
					yy[j].B=*(pp++);
				}
			}
		}
		else
		{
			if (m_nCurrAccumulation==0)
			{
				for(int i=0; i<mHeight; i++)
				{
					CPixelRGB8* yy=tempPixel[i];
					uchar* pp=lpvBits+i*mWidth*3;
					for(int j=0; j<mWidth; j++)
					{
						yy[j].R=*(pp++)/m_nTotalAccumulation;
						yy[j].G=*(pp++)/m_nTotalAccumulation;
						yy[j].B=*(pp++)/m_nTotalAccumulation;
					}
				}
			}
			else
			{
				for(int i=0; i<mHeight; i++)
				{
					CPixelRGB8* yy=tempPixel[i];
					uchar* pp=lpvBits+i*mWidth*3;
					for(int j=0; j<mWidth; j++)
					{
						yy[j].R+=*(pp++)/m_nTotalAccumulation;
						yy[j].G+=*(pp++)/m_nTotalAccumulation;
						yy[j].B+=*(pp++)/m_nTotalAccumulation;
					}
				}
			}
		}


#endif

		int ii=	m_nCount;
	

		if(ii%100==0)
			printf("Dump %d/%d saved\n", ii,m_nCount);

		TString filename(sz0::format("%s/%05d.jpg",fn.left(-4).ptr(), ii));
		//TString filename(sz0::format("%s/%05d.tga",fn.left(-4).ptr(), ii));

#ifdef _MSC_VER
		// 만약 아래부분에서 죽으면 화면 모드가 16bit에 되어있는것임.
		// Make sure that the screen mode is in 32bit.
		{
			if(bitmapSize==mWidth*mHeight*4)
			{
				// 32bit mode
				for(int i=0; i<mHeight; i++)
				{
					CPixelRGB8* yy=tempPixel[mHeight-i-1];
					XRGB* pp=((XRGB*)lpvBits)+i*mWidth;
					for(int j=0; j<mWidth; j++)
					{
						yy[j].R=pp[j].R;
						yy[j].G=pp[j].G;
						yy[j].B=pp[j].B;
					}
				}
			}
			else
			{
				// 32bit mode
				for(int i=0; i<mHeight; i++)
				{
					CPixelRGB8* yy=tempPixel[mHeight-i-1];
					RGB565* pp=((RGB565*)lpvBits)+i*mWidth;
					for(int j=0; j<mWidth; j++)
					{
						yy[j].R=pp[j].R();
						yy[j].G=pp[j].G();
						yy[j].B=pp[j].B();
					}
				}
			}
		}
#endif

		m_nCurrAccumulation++;
		if (m_nCurrAccumulation==m_nTotalAccumulation)
		{
			tempPixel.DrawPattern(cursorPos[ii*2], mHeight-1-cursorPos[ii*2+1], cursor, true, CPixelRGB8 (255,0,255));
			image.Save(filename);
			m_nCurrAccumulation=0;
			m_nCount++;
		}
	}
	catch (int &errorcode)
	{
		Msg::print("Screenshot error #%i", errorcode);
	}
	catch (...)
	{
		Msg::print("Screenshot error");
	}

#ifdef _MSC_VER
	if (OffscrDC) DeleteDC(OffscrDC);
	if (OffscrBmp) DeleteObject(OffscrBmp);
	if (lpbi) delete[] lpbi;
#endif

	return true;
}

/////////////////////////////////////
///////////////////////////////

#ifdef _MSC_VER
// Screenshot
//    -> FileName: Name of file to save screenshot to
//    -> lpDDS: DirectDraw surface to capture
//    <- Result: Success
//
bool ScreenshotNew(LPCTSTR FileName, HDC SurfDC, int Width, int Height)
{

	if (!FileName ) return false;

	bool Success=false;
	HBITMAP OffscrBmp=NULL; // bitmap that is converted to a DIB
	HDC OffscrDC=NULL;      // offscreen DC that we can select OffscrBmp into
	LPBITMAPINFO lpbi=NULL; // bitmap format info; used by GetDIBits
	LPVOID lpvBits=NULL;    // pointer to bitmap bits array
	HANDLE BmpFile=INVALID_HANDLE_VALUE;    // destination .bmp file
	BITMAPFILEHEADER bmfh;  // .bmp file header

	try
	{
		// We need an HBITMAP to convert it to a DIB:
		if ((OffscrBmp = CreateCompatibleBitmap(SurfDC, Width, Height)) == NULL)
			ERROR_BREAK(2);

		// The bitmap is empty, so let's copy the contents of the surface to it.
		// For that we need to select it into a device context. We create one.
		if ((OffscrDC = CreateCompatibleDC(SurfDC)) == NULL) ERROR_BREAK(3);
		// Select OffscrBmp into OffscrDC:
		HBITMAP OldBmp = (HBITMAP)SelectObject(OffscrDC, OffscrBmp);
		// Now we can copy the contents of the surface to the offscreen bitmap:
		BitBlt(OffscrDC, 0, 0, Width, Height, SurfDC, 0, 0, SRCCOPY);

		// We don't need SurfDC anymore. Free it:
		SurfDC = NULL;

		// GetDIBits requires format info about the bitmap. We can have GetDIBits
		// fill a structure with that info if we pass a NULL pointer for lpvBits:
		// Reserve memory for bitmap info (BITMAPINFOHEADER + largest possible
		// palette):
		if ((lpbi = (LPBITMAPINFO)(new char[sizeof(BITMAPINFOHEADER) +
			256 * sizeof(RGBQUAD)])) == NULL) ERROR_BREAK(4);
		ZeroMemory(&lpbi->bmiHeader, sizeof(BITMAPINFOHEADER));
		lpbi->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		// Get info but first de-select OffscrBmp because GetDIBits requires it:
		SelectObject(OffscrDC, OldBmp);
		if (!GetDIBits(OffscrDC, OffscrBmp, 0, Height, NULL, lpbi, DIB_RGB_COLORS))
			ERROR_BREAK(5);

		// Reserve memory for bitmap bits:
		if ((lpvBits = new char[lpbi->bmiHeader.biSizeImage]) == NULL)
			ERROR_BREAK(6);

		// Have GetDIBits convert OffscrBmp to a DIB (device-independent bitmap):
		if (!GetDIBits(OffscrDC, OffscrBmp, 0, Height, lpvBits, lpbi,
			DIB_RGB_COLORS)) ERROR_BREAK(7);

		// Create a file to save the DIB to:
		if ((BmpFile = CreateFile(FileName,
			GENERIC_WRITE,
			0, NULL,
			CREATE_ALWAYS,
			FILE_ATTRIBUTE_NORMAL,
			NULL)) == INVALID_HANDLE_VALUE) ERROR_BREAK(8);

		DWORD Written;    // number of bytes written by WriteFile

		// Write a file header to the file:
		bmfh.bfType = 19778;        // 'BM'
		// bmfh.bfSize = ???        // we'll write that later
		bmfh.bfReserved1 = bmfh.bfReserved2 = 0;
		// bmfh.bfOffBits = ???     // we'll write that later
		if (!WriteFile(BmpFile, &bmfh, sizeof(bmfh), &Written, NULL))
			ERROR_BREAK(9);
		if (Written < sizeof(bmfh)) ERROR_BREAK(9);

		// Write BITMAPINFOHEADER to the file:
		if (!WriteFile(BmpFile, &lpbi->bmiHeader, sizeof(BITMAPINFOHEADER),
			&Written, NULL)) ERROR_BREAK(10);
		if (Written < sizeof(BITMAPINFOHEADER)) ERROR_BREAK(10);

		// Calculate size of palette:
		int PalEntries;
		// 16-bit or 32-bit bitmaps require bit masks:
		if (lpbi->bmiHeader.biCompression == BI_BITFIELDS) PalEntries = 3;
		else
			// bitmap is palettized?
			PalEntries = (lpbi->bmiHeader.biBitCount <= 8) ?
			// 2^biBitCount palette entries max.:
			(int)(1 << lpbi->bmiHeader.biBitCount)
			// bitmap is TrueColor -> no palette:
			: 0;
		// If biClrUsed use only biClrUsed palette entries:
		if (lpbi->bmiHeader.biClrUsed) PalEntries = lpbi->bmiHeader.biClrUsed;

		// Write palette to the file:
		if (PalEntries)
		{
			if (!WriteFile(BmpFile, &lpbi->bmiColors, PalEntries * sizeof(RGBQUAD),
				&Written, NULL)) ERROR_BREAK(11);
			if (Written < PalEntries * sizeof(RGBQUAD)) ERROR_BREAK(11);
		}

		// The current position in the file (at the beginning of the bitmap bits)
		// will be saved to the BITMAPFILEHEADER:
		bmfh.bfOffBits = GetFilePointer(BmpFile);

		// Write bitmap bits to the file:
		if (!WriteFile(BmpFile, lpvBits, lpbi->bmiHeader.biSizeImage,
			&Written, NULL)) ERROR_BREAK(12);
		if (Written < lpbi->bmiHeader.biSizeImage) ERROR_BREAK(12);

		// The current pos. in the file is the final file size and will be saved:
		bmfh.bfSize = GetFilePointer(BmpFile);

		// We have all the info for the file header. Save the updated version:
		SetFilePointer(BmpFile, 0, 0, FILE_BEGIN);
		if (!WriteFile(BmpFile, &bmfh, sizeof(bmfh), &Written, NULL))
			ERROR_BREAK(13);
		if (Written < sizeof(bmfh)) ERROR_BREAK(13);

		Success = true;
	}
	catch (int &errorcode)
	{
		char Buf[100];
		wsprintf(Buf, "Screenshot error #%i", errorcode);
		OutputDebugString(Buf);
	}
	catch (...)
	{
		OutputDebugString("Screenshot error");
	}

	if (OffscrDC) DeleteDC(OffscrDC);
	if (OffscrBmp) DeleteObject(OffscrBmp);
	if (lpbi) delete[] lpbi;
	if (lpvBits) delete[] lpvBits;
	if (BmpFile != INVALID_HANDLE_VALUE) CloseHandle(BmpFile);

	return Success;
}

#endif
#endif
