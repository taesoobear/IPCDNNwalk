
#ifdef _MSC_VER
#include "stdafx.h"
#ifdef _MSC_VER

#pragma comment(lib,"wsock32")
#pragma comment(lib,"comctl32")

#ifndef VC_EXTRALEAN		
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

class FastCaptureWin32
{
protected:
//#define USE_STDLIB
#ifdef USE_STDLIB
	FILE* BmpFile;
#else
	HANDLE BmpFile;
#endif
	int m_nCount;
	int mWidth, mHeight;
	DWORD bitmapSize;
	LPVOID lpvBits;    // pointer to bitmap bits array

	intvectorn cursorPos;

	bool mbAskBeforeConverting;
public:

	// opens a dump file
	FastCaptureWin32(const char* FileName, bool bAskBeforeConverting=true);

	// add a screen shot to the dump file
	virtual bool Screenshot(Fl_Window* win, int Width, int Height);
	virtual bool _Screenshot(HDC SurfDC, int Width, int Height);

	// closes the dump file after converting it to jpeg sequence
	virtual ~FastCaptureWin32();
};

// Helper function to retrieve current position of file pointer:
inline int GetFilePointer(HANDLE FileHandle)
{
	return SetFilePointer(FileHandle, 0, 0, FILE_CURRENT);
}

// imagepixel에 있다. 나중에 그걸 쓰도록 고칠것. - static으로 고치면 됨.
/*
void ToTagColor(COLORREF color, tagCOLOR& tagCOLOR)
{
	tagCOLOR.R=GetRValue(color);
	tagCOLOR.G=GetGValue(color);
	tagCOLOR.B=GetBValue(color);
}*/

template <class T>	// image[y][x].R image[y][x].G image[y][x].B 만 지원하면 됨.
void drawPattern(T const& image, int imageWidth, int imageHeight, int x, int y, const CImagePixel& patternPixel, bool bUseColorKey, CPixelRGB8 sColorkey)
{
	int patternWidth=patternPixel.Width();
	int patternHeight=patternPixel.Height();

	int imagex, imagey;

	if(bUseColorKey)
	{
		for(int j=0; j<patternHeight; j++)
			for(int i=0; i<patternWidth; i++)
			{
				imagex=x+i; imagey=y+j;
				if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
				{
					if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(CPixelRGB8))!=0)
					{
						image[imagey][imagex].R=patternPixel.Pixel(i,j).R;
						image[imagey][imagex].G=patternPixel.Pixel(i,j).G;
						image[imagey][imagex].B=patternPixel.Pixel(i,j).B;
					}
				}
			}
	}
	else
	{
		for(int j=0; j<patternHeight; j++)
		{
			for(int i=0; i<patternWidth; i++)
			{
				image[y+j][x+i].R=patternPixel.Pixel(i,j).R;
				image[y+j][x+i].G=patternPixel.Pixel(i,j).G;
				image[y+j][x+i].B=patternPixel.Pixel(i,j).B;
			}
		}
	}
}





void printUCHAR(uchar c)
{
	BitArray  tt;
	tt.m_Bits=c;
	for(int i=0; i<8; i++)
	{
		if(tt[7-i]) printf("1") ;
		else printf("0");
	}
	printf("\n");
}
void testRGB565()
{
RGB565 test;
test.high=0x0f;
test.low=0xff;
printUCHAR(test.high);
printUCHAR(test.low);

printUCHAR(test.R());
printUCHAR(test.G());
printUCHAR(test.B());

}

struct XRGBimage
{
	XRGB* mPtr;
	int mWidth, mHeight;
	XRGBimage(XRGB* ptr, int width, int height):mPtr(ptr),mWidth(width), mHeight(height){}
	XRGB* operator[](int i) const { return mPtr+(mHeight-1-i)*mWidth;}
};

FastCapture::FastCapture(const char* FileName, bool bAskBeforeConverting)
{
	mbAskBeforeConverting=bAskBeforeConverting;
	last_x=-1;
	last_y=-1;
	fn=FileName;
	lpvBits=NULL;
	m_nCount=0;
	mWidth=0;
	mHeight=0;
#ifdef USE_STDLIB
// Create a file to save the DIB to:
	if ((BmpFile = fopen(FileName, "wb") )== NULL)
		ERROR_BREAK(8);
#else
	// Create a file to save the DIB to:
	if ((BmpFile = CreateFile(FileName,
		GENERIC_WRITE,
		0, NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		NULL)) == INVALID_HANDLE_VALUE) ERROR_BREAK(8);

#endif
}
#include "../BaseLib/utility/tfile.h"
void FastCapture_convert(const char* filename)
{

	TString fn=filename;

	intvectorn cursorPos;
	int m_nCount;
	LPVOID lpvBits;    // pointer to bitmap bits array

	DWORD bitmapSize;
	int mWidth;
	int mHeight;
	BinaryFile file(false, fn+".info");
	file.unpack(cursorPos);
	file.unpackInt(m_nCount);
	file.unpackInt(mWidth);
	file.unpackInt(mHeight);
	bitmapSize=file.unpackInt();
	file.close();

	if ((lpvBits = new char[bitmapSize]) == NULL)
	{
		ERROR_BREAK(6);
	}

#ifdef USE_STDLIB
	FILE* BmpFile;
#else
	HANDLE BmpFile;
#endif


#ifdef USE_STDLIB
	if((BmpFile= fopen(filename, "rb"))==NULL)
		Msg::error("????");
#else
	if((BmpFile= ::CreateFile(filename, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL))==INVALID_HANDLE_VALUE)
		Msg::error("????");
#endif

	Ogre::Image i;
	CImage image;
	CImagePixel tempPixel;

	if(!USE_OGRE_IMAGE)
	{
		image.Create(mWidth, mHeight);
		tempPixel.Init(&image);
	}

	CImage temp;
	CImagePixel cursor;

	if(cursorPos.size())
	{
		temp.Load("../resource/default/cursor2.bmp");
		cursor.Init(&temp);
	}

	createDirectory(fn.left(-4));
	for(int ii=0; ii<m_nCount; ii++)
	{
		BEGIN_TIMER(convert);

		DWORD read;

#ifdef USE_STDLIB
		fread(lpvBits, bitmapSize,1, BmpFile);
		//if(	//!=bitmapSize) ERROR_BREAK(12);
#else
		if (!ReadFile(BmpFile, lpvBits, bitmapSize,
			&read, NULL)) ERROR_BREAK(12);
		if (read< bitmapSize) ERROR_BREAK(12);
#endif


		if(ii%100==0)
			printf("Dump %d/%d saved\n", ii,m_nCount);

		TString filename(sz0::format("%s/%05d.jpg",fn.left(-4).ptr(), ii));
		//TString filename(sz0::format("%s/%05d.tga",fn.left(-4).ptr(), ii));

		// 만약 아래부분에서 죽으면 화면 모드가 16bit에 되어있는것임.
		// Make sure that the screen mode is in 32bit.

		if(USE_OGRE_IMAGE)
		{
			if(bitmapSize==mWidth*mHeight*4)
			{
				if(cursorPos.size())
				drawPattern( XRGBimage((XRGB*)lpvBits, mWidth, mHeight), mWidth, mHeight, cursorPos[ii*2],cursorPos[ii*2+1], cursor, true, CPixelRGB8(255,0,255));
	;
				i.loadDynamicImage((uchar* )lpvBits, mWidth, mHeight, Ogre::PF_X8R8G8B8);
			}
			else
			{
				i.loadDynamicImage((uchar* )lpvBits, mWidth, mHeight, Ogre::PF_R5G6B5 );
			//PF_B5G6R5 ??
			}
			i.flipAroundX();
			i.save(filename.ptr());
		}
		else
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
			tempPixel.DrawPattern(cursorPos[ii*2], cursorPos[ii*2+1], cursor, true, CPixelRGB8 (255,0,255));
			image.Save(filename);
		}
		END_TIMER(convert);
	}

	if (BmpFile != INVALID_HANDLE_VALUE) CloseHandle(BmpFile);

	printf("Dump finished\n");
	if (lpvBits) delete[] lpvBits;


}
FastCapture::~FastCapture()
{
#ifdef USE_STDLIB
	if (BmpFile) fclose(BmpFile);
#else
	if (BmpFile != INVALID_HANDLE_VALUE) CloseHandle(BmpFile);
#endif
	if (lpvBits) 
	{
		delete[] lpvBits;
		lpvBits=NULL;

		BinaryFile file(true, fn+".info");
		file.pack(cursorPos);
		file.packInt(m_nCount);
		file.packInt(mWidth);
		file.packInt(mHeight);
		file.packInt((int)bitmapSize);
		file.close();

		if(mbAskBeforeConverting)
		{
			if(Msg::confirm("do you want to convert %s to jpeg sequences?", fn))
				FastCapture_convert(fn.ptr());
		}
		else
			FastCapture_convert(fn.ptr());
	}
}

bool FastCapture::Screenshot(HDC SurfDC, int Width, int Height)
{
	if(mWidth==0)
	{
		mWidth=Width;
		mHeight=Height;
	}
	else
		Msg::verify(mWidth==Width && mHeight==Height, "Error! size changed");

	if(last_x!=-1)
	{
		cursorPos.resize(m_nCount*2+2);
		cursorPos[m_nCount*2]=last_x;
		cursorPos[m_nCount*2+1]=last_y;
	}

	bool Success=false;
	HBITMAP OffscrBmp=NULL; // bitmap that is converted to a DIB
	HDC OffscrDC=NULL;      // offscreen DC that we can select OffscrBmp into
	LPBITMAPINFO lpbi=NULL; // bitmap format info; used by GetDIBits

	try
	{
#define USE_BITBLT
#ifdef USE_BITBLT
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
#else
		// 동작안함.
		if ((OffscrBmp = CreateCompatibleBitmap(SurfDC, Width, Height)) == NULL)

		OffscrDC=SurfDC;
		HBITMAP OldBmp = (HBITMAP)SelectObject(OffscrDC, OffscrBmp);

#endif
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
			if ((lpvBits = new char[lpbi->bmiHeader.biSizeImage]) == NULL)
				ERROR_BREAK(6);
		}

		// Have GetDIBits convert OffscrBmp to a DIB (device-independent bitmap):
		if (!GetDIBits(OffscrDC, OffscrBmp, 0, Height, lpvBits, lpbi,
			DIB_RGB_COLORS)) ERROR_BREAK(7);

		DWORD Written;    // number of bytes written by WriteFile

		bitmapSize=lpbi->bmiHeader.biSizeImage;
		// Write bitmap bits to the file:

		//BEGIN_TIMER(screenshot);

#ifdef USE_STDLIB
		Written = fwrite(lpvBits, lpbi->bmiHeader.biSizeImage, 1, BmpFile);
#else
		if (!WriteFile(BmpFile, lpvBits, lpbi->bmiHeader.biSizeImage,
			&Written, NULL)) ERROR_BREAK(12);
#endif
		END_TIMER(screenshot);

		if (Written < lpbi->bmiHeader.biSizeImage) ERROR_BREAK(12);

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

	m_nCount++;
	return Success;
}





// 매우 느림.
class FastCaptureFltk
{
public:
	HANDLE BmpFile;
	int m_nCount;
	int mWidth, mHeight;
	DWORD bitmapSize;
	uchar *lpvBits;    // pointer to bitmap bits array
	TString fn;

	// opens a dump file
	FastCaptureFltk(const char* FileName);

	// add a screen shot to the dump file
	bool Screenshot(int Width, int Height);

	// closes the dump file after converting it to jpeg sequence
	~FastCaptureFltk();
};


// Screenshot
//    -> FileName: Name of file to save screenshot to
//    -> lpDDS: DirectDraw surface to capture
//    <- Result: Success
//

FastCaptureFltk::~FastCaptureFltk()
{
	if (BmpFile != INVALID_HANDLE_VALUE) CloseHandle(BmpFile);

	if((BmpFile= ::CreateFile(fn.ptr(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL))==INVALID_HANDLE_VALUE)
		Msg::error("????");

	Ogre::Image i;

	for(int ii=0; ii<m_nCount; ii++)
	{
		DWORD read;
		if (!ReadFile(BmpFile, lpvBits, bitmapSize,
			&read, NULL)) ERROR_BREAK(12);
		if (read< bitmapSize) ERROR_BREAK(12);

		i.loadDynamicImage((uchar* )lpvBits, mWidth, mHeight, Ogre::PF_X8R8G8B8);
		i.flipAroundX();

		if(ii%100==0)
			printf("Dump %d/%d saved\n", ii,m_nCount);

		TString filename(sz0::format0("../dump/dump%5d.jpg", ii));
		i.save(filename.ptr());
	}

	if (BmpFile != INVALID_HANDLE_VALUE) CloseHandle(BmpFile);

	if (lpvBits) delete[] lpvBits;
}

bool FastCaptureFltk::Screenshot(int width, int height)
{
	if(mWidth==0)
	{
		mWidth=width;
		mHeight=height;
	}
	else
		Msg::verify(mWidth==width && mHeight==height, "Error! size changed");

	bool Success=false;

	try
	{
		// Reserve memory for bitmap bits:

		bitmapSize=width*height*3;

		if(!lpvBits)
		{
			if ((lpvBits = new uchar[bitmapSize]) == NULL)
				ERROR_BREAK(6);
		}

		uchar *s_buffer=lpvBits;
		int bufferSize=0;
		if(bufferSize<width*height*3)
		{
			delete s_buffer;
			s_buffer=new uchar[width*height*3];
			bufferSize=width*height*3;
		}
#ifndef FL_DLL
		fl_read_image(s_buffer, 0, 0, width, height);
#else
		Msg::error("fl_read_image is not defined in DLL");
#endif
		DWORD Written;    // number of bytes written by WriteFile

		// Write bitmap bits to the file:
		if (!WriteFile(BmpFile, lpvBits, bitmapSize, &Written, NULL)) ERROR_BREAK(12);
		if (Written < bitmapSize) ERROR_BREAK(12);

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

	m_nCount++;
	return Success;
}



/*

[Tip] fseek 함수와 ftell 함수에 대하여...
  관리자      Date : 06-06-24 01:31     Hit : 1144
  트랙백 주소 : http://www.tipssoft.com/bulletin/tb.php/old_bbs/298
[질문 내용]

소스를 분석하는데 아래와 같은 함수들이 있습니다..

fseek(pAdd, 0L, SEEK_END);    <- fseek 가 뭐하는 건지??
lLen = ftell(pAdd);           <- ftell 뭐하는 건지?
도움말을 봐도 사용법을 잘 알수가 없네요..

[답변 내용]

안녕하세요~!

fseek 함수는 seek라는 단어가 말하듯이 파일속에서 위치를 찾는 함수입니다.
파일은 읽으면서 그 위치를 이동하는데, 이런 방식은 너무도 비효율적이죠.
그래서, 자신이 원하는 특정 위치로 바로 이동할수 있는 함수가 존재합니다.
바로, fseek와 같은 함수이죠...

fseek 함수의 첫번째 인자는 현재 열려있는 파일의 포인터이고 두번째 인자는
이동하고 싶은 크기이고, 세번째 인자는 어느 위치에서 이동을 시작할것인가를
지정하는 것입니다. 세번째 인자는 ,

   1. SEEK_SET  ( 파일의 처음 위치에서... )

   2. SEEK_CUR  ( 현재 읽고 있는 시점에서... )

   3. SEEK_END  ( 파일의 끝 위치에서... )

로 나누어 집니다... 그리고 ftell 이라는 함수는 현재 사용하는 파일 포인터의
작업 위치를 알려줍니다.. ftell 이라는 함수는 보통 파일의 크기를 파악할때
많이 사용합니다. fseek 와 ftell함수를 이용하는 간단한 예제를 볼까요,,,

FILE *p_file  = fopen("test.dat", "rb");
if(p_file != NULL){
   int pos1 = ftell(p_file); // 파일의 시작이기 때문에 pos1에 0 이
                             // 들어갑니다..
   fseek(p_file, 0, SEEK_END);   // 파일의 끝에서 0바이트 이동하라고
                                 // 했으니까,,, 현재 위치는 파일의
                                 // 끝이겠죠... ^^;
   int pos2 = ftell(p_file); // 파일의 끝이기 때문에 파일의 크기가 pos2에
                             // 들어가게 된다.
   fclose(p_file);
}

보통 파일의 크기를 알려고 할때, 파일을 직접 열어서 위 코드와 같이 확인하기도
합니다. 그렇게 효율적인 방법은 아니지만 제일 확실한 방법이기도 하죠.. ^^;
그냥, 간단한 예를 하나더 들면,

fseek(p_file, 50, SEEK_SET);

이렇게 적으면 파일의 처음 위치에서 50바이트 떨어진 곳에 작업 시점을 옮기
겠다는 명령어입니다. 작업시점이라는 것은 fread와 같은 읽기 함수나,
fwrite와 같이 쓰기 함수를 사용할때 읽거나 쓰기를 시작하는 시점을 말합니다.

옮기는 크기는 꼭, 양수만 적용되는 것은 아닙니다. 음수도 가능하죠. 예를들어
파일의 끝에서는 양수를 지정해도 의미가 없습니다. 파일의 끝을 벗어나기때문이죠.. 파일의 끝에서 50바이트 이전에 작업위치를 옮기고 싶다면...

fseek(p_file, -50, SEEK_END);

이렇게 하면 파일의 끝에서 50바이트 이전으로 작업 위치를 옮기는 일을 합니다.
그리고 ftell은 그때마다 그 위치를 확인할때, 사용하는 함수이구요, 파일의
크기가 570 바이트이고 fseek를 이용해서 파일의 끝으로 이동했을때,
ftell 이라는 함수를 호출하면, 570이라는 값이 넘어온다는것만 기억하시면
되겠죠.. ^^;

그럼, 즐거운 프로그램하세요~~!

*/
#endif
