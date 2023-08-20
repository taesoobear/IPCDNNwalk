#include "stdafx.h"
#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/utility/util.h"
#include "../BaseLib/utility/operatorString.h"
#include "../BaseLib/math/intervals.h"
#include "GnuPlot.h"


#ifdef _MSC_VER
#include <stdio.h>
#include <process.h>

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

#endif
bool gnuPlot::g_bPlot=false;
int gnuPlot::g_sx=640;
int gnuPlot::g_sy=480;
intervalN gnuPlot::mRange;
bool gnuPlot::g_bYUP=true;
void gnuPlot::setSyscallState(bool bPlot)
{
	gnuPlot::g_bPlot=bPlot;
}			 

void gnuPlot::setImageSize(int x, int y)
{
	gnuPlot::g_sx=x;
	gnuPlot::g_sy=y;
}

bool gnuPlot::setYUP(bool b)
{
	bool prev=gnuPlot::g_bPlot;
	gnuPlot::g_bYUP=b;
	return prev;
}

void gnuPlot::systemCall(const char* path, const char* filename, bool bwait)
{

#ifdef _MSC_VER
	char szCurrDirectory[512];
	GetCurrentDirectory(512, szCurrDirectory);

	TString currDir(szCurrDirectory);

	for(int i=0; i<currDir.length(); i++)
	{
		if(currDir[currDir.length()-i-1]=='\\')
		{
			currDir=currDir.left(-i-1);
			currDir+="\\gnuPlot";
			printf("%s\n", currDir.ptr());
			break;
		}
	}

	TString program;

	program=currDir+"\\bin\\wgnuPlot";

	TString workingFolder;
	workingFolder=currDir+"\\"+path;

	SetCurrentDirectory(workingFolder);

	printf("%s %s\n", program.ptr(), filename);

	int mode;
	if(bwait)
		mode=_P_WAIT;
	else
		mode=_P_NOWAIT ;
	if(_spawnl(mode, program.ptr(), program.ptr(), filename, 0)==-1)
	{
		int noerr=GetLastError();


		TCHAR* pTemp = NULL;
		int nLen = ::FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_IGNORE_INSERTS |
			FORMAT_MESSAGE_FROM_SYSTEM,
			NULL, 
			noerr,
			MAKELANGID( LANG_NEUTRAL, SUBLANG_DEFAULT ),
			(LPTSTR)&pTemp, 
			1, 
			NULL );

		printf("Spawn error : %d %s\n", noerr, pTemp);

		::LocalFree( pTemp );
	}

	SetCurrentDirectory(szCurrDirectory);
#endif
}

void gnuPlot::plotArrow(const char* filename, const matrixn& mat, const char* title, const char* xlabel, const char* ylabel)
{
	TString fn, dir, lfn;
	bool bPNG;

	processFilename(filename, bPNG, fn, dir, lfn);

	writeData(TString("")+fn+".dat", mat);

	// write script file
	FILE* script;
	script=fopen(TString("")+fn+".dem","wt");
	if(!script) Msg::error("gnuPlot file open error %s",filename);

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	if(title)
		fprintf(script, "set title \'%s\'\nset key off\n", title);

	if(mRange.size()==2)
	{
		fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
		fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
	}

	fprintf(script, "set xlabel \"%s\"\n", xlabel);
	fprintf(script, "set ylabel \"%s\"\n", ylabel);

	fprintf(script,	"plot '%s' using 1:2:3:4 with vectors\n", (const char*)(lfn+".dat"));	
	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");
	fclose(script);

	if(gnuPlot::g_bPlot || bPNG)
		systemCall(dir, lfn+".dem", bPNG);
}

void gnuPlot::plot1DSignal(const char* filename, const vectorn& data, const char* title, const char* label)
{
	matrixn data2;
	data2.setSize(data.size(), 2);
	data2.column(0).colon(0.0, 1.0, data.size());
	data2.column(1)=data;

	plot2DSignal(filename, data2,title, "t", label);
}

void gnuPlot::plot2DSignal(const char* filename, const matrixn& data, const char* title, const char* xlabel, 
							   const char* ylabel)
{
	matrixn newData;

	newData.setSize(data.rows()-1, 4);

	vectorn delta;
	for(int i=0; i<data.rows()-1; i++)
	{
		newData.row(i).range(0,2)=data.row(i);

		delta.sub(data.row(i+1), data.row(i));

		newData.row(i).range(2,4)=delta;
	}

	plotArrow(filename, newData, title, xlabel, ylabel);
}

void gnuPlot::mergedPlotScattered(const TStrings& filenames, int dim, const char* title, const char* xlabel, 
									  const char* ylabel, const char* zlabel)
{
	TString fn, dir, lfn;
	TStrings titles;
	titles.trimSamePrefix(filenames);

	TString prefix;
	prefix=filenames.prefix();
	bool bPNG;
	processFilename(filenames[0], bPNG, fn, dir, lfn);
	// gnuPlot/fn.dem
	FILE* script;
	TString scriptfn=TString("")+dir+"/"+prefix+"Merge"+title+".dem";
	script=fopen(scriptfn,"wt");
	if(!script) Msg::error("gnuPlot file open error %s",scriptfn.ptr());

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	if(title)
	{
		fprintf(script, "set title \'%s\'\n", title);
	}
	fprintf(script, "set key on\n");

	if(dim==2)
	{
		fprintf(script, "set xlabel \"%s\"\n", xlabel);
		fprintf(script, "set ylabel \"%s\"\n", ylabel);

		if(mRange.size()==dim)
		{
			fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
			fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
		}

		fprintf(script,	"plot '%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[0].ptr());

		for(int i=1; i<filenames.size(); i++)
		{
			processFilename(filenames[i], bPNG, fn, dir, lfn);
			fprintf(script, ",'%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[i].ptr());
		}
		fprintf(script, "\n\n");
	}
	else
	{	
		if(g_bYUP)
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", zlabel);
			fprintf(script, "set zlabel \"%s\"\n", ylabel);

			if(mRange.size()==dim)
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);			
			}
		}
		else
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", ylabel);
			fprintf(script, "set zlabel \"%s\"\n", zlabel);

			if(mRange.size()==dim)
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);			
			}
		}

		fprintf(script,	"splot '%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[0].ptr());

		for(int i=1; i<filenames.size(); i++)
		{
			processFilename(filenames[i], bPNG, fn, dir, lfn);
			fprintf(script, ",'%s' title '%s'\\\n", (const char*)(lfn+".dat"), titles[i].ptr());
		}
		fprintf(script, "\n\n");
	}

	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");

	fclose(script);


	if(gnuPlot::g_bPlot || bPNG)
		systemCall(dir, scriptfn, bPNG);
}

void gnuPlot::plotScattered(const char* filename, const matrixn& mat, const char* title, const char* xlabel, const char* ylabel, const char* zlabel)
{
	TString fn, dir, lfn;
	bool bPNG;
	processFilename(filename, bPNG, fn, dir, lfn);

	writeData(TString("")+fn+".dat", mat);

	// gnuPlot/fn.dem
	FILE* script;
	script=fopen(TString("")+fn+".dem","wt");
	if(!script) Msg::error("gnuPlot file open error %s",filename);

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",g_sx, g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	if(title)
	{
		fprintf(script, "set title \'%s\'\nset key off\n", title);
	}

	if(mat.cols()==2)
	{
		fprintf(script, "set xlabel \"%s\"\n", xlabel);
		fprintf(script, "set ylabel \"%s\"\n", ylabel);

		if(mRange.size()==mat.cols())
		{
			fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
			fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
		}

		fprintf(script,	"plot '%s'\n", (const char*)(lfn+".dat"));		
	}
	else
	{	
		if(g_bYUP)
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", zlabel);
			fprintf(script, "set zlabel \"%s\"\n", ylabel);

			if(mRange.size()==mat.cols())
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);			
			}
		}
		else
		{
			fprintf(script, "set xlabel \"%s\"\n", xlabel);
			fprintf(script, "set ylabel \"%s\"\n", ylabel);
			fprintf(script, "set zlabel \"%s\"\n", zlabel);

			if(mRange.size()==mat.cols())
			{
				fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
				fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
				fprintf(script, "set zrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);			
			}
		}
		fprintf(script,	"splot '%s'\n", (const char*)(lfn+".dat"));		
	}

	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");

	fclose(script);


	if(gnuPlot::g_bPlot || bPNG)
		systemCall(dir, lfn+".dem", bPNG);

	// script for line fitting.
	if(mat.cols()==2)
	{
		// gnuPlot/line.fnc
		script=fopen("../gnuPlot/line.fnc", "wt");
		fprintf(script, "l(x) = y0 + m*x\n");
		fclose(script);

		// gnuPlot/fn_fit.dem		
		script=fopen(TString("")+fn+"_fit.dem","wt");
		if(!script) Msg::error("gnuPlot file open error %s",filename);

		//	set title 'data for fit demo'
		fprintf(script, "plot '%s.dat'\n", fn.ptr());
		fprintf(script, "#set xlabel \"speed\"\n");
		fprintf(script, "#set ylabel \"deltaTurn\"\n");
		fprintf(script, "#set yrange [-0.5:0.5]\n");
		fprintf(script, "load 'line.fnc'\n");
		fprintf(script, "y0=0.0\n");
		fprintf(script, "m=0.0\n");
		fprintf(script, "show variables\n");
		fprintf(script, "fit l(x) '%s.dat' via y0, m\n", fn.ptr());
		fprintf(script,	"plot '%s.dat', l(x)\n", fn.ptr());
		fprintf(script, "pause -1 \"Hit return to continue\"\n");
		fclose(script);
	}
}


void gnuPlot::writeData(const char* filename, const matrixn& mat)
{	
	FILE* data;
	data=fopen(filename,"wt");
	if(!data) Msg::error("gnuPlot file open error %s",filename);

	enum {X=0, Y=1, Z=2};
	for(int i=0; i<mat.rows(); i++)
	{

		if(mat.cols()==2)
			fprintf(data,"%f %f\n", mat[i][X], mat[i][Y]);
		else if(mat.cols()==3)	
		{
			if(g_bYUP)
			{	// x, z, y 를 사용했다. 이유는 y축을 위로 향하게 plotting 하고 싶어서.		
				fprintf(data,"%f %f %f\n", mat[i][X], mat[i][Z], mat[i][Y]);
			}
			else
				fprintf(data,"%f %f %f\n", mat[i][X], mat[i][Y], mat[i][Z]);
		}
		else if(mat.cols()==4)
			fprintf(data,"%f %f %f %f\n", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
		else
		{
			TString temp;
			for(int j=0; j<mat.cols(); j++)
				temp.add("%f ", mat[i][j]);
			fprintf(data, "%s\n", temp.ptr());
		}
	}
	fclose(data);
}

void gnuPlot::processFilename(const char* filename, bool& bPNG, TString& fn, TString& dir, TString& lfn)
{
	
	fn=filename;

	bPNG=false;
	if(fn.right(4).toUpper()==".PNG")
	{
		bPNG=true;
		fn=fn.left(-4);		
	}

	lfn=sz1::filename(fn, dir);
}

void gnuPlot::setRange(const intervalN& range)
{
	mRange=range;
}

void gnuPlot::unsetRange()
{
	// draw using default setting.
	mRange.setSize(0);
}

gnuPlotQueue::gnuPlotQueue(const char* ffn, int dm, const char* title, const char* xlabel, 
						const char* ylabel, const char* zlabel)
						:filename(ffn), dim(dm)
{
	gnuPlot::processFilename(filename, bPNG, fn, dir, lfn);

	// gnuPlot/fn.dem
	script=fopen(TString("")+fn+".dem","wt");
	if(!script) Msg::error("gnuPlot file open error %s",filename.ptr());

	if(bPNG)
	{
		fprintf(script, "set terminal png size %d,%d\n",gnuPlot::g_sx, gnuPlot::g_sy);
		fprintf(script, "set output \'%s\'\n", (lfn+".PNG").ptr());
	}

	fprintf(script, "set xlabel \"%s\"\n", xlabel);
	fprintf(script, "set ylabel \"%s\"\n", ylabel);

	if(dim==3)
		fprintf(script, "set zlabel \"%s\"\n", zlabel);

	if(title)
		fprintf(script, "set title \'%s\'\nset key on\n", title);

	bFirst=true;
}

gnuPlotQueue::~gnuPlotQueue()
{
	fprintf(script, "\n\n");

	if(!bPNG)
		fprintf(script, "pause -1 \"Hit return to continue\"\n");
	fclose(script);

	if(gnuPlot::g_bPlot || bPNG)
		gnuPlot::systemCall(dir, lfn+".dem", bPNG);
}

void gnuPlotQueue::setRange(const intervalN& mRange)
{
	fprintf(script, "set xrange [%f:%f]\n", mRange.start()[0], mRange.end()[0]);
	fprintf(script, "set yrange [%f:%f]\n", mRange.start()[1], mRange.end()[1]);
	if(dim==3)
	fprintf(script, "set zrange [%f:%f]\n", mRange.start()[2], mRange.end()[2]);
}
void gnuPlotQueue::setRange(const intervals& mRange)
{
	fprintf(script, "set xrange [%f:%f]\n", mRange.row(0).start(), mRange.row(0).end());
	fprintf(script, "set yrange [%f:%f]\n", mRange.row(1).start(), mRange.row(1).end());
	if(dim==3)
	fprintf(script, "set zrange [%f:%f]\n", mRange.row(2).start(), mRange.row(2).end());
}

void gnuPlotQueue::plotSignal(const vectorn& data, const char* label)
{
	matrixn temp(data.size(),2);
	temp.column(0).colon(0, 1);
	temp.column(1)=data;

	plotScattered(temp, label);
}
/// 2D or 3D data plotting: data(n,2 or 3); when n==3 (x y z) where x,y,z are column vector.
/**
	Filename examples: "asfd" or "asdf.png".
	When a filename ends with ".png", .png file will be made. (You need gnuPlot.)
	Otherwise, .dem file will be made. (You can see the graph by running .dem in gnuPlot.)
*/
void gnuPlotQueue::plotScattered(const matrixn& data, const char* label)
{
	TString datfile("");
	datfile+=fn+"_"+TString(label)+".dat";
	writeData(datfile.ptr(), data);

	if(bFirst)
	{
		if(dim==3)
			fprintf(script, "splot '%s' title '%s'\\\n", datfile.ptr(), label);
		else
			fprintf(script, "plot '%s' title '%s'\\\n", datfile.ptr(), label);
		bFirst=false;
	}
	else
	{
		if(dim==3)
			fprintf(script, ",'%s' title '%s'\\\n", datfile.ptr(), label);
		else
			fprintf(script, ",'%s' title '%s'\\\n", datfile.ptr(), label);
	}

}

void gnuPlotQueue::plotParametric(const matrixn& data, const char* label, int nstep)
{
	TString datfile("");
	datfile+=fn+"_"+TString(label)+".dat";
	writeData(datfile.ptr(), data, nstep);

	if(bFirst)
	{
		if(dim==3)
			fprintf(script, "splot '%s' title '%s' with lines\\\n", datfile.ptr(), label);
		else
			fprintf(script, "plot '%s' title '%s' with lines\\\n", datfile.ptr(), label);
		bFirst=false;
	}
	else
	{
		if(dim==3)
			fprintf(script, ",'%s' title '%s' with lines\\\n", datfile.ptr(), label);
		else
			fprintf(script, ",'%s' title '%s' with lines\\\n", datfile.ptr(), label);
	}	
}

void gnuPlotQueue::processFilename(const char* filename, bool& bPNG, TString& fn, TString& dir, TString& lfn)
{
	ASSERT(0);
}

void gnuPlotQueue::writeData(const char* filename, const matrixn& mat, int nstep)
{	
	FILE* data;
	data=fopen(filename,"wt");
	if(!data) Msg::error("gnuPlot file open error %s",filename);

	enum {X=0, Y=1, Z=2};
	for(int i=0; i<mat.rows(); i++)
	{

		if(mat.cols()==2)
			fprintf(data,"%f %f\n", mat[i][X], mat[i][Y]);
		else if(mat.cols()==3)	
		{
			fprintf(data,"%f %f %f\n", mat[i][X], mat[i][Y], mat[i][Z]);
		}
		else if(mat.cols()==4)
			fprintf(data,"%f %f %f %f\n", mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
		else
		{
			TString temp;
			for(int j=0; j<mat.cols(); j++)
				temp.add("%f ", mat[i][j]);
			fprintf(data, "%s\n", temp.ptr());
		}

		if((i+1)%nstep==0)
			fprintf(data, "\n");
	}
	fclose(data);
}
