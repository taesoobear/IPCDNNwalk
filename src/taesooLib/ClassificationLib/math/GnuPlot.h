#ifndef GNUPLOT_H_
#define GNUPLOT_H_
#pragma once
#include "../../BaseLib/math/intervalN.h"
// namespace 에서 static class로 디자인 변경.
class gnuPlot
{
public:
	// if(bPlot==True) the plot result will be appear on the screen automatically.
	// otherwise, you should run .dem file manually. default==false.
	static void setSyscallState(bool bPlot);
	// affects only .png output.
	static void setImageSize(int x, int y);	

	// range
	static void setRange(const intervalN& range);
	static void unsetRange();	// draw using default setting.
	static bool setYUP(bool b);

	/// 2D arrow ploting: data(n,4) (x, y, dx, dy) where x, y, dx, dy are column vector)
	/**
		Filename examples: "asfd" or "asdf.png".
		When a filename ends with ".png", .png file will be made. (You need gnuPlot.)
		Otherwise, .dem file will be made. (You can see the graph by running .dem in gnuPlot.)
	*/
	static void plotArrow(const char* filename, const matrixn& data, const char* title=NULL, const char* xlabel="X axis", 
						const char* ylabel="Y axis");
	/// 2D arrow ploting: data(n,2); (x, y) where x, y are column vector)
	static void plot2DSignal(const char* filename, const matrixn& data, const char* title=NULL, const char* xlabel="X axis", 
						const char* ylabel="Y axis");

	static void plot1DSignal(const char* filename, const vectorn& data, const char* title=NULL, const char* label="value");
	/// 2D or 3D data plotting: data(n,2 or 3); when n==3 (x y z) where x,y,z are column vector.
	/**
		Filename examples: "asfd" or "asdf.png".
		When a filename ends with ".png", .png file will be made. (You need gnuPlot.)
		Otherwise, .dem file will be made. (You can see the graph by running .dem in gnuPlot.)
	*/
	static void plotScattered(const char* filename, const matrixn& data, const char* title=NULL, const char* xlabel="X axis", 
						const char* ylabel="Y axis", const char* zlabel="Z axis");

	static void mergedPlotScattered(const TStrings& filenames, int dim, const char* title=NULL, const char* xlabel="X axis", 
		const char* ylabel="Y axis", const char* zlabel="Z axis");
private:
	static bool g_bYUP;
	static intervalN mRange;
	static bool g_bPlot;
	static int g_sx, g_sy;
	static void systemCall(const char* path, const char* filename, bool bwait);
	static void writeData(const char* filename, const matrixn& data);
	static void processFilename(const char* filename, bool& bPNG, TString& fn, TString& dir, TString& lfn);
	friend class gnuPlotQueue;
};

// namespace 에서 static class로 디자인 변경.
// 한번에 여러개의 plot을 그릴때 사용.
class gnuPlotQueue
{
public:
	gnuPlotQueue(const char* filename, int dim, const char* title=NULL, const char* xlabel="X axis", 
						const char* ylabel="Y axis", const char* zlabel="Z axis");

	~gnuPlotQueue();
	// range
	void setRange(const intervalN& range);
	
	void plotSignal(const vectorn& data, const char* label);
	/// 2D or 3D data plotting: data(n,2 or 3); when n==3 (x y z) where x,y,z are column vector.
	/**
		Filename examples: "asfd" or "asdf.png".
		When a filename ends with ".png", .png file will be made. (You need gnuPlot.)
		Otherwise, .dem file will be made. (You can see the graph by running .dem in gnuPlot.)
	*/
	void plotScattered(const matrixn& data, const char* label);
	void plotParametric(const matrixn& data, const char* label, int nstep=INT_MAX);

private:
	int dim;
	TString filename;
	TString fn, dir, lfn;
	bool bPNG;
	FILE* script;
	bool bFirst;
	void writeData(const char* filename, const matrixn& data, int nstep=INT_MAX);
	void processFilename(const char* filename, bool& bPNG, TString& fn, TString& dir, TString& lfn);
};
#endif
