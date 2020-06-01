#ifndef CHECKPOINTS_H_
#define CHECKPOINTS_H_
#pragma once
#include "tfile.h"
/// release와 debug 모드 사이에 결과가 다르거나 시스템마다 결과가 다르거나, 컴파일 옵션에 따라 결과가 달라지는 등의
/// 디버깅이 어려운 문제에서, checkpoint마다 저장된 결과와 실행 결과를 비교하여 오류가 생긴 지점을 찾아내기 위한 클래스.

class checkPoints
{
public:
	/// bSave가true인 경우 파일에 저장하고, false인 경우 file에 저장된 값과 비교한다.
	checkPoints(const char* filename, bool bSave, m_real checkThr=0.0, bool throwError=true);
    virtual ~checkPoints(void);

	void check(const vectorn& vec);
	void check(const matrixn& vec);
	void check(const int& n);
	void check(const m_real& n);
private:
	void increment();
	bool isSimilar(const vectorn& vec, const vectorn& vec2);
	BinaryFile mFile;
	bool mbSave;	
	bool mbTrace;
	bool mbThrow;
	int mnCurrCheckPoint;
	const m_real mCheckThreshold;
	
};

#ifdef _MSC_VER
#include <windows.h>
typedef __int64 INT64;
#endif

class Profiler
{
	int m_nCount;
#ifdef _MSC_VER
	INT64 m_int64StartTime;
	INT64 m_int64MaxTime;
	INT64 m_int64AverageTime;
	INT64 m_int64Diff;
	static UINT64 m_ValuePerMilliSecond;
#endif
public:
	Profiler();
	~Profiler(){}

	void start();
	void stop();
	void draw();

};


class Stopwatch
{
private:
	int mState;
	enum {END, START, PAUSE};
public:
	Stopwatch(void);

public:
	~Stopwatch(void) {
	};

	// start()-> stop() ->start()->stop()->end() 이순서로 call되는 경우, 
	// start와 pause()사이의 시간이 누적되서 출력된다.

	// start the timer
	void start();
	
	inline void stop()
	{
#ifdef _MSC_VER
		Msg::verify(mState==START, "already stopped %d", mState);
		mState=PAUSE;
		QueryPerformanceCounter((LARGE_INTEGER*)&m_temp);
		m_pause.QuadPart=m_temp.QuadPart-m_start.QuadPart;	// m_pause는 여때까지 누적된 시간.
#endif
	};

	// end the timer and return the end time
	m_real end() ;

	inline m_real end(const char* str) 
	{
		m_real time=end();
		printf("%s : %f (%g)\n", str, time, time);
		return time;
	}

	inline m_real endOutput(const char* str)
	{
		m_real time=end();
		Msg::output(str, "%f (%g)", time, time);
		return time;
	}

private:	
#ifdef _MSC_VER
	LARGE_INTEGER m_freq, m_start, m_end, m_pause, m_temp;		// 64-bit integer type
#endif
	m_real m_time;
};


class Tracer
{
	TString mFilename;
public:
	Tracer(const char* filename);
	virtual ~Tracer();
	void trace(const char* pszFormat, ...);
};
#endif
