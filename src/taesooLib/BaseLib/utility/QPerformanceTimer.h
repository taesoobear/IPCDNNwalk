#ifndef QPERFORMANCE_TIMER_H
#define QPERFORMANCE_TIMER_H

#include "checkError.h"
/********************************************************************************
	Copyright (C) 2004 Sjaak Priester	

	This is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This file is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Tinter; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
********************************************************************************/

// QPerformanceTimer
// Class to measure performance time
//
// Usage: simply define a QPerformanceTimer variable in a C++ block.
// At construction, it measures the start time.
// At destruction, it measures the stop time. It calculates the difference
// and puts the result (in milliseconds) in the associated int.
// In other words: this class measures and records its own lifetime.
// If the output is set to -1, it's an indication of overflow, but this
// will only occur after more than three weeks (!).
//
// Example:
//
//
//	SomeFunction()
//	{
//		... some code...
//
//		int elapsedTime;
//		{	// start of the block we want to monitor
//			QPerformanceTimer(elapsedTime);
//
//			... some lengthy process...
//
//		}	// end of block, elapsed time is recorded in elapsedTime
//
//		printf("Time = %d milliseconds", elapsedTime);
//	}
//
// Version 1.0 (C) 2004, Sjaak Priester, Amsterdam.
// mailto:sjaak@sjaakpriester.nl

#pragma once

// 제일 마지막에 include할 것. ( windows.h를 include하기 때문.)

#ifdef _MSC_VER
#define NOMINMAX

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

#else
#define USE_GETTIMEOFDAY 
#ifdef USE_GETTIMEOFDAY
#include <sys/time.h>
#else
#include <ctime>
#endif

#endif
class QPerformanceTimer
{
public:
	QPerformanceTimer(int& MilliSeconds)
		: m_Output(MilliSeconds)
	{

#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Start);
#endif
	}

	~QPerformanceTimer(void)
	{
#ifdef _MSC_VER
		LARGE_INTEGER stop;
		LARGE_INTEGER freq;
		::QueryPerformanceCounter(&stop);
		::QueryPerformanceFrequency(&freq);

		stop.QuadPart -= m_Start.QuadPart;
		stop.QuadPart *= 1000;
		stop.QuadPart /= freq.QuadPart;

		if (stop.HighPart != 0) m_Output = -1;
		else m_Output = stop.LowPart;
#endif
	}
protected:
#ifdef _MSC_VER

	LARGE_INTEGER m_Start;
#endif
	int& m_Output;
};
/*
 * usage
 *
 * QPerformanceTimerCount2 gTimer;
 *
 *
 * for (int i=0; i<100; i++)
 * {
 * gTimer.start();
 *
 * //do some thing
 *
 * gTimer.pause();
 * }
 *
 * // print sum of time between start and pause 
 * printf("%gms\n",(double)gTimer.stop()/1000.0);
 *
 *
*/

#define BEGIN_TIMER(x)	QPerformanceTimer2 x
#define END_TIMER(x)	printf("%s Time= %d milliseconds\n", #x, x.stop())
#define END_TIMER2(x)	printf("%s Time= %lu microseconds\n", #x, x.stop2())

// taesoo modified
class QPerformanceTimer2
{
public:
	QPerformanceTimer2()
	{
		start();
	}

	~QPerformanceTimer2(void){}

	void start()
	{
#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Start);
#else
#ifdef USE_GETTIMEOFDAY
		gettimeofday(&m_Start, NULL);
#endif
#endif
	}

	int stop()
	{
#ifdef _MSC_VER
		LARGE_INTEGER stop;
		LARGE_INTEGER freq;
		::QueryPerformanceCounter(&stop);
		::QueryPerformanceFrequency(&freq);

		stop.QuadPart -= m_Start.QuadPart;
		stop.QuadPart *= 1000;
		stop.QuadPart /= freq.QuadPart;

		if (stop.HighPart != 0) return -1;
		return stop.LowPart;
#else
#ifdef USE_GETTIMEOFDAY
		gettimeofday(&m_Stop, NULL);
		//unsigned long microseconds=
		//	(m_Stop.tv_sec-m_Start.tv_sec)*1000000+(m_Stop.tv_usec-m_Start.tv_usec);
		int milliseconds=
			int((m_Stop.tv_sec-m_Start.tv_sec)*1000+(m_Stop.tv_usec-m_Start.tv_usec)/1000);
		return milliseconds;
#endif
#endif
	}
	unsigned long stop2() // returns microseconds
	{
#ifdef _MSC_VER
		LARGE_INTEGER stop;
		LARGE_INTEGER freq;
		::QueryPerformanceCounter(&stop);
		::QueryPerformanceFrequency(&freq);

		stop.QuadPart -= m_Start.QuadPart;
		stop.QuadPart *= 1000000;
		stop.QuadPart /= freq.QuadPart;

		if (stop.HighPart != 0) return -1;
		return stop.LowPart;
#else
#ifdef USE_GETTIMEOFDAY
		gettimeofday(&m_Stop, NULL);
		unsigned long microseconds=
			(m_Stop.tv_sec-m_Start.tv_sec)*1000000+(m_Stop.tv_usec-m_Start.tv_usec);
		//int milliseconds=
	//		int((m_Stop.tv_sec-m_Start.tv_sec)*1000+(m_Stop.tv_usec-m_Start.tv_usec)/1000);
		return microseconds;
#endif
#endif
	}

protected:
#ifdef _MSC_VER
	LARGE_INTEGER m_Start;
#else
#ifdef USE_GETTIMEOFDAY // HIGH RESOLUTION but SLOWWWWWW
	struct timeval m_Start;
	struct timeval m_Stop;
#endif
#endif
};

// add
class QPerformanceTimerCount
{
public:
	QPerformanceTimerCount(int gran=10, const char* msg=NULL)
	{
#ifdef _MSC_VER
		m_Sum.QuadPart=0;
		m_count=0;
		m_gran=gran;
		minTime=INT_MAX;
		maxTime=0;
		_msg=msg;
#endif
	}

	~QPerformanceTimerCount(void){}

	void start()
	{
#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Start);
#endif
	}

	void stop()
	{
#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Stop);

		m_Sum.QuadPart+=m_Stop.QuadPart-m_Start.QuadPart;
		m_count++;
		if(m_count==m_gran)
		{
			int milli=end();
			minTime=MIN(milli, minTime);
			maxTime=MAX(milli, maxTime);
			
			Msg::print("%s: %6.2f msec per call (%d calls took %d msec, global min %d max %d)\n", _msg.c_str(), float(milli)/float(m_gran), m_gran, milli, minTime, maxTime);
            
			m_count=0;
			m_Sum.QuadPart=0;
		}
#endif
	}

	

protected:
	std::string _msg;
#ifdef _MSC_VER
	int end()
	{
		LARGE_INTEGER freq;

		::QueryPerformanceFrequency(&freq);

		m_Sum.QuadPart *= 1000;
		m_Sum.QuadPart /= freq.QuadPart;

		if (m_Sum.HighPart != 0) return -1;
		return m_Sum.LowPart;
	}
	LARGE_INTEGER m_Start;
	LARGE_INTEGER m_Stop;
	LARGE_INTEGER m_Sum;
#endif
	int m_count, m_gran, minTime, maxTime;
};

class QPerformanceTimerCount2
{
public:
	QPerformanceTimerCount2()
	{
#ifdef _MSC_VER
		reset();
		minTime=INT_MAX;
		maxTime=0;
#else
		reset();
#endif
	}

	~QPerformanceTimerCount2(void){}

	inline void reset()
	{
#ifdef _MSC_VER
		m_Sum.QuadPart=0;
#else
		m_Sum=0;
#endif
		m_count=0;
		state=0;
	}

	inline void start()
	{
#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Start);
#else

#ifdef USE_GETTIMEOFDAY
		gettimeofday(&m_Start, NULL);
#else
		m_Start=clock();
#endif
		
#endif
		state=1;
	}

	inline void pause()
	{
		if(state!=1) 
			return;
#ifdef _MSC_VER
		::QueryPerformanceCounter(&m_Stop);

		m_Sum.QuadPart+=m_Stop.QuadPart-m_Start.QuadPart;
#else

#ifdef USE_GETTIMEOFDAY
		gettimeofday(&m_Stop, NULL);
		unsigned long microseconds=
			(m_Stop.tv_sec-m_Start.tv_sec)*1000000+(m_Stop.tv_usec-m_Start.tv_usec);
#else
		m_Stop=clock();
		unsigned long microseconds= (m_Stop-m_Start)*1000000/CLOCKS_PER_SEC;
#endif
		m_Sum+=microseconds;
#endif
		m_count++;
		state=2;
	}

	inline long stop()
	{
		if(state==1) pause();
		int micro=end();
#ifdef _MSC_VER
		minTime=MIN(micro, minTime);
		maxTime=MAX(micro, maxTime);
#endif
		reset();
		return micro;
	}

protected:
	int state;
	inline long end()
	{
#ifdef _MSC_VER
		LARGE_INTEGER freq;

		::QueryPerformanceFrequency(&freq);

		m_Sum.QuadPart *= 100000;
		m_Sum.QuadPart /= freq.QuadPart;

		if (m_Sum.HighPart != 0) return -1;
		return m_Sum.LowPart;
#else
		return m_Sum;
#endif
	}
#ifdef _MSC_VER
	LARGE_INTEGER m_Start;
	LARGE_INTEGER m_Stop;
	LARGE_INTEGER m_Sum;
#else
#ifdef USE_GETTIMEOFDAY // HIGH RESOLUTION but SLOWWWWWW
	struct timeval m_Start;
	struct timeval m_Stop;
#else  // LOW RESOLUTION
	clock_t m_Start;
	clock_t m_Stop;
#endif
	long m_Sum;
#endif
	int m_count, m_gran, minTime, maxTime;
};

class FractionTimer 
{
	static QPerformanceTimerCount2 gTimerInside;
	static QPerformanceTimerCount2 gTimerOutside;
	static int gCount;
	static double gOverhead;
public:
	inline FractionTimer()
	{
		gTimerInside.start();
		gTimerOutside.pause();
		gCount++;
	}

	static void init()
	{
		{
			// measure profiling overhead
			gTimerInside.reset();
			gTimerOutside.reset();
			gTimerOutside.start();
			int N=100000;
			for (int i=0; i<N; i++)
			{
				gTimerInside.start();
				gTimerInside.pause();
			}
			gOverhead=gTimerOutside.stop()/1000.0/(double)N;
			//printf("gOverhead %g %g\n", gOverhead*N, gTimerInside.stop()/1000.0);
		}
		gTimerInside.reset();
		gTimerOutside.reset();
		gTimerOutside.start();
		gCount=0;
	}
	static double stopInside()
	{
		return (double)gTimerInside.stop()/1000.0;
	}
	static double stopOutside()
	{
		return (double)gTimerOutside.stop()/1000.0;
	}

	static void printSummary(const char* msg, const char* insideName, const char* outsideName)
	{
		double inside=stopInside();
		double outside=stopOutside();
		double overhead=gOverhead*gCount;
#ifdef USE_GETTIMEOFDAY 
		double errorCorrection=0.4;
#else
		double errorCorrection=0.4;
#endif
		printf("Profiling %s finished: %s %gms, %s %gms, profiling overhead %gms, total time %gms\n", msg, insideName, inside-overhead-overhead*errorCorrection, outsideName, outside-overhead-overhead*errorCorrection, overhead*2, inside+outside-overhead*errorCorrection*2);
		// init without measuring
		gTimerInside.reset();
		gTimerOutside.reset();
		gCount=0;
		gTimerOutside.start();
	}

	static int count()
	{
		return gCount;
	}
	inline ~FractionTimer(void)
	{
		gTimerOutside.start();
		gTimerInside.pause();
	}
};
#endif
