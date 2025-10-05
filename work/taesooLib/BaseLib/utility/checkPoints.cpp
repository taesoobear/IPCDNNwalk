#include "stdafx.h"
#include "../math/mathclass.h"
#include "./checkPoints.h"

#ifdef _MSC_VER
#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN		// Windows 헤더에서 거의 사용되지 않는 내용을 제외시킵니다.
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
void GetSimpleAccurateTime(LARGE_INTEGER &iCounter)
{
    ::QueryPerformanceCounter(&iCounter);
    /*
	DWORD dwLow,dwHigh;
	__asm {
		rdtsc
		mov	dwLow, eax
		mov	dwHigh, edx
	}
	iCounter.QuadPart = ((unsigned __int64)dwHigh << 32)
                         | (unsigned __int64)dwLow; */
}
#endif




checkPoints::checkPoints(const char* filename, bool bSave, m_real checkThr, bool throwError)
:mbSave(bSave),
mFile(bSave, filename),
mCheckThreshold(checkThr),
mbThrow(throwError),
mbTrace(false)
{
	mnCurrCheckPoint=0;
}

checkPoints::~checkPoints(void)
{
}

bool checkPoints::isSimilar(const vectorn& vec, const vectorn& vec2)
{
	if(vec.size()!=vec2.size()) return false;
	for(int i=0; i<vec.size(); i++)
	{
		if(ABS(vec2[i]-0.0)<=10000)
		{
			if(ABS(vec[i]-vec2[i])>mCheckThreshold)
			{
				printf("error1 %g %g", vec[i], vec2[i]);
				return false;
			}
		}
		else
		{
			m_real frac=vec[i]/vec2[i];
			if(ABS(frac-1.0)>mCheckThreshold)
			{
				printf("error2 %g %g", vec[i], vec2[i]);
				return false;
			}
			if(!(ABS(frac-1.0)<=mCheckThreshold))
			{
				printf("error3 %g %g", vec[i], vec2[i]);
				return false;
			}
		}
	}
	return true;
}

void checkPoints::check(const vectorn& vec)
{
	if(mbTrace) printf("%d %s\n", mnCurrCheckPoint, vec.output().ptr());
	if(mbSave)
	{
		mFile.pack(vec);
	}
	else
	{
		vectorn temp;
		mFile.unpack(temp);

		if(!isSimilar(temp, vec))
		{
			TString errMsg;
			errMsg.add("checkpoint %d:\n", mnCurrCheckPoint);
			errMsg.add("temp %s\n", temp.output().ptr());
			errMsg.add("vec %s\n", vec.output().ptr());
			errMsg.add("div %s\n", vec.Each(s2::DIV, temp).output().ptr());
			printf("%s", errMsg.ptr());
			MOutputToFile("checkpoint.log", ("%s", errMsg.ptr()));

			ASSERT(0);
			if(mbThrow) throw "check error";
		}
	}

	increment();
}
void checkPoints::check(const matrixn& mat)
{
	if(mbTrace) printf("%d %s\n", mnCurrCheckPoint, mat.output().ptr());
	if(!mat.isValid())
	{
		printf("Invalid mat %d: %s\n", mnCurrCheckPoint, mat.output().ptr());
		assert(0);
	}

	if(mbSave)
	{
		mFile.pack(mat);
	}
	else
	{
		matrixn temp;
		mFile.unpack(temp);

		ASSERT(temp.rows()==mat.rows());
		ASSERT(temp.cols()==mat.cols());
		for(int i=0; i<temp.rows(); i++)
		{
			if(!isSimilar(temp.row(i), mat.row(i)))
			{
				TString errMsg;
				errMsg.add("checkpoint %d:\n", mnCurrCheckPoint);
				errMsg.add("temp row %d %s\n", i, temp.row(i).output().ptr());
				errMsg.add("mat row %d %s\n", i, mat.row(i).output().ptr());
				errMsg.add("div %s\n", mat.row(i).Each(s2::DIV, temp.row(i)).output().ptr());

				printf("%s", errMsg.ptr());
				MOutputToFile("checkpoint.log", ("%s", errMsg.ptr()));

				ASSERT(0);
				if(mbThrow) throw "check error";
			}
		}
	}
	increment();
}

void checkPoints::check(const int& n)
{
	if(mbSave)
	{
		mFile.packInt(n);
	}
	else
	{
		int temp=mFile.unpackInt();
		if(temp!=n)
		{
			TString errMsg;
			errMsg.add("checkpoint %d:\n", mnCurrCheckPoint);
			errMsg.add("temp %d n %d\n", temp, n);
			printf("%s", errMsg.ptr());
			MOutputToFile("checkpoint.log", ("%s", errMsg.ptr()));
			ASSERT(0);
			if(mbThrow) throw "check error";
		}
	}
	increment();
}
void checkPoints::check(const m_real& n)
{
	if(mbSave)
	{
		mFile.packFloat(n);
	}
	else
	{
		m_real temp=mFile.unpackFloat();
		if(ABS(temp-n)>mCheckThreshold)
		{
			TString errMsg;
			errMsg.add("checkpoint %d:\n", mnCurrCheckPoint);
			errMsg.add("temp %g n %g\n", temp, n);
			printf("%s", errMsg.ptr());
			MOutputToFile("checkpoint.log", ("%s", errMsg.ptr()));

			ASSERT(0);
			if(mbThrow) throw "check error";
		}
	}
	increment();
}

void checkPoints::increment()
{
	mnCurrCheckPoint++;
}

#ifdef _MSC_VER
UINT64 Profiler::m_ValuePerMilliSecond=0;
#endif

Profiler::Profiler()
{
#ifdef _MSC_VER
	m_nCount = 0; m_int64StartTime = 0;  	m_int64MaxTime = 0; 	m_int64AverageTime = 0;
	if(m_ValuePerMilliSecond==0)
	{
		LARGE_INTEGER	StartTime, StopTime;

		GetSimpleAccurateTime(StartTime);
		Sleep(1000);
		GetSimpleAccurateTime(StopTime);
		m_ValuePerMilliSecond = StopTime.QuadPart - StartTime.QuadPart;
		m_ValuePerMilliSecond /= 1000;
	}
#endif
}


// Start and Stop a counter.
void Profiler::start()
{
#ifdef _MSC_VER
	LARGE_INTEGER	StartTime;
	GetSimpleAccurateTime(StartTime);
	m_int64StartTime = (UINT64)(StartTime.QuadPart);
#endif
}

void Profiler::stop()
{
#ifdef _MSC_VER
	LARGE_INTEGER	StopTime;

	GetSimpleAccurateTime(StopTime);

	m_int64Diff = (StopTime.QuadPart-m_int64StartTime)/m_ValuePerMilliSecond;
	m_int64AverageTime = (m_int64AverageTime * m_nCount + m_int64Diff )/(m_nCount+1);
	m_nCount++;

	m_int64MaxTime = ((m_int64MaxTime > m_int64Diff )?m_int64MaxTime:m_int64Diff );
#endif
}


void Profiler::draw()
{
#ifdef _MSC_VER
	//			INT64 m_int64AverageTime=m_ValuePerMilliSecond;
	//			INT64 hour=m_int64AverageTime /(1000*60*60);
	//			INT64 minute=(m_int64AverageTime / (1000*60))%60;
	//			INT64 second=(m_int64AverageTime / (1000))%60;
	//			INT64 milli=(m_int64AverageTime)%1000;
	//			printf("(%dh,%dm,%ds,%dms,c:)\n\n",(int)hour,(int)minute,(int)second,(int)milli);

	printf("current %d ms, average %d ms, max %d ms\n", (int)m_int64Diff, (int)m_int64AverageTime, (int)m_int64MaxTime);
#endif
}

Tracer::Tracer(const char* filename)
:mFilename(filename)
{
	FILE* pFile;
	pFile = fopen(filename,"w");
	fprintf(pFile,"\n");
	fclose(pFile);
}

Tracer::~Tracer()
{

}

void Tracer::trace(const char* pszFormat, ...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	TString t;
	t._format(pszFormat, argptr);
	OutputToFile(mFilename, t);
}

	Stopwatch::Stopwatch(void)
  	{
#ifdef _MSC_VER
		QueryPerformanceFrequency((LARGE_INTEGER*)&m_freq);	// 클럭정보
		mState=END;
#endif
	};
void Stopwatch::start()
{
#ifdef _MSC_VER
	Msg::verify(mState==END || mState==PAUSE, "stopwatch start error %d", mState);
	QueryPerformanceCounter((LARGE_INTEGER*)&m_start);
//	printf("s %d %d \n", m_start.HighPart, m_start.LowPart);

	if(mState==PAUSE)
	{

		m_start.QuadPart-=m_pause.QuadPart;
//		printf("a %d %d \n", m_start.HighPart, m_start.LowPart);

	}
	mState=START;
#endif
};


m_real Stopwatch::end()
{
#ifdef _MSC_VER
	if(mState==PAUSE)
	{
		m_time = (m_real)((m_real)(m_pause.QuadPart) / (m_real)m_freq.QuadPart);
//		printf("pe %d %d %d %d %f\n", m_pause.HighPart, m_pause.LowPart, m_freq.HighPart, m_freq.LowPart, m_time);
	}
	else if(mState==START)
	{
		QueryPerformanceCounter((LARGE_INTEGER*)&m_end);
		m_time = (m_real)((m_real)(m_end.QuadPart - m_start.QuadPart) / (m_real)m_freq.QuadPart);
//		printf("se %d %d %d %d %d %d %f\n", m_start.HighPart, m_start.LowPart, m_end.HighPart, m_end.LowPart, m_freq.HighPart, m_freq.LowPart, m_time);

	}
	else
	{
		m_time=0.0;
		printf("Stopwatch not called\n");
	}
	mState=END;
	return m_time;
#else
	return 0;
#endif

}



