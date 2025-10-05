#include "stdafx.h"
#include "mathclass.h"
#include "intervals.h"
#include "../utility/tfile.h"

void intIntervals::load(const char* filename)
{
	CTextFile intervalMarking;

	int currSize=0;
	if(intervalMarking.OpenReadFile(filename))
	{
		char* token;
		while(token=intervalMarking.GetToken())
		{
			currSize++;
			if(currSize>size())
				resize(currSize);

			int index=atoi(token);
			start(currSize-1)=index;


			token=intervalMarking.GetToken();
			if(!token)
			{
				Msg::error("Error! cannot load %s", filename);
			}
			else
			{
				int index=atoi(token);
				end(currSize-1)=index;
				if(end(currSize-1)<start(currSize-1))
					Msg::error("Error while loading %s", filename);
			}				
		}
	}
	else
	{
		Msg::print("file open error %s", filename);
	}

	resize(currSize);
}

int intIntervals::findOverlap(int startf, int endf, int startInterval)
{
	for(int i=startInterval; i<size(); i++)
	{
		// case 1:  --=__ or --==---
		if(startf<=start(i) && endf>start(i))
			return i;
		// case 2:  __=-- or __==___
		if(startf>start(i) && startf<end(i))
			return i;
	}

	return -1;
}

void intIntervals::removeInterval(int i)
{
	if(i<size())
	{
		for(int j=i; j<size()-1; j++)
		{
			m_vStart[j]=m_vStart[j+1];
			m_vEnd[j]=m_vEnd[j+1];
		}

		resize(size()-1);
	}
}


void intIntervals::runLengthEncode(const boolN& source, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	setSize(0);

	bool bFindTrue=false;
	int i;
	for(i=start; i<end; i++)
	{
		if(bFindTrue)
		{
			if(!source[i])
			{
				m_vEnd.pushBack(i);	// end
				bFindTrue=false;
			}
		}
		else
		{
			if(source[i])
			{
				m_vStart.pushBack(i);	// start
				bFindTrue=true;
			}
		}
	}

	if(bFindTrue)
		m_vEnd.pushBack(i);	// end

	assert(m_vEnd.size()==m_vStart.size());
}

void intIntervals::findConsecutiveIntervals(const intvectorn& source)
{
	setSize(0);

	bool bFindTrue=false;	
	int i;
	for(i=0; i<source.size(); i++)
	{
		if(bFindTrue)
		{
			if(source[i]!=source[i-1]+1)
			{
				m_vEnd.pushBack(i);	// end
				m_vStart.pushBack(i);	// start
			}
		}
		else
		{
			m_vStart.pushBack(i);	// start
			bFindTrue=true;
		}
	}

	if(bFindTrue)
		m_vEnd.pushBack(i);	// end

	assert(m_vEnd.size()==m_vStart.size());

}


void intIntervals::runLengthEncode(const intvectorn& source)
{
	/**
	* RunLength encoding을 구한다.
	* [1 1 1 3 3 3 3 4 4] -> [0 1 3 3 7 4 9]
	* 0 부터 1이 3까지 나오고, 3이 7까지 나오고 4가 9까지 나온다는 뜻.
	* \param source 
	*/

	setSize(0);
	
	int start=0;
	int curValue=source[0];

	int  i;
	for(i=1; i<source.size(); i++)
	{
		if(curValue!=source[i])
		{
			pushBack(start, i);
			curValue=source[i];
			start=i;
		}
	}
	pushBack(start, i);
}

void intIntervals::runLengthEncodeCut(const boolN& cutState, int start, int end)
{
	if(start<0) start=0;
	if(end>cutState.size()) end=cutState.size();

	setSize(0);

	m_vStart.pushBack(start);  // start
	int i;
	for(i=start+1; i<end; i++)
	{
		if(cutState[i])
		{
			m_vEnd.pushBack(i);	// end
			m_vStart.pushBack(i);   // start
		}
	}

	m_vEnd.pushBack(i);	// end
}


void intIntervals::encodeIntoVector(intvectorn& out)
{
	out.setSize(size()*2);

	//* start=encoding[grp*2];
	//* end=encoding[grp*2+1];
	for(int grp=0; grp<size(); grp++)
	{
		out[grp*2]=start(grp);
		out[grp*2+1]=end(grp);
	}
}

void intIntervals::decodeFromVector(const intvectorn& in)
{
	setSize(in.size()/2);

	for(int grp=0; grp<size(); grp++)
	{
		start(grp)=in[grp*2];
		end(grp)=in[grp*2+1];
	}
}

void intIntervals::offset(int offset)
{
	m_vStart+=offset;
	m_vEnd+=offset;
}

void intIntervals::toBitvector(boolN& bitVector)
{
	bitVector.resize(0);
	for(int i=0; i<size(); i++)
	{
		bitVector.resize(MAX(bitVector.size(), end(i)));

		bitVector.setValue(start(i),end(i),true);
	}
}


#include "../BaseLib/math/mathclass.h"
#include "../BaseLib/math/interval.h"
#include "../BaseLib/math/Operator.h"
//#include "stitch.h"
intervalsView ::intervalsView (m_real* ptrr, int size, int str)
:intervals(ptrr,size,str)
{
}

intervalsView intervals::range(int start, int end, int step)
{
	return _range<intervalsView>(start, end, step);
}

intervals::intervals()
:_tvectorn<interval, m_real>()
{	
}


intervals::intervals(int n)
:_tvectorn<interval, m_real>()
{
	setSize(n);
}

intervals::~intervals()
{
}

matrixnView intervals::toMatrixn() const
{	
	return _column<matrixnView>();
}
