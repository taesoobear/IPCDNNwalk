// BVHLoader.h: interface for the BVHLoader class.
//      written by Taesoo Kwon.
//////////////////////////////////////////////////////////////////////

#pragma once
#include "MotionLoader.h"
class Parser;
//! BVH파일의 tree구조의 한 노드에 해당한다. 
/*! \ingroup group_motion */

class ChannelParser
{
public:
	ChannelParser();
	virtual ~ChannelParser();

	//! CHANNEL (DOF와 관련)
	enum { XPosition, YPosition, ZPosition, ZRotation, XRotation, YRotation};
	int m_numChannel;
	int *m_aeChannels;

	TString makeRotChannels() const;
	TString makeTransChannels() const;
	void makeQuaternionFromChannel(int channelStartIndex, m_real *aValue, quater& q, bool bRightToLeft);
};

class BVHTransform : public Bone
{
public:
	//! TYPE
	typedef enum { BVH_DUMMY, BVH_ROOT,BVH_JOINT,BVH_END, BVH_NO_MORE_NODE} BVHType;
		
	BVHTransform(BVHType NodeType);
	virtual ~BVHTransform();

	//! bone offset (translation term)
	vector3 m_offset;

	void Unpack(Parser* file);
	static BVHType CheckNodeType(Parser* file);

};

//! 가장 파일 내용의 Motion부분을 그대로 저장한다. 나중에 PostureIP로 적당히 변형된다.
class BVHIP
{
public:
	int m_numChannel;
	int m_numFrames;
	m_real m_fFrameTime;
	m_real **m_aaKeyvalue;

	void Unpack(Parser* file);

	BVHIP();
	~BVHIP();
};


//! BVH를 읽고 tree hierarchy와 PostureIP를 만든다.
/*! \ingroup group_motion */
class BVHLoader : public MotionLoader
{
public:
	// option=="loadSkeletonOnly" or NULL
	BVHLoader(const char *filename, const char* option=NULL);
	virtual ~BVHLoader();

	//virtual void SetTree(PLDPrim* pTarget)				{ SetTree(pTarget);}; 
	//virtual void SetTreeSkin(PLDPrimSkin *pTarget)		{ SetTreeSkin(pTarget);};	

	virtual void loadAnimation(Motion& mot, const char* filename) const;

	int countTotalChannels() const;
private:
	void MakePositionIPfromBVHIP(Motion& mot, const BVHIP& cBVHIP);
};

