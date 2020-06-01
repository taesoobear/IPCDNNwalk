// BVHLoader.cpp: implementation of the BVHLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "BVHLoader.h"
#include "../utility/util.h"
#include "../BaseLib/utility/Parser.h"
#include <iostream>
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TString ChannelParser ::makeRotChannels() const
{
	if(m_numChannel==0) return TString("");

	int startIndex=0;
	while(m_aeChannels[startIndex]<=ZPosition) startIndex++;

	TString channels;
	channels.reserve(4);

	for(int i=startIndex; i<m_numChannel; i++)
	{
		switch(m_aeChannels[i])
		{
		case ZRotation:
			channels.add("%c", 'Z');
			break;
		case XRotation:
			channels.add("%c", 'X');
			break;
		case YRotation:
			channels.add("%c", 'Y');
			break;
		}
	}
	return channels;
}
TString ChannelParser ::makeTransChannels() const
{
	int startIndex=0;
	
	TString channels;
	channels.reserve(4);

	for(int i=startIndex; i<m_numChannel; i++)
	{
		switch(m_aeChannels[i])
		{
		case XPosition:
			channels.add("%c", 'X');
			break;
		case YPosition:
			channels.add("%c", 'Y');
			break;
		case ZPosition:
			channels.add("%c", 'Z');
			break;
		default:
			break;
		}
	}
	return channels;
}

ChannelParser ::ChannelParser ()
{m_numChannel=0;
m_aeChannels=NULL;
}
ChannelParser ::~ChannelParser ()
{
	if(m_aeChannels) delete[] m_aeChannels;
	m_aeChannels=NULL;
}
void ChannelParser::makeQuaternionFromChannel(int channelStartIndex, m_real *aValue, quater& q, bool bRightToLeft)
{
	char aChannel[4];
	
	for(int i=0; i<3; i++)
	{
		switch(m_aeChannels[channelStartIndex+i])
		{
		case ChannelParser::XRotation: aChannel[i]='X'; break;
		case ChannelParser::YRotation: aChannel[i]='Y'; break;
		case ChannelParser::ZRotation: aChannel[i]='Z'; break;
		}
	}

	aChannel[3]=0;
	q.setRotation(aChannel, aValue, bRightToLeft);	
}

BVHTransform::BVHTransform(BVHType eType)
:Bone()
{
	m_offset.setValue(0,0,0);
};

BVHTransform::~BVHTransform()
{
};



BVHTransform::BVHType BVHTransform::CheckNodeType(Parser* file)
{
	TString token;
	BVHType eType;
	token=file->getToken();
	token.makeUpper();

	if(token=="ROOT")
		eType=BVH_ROOT;
	else if(token=="JOINT")
		eType=BVH_JOINT;
	else if(token=="END")
		eType=BVH_END;
	else if(token=="}")
		eType=BVH_NO_MORE_NODE;
	else ASSERT(0);
	return eType;
}

void BVHTransform::Unpack(Parser* file)
{
	TString token;
	token=file->getToken();
	NameId=new char[token.length()+1];
	strcpy(NameId,token.ptr());
	token=file->getToken();
	ASSERT(token=="{");
	
	token=file->getToken();	
	ASSERT(token.toUpper()=="OFFSET");

	m_offset.x=(float)atof(file->getToken());
	m_offset.y=(float)atof(file->getToken());
	m_offset.z=(float)atof(file->getToken());

	// initialize matrix
	m_transfOrig.rotation.identity();
	m_transfOrig.translation=m_offset;
	
	token=file->getToken();
	token.makeUpper();
	if(token=="}") return;
	if(token=="CHANNELS")
	{
		ChannelParser _c;

		_c.m_numChannel=atoi(file->getToken());
		_c.m_aeChannels=new int[_c.m_numChannel];
		for(int i=0; i<_c.m_numChannel; i++)
		{
			token=file->getToken().toUpper();

			if(token=="XPOSITION")
				_c.m_aeChannels[i]=ChannelParser::XPosition;
			else if(token=="YPOSITION")
				_c.m_aeChannels[i]=ChannelParser::YPosition;
			else if(token=="ZPOSITION")
				_c.m_aeChannels[i]=ChannelParser::ZPosition;
			else if(token=="ZROTATION")
				_c.m_aeChannels[i]=ChannelParser::ZRotation;
			else if(token=="XROTATION")
				_c.m_aeChannels[i]=ChannelParser::XRotation;
			else if(token=="YROTATION")
				_c.m_aeChannels[i]=ChannelParser::YRotation;
		}
		setChannels(_c.makeTransChannels() , _c.makeRotChannels() );

	}


	BVHType eType;
	// Recursively load every child nodes
	while((eType=CheckNodeType(file))!=BVH_NO_MORE_NODE)
	{
		BVHTransform* pchild=new BVHTransform(eType);
		AddChild(pchild);
		pchild->Unpack(file);
	}
}

BVHIP::BVHIP()
{
	m_aaKeyvalue=NULL;
}

BVHIP::~BVHIP()
{
	if(m_aaKeyvalue)
	{
		for(int i=0; i<m_numFrames; i++)
			delete[] m_aaKeyvalue[i];
		delete[] m_aaKeyvalue;
	}
}

void BVHIP::Unpack(Parser* file)
{
	// Load Motion
	TString token;
	token = file->getToken();
	if(token.toUpper()=="MOTION")
		token=file->getToken();
	ASSERT(token.toUpper()=="FRAMES:");
	m_numFrames=atoi(file->getToken());
	token=file->getToken();
	token=file->getToken();
	ASSERT(token.toUpper()=="TIME:");
	m_fFrameTime=(float)atof(file->getToken());

	float f_trlvalue;
	FILE* parseFile = file->getFilePointer();

	m_aaKeyvalue= new m_real*[m_numFrames];
	for(int i=0; i<m_numFrames; i++)
	{
		m_aaKeyvalue[i]=new m_real[m_numChannel];
		for( int j=0; j<m_numChannel; j++)
		{
			// gugugugugu
			// file: Parser
			// Parser->getFilePointer()
			// fscanf("%f", ...
			//m_aaKeyvalue[i][j]=atof(file->getToken());
			fscanf(parseFile,"%f",&f_trlvalue);
			m_aaKeyvalue[i][j]=f_trlvalue;
		}
	}
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

BVHLoader::BVHLoader(const char* filename, const char* option)
:MotionLoader()
{
	std::cout << "start of bvh: " << filename <<std::endl;
	Parser file(filename);
	
	Msg::verify(file.getToken().toUpper()=="HIERARCHY", "BVH file error: %s", filename);

	m_pTreeRoot=new BVHTransform(BVHTransform::BVH_DUMMY);	// dummy root를 달아 놓는다. 이는 나중에 모델을 자유롭게 translate또는 rotate하는데 사용될수 있다.
	m_pTreeRoot->NameId=new char[6];
	strcpy(m_pTreeRoot->NameId,"DUMMY");
	
	VERIFY(BVHTransform::CheckNodeType(&file)==BVHTransform::BVH_ROOT);
	m_pTreeRoot->AddChild(new BVHTransform(BVHTransform::BVH_ROOT));

	// Tree 생성 
	((BVHTransform*)(m_pTreeRoot->m_pChildHead))->Unpack(&file);

	
	int numChannel;
	// Count Total Number of channels, total Number of END node
	MakeBoneArrayFromTree(numChannel);	

	if(!option || TString("loadSkeletonOnly")!=option)
	{
		BVHIP cBVHIP;
		cBVHIP.m_numChannel=numChannel;
		cBVHIP.Unpack(&file);
		MakePositionIPfromBVHIP(m_cPostureIP, cBVHIP);
		
		//if(file.getToken()!="")
		//	printf("EOF warning\n");
	}
	_initDOFinfo();
}

int BVHLoader::countTotalChannels() const
{
	int numChannel=0;
	for(int i=0; i<GetNumTreeNode(); i++)
	{
		Bone& bone=getBoneByTreeIndex(i);
		if(bone.numChannels())
			numChannel+=bone.numChannels();
	}
	return numChannel;
}

void BVHLoader::loadAnimation(Motion& mot, const char* filename) const
{
	TRACE("Loading animation %s\n", filename);

	TString fn=filename;
	if(fn.right(4).toUpper()==".BVH") 
	{
		Parser file(filename);
		TString token;
		while(1)
		{
			token = file.getToken();
			VERIFY(token.length());
			if(token.toUpper()=="MOTION")
				break;
		}
		

		BVHIP cBVHIP;
		cBVHIP.m_numChannel=countTotalChannels() ;
		cBVHIP.Unpack(&file);
		((BVHLoader*)this)->MakePositionIPfromBVHIP(mot, cBVHIP);
		
		if(file.getToken()!="")
			printf("EOF warning\n");
	}
	else
		MotionLoader::loadAnimation(mot, filename);
}

void BVHLoader::MakePositionIPfromBVHIP(Motion& mot, const BVHIP& cBVHIP)
{
	Bone* pBone;
	
	//mot.InitSkeleton(this);
	//mot._Init(cBVHIP.m_numFrames, numRotJoint(), 1, cBVHIP.m_fFrameTime);

	//ASSERT(numTransJoint()==1);
	mot.InitEmpty(this, cBVHIP.m_numFrames, cBVHIP.m_fFrameTime);

	m_real aValue[3];

	TString channels;
	for(int iframe=0; iframe<cBVHIP.m_numFrames; iframe++)
	{
		int currChannel=0;
		

		/*
		int ijoint=0;
		pBone=&getBoneByRotJointIndex(ijoint);
		// numTransJoint()==1일때만 동작.

		mot.pose(iframe).m_aTranslations[0].x=cBVHIP.m_aaKeyvalue[iframe][0];
		mot.pose(iframe).m_aTranslations[0].y=cBVHIP.m_aaKeyvalue[iframe][1];
		mot.pose(iframe).m_aTranslations[0].z=cBVHIP.m_aaKeyvalue[iframe][2];
		// ( channel startIndex, channel value, quaternion)
		
		
		channels=pBone->getRotationalChannels();
		for(int c=0; c<channels.length(); c++)
			aValue[c]=TO_RADIAN(cBVHIP.m_aaKeyvalue[iframe][3+c]);

		mot.pose(iframe).m_aRotations[ijoint].setRotation(channels, aValue);

		currChannel+=pBone->numChannels();

		for(ijoint=1; ijoint<numRotJoint(); ijoint++)
		{
			pBone=&getBoneByRotJointIndex(ijoint);


			channels=pBone->getRotationalChannels();
			for(int c=0; c<channels.length(); c++)
				aValue[c]=TO_RADIAN(cBVHIP.m_aaKeyvalue[iframe][currChannel+c]);

			mot.pose(iframe).m_aRotations[ijoint].setRotation(channels, aValue);
			currChannel+=pBone->numChannels();
		}*/

		for(int ibone=1; ibone<numBone(); ibone++)
		{
			pBone=&getBoneByTreeIndex(ibone);
			
			TString transchannels=pBone->getTranslationalChannels();

			if(transchannels.length())
			{
				Msg::verify(transchannels=="XYZ", "transchannels");
				
				{
					mot.pose(iframe).m_aTranslations[pBone->transJointIndex()].x=cBVHIP.m_aaKeyvalue[iframe][currChannel+0];
					mot.pose(iframe).m_aTranslations[pBone->transJointIndex()].y=cBVHIP.m_aaKeyvalue[iframe][currChannel+1];
					mot.pose(iframe).m_aTranslations[pBone->transJointIndex()].z=cBVHIP.m_aaKeyvalue[iframe][currChannel+2];

					// I am not sure if the following is needed or not.
					//mot.pose(iframe).m_aTranslations[pBone->transJointIndex()]+=pBone->getOffsetTransform().translation;


				}

				currChannel+=3;
			} 
			
			channels=pBone->getRotationalChannels();

			if(channels.length())
			{
				for(int c=0; c<channels.length(); c++)
					aValue[c]=TO_RADIAN(cBVHIP.m_aaKeyvalue[iframe][currChannel+c]);

				mot.pose(iframe).m_aRotations[pBone->rotJointIndex()].setRotation(channels, aValue);
				currChannel+=channels.length();
			}
		}
	}
//	SetTargetIndex(&mot);
}

BVHLoader::~BVHLoader()
{
}

