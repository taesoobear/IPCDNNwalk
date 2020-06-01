// ASFLoader.cpp: implementation of the ASFLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ASFLoader.h"
#include "../BaseLib/utility/TextFile.h"

#define Token(token_name)	fast_strcmp(token,token_name)==0
#define ReadToken()	token=file.GetToken()
#define ReadPToken()	token=pFile->GetToken()

void ASFRoot::Unpack(CTextFile* pFile)
{
	char* token;
	while(1)
	{
		ReadPToken();
		if(!token) break;
		if(Token("order"))
		{
			while(1)
			{
				ReadPToken();
				if(Token("TX") || Token("tx"))
					order[numOrder++]=ChannelParser::XPosition;
				else if(Token("TY")|| Token("ty"))
					order[numOrder++]=ChannelParser::YPosition;
				else if(Token("TZ")|| Token("tz"))
					order[numOrder++]=ChannelParser::ZPosition;
				else if(Token("RZ")|| Token("rz"))
					order[numOrder++]=ChannelParser::ZRotation;
				else if(Token("RX")|| Token("rx"))
					order[numOrder++]=ChannelParser::XRotation;
				else if(Token("RY")|| Token("ry"))
					order[numOrder++]=ChannelParser::YRotation;
				else break;
			}
			pFile->Undo();
		}
		else if(Token("axis"))
		{
			strcpy(axis, pFile->GetToken());
		}
		else if(Token("position"))
		{
			position.x=atof(pFile->GetToken());
			position.y=atof(pFile->GetToken());
			position.z=atof(pFile->GetToken());
		}
		else if(Token("orientation"))
		{
			orientation.x=atof(pFile->GetToken());
			orientation.y=atof(pFile->GetToken());
			orientation.z=atof(pFile->GetToken());
		}
		else if(token[0]==':') break;
		else 
		{
			printf("unknown token %s\n", token);
		}
	}
	pFile->Undo();
};

void ASFBoneData::Unpack(CTextFile* pFile)
{
	char* token;
	while(1)
	{
		ReadPToken();
		if(!token) break;
		if(Token("id"))
			id=atoi(pFile->GetToken());
		else if(Token("name"))
		{
			ReadPToken();
			ASSERT(strlen(token)<MAX_NAME_LEN-1);
			strcpy(name, token);
		}
		else if(Token("direction"))
		{
			direction.x=atof(pFile->GetToken());
			direction.y=atof(pFile->GetToken());
			direction.z=atof(pFile->GetToken());
		}
		else if(Token("length"))
			length=atof(pFile->GetToken());
		else if(Token("axis"))
		{
			vecAxis.x=TO_RADIAN(atof(pFile->GetToken()));
			vecAxis.y=TO_RADIAN(atof(pFile->GetToken()));
			vecAxis.z=TO_RADIAN(atof(pFile->GetToken()));
			strcpy(axis, ReadPToken());
		}
		else if(Token("dof"))
		{
			while(1)
			{
				ReadPToken();
				if(Token("rx"))
					dof[numDof++]=ChannelParser::XRotation;
				else if(Token("ry"))
					dof[numDof++]=ChannelParser::YRotation;
				else if(Token("rz"))
					dof[numDof++]=ChannelParser::ZRotation;
				else if(Token("tx"))
					dof[numDof++]=ChannelParser::XPosition;
				else if(Token("ty"))
					dof[numDof++]=ChannelParser::YPosition;
				else if(Token("tz"))
					dof[numDof++]=ChannelParser::ZPosition;

				else break;
			}
			pFile->Undo();
		}
		else if(Token("end")) break;
	}
}

ASFTransform::ASFTransform()
:Bone()
{
	SetNameId("dummy");
	c.identity();
	c_inv.identity();
}

ASFTransform::ASFTransform(const ASFRoot& root)
:Bone()
{
	SetNameId("root");
	
	// initialize matrix
	m_transfOrig.rotation.identity();
	m_transfOrig.translation=root.position;

	{
		ChannelParser c;

		c.m_numChannel=6;
		c.m_aeChannels=new int[c.m_numChannel];
		for(int i=0; i<c.m_numChannel; i++)
			c.m_aeChannels[i]=root.order[i];

		setChannels(c.makeTransChannels(), c.makeRotChannels());
	}

	ASSERT((root.orientation-vector3(0,0,0)).length()<0.00001);	// 이외의 경우 귀찮아서 고려 안함. 
	c.identity();
	c_inv.identity();
}

ASFTransform::ASFTransform(const ASFBoneData& bone)
:Bone()
{		
	SetNameId(bone.name);
	
	// initialize matrix
	m_transfOrig.translation=vector3(
		bone.direction.x*bone.length, 
		bone.direction.y*bone.length, 
		bone.direction.z*bone.length);
	m_transfOrig.rotation.identity();

	{
		ChannelParser c;
		c.m_numChannel=bone.numDof;
		if(c.m_numChannel>0)
		{
			c.m_aeChannels=new int[c.m_numChannel];
			for(int i=0; i<c.m_numChannel; i++)
				c.m_aeChannels[i]=bone.dof[i];
		}
		setChannels(c.makeTransChannels(), c.makeRotChannels());
	}

	quater qc;
	vector3 axis(bone.vecAxis);
	qc.setRotation(bone.axis, axis,true);
	c=qc;	
	//c.identity();
	c_inv.inverse(c);
}
int AxisToChannel(char axis)
{
	switch(axis)
	{
	case 'X': return ChannelParser::XRotation;
	case 'Y': return ChannelParser::YRotation;
	case 'Z': return ChannelParser::ZRotation;
	}
	assert(0);
	return 	ChannelParser::XRotation;
}


ASFTransform::~ASFTransform()
{
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
ASFLoader::ASFLoader(const char* filename)
:MotionLoader()
{
	m_strName=filename;
	LoadASF(filename);

	TString file_name=filename;
	file_name=file_name.left(file_name.length()-3);
	file_name+="AMC";
	if(IsFileExist(file_name))
		LoadAMC(file_name, m_cPostureIP);
	_initDOFinfo();
}

ASFLoader::~ASFLoader()
{
}

void ASFLoader::LoadASF(const char* filename)
{
	m_bDataDegree=false;
	CTextFile file;
	file.OpenReadFile(filename);
	char* token;

	ASFRoot cRoot;
	std::vector<ASFBoneData> aBoneData;

	while(1)
	{
		ReadToken();
		if(!token) break;
		if(Token(":version"))
			file.GetToken();
		else if(Token(":name"))
			m_strName=file.GetToken();
		else if(Token(":units"))
		{
			while(ReadToken())
			{
				if(Token("angle"))
				{
					ReadToken();
					if(Token("deg"))
						m_bDataDegree=true;
				}
				else if(token[0]==':') break;
			}
			file.Undo();
		}
		else if(Token(":documentation"))
		{
			while(token=file.GetLine())
				if(token[0]==':') break;
			file.Undo();
		}
		else if(Token(":root"))
		{
			cRoot.Unpack(&file);
		}
		else if(Token(":bonedata"))
		{
			int count=0;
			while(1)
			{
				ReadToken();
				if(!token || !Token("begin")) break;
				aBoneData.push_back(ASFBoneData());
				aBoneData[count].Unpack(&file);
				count++;				
			}
			file.Undo();
		}
		else if(Token(":hierarchy"))
		{
			ReadHierarchy(file, cRoot, aBoneData);
			break;
		}
	}

	ConvertASFTreeToBVHTree();
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}

void ASFLoader::ConvertASFTreeToBVHTree()
{
	// ASF tree와 BVH tree는 크게 다르다. BVH tree가 joint 개념을 사용한 반면, ASF tree는 bone개념을 사용하고 있다.
	// 따라서 양쪽간에 offset의 개념이 다르다.
	// 즉 BVHTree의 한 체인은 EE기준으로 볼때 OpenGL matrix convention을 따라 표현하면 다음과 같다.
	// (T0*O0*R0*)*...*(On-1*Rn-1)*(On*Rn)*(On+1) 이다. (On+1은 site의 offset translation matrix에 해당)
	// 반면 ASFTree의 한 체인은 
	// (T0*R0*O0)*...*(Rn-1*On-1)*(Rn*On)이다.
	
	// 즉 Rn이 BVH에 비해 한칸 밀려쓰기 되어 있는데 이를 BVH로 바꿀려면 SITE 노드 한개를 추가하면 된다.
	// 물론 이렇게 하면 toes가 발목조인트에 해당하는 등 이름과 실제가 좀 매치가 안되게 되는데.. 어쩔수 없다.
	
	// The resulting BVHTree converted from ASFTree is
	// (T0*R0)*(O0*R1)*...*(On-1*Rn)*On


	// More complex case: what if there exists translational joints in the tree. (very specific example)

	// ASF chain:
	//      root       femur_dummy      femur        tibia        foot
	// M4=(T0*R0*O0) * (T1*R1*O1)  * (T2*R2*O2) * (T3*R3*O3) * (T4*R4*O4)

	// BVH chain converted from the ASF chain
	//        root        hip_dummy     hip           knee           ankle        toe(position only)
	// M4=  (T0*R0)  *  (O0*T1*R1) * (O1*T2*R2) * (O2*T3*R3)  *   (O3*T4*R4)   *  O4

	// Usually, in the converted BVH chain, hip_dummy matrix (O0*T1*R1) is identity. 
	// If you want to remove such unnecessary node, call removeAllRedundantBones.
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	ASFTransform* pParent;
	ASFTransform* pChild;
	ASFTransform* pGrandSon;
	
	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			// preorder traversal
		}
		m_TreeStack.Pop(&src);
		if(!src)
			break;

		// inorder traversal
		pParent=(ASFTransform*)m_TreeStack.GetTop();
		pChild=(ASFTransform*)src;
		if(!pChild->m_pChildHead)
		{
			// if leaf, add SITE node
			pGrandSon=new ASFTransform();
			pChild->AddChild(pGrandSon);
			pGrandSon->SetNameId("SITE");
			pGrandSon->m_transfOrig=pChild->m_transfOrig;
		}
		if(pParent)
			pChild->m_transfOrig=pParent->m_transfOrig;
		
		src=src->m_pSibling;
	}
}

void ASFLoader::ReadHierarchy(CTextFile& file, ASFRoot& cRoot, std::vector<ASFBoneData>& aBoneData)
{
	m_nNumTreeNode=1+1+aBoneData.size();	// Dummy+root+bones
	m_apNode.resize(m_nNumTreeNode);
	
	// dummy
	m_pTreeRoot=m_apNode[0]=new ASFTransform();
	// Root
	m_apNode[1]=new ASFTransform(cRoot);
	
	// Bones
	for(unsigned int i=0; i<aBoneData.size(); i++)
	{
		ASSERT(i+2<m_nNumTreeNode);
		m_apNode[i+2]=new ASFTransform(aBoneData[i]);
	}

	m_apNode[0]->AddChild(m_apNode[1]);
	char *token;
	ReadToken();
	ASSERT(Token("begin"));

	while(1)
	{
		ReadToken();
		if(Token("end"))
			break;

		int parent_index=GetIndex(token);
		ASSERT(parent_index!=-1);

		while(1)
		{
			// Read children
			bool bLineChange;
			token=file.GetToken(bLineChange);
			if(bLineChange) break;
			int child_index=GetIndex(token);
			ASSERT(child_index!=-1);
			m_apNode[parent_index]->AddChild(m_apNode[child_index]);
		}
		file.Undo();
	}
	TRACE("end");
}

void ASFLoader::LoadAMC(const char* filename, Motion& motionData) const
{
	CTextFile file;
	if(!file.OpenReadFile(filename))
		Msg::msgBox("%s not found", filename);
	
	char* token;

	std::vector<Posture*> vecPosture;
	int cur_posture=0;
	ReadToken();

	float FPS=120.f;
	if(Token(":SAMPLES-PER-SECOND"))
	{
		ReadToken();
		FPS=atof(token);
		ReadToken();
	}
	if(Token(":FRAME-RANGE"))
	{
		TString range=ReadToken();
		ReadToken();
		
	}
	motionData.frameTime(1.0/FPS);
	Msg::verify(Token(":FULLY-SPECIFIED"), "Error: token ':FULLY-SPECIFIED' expected but ..%s", token);
	ReadToken();
	Msg::verify(Token(":DEGREES"), "Error: token ':DEGREES' expected but ..%s", token);

	ASFTransform* pBone;
	m_real aValue[3];
	while(1)
	{
		ReadToken();
		if(!token) break;
		//ASSERT(atoi(token)==cur_posture+1);
		vecPosture.push_back(new Posture());
		pBone=(ASFTransform*)&getBoneByRotJointIndex(0);
		Posture& curPosture=*(vecPosture[cur_posture]);
		curPosture.Init(numRotJoint(), numTransJoint());
		ReadToken();
		ASSERT(Token(pBone->NameId));
		ReadToken();
		curPosture.m_aTranslations[0].x=atof(token);
		ReadToken();
		curPosture.m_aTranslations[0].y=atof(token);
		ReadToken();
		curPosture.m_aTranslations[0].z=atof(token);



		int numRotChannel=pBone->getRotationalChannels().length();
		for(int channel=0; channel<numRotChannel; channel++)
			aValue[channel]=TO_RADIAN(atof(file.GetToken()));

		curPosture.m_aRotations[0].setRotation(
			pBone->getRotationalChannels(),
			aValue, true);

		for(int i=1; i<numRotJoint(); i++)
		{
			ReadToken();
			int ijoint=getRotJointIndexByName(token);
			pBone=(ASFTransform*)&getBoneByRotJointIndex(ijoint);
			ASSERT(Token(pBone->NameId));

			if(pBone->getTranslationalChannels().length())
			{
				int numTransChannel=pBone->getTranslationalChannels().length();
				for(int channel=0; channel<numTransChannel; channel++)
					aValue[channel]=atof(file.GetToken());
				curPosture.m_aTranslations[pBone->transJointIndex()].setValue(aValue);
				curPosture.m_aTranslations[pBone->transJointIndex()]+=pBone->_getOffsetTransform().translation;
			}
			int numRotChannel=pBone->getRotationalChannels().length();
			for(int channel=0; channel<numRotChannel; channel++)
				aValue[channel]=TO_RADIAN(atof(file.GetToken()));
			
			quater q;
			q.setRotation(pBone->getRotationalChannels(), aValue, true);
			curPosture.m_aRotations[ijoint]=pBone->c*q*pBone->c_inv;
		}
		cur_posture++;
	}

	int numPosture=cur_posture;

//	ASSERT(numTransJoint()==1);

	motionData.InitEmpty((MotionLoader*)this, numPosture, 1.f/FPS);
	//motionData.InitSkeleton((MotionLoader*)this);
	//motionData._Init(numPosture, numRotJoint(), 1, 1.f/FPS);

	for(int i=0; i<numPosture; i++)
	{
		motionData.setPose(i, *vecPosture[i]);
		delete vecPosture[i];
	}
	
//	SetTargetIndex(&m_cPostureIP);
}

