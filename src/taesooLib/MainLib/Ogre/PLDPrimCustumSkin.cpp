#include "stdafx.h"
#ifndef NO_OGRE

#include "../OgreFltk/renderer.h"
#include "../OgreFltk/Mesh.h"
#include "PLDPrimCustumSkin.h"
#include "../Ogre/OgreSkinEntity.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreSkeleton.h>
#include <OgreSkeletonInstance.h>
#include <OgreBone.h>
PLDPrimCustumSkin::PLDPrimCustumSkin(MotionLoader* pLoader, Ogre::SkinEntity* pEntity, const char* convfilename, const OgreRenderer& renderer, bool bCurrPoseAsBindPose)
:PLDPrimSkin()
{
	m_pTimer=NULL;
	m_pEntity=pEntity;

	// Add entity to the scene node
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,0));
	m_pSceneNode->attachObject(pEntity);


	pEntity->getSkeleton()->_updateTransforms();

	TStrings convTable;
	convTable.resize(pLoader->numRotJoint());

	if(convfilename)
	{
		CTextFile file;

		Msg::verify(file.OpenReadFile(convfilename), "file open error %s", convfilename);

		char *token;
		while(token=file.GetToken())
		{
			char* jointName=token;

			int iindex=pLoader->GetIndex(jointName);

			if(iindex==-1)
			{
				printf("warning %s not exist\n", jointName);
			}
			else
			{
				int ijoint=pLoader->getRotJointIndexByTreeIndex(iindex);
				ASSERT(ijoint<pLoader->numRotJoint());
				convTable[ijoint]=file.GetToken();
			}
		}

		file.CloseFile();
	}
	else
	{
		// use joints name as ogre::bone name.
		for(int ijoint=0; ijoint<pLoader->numRotJoint(); ijoint++)
		{
			convTable[ijoint]=pLoader->GetName(pLoader->getTreeIndexByRotJointIndex(ijoint));
		}
	}

	m_aTargetIndex.setSize(pLoader->numRotJoint());
	m_aRotOrigComb.init(pLoader->numRotJoint());
	m_aInvRotOrigComb.init(pLoader->numRotJoint());
	m_aLocalRotOrig.init(pLoader->numRotJoint());
	m_aBindPose.init(pLoader->numRotJoint()+1);

	for(int i=0; i<pLoader->numRotJoint(); i++)
	{
		if(convTable[i].length())
		{
			Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
			Ogre::Bone* pBone;
			try
			{
				//pBone=pskel->getBone(Ogre::String(convTable[i]));--> 동작 안함. ex) HIPS로 찾으면 Hips가 안찾아짐.

				Ogre::Skeleton::BoneIterator iter = pskel->getBoneIterator();
				while(iter.hasMoreElements())
				{
					pBone=iter.getNext();
					if(fast_strcmp_upper(pBone->getName().c_str(), convTable[i])==0)
						break;
				}
			}
			catch( Ogre::Exception& e )
			{
				Msg::error(e.getFullDescription().c_str());
			}
			m_aTargetIndex[i]=(int)(pBone->getHandle());
			Msg::verify(pBone!=NULL && pBone==pskel->getBone((unsigned short)(m_aTargetIndex[i])), "There is no Bone %s", convTable[i].ptr());
			pBone->setManuallyControlled(true);
			m_aRotOrigComb[i]=ToBase(pBone->_getDerivedOrientation());
			m_aInvRotOrigComb[i].inverse(ToBase(pBone->_getDerivedOrientation()));
			m_aLocalRotOrig[i]=ToBase(pBone->getOrientation());

			if(bCurrPoseAsBindPose)
				pLoader->getBoneByRotJointIndex(i).getRotation(m_aBindPose[i]);
			else
				m_aBindPose[i].identity();

		}
		else
			m_aTargetIndex[i]=-1;


	}

	if(bCurrPoseAsBindPose)
	{
		// bind root position저장.
		vector3 trans;
		pLoader->getBoneByRotJointIndex(0).getTranslation(trans);
		m_aBindPose[pLoader->numRotJoint()].setValue(0, trans.x, trans.y, trans.z);
	}
	else
		m_aBindPose[pLoader->numRotJoint()].setValue(0, 0,0,0);


	parentIdx.setSize(pLoader->numRotJoint());
	parentIdx.setAllValue(-1);

	for(int i=1; i<pLoader->numRotJoint(); i++)
	{
		// missing joint에 대한 고려를 해주어야 한다. 따라서 실제 사용된 조인트 만으로 hierarchy를 다시 만든다.
		int j=i;
		do
		{
			j=dep_GetParentJoint(*pLoader,j);
			if(j==-1) break;
		}
		while(m_aTargetIndex[j]==-1);

		parentIdx[i]=j;
	}
}

PLDPrimCustumSkin::PLDPrimCustumSkin(const PLDPrimCustumSkin& other, const OgreRenderer& renderer, const char* nameid)
:PLDPrimSkin()
{
	m_pTimer=NULL;
	m_pEntity=other.m_pEntity->clone(nameid);

	// Add entity to the scene node
	m_pSceneNode=renderer.viewport().mScene->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(0,0,0));
	m_pSceneNode->attachObject(m_pEntity);


	m_pEntity->getSkeleton()->_updateTransforms();

	m_aTargetIndex=other.m_aTargetIndex;

	int numJoint=other.m_aRotOrigComb.size();
	m_aRotOrigComb.init(numJoint);
	m_aInvRotOrigComb.init(numJoint);
	m_aLocalRotOrig.init(numJoint);
	m_aTargetIndex=other.m_aTargetIndex;
	parentIdx=other.parentIdx;
	for(int i=0; i<numJoint; i++)
	{
		m_aRotOrigComb[i]=other.m_aRotOrigComb[i];
		m_aInvRotOrigComb[i]=other.m_aInvRotOrigComb[i];
		m_aLocalRotOrig[i]=other.m_aLocalRotOrig[i];
	}

	m_aBindPose.init(other.m_aBindPose.size());
	for(int i=0; i<m_aBindPose.size(); i++)
		m_aBindPose[i]=other.m_aBindPose[i];

	for(int i=0; i<numJoint; i++)
	{
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::SkeletonInstance* pskel=m_pEntity->getSkeleton();
			Ogre::Bone* pBone;

			try
			{
				pBone=pskel->getBone(m_aTargetIndex[i]);
			}
			catch( Ogre::Exception& e )
			{
				Msg::error(e.getFullDescription().c_str());
			}
			pBone->setManuallyControlled(true);
		}
	}
}

PLDPrimCustumSkin::~PLDPrimCustumSkin()
{

}

void PLDPrimCustumSkin::SetTranslation(float x, float y, float z)
{
	vector3 pos;
	pos.x=m_pSceneNode->getPosition().x;
	pos.y=m_pSceneNode->getPosition().y;
	pos.z=m_pSceneNode->getPosition().z;

	pos-=m_vTrans;
	m_vTrans.setValue(x,y,z);

	pos+=m_vTrans;
	m_pSceneNode->setPosition(ToOgre(pos));

}


void PLDPrimCustumSkin::SetPose(const Posture & posture, const MotionLoader& skeleton)
{
	quaterN aRotations(posture.numRotJoint());

	((MotionLoader&)skeleton).setPose(posture);

	for(int i=0; i<posture.numRotJoint(); i++)
		skeleton.getBoneByRotJointIndex(i).getRotation(aRotations[i]);



	Ogre::SkeletonInstance* skel=m_pEntity->getSkeleton();


	// root or root2 should not be missing. (for positioning)

	int curbone=0;

	if(m_aTargetIndex[0]!=-1)
	{
		Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[0]));



		//= L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

		pBone->setOrientation(ToOgre(
			m_aLocalRotOrig[0]* m_aInvRotOrigComb[0] * posture.m_aRotations[0]*m_aBindPose[0].inverse()* m_aRotOrigComb[0] ));
	//	skel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		m_pSceneNode->setPosition(ToOgre(m_vTrans+posture.m_aTranslations[0]-bindTrans));

		curbone++;
	}
	else
	{
		assert(m_aTargetIndex[1]!=-1 && posture.numTransJoint()>1);

		Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[1]));

		pBone->setOrientation(ToOgre(
			m_aLocalRotOrig[1]* m_aInvRotOrigComb[1] * aRotations[1]*m_aBindPose[1].inverse()* m_aRotOrigComb[1] ));
	//	skel->getRootBone()->setPosition(0,0,0);
	//	pBone->setPosition(0,0,0);
		quater& qbindTrans=m_aBindPose[m_aBindPose.size()-1];
		vector3 bindTrans(qbindTrans.x, qbindTrans.y, qbindTrans.z);
		vector3 v;
		skeleton.getBoneByRotJointIndex(1).getTranslation(v);
		m_pSceneNode->setPosition(ToOgre(m_vTrans+v-bindTrans));

		curbone=2;
	}


	for(int i=curbone; i<posture.numRotJoint(); i++)
	{
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[i]));

			/////////////////////////////////////////////////////////////////////////////////////////
			// Bind pose는 임의의 포즈가 될수 있다. (MaxExporter특성상, 주로 frame0를 사용한다.)

			// 현재 오우거 노드가 부모로부터 3번째 노드라고 하면.
			//  노드의 global orientation CN2=LN0*LN1*LN2
			//  C2가 모델의 binding pose에서의 global orientation이라 하자.

			// 목표:
			// binding pose를 SetPose함수에 입력한 경우, CN2가 C2가 되도록 해야한다.
			// C2BB 가 동작 데이타 바인딩 포즈의 combined mat이라 하자.
			// CN2 = C2B * C2BB.inv * C2 가 되면 된다.
			// (즉 C2BB와 C2B(SetPose함수의 입력자세)가 동일한 경우, CN2==C2)

			// CN2=CN1*LN2 이므로
			// -> LN2=CN1.inv * CN2
			//       =CN1.inv * C2B * C2BB.inv * C2
			//       = ( C1B * C1BB.inv* C1).inv * C2B * C2BB.inv * C2 ---------------1
			//		 = C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2
			//		 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2 ---------------2

			// 검산
			//  CN3= LN0 * lN1* LN2
			//  = (L0B * L0BB.inv * L0)*(L0.inv * L0BB * L0B.inv * C1B * C1BB.inv * C1)*(C1.inv* C1BB * C1B.inv * C2B * C2BB.inv * C2)
			//  =  C2B * C2BB.inv * C2


			pBone->setOrientation(ToOgre(
				m_aLocalRotOrig[i]*m_aInvRotOrigComb[i]			// C1.inv
				* m_aBindPose[parentIdx[i]]				// C1BB
				* aRotations[parentIdx[i]].inverse() * aRotations[i]	// C1B.inv * C2B
				* m_aBindPose[i].inverse()				// C2BB.inv
				* m_aRotOrigComb[i]));					// C2


			// 고려사항 1. 양쪽의 link길이가 다른경우. 예를 들어 Ogre쪽은 link가 4개(0,1,@,2)이지만, 포즈쪽은 link가 3개(0,1,2)라 하자
			// 위식 1 대로 계산하면
			// LN2 = L2 * C2.inv * C1BB * C1B.inv * C2B * C2BB.inv * C2;
			// LN@ = L@;
			// LN1 = L1 * C1.inv * C0BB * C0B.inv * C1B * C1BB.inv * C1
			// LN0 = C0B * C0BB.inv * L0;

			// 다곱하면.
			// CN2= C2B * C2BB.inv * C2; -> 문제 없음

			// 고려사항 2. 링크 구조가 다른경우. - 확실히 문제 생기는듯함. 수정요.
		}
	}

#ifdef _DEBUG_MAT
	skel->_updateTransforms();

	for(int i=1; i<posture.numRotJoint(); i++)
	{
		// 검산. (본 구조가 다르면 틀릴때가 있는거 같음. 수정할 필요)
		if(m_aTargetIndex[i]!=-1)
		{
			Ogre::Bone* pBone=skel->getBone((unsigned short)(m_aTargetIndex[i]));
			quater CNi=ToBase(pBone->_getDerivedOrientation());
			quater Ci=m_aRotOrigComb[i];
			//CN2 = C2B * C2BB.inv * C2
			quater CNi2= aRotations[i]* m_aBindPose[i].inverse() * m_aRotOrigComb[i];

			RE::output(TString(sz0::format("c%d", i)), "%s = %s =0 %s", CNi.output().ptr(),CNi2.output().ptr(),Ci.output().ptr());
			//

		}
	}
#endif

	//_drawConstraints(posture, skeleton);

}


void PLDPrimCustumSkin::ApplyAnim(const Motion& mot)
{


	AlzzaPostureIP* node;

	node=new AlzzaPostureIP();

	if(m_pTimer) delete m_pTimer;

	// target index가 reference가 아님
	node->Init(mot.NumJoints(),mot.frameTime());
	// reference (자료 공유)
	node->SetKeyValue(&mot);
	node->SetTarget(this);
	m_pTimer=new TimeSensor();
	m_pTimer->FirstInit(node->GetCycleInterval(), true);
	m_pTimer->AttachInterpolator(node);
	m_pTimer->StartAnim();

}

void PREPAIR_SKIN(const Motion & curMot, TString& meshFile, TString& mappingFile);

PLDPrimSkin* RE::createCustumSkin(const Motion& curMot)
{
	TString meshFile, mappingFile;
	PREPAIR_SKIN(curMot, meshFile, mappingFile);

	PLDPrimSkin* pSkin=new PLDPrimCustumSkin(&curMot.skeleton(), Ogre::createSkinEntity(*(RE::renderer().viewport().mScene), RE::generateUniqueName().ptr(), meshFile.ptr()), mappingFile, RE::renderer(), true);
	RE::renderer().addFrameMoveObject(pSkin);

	pSkin->ApplyAnim(curMot);

	return pSkin;

}

/*
#include "../Ogre/OgreSkinEntity.h"
#include "../Ogre/OgreSkinSubEntity.h"

using namespace Ogre;
void scale_Mesh(MeshLoader::Mesh& mesh, m_real scaleFactor);

void MeshShape_countIndicesAndVertices(SkinEntity * entity, size_t & index_count, size_t & vertex_count);

class MeshShape
{
public:
	MeshShape(){}





   //------------------------------------------------------------------------
   //////////////////////////////////////////////////////////////////////////
   /// Converts mesh vertex and face data into simple float arrays.
   /// If the buffer parameters are null then that data is not converted.
   /// @param[in]  entity              Entity to extract data from.
   /// @param[out] vertexBuf          Target vertex data array (can be null).
   /// @param[in]  size_t vertex_count Number of vertices.
   /// @param[out] faceData            Target face data array (can be null).
   /// @param[int] index_count         Number of indices.
   /// @author Yavin from the Ogre4J team
   //////////////////////////////////////////////////////////////////////////
   void MeshShape::convertMeshData(SkinEntity * entity,
      float * vertexBuf, size_t vertex_count,
      size_t * faceBuf, size_t index_count)
   {
      //---------------------------------------------------------------------
      // CONVERT MESH DATA
      //---------------------------------------------------------------------
      MeshPtr mesh = entity->getMesh();
      bool added_shared = false;
      size_t current_offset = 0;
      size_t shared_offset = 0;
      size_t next_offset = 0;
      size_t index_offset = 0;
      int numOfSubs = 0;

      bool useSoftwareBlendingVertices = entity->hasSkeleton();

      if (useSoftwareBlendingVertices)
      {
         entity->_updateAnimation();
      }

      // Run through the submeshes again, adding the data into the arrays
      for ( size_t i = 0; i < mesh->getNumSubMeshes(); ++i)
      {
         SubMesh* submesh = mesh->getSubMesh(i);
         bool useSharedVertices = submesh->useSharedVertices;

         if (vertexBuf)
         {
            //----------------------------------------------------------------
            // GET VERTEXDATA
            //----------------------------------------------------------------
            const VertexData * vertex_data;
            if(useSoftwareBlendingVertices)
               vertex_data = useSharedVertices ? entity->_getSkelAnimVertexData() : entity->getSkinSubEntity(i)->_getSkelAnimVertexData();
            else
               vertex_data = useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

            if((!useSharedVertices)||(useSharedVertices && !added_shared))
            {
               if(useSharedVertices)
               {
                  added_shared = true;
                  shared_offset = current_offset;
               }

               const VertexElement* posElem =
                  vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

               HardwareVertexBufferSharedPtr vbuf =
                  vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

               unsigned char* vertex =
                  static_cast<unsigned char*>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));

               // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
               //  as second argument. So make it float, to avoid trouble when Ogre::Real is
               //  comiled/typedefed as double:
               float* pReal;


               for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
               {
                  posElem->baseVertexPointerToElement(vertex, &pReal);

                  size_t n = current_offset*3 + j*3;

                  vertexBuf[n + 0] = pReal[0];
                  vertexBuf[n + 1] = pReal[1];
                  vertexBuf[n + 2] = pReal[2];
               }

               vbuf->unlock();
               next_offset += vertex_data->vertexCount;
            }
         }

         if (faceBuf)
         {
            //----------------------------------------------------------------
            // GET INDEXDATA
            //----------------------------------------------------------------
            IndexData* index_data = submesh->indexData;
            size_t numTris = index_data->indexCount / 3;
            HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

            bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);

            uint32 *pLong = static_cast<uint32*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
            uint16* pShort = reinterpret_cast<uint16*>(pLong);


            size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

            if ( use32bitindexes )
            {
               for ( size_t k = 0; k < numTris*3; ++k)
               {
                  faceBuf[index_offset++] = pLong[k] + static_cast<int>(offset);
               }
            }
            else
            {
               for ( size_t k = 0; k < numTris*3; ++k)
               {
                  faceBuf[index_offset++] = static_cast<int>(pShort[k]) + static_cast<int>(offset);
               }
            }

            ibuf->unlock();
         }

         current_offset = next_offset;
      }
   }

   std::vector<float> vertexBuf;
	std::vector<size_t> faceBuf;

	void getMesh(SkinEntity* entity, MeshLoader::Mesh& mesh)
	{
		size_t index_count;
		size_t vertex_count;
		MeshShape_countIndicesAndVertices(entity, index_count, vertex_count);

		vertexBuf.resize(vertex_count*3);
		faceBuf.resize(index_count);

		convertMeshData(entity, &vertexBuf[0], vertex_count, &faceBuf[0], index_count);

		mesh.m_arrayVertex.resize(vertex_count);
		for(int i=0; i<vertex_count; i++)
			mesh.m_arrayVertex[i].pos=vector3(vertexBuf[i*3], vertexBuf[i*3+1], vertexBuf[i*3+2]);

		int numTris=index_count/3;
		mesh.m_arrayFace.resize(numTris);
		for(int i=0; i<numTris; i++)
		{
			mesh.m_arrayFace[i].vertexIndex[0]=(int)faceBuf[i*3];
			mesh.m_arrayFace[i].vertexIndex[1]=(int)faceBuf[i*3+1];
			mesh.m_arrayFace[i].vertexIndex[2]=(int)faceBuf[i*3+2];
		}
	}

	intvectorn vertexMerge;
	intvectorn newVertexIndex;
	size_t index_count;
	size_t vertex_count;

	// 여러 vertex가 위치가 같은경우 한군데로 merge한다. (normal이 다르거나 하면 이런경우가 생긴다.)
	void getMeshMerge(SkinEntity* entity, MeshLoader::Mesh& mesh)
	{
		MeshShape_countIndicesAndVertices(entity, index_count, vertex_count);

		vertexBuf.resize(vertex_count*3);
		faceBuf.resize(index_count);

		convertMeshData(entity, &vertexBuf[0], vertex_count, &faceBuf[0], index_count);

		vertexMerge.colon(0, vertex_count);

		vector3N vertices;
		vertices.setSize(vertex_count);

		for(int i=0; i<vertex_count; i++)
			vertices[i]=vector3(vertexBuf[i*3], vertexBuf[i*3+1], vertexBuf[i*3+2]);


		double MaxDist=0;
		double dist;
		for(int i=0; i<vertex_count; i++)
		{
			for(int j=i+1; j<vertex_count; j++)
			{
				dist=vertices[i].distance(vertices[j]);
				if(dist>MaxDist) MaxDist=dist;
				if(dist<0.00001)
				{
					vertexMerge[j]=vertexMerge[i];
				}
			}
		}

		printf("max Dist=%f\n", MaxDist);
		newVertexIndex.setSize(vertex_count);
		int c=0;
		for(int i=0; i<vertex_count; i++)
		{
			if(vertexMerge[i]==i)
			{
				newVertexIndex[i]=c;
				c++;
			}
			else
			{
				newVertexIndex[i]=newVertexIndex[vertexMerge[i]];
			}
		}

		mesh.m_arrayVertex.resize(c);
		for(int i=0; i<vertex_count; i++)
			mesh.m_arrayVertex[newVertexIndex[i]].pos=vector3(vertexBuf[i*3], vertexBuf[i*3+1], vertexBuf[i*3+2]);

		int numTris=index_count/3;
		mesh.m_arrayFace.resize(numTris);
		for(int i=0; i<numTris; i++)
		{
			mesh.m_arrayFace[i].vertexIndex[0]=newVertexIndex[(int)faceBuf[i*3]];
			mesh.m_arrayFace[i].vertexIndex[1]=newVertexIndex[(int)faceBuf[i*3+1]];
			mesh.m_arrayFace[i].vertexIndex[2]=newVertexIndex[(int)faceBuf[i*3+2]];
		}
	}

	void updateMeshMerge(SkinEntity* entity, MeshLoader::Mesh& mesh)
	{
		convertMeshData(entity, &vertexBuf[0], vertex_count, NULL, index_count);
		for(int i=0; i<vertex_count; i++)
			mesh.m_arrayVertex[newVertexIndex[i]].pos=vector3(vertexBuf[i*3], vertexBuf[i*3+1], vertexBuf[i*3+2]);
	}

	void getVertexPos(SkinEntity* entity, vector3N & mesh)
	{
		convertMeshData(entity, &vertexBuf[0], vertex_count, NULL, index_count);
		mesh.setSize(vertex_count);
		for(int i=0; i<vertex_count; i++)
			mesh[newVertexIndex[i]]=vector3(vertexBuf[i*3], vertexBuf[i*3+1], vertexBuf[i*3+2]);
	}

};

void GetMeshAnimation_OLD::create(PLDPrimCustumSkin* pSkin, MeshLoader::Mesh& mesh, int iframe)
{
	if(m_data)
		delete (MeshShape*)m_data;

	mSkinOwner=false;
	mSkin=pSkin;

	m_data=_create(pSkin, mesh, iframe, false);
	if(mScaleFactor!=1.0)	scale_Mesh(mesh, mScaleFactor);

}

void GetMeshAnimation_OLD::create(Motion const& mot, MeshLoader::Mesh& mesh, int iframe)
{
	if(m_data)
		delete (MeshShape*)m_data;

	mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mot);
	mSkin->SetVisible(false);
	mSkinOwner=true;

	m_data=_create(mSkin, mesh, iframe, false);
	if(mScaleFactor!=1.0)	scale_Mesh(mesh, mScaleFactor);

}

void GetMeshAnimation_OLD::update(vector3N& vertexPos, int iframe)
{
	MeshShape& shape=*((MeshShape*)m_data);

	mSkin->setPose(iframe);

	Ogre::SkinEntity* pEntity=mSkin->getSkinEntity();
	ASSERT(pEntity);

	pEntity->addSoftwareAnimationRequest(false);

	pEntity->_updateAnimation();

	// todo: 안에서 임시로 생성되는 index와 vertex버퍼 대신 mesh를 사용하도록.
	shape.getVertexPos(pEntity, vertexPos);
	// 위치 정보 더해주기.

	vector3 pos=ToBase(mSkin->m_pSceneNode->getPosition());
	vertexPos+=pos;
	pEntity->removeSoftwareAnimationRequest(false);
}

void GetMeshAnimation_OLD::update(MeshLoader::Mesh& mesh, Posture const& pose)
{
	MeshShape& shape=*((MeshShape*)m_data);

	ASSERT(mSkin->m_pTimer);
	AlzzaPostureIP* pip=(AlzzaPostureIP*)mSkin->m_pTimer->GetFirstInterpolator();

	mSkin->SetPose(pose, pip->targetMotion().skeleton());

	Ogre::SkinEntity* pEntity=mSkin->getSkinEntity();
	ASSERT(pEntity);

	pEntity->addSoftwareAnimationRequest(false);

	pEntity->_updateAnimation();

	// todo: 안에서 임시로 생성되는 index와 vertex버퍼 대신 mesh를 사용하도록.
	shape.updateMeshMerge(pEntity, mesh);
	// 위치 정보 더해주기.

	vector3 pos=ToBase(mSkin->m_pSceneNode->getPosition());
	for(int i=0, e=mesh.m_arrayVertex.size(); i<e; i++)
	{
		mesh.m_arrayVertex[i].pos+=pos;
	}
	pEntity->removeSoftwareAnimationRequest(false);

	if(mScaleFactor!=1.0)	scale_Mesh(mesh, mScaleFactor);
}

void GetMeshAnimation_OLD::update(MeshLoader::Mesh& mesh, int iframe)
{
	ASSERT(mSkin->m_pTimer);
	AlzzaPostureIP* pip=(AlzzaPostureIP*)mSkin->m_pTimer->GetFirstInterpolator();

	update(mesh, pip->targetMotion().pose(iframe));
}

*/
#endif
