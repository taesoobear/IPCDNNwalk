
#include "stdafx.h"
#include "VRMLloaderView.h"
#include "../../BaseLib/motion/VRMLloader_internal.h"

struct VRMLTransformView: public VRMLTransform
{
	VRMLTransformView():VRMLTransform(){ reverted=false; }
	
	bool reverted;
	// In the original skeleton, NameId usually represents either joint name (such as HIP, KNEE, ELBOW...) or body name (tibia, femur...)
	// However, in the VRMLloaderView skeleton, NameId is meaningful only if it represents the joint name.
	// (For example, if NameId is tibia, precisely speaking it actually means knee joint. 
	// After the chain is reverted, although tibia still represents the same knee joint (with reverted sign), 
	// femur is attached to the knee joint instead of tibia.
	// So, I set variable bodyName = "femur" to avoid confusion. This variable will not be used internally.)
	TString bodyName; 

	void clearShape()
	{
		delete mShape;
		delete mSegment;
		mShape=NULL;
		mSegment=NULL;
	}

	void _revertChain()
	{
		if(mVRMLtype=="Joint" && reverted)
		{
			TString prevJA=mJoint->jointAxis;

			for(int i=0; i<prevJA.length(); i++)
				mJoint->jointAxis[i]=prevJA[prevJA.length()-i-1];
		}
		if(m_pChildHead) {((VRMLTransformView*)m_pChildHead)->_revertChain();}
		if(m_pSibling) {((VRMLTransformView*)m_pSibling)->_revertChain();}

	}
	void offsetLocalCoord(vector3 const&offset)
	{
		if(mSegment)
		{
			mSegment->centerOfMass+=offset;
		}

		if(mShape)
		{
			matrix4 trans(quater(1,0,0,0), offset);
			mShape->mesh.transform(trans);
		}
	}
	void getShapeFrom(VRMLTransformView* source, vector3 const& offset)
	{
		ASSERT(mShape==NULL);
		ASSERT(mSegment==NULL);

		mShape=source->mShape;
		mSegment=source->mSegment;

		offsetLocalCoord(offset);
		source->mShape=NULL;
		source->mSegment=NULL;
	}
};


static VRMLTransformView* copyTree(VRMLTransform * other)
{
  VRMLTransformView *out=new VRMLTransformView ();
  out->copyFrom(*other);
  out->bodyName=other->name();

  for(Node* c=other->m_pChildHead; c; c=c->m_pSibling)
    out->AddChild(copyTree((VRMLTransform*)c));

  return out;
};


static VRMLTransformView* copyTreeExcept(VRMLTransform* other, VRMLTransform* except)
{
  VRMLTransformView *out=new VRMLTransformView ();
  out->copyFrom(*other);
	out->bodyName=other->name();

  for(Node* c=other->m_pChildHead; c; c=c->m_pSibling)
  {
	  if(c!=(Node*)except)
		  out->AddChild(copyTree((VRMLTransform*)c));
  }
  return out;
};

#ifdef _DEBUG
static void printHierarchy(VRMLTransformView* node, int depth)
{
  for(int ii=0; ii<depth; ii++) printf(" ");

  if(node->mShape)
	  printf("B:%s J:%s (%s)-shape %s\n", node->bodyName.ptr(), node->NameId, node->mVRMLtype.ptr(), node->mShape->name.ptr());
  else
	  printf("B:%s J:%s (%s)\n", node->bodyName.ptr(), node->NameId, node->mVRMLtype.ptr());

  for(Node *i=node->m_pChildHead; i!=NULL; i=i->m_pSibling)
    printHierarchy((VRMLTransformView*)i,depth+1);
}
#endif

VRMLloaderView::VRMLloaderView(VRMLloader const& source, Bone& newRootBone, vector3 const& localPos)
	:VRMLloader(), _sourceSkel(source), _newRootBone(&newRootBone)
{
	assert(localPos==vector3(0,0,0));

	// newRootBone in the source skeleton will become the new root.
	url.format("%s_%s.wrl", source.url.left(-4).ptr(), newRootBone.name().ptr());
	
	// dummy.	
	m_pTreeRoot=new VRMLTransformView();	
	((VRMLTransformView*)m_pTreeRoot)->copyFrom((VRMLTransformView const&)(*source.m_pTreeRoot));
	name=source.name;
	version=source.version;
	info=source.info;

	//
	// original chain:
	//                       
	//            F   R   T   R   R   R   R   (F,R,T=Free, Rotation, Translation, respectively)
	//            0-  1-  2-  3-  4-  5-  6-   - denotes the body attached to the joint (the shape node and the segment node)
	//             \   \  
	//              \     9
	//               7-  8
	//                R   T
	//            let S0 be the global transformation of joint 0 
	//          i.e.  S0 = t0*r0  (translation * rotation)
	//            let S1 = S0*s1     where s1=t1*r1
	//                S2 = S0*s1*s2
	//                S3 = S0*s1*s2*s3
	//            ...
	//            let S6 = S0*s1*s2*s3*s4*s5*s6
	//                S7 = S0*s7  -- another chain
	//                S8 = S0*s7*s8
	//
	// new chain: let's assume S3 becomes the new root: T3
	//
	//           R   T   R   F   R   R   R   (F,R,T=Free, Rotation, Translation, respectively)
	//           1  -2  -3  -x-  4-  5-  6-
	//           |    \9
	//           0 (Fixed)
	//
	//           7- 8
	//                    
	//          i.e.  T3 = S3
	//                T4 = S4 = S3 *s4
	//                ...
	//                T6 = S5 = S3 *s4*s5*s6
	//          so children joints can use the same local rotations and translations. 
	//          But 
	//                T2 = S2 = S3 * inv(s3)
	//                T1 = S1 = S3 * inv(s3) * inv(s2)
	//                T0 = S0 = S3 * inv(s3) * inv(s2) * inv(s1)
	//                T7 = S7 = S3 * inv(s3) * inv(s2) * inv(s1) * s7 -- another chain
	//                T8 = S8 = S3 * inv(s3) * inv(s2) * inv(s1) * s7 * s8
	//
	//          Now we have several minor complications: 
	//             1. additional bones
	//
	//                inv(s3)=inv(r3)*inv(t3)  cannot be represented by a rotational joint 
	//                so we need one additional bone to represent the translation t1.
	//
	//                T0 = S3 * (I*inv(r3)) * (inv(t3)*inv(r2)) * (inv(t2)*inv(r1)) * (inv(t1) * I)
	//                T7 = S7 * (I*inv(r3)) * (inv(t3)*inv(r2)) * (inv(t2)*inv(r1)) * (inv(t1) * I) * s7
	//                          new bone3         new bone2           new bone1          bone0 dummy -> merge into bone 7
	//
	//             2. Different local coordinates 
	//             3. translation joint further complicates things.
	//                for example, let's assume s2 was translation joint, which means r2 is constant and t2 is variable.
	//                So, new bone2 becomes fixed joint, 
	//                and new bone1 cannot be represented by any supported joint and have to be further splited :
	//                         (inv(t2)*I)  *    ( I*inv(r1) )              
	//                      translation joint      rotation joint
	//              ->            (inv(t3)*I) * (inv(t2)*I) * (I *inv(r1)) 
	//                            fixed dummy    new bone2     new bone1
	//             
	
	((MotionLoader&)source).UpdateInitialBone();

	// root.
	m_pTreeRoot->AddChild(copyTree((VRMLTransform*)(&newRootBone)));

	// chain : prevRoot-> ...->newRootBone
	VRMLTransformView* tgt=(VRMLTransformView* )m_pTreeRoot->m_pChildHead;
	ASSERT(tgt->name()==newRootBone.name());
	tgt->mJoint->jointType=HRP_JOINT::FREE;
	tgt->mJoint->translation=newRootBone.getFrame().translation;
	tgt->mJoint->rotation=vector4(0,0,1,0);
	tgt->SetNameId(tgt->name()+"_ROOT");

	if (newRootBone.treeIndex()!=1)
	{
		// new bone3 (see the metaphor above)
		VRMLTransformView* newBone3=new VRMLTransformView();
		newBone3->copyFrom((VRMLTransform&)newRootBone);
		
		tgt->AddChild(newBone3);

		ASSERT(newBone3->mVRMLtype=="Joint");
		if (newBone3->mJoint->jointType==HRP_JOINT::ROTATE)
			newBone3->reverted=true;

		newBone3->clearShape();
		newBone3->bodyName="";
		newBone3->mJoint->translation.setValue(0,0,0);
		
		tgt=newBone3;
		VRMLTransform* src=(VRMLTransform*)&newRootBone;

		// revert the chain from the new root to the original root.
		while(true)
		{		
			ASSERT(((VRMLTransform*)src->parent())->mVRMLtype=="Joint");
			VRMLTransformView* tgt_child=copyTreeExcept((VRMLTransform*)src->parent(), src);
			tgt->bodyName=tgt_child->name();
			ASSERT(tgt->mJoint->rotation.w()==0); // I assumed this here for simplification.
			vector3 offset=src->mJoint->translation;

			tgt->getShapeFrom(tgt_child, offset*-1);
			tgt_child->mJoint->translation=offset*-1;

			if(tgt_child->child()) // ex: node 1, node 0 
			{
				std::list<Node*> children;
				tgt_child->detachAllChildren(children);
				// change local coord of childrens
				for(std::list<Node*>::iterator i=children.begin(); i!=children.end(); ++i)
				{
					ASSERT(((VRMLTransformView*)(*i))->mJoint->jointType!=HRP_JOINT::SLIDE); // otherwise, you have to insert dummy nodes.. but for now.. let's ignore that.
					((VRMLTransformView*)(*i))->mJoint->translation-=offset;
				}
				tgt->addChildren(children);			
			}

			VRMLTransformView* nextNode=tgt_child;
			if(tgt->mJoint->jointType==HRP_JOINT::SLIDE)
			{
				//            original:      tgt: bone2        tgt_child: bone 1
				//                           tgt               tgt_child
				//         shifted shape:   (inv(t3)*inv(r2)) * (inv(t2)*inv(r1))              
				//                      translation joint      rotation joint
				//              ->            (inv(t3)*I) * (inv(t2)*I) * (I *inv(r1)) 
				//                            fixed dummy    new bone2     new bone1
				//                             tgt            new_tgt_child tgt_child
				
				
				// create new sliding joint.
				VRMLTransformView* new_tgt_child=new VRMLTransformView();

				new_tgt_child->copyFrom(*tgt);
				new_tgt_child->clearShape();

				new_tgt_child->mJoint->translation=vector3(0,0,0);
				new_tgt_child->AddChild(tgt_child);
				new_tgt_child->reverted=true;
				tgt_child->reverted=true;
				tgt->mJoint->jointType=HRP_JOINT::FIXED;
				tgt->SetNameId(tgt->name()+"_dummy");

				tgt_child->mJoint->translation=vector3(0,0,0);
				nextNode=tgt_child;	
				tgt_child=new_tgt_child;			
			}


			if(((VRMLTransform*)src->parent()->parent())->mVRMLtype=="Joint")
			{
				tgt_child->reverted=true;
				tgt->AddChild(tgt_child);
				src=(VRMLTransform*)src->parent();
				tgt=nextNode;
			}
			else 
			{
				//tgt_child corresponds to the node 0
				_srcRootTransf=tgt;
				_srcRootOffset=offset*-1;
				delete tgt_child;
				break;
			}
		}
	}
	else
	{
		_srcRootTransf=tgt;
		_srcRootOffset=vector3(0,0,0);
	}

	((VRMLTransformView*)m_pTreeRoot->m_pChildHead)->_revertChain();

	// I need to compact unnecessary fixed joints
	#ifdef _DEBUG	
		printHierarchy((VRMLTransformView*)m_pTreeRoot,0);
	#endif	

	_initDOFinfo();


	_treeIndex2sourceTree.init(numBone());
	_DOFIndex2sourceTree.init(dofInfo.numDOF());
	for(int i=1; i<numBone(); i++)
	{
		int sti=source.getTreeIndexByName(bone(i).name());
		if(sti!=-1)
			_treeIndex2sourceTree.map(i, sti);
	}


	_conversionSign.setSize(dofInfo.numDOF());
	_conversionSign.setAllValue(1);
	
	for(int i=1; i<numBone(); i++)
	{
		VRMLTransform& b=VRMLbone(i);
		int dofStartIndex=dofInfo.startT(i);

		int srcTreeIndex=_treeIndex2sourceTree(i);

		if(srcTreeIndex!=-1)
		{
			VRMLTransform& srcb=source.VRMLbone(srcTreeIndex);

			if(((VRMLTransformView&)b).reverted)
			{				
				for(int j=0; j<dofInfo.numDOF(i); j++)
				{
					_conversionSign(dofStartIndex+j)=-1;
					_DOFIndex2sourceTree.map(dofStartIndex+j, srcb.DOFindex(srcb.numHRPjoints()-j-1));
				}
			}
			else
			{
				for(int j=0; j<dofInfo.numDOF(i); j++)
					_DOFIndex2sourceTree.map(dofStartIndex+j, srcb.DOFindex(j));
			}
		}
	}

	_treeIndex2sourceTree.map(1, newRootBone.treeIndex());

}

void VRMLloaderView::convertSourcePose(vectorn const& srcPose, vectorn& pose) const
{
	convertSourceDOFexceptRoot(srcPose, pose);

	if(dofInfo.numDOF(1)==7)
	{
		ASSERT(bone(1).getTranslationalChannels()=="XYZ");
		_sourceSkel.setPoseDOF(srcPose);

		transf const& rootTF=_sourceSkel.bone(_treeIndex2sourceTree(1)).getFrame();
		pose.setVec3(0, rootTF.translation);
		pose.setQuater(3, rootTF.rotation);
	}
	else ASSERT(dofInfo.numDOF(1)==0); // Currently only Free or Fixed joints are supported

}

void VRMLloaderView::convertPose(vectorn const& pose, vectorn& srcPose) const
{
	ASSERT(0);
}

void VRMLloaderView::convertSourceDOFexceptRoot(vectorn const& srcPose, vectorn& pose) const
{
	pose.setSize(dofInfo.numDOF());

	for(int i=0; i<pose.size(); i++)
	{
		int idx=_DOFIndex2sourceTree(i);
		if(idx!=-1)
			pose[i]=srcPose[idx]*_conversionSign[i];
		else
			pose[i]=0;
	}
}

void VRMLloaderView::convertDOFexceptRoot(vectorn const& pose, vectorn& srcPose) const
{
	srcPose.setSize(_sourceSkel.dofInfo.numDOF());

	for(int idx=0; idx<srcPose.size(); idx++)
	{
		int i=_DOFIndex2sourceTree.inverse(idx);
		if(i!=-1)
			srcPose[idx]=pose[i]*_conversionSign[i];
		else
			srcPose[idx]=0;
	}
}

void VRMLloaderView::convertSourceDQexceptRoot(vectorn const& src_dq, vectorn& _dq) const
{
	_dq.setSize(dofInfo.numActualDOF());

	for(int dq_i=0; dq_i<_dq.size(); dq_i++)
	{
		int i=dofInfo.DQtoDOF(dq_i);
		int idx=_DOFIndex2sourceTree(i);
		if(idx!=-1)
			_dq[dq_i]=src_dq[_sourceSkel.dofInfo.DOFtoDQ(idx)]*_conversionSign[i];
		else
			_dq[dq_i]=0;
	}
}

void VRMLloaderView::convertDQexceptRoot(vectorn const& _dq, vectorn& src_dq) const
{
	src_dq.setSize(_sourceSkel.dofInfo.numActualDOF());

	for(int dq_idx=0; dq_idx<src_dq.size(); dq_idx++)
	{
		//printf(":%d %s\n", dq_idx,_sourceSkel.dofInfo._sharedinfo->DQtoDOF.output().ptr());
		int idx=_sourceSkel.dofInfo.DQtoDOF(dq_idx);
		//printf("%d \n", idx);
		int i=_DOFIndex2sourceTree.inverse(idx);
		//printf("%d %d %d\n", dq_idx, idx, i);
		if(i!=-1)
			src_dq[dq_idx]=_dq[dofInfo.DOFtoDQ(i)]*_conversionSign[i];
		else
			src_dq[dq_idx]=0;
	}
}
